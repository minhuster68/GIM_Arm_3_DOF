#pragma once

#include <linux/can.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "gim_arm_hardware/gim6010_can_protocol.hpp"

namespace gim_arm_hardware
{
enum class EncoderZeroMethod {Can, Software};

struct EncoderZeroResult
{
  bool success{false};
  std::string error;
  std::vector<double> position_rev;
  std::vector<double> velocity_rev_s;
  std::vector<int> axis_states;
  std::vector<unsigned> encoder_samples;
  std::vector<unsigned> valid_samples;
  std::vector<double> zero_reference_rev;
};

// Bus must provide send() and nonblocking receive_all(), as SocketCanBus does.
template<typename Bus>
EncoderZeroResult set_zero_encoders_on_startup(
  Bus & bus, const std::vector<uint8_t> & node_ids,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1500),
  EncoderZeroMethod method = EncoderZeroMethod::Can)
{
  EncoderZeroResult result;
  const size_t n = node_ids.size();
  result.position_rev.assign(n, std::numeric_limits<double>::quiet_NaN());
  result.velocity_rev_s.assign(n, std::numeric_limits<double>::quiet_NaN());
  result.axis_states.assign(n, -1);
  result.encoder_samples.resize(n);
  result.valid_samples.resize(n);
  result.zero_reference_rev.assign(n, 0.0);
  const auto send_state = [&](uint8_t node, uint32_t state) {
      uint8_t data[8];
      gim6010::pack_u32_le(data, state);
      return bus.send(gim6010::make_can_id(node, gim6010::CmdId::SetAxisState), data, 8);
    };
  const auto fail = [&](const std::string & error) {
      result.error = error;
      // A partial reset cannot be rolled back. Release every axis on failure.
      for (const auto node : node_ids) {
        send_state(node, 1);
      }
      return result;
    };
  if (n == 0) {
    return fail("No CAN nodes configured");
  }
  for (size_t i = 0; i < n; ++i) {
    if (node_ids[i] > 63 ||
      std::count(node_ids.begin(), node_ids.end(), node_ids[i]) != 1)
    {
      result.error = "CAN node IDs must be unique and within [0, 63]";
      return result;
    }
  }

  std::vector<can_frame> frames;
  bus.receive_all(frames);
  for (const auto node : node_ids) {
    if (!send_state(node, 1)) {
      return fail("Failed to send IDLE to node " + std::to_string(node));
    }
  }

  const auto collect = [&](uint8_t expected_state, bool verify_zero) {
      std::vector<bool> ready(n, false);
      std::fill(result.encoder_samples.begin(), result.encoder_samples.end(), 0);
      std::fill(result.valid_samples.begin(), result.valid_samples.end(), 0);
      const auto deadline = std::chrono::steady_clock::now() + timeout;
      auto next_keepalive = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
      while (std::chrono::steady_clock::now() < deadline) {
        if (expected_state == 8 && std::chrono::steady_clock::now() >= next_keepalive) {
          for (const auto node : node_ids) {
            uint8_t data[8];
            gim6010::pack_set_input_torque(data, 0.0);
            if (!bus.send(gim6010::make_can_id(node, gim6010::CmdId::SetInputTorque), data, 8)) {
              result.error = "Failed to maintain zero torque on node " + std::to_string(node);
              return false;
            }
          }
          next_keepalive = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
        }
        frames.clear();
        bus.receive_all(frames);
        for (const auto & frame : frames) {
          if (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) {
            continue;
          }
          const auto node = static_cast<uint8_t>((frame.can_id & CAN_SFF_MASK) >> 5);
          const auto it = std::find(node_ids.begin(), node_ids.end(), node);
          if (it == node_ids.end()) {
            continue;
          }
          const size_t i = static_cast<size_t>(it - node_ids.begin());
          const auto cmd = static_cast<uint8_t>(frame.can_id & 0x1F);
          if (cmd == static_cast<uint8_t>(gim6010::CmdId::Heartbeat) && frame.can_dlc >= 5) {
            uint32_t axis_error;
            std::memcpy(&axis_error, frame.data, 4);
            result.axis_states[i] = frame.data[4];
            if (axis_error != 0) {
              std::ostringstream error;
              error << "Axis error on node " << static_cast<unsigned>(node)
                    << ": 0x" << std::hex << axis_error;
              result.error = error.str();
              return false;
            }
            ready[i] = frame.data[4] == expected_state;
            if (!ready[i]) {
              result.valid_samples[i] = 0;
            }
          } else if (frame.can_dlc >= 8 &&
            cmd == static_cast<uint8_t>(gim6010::CmdId::GetEncoderEstimates))
          {
            float pos, vel;
            std::memcpy(&pos, frame.data, 4);
            std::memcpy(&vel, frame.data + 4, 4);
            result.position_rev[i] = pos;
            result.velocity_rev_s[i] = vel;
            ++result.encoder_samples[i];
            const bool valid = std::isfinite(pos) && std::isfinite(vel) &&
              (method == EncoderZeroMethod::Software || std::abs(pos) <= 0.01) &&
              std::abs(vel) <= 0.02;
            if (verify_zero) {
              result.valid_samples[i] = ready[i] && valid ? result.valid_samples[i] + 1 : 0;
            }
          }
        }
        bool done = true;
        for (size_t i = 0; i < n; ++i) {
          done = done && ready[i] && (!verify_zero || result.valid_samples[i] >= 3);
        }
        if (done) {
          return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      std::ostringstream error;
      error << "Timeout waiting for " << (verify_zero ?
        (method == EncoderZeroMethod::Software ? "stationary encoder reference" :
        "stationary encoder zero") :
        (expected_state == 8 ? "CLOSED_LOOP heartbeat" : "IDLE heartbeat"));
      for (size_t i = 0; i < n; ++i) {
        error << "; node " << static_cast<unsigned>(node_ids[i])
              << " [state=" << result.axis_states[i]
              << ", heartbeat_ok=" << ready[i]
              << ", encoder_samples=" << result.encoder_samples[i]
              << ", valid_samples=" << result.valid_samples[i]
              << ", pos_rev=" << result.position_rev[i]
              << ", vel_rev_s=" << result.velocity_rev_s[i] << "]";
      }
      result.error = error.str();
      return false;
    };
  if (!collect(1, false)) {
    return fail(result.error);
  }

  for (const auto node : node_ids) {
    uint8_t data[8];
    // Verify with zero torque: an unsupported reset must not pull toward an old zero.
    gim6010::pack_u32_le(data, 1, 1);
    if (!bus.send(gim6010::make_can_id(node, gim6010::CmdId::SetControllerMode), data, 8)) {
      return fail("Failed to set torque mode on node " + std::to_string(node));
    }
    gim6010::pack_set_input_torque(data, 0.0);
    if (!bus.send(gim6010::make_can_id(node, gim6010::CmdId::SetInputTorque), data, 8)) {
      return fail("Failed to clear torque setpoint on node " + std::to_string(node));
    }
  }
  frames.clear();
  bus.receive_all(frames);
  for (const auto node : node_ids) {
    if (!send_state(node, 8)) {
      return fail("Failed to enable CLOSED_LOOP on node " + std::to_string(node));
    }
  }
  if (!collect(8, false)) {
    return fail(result.error);
  }
  // Reset after CLOSED_LOOP has initialized the driver's encoder estimate.
  // Keep torque at zero throughout; never test an unverified zero in position mode.
  if (method == EncoderZeroMethod::Can) {
    for (const auto node : node_ids) {
      const uint8_t data[8]{};
      if (!bus.send(gim6010::make_can_id(node, gim6010::CmdId::SetLinearCount), data, 8)) {
        return fail("Failed to reset encoder on node " + std::to_string(node));
      }
    }
  }
  frames.clear();
  bus.receive_all(frames);
  if (!collect(8, true)) {
    return fail(result.error);
  }
  if (method == EncoderZeroMethod::Software) {
    result.zero_reference_rev = result.position_rev;
  }
  for (size_t i = 0; i < n; ++i) {
    uint8_t data[8];
    gim6010::pack_set_input_pos(data, result.position_rev[i]);
    if (!bus.send(gim6010::make_can_id(node_ids[i], gim6010::CmdId::SetInputPos), data, 8)) {
      return fail("Failed to latch position on node " + std::to_string(node_ids[i]));
    }
    gim6010::pack_u32_le(data, 3, 1);
    if (!bus.send(gim6010::make_can_id(node_ids[i], gim6010::CmdId::SetControllerMode), data, 8)) {
      return fail("Failed to set position mode on node " + std::to_string(node_ids[i]));
    }
  }
  result.success = true;
  return result;
}
}  // namespace gim_arm_hardware
