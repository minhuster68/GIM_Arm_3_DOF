#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "gim_arm_hardware/gim6010_can_protocol.hpp"

namespace gim_arm_hardware
{

// Raw driver velocity can alternate around zero even while position is held.
// Qualify holding torque with fresh, tightly bounded encoder positions instead.
class PositionHoldWindow
{
public:
  void clear() { count_ = next_ = 0; }

  void add(double q, double now)
  {
    if (!std::isfinite(q) || !std::isfinite(now)) {clear(); return;}
    if (count_) {
      const size_t previous = (next_ + samples_.size() - 1) % samples_.size();
      if (now <= samples_[previous].time || now - samples_[previous].time > 0.05) {clear();}
    }
    samples_[next_] = {q, now};
    next_ = (next_ + 1) % samples_.size();
    count_ = std::min(count_ + 1, samples_.size());
  }

  bool stable(double now) const
  {
    if (count_ < 2 || !std::isfinite(now)) {return false;}
    const size_t last_index = (next_ + samples_.size() - 1) % samples_.size();
    const Sample & last = samples_[last_index];
    if (now < last.time || now - last.time > 0.05) {return false;}
    double low = last.q, high = last.q;
    for (size_t k = 1; k < count_; ++k) {
      const Sample & sample = samples_[(last_index + samples_.size() - k) % samples_.size()];
      low = std::min(low, sample.q);
      high = std::max(high, sample.q);
      if (high - low > 0.005) {return false;}
      const double span = last.time - sample.time;
      if (span >= 0.3) {
        return span <= 0.35 && std::abs(last.q - sample.q) / span <= 0.05;
      }
    }
    return false;
  }

private:
  struct Sample {double q, time;};
  std::array<Sample, 64> samples_{};
  size_t count_{0}, next_{0};
};

// A short mean of fresh position-mode torque feedback, already in joint Nm.
class HoldingTorque
{
public:
  void clear() { count_ = next_ = 0; stamp_ = -1.0; }

  void add(double torque, double now)
  {
    if (!std::isfinite(torque) || !std::isfinite(now)) {return;}
    if (stamp_ >= 0.0 && now - stamp_ > 0.2) {clear();}
    samples_[next_] = torque;
    next_ = (next_ + 1) % samples_.size();
    count_ = std::min(count_ + 1, samples_.size());
    stamp_ = now;
  }

  bool fresh(double now) const
  {
    return count_ >= 3 && now >= stamp_ && now - stamp_ <= 0.2;
  }

  double mean() const
  {
    if (!count_) {return std::numeric_limits<double>::quiet_NaN();}
    double sum = 0.0;
    for (size_t i = 0; i < count_; ++i) {sum += samples_[i];}
    return sum / static_cast<double>(count_);
  }

private:
  std::array<double, 5> samples_{};
  size_t count_{0}, next_{0};
  double stamp_{-1.0};
};

inline double blend_handoff_torque(
  double holding, double requested, double elapsed, double duration, double limit)
{
  holding = std::clamp(holding, -limit, limit);
  requested = std::clamp(requested, -limit, limit);
  const double u = duration > 0.0 ? std::clamp(elapsed / duration, 0.0, 1.0) : 1.0;
  const double blend = u * u * (3.0 - 2.0 * u);
  return std::clamp((1.0 - blend) * holding + blend * requested, -limit, limit);
}

// Mode and matching first torque are queued consecutively for this motor.
// Do not wait for the next ros2_control write cycle after changing its mode.
template<class Bus>
bool start_torque_with_hold(Bus & bus, uint8_t node, double driver_torque)
{
  if (!std::isfinite(driver_torque)) {return false;}
  uint8_t data[8];
  gim6010::pack_u32_le(data, 1u, 1u);
  if (!bus.send(gim6010::make_can_id(node, gim6010::CmdId::SetControllerMode), data, 8)) {
    return false;
  }
  gim6010::pack_set_input_torque(data, driver_torque);
  return bus.send(gim6010::make_can_id(node, gim6010::CmdId::SetInputTorque), data, 8);
}

}  // namespace gim_arm_hardware
