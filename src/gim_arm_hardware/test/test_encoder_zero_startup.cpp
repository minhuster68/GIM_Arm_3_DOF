#include <gtest/gtest.h>

#include <array>
#include <limits>

#include "gim_arm_hardware/encoder_zero_startup.hpp"
#include "gim_arm_hardware/encoder_position.hpp"

namespace
{
using gim6010::CmdId;

struct FakeBus
{
  std::array<uint32_t, 3> states{};
  std::array<uint32_t, 3> modes{};
  std::vector<can_frame> sent;
  int missing_node{-1};
  int failing_cmd{-1};
  bool idle_only{false};
  bool closed_heartbeat{true};
  bool ignore_reset{false};
  bool axis_error{false};
  bool extended_feedback{false};
  bool initialize_encoder_on_closed_loop{false};
  std::array<bool, 3> reset_applied{};
  float pos{0.001F};
  float vel{0.0F};

  bool send(uint32_t id, const uint8_t * data, uint8_t dlc)
  {
    can_frame frame{};
    frame.can_id = id;
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);
    sent.push_back(frame);
    if (static_cast<int>(id & 0x1F) == failing_cmd) {
      return false;
    }
    const size_t node = id >> 5;
    if (node >= states.size()) {
      return false;
    }
    if ((id & 0x1F) == static_cast<uint32_t>(CmdId::SetAxisState)) {
      std::memcpy(&states[node], data, 4);
      if (initialize_encoder_on_closed_loop && states[node] == 8) {
        reset_applied[node] = false;
      }
    } else if ((id & 0x1F) == static_cast<uint32_t>(CmdId::SetControllerMode)) {
      std::memcpy(&modes[node], data, 4);
    } else if ((id & 0x1F) == static_cast<uint32_t>(CmdId::SetLinearCount)) {
      reset_applied[node] = true;
    }
    return true;
  }

  void receive_all(std::vector<can_frame> & out)
  {
    for (size_t node = 0; node < states.size(); ++node) {
      if (static_cast<int>(node) == missing_node || states[node] == 0) {
        continue;
      }
      const auto state = idle_only ? 1U : states[node];
      can_frame heartbeat{};
      heartbeat.can_id = gim6010::make_can_id(node, CmdId::Heartbeat);
      heartbeat.can_dlc = 8;
      gim6010::pack_u32_le(heartbeat.data, axis_error ? 1 : 0, state);
      if (extended_feedback) {
        heartbeat.can_id |= CAN_EFF_FLAG;
      }
      if (state != 8 || closed_heartbeat) {
        out.push_back(heartbeat);
      }
      can_frame encoder{};
      encoder.can_id = gim6010::make_can_id(node, CmdId::GetEncoderEstimates);
      encoder.can_dlc = 8;
      const float p = ignore_reset ||
        (initialize_encoder_on_closed_loop && !reset_applied[node]) ? 2.0F : pos;
      std::memcpy(encoder.data, &p, 4);
      std::memcpy(encoder.data + 4, &vel, 4);
      out.push_back(encoder);
    }
  }

  void expect_idle() const
  {
    for (const auto state : states) {
      EXPECT_EQ(state, 1U);
    }
  }

  bool has_command(CmdId cmd) const
  {
    return std::any_of(sent.begin(), sent.end(), [cmd](const can_frame & frame) {
      return (frame.can_id & 0x1F) == static_cast<uint32_t>(cmd);
    });
  }
};

gim_arm_hardware::EncoderZeroResult run(FakeBus & bus)
{
  return gim_arm_hardware::set_zero_encoders_on_startup(
    bus, {0, 1, 2}, std::chrono::milliseconds(20));
}

TEST(EncoderZeroStartup, ResetAllAxesAndVerifyBeforePositionHold)
{
  FakeBus bus;
  const auto result = run(bus);
  ASSERT_TRUE(result.success) << result.error;
  for (size_t node = 0; node < 3; ++node) {
    EXPECT_EQ(bus.states[node], 8U);
    EXPECT_EQ(bus.modes[node], 3U);
    EXPECT_NEAR(result.position_rev[node], bus.pos, 1e-7);
    std::vector<CmdId> commands;
    for (const auto & frame : bus.sent) {
      if ((frame.can_id >> 5) != node) {
        continue;
      }
      commands.push_back(static_cast<CmdId>(frame.can_id & 0x1F));
      if (commands.back() == CmdId::SetLinearCount) {
        EXPECT_EQ(frame.can_dlc, 8);
        for (const auto byte : frame.data) {
          EXPECT_EQ(byte, 0);
        }
      }
    }
    EXPECT_EQ(commands, (std::vector<CmdId>{
      CmdId::SetAxisState, CmdId::SetControllerMode, CmdId::SetInputTorque,
      CmdId::SetAxisState, CmdId::SetLinearCount, CmdId::SetInputPos,
      CmdId::SetControllerMode}));
  }
}

TEST(EncoderZeroStartup, MissingAxisPreventsReset)
{
  FakeBus bus;
  bus.missing_node = 1;
  EXPECT_FALSE(run(bus).success);
  EXPECT_FALSE(bus.has_command(CmdId::SetLinearCount));
  bus.expect_idle();
}

TEST(EncoderZeroStartup, UnsupportedResetNeverEnablesPositionControl)
{
  FakeBus bus;
  bus.ignore_reset = true;
  EXPECT_FALSE(run(bus).success);
  EXPECT_FALSE(bus.has_command(CmdId::SetInputPos));
  bus.expect_idle();
}

TEST(EncoderZeroStartup, IdleZerosDoNotCountAsVerification)
{
  FakeBus bus;
  bus.idle_only = true;
  bus.pos = 0.0F;
  EXPECT_FALSE(run(bus).success);
  EXPECT_FALSE(bus.has_command(CmdId::SetInputPos));
  bus.expect_idle();
}

TEST(EncoderZeroStartup, EstimatesWithoutClosedLoopHeartbeatAreRejected)
{
  FakeBus bus;
  bus.closed_heartbeat = false;
  EXPECT_FALSE(run(bus).success);
  bus.expect_idle();
}

TEST(EncoderZeroStartup, MovingOrNonFiniteFeedbackIsRejected)
{
  for (const auto p : {0.001F, std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity()})
  {
    FakeBus bus;
    bus.pos = p;
    bus.vel = p == 0.001F ? 1.0F : 0.0F;
    EXPECT_FALSE(run(bus).success);
    bus.expect_idle();
  }
}

TEST(EncoderZeroStartup, SendFailuresRequestIdleForEveryAxis)
{
  for (const auto cmd : {CmdId::SetLinearCount, CmdId::SetControllerMode,
      CmdId::SetInputTorque, CmdId::SetInputPos})
  {
    FakeBus bus;
    bus.failing_cmd = static_cast<int>(cmd);
    EXPECT_FALSE(run(bus).success);
    bus.expect_idle();
  }
}

TEST(EncoderZeroStartup, AxisErrorPreventsReset)
{
  FakeBus bus;
  bus.axis_error = true;
  EXPECT_FALSE(run(bus).success);
  EXPECT_FALSE(bus.has_command(CmdId::SetLinearCount));
  bus.expect_idle();
}

TEST(EncoderZeroStartup, ExtendedFramesAreNotStandardNodeFeedback)
{
  FakeBus bus;
  bus.extended_feedback = true;
  EXPECT_FALSE(run(bus).success);
  EXPECT_FALSE(bus.has_command(CmdId::SetLinearCount));
}

TEST(EncoderZeroStartup, DuplicateNodeIdsFailBeforeSending)
{
  FakeBus bus;
  const auto result = gim_arm_hardware::set_zero_encoders_on_startup(bus, {0, 0, 2});
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(bus.sent.empty());
}

TEST(EncoderZeroStartup, ResetHappensAfterEncoderInitializationInClosedLoop)
{
  FakeBus bus;
  bus.initialize_encoder_on_closed_loop = true;
  const auto result = run(bus);
  EXPECT_TRUE(result.success) << result.error;
  for (const auto applied : bus.reset_applied) {
    EXPECT_TRUE(applied);
  }
}

TEST(EncoderZeroStartup, TimeoutReportsActualFeedbackForAllNodes)
{
  FakeBus bus;
  bus.ignore_reset = true;
  const auto result = run(bus);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error.find("stationary encoder zero"), std::string::npos);
  EXPECT_NE(result.error.find("node 1 [state=8, heartbeat_ok=1"), std::string::npos);
  EXPECT_NE(result.error.find("pos_rev=2"), std::string::npos);
  EXPECT_NE(result.error.find("vel_rev_s=0"), std::string::npos);
  EXPECT_NE(result.error.find("valid_samples=0"), std::string::npos);
  EXPECT_GT(result.encoder_samples[1], 0U);
}

TEST(EncoderZeroStartup, ZeroTorqueIsMaintainedWhileWaitingForFeedback)
{
  FakeBus bus;
  bus.ignore_reset = true;
  const auto result = gim_arm_hardware::set_zero_encoders_on_startup(
    bus, {0, 1, 2}, std::chrono::milliseconds(70));
  EXPECT_FALSE(result.success);
  for (size_t node = 0; node < 3; ++node) {
    unsigned torque_commands = 0;
    for (const auto & frame : bus.sent) {
      if ((frame.can_id >> 5) == node &&
        (frame.can_id & 0x1F) == static_cast<uint32_t>(CmdId::SetInputTorque))
      {
        ++torque_commands;
        float torque;
        std::memcpy(&torque, frame.data, 4);
        EXPECT_EQ(torque, 0.0F);
      }
    }
    EXPECT_GE(torque_commands, 3U);
  }
  bus.expect_idle();
}

TEST(EncoderZeroStartup, SoftwareZeroCapturesAbsoluteEncoderAndHoldsSamePose)
{
  FakeBus bus;
  bus.ignore_reset = true;
  const auto result = gim_arm_hardware::set_zero_encoders_on_startup(
    bus, {0, 1, 2}, std::chrono::milliseconds(20),
    gim_arm_hardware::EncoderZeroMethod::Software);
  ASSERT_TRUE(result.success) << result.error;
  EXPECT_FALSE(bus.has_command(CmdId::SetLinearCount));
  for (size_t node = 0; node < 3; ++node) {
    EXPECT_EQ(result.zero_reference_rev[node], 2.0);
    EXPECT_EQ(bus.modes[node], 3U);
    bool found_position = false;
    for (const auto & frame : bus.sent) {
      if ((frame.can_id >> 5) == node &&
        (frame.can_id & 0x1F) == static_cast<uint32_t>(CmdId::SetInputPos))
      {
        float position;
        std::memcpy(&position, frame.data, 4);
        EXPECT_EQ(position, 2.0F);
        found_position = true;
      }
    }
    EXPECT_TRUE(found_position);
  }
}

TEST(EncoderZeroStartup, SoftwareZeroRejectsMotionOrInvalidFeedback)
{
  for (const float velocity : {1.0F, std::numeric_limits<float>::quiet_NaN()}) {
    FakeBus bus;
    bus.ignore_reset = true;
    bus.vel = velocity;
    const auto result = gim_arm_hardware::set_zero_encoders_on_startup(
      bus, {0, 1, 2}, std::chrono::milliseconds(20),
      gim_arm_hardware::EncoderZeroMethod::Software);
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(bus.has_command(CmdId::SetInputPos));
    bus.expect_idle();
  }
  FakeBus bus;
  bus.pos = std::numeric_limits<float>::quiet_NaN();
  const auto result = gim_arm_hardware::set_zero_encoders_on_startup(
    bus, {0, 1, 2}, std::chrono::milliseconds(20),
    gim_arm_hardware::EncoderZeroMethod::Software);
  EXPECT_FALSE(result.success);
  bus.expect_idle();
}

TEST(EncoderPosition, StartupOffsetsMapRosZeroToSameEncoderPoseForEveryJoint)
{
  const std::array<double, 3> raw{0.000663757, 0.235048, -2.74429};
  const std::array<double, 3> gear{8.0, 64.0, 8.0};
  const std::array<double, 3> direction{-1.0, 1.0, -1.0};
  for (size_t i = 0; i < 3; ++i) {
    const double offset = gim_arm_hardware::encoder_rev_to_joint_rad(
      raw[i], gear[i], direction[i], 0.0);
    EXPECT_DOUBLE_EQ(gim_arm_hardware::encoder_rev_to_joint_rad(
        raw[i], gear[i], direction[i], offset), 0.0);
    EXPECT_NEAR(gim_arm_hardware::joint_rad_to_encoder_rev(
        0.0, gear[i], direction[i], offset), raw[i], 1e-12);
    for (const double command : {-0.1, 0.1}) {
      const double encoder_command = gim_arm_hardware::joint_rad_to_encoder_rev(
        command, gear[i], direction[i], offset);
      EXPECT_NEAR(gim_arm_hardware::encoder_rev_to_joint_rad(
          encoder_command, gear[i], direction[i], offset), command, 1e-12);
    }
  }
}
}  // namespace
