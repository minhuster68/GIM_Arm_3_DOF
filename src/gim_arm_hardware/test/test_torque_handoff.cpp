#include <gtest/gtest.h>
#include <linux/can.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "gim_arm_hardware/torque_handoff.hpp"

namespace
{
using gim_arm_hardware::HoldingTorque;
using gim_arm_hardware::blend_handoff_torque;
using gim_arm_hardware::handoff_seed_from_feedback;
using gim_arm_hardware::start_torque_with_hold;
using gim6010::CmdId;

struct Driver
{
  std::vector<can_frame> frames;
  int fail_command{-1};
  uint32_t mode{3};
  float torque{0};

  bool send(uint32_t id, const uint8_t * data, uint8_t dlc)
  {
    const int cmd = id & 0x1f;
    if (cmd == fail_command) {return false;}
    can_frame frame{};
    frame.can_id = id;
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);
    frames.push_back(frame);
    if (cmd == static_cast<int>(CmdId::SetControllerMode)) {
      std::memcpy(&mode, data, 4);
    } else if (cmd == static_cast<int>(CmdId::SetInputTorque)) {
      if (mode != 1) {return false;}
      std::memcpy(&torque, data, 4);
    }
    return true;
  }
};

TEST(TorqueHandoff, BaseGetsMatchingTorqueImmediatelyAfterModeFrame)
{
  Driver bus;
  const double measured_joint = 1.3;
  ASSERT_TRUE(start_torque_with_hold(bus, 0, -measured_joint));
  ASSERT_EQ(bus.frames.size(), 2u);
  EXPECT_EQ(bus.frames[0].can_id, gim6010::make_can_id(0, CmdId::SetControllerMode));
  EXPECT_EQ(bus.frames[1].can_id, gim6010::make_can_id(0, CmdId::SetInputTorque));
  EXPECT_FLOAT_EQ(bus.torque, -1.3F);
  EXPECT_EQ(bus.mode, 1u);
}

TEST(TorqueHandoff, ShoulderUsesExternalGearAndOnlyItsCanNode)
{
  Driver bus;
  ASSERT_TRUE(start_torque_with_hold(bus, 1, 8.0 / 8.0));
  EXPECT_FLOAT_EQ(bus.torque, 1.0F);
  for (const auto & frame : bus.frames) {EXPECT_EQ(frame.can_id >> 5, 1u);}
}

TEST(TorqueHandoff, OptionalFeedbackSignOnlyChangesTheHandoffSeed)
{
  EXPECT_DOUBLE_EQ(handoff_seed_from_feedback(-3.855, 1.0, 40.0), -3.855);
  EXPECT_DOUBLE_EQ(handoff_seed_from_feedback(-3.855, -1.0, 40.0), 3.855);
  EXPECT_DOUBLE_EQ(handoff_seed_from_feedback(-50.0, -1.0, 40.0), 40.0);
}

TEST(TorqueHandoff, ReportsCanFailureAndRejectsInvalidSeedBeforeModeChange)
{
  Driver bus;
  EXPECT_FALSE(start_torque_with_hold(bus, 0, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_TRUE(bus.frames.empty());
  bus.fail_command = static_cast<int>(CmdId::SetControllerMode);
  EXPECT_FALSE(start_torque_with_hold(bus, 0, -1.3));
  EXPECT_TRUE(bus.frames.empty());
  bus.fail_command = static_cast<int>(CmdId::SetInputTorque);
  EXPECT_FALSE(start_torque_with_hold(bus, 0, -1.3));
}

TEST(TorqueHandoff, RequiresSeveralFreshSamplesAndDropsOldWindow)
{
  HoldingTorque sample;
  EXPECT_FALSE(sample.fresh(0.0));
  sample.add(1.1, 0.0);
  sample.add(1.3, 0.05);
  EXPECT_FALSE(sample.fresh(0.06));
  sample.add(1.5, 0.1);
  EXPECT_TRUE(sample.fresh(0.15));
  EXPECT_NEAR(sample.mean(), 1.3, 1e-12);
  EXPECT_FALSE(sample.fresh(0.31));
  sample.add(2.0, 1.0);
  EXPECT_FALSE(sample.fresh(1.0));
  EXPECT_DOUBLE_EQ(sample.mean(), 2.0);
}

TEST(TorqueHandoff, InvalidFeedbackDoesNotRefreshWindowAndPositionReentryClearsIt)
{
  HoldingTorque sample;
  for (int i = 0; i < 6; ++i) {sample.add(i, i * 0.01);}
  EXPECT_DOUBLE_EQ(sample.mean(), 3.0);
  sample.add(std::numeric_limits<double>::quiet_NaN(), 1.0);
  EXPECT_FALSE(sample.fresh(1.0));
  sample.clear();
  EXPECT_FALSE(sample.fresh(1.0));
}

TEST(TorqueHandoff, SmoothlyTransfersMeasuredHoldToPidWithoutZeroOrLimitOvershoot)
{
  double previous = 1.3;
  for (int i = 0; i <= 100; ++i) {
    const double torque = blend_handoff_torque(1.3, 0.56, i * 0.01, 1.0, 5.0);
    EXPECT_GE(torque, 0.56 - 1e-12);
    EXPECT_LE(torque, previous + 1e-12);
    EXPECT_LE(previous - torque, 0.012);
    previous = torque;
  }
  EXPECT_DOUBLE_EQ(blend_handoff_torque(1.3, 0.56, 0.0, 1.0, 5.0), 1.3);
  EXPECT_DOUBLE_EQ(blend_handoff_torque(1.3, 0.56, 1.0, 1.0, 5.0), 0.56);
  EXPECT_DOUBLE_EQ(blend_handoff_torque(1.3, 0.8, 2.0, 1.0, 5.0), 0.8);
  EXPECT_DOUBLE_EQ(blend_handoff_torque(10.0, -10.0, 0.5, 1.0, 5.0), 0.0);
}
}  // namespace
