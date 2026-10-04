#pragma once

namespace gim_arm_hardware
{
inline double encoder_rev_to_joint_rad(
  double position_rev, double gear_ratio, double direction, double zero_offset_rad)
{
  constexpr double two_pi = 6.28318530717958647692;
  return position_rev / gear_ratio * two_pi * direction - zero_offset_rad;
}

inline double joint_rad_to_encoder_rev(
  double position_rad, double gear_ratio, double direction, double zero_offset_rad)
{
  constexpr double two_pi = 6.28318530717958647692;
  return (position_rad + zero_offset_rad) * direction / two_pi * gear_ratio;
}
}  // namespace gim_arm_hardware
