#pragma once
// GIM6010-8 (SteadyWin) CAN protocol helpers
// Derived from: SteadyWin_GIM6010-8 Micromotor User Manual rev1.3, section 4.1
//
// CAN ID (11-bit standard frame) = (node_id << 5) | cmd_id      [manual 4.1.1]
//   bit10~bit5 = node_id (6 bits, odrv0.axis0.config.can.node_id)
//   bit4~bit0  = cmd_id  (5 bits)
// Data: 8 bytes, little-endian for the float32 fields used by the other
// commands (Set_Input_Pos, Get_Encoder_Estimates, ...). Mit_Control does NOT
// use float32 -- it uses the fixed-point bit-packing implemented below.
//
// IMPORTANT (manual 3.1.6 and 4.1.2, Mit_Control note, repeated twice):
// Over CAN, Mit_Control position/velocity/torque are OUTPUT-SHAFT side
// (after the 8:1 gearbox) -- NOT rotor side like odrivetool/USB commands.
// Do not multiply/divide by the gear ratio yourself for this command; the
// firmware already does that conversion for you.
//
// Before Mit_Control frames do anything, the axis needs, once per session:
//   1) Set_Controller_Mode (0x00B): control_mode = 3, input_mode = 9
//   2) Set_Axis_State      (0x007): requested_state = 8 (CLOSED_LOOP_CONTROL)
// Both are plain little-endian uint32 pairs -- see pack_u32_le() below.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace gim6010 {

// ---- cmd_id table (manual 4.1.2) ----
enum class CmdId : uint8_t {
  Heartbeat              = 0x001,
  Estop                  = 0x002,
  GetError               = 0x003,
  SetAxisNodeId          = 0x006,
  SetAxisState           = 0x007,
  MitControl             = 0x008,
  GetEncoderEstimates    = 0x009,
  GetEncoderCount        = 0x00A,
  SetControllerMode      = 0x00B,
  SetInputPos            = 0x00C,
  SetInputVel            = 0x00D,
  SetInputTorque         = 0x00E,
  SetLimits              = 0x00F,
  StartAnticogging       = 0x010,
  SetTrajVelLimit        = 0x011,
  SetTrajAccelLimits     = 0x012,
  SetTrajInertia         = 0x013,
  GetIq                  = 0x014,
  GetSensorlessEstimates = 0x015,
  Reboot                 = 0x016,
  GetBusVoltageCurrent   = 0x017,
  ClearErrors            = 0x018,
  SetLinearCount         = 0x019,
  SetPosGain             = 0x01A,
  SetVelGains            = 0x01B,
  GetTorques             = 0x01C,  // Torque_Setpoint, Torque (measured, Nm) -- best source for sensorless force estimate
  GetPowers              = 0x01D,
  DisableCan             = 0x01E,
  SaveConfiguration      = 0x01F,
};

// Internal gearbox of the GIM6010-8 itself, i.e. what the firmware means by
// "output shaft" in the Mit_Control note above. A joint whose total ratio is
// larger than this has an EXTRA gearbox bolted on outside the motor, which the
// firmware cannot know about -- the host must convert for it. See
// external_ratios_ in gim_arm_system.hpp.
constexpr double kInternalGearRatio = 8.0;

inline uint32_t make_can_id(uint8_t node_id, CmdId cmd) {
  return (static_cast<uint32_t>(node_id) << 5) | static_cast<uint32_t>(cmd);
}

// Plain little-endian uint32 pair -- covers Set_Axis_State (only field 0 used)
// and Set_Controller_Mode (control_mode, input_mode).
inline void pack_u32_le(uint8_t data[8], uint32_t field0, uint32_t field1 = 0) {
  std::memcpy(&data[0], &field0, 4);
  std::memcpy(&data[4], &field1, 4);
}

// ---- Mit_Control (cmd_id 0x008) fixed-point ranges, manual 4.1.2 ----
constexpr double kPosMin = -12.5, kPosMax = 12.5;  // rad, output shaft, 16 bit
constexpr double kVelMin = -65.0, kVelMax = 65.0;  // rad/s, output shaft, 12 bit
constexpr double kKpMin  = 0.0,   kKpMax  = 500.0; // Nm/rad, 12 bit
constexpr double kKdMin  = 0.0,   kKdMax  = 5.0;   // Nm*s/rad, 12 bit
constexpr double kTrqMin = -50.0, kTrqMax = 50.0;  // Nm, output shaft, 12 bit

// LAM TRON, khong cat. Truoc day dung static_cast<uint16_t> truc tiep, tuc la
// CAT xuong (truncate toward zero). Vi (x - lo) luon >= 0 sau khi clamp, phep
// cat luon lech ve phia `lo` -- KHONG phai nhieu doi xung ma la SAI SO MOT
// CHIEU. Voi field torque (lo = -50 Nm) nghia la mo-men gui xuong luon nho hon
// mong muon, do lech trung binh nua LSB.
//
// Con so do duoc (200 gia tri g(q) ngau nhien, quy ve phia khop):
//   base/elbow (r=1): trung binh -0.0127 Nm  -> khong dang ke
//   shoulder   (r=8): trung binh -0.0947 Nm, |max| 0.193 Nm
// Shoulder nang vi send_mit_command() chia mo-men cho r truoc khi ma hoa, roi
// hop so ngoai nhan lai r lan -- sai so ma hoa bi nhan theo.
//
// -0.095 Nm bu THIEU mot chieu o shoulder trong y het "khoi luong trong URDF
// thap hon that ~2.4%", nen se lam sai ket luan cua phep thu troi tu do (dat
// mit_kp = 0, torque_ff = g(q), xem tay may co lo lung khong). Doi sang lround
// thi lech trung binh con -0.0019 Nm va het lech mot chieu.
inline uint16_t encode_range(double x, double lo, double hi, int bits) {
  x = std::clamp(x, lo, hi);
  const double scale = static_cast<double>((1u << bits) - 1);
  // Sau clamp, bieu thuc nam trong [0, scale] nen lround khong the vuot uint16
  // voi bits <= 16 (Mit_Control dung nhieu nhat 16 bit cho vi tri).
  return static_cast<uint16_t>(std::lround((x - lo) * scale / (hi - lo)));
}

inline double decode_range(uint16_t x_int, double lo, double hi, int bits) {
  const double scale = static_cast<double>((1u << bits) - 1);
  return static_cast<double>(x_int) * (hi - lo) / scale + lo;
}

// Host -> motor: pack one Mit_Control frame (8 data bytes).
// pos_rad / vel_rad_s / torque_nm are OUTPUT-SHAFT values (see note above).
inline void pack_mit_control(uint8_t data[8], double pos_rad, double vel_rad_s,
                              double kp, double kd, double torque_nm) {
  const uint16_t p    = encode_range(pos_rad,   kPosMin, kPosMax, 16);
  const uint16_t v    = encode_range(vel_rad_s, kVelMin, kVelMax, 12);
  const uint16_t kp_i = encode_range(kp,        kKpMin,  kKpMax,  12);
  const uint16_t kd_i = encode_range(kd,        kKdMin,  kKdMax,  12);
  const uint16_t t    = encode_range(torque_nm, kTrqMin, kTrqMax, 12);

  data[0] = (p >> 8) & 0xFF;
  data[1] = p & 0xFF;
  data[2] = (v >> 4) & 0xFF;
  data[3] = static_cast<uint8_t>(((v & 0xF) << 4) | ((kp_i >> 8) & 0xF));
  data[4] = kp_i & 0xFF;
  data[5] = (kd_i >> 4) & 0xFF;
  data[6] = static_cast<uint8_t>(((kd_i & 0xF) << 4) | ((t >> 8) & 0xF));
  data[7] = t & 0xFF;
}

struct MitFeedback {
  uint8_t node_id;
  double position_rad;    // output shaft
  double velocity_rad_s;  // output shaft
  double torque_nm;       // output shaft
};

// Motor -> host: decode the Mit_Control feedback frame (8 data bytes).
inline MitFeedback unpack_mit_feedback(const uint8_t data[8]) {
  MitFeedback fb;
  fb.node_id = data[0];
  const uint16_t p = (static_cast<uint16_t>(data[1]) << 8) | data[2];
  const uint16_t v = (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4);
  const uint16_t t = (static_cast<uint16_t>(data[4] & 0xF) << 8) | data[5];

  fb.position_rad   = decode_range(p, kPosMin, kPosMax, 16);
  fb.velocity_rad_s = decode_range(v, kVelMin, kVelMax, 12);
  fb.torque_nm      = decode_range(t, kTrqMin, kTrqMax, 12);
  return fb;
}

}  // namespace gim6010