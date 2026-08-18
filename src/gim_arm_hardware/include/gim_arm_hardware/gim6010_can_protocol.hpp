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

inline uint32_t make_can_id(uint8_t node_id, CmdId cmd) {
  return (static_cast<uint32_t>(node_id) << 5) | static_cast<uint32_t>(cmd);
}

// Plain little-endian uint32 pair -- covers Set_Axis_State (only field 0 used)
// and Set_Controller_Mode (control_mode, input_mode).
inline void pack_u32_le(uint8_t data[8], uint32_t field0, uint32_t field1 = 0) {
  std::memcpy(&data[0], &field0, 4);
  std::memcpy(&data[4], &field1, 4);
}

// ---- Set_Input_Pos (cmd_id 0x00C), manual 4.1.2 ----
// Layout 8 byte: [0..3] Input_Pos float32 (rev), [4..5] Vel_FF int16, [6..7]
// Torque_FF int16. Hai trường FF là số nguyên thang 0.001 -- tức Vel_FF đếm
// theo 0.001 rev/s và Torque_FF theo 0.001 Nm.
//
// KHÁC Mit_Control: cả 3 trường ở đây là PHÍA ROTOR (trước hộp số), giống
// odrivetool/USB. Bên gọi phải tự chia gear_ratio * direction. Đừng lẫn với
// ghi chú "output shaft" phía trên -- ghi chú đó CHỈ đúng cho 0x008.
//
// Vì sao 2 trường FF đáng quan tâm: vòng P vị trí của driver phải duy trì một
// sai số thường trực e = v / pos_gain mới sinh ra được lệnh vận tốc -- đó là
// nguồn trễ bám chính, và KHÔNG mất đi khi đổi input_mode. Điền Vel_FF khiến
// vòng P không phải "kiếm" vận tốc từ sai số nữa; điền Torque_FF = G(q) khiến
// nó không phải kiếm mô-men chống trọng lực từ sai số. Đo trong mô phỏng
// (kinematics_test/compare_architectures.py): sai số đầu tay 2.066mm -> 0.437mm
// khi thêm Vel_FF, -> 0.266mm khi thêm cả Torque_FF = G(q).
inline int16_t encode_milli_i16(double x) {
  const double scaled = std::round(x * 1000.0);
  return static_cast<int16_t>(std::clamp(scaled, -32768.0, 32767.0));
}

inline void pack_set_input_pos(
  uint8_t data[8], double pos_rev, double vel_ff_rev_s = 0.0, double torque_ff_nm = 0.0) {
  const float pos_f = static_cast<float>(pos_rev);
  const int16_t vel_i = encode_milli_i16(vel_ff_rev_s);
  const int16_t trq_i = encode_milli_i16(torque_ff_nm);
  std::memcpy(&data[0], &pos_f, 4);
  std::memcpy(&data[4], &vel_i, 2);
  std::memcpy(&data[6], &trq_i, 2);
}
// ---- Set_Input_Torque (cmd_id 0x00E), manual 4.1.2 ----
// 8 byte: [0..3] Input_Torque float32, [4..7] không dùng.
// ĐƠN VỊ PHÍA ROTOR (Nm trước hộp số), giống 0x00C và khác 0x008. Bên gọi phải
// CHIA gear_ratio -- nhân nhầm là shoulder lệch 64 lần.
inline void pack_set_input_torque(uint8_t data[8], double torque_rotor_nm) {
  const float t = static_cast<float>(torque_rotor_nm);
  std::memset(data, 0, 8);
  std::memcpy(&data[0], &t, 4);
}

// ---- Mit_Control (cmd_id 0x008) fixed-point ranges, manual 4.1.2 ----
constexpr double kPosMin = -12.5, kPosMax = 12.5;  // rad, output shaft, 16 bit
constexpr double kVelMin = -65.0, kVelMax = 65.0;  // rad/s, output shaft, 12 bit
constexpr double kKpMin  = 0.0,   kKpMax  = 500.0; // Nm/rad, 12 bit
constexpr double kKdMin  = 0.0,   kKdMax  = 5.0;   // Nm*s/rad, 12 bit
constexpr double kTrqMin = -50.0, kTrqMax = 50.0;  // Nm, output shaft, 12 bit

inline uint16_t encode_range(double x, double lo, double hi, int bits) {
  x = std::clamp(x, lo, hi);
  const double scale = static_cast<double>((1u << bits) - 1);
  return static_cast<uint16_t>((x - lo) * scale / (hi - lo));
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