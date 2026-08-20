#include "gim_arm_hardware/gim_arm_system.hpp"

#include <linux/can.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gim_arm_hardware
{

// ====================================================================
//                           BẢNG CHẾ ĐỘ
// ====================================================================
// Đây là NƠI DUY NHẤT mô tả các chế độ. write(), perform_command_mode_switch()
// và on_deactivate() chỉ tra bảng này rồi gọi con trỏ hàm -- chúng không biết
// và không cần biết có bao nhiêu chế độ.
//
// Thêm chế độ mới: thêm 1 dòng ở đây + 2 hàm enter/write + 1 hằng trong enum.
// Không phải sửa gì khác.
const GimArmSystemHardware::ModeSpec & GimArmSystemHardware::mode_spec(ControlMode m)
{
  // Cột drv_control_mode / drv_input_mode = 2 trường của Set_Controller_Mode
  // (0x00B), manual 3.1.6:
  //   control_mode: 1 = TORQUE, 2 = VELOCITY, 3 = POSITION
  //   input_mode:   1 = PASSTHROUGH, 3 = POS_FILTER, 9 = MIT_CONTROL
  static const ModeSpec table[] = {
    // id                  name             ctrl in  giữ tay
    {ControlMode::Position, "VI TRI",          3u, 1u, true,
      &GimArmSystemHardware::enter_position_mode,
      &GimArmSystemHardware::write_position_mode},

    {ControlMode::Velocity, "VAN TOC",         2u, 1u, false,
      &GimArmSystemHardware::enter_velocity_mode,
      &GimArmSystemHardware::write_velocity_mode},

    {ControlMode::Torque,   "MO-MEN",          1u, 1u, false,
      &GimArmSystemHardware::enter_torque_mode,
      &GimArmSystemHardware::write_torque_mode},

    // MIT: driver vẫn giữ tay -- nhưng bằng kp/kd MỀM quanh setpoint, không
    // phải bằng vòng vị trí cứng. Đánh dấu true vì PC chết thì tay ĐỨNG, không
    // rơi; nhưng nếu mit_kp = 0 thì nó thoái hoá thành mô-men thuần và
    // enter_mit_mode() sẽ cảnh báo riêng chuyện đó.
    {ControlMode::Mit,      "MIT (impedance)", 3u, 9u, true,
      &GimArmSystemHardware::enter_mit_mode,
      &GimArmSystemHardware::write_mit_mode},
  };

  for (const auto & s : table) {
    if (s.id == m) {
      return s;
    }
  }
  // Unknown / hằng lạ: trả về chế độ vị trí. Không bao giờ tới được đây vì
  // switch_to_mode() đã lọc Unknown, nhưng rơi về chế độ AN TOÀN NHẤT thay vì
  // trả tham chiếu treo.
  return table[0];
}

// Bộ command interface đang được claim -> chế độ. Thứ tự xét là thứ tự ƯU TIÊN
// và nó có ý nghĩa:
//   - position + effort  -> MIT (cả 2 cùng lúc chỉ hợp lý ở impedance)
//   - effort             -> mô-men thuần (LQI)
//   - position [+ velocity] -> vị trí; velocity ở đây là Vel_FF, KHÔNG phải
//     lệnh chính -- đây chính là bộ mà JointTrajectoryController claim
//   - chỉ velocity       -> chế độ vận tốc
GimArmSystemHardware::ControlMode GimArmSystemHardware::resolve_mode(
  bool has_pos, bool has_vel, bool has_eff, bool mit_enabled)
{
  if (has_pos && has_eff) {
    return mit_enabled ? ControlMode::Mit : ControlMode::Unknown;
  }
  if (has_eff) {
    return ControlMode::Torque;
  }
  if (has_pos) {
    return ControlMode::Position;
  }
  if (has_vel) {
    return ControlMode::Velocity;
  }
  return ControlMode::Unknown;
}

void GimArmSystemHardware::apply_driver_mode(const ModeSpec & spec)
{
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    uint8_t data[8];
    gim6010::pack_u32_le(data, spec.drv_control_mode, spec.drv_input_mode);
    can_bus_.send(
      gim6010::make_can_id(can_node_ids_[i], gim6010::CmdId::SetControllerMode), data, 8);
  }
}

void GimArmSystemHardware::switch_to_mode(ControlMode target)
{
  if (target == ControlMode::Unknown || target == active_mode_) {
    return;
  }

  const ModeSpec & spec = mode_spec(target);

  // 1) Driver đổi chế độ TRƯỚC. Gửi setpoint của chế độ mới khi driver còn ở
  //    chế độ cũ thì frame đó bị bỏ qua im lặng (vd 0x00C tới driver đang ở
  //    control_mode=1).
  apply_driver_mode(spec);
  active_mode_ = target;

  // 2) Xoá trạng thái watchdog: chu kỳ đầu của chế độ mới phải được đánh giá
  //    sạch, không kế thừa số đếm của chế độ trước.
  std::fill(
    last_cmd_seen_.begin(), last_cmd_seen_.end(), std::numeric_limits<double>::quiet_NaN());
  stale_cycles_ = 0;

  // 3) Chế độ tự lập trạng thái an toàn ban đầu của nó.
  (this->*spec.on_enter)();

  if (spec.driver_holds_arm) {
    RCLCPP_INFO(
      rclcpp::get_logger("GimArmSystemHardware"),
      "-> CHE DO %s (control_mode=%u, input_mode=%u)",
      spec.name, spec.drv_control_mode, spec.drv_input_mode);
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("GimArmSystemHardware"),
      "-> CHE DO %s (control_mode=%u). DRIVER KHONG CON GIU TAY. "
      "PC ngung gui = tay roi/troi. Khong chay che do nay khi tay dang deo tren nguoi.",
      spec.name, spec.drv_control_mode);
  }
}

// ====================================================================
//                    CÁC CHẾ ĐỘ: enter + write
// ====================================================================

void GimArmSystemHardware::enter_position_mode()
{
  // Chốt setpoint tại CHỖ TAY ĐANG ĐỨNG trước khi trả quyền cho vòng vị trí
  // của driver. Bỏ bước này thì driver dùng lại input_pos cũ từ trước lúc rời
  // chế độ vị trí, và tay GIẬT về đó.
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (std::isnan(hw_states_position_[i])) {
      continue;  // chưa đọc được encoder khớp này -- đừng chốt vào NaN
    }
    send_position_command(i, hw_states_position_[i], 0.0, 0.0);
    hw_commands_[i] = hw_states_position_[i];
  }
}

void GimArmSystemHardware::write_position_mode()
{
  const size_t n = info_.joints.size();

  // Bù trọng lực tính MỘT LẦN cho cả tay: G(q) là hàm của TOÀN BỘ cấu hình,
  // không tách rời từng khớp được (mô-men giữ ở vai phụ thuộc cả góc khuỷu).
  // Chỉ tính khi CẢ 3 lệnh đều hợp lệ -- thiếu 1 khớp là q sai, mà q sai thì
  // G(q) sai ở cả 3 khớp chứ không riêng khớp thiếu.
  std::vector<double> tau_gravity(n, 0.0);
  if (gravity_feedforward_) {
    const bool all_valid = std::none_of(
      hw_commands_.begin(), hw_commands_.end(),
      [](double c) {return std::isnan(c);});
    if (all_valid) {
      compute_gravity_torque(hw_commands_, tau_gravity);
    }
  }

  for (size_t i = 0; i < n; ++i) {
    if (std::isnan(hw_commands_[i])) {
      continue;  // chưa có lệnh hợp lệ, đừng gửi rác xuống CAN
    }
    // Controller có thể chỉ claim "position" -> hw_commands_velocity_ ở nguyên
    // NaN. Coi NaN là 0 chứ không bỏ qua cả frame: vị trí vẫn phải được gửi.
    const double vel_cmd =
      (velocity_feedforward_ && !std::isnan(hw_commands_velocity_[i]))
      ? hw_commands_velocity_[i] : 0.0;
    send_position_command(i, hw_commands_[i], vel_cmd, tau_gravity[i]);
  }
}

void GimArmSystemHardware::enter_velocity_mode()
{
  // Xoá lệnh vận tốc cũ. Ở chế độ vị trí, mảng này là Vel_FF do JTC ghi -- giữ
  // lại nó là chu kỳ đầu của chế độ vận tốc lấy luôn vận tốc mong muốn cuối
  // cùng của quỹ đạo trước làm lệnh, tay chạy tiếp thay vì đứng.
  std::fill(
    hw_commands_velocity_.begin(), hw_commands_velocity_.end(),
    std::numeric_limits<double>::quiet_NaN());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    send_velocity_command(i, 0.0, 0.0);
  }
}

void GimArmSystemHardware::write_velocity_mode()
{
  const size_t n = info_.joints.size();

  const bool all_valid = std::none_of(
    hw_commands_velocity_.begin(), hw_commands_velocity_.end(),
    [](double c) {return std::isnan(c);});

  bool stale = false;
  if (all_valid) {
    stale = command_stale(hw_commands_velocity_);
  } else {
    stale_cycles_ = 0;
  }

  std::vector<double> vel(n, 0.0);
  if (all_valid && !stale) {
    vel = hw_commands_velocity_;
  } else if (stale) {
    // Ở chế độ vận tốc, "giữ nguyên lệnh cũ" là kịch bản TỆ NHẤT: driver sẽ
    // quay đều mãi cho tới khi đập vào cữ. Tụt về 0 rad/s.
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("GimArmSystemHardware"), throttle_clock_, 1000,
      "Lenh van toc khong doi %d chu ky -- coi nhu nguon phat da chet. Tut ve 0 rad/s.",
      stale_cycles_);
  }

  // Bù trọng lực để vòng vận tốc của driver không phải "kiếm" mô-men giữ tay
  // từ sai số vận tốc. Tính từ vị trí ĐO ĐƯỢC -- ở chế độ này không tồn tại
  // vị trí lệnh nào cả.
  std::vector<double> tau_gravity(n, 0.0);
  if (gravity_feedforward_) {
    compute_gravity_torque(hw_states_position_, tau_gravity);
  }

  for (size_t i = 0; i < n; ++i) {
    send_velocity_command(i, vel[i], tau_gravity[i]);
  }
}

void GimArmSystemHardware::enter_torque_mode()
{
  // Xoá lệnh cũ: chu kỳ đầu rơi vào nhánh dự phòng G(q) chứ không tống ra một
  // giá trị mô-men còn sót từ lần chạy trước.
  std::fill(
    hw_commands_effort_.begin(), hw_commands_effort_.end(),
    std::numeric_limits<double>::quiet_NaN());
}

void GimArmSystemHardware::write_torque_mode()
{
  const size_t n = info_.joints.size();

  const bool all_valid = std::none_of(
    hw_commands_effort_.begin(), hw_commands_effort_.end(),
    [](double c) {return std::isnan(c);});

  bool stale = false;
  if (all_valid) {
    stale = command_stale(hw_commands_effort_);
  } else {
    stale_cycles_ = 0;
  }

  std::vector<double> tau(n, 0.0);
  if (all_valid && !stale) {
    tau = hw_commands_effort_;
  } else {
    // Dự phòng: chỉ bù trọng lực -> tay "không trọng lượng", không bị kéo về
    // đâu cả. KHÔNG phải lưới an toàn thật (bị đẩy là trôi), nhưng hơn hẳn
    // phát 0 Nm (rơi) hoặc giữ nguyên lệnh cũ (chạy mù). Tính từ VỊ TRÍ ĐO
    // được, không phải vị trí lệnh -- ở đây không có vị trí lệnh nào cả.
    if (!compute_gravity_torque(hw_states_position_, tau)) {
      tau.assign(n, 0.0);
    }
    // Tách 2 nhánh thay vì dùng toán tử ba ngôi cho chuỗi định dạng: macro
    // này có thuộc tính format của printf, nên chuỗi phải là string literal
    // thật, không phải biểu thức chọn giữa 2 literal.
    if (stale) {
      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("GimArmSystemHardware"), throttle_clock_, 1000,
        "Lenh mo-men khong doi %d chu ky -- coi nhu nguon phat da chet. "
        "Tut ve bu trong luc.", stale_cycles_);
    } else {
      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("GimArmSystemHardware"), throttle_clock_, 1000,
        "Chua co lenh mo-men hop le (NaN). Tut ve bu trong luc.");
    }
  }

  for (size_t i = 0; i < n; ++i) {
    send_torque_command(i, tau[i]);
  }
}

void GimArmSystemHardware::enter_mit_mode()
{
  // Chốt setpoint tại chỗ tay đang đứng: frame MIT đầu tiên mang kp/kd, nên
  // một setpoint cũ sẽ kéo tay về đó ngay lập tức.
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (!std::isnan(hw_states_position_[i])) {
      hw_commands_[i] = hw_states_position_[i];
    }
  }
  std::fill(
    hw_commands_effort_.begin(), hw_commands_effort_.end(),
    std::numeric_limits<double>::quiet_NaN());
  std::fill(
    hw_commands_velocity_.begin(), hw_commands_velocity_.end(),
    std::numeric_limits<double>::quiet_NaN());

  const bool no_stiffness = std::all_of(
    mit_kp_.begin(), mit_kp_.end(), [](double k) {return k <= 0.0;});
  if (no_stiffness) {
    RCLCPP_WARN(
      rclcpp::get_logger("GimArmSystemHardware"),
      "MIT: mit_kp = 0 o TAT CA cac khop -> khong co do cung, che do nay thoai "
      "hoa thanh mo-men thuan va DRIVER KHONG GIU TAY. Khai <param name=\"mit_kp\"> "
      "trong tung <joint> neu ban muon impedance that.");
  }
}

void GimArmSystemHardware::write_mit_mode()
{
  const size_t n = info_.joints.size();

  // Không có watchdog "nguồn phát chết" ở đây: MIT giữ tay bằng kp/kd quanh
  // setpoint cuối cùng, y như chế độ vị trí. PC im lặng = tay đứng mềm tại chỗ,
  // không rơi và không trôi -- không có gì cần tụt về.
  const bool pos_valid = std::none_of(
    hw_commands_.begin(), hw_commands_.end(), [](double c) {return std::isnan(c);});

  std::vector<double> tau_ff(n, 0.0);
  const bool eff_valid = std::none_of(
    hw_commands_effort_.begin(), hw_commands_effort_.end(),
    [](double c) {return std::isnan(c);});
  if (eff_valid) {
    tau_ff = hw_commands_effort_;
  } else if (gravity_feedforward_ && pos_valid) {
    compute_gravity_torque(hw_commands_, tau_ff);
  }

  for (size_t i = 0; i < n; ++i) {
    if (std::isnan(hw_commands_[i])) {
      continue;  // chưa có setpoint hợp lệ, đừng gửi rác xuống CAN
    }
    const double vel_cmd =
      std::isnan(hw_commands_velocity_[i]) ? 0.0 : hw_commands_velocity_[i];
    send_mit_command(i, hw_commands_[i], vel_cmd, tau_ff[i]);
  }
}

bool GimArmSystemHardware::command_stale(const std::vector<double> & cmd)
{
  // So bit-identical: lệnh từ một vòng điều khiển đang chạy thực tế không lặp
  // lại y hệt hàng chục chu kỳ liền. forward_command_controller ghi lại giá trị
  // cuối cùng mãi mãi nếu node phát chết, nên plugin không có cách nào khác để
  // biết nguồn phát còn sống.
  bool changed = false;
  for (size_t i = 0; i < cmd.size(); ++i) {
    if (!(cmd[i] == last_cmd_seen_[i])) {
      changed = true;
    }
    last_cmd_seen_[i] = cmd[i];
  }
  stale_cycles_ = changed ? 0 : stale_cycles_ + 1;
  return stale_cycles_ > stale_limit_;
}

// ====================================================================
//                            LIFECYCLE
// ====================================================================

hardware_interface::CallbackReturn GimArmSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto n_joints = info_.joints.size();
  hw_commands_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocity_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  hw_commands_effort_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  last_cmd_seen_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  hw_states_position_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  can_node_ids_.resize(n_joints, 0);
  gear_ratios_.resize(n_joints, 8.0);
  directions_.resize(n_joints, 1.0);
  zero_offsets_rad_.resize(n_joints, 0.0);
  max_torque_joint_nm_.resize(n_joints, 2.0);
  max_velocity_joint_rad_s_.resize(n_joints, 1.0);
  mit_kp_.resize(n_joints, 0.0);
  mit_kd_.resize(n_joints, 0.0);

  // node_id riêng từng khớp: khai trong URDF <joint><param name="can_node_id">N</param>
  for (size_t i = 0; i < n_joints; ++i) {
    const auto it = info_.joints[i].parameters.find("can_node_id");
    if (it == info_.joints[i].parameters.end()) {
      RCLCPP_FATAL(
        rclcpp::get_logger("GimArmSystemHardware"),
        "Khớp '%s' thiếu <param name=\"can_node_id\"> trong URDF",
        info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    can_node_ids_[i] = static_cast<uint8_t>(std::stoi(it->second));
  }

  // Tỉ số truyền TỔNG mỗi khớp: khai <param name="gear_ratio">N</param>, không
  // bắt buộc -- thiếu thì mặc định 8.0 (đúng bằng hộp số nội bộ GIM6010-8).
  // Khớp có thêm hộp số ngoài (vd shoulder_joint) PHẢI khai giá trị tổng
  // (8 nội bộ x 8 ngoài = 64), không thì góc thật sẽ sai lệch 8 lần.
  for (size_t i = 0; i < n_joints; ++i) {
    const auto it = info_.joints[i].parameters.find("gear_ratio");
    if (it != info_.joints[i].parameters.end()) {
      gear_ratios_[i] = std::stod(it->second);
    }
    RCLCPP_INFO(
      rclcpp::get_logger("GimArmSystemHardware"),
      "Khớp '%s': gear_ratio = %.3f%s",
      info_.joints[i].name.c_str(), gear_ratios_[i],
      it == info_.joints[i].parameters.end() ? " (mặc định, không khai trong URDF)" : "");
  }

  // Dấu chiều quay: khai <param name="invert_direction">true</param>, không
  // bắt buộc -- thiếu thì mặc định false (+1.0, không đảo).
  for (size_t i = 0; i < n_joints; ++i) {
    const auto it = info_.joints[i].parameters.find("invert_direction");
    const bool inverted = (it != info_.joints[i].parameters.end() && it->second == "true");
    directions_[i] = inverted ? -1.0 : 1.0;
    RCLCPP_INFO(
      rclcpp::get_logger("GimArmSystemHardware"),
      "Khớp '%s': direction = %.0f%s",
      info_.joints[i].name.c_str(), directions_[i], inverted ? " (đã đảo chiều)" : "");
  }

  // Offset "điểm 0" mỗi khớp (rad): khai <param name="zero_offset_rad">X</param>,
  // không bắt buộc -- thiếu thì mặc định 0.0 (dùng luôn "0" thô của encoder).
  // Cách lấy đúng giá trị: để mặc định 0, xoay khớp về đúng tư thế muốn coi
  // là "0", đọc /joint_states lúc đó -- số đọc được chính là giá trị cần điền.
  for (size_t i = 0; i < n_joints; ++i) {
    const auto it = info_.joints[i].parameters.find("zero_offset_rad");
    if (it != info_.joints[i].parameters.end()) {
      zero_offsets_rad_[i] = std::stod(it->second);
    }
    RCLCPP_INFO(
      rclcpp::get_logger("GimArmSystemHardware"),
      "Khớp '%s': zero_offset_rad = %.4f%s",
      info_.joints[i].name.c_str(), zero_offsets_rad_[i],
      it == info_.joints[i].parameters.end() ? " (mặc định, không khai trong URDF)" : "");
  }

  // Trần mô-men riêng từng khớp: khai <param name="max_torque_joint_nm">X</param>
  // TRONG <joint>, không phải trong <hardware>. Thiếu thì mặc định 2.0 Nm --
  // cố tình thấp, để việc quên khai biểu hiện thành tay chạy YẾU (dễ nhận ra,
  // không hỏng gì) thay vì tay chạy quá mạnh.
  for (size_t i = 0; i < n_joints; ++i) {
    const auto it = info_.joints[i].parameters.find("max_torque_joint_nm");
    if (it != info_.joints[i].parameters.end()) {
      max_torque_joint_nm_[i] = std::stod(it->second);
    }
    RCLCPP_INFO(
      rclcpp::get_logger("GimArmSystemHardware"),
      "Khớp '%s': max_torque_joint_nm = %.3f%s",
      info_.joints[i].name.c_str(), max_torque_joint_nm_[i],
      it == info_.joints[i].parameters.end() ? " (mặc định, không khai trong URDF)" : "");
  }

  // Tham số riêng từng khớp của các chế độ THÊM SAU (vận tốc, MIT). Không log
  // từng dòng như 4 khối trên: chúng chỉ có tác dụng khi chế độ tương ứng được
  // kích hoạt, và lúc đó enter_xxx_mode() sẽ nói.
  const auto joint_double = [this](
    size_t i, const char * key, double & target) {
      const auto it = info_.joints[i].parameters.find(key);
      if (it != info_.joints[i].parameters.end()) {
        target = std::stod(it->second);
      }
    };
  for (size_t i = 0; i < n_joints; ++i) {
    joint_double(i, "max_velocity_joint_rad_s", max_velocity_joint_rad_s_[i]);
    joint_double(i, "mit_kp", mit_kp_[i]);
    joint_double(i, "mit_kd", mit_kd_[i]);
  }

  // Tên interface CAN: <ros2_control><hardware><param name="can_interface">can0</param>
  can_interface_name_ = info_.hardware_parameters.count("can_interface")
    ? info_.hardware_parameters.at("can_interface")
    : "can0";

  // ---- Feedforward: mặc định TẮT, phải khai rõ trong URDF mới bật ----
  const auto bool_param = [this](const std::string & key, bool fallback) {
      const auto it = info_.hardware_parameters.find(key);
      return it == info_.hardware_parameters.end() ? fallback : (it->second == "true");
    };
  velocity_feedforward_ = bool_param("velocity_feedforward", false);
  gravity_feedforward_ = bool_param("gravity_feedforward", false);
  // Cùng nguyên tắc: MIT chỉ tồn tại khi khai rõ. Không khai thì claim đồng
  // thời position + effort vẫn bị TỪ CHỐI y như trước.
  mit_enabled_ = bool_param("enable_mit_mode", false);
  if (info_.hardware_parameters.count("max_torque_ff_rotor_nm")) {
    max_torque_ff_rotor_nm_ = std::stod(info_.hardware_parameters.at("max_torque_ff_rotor_nm"));
  }
  if (info_.hardware_parameters.count("effort_stale_cycles")) {
    stale_limit_ = std::stoi(info_.hardware_parameters.at("effort_stale_cycles"));
  }
  // torque_sign: mặc định +1 cho cả ba, ghi đè per-joint bằng
  // <param name="torque_sign">-1</param> TRONG <joint>. Vẫn nhận cả khai ở
  // <hardware> để làm mặc định chung, cho tương thích ngược.
  double sign_default = 1.0;
  if (info_.hardware_parameters.count("torque_sign")) {
    sign_default = std::stod(info_.hardware_parameters.at("torque_sign")) < 0.0 ? -1.0 : 1.0;
  }
  torque_sign_.assign(n_joints, sign_default);
  // Mặc định phía ROTOR = chia gear TỔNG. Sai theo hướng yếu đi, an toàn hơn.
  torque_gear_ratio_.assign(n_joints, 1.0);
  for (size_t i = 0; i < n_joints; ++i) {
    torque_gear_ratio_[i] = gear_ratios_[i];
    const auto it = info_.joints[i].parameters.find("torque_gear_ratio");
    if (it != info_.joints[i].parameters.end()) {
      torque_gear_ratio_[i] = std::stod(it->second);
    }
  }
  for (size_t i = 0; i < n_joints; ++i) {
    const auto it = info_.joints[i].parameters.find("torque_sign");
    if (it != info_.joints[i].parameters.end()) {
      torque_sign_[i] = std::stod(it->second) < 0.0 ? -1.0 : 1.0;
    }
  }

  // Có khớp nào khai <command_interface name="effort"/> hay không -- quyết định
  // việc có phải nạp Pinocchio hay không (xem ngay dưới).
  const bool any_effort_cmd = std::any_of(
    info_.joints.begin(), info_.joints.end(), [](const auto & j) {
      return std::any_of(
        j.command_interfaces.begin(), j.command_interfaces.end(),
        [](const auto & c) {return c.name == hardware_interface::HW_IF_EFFORT;});
    });

  // ---- Nạp mô hình Lagrange để tính G(q) ----
  // Chỉ nạp khi thật sự cần. Ngoài gravity_feedforward, chế độ MÔ-MEN cũng
  // LUÔN cần G(q) cho trạng thái dự phòng khi lệnh mô-men mất hoặc NaN -- nên
  // có khai `effort` là phải nạp, không phụ thuộc gravity_feedforward.
  if (gravity_feedforward_ || any_effort_cmd) {
    std::string urdf_path;
    if (info_.hardware_parameters.count("urdf_path")) {
      urdf_path = info_.hardware_parameters.at("urdf_path");
    } else {
      try {
        urdf_path = ament_index_cpp::get_package_share_directory("gim_arm_description") +
          "/urdf/gim_arm.urdf";
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          rclcpp::get_logger("GimArmSystemHardware"),
          "Không tìm được gim_arm_description để nạp URDF: %s", e.what());
      }
    }

    try {
      pinocchio::urdf::buildModel(urdf_path, model_);
      model_data_ = std::make_unique<pinocchio::Data>(model_);

      // Khớp tên -> chỉ số q/v của Pinocchio. Thứ tự khớp trong <ros2_control>
      // KHÔNG bắt buộc trùng thứ tự Pinocchio dựng cây, nên phải tra theo tên;
      // giả định trùng thứ tự là đúng kiểu lỗi im lặng đẩy sai khớp.
      pin_idx_q_.assign(n_joints, -1);
      pin_idx_v_.assign(n_joints, -1);
      bool all_found = true;
      for (size_t i = 0; i < n_joints; ++i) {
        const std::string & name = info_.joints[i].name;
        if (!model_.existJointName(name)) {
          RCLCPP_ERROR(
            rclcpp::get_logger("GimArmSystemHardware"),
            "URDF '%s' không có khớp '%s' -- tắt bù trọng lực.",
            urdf_path.c_str(), name.c_str());
          all_found = false;
          break;
        }
        const auto jid = model_.getJointId(name);
        pin_idx_q_[i] = static_cast<int>(model_.joints[jid].idx_q());
        pin_idx_v_[i] = static_cast<int>(model_.joints[jid].idx_v());
      }
      model_ready_ = all_found;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("GimArmSystemHardware"),
        "Không nạp được mô hình từ '%s': %s -- tắt bù trọng lực.",
        urdf_path.c_str(), e.what());
      model_ready_ = false;
    }

    if (!model_ready_) {
      gravity_feedforward_ = false;
      if (any_effort_cmd) {
        // Không có G(q) thì nhánh dự phòng của chế độ mô-men chỉ còn 0 Nm =
        // tay rơi. Thà không nạp nổi hardware còn hơn chạy với lưới hỏng.
        RCLCPP_FATAL(
          rclcpp::get_logger("GimArmSystemHardware"),
          "Có khai command_interface 'effort' nhưng KHÔNG nạp được mô hình "
          "Pinocchio. Chế độ mô-men sẽ không có trạng thái dự phòng G(q). Dừng.");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("GimArmSystemHardware"),
    "Feedforward: vel_ff = %s, torque_ff = G(q) %s (trần %.3f Nm phía rotor)",
    velocity_feedforward_ ? "BẬT" : "tắt",
    gravity_feedforward_ ? "BẬT" : "tắt",
    max_torque_ff_rotor_nm_);

  if (any_effort_cmd) {
    RCLCPP_INFO(
      rclcpp::get_logger("GimArmSystemHardware"),
      "Chế độ mô-men KHẢ DỤNG (có command_interface 'effort'): watchdog %d chu kỳ. "
      "Chỉ kích hoạt khi controller claim 'effort'.",
      stale_limit_);
    for (size_t i = 0; i < n_joints; ++i) {
      // In hệ số quy đổi ĐÃ TÍNH RA, không phải tham số thô: để sai hệ số 8 ở
      // gear ngoài lộ ra ngay trong log thay vì lộ ra khi tay võng.
      RCLCPP_INFO(
        rclcpp::get_logger("GimArmSystemHardware"),
        "  '%s': tau_driver = %+.0f x tau_khop / %.1f   "
        "(gear tong %.1f -> %s)",
        info_.joints[i].name.c_str(), torque_sign_[i], torque_gear_ratio_[i],
        gear_ratios_[i],
        (std::abs(torque_gear_ratio_[i] - gear_ratios_[i]) < 1e-6)
        ? "PHIA ROTOR" : "phia khac -- khai torque_gear_ratio trong URDF");
    }
  }

  // Liệt kê thẳng ra các chế độ đang mở và cách kích hoạt từng cái -- để "có
  // những mode nào" là thứ ĐỌC ĐƯỢC TRONG LOG, không phải thứ phải đi đọc code.
  RCLCPP_INFO(
    rclcpp::get_logger("GimArmSystemHardware"),
    "Bảng chế độ (chọn bằng bộ command_interface mà controller claim):\n"
    "  VI TRI  <- claim 'position' [+ 'velocity' = Vel_FF]   (JointTrajectoryController)\n"
    "  VAN TOC <- claim CHI 'velocity'                        %s\n"
    "  MO-MEN  <- claim 'effort'                              %s\n"
    "  MIT     <- claim 'position' + 'effort'                 %s",
    "(chưa kiểm chứng trên phần cứng)",
    any_effort_cmd ? "(sẵn sàng)" : "(URDF chưa khai command_interface 'effort')",
    mit_enabled_ ? "(BẬT, chưa kiểm chứng trên phần cứng)"
                 : "(TẮT -- khai <param name=\"enable_mit_mode\">true</param> để mở)");

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool GimArmSystemHardware::compute_gravity_torque(
  const std::vector<double> & q_joint, std::vector<double> & tau_out)
{
  if (!model_ready_) {
    return false;
  }

  // q của Pinocchio dựng từ chính URDF nên KHÔNG có zero_offset_rad ở đây --
  // offset chỉ là phép dịch khi quy sang đơn vị encoder, xem send_position_command.
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
  for (size_t i = 0; i < q_joint.size(); ++i) {
    if (std::isnan(q_joint[i])) {
      return false;  // q chưa hợp lệ -> G(q) vô nghĩa ở CẢ 3 khớp, không riêng khớp này
    }
    q[pin_idx_q_[i]] = q_joint[i];
  }

  pinocchio::computeGeneralizedGravity(model_, *model_data_, q);

  tau_out.assign(q_joint.size(), 0.0);
  for (size_t i = 0; i < q_joint.size(); ++i) {
    tau_out[i] = model_data_->g[pin_idx_v_[i]];
  }
  return true;
}

hardware_interface::CallbackReturn GimArmSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("GimArmSystemHardware"),
    "Configuring... đang mở SocketCAN interface '%s'", can_interface_name_.c_str());

  if (!can_bus_.open_bus(can_interface_name_)) {
    RCLCPP_FATAL(
      rclcpp::get_logger("GimArmSystemHardware"),
      "Không mở được CAN interface '%s' -- đã chạy "
      "'sudo ip link set %s up type can bitrate <baud>' chưa?",
      can_interface_name_.c_str(), can_interface_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("GimArmSystemHardware"), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ====================================================================
//                       GỬI FRAME XUỐNG CAN
// ====================================================================

void GimArmSystemHardware::send_position_command(
  size_t i, double position_rad, double velocity_rad_s, double torque_ff_joint_nm)
{
  // position_rad đến từ hw_commands_ (không gian URDF, đã trừ zero_offset_rad_)
  // -- cộng lại offset để ra đúng "rad thô" khớp với quy ước encoder thật,
  // TRƯỚC KHI áp dụng gear_ratio/direction như cũ.
  const double position_rad_raw = position_rad + zero_offsets_rad_[i];

  // Set_Input_Pos dùng đơn vị REV, không phải RAD (manual 4.1.2) -- nhân
  // gear_ratios_[i] (tỉ số truyền TỔNG của riêng khớp này), đã xác nhận đúng
  // bằng test thật (lệnh -3.14 rad -> quay đúng 180 độ trên elbow, gear_ratio=8).
  // gear_ratios_[i] quy đổi rad<->rev; directions_[i] (+1/-1) bù chiều lắp
  // đặt vật lý thật của motor, không liên quan tới <axis> trong URDF.
  const double pos_rev = (position_rad_raw * directions_[i] / (2.0 * M_PI)) * gear_ratios_[i];

  // Vel_FF: cùng phép quy đổi như vị trí, trừ zero_offset (đạo hàm của hằng
  // số = 0, giống hệt lý do ở read()).
  const double vel_ff_rev_s =
    (velocity_rad_s * directions_[i] / (2.0 * M_PI)) * gear_ratios_[i];

  // Torque_FF: quy từ mô-men KHỚP về đơn vị mô-men của driver. Chia gear NGOÀI,
  // KHÔNG phải gear tổng, và KHÔNG nhân directions_ -- cùng quy ước với
  // send_torque_command, xem ghi chú đầy đủ ở đó (phép thử treo 1 kg).
  // Firmware nạp torque_constant = 0.669 = 8.27/kv với kv = 12.3 rpm/V, mà
  // 12.3 rpm/V đi cùng bộ rated speed 120 rpm tức PHÍA TRỤC RA. Mọi trường
  // mô-men qua CAN đều dùng chung torque_constant đó, nên đều ở phía trục ra.
  // Chia gear TỔNG là yếu đi đúng 8 lần.
  // Lớp bảo vệ thứ nhất: kẹp ở phía KHỚP bằng đúng trần per-joint mà chế độ
  // mô-men dùng, để hai chế độ không có hai giới hạn khác nhau.
  const double tff_joint = std::clamp(
    torque_ff_joint_nm, -max_torque_joint_nm_[i], max_torque_joint_nm_[i]);

  const double torque_ff_rotor_raw =
    torque_sign_[i] * tff_joint / torque_gear_ratio_[i];
  const double torque_ff_rotor =
    std::clamp(torque_ff_rotor_raw, -max_torque_ff_rotor_nm_, max_torque_ff_rotor_nm_);

  uint8_t data[8];
  gim6010::pack_set_input_pos(data, pos_rev, vel_ff_rev_s, torque_ff_rotor);

  can_bus_.send(gim6010::make_can_id(can_node_ids_[i], gim6010::CmdId::SetInputPos), data, 8);
}

void GimArmSystemHardware::send_velocity_command(
  size_t i, double velocity_rad_s, double torque_ff_joint_nm)
{
  // Trần cứng phía KHỚP -- vai trò y hệt max_torque_joint_nm_ ở chế độ mô-men.
  // Ở chế độ vận tốc, driver KHÔNG có cữ nào cả: một lệnh sai đơn vị (rev/s
  // nhầm thành rad/s) là tay phóng nhanh gấp 2*pi lần dự tính.
  const double v = std::clamp(
    velocity_rad_s, -max_velocity_joint_rad_s_[i], max_velocity_joint_rad_s_[i]);

  // Cùng phép quy đổi rad->rev như vị trí (NHÂN gear_ratio).
  const double vel_rev_s = (v * directions_[i] / (2.0 * M_PI)) * gear_ratios_[i];

  // Torque_FF: chia gear NGOÀI, không nhân directions_ -- giống
  // send_position_command và send_torque_command.
  // Lớp bảo vệ thứ nhất: kẹp ở phía KHỚP bằng đúng trần per-joint mà chế độ
  // mô-men dùng, để hai chế độ không có hai giới hạn khác nhau.
  const double tff_joint = std::clamp(
    torque_ff_joint_nm, -max_torque_joint_nm_[i], max_torque_joint_nm_[i]);
  const double tff_raw = torque_sign_[i] * tff_joint / torque_gear_ratio_[i];
  const double tff = std::clamp(tff_raw, -max_torque_ff_rotor_nm_, max_torque_ff_rotor_nm_);

  uint8_t data[8];
  gim6010::pack_set_input_vel(data, vel_rev_s, tff);
  can_bus_.send(gim6010::make_can_id(can_node_ids_[i], gim6010::CmdId::SetInputVel), data, 8);
}

void GimArmSystemHardware::send_torque_command(size_t i, double torque_joint_nm)
{
  // Trần cứng phía KHỚP -- lớp bảo vệ độc lập với tau_scale của node Python.
  // Một lỗi tham số ROS không được phép đi xuyên qua đây.
  const double tau = std::clamp(
    torque_joint_nm, -max_torque_joint_nm_[i], max_torque_joint_nm_[i]);

  // ĐƠN VỊ: chia gear NGOÀI, KHÔNG phải gear tổng, và KHÔNG nhân directions_.
  //
  // Đo trực tiếp bằng phép thử treo 1 kg đã biết vào đầu công cụ, so độ thay
  // đổi số đọc Get_Torques với J^T·F tính từ mô hình:
  //     shoulder  đo -5.023 Nm / dự đoán -5.182  ->  0.969
  //     elbow     đo -3.628 Nm / dự đoán -3.440  ->  1.055
  // Khớp trong 3-6% khi dùng gear NGOÀI [1, 8, 1] và KHÔNG có thừa số
  // direction. Dùng gear TỔNG [8, 64, 8] cho ra hệ số sai 8 lần.
  //
  // Lý do vật lý: firmware nạp torque_constant = 0.669 = 8.27 / kv với
  // kv = 12.3 rpm/V lấy từ datasheet -- và 12.3 rpm/V đi cùng bộ với rated
  // speed 120 rpm, tức PHÍA TRỤC RA. Nên mọi số mô-men qua CAN đều ở phía trục
  // ra của hộp số nội bộ 8:1, và chỉ còn phải quy đổi phần hộp số NGOÀI.
  // (Datasheet tự mâu thuẫn: nó ghi thêm 0.47 Nm/A, lệch 1.42 lần. Phép thử
  //  treo tải bác bỏ con số đó.)
  const double tau_drv = torque_sign_[i] * tau / torque_gear_ratio_[i];

  uint8_t data[8];
  gim6010::pack_set_input_torque(data, tau_drv);
  can_bus_.send(gim6010::make_can_id(can_node_ids_[i], gim6010::CmdId::SetInputTorque), data, 8);
}

void GimArmSystemHardware::send_mit_command(
  size_t i, double position_rad, double velocity_rad_s, double torque_joint_nm)
{
  // ĐƠN VỊ CỦA 0x008 KHÁC HẲN 3 LỆNH TRÊN: manual 3.1.6 và 4.1.2 nói rõ (2 lần)
  // rằng pos/vel/torque của Mit_Control là PHÍA TRỤC RA của driver, tức SAU hộp
  // số NỘI BỘ 8:1 -- firmware tự quy đổi phần đó. Nên ở đây chỉ được quy đổi
  // phần hộp số NGOÀI. Với shoulder (gear_ratio tổng 64 = 8 nội x 8 ngoài),
  // hệ số đúng là 8, không phải 64. Dùng nhầm gear_ratios_[i] là lệch 8 lần.
  const double ext = gear_ratios_[i] / kDriverInternalRatio;

  const double pos_shaft = (position_rad + zero_offsets_rad_[i]) * directions_[i] * ext;
  const double vel_shaft = velocity_rad_s * directions_[i] * ext;

  // Trần mô-men vẫn áp ở phía KHỚP trước khi quy đổi -- cùng con số, cùng ý
  // nghĩa với chế độ mô-men, để hai chế độ không có hai giới hạn khác nhau.
  const double tau = std::clamp(
    torque_joint_nm, -max_torque_joint_nm_[i], max_torque_joint_nm_[i]);
  // Cùng quy ước với send_torque_command: KHÔNG nhân directions_. Xem ghi chú
  // ở đó (phép thử treo 1 kg). MIT vốn đã ở phía trục ra nên `ext` là đúng.
  const double tau_shaft = torque_sign_[i] * tau / ext;

  // pack_mit_control kẹp pos vào ±12.5 rad phía trục ra. Với ext = 8 (shoulder)
  // thì đó chỉ còn ±1.56 rad phía khớp -- đủ cho tay này, nhưng là một cữ THẦM
  // LẶNG: vượt qua là setpoint bị kẹp chứ không báo lỗi.
  uint8_t data[8];
  gim6010::pack_mit_control(data, pos_shaft, vel_shaft, mit_kp_[i], mit_kd_[i], tau_shaft);
  can_bus_.send(gim6010::make_can_id(can_node_ids_[i], gim6010::CmdId::MitControl), data, 8);
}

// ====================================================================
//                        ĐỔI CHẾ ĐỘ (ros2_control)
// ====================================================================

// Tách phần đuôi "<ten_khop>/<ten_interface>" của một khoá interface.
namespace
{
std::string interface_suffix(const std::string & key)
{
  const auto slash = key.rfind('/');
  return slash == std::string::npos ? key : key.substr(slash + 1);
}
}  // namespace

std::set<std::string> GimArmSystemHardware::projected_claim(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces) const
{
  std::set<std::string> claim = claimed_command_interfaces_;
  for (const auto & key : stop_interfaces) {
    claim.erase(key);
  }
  for (const auto & key : start_interfaces) {
    claim.insert(key);
  }
  return claim;
}

GimArmSystemHardware::ControlMode GimArmSystemHardware::mode_for_claim(
  const std::set<std::string> & claim) const
{
  bool has_pos = false, has_vel = false, has_eff = false;
  for (const auto & key : claim) {
    const std::string name = interface_suffix(key);
    if (name == hardware_interface::HW_IF_POSITION) {
      has_pos = true;
    } else if (name == hardware_interface::HW_IF_VELOCITY) {
      has_vel = true;
    } else if (name == hardware_interface::HW_IF_EFFORT) {
      has_eff = true;
    }
  }
  return resolve_mode(has_pos, has_vel, has_eff, mit_enabled_);
}

hardware_interface::return_type GimArmSystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Tính trên TẬP SẼ ĐƯỢC GIỮ SAU lần switch này, không phải trên start_interfaces.
  // Xem ghi chú dài ở khai báo projected_claim() trong header: dùng thẳng
  // start_interfaces làm chốt chặn này VÔ HIỆU đúng trong kịch bản nguy hiểm
  // nhất -- bật lqi_effort_controller trong khi JTC vẫn đang chạy.
  const std::set<std::string> claim = projected_claim(start_interfaces, stop_interfaces);

  if (claim.empty()) {
    return hardware_interface::return_type::OK;  // không còn ai giữ -> rơi về VỊ TRÍ
  }

  // Chốt chặn quan trọng nhất của cả bản patch. controller_manager KHÔNG tự
  // chặn việc bật đồng thời JTC (position+velocity) và lqi_effort_controller
  // (effort), vì chúng claim 2 bộ interface KHÁC nhau. Nhưng plugin chỉ gửi
  // được MỘT loại frame mỗi chu kỳ, nên bật cả hai là hành vi không xác định
  // trên một thiết bị có thể làm người bị thương.
  //
  // Bây giờ điều kiện này do resolve_mode() phát biểu: bộ interface nào không
  // ứng với đúng MỘT dòng trong bảng chế độ thì từ chối thẳng. (position +
  // effort là ngoại lệ DUY NHẤT, và chỉ khi enable_mit_mode = true.)
  if (mode_for_claim(claim) == ControlMode::Unknown) {
    std::string keys;
    for (const auto & k : claim) {
      keys += (keys.empty() ? "" : ", ") + k;
    }
    RCLCPP_ERROR(
      rclcpp::get_logger("GimArmSystemHardware"),
      "Từ chối: sau lần switch này các command interface được giữ sẽ là {%s} -- "
      "bộ này không ứng với chế độ nào trong bảng. Claim đồng thời 'position' và "
      "'effort' chỉ hợp lệ ở chế độ MIT, và MIT đang %s. "
      "Deactivate controller bên kia trước.",
      keys.c_str(), mit_enabled_ ? "BẬT" : "TẮT");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GimArmSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  const std::set<std::string> claim = projected_claim(start_interfaces, stop_interfaces);

  // Không còn controller nào giữ interface -> rơi về VỊ TRÍ. Đây là chế độ duy
  // nhất mà driver tự giữ tay khi PC im lặng, nên là trạng thái an toàn mặc
  // định. enter_position_mode() lo phần chốt setpoint tại chỗ tay đang đứng --
  // bỏ bước đó thì driver dùng lại input_pos cũ và tay GIẬT về đó.
  //
  // "Rỗng" ở đây là rỗng THẬT (không ai giữ gì nữa), không phải "lần switch này
  // không có ai bật thêm" -- bật/tắt một broadcaster cũng cho start_interfaces
  // rỗng, và trước đây điều đó kéo tay ra khỏi chế độ mô-men giữa chừng.
  if (claim.empty()) {
    claimed_command_interfaces_.clear();
    switch_to_mode(ControlMode::Position);
    return hardware_interface::return_type::OK;
  }

  const ControlMode target = mode_for_claim(claim);
  if (target == ControlMode::Unknown) {
    // prepare_command_mode_switch() đã chặn rồi -- đây là dây bảo hiểm cho
    // trường hợp ros2_control gọi perform mà không gọi prepare. KHÔNG cập nhật
    // claimed_command_interfaces_: lần switch này coi như không xảy ra.
    RCLCPP_ERROR(
      rclcpp::get_logger("GimArmSystemHardware"),
      "perform_command_mode_switch: bộ interface không ứng với chế độ nào. Bỏ qua.");
    return hardware_interface::return_type::ERROR;
  }

  claimed_command_interfaces_ = claim;
  switch_to_mode(target);
  return hardware_interface::return_type::OK;
}

// ====================================================================
//                         ACTIVATE / DEACTIVATE
// ====================================================================

hardware_interface::CallbackReturn GimArmSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("GimArmSystemHardware"), "Activating...");

  // 1) Đặt controller mode = VỊ TRÍ. Không dùng switch_to_mode() ở đây: nó bỏ
  //    qua khi target == active_mode_, mà lúc này ta CẦN gửi 0x00B thật xuống
  //    driver (driver vừa cấp nguồn, hoặc còn nhớ chế độ của phiên trước).
  //    enter_position_mode() cũng chưa gọi được -- chưa đọc nổi encoder, xem
  //    bước 2/3.
  active_mode_ = ControlMode::Position;
  apply_driver_mode(mode_spec(ControlMode::Position));
  std::fill(
    last_cmd_seen_.begin(), last_cmd_seen_.end(), std::numeric_limits<double>::quiet_NaN());
  stale_cycles_ = 0;
  // Chưa controller nào giữ gì lúc vừa activate. Không xoá ở đây thì tập của
  // phiên trước sống sót qua deactivate/activate và chọn nhầm chế độ.
  claimed_command_interfaces_.clear();

  // Socket phải còn mở. Bình thường on_configure đã mở; nếu vì lý do nào đó nó
  // đóng thì mở lại NGAY tại đây thay vì chạy tiếp -- gửi lệnh vào fd = -1 chỉ
  // trả false im lặng, tay đứng yên mà không có dòng lỗi nào.
  if (!can_bus_.is_open() && !can_bus_.open_bus(can_interface_name_)) {
    RCLCPP_FATAL(
      rclcpp::get_logger("GimArmSystemHardware"),
      "CAN interface '%s' không mở được lúc activate.", can_interface_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 2) Vào closed-loop NGAY -- xác nhận bằng candump thật (2026-08-05): driver
  // GIM6010-8 chỉ điền dữ liệu thật vào Get_Encoder_Estimates SAU KHI vào
  // closed-loop, lúc IDLE nó luôn phát 0 dù vẫn broadcast đều 10ms. Không có
  // cách đọc vị trí thật trước khi vào closed-loop -- chấp nhận vào trước,
  // rồi tranh thủ sửa lại càng nhanh càng tốt ở bước 3 để giảm tối đa (không
  // loại bỏ hoàn toàn được) khoảng "giật" lúc driver dùng setpoint cũ/mặc định.
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    uint8_t data[8];
    gim6010::pack_u32_le(data, /*requested_state=*/8);  // AXIS_STATE_CLOSED_LOOP_CONTROL
    can_bus_.send(gim6010::make_can_id(can_node_ids_[i], gim6010::CmdId::SetAxisState), data, 8);
  }

  // 3) Đua để đọc + CHỐT lại setpoint từng khớp NGAY khi có số thật đầu tiên
  // -- không đợi đủ cả 3 khớp mới sửa, khớp nào có số trước sửa trước.
  // Get_Encoder_Estimates broadcast định kỳ 10ms mặc định (manual 4.1.5) nên
  // đợi tối đa 300ms là đủ dư trong điều kiện bình thường.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
  std::vector<bool> corrected(info_.joints.size(), false);

  while (std::chrono::steady_clock::now() < deadline) {
    std::vector<struct can_frame> frames;
    can_bus_.receive_all(frames);

    for (const auto & frame : frames) {
      const uint32_t std_id = frame.can_id & CAN_SFF_MASK;
      const uint8_t node_id = (std_id >> 5) & 0x3F;
      const uint8_t cmd_id = std_id & 0x1F;
      if (cmd_id != static_cast<uint8_t>(gim6010::CmdId::GetEncoderEstimates) ||
        frame.can_dlc < 8)
      {
        continue;
      }
      for (size_t i = 0; i < can_node_ids_.size(); ++i) {
        if (can_node_ids_[i] != node_id || corrected[i]) {
          continue;
        }
        float pos_rev;
        std::memcpy(&pos_rev, &frame.data[0], 4);
        const double pos_rad_raw = (pos_rev / gear_ratios_[i]) * 2.0 * M_PI * directions_[i];
        const double pos_rad = pos_rad_raw - zero_offsets_rad_[i];  // trừ offset "điểm 0"
        hw_commands_[i] = pos_rad;
        hw_states_position_[i] = pos_rad;
        send_position_command(i, pos_rad);  // chốt setpoint ngay, chặn trôi tiếp
        corrected[i] = true;
        RCLCPP_INFO(
          rclcpp::get_logger("GimArmSystemHardware"),
          "Khớp '%s' (node_id %d): CHỐT vị trí thật = %.4f rad ngay sau khi vào closed-loop",
          info_.joints[i].name.c_str(), node_id, pos_rad);
      }
    }

    if (std::all_of(corrected.begin(), corrected.end(), [](bool b) {return b;})) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (!corrected[i]) {
      RCLCPP_WARN(
        rclcpp::get_logger("GimArmSystemHardware"),
        "Khớp '%s' (node_id %d) KHÔNG nhận được phản hồi vị trí thật trong 300ms sau"
        " khi vào closed-loop -- kiểm tra dây CAN/node_id của khớp này.",
        info_.joints[i].name.c_str(), can_node_ids_[i]);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("GimArmSystemHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GimArmSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("GimArmSystemHardware"), "Deactivating...");

  // Trả driver về chế độ vị trí TRƯỚC khi cho IDLE. Bỏ bước này thì lần cấp
  // nguồn sau driver còn nhớ control_mode = 1 trong RAM, và bất cứ thứ gì gửi
  // 0x00C tới nó sẽ bị bỏ qua một cách im lặng.
  // Gọi thẳng apply_driver_mode chứ không switch_to_mode: cần frame 0x00B được
  // gửi thật kể cả khi đang ở sẵn chế độ vị trí, và KHÔNG cần chốt setpoint
  // (ngay sau đây là IDLE, driver nhả lực).
  apply_driver_mode(mode_spec(ControlMode::Position));
  active_mode_ = ControlMode::Position;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    uint8_t data[8];
    gim6010::pack_u32_le(data, /*requested_state=*/1);  // AXIS_STATE_IDLE
    can_bus_.send(gim6010::make_can_id(can_node_ids_[i], gim6010::CmdId::SetAxisState), data, 8);
  }

  // KHÔNG đóng socket ở đây. active -> inactive -> active là một vòng đời hợp
  // lệ và KHÔNG gọi lại on_configure(), nên đóng ở đây khiến lần activate thứ
  // hai chạy với fd = -1: mọi send() trả false im lặng và read() không nhận
  // được frame nào -- tay đứng yên, không một dòng lỗi nào để lần ra. Việc đóng
  // thuộc về on_cleanup()/on_shutdown(), hai transition thật sự có nghĩa "thôi
  // dùng phần cứng này".
  claimed_command_interfaces_.clear();

  RCLCPP_INFO(rclcpp::get_logger("GimArmSystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GimArmSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  can_bus_.close_bus();
  RCLCPP_INFO(rclcpp::get_logger("GimArmSystemHardware"), "CAN đã đóng (cleanup).");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GimArmSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  can_bus_.close_bus();
  RCLCPP_INFO(rclcpp::get_logger("GimArmSystemHardware"), "CAN đã đóng (shutdown).");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ====================================================================
//                        EXPORT INTERFACES
// ====================================================================

std::vector<hardware_interface::StateInterface> GimArmSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GimArmSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));

    // Chỉ xuất "velocity" nếu URDF có khai <command_interface name="velocity"/>
    // cho khớp đó. Xuất bừa một interface không khai trong URDF sẽ khiến
    // resource_manager báo lỗi không khớp và controller_manager không nạp nổi
    // hardware. Đây cũng là lý do giữ được tương thích ngược: URDF cũ (chỉ có
    // position) vẫn chạy y như trước.
    const auto & cmds = info_.joints[i].command_interfaces;
    const bool has_velocity_cmd = std::any_of(
      cmds.begin(), cmds.end(),
      [](const auto & c) {return c.name == hardware_interface::HW_IF_VELOCITY;});
    if (has_velocity_cmd) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_velocity_[i]));
    }

    // "effort": đường vào của LQI. Cùng nguyên tắc như velocity -- chỉ xuất khi
    // URDF khai, để URDF cũ vẫn chạy y như trước.
    const bool has_effort_cmd = std::any_of(
      cmds.begin(), cmds.end(),
      [](const auto & c) {return c.name == hardware_interface::HW_IF_EFFORT;});
    if (has_effort_cmd) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &hw_commands_effort_[i]));
    }
  }
  return command_interfaces;
}

// ====================================================================
//                          READ / WRITE
// ====================================================================

hardware_interface::return_type GimArmSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::vector<struct can_frame> frames;
  can_bus_.receive_all(frames);

  for (const auto & frame : frames) {
    const uint32_t std_id = frame.can_id & CAN_SFF_MASK;
    const uint8_t node_id = (std_id >> 5) & 0x3F;
    const uint8_t cmd_id = std_id & 0x1F;

    if (cmd_id != static_cast<uint8_t>(gim6010::CmdId::GetEncoderEstimates) || frame.can_dlc < 8) {
      continue;  // chỉ xử lý frame quan tâm ở bước này, bỏ qua Heartbeat/khác
    }

    for (size_t i = 0; i < can_node_ids_.size(); ++i) {
      if (can_node_ids_[i] != node_id) {
        continue;
      }

      float pos_rev, vel_rev_s;
      std::memcpy(&pos_rev, &frame.data[0], 4);
      std::memcpy(&vel_rev_s, &frame.data[4], 4);

      // gear_ratios_[i]/directions_[i] riêng của khớp này -- xem giải thích ở on_init().
      // zero_offsets_rad_[i]: chỉ trừ ở VỊ TRÍ, không áp dụng cho vận tốc
      // (offset là 1 hằng số dịch góc, đạo hàm của hằng số = 0).
      hw_states_position_[i] = (static_cast<double>(pos_rev) / gear_ratios_[i]) * 2.0 * M_PI *
        directions_[i] - zero_offsets_rad_[i];
      hw_states_velocity_[i] = (static_cast<double>(vel_rev_s) / gear_ratios_[i]) * 2.0 * M_PI *
        directions_[i];
      break;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GimArmSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Toàn bộ nội dung của write() là: tra bảng, gọi hàm của chế độ đang chạy.
  // Thêm chế độ mới KHÔNG phải sửa hàm này.
  (this->*mode_spec(active_mode_).write_cycle)();
  return hardware_interface::return_type::OK;
}

}  // namespace gim_arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  gim_arm_hardware::GimArmSystemHardware, hardware_interface::SystemInterface)