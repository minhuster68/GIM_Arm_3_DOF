#include "gim_arm_hardware/gim_arm_system.hpp"

#include <linux/can.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gim_arm_hardware
{

hardware_interface::CallbackReturn GimArmSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto n_joints = info_.joints.size();
  hw_commands_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  hw_states_position_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  can_node_ids_.resize(n_joints, 0);
  gear_ratios_.resize(n_joints, 8.0);
  directions_.resize(n_joints, 1.0);

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

  // Tên interface CAN: <ros2_control><hardware><param name="can_interface">can0</param>
  can_interface_name_ = info_.hardware_parameters.count("can_interface")
    ? info_.hardware_parameters.at("can_interface")
    : "can0";

  return hardware_interface::CallbackReturn::SUCCESS;
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

void GimArmSystemHardware::send_position_command(size_t i, double position_rad)
{
  // Set_Input_Pos dùng đơn vị REV, không phải RAD (manual 4.1.2) -- nhân
  // gear_ratios_[i] (tỉ số truyền TỔNG của riêng khớp này), đã xác nhận đúng
  // bằng test thật (lệnh -3.14 rad -> quay đúng 180 độ trên elbow, gear_ratio=8).
  // gear_ratios_[i] quy đổi rad<->rev; directions_[i] (+1/-1) bù chiều lắp
  // đặt vật lý thật của motor, không liên quan tới <axis> trong URDF.
  const double pos_rev = (position_rad * directions_[i] / (2.0 * M_PI)) * gear_ratios_[i];
  const float pos_rev_f = static_cast<float>(pos_rev);
  const int16_t vel_ff = 0;     // chưa dùng feedforward ở bước bench-test này
  const int16_t torque_ff = 0;

  uint8_t data[8];
  std::memcpy(&data[0], &pos_rev_f, 4);
  std::memcpy(&data[4], &vel_ff, 2);
  std::memcpy(&data[6], &torque_ff, 2);

  can_bus_.send(gim6010::make_can_id(can_node_ids_[i], gim6010::CmdId::SetInputPos), data, 8);
}

hardware_interface::CallbackReturn GimArmSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("GimArmSystemHardware"), "Activating...");

  // 1) Đặt controller mode.
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    uint8_t data[8];
    // control_mode=3 (position), input_mode=3 (filtered position) -- manual
    // 3.1.6. CHƯA dùng Mit_Control ở bước bench-test này.
    gim6010::pack_u32_le(data, /*control_mode=*/3, /*input_mode=*/3);
    can_bus_.send(
      gim6010::make_can_id(can_node_ids_[i], gim6010::CmdId::SetControllerMode), data, 8);
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
        const double pos_rad = (pos_rev / gear_ratios_[i]) * 2.0 * M_PI * directions_[i];
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

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    uint8_t data[8];
    gim6010::pack_u32_le(data, /*requested_state=*/1);  // AXIS_STATE_IDLE
    can_bus_.send(gim6010::make_can_id(can_node_ids_[i], gim6010::CmdId::SetAxisState), data, 8);
  }
  can_bus_.close_bus();

  RCLCPP_INFO(rclcpp::get_logger("GimArmSystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

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
  }
  return command_interfaces;
}

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
      hw_states_position_[i] = (static_cast<double>(pos_rev) / gear_ratios_[i]) * 2.0 * M_PI *
        directions_[i];
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
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (std::isnan(hw_commands_[i])) {
      continue;  // chưa có lệnh hợp lệ, đừng gửi rác xuống CAN
    }
    send_position_command(i, hw_commands_[i]);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace gim_arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  gim_arm_hardware::GimArmSystemHardware, hardware_interface::SystemInterface)