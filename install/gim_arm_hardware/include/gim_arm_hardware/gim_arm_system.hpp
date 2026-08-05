#ifndef GIM_ARM_HARDWARE__GIM_ARM_SYSTEM_HPP_
#define GIM_ARM_HARDWARE__GIM_ARM_SYSTEM_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "gim_arm_hardware/gim6010_can_protocol.hpp"
#include "gim_arm_hardware/socketcan_bus.hpp"

namespace gim_arm_hardware
{
class GimArmSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GimArmSystemHardware)

  // 1. Hàm khởi tạo: Đọc thông số từ URDF
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // 2. Hàm cấu hình: Mở SocketCAN (chưa cấp lực)
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  // 3. Hàm kích hoạt: chuyển mode, vào closed-loop, đọc vị trí hiện tại
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 4. Hàm hủy kích hoạt: về IDLE, đóng CAN an toàn
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 5. Khai báo bộ nhớ chia sẻ cho State (Encoder đọc về)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // 6. Khai báo bộ nhớ chia sẻ cho Command (Lệnh bắn xuống)
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 7. Vòng lặp Read: Đọc CAN bus
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // 8. Vòng lặp Write: Gửi lệnh xuống CAN bus
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Các mảng lưu trữ giá trị cho 3 khớp (base, shoulder, elbow) -- đơn vị RAD, phía trục ra
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  // Mỗi khớp 1 node_id CAN riêng -- đọc từ <param name="can_node_id"> trong URDF
  std::vector<uint8_t> can_node_ids_;

  // Tên interface CAN (vd "can0") -- đọc từ <hardware><param name="can_interface">
  std::string can_interface_name_;

  SocketCanBus can_bus_;

  // Tỉ số truyền GIM6010-8: 8:1. CẦN xác nhận bằng test thật (xem TODO trong .cpp)
  // trước khi tin số này -- manual chỉ xác nhận rõ Mit_Control là phía trục ra,
  // không nói rõ Set_Input_Pos/Get_Encoder_Estimates là rotor hay trục ra.
  static constexpr double kGearRatio = 8.0;
};

}  // namespace gim_arm_hardware

#endif  // GIM_ARM_HARDWARE__GIM_ARM_SYSTEM_HPP_