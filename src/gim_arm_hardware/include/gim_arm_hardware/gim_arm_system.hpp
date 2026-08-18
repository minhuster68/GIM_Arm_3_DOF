#ifndef GIM_ARM_HARDWARE__GIM_ARM_SYSTEM_HPP_
#define GIM_ARM_HARDWARE__GIM_ARM_SYSTEM_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

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
  // Gửi 1 lệnh Set_Input_Pos cho khớp `i`. position_rad / velocity_rad_s /
  // torque_ff_joint_nm đều ở KHÔNG GIAN KHỚP (URDF); hàm này lo phần quy đổi
  // sang phía rotor. Dùng chung bởi write() và on_activate() -- on_activate gọi
  // với 2 tham số sau = 0 (lúc chốt setpoint thì không feedforward gì cả).
  void send_position_command(
    size_t i, double position_rad, double velocity_rad_s = 0.0,
    double torque_ff_joint_nm = 0.0);

  // Tính G(q) -- mô-men chống trọng lực tại từng khớp (Nm, phía khớp) bằng
  // Pinocchio, từ VỊ TRÍ LỆNH. Trả về false nếu mô hình chưa nạp được.
  bool compute_gravity_torque(
    const std::vector<double> & q_joint, std::vector<double> & tau_out);

  // Các mảng lưu trữ giá trị cho 3 khớp (base, shoulder, elbow) -- đơn vị RAD, phía trục ra
  std::vector<double> hw_commands_;

  // Lệnh vận tốc (rad/s, phía khớp). Chỉ tồn tại khi URDF khai
  // <command_interface name="velocity"/> VÀ controller khai velocity trong
  // command_interfaces -- nếu không, mảng này ở nguyên NaN và bị bỏ qua.
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  // Mỗi khớp 1 node_id CAN riêng -- đọc từ <param name="can_node_id"> trong URDF
  std::vector<uint8_t> can_node_ids_;

  // Tỉ số truyền TỔNG mỗi khớp (rotor GIM6010-8 -> góc khớp thật ở URDF).
  // Mặc định 8.0 (đúng bằng hộp số nội bộ của GIM6010-8). Khớp nào có thêm
  // hộp giảm tốc ngoài (vd shoulder_joint: thêm 8:1 -> tổng 64) phải khai rõ
  // <param name="gear_ratio">64.0</param> trong URDF, nếu không sẽ dùng mặc
  // định 8.0 và bị lệch góc thật.
  std::vector<double> gear_ratios_;

  // Dấu chiều quay mỗi khớp: +1.0 (mặc định) hoặc -1.0. Bù cho việc motor
  // được LẮP ĐẶT VẬT LÝ theo chiều khác nhau -- không liên quan gì tới <axis>
  // trong URDF (axis chỉ ảnh hưởng RViz/TF/Jacobian, không chạm vào giá trị
  // gửi xuống CAN). Khai <param name="invert_direction">true</param> nếu
  // khớp đó quay ngược so với ý muốn khi gửi cùng 1 giá trị dương.
  std::vector<double> directions_;

  // Offset "điểm 0" mỗi khớp (rad), áp dụng SAU khi đã quy đổi gear_ratio +
  // direction -- bù cho việc encoder tuyệt đối của driver không có cách nào
  // đặt lại "0" tin cậy qua reboot (đã thử index_offset và set_linear_count
  // của ODrive fork này, cả 2 đều không lưu được qua save_configuration()).
  // Mặc định 0.0 nếu không khai <param name="zero_offset_rad">. Cách lấy giá
  // trị: để mặc định 0, xoay khớp về đúng tư thế muốn coi là "0", đọc
  // /joint_states lúc đó -- số đọc được CHÍNH LÀ giá trị cần điền vào đây.
  std::vector<double> zero_offsets_rad_;

  // Tên interface CAN (vd "can0") -- đọc từ <hardware><param name="can_interface">
  std::string can_interface_name_;

  // ---- Feedforward (xem ghi chú pack_set_input_pos trong gim6010_can_protocol.hpp) ----
  // MẶC ĐỊNH TẮT CẢ HAI. Cố ý: build lại plugin KHÔNG được âm thầm đổi hành vi
  // của một thiết bị đang đeo trên tay người. Muốn bật thì khai rõ trong URDF:
  //   <param name="velocity_feedforward">true</param>
  //   <param name="gravity_feedforward">true</param>
  bool velocity_feedforward_{false};
  bool gravity_feedforward_{false};

  // Trần cho Torque_FF, PHÍA ROTOR (Nm). 0.625 = mô-men định mức phía rotor
  // (5 Nm ở trục ra hộp số nội bộ 8:1). Đây là dây bảo hiểm: nếu mô hình sai
  // hoặc q lệch bất thường thì feedforward cũng không đẩy quá mức định mức.
  // Trọng lực thật lớn nhất trên quỹ đạo chỉ ~0.057 Nm rotor, nên trần này rất
  // rộng -- siết lại được qua <param name="max_torque_ff_rotor_nm">.
  double max_torque_ff_rotor_nm_{0.625};

  // Mô hình Lagrange để tính G(q). Nạp 1 lần ở on_init từ chính URDF.
  bool model_ready_{false};
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> model_data_;
  // info_.joints[i] -> chỉ số trong q / v của Pinocchio (thứ tự khớp của
  // Pinocchio KHÔNG bắt buộc trùng thứ tự khai trong <ros2_control>).
  std::vector<int> pin_idx_q_;
  std::vector<int> pin_idx_v_;

  SocketCanBus can_bus_;
};

}  // namespace gim_arm_hardware

#endif  // GIM_ARM_HARDWARE__GIM_ARM_SYSTEM_HPP_