#ifndef GIM_ARM_HARDWARE__GIM_ARM_SYSTEM_HPP_
#define GIM_ARM_HARDWARE__GIM_ARM_SYSTEM_HPP_

// QUAN TRỌNG: pinocchio/fwd.hpp PHẢI là include đầu tiên trong toàn bộ file
// (trước cả <cstdint>/<memory> tiêu chuẩn) -- theo đúng khuyến nghị chính
// thức của Pinocchio (README dự án), để tránh lỗi biên dịch "using invalid
// field boost::variant<...>::storage_" do kích thước Boost::variant bị xác
// định khác nhau nếu có header khác include boost/variant trước với cấu
// hình khác. Không phải lỗi hệ thống/boost, chỉ là thứ tự include sai.
#include "pinocchio/fwd.hpp"

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

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"

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
  // Gửi 1 lệnh Mit_Control cho khớp `i`: vị trí đích (rad, phía trục ra),
  // vận tốc đích (rad/s), Kp/Kd riêng khớp đó, và mô-men feedforward (Nm) --
  // dùng để cộng thẳng g(q) (bù trọng lực) vào, không đi qua vòng P/PI nội
  // bộ của driver như Set_Input_Pos trước đây.
  void send_mit_command(
    size_t i, double pos_rad, double vel_rad_s, double kp, double kd, double torque_ff_nm);

  // Các mảng lưu trữ giá trị cho 3 khớp (base, shoulder, elbow) -- đơn vị RAD, phía trục ra
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  // Mỗi khớp 1 node_id CAN riêng -- đọc từ <param name="can_node_id"> trong URDF
  std::vector<uint8_t> can_node_ids_;

  // Tỉ số truyền TỔNG mỗi khớp (rotor GIM6010-8 -> góc khớp thật ở URDF).
  // Mặc định 8.0 (đúng bằng hộp số nội bộ của GIM6010-8). Khớp nào có thêm
  // hộp giảm tốc ngoài (vd shoulder_joint: thêm 8:1 -> tổng 64) phải khai rõ
  // <param name="gear_ratio">64.0</param> trong URDF, nếu không sẽ dùng mặc
  // định 8.0 và bị lệch góc thật.
  // LƯU Ý (MIT mode): gear_ratios_ vẫn cần cho read() (quy đổi rev đo được
  // từ Get_Encoder_Estimates ra rad, encoder báo phía ROTOR nên chia tỉ số TỔNG).
  std::vector<double> gear_ratios_;

  // Tỉ số hộp số NGOÀI mỗi khớp = gear_ratios_[i] / kInternalGearRatio.
  //
  // Manual nói field của Mit_Control ở "phía trục ra", nhưng đó là trục ra SAU
  // HỘP SỐ NỘI BỘ 8:1 của chính động cơ -- firmware KHÔNG THỂ biết khớp còn
  // hộp số ngoài nào nữa. Với base/elbow (tổng 8) hệ số này = 1.0, gửi thẳng
  // góc khớp là đúng. Với shoulder (tổng 64 = 8 nội bộ x 8 ngoài) hệ số = 8.0,
  // và nếu KHÔNG quy đổi thì: vị trí chỉ đi 1/8 góc mong muốn, còn torque bù
  // trọng lực bị hộp số ngoài nhân thêm 8 lần -> ở 90 độ là ~34 Nm thay vì
  // 4.31 Nm, đủ để giật tung tay máy.
  //
  // Quy đổi (phía firmware <- phía khớp): vị trí/vận tốc NHÂN, mô-men CHIA
  // (hộp số giảm tốc thì nhân mô-men lên và chia tốc độ xuống).
  std::vector<double> external_ratios_;

  // Dấu chiều quay mỗi khớp: +1.0 (mặc định) hoặc -1.0. Bù cho việc motor
  // được LẮP ĐẶT VẬT LÝ theo chiều khác nhau -- không liên quan gì tới <axis>
  // trong URDF (axis chỉ ảnh hưởng RViz/TF/Jacobian, không chạm vào giá trị
  // gửi xuống CAN). Khai <param name="invert_direction">true</param> nếu
  // khớp đó quay ngược so với ý muốn khi gửi cùng 1 giá trị dương. Áp dụng
  // cho CẢ Set_Input_Pos (cũ) lẫn Mit_Control (mới) -- quy ước dấu vật lý
  // không đổi theo cách gửi lệnh.
  std::vector<double> directions_;

  // Offset "điểm 0" mỗi khớp (rad) -- để mặc định 0.0 kể từ khi chuyển hẳn
  // sang set zero ở tầng driver (không còn cần bù phần mềm). Giữ lại cơ chế
  // này (không xoá) để dự phòng nếu sau này cần dùng lại.
  std::vector<double> zero_offsets_rad_;

  // Kp/Kd riêng từng khớp cho Mit_Control -- KHÁC HẲN pos_gain/vel_gain (đó
  // là gain nội bộ driver dùng cho Set_Input_Pos, không dùng được ở đây).
  // Đơn vị: Kp (Nm/rad), Kd (Nm*s/rad) -- xem <param name="mit_kp">/"mit_kd">
  // trong URDF. CHƯA ĐƯỢC TUNE -- giá trị mặc định chỉ là điểm khởi đầu thận
  // trọng, PHẢI tune riêng bằng thực nghiệm trước khi tin tưởng.
  //
  // QUY ƯỚC ĐƠN VỊ: đây là giá trị gửi THÔ xuống firmware, KHÔNG quy đổi qua
  // external_ratios_ (khác với pos/vel/torque). Lý do: gửi thô thì "số mình
  // ghi = số driver nhận", dễ suy luận khi tune. Hệ quả PHẢI nhớ: độ cứng
  // thực tế cảm nhận Ở KHỚP = mit_kp * external_ratio^2 (hộp số ngoài nhân
  // mô-men r lần VÀ nhân sai số góc r lần). Nên cùng con số mit_kp=2.0 thì
  // shoulder cứng gấp 64 lần base/elbow. on_init() log sẵn giá trị quy đổi ở
  // khớp cho từng khớp -- đọc log đó khi tune, đừng so mit_kp giữa các khớp.
  std::vector<double> mit_kp_;
  std::vector<double> mit_kd_;

  // Tên interface CAN (vd "can0") -- đọc từ <hardware><param name="can_interface">
  std::string can_interface_name_;

  // Mô hình Pinocchio, dùng để tính g(q) (bù trọng lực) mỗi chu kỳ write().
  // Nạp từ <hardware><param name="urdf_path"> -- ĐƯỜNG DẪN TUYỆT ĐỐI tới
  // chính file gim_arm.urdf (không dùng package:// -- Pinocchio không hiểu
  // quy ước đó, giống hệt bài học đã gặp với MuJoCo).
  std::unique_ptr<pinocchio::Model> pin_model_;
  std::unique_ptr<pinocchio::Data> pin_data_;

  // Ánh xạ khớp thứ i của ros2_control -> chỉ số trong vector q/g(q) của
  // Pinocchio. KHÔNG được giả định pin_q_index_[i] == i: Pinocchio tự đánh số
  // theo thứ tự duyệt cây URDF, còn info_.joints theo thứ tự các thẻ <joint>
  // trong <ros2_control>. Với tay 3 DOF nối tiếp hiện tại 2 thứ tự TÌNH CỜ
  // trùng nhau, nhưng chỉ cần thêm khớp / đổi thứ tự thẻ là lệch ngay, và khi
  // lệch thì g(q) sai IM LẶNG (bù trọng lực của khớp này đắp sang khớp khác) --
  // rất khó lần ra. Nên tra cứu theo TÊN khớp một lần ở on_init().
  std::vector<Eigen::Index> pin_q_index_;

  // Bộ đệm q dùng lại mỗi chu kỳ write() -- cấp phát sẵn ở on_init() để
  // không xin bộ nhớ heap trong vòng lặp thời gian thực.
  Eigen::VectorXd pin_q_;

  SocketCanBus can_bus_;
};

}  // namespace gim_arm_hardware

#endif  // GIM_ARM_HARDWARE__GIM_ARM_SYSTEM_HPP_