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

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"

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
  gear_ratios_.resize(n_joints, gim6010::kInternalGearRatio);
  external_ratios_.resize(n_joints, 1.0);
  directions_.resize(n_joints, 1.0);
  zero_offsets_rad_.resize(n_joints, 0.0);
  mit_kp_.resize(n_joints, 2.0);   // mac dinh RAT THAN TRONG, chua tune
  mit_kd_.resize(n_joints, 0.1);   // mac dinh RAT THAN TRONG, chua tune

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
    if (!(gear_ratios_[i] >= gim6010::kInternalGearRatio)) {
      // Nhỏ hơn hộp số nội bộ là vô nghĩa vật lý, và sẽ cho external_ratio < 1
      // -> quy đổi mô-men sai theo chiều KHUẾCH ĐẠI. Chặn ngay, đừng để chạy.
      RCLCPP_FATAL(
        rclcpp::get_logger("GimArmSystemHardware"),
        "Khớp '%s': gear_ratio = %.3f không hợp lệ -- phải >= %.1f (hộp số nội bộ của "
        "GIM6010-8). Giá trị khai là tỉ số TỔNG (nội bộ x ngoài).",
        info_.joints[i].name.c_str(), gear_ratios_[i], gim6010::kInternalGearRatio);
      return hardware_interface::CallbackReturn::ERROR;
    }
    external_ratios_[i] = gear_ratios_[i] / gim6010::kInternalGearRatio;
    RCLCPP_INFO(
      rclcpp::get_logger("GimArmSystemHardware"),
      "Khớp '%s': gear_ratio = %.3f%s -> hộp số ngoài = %.3f",
      info_.joints[i].name.c_str(), gear_ratios_[i],
      it == info_.joints[i].parameters.end() ? " (mặc định, không khai trong URDF)" : "",
      external_ratios_[i]);
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

  // Kp/Kd riêng từng khớp cho Mit_Control: khai <param name="mit_kp">/"mit_kd">,
  // không bắt buộc -- thiếu thì dùng mặc định RẤT THẬN TRỌNG (2.0/0.1).
  // BẮT BUỘC tune lại bằng thực nghiệm trước khi tin tưởng số này cho việc gì
  // quan trọng -- đây không phải pos_gain/vel_gain đã tune trước đó, khác cơ
  // chế hoàn toàn (xem giải thích trong .hpp).
  for (size_t i = 0; i < n_joints; ++i) {
    const auto it_kp = info_.joints[i].parameters.find("mit_kp");
    if (it_kp != info_.joints[i].parameters.end()) {
      mit_kp_[i] = std::stod(it_kp->second);
    }
    const auto it_kd = info_.joints[i].parameters.find("mit_kd");
    if (it_kd != info_.joints[i].parameters.end()) {
      mit_kd_[i] = std::stod(it_kd->second);
    }
    // Độ cứng CẢM NHẬN Ở KHỚP = mit_kp * r^2 (xem .hpp) -- log ra để khi tune
    // biết mình đang thật sự đặt bao nhiêu, đừng so mit_kp thô giữa các khớp.
    const double r2 = external_ratios_[i] * external_ratios_[i];
    RCLCPP_INFO(
      rclcpp::get_logger("GimArmSystemHardware"),
      "Khớp '%s': mit_kp=%.3f, mit_kd=%.3f%s -> quy đổi Ở KHỚP: Kp=%.3f Nm/rad, "
      "Kd=%.3f Nm*s/rad",
      info_.joints[i].name.c_str(), mit_kp_[i], mit_kd_[i],
      (it_kp == info_.joints[i].parameters.end()) ? " (CHƯA TUNE, dùng mặc định)" : "",
      mit_kp_[i] * r2, mit_kd_[i] * r2);

    // Field Kp/Kd của Mit_Control chỉ 12 bit (Kp 0..500, Kd 0..5), nên bước
    // lượng tử là 500/4095 và 5/4095. Đặt giá trị nhỏ hơn 1 bước thì firmware
    // nhận đúng 0 -> khớp MẤT HẲN vòng phản hồi vị trí, chỉ còn torque bù
    // trọng lực (hở vòng, sẽ trôi/rơi). Đây là kiểu hỏng rất dễ tưởng là
    // "tune chưa tới", nên cảnh báo thẳng.
    const double kp_lsb = (gim6010::kKpMax - gim6010::kKpMin) / 4095.0;
    const double kd_lsb = (gim6010::kKdMax - gim6010::kKdMin) / 4095.0;
    if (mit_kp_[i] > 0.0 && mit_kp_[i] < kp_lsb) {
      RCLCPP_WARN(
        rclcpp::get_logger("GimArmSystemHardware"),
        "Khớp '%s': mit_kp=%.5f NHỎ HƠN 1 bước lượng tử (%.5f) -> firmware nhận Kp=0, "
        "khớp sẽ KHÔNG giữ vị trí. Dùng mit_kp >= %.5f.",
        info_.joints[i].name.c_str(), mit_kp_[i], kp_lsb, kp_lsb);
    }
    if (mit_kd_[i] > 0.0 && mit_kd_[i] < kd_lsb) {
      RCLCPP_WARN(
        rclcpp::get_logger("GimArmSystemHardware"),
        "Khớp '%s': mit_kd=%.5f NHỎ HƠN 1 bước lượng tử (%.5f) -> firmware nhận Kd=0 "
        "(không giảm chấn). Dùng mit_kd >= %.5f.",
        info_.joints[i].name.c_str(), mit_kd_[i], kd_lsb, kd_lsb);
    }
  }

  // Nạp mô hình Pinocchio để tính g(q) mỗi chu kỳ write() -- cần đường dẫn
  // TUYỆT ĐỐI tới file URDF (Pinocchio không hiểu package://, giống hệt bài
  // học đã gặp với MuJoCo trước đây).
  {
    const auto it_urdf = info_.hardware_parameters.find("urdf_path");
    if (it_urdf == info_.hardware_parameters.end()) {
      RCLCPP_FATAL(
        rclcpp::get_logger("GimArmSystemHardware"),
        "Thiếu <hardware><param name=\"urdf_path\">/đường/dẫn/tuyệt/đối/gim_arm.urdf</param> "
        "-- bắt buộc để tính g(q) bù trọng lực qua Pinocchio.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    try {
      pin_model_ = std::make_unique<pinocchio::Model>();
      pinocchio::urdf::buildModel(it_urdf->second, *pin_model_);
      pin_data_ = std::make_unique<pinocchio::Data>(*pin_model_);
      RCLCPP_INFO(
        rclcpp::get_logger("GimArmSystemHardware"),
        "Đã nạp mô hình Pinocchio từ '%s' (%d khớp) để tính g(q).",
        it_urdf->second.c_str(), pin_model_->njoints - 1);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("GimArmSystemHardware"),
        "Lỗi nạp URDF vào Pinocchio từ '%s': %s", it_urdf->second.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Tra cứu chỉ số q của Pinocchio theo TÊN khớp -- xem giải thích dài ở
    // khai báo pin_q_index_ trong .hpp (tuyệt đối không giả định trùng thứ tự).
    pin_q_index_.resize(n_joints);
    for (size_t i = 0; i < n_joints; ++i) {
      const std::string & jname = info_.joints[i].name;
      if (!pin_model_->existJointName(jname)) {
        RCLCPP_FATAL(
          rclcpp::get_logger("GimArmSystemHardware"),
          "Khớp '%s' khai trong <ros2_control> nhưng KHÔNG có trong mô hình Pinocchio nạp từ "
          "'%s' -- sai tên khớp, hoặc khớp đó là 'fixed' trong URDF.",
          jname.c_str(), it_urdf->second.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      const auto jid = pin_model_->getJointId(jname);
      pin_q_index_[i] = pin_model_->joints[jid].idx_q();
      RCLCPP_INFO(
        rclcpp::get_logger("GimArmSystemHardware"),
        "Khớp '%s': ros2_control[%zu] -> pinocchio q[%ld]",
        jname.c_str(), i, static_cast<long>(pin_q_index_[i]));
    }

    // Số bậc tự do phải khớp: nếu URDF có khớp revolute/prismatic KHÁC ngoài 3
    // khớp khai trong <ros2_control> (vd thêm gripper), q sẽ thiếu phần tử và
    // computeGeneralizedGravity() ném exception NGAY TRONG write() -- tức là
    // giữa vòng lặp thời gian thực, lúc tay máy đang mang lực. Chặn ngay ở đây.
    if (pin_model_->nq != static_cast<int>(n_joints)) {
      RCLCPP_FATAL(
        rclcpp::get_logger("GimArmSystemHardware"),
        "Mô hình Pinocchio có nq=%d nhưng <ros2_control> khai %zu khớp -- không khớp. "
        "Mọi khớp chuyển động trong URDF đều phải được khai trong <ros2_control>.",
        pin_model_->nq, n_joints);
      return hardware_interface::CallbackReturn::ERROR;
    }
    pin_q_ = Eigen::VectorXd::Zero(pin_model_->nq);
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

void GimArmSystemHardware::send_mit_command(
  size_t i, double pos_rad, double vel_rad_s, double kp, double kd, double torque_ff_nm)
{
  // Mit_Control nhận giá trị phía trục ra SAU HỘP SỐ NỘI BỘ 8:1, KHÔNG phải
  // góc khớp thật -- nên vẫn phải quy đổi qua hộp số NGOÀI. Xem giải thích dài
  // ở external_ratios_ trong .hpp. Với base/elbow ratio = 1.0 nên không đổi gì;
  // chỉ shoulder (ratio 8.0) thực sự bị quy đổi.
  //
  // directions_[i]: quy ước DẤU vật lý của cách lắp motor, không phụ thuộc lệnh
  // CAN nào đang dùng.
  //
  // zero_offsets_rad_[i]: pos_rad đến từ hw_commands_ (không gian góc khớp
  // URDF, read() ĐÃ TRỪ offset đi), nên ở chiều ngược lại phải CỘNG LẠI để về
  // không gian thô của encoder -- đúng như send_position_command() cũ đã làm.
  // Hiện offset mặc định 0.0 nên không cộng cũng chưa sai, nhưng thiếu dòng
  // này thì cơ chế offset (vẫn giữ để dự phòng) sẽ hỏng IM LẶNG khi dùng lại:
  // read() trừ mà write() không cộng -> lệch đúng bằng offset.
  const double r = external_ratios_[i];
  const double pos_signed = (pos_rad + zero_offsets_rad_[i]) * directions_[i] * r;
  const double vel_signed = vel_rad_s * directions_[i] * r;
  // Mô-men CHIA cho r: hộp số ngoài nhân mô-men lên r lần trên đường ra khớp,
  // nên muốn khớp nhận đúng torque_ff_nm thì firmware chỉ được cấp 1/r.
  const double torque_signed = torque_ff_nm * directions_[i] / r;

  uint8_t data[8];
  gim6010::pack_mit_control(data, pos_signed, vel_signed, kp, kd, torque_signed);
  can_bus_.send(gim6010::make_can_id(can_node_ids_[i], gim6010::CmdId::MitControl), data, 8);
}

hardware_interface::CallbackReturn GimArmSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("GimArmSystemHardware"), "Activating...");

  // 1) Đặt controller mode.
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    uint8_t data[8];
    // control_mode=3 (position), input_mode=9 (Mit_Control) -- chuyển hẳn
    // sang MIT mode để có kênh torque_ff (bù trọng lực), khác input_mode=3
    // (Filtered Position) đã dùng suốt giai đoạn tune PID trước đây.
    gim6010::pack_u32_le(data, /*control_mode=*/3, /*input_mode=*/9);
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
        const double pos_rad_raw = (pos_rev / gear_ratios_[i]) * 2.0 * M_PI * directions_[i];
        const double pos_rad = pos_rad_raw - zero_offsets_rad_[i];  // trừ offset "điểm 0"
        hw_commands_[i] = pos_rad;
        hw_states_position_[i] = pos_rad;
        // Chốt setpoint ngay, chặn trôi tiếp -- torque_ff=0 ở ĐÚNG thời điểm
        // này vì chưa chắc đã có đủ vị trí thật của cả 3 khớp để tính g(q)
        // đúng (mỗi khớp về closed-loop KHÔNG cùng lúc). write() ngay sau
        // đây sẽ tính g(q) đầy đủ mỗi chu kỳ khi đã có đủ dữ liệu.
        send_mit_command(i, pos_rad, /*vel_rad_s=*/0.0, mit_kp_[i], mit_kd_[i], /*torque_ff_nm=*/0.0);
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
  // Pinocchio cần ĐỦ cả 3 vị trí thật mới tính g(q) có nghĩa -- thiếu 1 khớp
  // (vd chưa kịp đọc CAN lần đầu) sẽ làm sai cho cả 3, không chỉ khớp thiếu.
  bool have_full_state = true;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (std::isnan(hw_states_position_[i])) {
      have_full_state = false;
      break;
    }
  }

  if (have_full_state) {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      // Dùng VỊ TRÍ THẬT (hw_states_position_), không phải desired -- trọng
      // lực tác động theo cấu hình vật lý THẬT của tay máy ngay lúc này.
      // hw_states_position_ đã ở quy ước góc khớp của URDF (đã áp directions_
      // và zero_offsets_rad_ trong read()), đúng hệ quy chiếu Pinocchio cần.
      pin_q_[pin_q_index_[i]] = hw_states_position_[i];
    }
    // Trả về tham chiếu tới pin_data_->g, không cấp phát -- nhận bằng const &.
    pinocchio::computeGeneralizedGravity(*pin_model_, *pin_data_, pin_q_);
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (std::isnan(hw_commands_[i])) {
      continue;  // chưa có lệnh hợp lệ, đừng gửi rác xuống CAN
    }
    // g(q) là mô-men cần cấp để GIỮ tay máy chống trọng lực (phương trình
    // động lực học M*qdd + C + g = tau), nên bù trọng lực là CỘNG +g(q).
    const double torque_ff = have_full_state ? pin_data_->g[pin_q_index_[i]] : 0.0;
    // vel_rad_s=0: command_interfaces hiện chỉ có "position", chưa có
    // velocity feedforward -- để dành cải tiến sau nếu cần bám nhanh hơn.
    send_mit_command(i, hw_commands_[i], /*vel_rad_s=*/0.0, mit_kp_[i], mit_kd_[i], torque_ff);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace gim_arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  gim_arm_hardware::GimArmSystemHardware, hardware_interface::SystemInterface)