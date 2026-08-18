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

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
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
  hw_commands_velocity_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  hw_commands_effort_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  last_effort_seen_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  hw_states_position_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  can_node_ids_.resize(n_joints, 0);
  gear_ratios_.resize(n_joints, 8.0);
  directions_.resize(n_joints, 1.0);
  zero_offsets_rad_.resize(n_joints, 0.0);

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
  if (info_.hardware_parameters.count("max_torque_ff_rotor_nm")) {
    max_torque_ff_rotor_nm_ = std::stod(info_.hardware_parameters.at("max_torque_ff_rotor_nm"));
  }
  if (info_.hardware_parameters.count("max_torque_joint_nm")) {
    max_torque_joint_nm_ = std::stod(info_.hardware_parameters.at("max_torque_joint_nm"));
  }
  if (info_.hardware_parameters.count("effort_stale_cycles")) {
    effort_stale_limit_ = std::stoi(info_.hardware_parameters.at("effort_stale_cycles"));
  }
  if (info_.hardware_parameters.count("torque_sign")) {
    torque_sign_ = std::stod(info_.hardware_parameters.at("torque_sign")) < 0.0 ? -1.0 : 1.0;
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
      "Chế độ mô-men KHẢ DỤNG (có command_interface 'effort'): trần %.3f Nm "
      "phía khớp, torque_sign = %+.0f, watchdog %d chu kỳ. Chỉ kích hoạt khi "
      "controller claim 'effort'.",
      max_torque_joint_nm_, torque_sign_, effort_stale_limit_);
  }

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

  // Torque_FF: quy từ mô-men KHỚP về mô-men ROTOR -- CHIA cho gear_ratio, ngược
  // chiều với vị trí (vị trí NHÂN). Nhân nhầm ở đây là shoulder lệch 64 lần.
  // directions_ vẫn phải có: khớp đảo chiều mà quên là feedforward đẩy ngược,
  // chống lại chính vòng vị trí.
  const double torque_ff_rotor_raw =
    torque_ff_joint_nm / (gear_ratios_[i] * directions_[i]);
  const double torque_ff_rotor =
    std::clamp(torque_ff_rotor_raw, -max_torque_ff_rotor_nm_, max_torque_ff_rotor_nm_);

  uint8_t data[8];
  gim6010::pack_set_input_pos(data, pos_rev, vel_ff_rev_s, torque_ff_rotor);

  can_bus_.send(gim6010::make_can_id(can_node_ids_[i], gim6010::CmdId::SetInputPos), data, 8);
}

void GimArmSystemHardware::send_torque_command(size_t i, double torque_joint_nm)
{
  // Trần cứng phía KHỚP -- lớp bảo vệ độc lập với tau_scale của node Python.
  // Một lỗi tham số ROS không được phép đi xuyên qua đây.
  const double tau = std::clamp(torque_joint_nm, -max_torque_joint_nm_, max_torque_joint_nm_);

  // CHIA gear_ratio -- NGƯỢC chiều với vị trí (vị trí NHÂN). Nhân nhầm ở đây là
  // shoulder lệch 64 lần. directions_ vẫn phải có: khớp đảo chiều mà quên là
  // mô-men đẩy ngược, và ở chế độ mô-men KHÔNG còn vòng vị trí nào kéo lại.
  // torque_sign_ là thứ KHÁC: quy ước dấu của firmware (Input_Torque dương làm
  // encoder tăng hay giảm), đo bằng cansend -- xem RUNBOOK Phase 2.
  const double tau_rotor =
    torque_sign_ * tau / (gear_ratios_[i] * directions_[i]);

  uint8_t data[8];
  gim6010::pack_set_input_torque(data, tau_rotor);
  can_bus_.send(gim6010::make_can_id(can_node_ids_[i], gim6010::CmdId::SetInputTorque), data, 8);
}

void GimArmSystemHardware::set_all_control_mode(bool torque)
{
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    uint8_t data[8];
    // (control_mode, input_mode). 1 = TORQUE_CONTROL, 3 = POSITION_CONTROL.
    gim6010::pack_u32_le(data, torque ? 1u : 3u, 1u);
    can_bus_.send(
      gim6010::make_can_id(can_node_ids_[i], gim6010::CmdId::SetControllerMode), data, 8);
  }
}

hardware_interface::return_type GimArmSystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  bool want_pos = false, want_eff = false;
  for (const auto & key : start_interfaces) {
    if (key.find("/" + std::string(hardware_interface::HW_IF_EFFORT)) != std::string::npos) {
      want_eff = true;
    }
    if (key.find("/" + std::string(hardware_interface::HW_IF_POSITION)) != std::string::npos) {
      want_pos = true;
    }
  }

  // Chốt chặn quan trọng nhất của cả bản patch. controller_manager KHÔNG tự
  // chặn việc bật đồng thời JTC (position+velocity) và lqi_effort_controller
  // (effort), vì chúng claim 2 bộ interface KHÁC nhau. Nhưng plugin chỉ gửi
  // được MỘT loại frame mỗi chu kỳ, nên bật cả hai là hành vi không xác định
  // trên một thiết bị có thể làm người bị thương. Từ chối thẳng.
  if (want_pos && want_eff) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GimArmSystemHardware"),
      "Từ chối: không được claim đồng thời 'position' và 'effort'. "
      "Deactivate controller bên kia trước.");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GimArmSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (const auto & key : stop_interfaces) {
    if (key.find("/" + std::string(hardware_interface::HW_IF_EFFORT)) == std::string::npos) {
      continue;
    }
    // RỜI chế độ mô-men -> phải chốt setpoint vị trí tại CHỖ TAY ĐANG ĐỨNG
    // trước khi trả quyền cho vòng vị trí của driver. Bỏ bước này thì driver
    // dùng lại input_pos cũ từ trước lúc chuyển sang mô-men, và tay GIẬT về đó.
    set_all_control_mode(false);
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      send_position_command(i, hw_states_position_[i], 0.0, 0.0);
      hw_commands_[i] = hw_states_position_[i];
    }
    torque_mode_active_ = false;
    RCLCPP_INFO(
      rclcpp::get_logger("GimArmSystemHardware"), "-> CHE DO VI TRI (control_mode=3)");
    break;
  }

  for (const auto & key : start_interfaces) {
    if (key.find("/" + std::string(hardware_interface::HW_IF_EFFORT)) == std::string::npos) {
      continue;
    }
    // Xoá lệnh cũ: chu kỳ đầu rơi vào nhánh dự phòng G(q) chứ không tống ra một
    // giá trị mô-men còn sót từ lần chạy trước.
    std::fill(
      hw_commands_effort_.begin(), hw_commands_effort_.end(),
      std::numeric_limits<double>::quiet_NaN());
    std::fill(
      last_effort_seen_.begin(), last_effort_seen_.end(),
      std::numeric_limits<double>::quiet_NaN());
    effort_stale_cycles_ = 0;
    set_all_control_mode(true);
    torque_mode_active_ = true;
    RCLCPP_WARN(
      rclcpp::get_logger("GimArmSystemHardware"),
      "-> CHE DO MO-MEN (control_mode=1). DRIVER KHONG CON GIU TAY. "
      "PC ngung gui = tay roi. Khong chay che do nay khi tay dang deo tren nguoi.");
    break;
  }
  return hardware_interface::return_type::OK;
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
    gim6010::pack_u32_le(data, /*control_mode=*/3, /*input_mode=*/1);
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
  set_all_control_mode(false);
  torque_mode_active_ = false;

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
  const size_t n = info_.joints.size();

  // ================= NHÁNH MÔ-MEN (LQI) =================
  if (torque_mode_active_) {
    const bool all_valid = std::none_of(
      hw_commands_effort_.begin(), hw_commands_effort_.end(),
      [](double c) {return std::isnan(c);});

    // Phát hiện lệnh ĐỨNG YÊN. forward_command_controller ghi lại giá trị cuối
    // cùng mãi mãi nếu lqi_node chết, nên plugin không có cách nào khác để biết
    // nguồn phát còn sống. So bit-identical: mô-men từ một vòng LQI đang chạy
    // thực tế không lặp lại y hệt hàng chục chu kỳ liền.
    bool changed = false;
    if (all_valid) {
      for (size_t i = 0; i < n; ++i) {
        if (!(hw_commands_effort_[i] == last_effort_seen_[i])) {
          changed = true;
        }
        last_effort_seen_[i] = hw_commands_effort_[i];
      }
      effort_stale_cycles_ = changed ? 0 : effort_stale_cycles_ + 1;
    } else {
      effort_stale_cycles_ = 0;
    }
    const bool stale = effort_stale_cycles_ > effort_stale_limit_;

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
      RCLCPP_ERROR_THROTTLE(
        rclcpp::get_logger("GimArmSystemHardware"), *rclcpp::Clock().get_clock(), 1000,
        stale
        ? "Lenh mo-men khong doi %d chu ky -- coi nhu nguon phat da chet. Tut ve bu trong luc."
        : "Chua co lenh mo-men hop le (NaN). Tut ve bu trong luc. (%d)",
        effort_stale_cycles_);
    }

    for (size_t i = 0; i < n; ++i) {
      send_torque_command(i, tau[i]);
    }
    return hardware_interface::return_type::OK;
  }

  // ================= NHÁNH VỊ TRÍ (giữ NGUYÊN như cũ) =================
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

  return hardware_interface::return_type::OK;
}

}  // namespace gim_arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  gim_arm_hardware::GimArmSystemHardware, hardware_interface::SystemInterface)