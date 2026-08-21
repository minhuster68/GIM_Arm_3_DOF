#ifndef GIM_ARM_HARDWARE__GIM_ARM_SYSTEM_HPP_
#define GIM_ARM_HARDWARE__GIM_ARM_SYSTEM_HPP_

#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
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

  // 4. Hàm hủy kích hoạt: về IDLE. KHÔNG đóng CAN -- xem on_cleanup().
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 4b. Đóng SocketCAN. PHẢI ở đây chứ không ở on_deactivate: vòng đời
  //     active -> inactive -> active là hợp lệ và KHÔNG gọi lại on_configure,
  //     nên đóng socket lúc deactivate khiến lần activate thứ hai chạy với
  //     fd = -1 -- mọi send() trả false im lặng, read() không có frame nào,
  //     tay đứng yên mà không có một dòng lỗi nào. on_cleanup/on_shutdown mới
  //     là 2 transition thật sự nghĩa là "thôi dùng phần cứng này".
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  // 5. Khai báo bộ nhớ chia sẻ cho State (Encoder đọc về)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // 6. Khai báo bộ nhớ chia sẻ cho Command (Lệnh bắn xuống)
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 6b. ros2_control gọi 2 hàm này khi controller đổi trạng thái. Đây là chỗ
  //     ĐÚNG để đổi control_mode của driver: chúng chạy ĐỒNG BỘ với việc
  //     controller nhả/giữ command interface, nên không có cửa sổ thời gian nào
  //     mà driver ở chế độ này còn controller lại tưởng nó ở chế độ kia.
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // 7. Vòng lặp Read: Đọc CAN bus
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // 8. Vòng lặp Write: Gửi lệnh xuống CAN bus
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ==================================================================
  //                      BẢNG CHẾ ĐỘ ĐIỀU KHIỂN
  // ==================================================================
  // Mọi chế độ được VIẾT SẴN ở đây, mỗi chế độ một dòng trong bảng
  // mode_spec(). Lúc chạy, plugin chỉ TRỎ vào một dòng và gọi các hàm trong
  // đó -- không có `if (che_do == ...)` nào rải rác trong write().
  //
  // THÊM MỘT CHẾ ĐỘ MỚI = 3 việc, không hơn:
  //   1) thêm một hằng vào enum ControlMode
  //   2) viết 2 hàm thành viên enter_xxx_mode() / write_xxx_mode()
  //   3) thêm một dòng vào bảng trong mode_spec()
  // resolve_mode() quyết định interface nào chọn chế độ nào; write(),
  // perform_command_mode_switch() và on_deactivate() KHÔNG cần sửa.
  //
  // GIỚI HẠN CẦN BIẾT: cơ chế này phân biệt được các chế độ dùng BỘ COMMAND
  // INTERFACE KHÁC NHAU, vì đó là thứ duy nhất ros2_control báo cho plugin.
  // Hai biến thể cùng dùng `position` (vd "vị trí có bù trọng lực" vs "vị trí
  // trơn") thì switch_controllers KHÔNG phân biệt được -- phải chọn bằng param
  // URDF lúc launch (như gravity_feedforward hiện nay), hoặc thêm một command
  // interface "mode" riêng để controller ghi số chế độ vào.
  enum class ControlMode : uint8_t
  {
    Position,   // 0x00C Set_Input_Pos  -- driver giữ vòng vị trí (mặc định, an toàn nhất)
    Velocity,   // 0x00D Set_Input_Vel  -- driver giữ vòng vận tốc
    Torque,     // 0x00E Set_Input_Torque -- driver KHÔNG giữ gì cả (đường LQI)
    Mit,        // 0x008 Mit_Control    -- impedance: pos + vel + kp/kd + tau_ff
    Unknown,    // bộ interface không ứng với chế độ nào -> từ chối switch
  };

  // Một dòng của bảng. Tất cả những gì phân biệt chế độ này với chế độ khác
  // đều nằm ở đây -- không có thông tin nào về chế độ bị bỏ quên ở nơi khác.
  struct ModeSpec
  {
    ControlMode id;
    const char * name;            // dùng cho log, viết KHÔNG DẤU vì đi qua printf
    uint32_t drv_control_mode;    // trường 0 của Set_Controller_Mode (0x00B)
    uint32_t drv_input_mode;      // trường 1 của Set_Controller_Mode (0x00B)
    // false = driver không tự giữ tay, PC ngừng gửi là tay rơi/trôi. Quyết định
    // log lúc vào chế độ là INFO hay WARN. Cố ý để trong bảng chứ không suy ra
    // từ id: thêm chế độ mới là phải trả lời câu hỏi này ngay tại dòng đó.
    bool driver_holds_arm;
    // Chạy MỘT LẦN ngay sau khi driver đã nhận control_mode mới. Chỗ để chốt
    // setpoint an toàn / xoá lệnh cũ. Không có on_exit: mọi lần rời chế độ đều
    // kéo theo một lần vào chế độ khác, nên việc dọn dẹp thuộc về enter của
    // chế độ ĐÍCH -- nơi biết rõ trạng thái an toàn cần lập là gì.
    void (GimArmSystemHardware::*on_enter)();
    // Chạy mỗi chu kỳ write(). Đây là toàn bộ nội dung của write().
    void (GimArmSystemHardware::*write_cycle)();
  };

  // Tra bảng. static: để lấy được con trỏ tới hàm thành viên private, hàm này
  // phải nằm TRONG lớp.
  static const ModeSpec & mode_spec(ControlMode m);

  // Bộ interface đang được claim -> chế độ tương ứng. Đây là chỗ DUY NHẤT quy
  // định "interface nào chọn mode nào".
  static ControlMode resolve_mode(bool has_pos, bool has_vel, bool has_eff, bool mit_enabled);

  // Tập command interface sẽ được giữ SAU khi lần switch này hoàn tất =
  // (đang giữ - stop_interfaces) + start_interfaces.
  //
  // VÌ SAO KHÔNG DÙNG THẲNG start_interfaces: ros2_control chỉ đưa vào
  // start_interfaces những interface của controller ĐANG BẬT LẦN NÀY, không
  // phải toàn bộ những gì đang được giữ. Nên nếu chỉ nhìn start_interfaces:
  //   - JTC đang chạy (position+velocity), bật thêm lqi_effort_controller ->
  //     start = [effort] -> "chế độ mô-men" -> ĐƯỢC CHẤP NHẬN, driver đổi sang
  //     control_mode = 1 trong khi JTC vẫn đang ghi vị trí. Đây đúng là thứ mà
  //     chốt chặn trong prepare_command_mode_switch() nói là nó chặn.
  //   - ngược lại, bật/tắt một broadcaster (không claim command interface nào)
  //     -> start rỗng -> tưởng là "không còn ai giữ" -> kéo tay về chế độ VỊ
  //     TRÍ ngay giữa lúc LQI đang chạy mô-men.
  // Cả hai chỉ biến mất khi tính trên TẬP ĐANG GIỮ.
  std::set<std::string> projected_claim(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) const;

  // Tra chế độ từ một tập command interface đầy đủ.
  ControlMode mode_for_claim(const std::set<std::string> & claim) const;

  // Gửi Set_Controller_Mode (0x00B) của một chế độ xuống cả 3 driver.
  void apply_driver_mode(const ModeSpec & spec);

  // Đổi chế độ: gửi 0x00B -> đặt active_mode_ -> chạy on_enter -> log.
  // Không làm gì nếu đã ở đúng chế độ đó.
  void switch_to_mode(ControlMode target);

  // ---- Các chế độ: mỗi chế độ đúng 2 hàm, xem bảng trong mode_spec() ----
  void enter_position_mode();
  void write_position_mode();
  void enter_velocity_mode();
  void write_velocity_mode();
  void enter_torque_mode();
  void write_torque_mode();
  void enter_mit_mode();
  void write_mit_mode();

  // ==================================================================
  //                      GỬI FRAME XUỐNG CAN
  // ==================================================================
  // Gửi 1 lệnh Set_Input_Pos cho khớp `i`. position_rad / velocity_rad_s /
  // torque_ff_joint_nm đều ở KHÔNG GIAN KHỚP (URDF); hàm này lo phần quy đổi
  // sang phía rotor. Dùng chung bởi write_position_mode() và on_activate() --
  // on_activate gọi với 2 tham số sau = 0 (lúc chốt setpoint thì không
  // feedforward gì cả).
  void send_position_command(
    size_t i, double position_rad, double velocity_rad_s = 0.0,
    double torque_ff_joint_nm = 0.0);

  // Gửi 1 lệnh Set_Input_Vel (0x00D). velocity_rad_s ở KHÔNG GIAN KHỚP, bị kẹp
  // bởi max_velocity_joint_rad_s_[i] trước khi quy về phía rotor.
  void send_velocity_command(
    size_t i, double velocity_rad_s, double torque_ff_joint_nm = 0.0);

  // Gửi 1 lệnh Set_Input_Torque (0x00E) cho khớp `i`. torque_joint_nm ở KHÔNG
  // GIAN KHỚP; hàm này lo quy đổi về phía rotor (CHIA gear_ratio, ngược chiều
  // với vị trí vốn NHÂN).
  void send_torque_command(size_t i, double torque_joint_nm);

  // Gửi 1 lệnh Mit_Control (0x008). CẨN THẬN: frame này dùng đơn vị PHÍA TRỤC
  // RA CỦA DRIVER (sau hộp số NỘI BỘ 8:1), không phải phía rotor và cũng không
  // phải phía khớp. Với khớp có hộp số NGOÀI (shoulder 64 = 8 nội x 8 ngoài),
  // hàm này chỉ quy đổi phần NGOÀI = gear_ratio / 8.
  void send_mit_command(
    size_t i, double position_rad, double velocity_rad_s, double torque_joint_nm);

  // Tính G(q) -- mô-men chống trọng lực tại từng khớp (Nm, phía khớp) bằng
  // Pinocchio, từ VỊ TRÍ LỆNH. Trả về false nếu mô hình chưa nạp được.
  bool compute_gravity_torque(
    const std::vector<double> & q_joint, std::vector<double> & tau_out);

  // Watchdog "nguồn phát đã chết". Trả về true khi vector lệnh KHÔNG đổi một
  // bit nào quá stale_limit_ chu kỳ liên tiếp. Dùng chung cho MỌI chế độ mà
  // driver không tự giữ tay (Torque, Velocity) -- xem ghi chú ở last_cmd_seen_.
  bool command_stale(const std::vector<double> & cmd);

  // Các mảng lưu trữ giá trị cho 3 khớp (base, shoulder, elbow) -- đơn vị RAD, phía trục ra
  std::vector<double> hw_commands_;

  // Lệnh vận tốc (rad/s, phía khớp). Ở chế độ Position nó là Vel_FF; ở chế độ
  // Velocity nó LÀ lệnh chính. Chỉ tồn tại khi URDF khai
  // <command_interface name="velocity"/> -- nếu không, mảng này ở nguyên NaN.
  std::vector<double> hw_commands_velocity_;

  // Lệnh MÔ-MEN (Nm, phía khớp). Chỉ tồn tại khi URDF khai
  // <command_interface name="effort"/>. NaN = controller chưa ghi gì.
  std::vector<double> hw_commands_effort_;

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

  // Hộp số NỘI BỘ của GIM6010-8. Hằng số vật lý của động cơ, không phải tham
  // số cấu hình. Chỉ Mit_Control cần tới nó (xem send_mit_command).
  static constexpr double kDriverInternalRatio = 8.0;

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

  // Các command interface ĐANG được controller giữ, dạng khoá đầy đủ
  // "<ten_khop>/<ten_interface>". Cập nhật trong perform_command_mode_switch()
  // -- đó là nơi duy nhất ros2_control báo cho plugin biết ai giữ cái gì.
  std::set<std::string> claimed_command_interfaces_;

  // ---- Chế độ đang chạy ----
  // Thay cho `bool torque_mode_active_` cũ. Mặc định Position: chế độ duy nhất
  // mà driver tự giữ tay khi PC im lặng, nên là trạng thái an toàn để rơi về.
  ControlMode active_mode_{ControlMode::Position};

  // Mit_Control PHẢI khai rõ mới bật được: <param name="enable_mit_mode">true</param>.
  // Mặc định false -> claim đồng thời position+effort vẫn bị TỪ CHỐI y như
  // trước bản refactor này. Cùng lý do với velocity_feedforward: build lại
  // plugin không được âm thầm biến một đường bị chặn thành một đường chạy được.
  bool mit_enabled_{false};

  // Đếm số chu kỳ liên tiếp mà lệnh KHÔNG đổi một bit nào.
  // forward_command_controller ghi lại giá trị cuối MÃI MÃI nếu node phát chết,
  // nên plugin không có cách nào khác để phân biệt "mới" với "cũ". Lệnh từ một
  // vòng điều khiển đang sống thực tế không lặp lại y hệt hàng chục chu kỳ liền.
  // Dùng chung cho mọi chế độ: chỉ một chế độ chạy tại một thời điểm, và
  // switch_to_mode() xoá sạch trạng thái này mỗi lần đổi.
  std::vector<double> last_cmd_seen_;
  int stale_cycles_{0};
  int stale_limit_{50};              // 50 chu kỳ @100Hz = 0.5 s

  // Trần cứng phía KHỚP cho chế độ mô-men (Nm), RIÊNG TỪNG KHỚP. Lớp bảo vệ
  // cuối, độc lập với tau_scale bên node Python.
  //
  // PHẢI theo từng khớp, không được dùng một số chung: ba khớp lệch nhau 8 lần
  // về mô-men. Mô-men thật sự cần trên quỹ đạo (feedforward đỉnh + phần phản
  // hồi tối đa): base 1.58, shoulder 4.99, elbow 1.80 Nm. Một trần chung 2.5
  // sẽ CHẶN MẤT 50% mô-men của shoulder -> tay võng ở vai, và bạn sẽ chẩn đoán
  // sai thành lỗi dấu hoặc lỗi mô hình trọng lực.
  //
  // Khai <param name="max_torque_joint_nm">X</param> TRONG TỪNG <joint>.
  // Thiếu thì lấy mặc định của khớp đó = giá trị dưới, cố tình đặt thấp để
  // việc quên khai lộ ra bằng tay chạy yếu, không phải bằng tay chạy quá mạnh.
  std::vector<double> max_torque_joint_nm_;

  // Trần cứng VẬN TỐC phía KHỚP (rad/s), riêng từng khớp -- vai trò y hệt
  // max_torque_joint_nm_ nhưng cho chế độ Velocity. Khai
  // <param name="max_velocity_joint_rad_s">X</param> trong <joint>.
  // Mặc định 1.0 rad/s: cố tình chậm, quên khai thì tay bò chứ không phóng.
  std::vector<double> max_velocity_joint_rad_s_;

  // Độ cứng / giảm chấn cho Mit_Control, đơn vị PHÍA TRỤC RA DRIVER
  // (Nm/rad và Nm*s/rad). Khai <param name="mit_kp">, <param name="mit_kd">
  // trong <joint>. Mặc định 0 = MIT thoái hoá thành chế độ mô-men thuần: cố ý,
  // để việc quên khai biểu hiện thành tay mềm oặt (thấy ngay) chứ không thành
  // tay giật về setpoint bằng một độ cứng ngẫu nhiên.
  std::vector<double> mit_kp_;
  std::vector<double> mit_kd_;

  // Đồng hồ cho RCLCPP_*_THROTTLE. PHẢI là biến thành viên (lvalue): macro
  // bind [&c = clock], nên truyền tạm thời như rclcpp::Clock() sẽ không biên
  // dịch được. Dùng STEADY_TIME để việc bóp log không phụ thuộc /clock.
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

  // Dấu quy ước mô-men của FIRMWARE: +1 nếu Input_Torque dương làm encoder
  // TĂNG, -1 nếu ngược. Đo bằng cansend (RUNBOOK Phase 2) rồi khai
  // <param name="torque_sign">-1</param> nếu cần. Đây KHÔNG phải
  // invert_direction -- cái đó đã nằm trong directions_.
  // Dấu quy ước mô-men, RIÊNG TỪNG KHỚP. Khai
  // <param name="torque_sign">-1</param> trong <joint> nếu cần đảo.
  //
  // VÌ SAO PHẢI PER-JOINT, và vì sao KHÔNG nhân directions_:
  // Đo bằng phép thử treo 1 kg (xem ghi chú trong send_torque_command) cho thấy
  // τ_khớp = τ_driver × gear_NGOÀI, KHÔNG có thừa số direction, ở cả shoulder
  // (direction = +1) lẫn elbow (direction = -1). Nếu áp công thức công ảo với
  // ánh xạ vị trí thì đáng ra phải có direction -- mâu thuẫn đó chỉ giải thích
  // được nếu <axis> của khớp đó vốn đã ngược so với vật lý và invert_direction
  // được thêm vào để bù, khiến hai lần đảo triệt tiêu nhau.
  // Vì không phân biệt được hai trường hợp từ dữ liệu hiện có, để mặc định +1
  // cho cả ba và bắt buộc kiểm bằng phép thử "thả nổi G(q)" trước khi bám.
  std::vector<double> torque_sign_;

  // Hệ số CHIA khi quy mô-men KHỚP -> đơn vị mô-men của driver:
  //     tau_driver = torque_sign * tau_khop / torque_gear_ratio
  //
  // VÌ SAO TÁCH RIÊNG KHỎI gear_ratios_: đường ĐỌC và đường GHI có thể ở hai
  // phía khác nhau, và ta đã đo được là chúng KHÁC nhau.
  //   - Get_Torques (0x01C): phép thử treo 1 kg cho thấy phía TRỤC RA, tức
  //     tau_khop = tau_driver x gear_NGOÀI [1, 8, 1].
  //   - Set_Input_Torque (0x00E): manual (chú thích trong gim6010_can_protocol.hpp)
  //     nói phía ROTOR, tức phải chia gear_TỔNG [8, 64, 8].
  // Chênh nhau đúng 8 lần (hộp số nội bộ). Dùng nhầm gear ngoài cho đường ghi
  // là phát gấp 8 lần -- tay bị đẩy ngược lên chống lại trọng lực.
  //
  // MẶC ĐỊNH = gear_ratios_ (phía rotor). Chọn mặc định này vì nếu sai thì nó
  // sai theo hướng YẾU đi (tay võng xuống), không phải mạnh lên (tay bật lên).
  // Khai <param name="torque_gear_ratio">1.0</param> trong <joint> để đổi.
  std::vector<double> torque_gear_ratio_;

  // Khớp nào ĐƯỢC PHÉP vào chế độ mô-men. Mặc định true cho cả ba.
  //
  // MỤC ĐÍCH: cô lập từng khớp khi dò dấu/hệ số. control_mode đặt RIÊNG cho
  // từng driver, nên có thể để 2 khớp ở chế độ vị trí (driver giữ cứng) trong
  // khi chỉ 1 khớp thả tự do. Thí nghiệm một biến, đọc kết quả không mơ hồ.
  // Nếu thả cả 3 cùng lúc thì động lực học ghép chéo: khớp này rơi làm đổi
  // G(q) của khớp kia, và không biết chuyển động là do dấu sai hay do ghép.
  //
  // Khai <param name="torque_mode_enable">false</param> trong <joint> để khoá
  // khớp đó lại. Khớp bị khoá vẫn nhận Set_Input_Pos giữ tại chỗ nó đang đứng
  // lúc chuyển chế độ.
  std::vector<bool> torque_mode_enable_;

  // Vị trí chốt cho các khớp bị khoá, lấy tại thời điểm chuyển sang chế độ
  // mô-men. Không dùng hw_commands_ vì controller đang active là
  // lqi_effort_controller, nó không ghi vào mảng đó.
  std::vector<double> hold_position_;

  // ---- Feedforward (xem ghi chú pack_set_input_pos trong gim6010_can_protocol.hpp) ----
  // MẶC ĐỊNH TẮT CẢ HAI. Cố ý: build lại plugin KHÔNG được âm thầm đổi hành vi
  // của một thiết bị đang đeo trên tay người. Muốn bật thì khai rõ trong URDF:
  //   <param name="velocity_feedforward">true</param>
  //   <param name="gravity_feedforward">true</param>
  bool velocity_feedforward_{false};
  bool gravity_feedforward_{false};

  // Trần cho Torque_FF, tính ở PHÍA TRỤC RA của driver (Nm) -- cùng đơn vị với
  // mọi trường mô-men qua CAN, xem ghi chú ở send_torque_command.
  // 5.0 = mô-men định mức của GIM6010-8 ở trục ra.
  //
  // TRƯỚC ĐÂY LÀ 0.625 và ĐÓ LÀ LỖI: 0.625 là định mức phía ROTOR, dùng khi
  // tưởng nhầm các trường mô-men ở phía rotor. Với đơn vị đúng, trần 0.625 sẽ
  // kẹp mất feedforward trọng lực ở base (cần tới 1.18 Nm) và elbow (1.52 Nm)
  // xuống còn 41% và 60% -- tức bật gravity_feedforward lên mà gần như không
  // có tác dụng, và không có gì báo lỗi.
  //
  // Đây là lớp bảo vệ THỨ HAI. Lớp thứ nhất là max_torque_joint_nm_ per-joint,
  // áp ở phía KHỚP trước khi quy đổi, nên hai chế độ vị trí và mô-men dùng
  // chung một giới hạn.
  double max_torque_ff_rotor_nm_{5.0};

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