# Tune SMC trên tay thật — bộ chốt 2026-10-24

Branch `smc-test`. Chỉ còn một YAML phần cứng:
`src/gim_arm_controller_smc/config/smc_circle_from_terminal_tuning.yaml`.
Launch vòng tròn và terminal tune mặc định cùng đọc file này. Các bản tuning,
snapshot và bản ghép chéo cũ đã được xóa để tránh chọn nhầm.

Bộ chốt dùng super-twisting, FF đầy đủ theo tham chiếu và phản hồi độc lập:

```text
e_i = q_i - q_ref_i
ed_i = qd_used_i - qd_ref_i
s_i = ed_i + lambda_i * e_i
a_fb_i = -lambda_i * ed_i - k1_i * sqrt(abs(s_i)) * sign(s_i) + z_i
z_dot_i = -k2_i * sign(s_i)
tau_ff = inverse_dynamics(q_ref, qd_ref, qdd_ref)
tau_fb_i = M_ii(q_measured) * a_fb_i
tau_cmd = clip(tau_ff + slew(tau_fb), torque_limits)
```

Riêng tau_ff base nhân alpha=2.46. gravity_scale=[1,1,1], không trừ thêm G3.
Phản hồi độc lập do feedback_offdiag_scale=[0,0,0]; tương tác cơ khí vẫn còn.
Thứ tự vector luôn là [base/id0, shoulder/id1, elbow/id2]:

| Tham số | Base | Shoulder | Elbow |
|---|---:|---:|---:|
| lambda_gain | 48 | 32 | 12 |
| k1 | 10 | 12 | 3.5 |
| k2 | 4 | 3 | 4 |
| z_limit (rad/s²) | 20 | 40 | 30 |
| torque_slew_rate (Nm/s) | 6 | 16 | 1.5 |

Control 100 Hz, tau_scale=1. Kalman [v,a] chỉ bật elbow: jerk PSD=1,
measurement variance=0.0004; LPF tắt. Giá trị k1 elbow=3.5 là giá trị đã chạy
vòng tròn mới, không phải 4.5 của lượt tune độc lập cũ.

## 1. Chuẩn bị terminal và phần cứng

Trong mỗi terminal mới:

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
```

Đặt và đỡ tay đúng HOME cơ khí trước khi đặt software zero. Chỉ giữ một launch
phần cứng. Tune khớp nào thì chọn torque_joint khớp đó; ví dụ elbow/id2:

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 \
  set_zero_on_startup:=true \
  zero_method:=software \
  torque_joint:=elbow
```

Nếu vừa đổi branch/source, build trước khi launch:

```bash
colcon build --symlink-install --packages-select gim_control gim_arm_controller_smc
source install/setup.bash
```

Chờ position controller active, effort inactive. HOME ROS chỉ đúng HOME cơ khí
nếu mốc zero lúc launch đúng. Lệnh về ROS 0 khi position đang active:

```bash
ros2 action send_goal --feedback /gim_arm_group_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [base_joint, shoulder_joint, elbow_joint], points: [{positions: [0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0], time_from_start: {sec: 8}}]}}"
```

## 2. Tune thử mà không ghi đè bộ chốt

Chỉ cần chạy vòng tròn thì không tune lại: xem README_SMC_INDEPENDENT_FEEDBACK.md.
Nếu muốn thử gain khác, sao bộ chốt ra file tạm rồi mở terminal tune:

```bash
cp src/gim_arm_controller_smc/config/smc_circle_from_terminal_tuning.yaml /tmp/smc_trial.yaml
bash tools/tune_smc_terminal.sh --joint elbow --params-file /tmp/smc_trial.yaml --log-dir results_new/smc/joint_trials --prestart-timeout 12
```

Nếu bỏ --params-file, terminal đọc trực tiếp bộ chốt và lệnh save sẽ ghi vào
bộ chốt. --joint chọn khớp điều khiển cho từng bài, không cần YAML riêng cho
base/shoulder/elbow. Tuner tự đặt controlled_joint cho node thử; file vòng tròn
vẫn giữ controlled_joint: all khi save.

```text
show
test 3 6 2
```

Test đi thêm 3° từ HOME, 6 giây mỗi chiều, giữ 2 giây ở mỗi đầu. Chỉ nhập bài
biên độ tiếp theo sau khi đọc kết quả bài trước và kiểm tra HOME. Hai khớp còn
lại được giữ position. Tuner tự switch controller và trả position khi kết thúc.
Terminal có guard riêng: sai số khớp thử 2°, khớp giữ trôi 1°, trạng thái quá
hạn và giới hạn URDF; các guard này độc lập với cờ abort của profile vòng tròn.

Terminal mở socket CAN chỉ đọc: chờ ba Get_Torques hợp lệ ở vận tốc thấp,
sau switch kiểm tra máy tính gửi frame torque mode và lệnh torque. Đây là
kiểm tra frame gửi, không phải ACK mode nội bộ driver. Không qua bàn giao thì
không bắt đầu quỹ đạo. Khi Ctrl+C trong test, chờ thông báo trả position.

## 3. Chỉnh tham số

| Lệnh/tham số | Có thể tăng khi | Có thể giảm khi | Tác dụng |
|---|---|---|---|
| lambda VALUE | Bám vị trí chậm, còn dư torque | Vượt đích/rung khi đảo chiều | Đặt quan hệ vị trí-vận tốc trên mặt trượt; cũng có trong -lambda*ed |
| k1 VALUE | Rời mặt trượt lâu, hồi phục chậm | Torque/vận tốc giật nhanh | Tăng hiệu chỉnh tức thời theo sqrt(abs(s)) |
| k2 VALUE | Lệch dai dẳng khi giữ/tải thay đổi | Vượt khi đảo chiều, z tích lũy kéo ngược | Tăng tốc tích lũy z, không tích phân trực tiếp e |
| zlimit VALUE | z chạm giới hạn lâu và bù không đủ | z còn lớn sau khi sai số đổi dấu | Trần biên độ gia tốc tích lũy |
| slew VALUE | Torque bị giới hạn tốc độ đổi nhiều, gây trễ | Torque thay đổi quá gấp | Trần tốc độ đổi feedback, không đổi giới hạn tổng torque |
| alpha VALUE | Cần hiệu chỉnh FF base đã được kiểm chứng | FF base quá lớn theo phép đo | Chỉ dùng khi chọn base; không thay dấu torque |

Ví dụ chỉnh elbow (chưa tự ghi file):

```text
set 12 3.5 4
slew 1.5
zlimit 30
test 3 6 2
save
```

save ghi đúng đường dẫn show in ra. reset đọc lại file, bỏ thay đổi chưa save.
next chuyển elbow → shoulder → base nhưng vẫn phải restart hardware với
đúng torque_joint ở HOME cơ khí. Nên đổi một tham số mỗi lượt, so cùng bài.

CSV và ảnh mặc định ở results_new/smc/joint_trials. Mỗi khớp có đồ thị vị trí/
torque và vận tốc/sai số vận tốc riêng. Xem cả RMS/MAX, slew_limited, z và
vận tốc; không chỉ nhìn đường vị trí. Giữ log cũ làm bằng chứng từng lượt,
không thay tham số lịch sử trong CSV để giống bộ chốt.
