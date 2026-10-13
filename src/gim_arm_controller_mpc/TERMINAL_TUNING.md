# Tune MPC từng khớp trên tay thật

Làm việc ở `/home/mirabo/GIM_Arm_3_DOF`, branch `mpc-test`, trong cùng thư
mục với các branch khác. Không dùng worktree riêng. Bộ LQR đã hoàn tất
được lưu trên branch `lqr-test`.

Tune theo thứ tự elbow → shoulder → base. Khớp đang chọn nhận torque;
hai khớp còn lại giữ position bằng motor. MPC tính
`tau = tau_ff + tau_fb`, với tau_ff là inverse dynamics từ đúng URDF đọc
từ `/controller_manager`, tau_fb là input đầu tiên của QP cho trạng thái
`[integral(q-q_ref), q-q_ref, qd-qd_ref]`. Chỉ feedforward base được nhân
`base_ff_alpha`. Alpha được áp dụng cả trong tính miền feedback còn lại
để tổng torque giữ trong giới hạn.

Profile `config/mpc_joint_tuning.yaml` là bộ ban đầu để tune MPC:
100 Hz, tau_scale=1, alpha base=2.2, slew `[5,20,5]` Nm/s, tích phân ±0.05.
Q có trọng số vận tốc; đánh giá tracking bằng RMS/max sai số vị trí.
Q/R của LQR không được dùng làm bộ MPC đã tune.

## Khởi động elbow

Thoát terminal LQR, chờ position active rồi dừng launch phần cứng cũ.
Đặt/đỡ tay tại đúng tư thế zero trước startup, vì software zero chụp
encoder lúc này làm mốc.

Terminal 1:

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 set_zero_on_startup:=true \
  zero_method:=software torque_joint:=elbow
```

Terminal 2:

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
ros2 action send_goal --feedback \
  /gim_arm_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [base_joint, shoulder_joint, elbow_joint], points: [{positions: [0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0], time_from_start: {sec: 8}}]}}"
```

Chờ goal SUCCEEDED, rồi mở terminal tune tự source ROS/install và domain 42:

```bash
bash tools/tune_mpc_terminal.sh --joint elbow --interface can0
```

Trong prompt `mpc[khâu 3/elbow]>`:

```text
show
test 3 6 3
```

`test 3 6 3` đi thêm 3° từ HOME đo trước switch trong 6 s, giữ 3 s,
quay về HOME trong 6 s rồi giữ 3 s. Không bật autostart bằng lệnh riêng.
Terminal gửi torque giữ trước khi switch, plugin bàn giao torque trong
1 s, sau đó chờ HOME ổn định liên tục 0.3 s trước chuyển động. Giới hạn
vận tốc 0.05 rad/s, sai lệch HOME 0.02 rad, thời gian chờ 6 s. HOME gốc
được giữ nguyên. `--prestart-timeout 8` đổi thời gian chờ.

## Chỉnh trọng số

`show` hiển thị trọng số thực trong Q/R/Rd. Với `matlab_squared`:

```text
QE = (q_position / output_scale_position)²
QI = (q_integral / output_scale_integral)²
QV = (q_velocity / output_scale_velocity)²
R  = torque_penalty_scale × (r_input / input_scale)²
RD = torque_penalty_scale × (r_rate / input_scale)²
```

`set QE QI QV R [RD]` chỉnh riêng khớp được chọn, tự đổi về các tham số YAML
tương đương. Bỏ RD để giữ nguyên. Có các lệnh riêng `qe`, `qi`, `qv`, `r`,
`rd`, `slew`. QE/QI/QV/R phải dương, RD không âm. `alpha` chỉ chỉnh ở base.
Mỗi lần đổi một tham số rồi giữ cùng biên độ/thời gian/HOME để so sánh.
Đây là các trọng số tối ưu, không phải gain Kp/Ki/Kd.

RD phạt thay đổi feedback torque; slew giới hạn cứng mức đổi **feedback**
torque giữa các bước QP. Feedforward thay đổi theo tham chiếu, vì vậy slew
MPC này không trực tiếp kẹp đạo hàm của toàn bộ tau_ff + tau_fb.

`np N` và `nc N` đổi prediction/control horizon chung cho ba khớp,
không phải riêng khớp đang chọn. Mặc định Np=40 (0.4 s), Nc=10 (0.1 s)
ở 100 Hz. Cần 1 ≤ Nc ≤ Np ≤ 200. Giữ mặc định trong các lượt đầu.
Đổi horizon có thể tăng thời gian giải QP; log có solve_time/solver_iterations.

Chỉ `save` ghi YAML. Các lần `test` dùng bộ đang chỉnh mà chưa cần save.
Sau khi elbow đạt yêu cầu:

```text
save
next
```

Dừng launch cũ sau khi position active; đặt/đỡ zero và launch lại với
`torque_joint:=shoulder`, chạy HOME rồi test. Lặp lại cho base.
Mở lại một khớp bằng `--joint shoulder` hoặc `--joint base`.

## Log và dừng bài thử

`abort_on_tracking_error` mặc định `false` cho PID/LQR/MPC/SMC theo yêu cầu
thử nghiệm. Profile `mpc_joint_tuning.yaml` cũng đặt `false`: APPROACH, TRACK, RETURN và
HOLD không abort chỉ vì vượt `max_transition_error_rad`/`max_track_error_rad`.
Các giới hạn torque/góc khớp, kiểm tra state cũ và điều kiện trước start vẫn
hoạt động. Node đang chạy có thể bật lại kiểm tra bằng:

```bash
ros2 param set /mpc_controller abort_on_tracking_error true
```

Đặt lại `abort_on_tracking_error: true` trong YAML để khôi phục cho các lần
khởi động tiếp theo. Sau khi cập nhật source, build lại `gim_control`, mở lại
node hoặc tuner để dùng công tắc mới.

Khi khớp đứng rồi bật đi dù torque lệnh tăng, có thể ghi thêm torque do
driver báo ở một terminal khác, trước khi nhập `test`:

```bash
python3 tools/log_motor_torque.py --can can0 --joint shoulder \
  --duration 60 --output results/shoulder_motor_torque.csv
```

Script chỉ gửi yêu cầu đọc `Get_Torques` bằng CAN RTR, mặc định 20 Hz;
không đổi mode hay gửi torque điều khiển. CSV có timestamp `t_wall` để
đối chiếu với `tracking.csv`, cả setpoint và torque đo do driver báo.
Quy đổi sang Nm phía khớp dùng `torque_gear_ratio` và `torque_sign` trong
URDF (shoulder ×8), không dùng tỉ số encoder tổng 64. Driver torque đo
là telemetry của driver, không phải cảm biến torque độc lập.
Đổi `--joint base` để đo base. Ctrl+C hoặc hết `--duration` sẽ đóng CSV;
script báo lỗi nếu không nhận được mẫu nào. Script Python này không cần build.

Mỗi lượt lưu `results/mpc_joint_trials/mpc_<joint>_<timestamp>/`, gồm
`tracking.csv`, `parameters.yaml`, `trial.json`, `model.urdf` và ba ảnh PNG.
CSV có tau_ff, tau_fb/tau_feedback, integral, trạng thái QP, vi phạm ràng
buộc, thời gian giải và saturation. MPC không chia feedback thành gain
P/I/D cố định; chart chỉ hiện feedforward và feedback.

Kết thúc, abort hoặc Ctrl-C: terminal xác nhận position active trước khi
đóng publisher torque, rồi lưu log và vẽ hình. `--no-show` chỉ lưu hình.
Khi chưa xác nhận switch về position, publisher được giữ chạy trong lúc
thử lại. Đóng đồ thị để quay lại prompt.

Thử lệnh mà không kết nối ROS/CAN hoặc ghi YAML:

```bash
bash tools/tune_mpc_terminal.sh --joint elbow --dry-run
```

Sau khi đổi branch hoặc sửa source, build:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  gim_arm_description gim_arm_hardware gim_control gim_arm_controller_mpc
source install/setup.bash
```
