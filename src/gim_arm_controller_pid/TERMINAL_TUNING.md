# Terminal tune PID một vòng: khâu 3 → 2 → 1

Terminal hiện có `src/gim_arm_hardware/scripts/tune_motor_gains.py` mặc định
chỉnh PID vị trí **trên PC**. Backend là `tune_position_pid.py`.
Lượt thử dùng cùng `PositionPidController` với các launch PID của nhánh `pid-test`:

```text
e = q_ref - q
tau_ff = M(q_ref)*qdd_ref + C(q_ref, qd_ref)*qd_ref + G(q_ref)
tau_fb = Kp*e + Ki*integral(e) + Kd*(qd_ref - qd)
tau_cmd = clip(tau_ff + tau_fb, -tau_limit, tau_limit)
```

`tau_ff` tính bằng inverse dynamics từ URDF đang chạy, đọc qua
parameter `robot_description` của `/controller_manager`, đúng cấu hình được
nạp vào phần cứng. Mỗi lượt lưu lại URDF đó trong
`model.urdf`. Vận tốc chỉ phục vụ D và feedforward; không có vòng PI vận tốc.

Chỉ khớp đang tune nhận lệnh mô-men. Hai khớp còn lại được driver giữ bằng
position mode tại góc chụp lúc switch sang effort. Đây là giữ bằng motor.
Terminal kiểm `torque_mode_enable` của cả ba khớp trước khi chạy thử.

| Bước | Khớp được tune | `torque_joint` | Hai khớp giữ vị trí |
| --- | --- | --- | --- |
| 1 | Khâu 3 — elbow, CAN ID 2 | `elbow` | base và shoulder |
| 2 | Khâu 2 — shoulder, CAN ID 1 | `shoulder` | base và elbow |
| 3 | Khâu 1 — base, CAN ID 0 | `base` | shoulder và elbow |

## Build và mở các terminal

Build một lần trước khi chạy; các terminal sau đều cần hai lệnh `source`:

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  gim_arm_description gim_control gim_arm_controller_pid gim_arm_hardware
source install/setup.bash
```

## Bước 1 — tune khâu 3

Terminal 1: đặt tay tại đúng tư thế zero rồi khởi động phần cứng cho elbow.
Giữ nguyên interface CAN đã cấu hình trên máy.

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 set_zero_on_startup:=true zero_method:=software \
  torque_joint:=elbow
```

Terminal 2: chờ `gim_arm_group_controller` active và đưa tay về HOME.
Chờ goal hoàn tất trước khi gõ `test` ở terminal tune.

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control list_controllers
ros2 action send_goal --feedback \
  /gim_arm_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [base_joint, shoulder_joint, elbow_joint], points: [{positions: [0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0], time_from_start: {sec: 6}}]}}"
```

Terminal 3: mở terminal PID; không chạy thêm node PID khác song song.
File `.sh` tự nạp `/opt/ros/humble/setup.bash` và `install/setup.bash`,
không cần nhập hai lệnh `source` ở terminal này:

```bash
cd /home/mirabo/GIM_Arm_3_DOF
bash tools/tune_pid_terminal.sh --joint elbow --interface can0
```

Mở file chạy bằng gedit: `gedit tools/tune_pid_terminal.sh`.

Trong terminal tune:

```text
show
test 5 6 3
```

`test 5 6 3` đi **thêm 5 độ từ góc đo hiện tại**, đi trong 6 s, giữ 3 s,
về điểm đầu trong 6 s và giữ thêm 3 s. Hai reference còn lại giữ nguyên.
Terminal tự dựng PID, switch sang effort, chạy thử, switch về position,
ghi CSV và mở đồ thị. Đóng cửa sổ đồ thị để tiếp tục nhập lệnh.
Ctrl-C trong lượt thử cũng thực hiện bước trả về position.

Lệnh chỉnh gain chỉ thay giá trị của khớp đang chọn trong bộ nhớ terminal:

```text
kp GIÁ_TRỊ_KP
ki GIÁ_TRỊ_KI
kd GIÁ_TRỊ_KD
set KP KI KD
test 5 6 3
save
```

Thay các tên viết hoa bằng số. `set` có thứ tự **Kp Ki Kd**.
Có thể đặt Ki bằng 0 để tune P và D trước, rồi thêm I để giảm sai số tĩnh.
Gain mới được áp dụng khi bắt đầu lượt `test` tiếp theo, không đổi giữa lượt.
`save` ghi YAML trên PC, giữ gain của hai khớp khác; không ghi flash motor.
Mở terminal hoặc gõ `show/set/next` đều không làm tay chuyển động.

## Bước 2 — tune khâu 2

Sau khi lượt elbow đã trả về position và chọn được gain, gõ trong terminal 3:

```text
save
next
```

Terminal chuyển sang shoulder. Đổi khớp cần khởi động lại launch phần cứng
vì `torque_joint` được đọc khi khởi tạo. Ở terminal 1, dừng launch cũ,
đưa tay về đúng tư thế zero và chạy:

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 set_zero_on_startup:=true zero_method:=software \
  torque_joint:=shoulder
```

Ở terminal 2, chờ JTC active rồi gửi lại goal HOME như bước 1.
Terminal 3 vẫn giữ toàn bộ gain đã chỉnh; bắt đầu các lượt shoulder:

```text
show
test 5 6 3
```

Lúc này chỉ shoulder nhận `tau_ff + tau_fb`; base và elbow giữ position.
Giữa các lần chỉnh gain **cùng shoulder**, giữ launch phần cứng chạy và
lặp `kp/ki/kd`, `test`, `save`; không cần khởi động lại phần cứng.

## Bước 3 — tune khâu 1

Sau khi chọn được gain shoulder:

```text
save
next
```

Dừng launch phần cứng, đưa tay về đúng zero rồi khởi động lại cho base:

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 set_zero_on_startup:=true zero_method:=software \
  torque_joint:=base
```

Chờ JTC active và hoàn tất goal HOME ở terminal 2. Trong terminal 3:

```text
show
test 5 6 3
```

Chỉ base nhận `tau_ff + tau_fb`; shoulder và elbow giữ position.
Sau khi chọn xong gain:

```text
save
quit
```

Nếu muốn mở lại từ một khớp đã tune dở, dùng `--joint shoulder` hoặc `--joint base`.

## Config và kết quả

YAML mặc định: `src/gim_arm_controller_pid/config/pid_hardware_soft.yaml`.
Vector có thứ tự **[base, shoulder, elbow]**, dù trình tự tune là 3 → 2 → 1:

```yaml
position_kp: [20.0, 24.5, 7.0]
position_ki: [2.0, 1.5, 4.0]
position_kd: [0.8, 0.7, 0.7]
position_integral_limit: [0.3, 0.3, 0.3]
```

Đây là baseline hiện có, chưa phải bộ gain đã tune.
Mỗi lượt tạo thư mục riêng trong `results/position_pid_trials/`, chứa:

- `tracking.csv`: góc thực tế/tham chiếu và `tau_ff`, `tau_fb`, P/I/D, saturation.
- `model.urdf`: URDF dùng để tính feedforward của đúng lượt đó.
- `parameters.yaml` và `trial.json`: gain, HOME, góc đích và thời gian.
- Ba PNG: vị trí, sai số vị trí và mô-men của từng khớp; xem cả hai khớp cố định.

`--no-show` chỉ lưu hình. `--dry-run` thử các lệnh mà không mở ROS/CAN hoặc ghi YAML:

```bash
python3 src/gim_arm_hardware/scripts/tune_motor_gains.py \
  --params-file src/gim_arm_controller_pid/config/pid_hardware_soft.yaml --dry-run
```

Sau build cũng có thể dùng `ros2 run gim_arm_hardware tune_motor_gains.py`.
Terminal driver cũ chỉ được chọn rõ bằng `--driver-gains`.
