## PID một vòng bám vị trí

Luật điều khiển trên nhánh `pid-test` lấy từ `PositionPidController` của
nhánh tham khảo `sim_gazebo2`:

```text
e = q_ref - q
de/dt = qd_ref - qd
tau_fb = Kp*e + Ki*integral(e) + Kd*de/dt
tau_cmd = clip(inverse_dynamics(q_ref, qd_ref, qdd_ref) + tau_fb)
```

Không có vòng PI vận tốc bên trong. Vận tốc dùng để tính đạo hàm sai số
vị trí và feedforward nghịch động lực học. Các thành phần PID xuất Nm
trực tiếp; tích phân sai số vị trí có đơn vị rad*s và có anti-windup.

Gain YAML mới: `position_kp` [Nm/rad], `position_ki` [Nm/(rad*s)],
`position_kd` [Nm/(rad/s)], `position_integral_limit` [rad*s]. Gain hiện tại
là baseline, chưa tune lại từng khớp. YAML cũ dùng `kpp/kvp/kvi` cần cập nhật;
không thể chép nguyên gain tích phân vận tốc thành gain tích phân vị trí.

Các launch thử từng khớp và vòng quét vẫn dùng cùng PID vị trí trong các
pha đi, giữ đích, về HOME và giữ HOME. `cascade_hold` và tên node
`/cascade_pid_controller` chỉ được giữ để tương thích các lệnh cũ.
CSV bổ sung `tau_d`, `error_rate`, `kp/ki/kd`; đồ thị PID gồm góc,
sai số góc và mô-men với các thành phần feedforward/P/I/D.
Hướng dẫn từng khớp: [gim_arm_controller_pid/README.md](gim_arm_controller_pid/README.md).

<!-- PID -->
T1:
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gim_control gazebo_effort_control.launch.py \
  gui:=true \
  verbose:=false \
  payload_mass_kg:=0.5
T2:
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p results

ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=pid \
  pid_profile:=gazebo_smooth \
  autostart:=false \
  use_sim_time:=false \
  log_file:=/home/minh/git_gim_ws/GIM_Arm_3_DOF/results/gim_pid_run.csv
T3:
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service call /gim_arm/set_hand_guiding \
  std_srvs/srv/SetBool "{data: false}"

ros2 control switch_controllers \
  --strict \
  --activate-asap \
  --deactivate forward_position_controller \
  --activate gim_arm_effort_controller

sleep 5

ros2 topic echo /joint_states --once

ros2 param set /cascade_pid_controller autostart true
T4:
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control switch_controllers \
  --deactivate gim_arm_effort_controller \
  --activate forward_position_controller

ros2 topic pub --once \
  /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  '{data: [0.0, 0.0, 0.0]}'

sleep 2

pkill -INT -f '/install/gim_arm_controller_pid/lib/gim_arm_controller_pid/cascade_pid_node'

sleep 3

ros2 run gim_control plot_joint_tracking \
  /home/minh/git_gim_ws/GIM_Arm_3_DOF/results/gim_pid_run.csv \
  --show \
  --max-plot-points 30000

<!-- LQR -->
T1:
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gim_control gazebo_effort_control.launch.py \
  gui:=true \
  verbose:=false \
  payload_mass_kg:=0.5
T2:
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p results

ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=lqr \
  lqr_profile:=safe_100hz \
  autostart:=false \
  use_sim_time:=false \
  log_file:=/home/minh/git_gim_ws/GIM_Arm_3_DOF/results/gim_lqr_run.csv
T3:
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service call /gim_arm/set_hand_guiding \
  std_srvs/srv/SetBool "{data: false}"

ros2 control switch_controllers \
  --strict \
  --activate-asap \
  --deactivate forward_position_controller \
  --activate gim_arm_effort_controller

sleep 5

ros2 param set /lqr_controller autostart true
T4:
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

pkill -INT -f '/install/gim_arm_controller_lqr/lib/gim_arm_controller_lqr/lqr_node'

sleep 3

ros2 run gim_control plot_joint_tracking \
  /home/minh/git_gim_ws/GIM_Arm_3_DOF/results/gim_lqr_run.csv \
  --show \
  --max-plot-points 30000

<!-- SMC -->
## Chạy SMC trên Gazebo

Profile `gazebo_safe` là bộ tham số SMC đã được kiểm tra ở 100 Hz. Không dùng
`smc_profile:=matlab_reference` để điều khiển Gazebo/robot ở 100 Hz vì bộ gain
liên tục gốc không ổn định sau khi rời rạc hóa ở chu kỳ này.

### Terminal 1 - chạy mô phỏng

Không tải:

```bash
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gim_control gazebo_effort_control.launch.py \
  gui:=true \
  verbose:=false \
  payload_mass_kg:=0.0
```

Để thử tải 0,5 kg tại tool tip, chỉ đổi thành `payload_mass_kg:=0.5`.

### Terminal 2 - khởi động SMC nhưng chưa cho chạy quỹ đạo

```bash
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p results

ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=smc \
  smc_profile:=gazebo_safe \
  autostart:=false \
  use_sim_time:=false \
  log_file:=/home/minh/git_gim_ws/GIM_Arm_3_DOF/results/gim_smc_run.csv
```

Nếu Terminal 1 đang dùng payload 0,5 kg, thêm `tau_scale:=0.75` vào lệnh trên.
Không tải dùng mặc định `tau_scale:=0.50`.

### Terminal 3 - chuyển sang effort rồi mới bật autostart

Chờ Terminal 2 báo `WAIT -> GRAVITY`, sau đó chạy:

```bash
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service call /gim_arm/set_hand_guiding \
  std_srvs/srv/SetBool "{data: false}"

ros2 control switch_controllers \
  --strict \
  --activate-asap \
  --deactivate forward_position_controller \
  --activate gim_arm_effort_controller

sleep 5

ros2 topic echo /joint_states --once

ros2 param set /smc_controller autostart true
```

### Terminal 4 - dừng an toàn và vẽ kết quả

Chờ SMC chạy hết và báo `HOLD`, sau đó:

```bash
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control switch_controllers \
  --strict \
  --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate forward_position_controller

ros2 topic pub --once \
  /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  '{data: [0.0, 0.0, 0.0]}'
```

Nhấn `Ctrl+C` ở Terminal 2 để đóng node SMC và ghi xong file CSV, rồi vẽ:

```bash
ros2 run gim_control plot_joint_tracking \
  /home/minh/git_gim_ws/GIM_Arm_3_DOF/results/gim_smc_run.csv \
  --show \
  --max-plot-points 30000
```
