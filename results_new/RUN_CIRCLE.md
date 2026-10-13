# Chạy quỹ đạo tròn trên tay thật: PID → LQR → MPC → SMC

Thư mục kết quả là `results_new/{pid,lqr,mpc,smc}`. Xem `TUNED_PARAMETERS.md` và các bản sao `params_tuned.yaml` để lấy số báo cáo.

Mỗi thuật toán dùng đúng branch riêng. Dừng các node của bài trước rồi mới switch/build. Giữ phiên Terminal 1 (hardware) chạy suốt bài thử, Terminal 2 (thuật toán) để chạy và ghi CSV, Terminal 3 để chuyển mode, bắt đầu và plot.

Khi khởi động hardware với `set_zero_on_startup=true zero_method=software`, đặt và kê tay ở đúng tư thế q=[0,0,0] trước khi chạy: tùy chọn này ghi nhận tư thế hiện tại làm mốc zero. Đây không phải lệnh đưa tay từ vị trí bất kỳ về home.

Kiểm tra can0 trước khi chạy. Nếu đang UP và giao tiếp được với driver, giữ cấu hình hiện tại:

```bash
ip -details link show can0
```

Nếu can0 chưa được cấu hình, README phần cứng của repo dùng 500 kbit/s; tốc độ bus phải khớp với cấu hình driver:

```bash
sudo ip link set can0 up type can bitrate 500000
```

## PID

### Chuẩn bị: switch và build

```bash
cd ~/GIM_Arm_3_DOF
git switch pid-test
source /opt/ros/humble/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
colcon build --symlink-install --packages-select \
  gim_arm_description gim_arm_hardware gim_control gim_arm_controller_pid
source install/setup.bash
```

### Terminal 1: hardware và mốc zero

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 \
  set_zero_on_startup:=true zero_method:=software \
  torque_joint:=all
```

Chờ `gim_arm_group_controller` active, `gim_arm_effort_controller` inactive.

### Terminal 2: quỹ đạo và CSV

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
ros2 launch gim_control pid_sweep_hardware.launch.py \
  trajectory_shape:=circle \
  approach_time:=16 return_time:=16 \
  params_file:=results_new/pid/params_tuned.yaml \
  log_file:=results_new/pid/circle_run01.csv
```

Chờ node chuẩn bị xong và về READY; tay ở home và đứng yên.

### Terminal 3: bắt đầu

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_group_controller \
  --activate gim_arm_effort_controller
ros2 param set /cascade_pid_controller autostart true
```

Theo dõi Terminal 2: `GRAVITY → APPROACH → TRACK → RETURN → HOLD`. Khi đã về HOLD_HOME:

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate gim_arm_group_controller
```

Sau khi switch thành công, Ctrl+C **Terminal 2** để node ghi/đóng CSV. Terminal 3 xuất hình:

```bash
ros2 run gim_control plot_joint_tracking \
  results_new/pid/circle_run01.csv --show
```

CSV và 6 hình PNG cùng trong `results_new/pid`. Lần tiếp theo đổi `run01` thành `run02` để giữ kết quả cũ. Đóng đồ thị, kết thúc Terminal 1 sau bài thử rồi mới chuyển branch tiếp theo.

## LQR

### Chuẩn bị: switch và build

```bash
cd ~/GIM_Arm_3_DOF
git switch lqr-test
source /opt/ros/humble/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
colcon build --symlink-install --packages-select \
  gim_arm_description gim_arm_hardware gim_control gim_arm_controller_lqr
source install/setup.bash
```

### Terminal 1: hardware và mốc zero

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 \
  set_zero_on_startup:=true zero_method:=software \
  torque_joint:=all
```

Chờ `gim_arm_group_controller` active, `gim_arm_effort_controller` inactive.

### Terminal 2: quỹ đạo và CSV

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
ros2 launch gim_control lqr_sweep_hardware.launch.py \
  trajectory_shape:=circle \
  approach_time:=16 return_time:=16 \
  params_file:=results_new/lqr/params_tuned.yaml \
  log_file:=results_new/lqr/circle_run01.csv
```

Chờ node chuẩn bị xong và về READY; tay ở home và đứng yên.

### Terminal 3: bắt đầu

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_group_controller \
  --activate gim_arm_effort_controller
ros2 param set /lqr_controller autostart true
```

Theo dõi Terminal 2: `GRAVITY → APPROACH → TRACK → RETURN → HOLD`. Khi đã về HOLD_HOME:

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate gim_arm_group_controller
```

Sau khi switch thành công, Ctrl+C **Terminal 2** để node ghi/đóng CSV. Terminal 3 xuất hình:

```bash
ros2 run gim_control plot_joint_tracking \
  results_new/lqr/circle_run01.csv --show
```

CSV và 6 hình PNG cùng trong `results_new/lqr`. Lần tiếp theo đổi `run01` thành `run02` để giữ kết quả cũ. Đóng đồ thị, kết thúc Terminal 1 sau bài thử rồi mới chuyển branch tiếp theo.

## MPC

### Chuẩn bị: switch và build

```bash
cd ~/GIM_Arm_3_DOF
git switch mpc-test
source /opt/ros/humble/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
colcon build --symlink-install --packages-select \
  gim_arm_description gim_arm_hardware gim_control gim_arm_controller_mpc
source install/setup.bash
```

### Terminal 1: hardware và mốc zero

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 \
  set_zero_on_startup:=true zero_method:=software \
  torque_joint:=all
```

Chờ `gim_arm_group_controller` active, `gim_arm_effort_controller` inactive.

### Terminal 2: quỹ đạo và CSV

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
ros2 launch gim_control mpc_sweep_hardware.launch.py \
  trajectory_shape:=circle \
  approach_time:=16 return_time:=16 \
  params_file:=results_new/mpc/params_tuned.yaml \
  log_file:=results_new/mpc/circle_run01.csv
```

Chờ node chuẩn bị xong và về READY; tay ở home và đứng yên.

### Terminal 3: bắt đầu

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_group_controller \
  --activate gim_arm_effort_controller
ros2 param set /mpc_controller autostart true
```

Theo dõi Terminal 2: `GRAVITY → APPROACH → TRACK → RETURN → HOLD`. Khi đã về HOLD_HOME:

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate gim_arm_group_controller
```

Sau khi switch thành công, Ctrl+C **Terminal 2** để node ghi/đóng CSV. Terminal 3 xuất hình:

```bash
ros2 run gim_control plot_joint_tracking \
  results_new/mpc/circle_run01.csv --show
```

CSV và 6 hình PNG cùng trong `results_new/mpc`. Lần tiếp theo đổi `run01` thành `run02` để giữ kết quả cũ. Đóng đồ thị, kết thúc Terminal 1 sau bài thử rồi mới chuyển branch tiếp theo.

## SMC

### Chuẩn bị: switch và build

```bash
cd ~/GIM_Arm_3_DOF
git switch smc-test
source /opt/ros/humble/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
colcon build --symlink-install --packages-select \
  gim_arm_description gim_arm_hardware gim_control gim_arm_controller_smc
source install/setup.bash
```

### Terminal 1: hardware và mốc zero

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 \
  set_zero_on_startup:=true zero_method:=software \
  torque_joint:=all
```

Chờ `gim_arm_group_controller` active, `gim_arm_effort_controller` inactive.

### Terminal 2: quỹ đạo và CSV

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
ros2 launch gim_control smc_sweep_hardware.launch.py \
  trajectory_shape:=circle \
  approach_time:=16 return_time:=16 \
  params_file:=results_new/smc/params_tuned.yaml \
  log_file:=results_new/smc/circle_run01.csv
```

Chờ node chuẩn bị xong và về READY; tay ở home và đứng yên.

### Terminal 3: bắt đầu

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_group_controller \
  --activate gim_arm_effort_controller
ros2 param set /smc_controller autostart true
```

Theo dõi Terminal 2: `GRAVITY → APPROACH → TRACK → RETURN → HOLD`. Khi đã về HOLD_HOME:

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate gim_arm_group_controller
```

Sau khi switch thành công, Ctrl+C **Terminal 2** để node ghi/đóng CSV. Terminal 3 xuất hình:

```bash
ros2 run gim_control plot_joint_tracking \
  results_new/smc/circle_run01.csv --show
```

CSV và 6 hình PNG cùng trong `results_new/smc`. Lần tiếp theo đổi `run01` thành `run02` để giữ kết quả cũ. Đóng đồ thị, kết thúc Terminal 1 sau bài thử rồi mới chuyển branch tiếp theo.

## Về home khi dừng giữa quỹ đạo

Chuyển sang position trước, rồi gửi goal về zero:

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate gim_arm_group_controller
ros2 action send_goal --feedback \
  /gim_arm_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [base_joint, shoulder_joint, elbow_joint], points: [{positions: [0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0], time_from_start: {sec: 8}}]}}"
```

Ctrl+C Terminal thuật toán để đóng CSV, rồi plot đường dẫn CSV tương ứng; log của bài dừng giữa chừng vẫn có thể xem.
