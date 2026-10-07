# PID — chạy phần cứng

## Chuẩn bị

```bash
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF_pid_test
git branch --show-current
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gim_arm_description gim_arm_hardware gim_control gim_arm_controller_pid
source install/setup.bash
```

## Terminal 1 — CAN, zero phần mềm, position controller

```bash
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF_pid_test
source /opt/ros/humble/setup.bash
source install/setup.bash
ip -details link show can0
# Đặt và kê đỡ tay ở đúng tư thế zero trước khi chụp mốc phần mềm.
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 set_zero_on_startup:=true zero_method:=software \
  torque_joint:=all
```

### Output mong muốn

```text
joint_state_broadcaster: active
gim_arm_group_controller: active
gim_arm_effort_controller: inactive
```

## Terminal 2 — PID và quỹ đạo

```bash
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF_pid_test
source /opt/ros/humble/setup.bash
source install/setup.bash
# trajectory_shape: circle (vòng tròn), r (chữ R), a (chữ A)
ros2 launch gim_control pid_sweep_hardware.launch.py \
  trajectory_shape:=circle approach_time:=16 return_time:=16 \
  params_file:=src/gim_arm_controller_pid/config/pid_hardware_soft.yaml \
  log_file:=results/pid_circle_run01.csv
```

### Output mong muốn

```text
WAIT -> READY_PID
HOME gần [0, 0, 0]; tay đứng yên.
```

## Terminal 3 — bắt đầu

```bash
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF_pid_test
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_group_controller \
  --activate gim_arm_effort_controller
ros2 param set /cascade_pid_controller autostart true
```

### Output mong muốn — Terminal 2

```text
GRAVITY -> APPROACH (16s)
APPROACH -> TRACK
TRACK -> RETURN (16s)
RETURN -> HOLD_HOME cascade PID
```

## Terminal 3 — kết thúc hoặc dừng bài thử

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate gim_arm_group_controller
```

### Output mong muốn

```text
gim_arm_group_controller: active
gim_arm_effort_controller: inactive
```

## Terminal 2 — ghi CSV

```text
Ctrl+C
```

### Output mong muốn

```text
Ghi ... dòng -> results/pid_circle_run01.csv
```

## Terminal 3 — kết quả vị trí

```bash
ros2 run gim_control plot_joint_tracking results/pid_circle_run01.csv --show
```

### Output mong muốn

```text
RMS e_q [deg], MAX |e_q| [deg], MAX |tau| [Nm].
3 cửa sổ đồ thị: vị trí, sai số vị trí, mô-men (giữ tau_ff/tau_fb khi có).
results/pid_circle_run01_joints_q1_base.png
results/pid_circle_run01_joints_q2_shoulder.png
results/pid_circle_run01_joints_q3_elbow.png
```

## Xem trước ba quỹ đạo

```bash
python3 tools/preview_trajectories.py
```

### Output mong muốn

```text
results/trajectory_shapes/reference_paths.png
results/trajectory_shapes/{circle,r,a}_reference.csv: t,x,y,z,q1..q3,qd1..qd3,qdd1..qdd3.
```
