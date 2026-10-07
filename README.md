# GIM Arm 3 DOF — lệnh mô phỏng Gazebo

## Chuẩn bị

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-pinocchio \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  python3-numpy \
  python3-scipy \
  python3-matplotlib
```

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  gim_arm_description \
  gim_control \
  gim_arm_controller_pid \
  gim_arm_controller_lqr \
  gim_arm_controller_mpc \
  gim_arm_controller_smc

source install/setup.bash
```

#### Output mong muốn

```text
Summary: 6 packages finished
```

## Xem trước quỹ đạo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 tools/preview_trajectories.py
```

#### Output mong muốn

```text
results/trajectory_shapes/reference_paths.png
results/trajectory_shapes/reference_paths.svg
results/trajectory_shapes/circle_reference.csv
results/trajectory_shapes/r_reference.csv
results/trajectory_shapes/a_reference.csv
```

## 1. PID

### Terminal 1 — mở Gazebo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

# payload_mass_kg: 0.0 (không tải) hoặc 0.5 (tải 0,5 kg)
ros2 launch gim_control gazebo_effort_control.launch.py \
  gui:=true \
  verbose:=false \
  payload_mass_kg:=0.0
```

#### Output mong muốn

```text
Gazebo: robot tại HOME [0.0, 0.0, 0.0].
forward_position_controller: active
gim_arm_effort_controller: inactive
```

### Terminal 2 — mở bộ điều khiển PID

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p results

# trajectory_shape: circle (vòng tròn), r (chữ R), a (chữ A)
# tau_scale: 0.50 khi không tải; 0.75 khi tải 0,5 kg
ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=pid \
  pid_profile:=gazebo_smooth \
  trajectory_shape:=circle \
  tau_scale:=0.50 \
  autostart:=false \
  use_sim_time:=false \
  log_file:=/home/mirabo/GIM_Arm_3_DOF/results/gim_pid_run.csv
```

#### Output mong muốn

```text
WAIT -> GRAVITY, đã chụp HOME=...
```

### Terminal 3 — bắt đầu bám quỹ đạo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
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

ros2 param set /cascade_pid_controller autostart true
```

#### Output mong muốn — Terminal 2

```text
GRAVITY -> APPROACH
APPROACH -> TRACK
TRACK -> RETURN
RETURN -> HOLD_HOME
```

### Terminal 4 — chuyển về bộ điều khiển vị trí và giữ HOME

```bash
cd /home/mirabo/GIM_Arm_3_DOF
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

#### Output mong muốn

```text
forward_position_controller: active
gim_arm_effort_controller: inactive
Robot giữ HOME [0.0, 0.0, 0.0].
```

### Terminal 2 — dừng controller và ghi CSV

```text
Ctrl+C
```

#### Output mong muốn

```text
Ghi ... dòng -> /home/mirabo/GIM_Arm_3_DOF/results/gim_pid_run.csv
Node điều khiển đã dừng.
```

### Terminal 4 — vẽ kết quả

```bash
ros2 run gim_control plot_joint_tracking \
  /home/mirabo/GIM_Arm_3_DOF/results/gim_pid_run.csv \
  --show \
  --max-plot-points 30000
```

#### Output mong muốn

```text
Metrics từng khớp: RMS e_q [deg], MAX |e_q| [deg], MAX |tau| [Nm].
3 cửa sổ đồ thị; mỗi cửa sổ gồm vị trí, sai số vị trí và mô-men.
results/gim_pid_run_joints_q1_base.png
results/gim_pid_run_joints_q2_shoulder.png
results/gim_pid_run_joints_q3_elbow.png
```

### Terminal 1 — đóng Gazebo

```text
Ctrl+C
```

#### Output mong muốn

```text
Các tiến trình Gazebo đã dừng.
```

## 2. LQR

### Terminal 1 — mở Gazebo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

# payload_mass_kg: 0.0 (không tải) hoặc 0.5 (tải 0,5 kg)
ros2 launch gim_control gazebo_effort_control.launch.py \
  gui:=true \
  verbose:=false \
  payload_mass_kg:=0.0
```

#### Output mong muốn

```text
Gazebo: robot tại HOME [0.0, 0.0, 0.0].
forward_position_controller: active
gim_arm_effort_controller: inactive
```

### Terminal 2 — mở bộ điều khiển LQR

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p results

# trajectory_shape: circle (vòng tròn), r (chữ R), a (chữ A)
# tau_scale: 0.50 khi không tải; 0.75 khi tải 0,5 kg
ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=lqr \
  lqr_profile:=gazebo_matlab \
  trajectory_shape:=circle \
  tau_scale:=0.50 \
  autostart:=false \
  use_sim_time:=false \
  log_file:=/home/mirabo/GIM_Arm_3_DOF/results/gim_lqr_run.csv
```

#### Output mong muốn

```text
Precompute xong ... gain LQR ...
WAIT -> GRAVITY, đã chụp HOME=...
```

### Terminal 3 — bắt đầu bám quỹ đạo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
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
```

#### Output mong muốn — Terminal 2

```text
GRAVITY -> APPROACH
APPROACH -> TRACK
TRACK -> RETURN
RETURN -> HOLD_HOME
```

### Terminal 4 — chuyển về bộ điều khiển vị trí và giữ HOME

```bash
cd /home/mirabo/GIM_Arm_3_DOF
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

#### Output mong muốn

```text
forward_position_controller: active
gim_arm_effort_controller: inactive
Robot giữ HOME [0.0, 0.0, 0.0].
```

### Terminal 2 — dừng controller và ghi CSV

```text
Ctrl+C
```

#### Output mong muốn

```text
Ghi ... dòng -> /home/mirabo/GIM_Arm_3_DOF/results/gim_lqr_run.csv
Node điều khiển đã dừng.
```

### Terminal 4 — vẽ kết quả

```bash
ros2 run gim_control plot_joint_tracking \
  /home/mirabo/GIM_Arm_3_DOF/results/gim_lqr_run.csv \
  --show \
  --max-plot-points 30000
```

#### Output mong muốn

```text
Metrics từng khớp: RMS e_q [deg], MAX |e_q| [deg], MAX |tau| [Nm].
3 cửa sổ đồ thị; mỗi cửa sổ gồm vị trí, sai số vị trí và mô-men.
results/gim_lqr_run_joints_q1_base.png
results/gim_lqr_run_joints_q2_shoulder.png
results/gim_lqr_run_joints_q3_elbow.png
```

### Terminal 1 — đóng Gazebo

```text
Ctrl+C
```

#### Output mong muốn

```text
Các tiến trình Gazebo đã dừng.
```

## 3. MPC

### Terminal 1 — mở Gazebo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

# payload_mass_kg: 0.0 (không tải) hoặc 0.5 (tải 0,5 kg)
ros2 launch gim_control gazebo_effort_control.launch.py \
  gui:=true \
  verbose:=false \
  payload_mass_kg:=0.0
```

#### Output mong muốn

```text
Gazebo: robot tại HOME [0.0, 0.0, 0.0].
forward_position_controller: active
gim_arm_effort_controller: inactive
```

### Terminal 2 — mở bộ điều khiển MPC

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p results

# trajectory_shape: circle (vòng tròn), r (chữ R), a (chữ A)
# tau_scale: 1.0 khi không tải; 1.0 khi tải 0,5 kg
ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=mpc \
  mpc_profile:=gazebo_position \
  trajectory_shape:=circle \
  mpc_torque_penalty_scale:=10000.0 \
  tau_scale:=1.0 \
  autostart:=false \
  use_sim_time:=false \
  log_file:=/home/mirabo/GIM_Arm_3_DOF/results/gim_mpc_run.csv
```

#### Output mong muốn

```text
Precompute xong 3701 mẫu tham chiếu MPC ...
WAIT -> GRAVITY, đã chụp HOME=...
```

### Terminal 3 — bắt đầu bám quỹ đạo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
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

ros2 param set /mpc_controller autostart true
```

#### Output mong muốn — Terminal 2

```text
GRAVITY -> APPROACH
APPROACH -> TRACK
TRACK -> RETURN
RETURN -> HOLD_HOME
```

### Terminal 4 — chuyển về bộ điều khiển vị trí và giữ HOME

```bash
cd /home/mirabo/GIM_Arm_3_DOF
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

#### Output mong muốn

```text
forward_position_controller: active
gim_arm_effort_controller: inactive
Robot giữ HOME [0.0, 0.0, 0.0].
```

### Terminal 2 — dừng controller và ghi CSV

```text
Ctrl+C
```

#### Output mong muốn

```text
Ghi ... dòng -> /home/mirabo/GIM_Arm_3_DOF/results/gim_mpc_run.csv
Node điều khiển đã dừng.
```

### Terminal 4 — vẽ kết quả

```bash
ros2 run gim_control plot_joint_tracking \
  /home/mirabo/GIM_Arm_3_DOF/results/gim_mpc_run.csv \
  --show \
  --max-plot-points 30000
```

#### Output mong muốn

```text
Metrics từng khớp: RMS e_q [deg], MAX |e_q| [deg], MAX |tau| [Nm].
3 cửa sổ đồ thị; mỗi cửa sổ gồm vị trí, sai số vị trí và mô-men.
results/gim_mpc_run_joints_q1_base.png
results/gim_mpc_run_joints_q2_shoulder.png
results/gim_mpc_run_joints_q3_elbow.png
```

### Terminal 1 — đóng Gazebo

```text
Ctrl+C
```

#### Output mong muốn

```text
Các tiến trình Gazebo đã dừng.
```

## 4. SMC

### Terminal 1 — mở Gazebo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

# payload_mass_kg: 0.0 (không tải) hoặc 0.5 (tải 0,5 kg)
ros2 launch gim_control gazebo_effort_control.launch.py \
  gui:=true \
  verbose:=false \
  payload_mass_kg:=0.0
```

#### Output mong muốn

```text
Gazebo: robot tại HOME [0.0, 0.0, 0.0].
forward_position_controller: active
gim_arm_effort_controller: inactive
```

### Terminal 2 — mở bộ điều khiển SMC

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p results

# trajectory_shape: circle (vòng tròn), r (chữ R), a (chữ A)
# tau_scale: 0.50 khi không tải; 0.75 khi tải 0,5 kg
ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=smc \
  smc_profile:=gazebo_safe \
  trajectory_shape:=circle \
  tau_scale:=0.50 \
  autostart:=false \
  use_sim_time:=false \
  log_file:=/home/mirabo/GIM_Arm_3_DOF/results/gim_smc_run.csv
```

#### Output mong muốn

```text
WAIT -> GRAVITY, đã chụp HOME=...
```

### Terminal 3 — bắt đầu bám quỹ đạo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
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

ros2 param set /smc_controller autostart true
```

#### Output mong muốn — Terminal 2

```text
GRAVITY -> APPROACH
APPROACH -> TRACK
TRACK -> RETURN
RETURN -> HOLD_HOME
```

### Terminal 4 — chuyển về bộ điều khiển vị trí và giữ HOME

```bash
cd /home/mirabo/GIM_Arm_3_DOF
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

#### Output mong muốn

```text
forward_position_controller: active
gim_arm_effort_controller: inactive
Robot giữ HOME [0.0, 0.0, 0.0].
```

### Terminal 2 — dừng controller và ghi CSV

```text
Ctrl+C
```

#### Output mong muốn

```text
Ghi ... dòng -> /home/mirabo/GIM_Arm_3_DOF/results/gim_smc_run.csv
Node điều khiển đã dừng.
```

### Terminal 4 — vẽ kết quả

```bash
ros2 run gim_control plot_joint_tracking \
  /home/mirabo/GIM_Arm_3_DOF/results/gim_smc_run.csv \
  --show \
  --max-plot-points 30000
```

#### Output mong muốn

```text
Metrics từng khớp: RMS e_q [deg], MAX |e_q| [deg], MAX |tau| [Nm].
3 cửa sổ đồ thị; mỗi cửa sổ gồm vị trí, sai số vị trí và mô-men.
results/gim_smc_run_joints_q1_base.png
results/gim_smc_run_joints_q2_shoulder.png
results/gim_smc_run_joints_q3_elbow.png
```

### Terminal 1 — đóng Gazebo

```text
Ctrl+C
```

#### Output mong muốn

```text
Các tiến trình Gazebo đã dừng.
```

## Kiểm tra tự động — PID

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 tools/validate_pid_gazebo.py --trajectory-shape circle
python3 tools/validate_pid_gazebo.py --trajectory-shape r
python3 tools/validate_pid_gazebo.py --trajectory-shape a
```

#### Output mong muốn

```text
Không tải và tải 0,5 kg: final_phase=HOLD, passed=true.
results/pid_inverse_dynamics_gazebo/
results/pid_letter_r_gazebo/
results/pid_letter_a_gazebo/
Mỗi thư mục: CSV, PNG từng khớp, log, validation.json.
```

## Kiểm tra tự động — LQR

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/validate_lqr_gazebo.py
```

#### Output mong muốn

```text
Không tải và tải 0,5 kg: final_phase=HOLD, passed=true.
results/lqr_matlab_gazebo/: CSV, PNG từng khớp, log, validation.json.
```

## Kiểm tra tự động — MPC

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/validate_mpc_gazebo.py
```

#### Output mong muốn

```text
Không tải và tải 0,5 kg: final_phase=HOLD, passed=true.
results/mpc_position_gazebo/: CSV, PNG từng khớp, log, validation.json.
```

## Lỗi Package 'gim_control' not found

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix gim_control
```

#### Output mong muốn

```text
/home/mirabo/GIM_Arm_3_DOF/install/gim_control
```
