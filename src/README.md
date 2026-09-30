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
