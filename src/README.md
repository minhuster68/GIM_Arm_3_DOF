<!-- PID -->
T1:
cd ~/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gim_control gazebo_effort_control.launch.py
T2:
cd ~/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=pid control_hz:=2000.0 tau_scale:=0.50 \
  max_transition_error_rad:=0.35 max_track_error_rad:=0.10 \
  approach_time:=5.0 return_time:=5.0 loops:=1.0 \
  command_heartbeat_nm:=0.0 use_sim_time:=false autostart:=false \
  log_file:=/tmp/gim_pid_full.csv
T3:
cd ~/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 control switch_controllers \
  --deactivate forward_position_controller \
  --activate gim_arm_effort_controller
ros2 param set /cascade_pid_controller autostart true
T4:
source /opt/ros/humble/setup.bash
source ~/git_gim_ws/GIM_Arm_3_DOF/install/setup.bash
ros2 run gim_control plot_joint_tracking \
  /tmp/gim_pid_short_segment_20260928.csv \
  --show --max-plot-points 30000
