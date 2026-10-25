# MPC + ESO — branch `mpc-eso-test`

Branch này chỉ chứa package thuật toán `src/gim_arm_controller_mpc`.
Các thuật toán còn lại nằm trong branch tương ứng; không cần build chúng.

Giữ chung `gim_control` (runner ROS, động lực học URDF, quỹ đạo, plotting),
`gim_arm_hardware`, `gim_arm_description` và các package mô phỏng/MoveIt của robot.
Các file trong `results_new` và bộ tham số đã tune được giữ nguyên.

## Build

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gim_control gim_arm_controller_mpc
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
```

## Chạy vòng tròn

Chỉ giữ một launch phần cứng với `torque_joint:=all` và một node thuật toán.
Khi position controller đang giữ HOME, chạy:

```bash
ros2 launch gim_control mpc_sweep_hardware.launch.py \
  trajectory_shape:=circle approach_time:=16 return_time:=16 \
  params_file:=src/gim_arm_controller_mpc/config/mpc_eso_hardware.yaml \
  log_file:=results_new/mpc/eso/circle_run01.csv
```

Sau READY, chuyển controller thành công rồi bắt đầu:

```bash
ros2 control switch_controllers --strict --activate-asap \
  --deactivate gim_arm_group_controller --activate gim_arm_effort_controller
ros2 param set /mpc_controller autostart true
```

Kết thúc: chuyển về position trước khi Ctrl+C node thuật toán để đóng log.

```bash
ros2 control switch_controllers --strict --activate-asap \
  --deactivate gim_arm_effort_controller --activate gim_arm_group_controller
```

Build/install/log là dữ liệu sinh trên máy, không được dùng để xác định source
của branch. Sau khi đổi branch cần build và source lại như trên; binary của
package cũ có thể vẫn còn trong install nhưng không thuộc source branch này.
