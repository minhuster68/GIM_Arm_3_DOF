# Quỹ đạo vuông bo góc — sim_gazebo2

Đường đi nhìn từ trước là hình vuông 20 × 20 cm, bo góc 2.5 cm. Chiều sâu
thay đổi theo mặt cầu quanh vai để IK có nghiệm; đây là đường gần vuông
trong không gian, không ép hướng EE. Cả ba khớp đều chuyển động.

Một vòng TRACK dài 27 s. Mỗi đoạn dùng spline IK và time-scaling quintic:

```text
q(t)   = Q(h(t))
qd(t)  = Q'(h(t)) * hd(t)
qdd(t) = Q''(h(t)) * hd(t)^2 + Q'(h(t)) * hdd(t)
```

Vận tốc/gia tốc bằng 0 tại đầu/cuối và các điểm nối đoạn, nên chuyển động
C2, có các lần giảm tốc/dừng êm giữa các đoạn. Không đi qua góc nhọn với
vận tốc khác 0. Giữ nguyên quỹ đạo circle/R/A cũ và điểm đầu/cuối chung.

## Xuất file tham chiếu

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/src/gim_arm_control${PYTHONPATH:+:$PYTHONPATH}"
python3 tools/preview_trajectories.py \
  --shapes square --output-dir results_new/gazebo/square
```

- `square_reference.csv`: 2701 mẫu, dt=0.01 s, có cả hai điểm đầu/cuối.
- `t`: giây; `x,y,z`: vị trí EE trong world của URDF gốc, mét.
- `q1..q3`: rad; `qd1..qd3`: rad/s; `qdd1..qdd3`: rad/s².
- `vx,vy,vz`: m/s; `ax,ay,az`: m/s², tính từ động học Pinocchio với q/qd/qdd.
- `reference_paths.png/.svg`: hình chiếu trước và đường 3D.
- `square_joint_reference.png`: vị trí, vận tốc và gia tốc ba khớp.

Gazebo nâng cả robot thêm 0.30 m trên nền; đường hiển thị cũng được cộng
cùng độ cao. CSV giữ hệ world gốc để dùng chung với các controller.

## File dùng cho MATLAB / Simulink

File đã xuất: [matlab/trajectory_new.mat](results_new/gazebo/square/matlab/trajectory_new.mat),
cấu trúc tương thích `trajectory_new.mat` của repo `minhuster68/matlab-sim`.
Xem [hướng dẫn thay quỹ đạo trong MATLAB](results_new/gazebo/square/matlab/README.md).

File này gồm đủ APPROACH 5 s + SQUARE 27 s + RETURN 5 s, 3701 mẫu ở 100 Hz;
phần SQUARE có q/qd/qdd trùng CSV/Gazebo. Tạo lại bằng:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/src/gim_arm_control${PYTHONPATH:+:$PYTHONPATH}"
python3 -m gim_control.export_mat --shape square \
  --urdf src/gim_arm_description/urdf/gim_arm.urdf \
  --output results_new/gazebo/square/matlab/trajectory_new.mat
```

## Mở Gazebo và tự chạy xem hình

Build riêng để không thay install đang được launch phần cứng sử dụng:

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
colcon build --build-base build_sim_gazebo --install-base install_sim_gazebo \
  --symlink-install --packages-select gim_arm_description gim_control
source install_sim_gazebo/setup.bash
export ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11373
ros2 launch gim_control gazebo_square_preview.launch.py gui:=true repeat:=true
```

Không mở launch Gazebo thứ hai trên cùng cổng. Đóng launch trước bằng Ctrl+C
nếu muốn mở lại. `repeat:=false` để chỉ chạy một lượt.

Đường xanh lá là tham chiếu, quả cầu đỏ gắn ở đúng tool offset là EE. Node
tự chờ Gazebo/controller sẵn sàng, thêm đường tham chiếu vào scene, chuyển
từ forward_position_controller sang gim_arm_group_controller, rồi gửi
FollowJointTrajectory với đầy đủ positions/velocities/accelerations.

Chu trình gồm 5 s APPROACH, 27 s TRACK, 5 s RETURN, sau đó lặp nếu bật repeat.
Goal preview dùng sai số vị trí cuối 0.001 rad và vận tốc cuối 0.05 rad/s
để chấp nhận dao động số nhỏ của Gazebo lúc đứng yên; timeout cuối 2 s.
Đây là preview bằng position trajectory controller của Gazebo, không phải
kết quả đánh giá PID/LQR/MPC/SMC. Node kiểm robot_description phải dùng
gazebo_ros2_control/GazeboSystem trước khi gửi goal; không sử dụng CAN.

## Dùng cùng đường đi với thuật toán mô phỏng

`run_effort_algorithm.launch.py` đã nhận `trajectory_shape:=square` cho cả
bốn controller. Ví dụ PID, sau khi launch Gazebo như workflow README chính:

```bash
ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=pid pid_profile:=gazebo_smooth trajectory_shape:=square \
  tau_scale:=0.50 autostart:=false
```

Chỉ chạy một loại controller tại một thời điểm; dừng preview trước khi thử
controller torque. Workflow chuyển controller vẫn theo README chính.

## Kiểm tra

22 test gồm IK/FK theo URDF, giới hạn khớp/tốc độ/khoảng hở, đạo hàm giải
tích, tính liên tục C2, thời gian goal tăng nghiêm ngặt và marker không
thêm tải/collision. Các kiểm tra hình học không thay thế thử nghiệm tay thật.
