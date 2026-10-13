# Đồ thị sau khi chạy quỹ đạo

Công cụ plot chung của PID, LQR, MPC và SMC xuất **6 hình PNG: 2 hình riêng cho mỗi khớp**:

- `tracking_q1_base.png`: vị trí thực tế/mong muốn, sai số vị trí và torque.
- `tracking_q1_base_velocity.png`: vận tốc thực tế/mong muốn và sai số vận tốc.
- Khớp shoulder và elbow có cặp hình tương tự, tên `q2_shoulder` và `q3_elbow`.

Terminal tune từng khớp tự gọi công cụ này sau mỗi trial. Với log quỹ đạo đầy đủ, chạy:

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run gim_control plot_joint_tracking results/mpc_circle_no_error_abort_run01.csv --show
```

Thay đường dẫn CSV bằng log cần xem. Mặc định tên hình bắt đầu bằng tên CSV rồi thêm `_joints`; ví dụ `mpc_circle_no_error_abort_run01_joints_q1_base_velocity.png`. `--output results/tracking.png` đổi prefix thành `tracking`. `--show` mở cả 6 hình sau khi lưu; bỏ tùy chọn này để chỉ xuất file.

Vị trí dùng độ, vận tốc dùng độ/s, torque dùng Nm. Vận tốc được lấy trực tiếp từ các cột `qd_*` và `qdref_*` trong CSV. Sai số vận tốc bằng vận tốc thực tế trừ vận tốc mong muốn. RMS và MAX trên hình vận tốc tính từ toàn bộ mẫu, kể cả khi giảm mẫu hiển thị bằng `--max-plot-points`. Bảng đánh giá tracking trên terminal tiếp tục dùng sai số vị trí và torque.
