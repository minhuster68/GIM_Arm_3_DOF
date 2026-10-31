# Đồ thị sau khi chạy quỹ đạo

Công cụ plot xuất **9 hình PNG: 3 hình riêng cho mỗi khớp**:

- `tracking_q1_base.png`: vị trí thực tế/mong muốn và sai số vị trí.
- `tracking_q1_base_torque.png`: torque đầu ra, feedforward, feedback và các thành phần có trong CSV.
- `tracking_q1_base_velocity.png`: vận tốc thực tế/mong muốn và sai số vận tốc.
- Khớp shoulder và elbow có ba hình tương tự, tên `q2_shoulder` và `q3_elbow`.

Terminal tune từng khớp tự gọi công cụ này sau mỗi trial. Với log quỹ đạo đầy đủ, chạy:

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run gim_control plot_joint_tracking \
  results_new/mpc/fine_tune/circle_coupled_run01.csv \
  --show --max-plot-points 1000000
```

Thay đường dẫn CSV bằng log cần xem. Mặc định tên hình bắt đầu bằng tên CSV rồi thêm `_joints`; ví dụ `circle_coupled_run01_joints_q3_elbow_torque.png`. `--output results_new/mpc/tracking.png` đổi prefix thành `tracking`. `--show` mở cả 9 hình sau khi lưu; bỏ tùy chọn này để chỉ xuất file. File PNG là ảnh tĩnh; thao tác tương tác nằm trong cửa sổ mở bằng `--show` và cần desktop/backend GUI của Matplotlib.

Trong cửa sổ torque:

- Cuộn chuột lên/xuống để phóng to/thu nhỏ hai trục quanh vị trí con trỏ.
- Bấm vào đường mẫu trong chú giải để ẩn/hiện một thành phần torque. Các đường khác giữ nguyên thang đo để dễ đối chiếu.
- Dùng **Zoom** trên thanh công cụ để khoanh vùng, **Pan** để kéo, **Home** để trở về toàn bộ đồ thị, **Back/Forward** để quay lại các mức zoom, **Save** để lưu hình đang xem.

Mặc định công cụ giảm số điểm vẽ xuống tối đa 6000 để mở nhanh. Nếu cần soi rung ở các đoạn ngắn, tăng `--max-plot-points` như lệnh trên để giữ toàn bộ mẫu của các lượt chạy thông thường. Zoom không khôi phục các mẫu đã bị giảm khi vẽ. RMS/MAX vẫn tính trên toàn bộ dữ liệu CSV.

Vị trí dùng độ, vận tốc dùng độ/s, torque dùng Nm. Vận tốc được lấy trực tiếp từ các cột `qd_*` và `qdref_*` trong CSV. Sai số vận tốc bằng vận tốc thực tế trừ vận tốc mong muốn. RMS và MAX trên hình vận tốc tính từ toàn bộ mẫu, kể cả khi giảm mẫu hiển thị bằng `--max-plot-points`. Bảng đánh giá tracking trên terminal tiếp tục dùng sai số vị trí và torque.
