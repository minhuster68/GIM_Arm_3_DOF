# Báo cáo chẩn đoán PID/LQR trên Gazebo — 2026-09-28

## Phạm vi

- Chỉ chạy Gazebo Classic; không kết nối hay phát lệnh tới phần cứng thật.
- Plant được giữ nguyên giữa các lượt thử, mỗi lượt chỉ đổi một biến.
- Bài tái lập ngắn đi qua đúng vùng quỹ đạo từng mất ổn định; sau khi tìm được
  cấu hình ổn định, PID và LQR đều được chạy lại trên toàn bộ quỹ đạo 27 s.

## Kết luận

Nguồn ban đầu của cú giật không phải gain scheduling của LQR. Đó là một mode
vận tốc số của ODE Quick solver khi dùng mặc định 50 iterations cho mô hình có
tỉ số khối lượng/quán tính khó hội tụ. Ở physics 2 kHz, mode trội là 191.3 Hz;
khi chỉ tăng physics lên 4 kHz, mode chuyển thành 388.4 Hz. Việc tần số gần
gấp đôi theo bước tích phân là dấu hiệu mode bị khóa theo bước solver, không
phải mode cơ học hay cực điều khiển.

Sau khi mode đã bắt đầu, vận tốc shoulder vượt giới hạn URDF 1.963 rad/s.
Gazebo khi đó zero lực có chiều làm vận tốc vượt thêm (`CheckAndTruncateForce`),
làm dao động nặng lên. Đây là cơ chế khuếch đại thứ cấp, không phải nguồn khởi
phát.

Sửa bền vững là giữ physics và ros2_control ở 2 kHz, đồng thời tăng ODE Quick
solver từ 50 lên 200 iterations. Không dùng lọc vận tốc để che dao động và
không nới giới hạn an toàn.

## Ma trận thử nghiệm chính

| Trường hợp | Kết quả |
|---|---|
| LQR 200 Hz, ODE mặc định 50 iterations, physics 2 kHz | ABORT sau 1.175 s TRACK; mode qd 191.3 Hz |
| PID cùng plant và 200 Hz | ABORT sau khoảng 1.235 s TRACK |
| LQR, chỉ đổi physics 2 kHz → 4 kHz | ABORT; mode chuyển thành 388.4 Hz |
| LQR + low-pass qd 30 Hz | Qua TRACK nhưng hỏng ở RETURN; qd thô vượt 100 rad/s, nên loại bỏ |
| LQR, chỉ đổi ODE iterations 50 → 200 | Qua APPROACH, TRACK, RETURN, HOLD |
| PID, ODE 200 iterations | Qua APPROACH, TRACK, RETURN, HOLD |
| LQR đầy đủ, 27 s TRACK, ODE 200 | Hoàn tất, không ABORT/clipping |
| PID đầy đủ 2 kHz, 27 s TRACK, ODE 200 | Hoàn tất, không ABORT |

## Kết quả toàn quỹ đạo

Sai số dưới đây là trị tuyệt đối lớn nhất theo `[base, shoulder, elbow]` trong
pha TRACK.

| Controller | Sai số góc cực đại (deg) | RMS sai số (deg) | Mô-men cực đại (Nm) |
|---|---:|---:|---:|
| LQR `safe_100hz`, `tau_scale=0.35` | `[0.122, 0.190, 0.256]` | `[0.023, 0.027, 0.036]` | `[0.778, 3.631, 1.548]` |
| PID `gazebo_smooth`, `tau_scale=0.50` | `[0.220, 0.704, 0.209]` | `[0.053, 0.278, 0.069]` | `[0.771, 3.625, 1.542]` |

Với LQR ổn định:

- qd lớn nhất ở log 2 kHz là `[0.282, 0.319, 0.285] rad/s`, thấp hơn nhiều so
  với giới hạn vận tốc URDF;
- không có mẫu torque bị clipping;
- biên độ phổ trong dải 175–210 Hz chỉ khoảng `7e-6 rad/s`, tức mode 191 Hz
  trước đó đã biến mất;
- chu kỳ `/joint_states` trung vị vẫn là 0.5 ms và mô phỏng giữ được real-time.

## Thay đổi dùng để chẩn đoán và sửa

- `gim_arm_zero_gravity.world`: khai báo ODE Quick solver 200 iterations.
- `gazebo_description.py`: chỉ trong URDF runtime của Gazebo, xuất thêm effort
  state để phân biệt torque controller yêu cầu và lực Gazebo thực sự áp dụng.
- `effort_controller_node.py`: ghi effort state vào log LQR tốc độ cao.
- `analyze_lqr_diagnostics.py`: kiểm qd theo bước semi-implicit, phổ tốc độ,
  độ trễ command→effort và hiện tượng cắt lực theo velocity limit.

## Log chính

- Baseline lỗi LQR: `/tmp/gim_lqr_effort_state_baseline_20260928.csv`
- Physics 4 kHz: `/tmp/gim_lqr_physics4k_ctrl200_20260928.csv`
- ODE 200, đoạn chẩn đoán: `/tmp/gim_lqr_ode_iters200_20260928.csv`
- LQR toàn quỹ đạo: `/tmp/gim_lqr_full_ode200_rerun_20260928.csv`
- State LQR 2 kHz: `/tmp/gim_lqr_full_ode200_rerun_20260928_states.csv`
- PID toàn quỹ đạo: `/tmp/gim_pid_full_ode200_20260928.csv`

Các file `/tmp` là artifact cục bộ và có thể mất sau khi reboot.

## Bổ sung 2026-09-29: ổn định lúc switch controller

Một lỗi độc lập được tái lập trong pha `GRAVITY`, trước khi LQR bắt đầu:
gravity-hold PID chạy 100 Hz làm các khớp quán tính thấp mất ổn định khoảng
1.5 s sau khi chuyển `position -> effort`. Elbow đi tới vùng đệm lower-limit
và safety ABORT đúng thiết kế. Đây không phải mode 191 Hz trong TRACK.

Cấu hình `safe_100hz` hiện dùng torque/HOLD loop 200 Hz và cập nhật gain mỗi
hai chu kỳ, tức gain schedule vẫn chạy 100 Hz. Sau thay đổi, GRAVITY giữ HOME
ổn định hơn một phút và LQR hoàn tất lại toàn bộ 5 s APPROACH + 27 s TRACK +
5 s RETURN, không ABORT hay torque clipping. Log xác nhận:
`/tmp/gim_lqr_hold200_test.csv` và
`/tmp/gim_lqr_hold200_test_states.csv`.

## Kiểm tra build/test

- `colcon build --packages-select gim_control gim_arm_controller_pid
  gim_arm_controller_lqr --symlink-install`: đạt.
- Python compile và XML parse cho các file thay đổi: đạt.
- Test package hiện không xanh do các lỗi flake8/pep257 đã tồn tại rải rác trong
  package và do plugin `anyio` của user site không tương thích pytest hệ thống;
  không có lỗi lint nào được báo trong ba file chẩn đoán thay đổi ở lượt này.
