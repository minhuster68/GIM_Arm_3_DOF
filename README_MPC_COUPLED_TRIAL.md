# Thử MPC coupled — 2026-10-27

Branch: `mpc-test`. Profile:
`src/gim_arm_controller_mpc/config/mpc_circle_coupled_trial.yaml`.

Profile này lấy nguyên `mpc_circle_damping_trial.yaml` của lượt
`circle_damping_run01`, chỉ đổi `feedback_model_mode: coupled`. Đây là tên
chế độ trong source cho phản hồi có tương tác giữa các khớp. Mô hình dự đoán
dùng đầy đủ ma trận quán tính, đạo hàm vị trí và đạo hàm vận tốc; solver giải
một QP 30 biến thay cho ba QP 10 biến. Feedforward vẫn là nghịch động lực học
đầy đủ trên tham chiếu. Q/R/Rd, Kalman, horizon, tích phân và các giới hạn
không đổi so với lượt damping.

Không thay file independent, không đổi controller đang chạy. Phải dừng node
MPC cũ sau khi trả về position và khởi động node mới bằng profile này.
Không cần sửa thuật toán Python vì source đã hỗ trợ cả hai chế độ.

## Kiểm tra offline

- 27 test của chế độ independent/coupled và phân rã phản hồi đã qua.
- Tái phát 813 mẫu số đo từ `circle_damping_run01`: 813 nghiệm được solver
  chấp nhận, vi phạm ràng buộc lớn nhất khoảng 2,22e-16.
- Thời gian compute p50/p95/p99/max: 2,52/6,09/7,22/9,73 ms trong lần đo này.
- Xem `results_new/mpc/coupled_trial_check/offline_check.json`.

Đây là tái phát số đo với bảng dự đoán lấy mẫu, không phải mô phỏng chuyển
động vòng kín coupled. Không kết luận tay sẽ bớt rung hoặc giữ được ngân
sách 10 ms chỉ từ kết quả này. Chưa chạy tay thật, chưa thêm tải vào mô hình.

## Chạy vòng tròn

Phần cứng đã launch với `torque_joint:=all`, tại HOME. Dùng lượt không tải
đầu tiên để so với lượt damping vừa chạy. Terminal chạy MPC:

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1

ros2 launch gim_control mpc_sweep_hardware.launch.py \
  trajectory_shape:=circle \
  approach_time:=16 \
  return_time:=16 \
  params_file:=src/gim_arm_controller_mpc/config/mpc_circle_coupled_trial.yaml \
  log_file:=results_new/mpc/fine_tune/circle_coupled_run01.csv
```

Launch tự tạo thư mục log. Vì dùng trực tiếp đường dẫn YAML trong source
và không sửa mã Python, không cần build lại chỉ để dùng profile này.
Khi khởi động cần thấy `feedback_model_mode=coupled` trong mô tả controller.

Terminal khác, sau khi thấy READY_MPC:

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1

ros2 control switch_controllers --strict --activate-asap \
  --deactivate gim_arm_group_controller \
  --activate gim_arm_effort_controller
```

Chỉ sau khi switch thành công:

```bash
ros2 param set /mpc_controller autostart true
```

Kết thúc, chuyển về position trước khi Ctrl+C terminal MPC để ghi CSV:

```bash
ros2 control switch_controllers --strict --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate gim_arm_group_controller
```

Sau khi MPC ghi file:

```bash
ros2 run gim_control plot_joint_tracking \
  results_new/mpc/fine_tune/circle_coupled_run01.csv --show
```

Nếu tên log đã tồn tại, đổi số lượt ở cả launch và plot. Để quay lại phép
thử độc lập cùng bộ trọng số, restart MPC với
`params_file:=src/gim_arm_controller_mpc/config/mpc_circle_damping_trial.yaml`.
File `mpc_circle_independent_feedback.yaml` là profile independent với Rd
trước lượt damping; hai profile independent này khác trọng số r_rate.

## Giới hạn tích phân theo từng khớp

`integral_limit` vẫn là giới hạn chung, đơn vị rad.s. Có thể ghi đè từng
khớp bằng `integral_limit_per_joint: [0.6, 0.6, 0.9]`, theo thứ tự base,
shoulder, elbow. Phần tử bằng 0 kế thừa giới hạn chung; config cũ không có
vector vẫn giữ hành vi trước đây. Log khởi động in giới hạn thực được dùng.

Profile `mpc_circle_coupled_no_kalman_trial.yaml` hiện nới riêng elbow lên
±0.9 rad.s; base/shoulder vẫn ±0.6 rad.s. Đây là giới hạn I-state, không
phải giới hạn torque tích phân tính bằng Nm. Thay đổi giới hạn không tự
loại bỏ ràng buộc slew trong horizon; phải restart node MPC để nạp lại.

## Thử slew elbow 20 Nm/s — 2026-10-30

Profile `mpc_circle_coupled_no_kalman_trial.yaml` tăng riêng
`torque_slew_rate` từ `[12, 80, 15]` lên `[12, 80, 20]` Nm/s.
Ở 100 Hz, feedback elbow được đổi tối đa 0.20 Nm mỗi bước dự đoán,
thay cho 0.15 Nm. Đây là giới hạn tốc độ đổi feedback trong QP;
feedforward vẫn tính từ tham chiếu và torque tổng vẫn có trần riêng.
Giữ I-limit `[0.6, 0.6, 0.9]`, Q/R/Rd, mô hình coupled và tắt Kalman
để so với `circle_coupled_integral09_run01` có tải 0.5 kg.

Chưa đổi luật chống tích phân bão hòa trong lượt thử này. Nghiệm QP tự do
khác nghiệm constrained không đồng nghĩa toàn bộ phần tích phân mới cần
bị chặn: với tải chưa biết, tích phân vẫn cần tích lũy để bù tải. Đặc biệt
ở chế độ coupled, chênh lệch torque một khớp có thể do trạng thái các
khớp khác. Không dùng riêng `tau_mpc_correction` làm điều kiện đóng băng
tích phân. Cần đánh giá TRACK và RETURN sau bước đổi slew trước khi
thiết kế thêm cơ chế chống windup xét ràng buộc QP.

Không cần build lại khi chỉ sửa YAML; phải restart node MPC.
Đổi `params_file` trong lệnh chạy sang profile no-Kalman trên và dùng log
mới `results_new/mpc/fine_tune/circle_coupled_slew20_run01.csv`.

## Thử prediction horizon 20 bước sau integral09_run03 — 2026-10-30

Run03 được người vận hành xác nhận vẫn có tải 0.5 kg và đã nạp
`q_position` elbow 1.2. So với số liệu run02 đã phân tích, elbow có
RMS APPROACH 4.09 → 5.05°, TRACK 8.24 → 10.14°, RETURN 5.73 → 5.25°.
Trong TRACK, độ lệch phía dưới tham chiếu tăng từ 13.89 lên 23.00°,
độ lệch phía trên giảm từ 22.35 xuống 12.16°. Tích phân chạm -0.9 trong
54.52% TRACK. Chưa chốt được bộ số này cho tải.

Lượt thử tiếp theo trong `mpc_circle_coupled_no_kalman_trial.yaml` chỉ
giảm `prediction_horizon: 40 → 20`, tương ứng 0.40 → 0.20 s ở 100 Hz.
Giữ `control_horizon: 10`, Q/R/Rd, I-limit, slew và chế độ coupled.
Horizon là tham số chung cho cả ba khớp; cần đọc lại cả base/shoulder.

Tái dựng offline tại các trạng thái cố định của run03 cho torque elbow:

| Mẫu | Horizon 40 | Horizon 20 |
|---|---:|---:|
| TRACK thiếu góc lớn nhất | 3.406 Nm | 3.602 Nm |
| TRACK vượt góc lớn nhất | 2.895 Nm | 2.691 Nm |
| RETURN vượt góc lớn nhất | 2.657 Nm | 2.477 Nm |

Tái phát 90 trạng thái ghi nhận: horizon 20 có 90 nghiệm được chấp nhận
và solver báo thành công, vi phạm ràng buộc tối đa 2.22e-16. Đây là phép
tính dùng số đo/tích phân/feedback trước đó từ CSV, không phải mô phỏng
vòng kín có tải. Kiểm tra cực của mô hình danh định tại các điểm đó cũng
không thay thế kiểm chứng ổn định khi có tải và ràng buộc.

Không chạy phần cứng trong bước này. Restart MPC để nạp YAML, không cần
build lại. Dùng log mới `circle_coupled_horizon20_run01.csv`.
Để quay lại bộ đã chạy run03, đặt `prediction_horizon: 40`.

## Thử prediction horizon 18 sau integral09_run07 — 2026-10-30

Run07 dùng horizon 20/10, q_position elbow 1.25 và I-limit elbow 1.0.
RMS elbow APPROACH/TRACK/RETURN lần lượt 3.91/3.75/4.89°, đỉnh RETURN
9.97°. TRACK cân bằng hơn hai phía (-6.79/+6.81°), nhưng RETURN chưa
cải thiện. Đây là các số đo trên mẫu CONTROL, không gồm thời gian đứng yên.

Lượt thử tiếp giảm riêng `prediction_horizon: 20 → 18` (0.18 s ở 100 Hz),
giữ `control_horizon: 10`. Giữ Q/R/Rd, I-limit `[0.6, 0.6, 1.0]`, slew
`[12, 80, 20]`, q_position elbow 1.25 và tắt Kalman. Horizon dùng chung
cho cả ba khớp; phải đánh giá lại base/shoulder cùng elbow.

Tại các trạng thái cố định của run07, torque elbow offline:

| Mẫu | Np=20 | Np=18 |
|---|---:|---:|
| TRACK thiếu góc lớn nhất | 3.397 Nm | 3.473 Nm |
| TRACK vượt góc lớn nhất | 2.822 Nm | 2.772 Nm |
| RETURN vượt góc lớn nhất | 1.372 Nm | 1.147 Nm |

Tái phát 90 trạng thái của ba pha chuyển động với Np=18: 90 nghiệm
được chấp nhận và solver báo thành công, vi phạm lớn nhất 2.22e-16.
Đây không phải mô phỏng vòng kín có tải hay kiểm chứng độ bám thực tế.
Không chạy phần cứng trong bước cập nhật. Restart node MPC để áp dụng;
không cần build lại. Để quay lại cấu hình run07, đặt Np=20.
