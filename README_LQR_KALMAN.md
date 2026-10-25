# Kalman bậc 2 cho vận tốc LQR

Đã bật trong `src/gim_arm_controller_lqr/config/lqr_circle_hardware.yaml`,
branch `lqr-test`, cho cả base/id0, shoulder/id1 và elbow/id2.
Gain LQR, feedforward theo tham chiếu, alpha base, trần torque và slew giữ nguyên.
Profile `lqr_joint_tuning.yaml` chưa bật lọc.

## Đường tín hiệu

```text
/joint_states.velocity (thô)
    -> 3 Kalman độc lập, mỗi bộ có trạng thái [vận tốc, gia tốc]
    -> vận tốc sau lọc
    -> e_v = vận tốc sau lọc - vận tốc tham chiếu
    -> LQR feedback -> giới hạn torque/slew -> driver
```

Đây là ước lượng vận tốc/gia tốc từ **đo vận tốc**, không phải bộ quan sát
[vị trí, vận tốc]. Không lọc góc, vận tốc tham chiếu hay torque đầu ra.
Feedforward vẫn tính từ quỹ đạo tham chiếu. Bộ lọc không dùng chéo số đo các khớp.
Kiểm tra đứng yên trước bài chạy vẫn dùng vận tốc thô; giới hạn khớp và timeout
vẫn dùng số đo gốc. Không tự chuyển mode hay chạy động cơ khi build.

## Tham số ban đầu

```yaml
velocity_filter_type: kalman2
velocity_kalman_measurement_std: [0.03, 0.01, 0.04]  # rad/s
velocity_kalman_jerk_std: [180.0, 60.0, 240.0]       # rad/s³
velocity_kalman_reset_gap_s: 0.05
```

Thứ tự luôn là base, shoulder, elbow. Đây là giá trị khởi đầu để thử, chưa phải
hiệp phương sai nhiễu đã nhận dạng trên tay thật.

Mô hình dự đoán: `v_next = v + dt*a`, `a_next = a`. Nhiễu quá trình giả sử
jerk ngẫu nhiên không đổi trong một khoảng mẫu: `Q_filter = jerk_std² * G*Gᵀ`,
`G=[dt²/2, dt]ᵀ`; `R_filter = measurement_std²`. Hai ma trận nhiễu này khác
với Q/R tối ưu điều khiển LQR.

- Tăng `measurement_std` hoặc giảm `jerk_std`: tin số đo ít hơn, lọc mạnh hơn,
  đáp ứng có thể chậm hơn và ảnh hưởng ổn định vòng kín.
- Giảm `measurement_std` hoặc tăng `jerk_std`: theo chuyển động nhanh hơn,
  giữ lại nhiều nhiễu hơn.
- Nhân cả hai std của một khớp cùng hệ số giữ nguyên gain lọc ở trạng thái ổn định.

Ở mẫu đều 100 Hz, cấu hình hiện tại có gain cập nhật vận tốc khoảng 0.6611
và gain gia tốc nhân dt khoảng 0.3493, giống nhau cho ba khớp do cùng tỷ lệ
nhiễu. Kalman có dự đoán gia tốc, không tương đương LPF một cực và không có
một con số độ trễ cố định cho mọi tần số.

Lọc chạy một lần khi có JointState mới, dùng dt từ header (dùng thời gian nhận
monotonic khi header bằng 0). Mẫu trùng header không cập nhật lại hoặc làm mới
timeout. Mất mẫu quá 50 ms hoặc timestamp quay ngược thì khởi tạo lại bằng vận
tốc đo, gia tốc bằng 0; không khởi tạo vận tốc bằng 0 khi khớp đang chuyển động.
Không cập nhật bộ lọc bằng cách lặp lại số đo cũ ở timer điều khiển.

## Chạy circle và xem kết quả

Đã build hai package `gim_control` và `gim_arm_controller_lqr`. Dừng node LQR cũ
trước khi chạy node mới để nạp code và YAML mới. Mỗi terminal dùng:

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
```

Launch phần cứng cần `torque_joint:=all` cho circle cả tay. Nếu chưa launch,
đặt/đỡ tay đúng HOME cơ khí trước khi đặt software zero:

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 set_zero_on_startup:=true zero_method:=software \
  torque_joint:=all
```

Terminal LQR, khi position controller đang giữ HOME:

```bash
ros2 launch gim_control lqr_sweep_hardware.launch.py \
  trajectory_shape:=circle approach_time:=16 return_time:=16 \
  params_file:=src/gim_arm_controller_lqr/config/lqr_circle_hardware.yaml \
  log_file:=results_new/lqr/circle_kalman_run01.csv
```

Sau khi node báo `READY_LQR`, terminal khác:

```bash
ros2 control switch_controllers --strict --activate-asap \
  --deactivate gim_arm_group_controller --activate gim_arm_effort_controller
ros2 param set /lqr_controller autostart true
```

Kết thúc chạy, trả về position trước khi dừng node LQR:

```bash
ros2 control switch_controllers --strict --activate-asap \
  --deactivate gim_arm_effort_controller --activate gim_arm_group_controller
```

Ctrl-C terminal LQR để ghi hết CSV, rồi plot:

```bash
ros2 run gim_control plot_joint_tracking \
  results_new/lqr/circle_kalman_run01.csv --show
```

CSV chính: `qd_raw_*` là vận tốc driver, `qd_filtered_*` và `qd_*` là vận tốc
đưa vào điều khiển. CSV `_states.csv` vẫn giữ `qd_*` là vận tốc driver gốc.
Mỗi hình vận tốc có đường thô màu xám, sau lọc màu khớp và tham chiếu đỏ;
hình vị trí/torque vẫn riêng. Không đánh giá giảm rung chỉ từ đường sau lọc:
cần so thêm vận tốc thô, sai số vị trí và torque giữa hai lượt.

Để đối chiếu không lọc, đổi `velocity_filter_type: none`, restart node LQR,
dùng tên log khác. Không cần sửa gain LQR để bật/tắt lọc.

## Kiểm chứng hiện tại

Kiểm tra mô hình đã tính thêm trạng thái bộ lọc, không chỉ kiểm `Ad-Bd*K`
với vận tốc lý tưởng. Cấu hình hiện tại qua 24 tư thế kiểm tra ban đầu,
591 mẫu bảng gain và 5.901 mẫu nội suy circle; rho lớn nhất dọc quỹ đạo
khoảng 0.99662. Hai rollout URDF không thêm trễ lệnh, bước tích phân 1 ms
và 0.5 ms, đều hoàn thành. Đây không phải bằng chứng bám tốt trên tay thật.

Bài kiểm tra thêm **một tick trễ lệnh 10 ms** không đạt (rho khoảng 1.53,
rollout dừng ở giới hạn khớp). Đối chiếu tại HOME: không lọc cũng không đạt
kiểm tra trễ này (rho khoảng 1.441; có lọc khoảng 1.488). Vì vậy không thể
coi bộ gain hiện tại chịu được thêm trễ 10 ms, và không khẳng định toàn bộ
bài audit đã pass. Xem `results_new/lqr/kalman_validation/offline_check.json`.
Không tự chạy tay thật trong lần tích hợp này.
