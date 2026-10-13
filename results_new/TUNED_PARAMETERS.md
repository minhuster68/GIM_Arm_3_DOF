# Tham số đang lưu để chạy so sánh hình tròn

Snapshot ngày 2026-10-13, lấy trực tiếp từ từng branch local. Đây là bộ số đang lưu sau các lượt tune; báo cáo kết quả phải gắn với log thực nghiệm tương ứng.

CSV, ảnh và YAML của mỗi thuật toán nằm trong **branch tương ứng**: `pid-test` giữ `results_new/pid/`, `lqr-test` giữ `results_new/lqr/`, `mpc-test` giữ `results_new/mpc/`, `smc-test` giữ `results_new/smc/`. Chuyển branch để xem dữ liệu của thuật toán khác. Bảng tham số tổng hợp này được giữ trên cả bốn branch; xem [README.md](README.md) để tìm các bài chạy đã lưu.

Thứ tự tất cả vector: **base (q1/ID0), shoulder (q2/ID1), elbow (q3/ID2)**. Đơn vị nguồn: rad, rad/s, Nm. Các bản sao `pid/params_tuned.yaml`, `lqr/params_tuned.yaml`, `mpc/params_tuned.yaml`, `smc/params_tuned.yaml` được dùng trực tiếp trong các lệnh chạy.

| Thuật toán | Branch | Commit nguồn | File YAML gốc |
|---|---|---|---|
| PID | `pid-test` | `1a4ab5bc5499` | `src/gim_arm_controller_pid/config/pid_hardware_soft.yaml` |
| LQR | `lqr-test` | `38d8d3287a61` | `src/gim_arm_controller_lqr/config/lqr_joint_tuning.yaml` |
| MPC | `mpc-test` | `8318f40a54b9` | `src/gim_arm_controller_mpc/config/mpc_joint_tuning.yaml` |
| SMC | `smc-test` | `a60481f70563` | `src/gim_arm_controller_smc/config/smc_joint_tuning.yaml` |

## Thiết lập chạy chung

- Quỹ đạo `circle`, 1 vòng; `approach_time=16 s`, `return_time=16 s`.
- `control_hz=100 Hz`, `tau_scale=1`; giới hạn torque từ URDF `[5, 40, 5] Nm`.
- `abort_on_tracking_error=false` trên cả bốn bộ. Không abort chỉ vì vượt ngưỡng sai số bám 0.1 rad; các kiểm tra dữ liệu trạng thái và giới hạn khớp vẫn hoạt động.
- `state_timeout=0.25 s`, `joint_margin_rad=0.05`, ngưỡng vận tốc bắt đầu `0.05 rad/s` do launch đặt.
- `base_ff_alpha`: PID `2.0`, LQR `2.2`, MPC `2.2`, SMC `2.46`. Ghi rõ các giá trị khác nhau này khi so sánh.
- Launch nạp YAML rồi ghi đè các tham số quỹ đạo nêu trên, `autostart=false`, chế độ giữ của thuật toán. Người chạy bật `autostart=true` sau khi chuyển controller.
- Mỗi bài lưu CSV và 6 PNG vào thư mục thuật toán. So sánh sai số vị trí nên dùng cùng pha `TRACK`; hình vận tốc bổ sung để quan sát độ mượt.

## PID một vòng vị trí + feedforward

| Tham số | Vector base, shoulder, elbow |
|---|---|
| Kp | `[30, 22, 2.5]` |
| Ki | `[2.5, 3, 0.6]` |
| Kd | `[0.1, 10, 0.5]` |
| Giới hạn tích phân | `[0.3, 0.3, 0.3]` rad.s |

Tên node `cascade_pid_controller` được giữ để tương thích; nhánh pid-test xây dựng `PositionPidController` một vòng.

## LQR có tích phân

Thứ tự trạng thái `x=[integral(e_q), e_q, e_qdot]`, mỗi nhóm gồm ba khớp.
`Q=diag([1/max_int_e², 1/max_e², 1/max_de²])`, `R=diag(tau_penalty_scale/max_tau²)`.

| Tham số | Vector base, shoulder, elbow |
|---|---|
| max_int_e | `[0.040824829, 0.0577350269, 0.0577350269]` |
| max_e | `[0.025, 0.0333333333, 0.0223606798]` |
| max_de | `[1, 0.707106781, 1]` |
| max_tau để tính R | `[7.15541753, 40, 7.15541753]` |
| Q tích phân thực tế | `[600, 300, 300]` |
| Q vị trí thực tế | `[1600, 900, 2000]` |
| Q vận tốc thực tế | `[1, 2, 1]` |
| R thực tế | `[10, 0.32, 10]` |
| Torque slew rate | `[5, 20, 5]` Nm/s |

`tau_penalty_scale=512`, `integral_limit=0.05 rad.s`, `position_tracking_only=false`, `use_discrete_lqr=true`, `gain_schedule_mode=trajectory_time`, `gain_schedule_hz=10 Hz`. `max_tau` là thang tính R, giới hạn torque thật vẫn `[5,40,5] Nm`. Ma trận gain K thay đổi theo điểm tuyến tính hóa; không có một bộ K cố định cho toàn bộ quỹ đạo.

## MPC

Cùng thứ tự trạng thái `x=[integral(e_q), e_q, e_qdot]`. `weight_convention=matlab_squared`:

```text
Q_integral = (q_integral/output_scale_integral)^2
Q_position = (q_position/output_scale_position)^2
Q_velocity = (q_velocity/output_scale_velocity)^2
R          = torque_penalty_scale*(r_input/input_scale)^2
R_delta    = torque_penalty_scale*(r_rate/input_scale)^2
```

| Trọng số dùng trong bài toán tối ưu | Vector đường chéo base, shoulder, elbow |
|---|---|
| Q tích phân | `[256, 1000, 256]` |
| Q vị trí | `[300, 2000, 400]` |
| Q vận tốc | `[0.2, 5, 0.04]` |
| R torque | `[0.037636, 0.0005880625, 0.037636]` |
| R_delta biến thiên torque | `[0.36, 0.05, 0.36]` |
| Torque slew rate | `[12, 50, 6.5]` Nm/s |

`prediction_horizon=40` (0.4 s tại 100 Hz), `control_horizon=10` (10 bước điều khiển), `integral_limit=0.05 rad.s`, `qp_solver=active_set`, `max_solver_iterations=100`, `precompute_prediction=true`, `feedback_bounds_mode=trajectory_global`.

Thang chuẩn hóa: integral `[0.25, 0.25, 0.25]`, vị trí `[0.05, 0.05, 0.05]`, vận tốc `[0.1, 0.1, 0.1]`, torque `[5, 40, 5]`; `torque_penalty_scale=10000`.

**R hiện tại không phải 1**; số `r_input=0.0097` trong YAML phải được quy đổi như trên. Không thay đổi các trọng số trong đợt chuẩn bị này.

## SMC

| Tham số | Vector base, shoulder, elbow |
|---|---|
| lambda_gain | `[15.7, 20, 18]` |
| ks | `[85, 44, 30]` |
| kr | `[3, 4.2, 1]` |
| phi | `[0.2, 0.15, 0.5]` |
| Torque slew rate | `[6.5, 48, 7]` Nm/s |

`controlled_joint=all`. Các phương trình và cách áp dụng lớp biên lấy từ source smc-test, không lấy package SMC trên các branch khác.
