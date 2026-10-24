# SMC phản hồi độc lập — 2026-10-24

Branch: `smc-test`. Profile đang dùng:
`src/gim_arm_controller_smc/config/smc_circle_from_terminal_tuning.yaml`.
Đây là YAML phần cứng duy nhất sau khi dọn cấu hình. Các bản tuning/snapshot
cũ đã xóa; dữ liệu CSV/PNG của những lượt trước vẫn được giữ nguyên.

## Luật điều khiển

Với e = q - q_ref, de = qd_used - qd_ref:

```text
s_i       = de_i + lambda_i * e_i
a_fb_i    = -lambda_i * de_i - k1_i * sqrt(abs(s_i)) * sign(s_i) + z_i
z_dot_i   = -k2_i * sign(s_i)
tau_ff    = inverse_dynamics(q_ref, qd_ref, qdd_ref)
tau_fb_i  = M_ii(q_measured) * a_fb_i
tau_cmd   = clip(tau_ff + slew(tau_fb), torque_limits)
```

Hiệu chỉnh `base_ff_alpha=2.46` vẫn nhân riêng tau_ff của base.
Chế độ này dùng `feedback_offdiag_scale: [0.0, 0.0, 0.0]`, nên không có
`M_ij * a_fb_j` (i khác j) trong torque phản hồi. Các profile khác giữ chế độ
đã khai báo; mặc định vẫn ghép đầy đủ nếu không đặt tham số này.

FF vẫn dùng mô hình đầy đủ. M_ii có thể phụ thuộc toàn bộ tư thế q; đây không
phải tách rời động lực học cơ khí. Bộ này đã chạy đủ APPROACH/TRACK/RETURN/HOLD trên tay thật trong
circle_independent_fb_run01.csv. RMS vị trí TRACK [base, shoulder, elbow]
là [0.358, 0.619, 1.413] độ; MAX là [1.724, 2.576, 4.873] độ.
Đây là bộ đã chốt cho quỹ đạo và điều kiện đã thử.

Giữ gain tại lúc chuyển cấu trúc để so sánh cùng điều kiện, thứ tự [base, shoulder, elbow]:

```yaml
lambda_gain: [48.0, 32.0, 12.0]
k1: [10.0, 12.0, 3.5]
k2: [4.0, 3.0, 4.0]
z_limit: [20.0, 40.0, 30.0]
torque_slew_rate: [6.0, 16.0, 1.5]
```

Giá trị elbow k1=3.5 là giá trị hiện tại trong profile/nguyên lượt circle trước,
không phải giá trị 4.5 từng kiểm chứng khi tune riêng.

Các thành phần tau_lambda, tau_k1, tau_k2 và anti-windup z dùng cùng ma trận
phản hồi đã chọn. Với profile này, giới hạn torque khớp khác không trực tiếp
khóa tích lũy z_i qua phần tử M_ji đã tắt. Giới hạn của chính khớp vẫn áp dụng.
Trong CSV mới, tau_feedback_cross là đóng góp chéo thực tế sau hệ số scale,
bằng 0 trong chế độ độc lập; log cũ vẫn giữ nguyên ý nghĩa/dữ liệu lịch sử.

## Chạy vòng tròn

Trong mỗi terminal mới:

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
```

Nếu hardware đang launch với torque_joint=elbow/base/shoulder, dừng node SMC
và launch phần cứng cũ sau khi đã trả về position. Đặt/đỡ tay đúng HOME cơ khí
rồi mới đặt software zero bằng launch sau. Chỉ giữ một launch phần cứng:

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 \
  set_zero_on_startup:=true \
  zero_method:=software \
  torque_joint:=all
```

Terminal SMC:

```bash
ros2 launch gim_control smc_sweep_hardware.launch.py \
  trajectory_shape:=circle \
  approach_time:=16 \
  return_time:=16 \
  params_file:=src/gim_arm_controller_smc/config/smc_circle_from_terminal_tuning.yaml \
  log_file:=results_new/smc/circle_independent_fb_run01.csv
```

Log khởi động cần có `tau_fb_raw=diag(M(q_measured)) @ a_fb` và
`feedback_offdiag_scale=[0. 0. 0.]`. Chờ READY_SMC, tay đứng yên, rồi chạy
trong terminal khác khi đang giám sát:

```bash
ros2 control switch_controllers --deactivate gim_arm_group_controller --activate gim_arm_effort_controller --strict
ros2 param set /smc_controller autostart true
```

Khi RETURN -> HOLD_HOME, hoặc cần dừng bài, trả về position trước khi Ctrl+C
node SMC để ghi CSV:

```bash
ros2 control switch_controllers --deactivate gim_arm_effort_controller --activate gim_arm_group_controller --strict
```

Sau Ctrl+C node SMC:

```bash
ros2 run gim_control plot_joint_tracking results_new/smc/circle_independent_fb_run01.csv --show
```

Không ghi đè log nếu chạy lại: đổi run01 thành run02, run03...
So sánh RMS/MAX trong cùng pha APPROACH/TRACK, không gộp thời gian chờ HOME.
Theo dõi thêm peak torque, slew_limited, integral/z và vận tốc; phản hồi độc lập
không tự bảo đảm bám tốt hơn hoặc hết rung.
