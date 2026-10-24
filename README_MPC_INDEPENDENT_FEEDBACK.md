# MPC phản hồi độc lập — 2026-10-24

Thay đổi thuộc branch `mpc-test`. Đây là chế độ thử, chưa kiểm chứng trên tay thật.
Profile mới: `src/gim_arm_controller_mpc/config/mpc_circle_independent_feedback.yaml`.
Profile MPC cũ và `results_new/mpc/params_tuned.yaml` giữ nguyên để so sánh.

## Cấu trúc

```text
tau_ff = inverse_dynamics(q_ref, qd_ref, qdd_ref)   # mô hình đầy đủ
tau_ff_base *= base_ff_alpha                      # giữ 2.2 của MPC
tau_i = clip(tau_ff_i + u_i, torque_limits_i)
```

`feedback_model_mode: independent` thay mô hình sai số ghép bằng các mô hình
riêng cho mỗi khớp:

```text
M_ii(q_ref) * e_ddot_i + Hq_ii * e_i + Hv_ii * e_dot_i = u_i
x_i = [integral(e_i), e_i, e_dot_i]
```

Hq/Hv là đạo hàm của inverse dynamics đầy đủ quanh quỹ đạo tham chiếu; chỉ lấy
phần tử đường chéo trong mô hình sai số độc lập. Dùng `1/M_ii`, không dùng
phần tử đường chéo của `inv(M)` vì hai đại lượng này khác nhau khi M có hạng chéo.
Mô hình cơ khí dùng ZOH, trạng thái tích phân vẫn cập nhật Euler như bộ cũ.

QP được tách thành ba bài toán riêng. Với horizon điều khiển 10, mỗi QP có 10
biến thay cho QP ghép 30 biến. Mỗi khớp dùng riêng warm-start, kiểm tra nghiệm
và fallback, kể cả khi một solver đạt giới hạn vòng lặp hoặc trả nghiệm không
hợp lệ. Hỗ trợ active_set, ADMM và SLSQP. solver_iterations tổng là tổng số
vòng lặp của các QP; solver_accepted tổng chỉ true nếu mọi khớp chấp nhận nghiệm.
Controller.last còn có solver_accepted_joints cho chẩn đoán trong Python.

Các trọng số vị trí/vận tốc/tích phân, r_input/r_rate, horizon, FF, giới hạn
torque, slew và tích phân giữ nguyên bộ MPC đã tune. Chỉ profile mới bật chế độ
độc lập; mặc định `coupled` giữ luật cũ. Không loại bỏ tương tác cơ khí thật;
M_ii và các đạo hàm vẫn phụ thuộc tư thế tham chiếu cả tay.

Đây là mô hình sai số xấp xỉ. Kết quả SMC tốt hơn không bảo đảm MPC sẽ cải thiện;
cần chạy đối chiếu cùng quỹ đạo và cùng tham số, rồi mới quyết định tune lại Q/R.

## Kiểm tra phần mềm

- 33 test MPC qua, gồm bộ hồi quy coupled và test độc lập cả ba solver.
- Kiểm tra mô hình/QP không có hạng liên khớp; FF vẫn đầy đủ; sai số, warm-start
  và fallback của khớp khác không thay phản hồi khớp đang xét.
- Kiểm tra cache so với tính online, giới hạn torque/slew và tham số không hợp lệ.
- Package build thành công trong thư mục kiểm tra tạm, không thay bản install
  đang chạy SMC. Cần build trong workspace sau khi chuyển sang mpc-test.
- Bài đo phần mềm với URDF, 160 tick, cache tham chiếu và sai số tổng hợp,
  OPENBLAS/OMP=1: coupled median/p95/max 2.85/7.47/13.28 ms;
  independent 2.80/4.70/6.26 ms (bỏ 10 tick đầu khi tính percentile).
  Mỗi chế độ chấp nhận nghiệm 160/160 tick. Tham chiếu lấy mẫu từ log SMC mới;
  gia tốc đặt 0 trong bài đo này. Đây không phải mô phỏng vòng kín hoặc bảo đảm
  deadline 10 ms trên hệ thật.

## Chạy vòng tròn

Dừng bài điều khiển hiện tại, trả về position trước khi dừng node thuật toán.
Nếu workspace còn thay đổi SMC chưa lưu, lưu vào stash trước khi switch; không
dùng reset/clean để ép chuyển nhánh. Ví dụ:

```bash
cd ~/GIM_Arm_3_DOF
git stash push -u -m "SMC before MPC independent feedback"
git switch mpc-test
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gim_control gim_arm_controller_mpc
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
```

Trong mỗi terminal mới phải source ROS/install và export hai biến như trên.
Sau khi build, restart phần cứng tại HOME cơ khí nếu cần để dùng đúng source
của branch; chỉ giữ một launch phần cứng, với `torque_joint:=all`:

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 \
  set_zero_on_startup:=true \
  zero_method:=software \
  torque_joint:=all
```

Đặt/đỡ tay đúng HOME trước khi đặt software zero. Terminal MPC:

```bash
ros2 launch gim_control mpc_sweep_hardware.launch.py \
  trajectory_shape:=circle \
  approach_time:=16 \
  return_time:=16 \
  params_file:=src/gim_arm_controller_mpc/config/mpc_circle_independent_feedback.yaml \
  log_file:=results_new/mpc/circle_independent_fb_run01.csv
```

Xác nhận log có `feedback_model_mode=independent`. Chờ READY_MPC và tay đứng
yên rồi, khi đang giám sát, chạy trong terminal khác:

```bash
ros2 control switch_controllers --deactivate gim_arm_group_controller --activate gim_arm_effort_controller --strict
ros2 param set /mpc_controller autostart true
```

Khi RETURN -> HOLD_HOME, hoặc cần dừng bài, trả về position trước khi Ctrl+C
node MPC để ghi CSV:

```bash
ros2 control switch_controllers --deactivate gim_arm_effort_controller --activate gim_arm_group_controller --strict
```

Sau Ctrl+C node MPC:

```bash
ros2 run gim_control plot_joint_tracking results_new/mpc/circle_independent_fb_run01.csv --show
```

Không ghi đè log; đổi run01 khi chạy lại. Muốn so cấu hình ghép thì dùng
`results_new/mpc/params_tuned.yaml` và tên CSV khác. Chỉ so RMS/MAX ở cùng pha
APPROACH/TRACK/RETURN, cùng thời lượng; theo dõi cả vận tốc, torque, solver và
thời gian tính toán. Khi quay lại SMC, switch smc-test rồi áp dụng đúng stash
đã lưu (xem `git stash list` trước), không pop nhầm stash khác.
