# MPC phản hồi độc lập — 2026-10-24

Thay đổi thuộc branch `mpc-test`. Đã chạy vòng tròn ban đầu và đang retune từng
khớp trên tay thật; chưa kiểm chứng vòng tròn với bộ retune mới nhất.
Profile mới: `src/gim_arm_controller_mpc/config/mpc_circle_independent_feedback.yaml`.
Profile MPC coupled cũ còn trong source; file kết quả cũ có thể đã được dọn.

Elbow/id2 đã retune trên tay thật ngày 2026-10-24 với QE=400, QI=4096,
QV=0.04, R=0.037636, RD=0.72, slew=6.5 Nm/s. Đã lưu vào profile independent
ở trên và kiểm chứng các bài 10°,20°,30°,40°; chưa chạy vòng tròn với bộ mới.
Chi tiết tại `results_new/mpc/elbow_independent_retune/README.md`.
Shoulder/id1 đã retune cùng ngày: QE=4000, QI=16000, QV=0.5,
R=0.0005880625, RD=0.1, slew=50 Nm/s. Hoàn thành các bài 3°,10°,20°,30°,40°.
Chi tiết tại `results_new/mpc/shoulder_independent_retune/README.md`, gồm cả một
dao động 3.33° lúc handoff cần tiếp tục theo dõi. Đã lưu chung profile trên;
base còn giữ bộ trước.

Tuner MPC kiểm tra đứng yên theo cửa sổ 0.3s của vị trí/vận tốc để giảm báo
nhầm do vận tốc khớp đang giữ position nhiễu. Vẫn kiểm tra vận tốc tức thời
khớp đang tune, trôi HOME, dữ liệu mới và giới hạn URDF. Không lọc tín hiệu
phản hồi MPC; gate của runner vòng tròn vẫn giữ cách kiểm tra cũ.

## Tune lại từng khớp

Terminal hiện mặc định đọc `mpc_circle_independent_feedback.yaml`, in rõ
`feedback_model_mode=independent` và lưu kết quả ở `results_new/mpc/joint_trials`.
Lệnh `save` ghi đúng file này, nên bộ sau tune cũng chính là bộ chạy vòng tròn.
Không cần chỉnh mode trước mỗi bài. Profile coupled cũ chỉ được dùng nếu
truyền đường dẫn đó một cách tường minh.

Thứ tự elbow/id2 -> shoulder/id1 -> base/id0. Dừng node vòng tròn sau khi đã
trả position. Đổi launch phần cứng sang `torque_joint:=elbow` ở đúng HOME cơ khí,
rồi mở terminal khác:

```bash
cd ~/GIM_Arm_3_DOF
bash tools/tune_mpc_terminal.sh \
  --joint elbow \
  --params-file src/gim_arm_controller_mpc/config/mpc_circle_independent_feedback.yaml \
  --log-dir results_new/mpc/joint_trials \
  --prestart-timeout 12
```

Tuner tự source ROS/install và đặt domain 42. Prompt hỗ trợ:

```text
show
test 3 6 2
```

Bài trên đi thêm 3° từ HOME, 6 giây mỗi chiều, giữ 2 giây mỗi đầu. Tuner tự
switch và trả position khi xong; hai khớp còn lại giữ position. Chỉ tăng biên độ
sau khi đọc kết quả và kiểm tra HOME. Đổi khớp cần restart hardware với đúng
torque_joint; không dùng launch all để tune độc lập từng khớp.

Trọng số thực ban đầu của elbow trước đợt retune (sau chuẩn hóa của factory):

```text
QE=400  QI=256  QV=0.04  R=0.037636  RD=0.36
slew=6.5 Nm/s, Np=40, Nc=10
```

Đây là mốc, chưa phải bộ retune mới. Terminal nhận trọng số thực:

```text
set QE QI QV R RD
qe VALUE
qi VALUE
qv VALUE
r VALUE
rd VALUE
slew VALUE
save
```

QE tăng ưu tiên vị trí; QI tăng ưu tiên loại sai lệch tích lũy; QV tăng phạt
sai số vận tốc nhưng có thể nhạy hơn với nhiễu vận tốc. R tăng làm phản hồi
tiết chế hơn; RD tăng làm thay đổi torque đắt hơn, có thể giảm rung nhưng tăng
trễ. Slew là ràng buộc cứng, khác RD. Giữ horizon và alpha cố định trong các
lượt đầu để đánh giá thay đổi từng trọng số. Sau tune từng khớp cần chạy lại
vòng tròn vì các tư thế và tương tác cơ khí khác bài độc lập.

Lượt vòng tròn independent ban đầu đã hoàn thành: RMS TRACK
[base, shoulder, elbow]=[0.912,0.600,0.718] độ; vấn đề lớn nhất là base ở RETURN
với RMS/MAX 4.249/11.270 độ. Mọi tick CONTROL trong chuyển động chấp nhận nghiệm;
không coi sai số bám này là solver thất bại. Báo cáo nằm tại
`results_new/mpc/independent_baseline_run01_analysis`.

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
