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
khớp đang tune, trôi HOME, dữ liệu mới và giới hạn URDF. Kiểm tra đứng yên
dùng vận tốc thô, độc lập với bộ lọc phản hồi.
Từ 2026-10-25, profile independent bật Kalman vận tốc bậc 2 trước phản hồi MPC
cho cả ba khớp; xem phần dưới. Bộ lọc chưa được kiểm chứng trên tay thật với MPC.

## Kalman vận tốc bậc 2 — 2026-10-25

```text
/joint_states.velocity thô
    -> 3 bộ Kalman riêng [vận tốc, gia tốc]
    -> vận tốc sau lọc
    -> e_dot = vận tốc sau lọc - qd_ref
    -> trạng thái đầu vào QP independent của MPC
```

Chỉ đo vận tốc; gia tốc là trạng thái ước lượng nội bộ để dự đoán vận tốc ở
mẫu tiếp theo. Gia tốc này không thay `qdd_ref` và không đưa vào feedforward.
Góc đo, q/qd/qdd tham chiếu và torque đầu ra không được lọc. QP vẫn dùng mô hình
sai số hiện có, không bổ sung trạng thái Kalman vào prediction horizon.

Đã bật trong `src/gim_arm_controller_mpc/config/mpc_circle_independent_feedback.yaml`:

```yaml
# base/id0, shoulder/id1, elbow/id2
velocity_filter_type: kalman2
velocity_kalman_measurement_std: [0.03, 0.01, 0.04]  # rad/s
velocity_kalman_jerk_std: [180.0, 60.0, 240.0]       # rad/s³
velocity_kalman_reset_gap_s: 0.05
```

Đây là bộ số khởi đầu dùng cùng cách lọc LQR, chưa phải bộ nhiễu nhận dạng
riêng cho MPC. Trọng số Q/R/RD, horizon, precompute, alpha feedforward và các
giới hạn torque/slew của MPC được giữ nguyên. Profile coupled và mô phỏng cũ
mặc định `velocity_filter_type: none`; phải truyền đúng profile independent
khi launch vòng tròn. Terminal tune mặc định dùng profile independent này
nên cũng bật lọc, còn kiểm tra đứng yên vẫn lấy tín hiệu thô.

Mô hình nhiễu là jerk ngẫu nhiên không đổi trong một khoảng mẫu:
`F=[[1,dt],[0,1]]`, `G=[dt²/2,dt]ᵀ`, `Q_filter=jerk_std²*G*Gᵀ`,
`R_filter=measurement_std²`. Đây là Q/R nhiễu, khác Q/R chi phí tối ưu MPC.
Tăng measurement std hoặc giảm jerk std làm lọc mạnh hơn nhưng có thể tăng
trễ; Kalman không bảo đảm giảm mọi dải dao động. Chưa thay trọng số MPC cùng
lúc để có thể so ảnh hưởng của việc lọc.

Bộ lọc cập nhật một lần theo mỗi JointState mới, dùng dt từ header; header
bằng 0 thì dùng thời gian nhận monotonic. Mẫu trùng không cập nhật lại hoặc
làm mới timeout. Khoảng mất mẫu trên 50 ms, hoặc thời gian quay ngược, sẽ
khởi tạo lại bằng vận tốc đo và gia tốc 0. Timer dùng số đo cũ không chạy
lọc lại; reset tích phân/warm-start không reset bộ lọc mỗi tick HOLD.
Không có luồng riêng hay chờ một chu kỳ để xuất kết quả.

CSV chính khi bật lọc có `qd_raw_*` là driver velocity, `qd_filtered_*` và
`qd_*` là tín hiệu đưa vào MPC. Plot vận tốc chồng cả thô, sau lọc và tham
chiếu; sai số vận tốc cũng có đường thô để so sánh. Các log cũ vẫn đọc được.
Đồ thị vị trí/torque giữ riêng. Không kết luận cơ cấu bớt rung chỉ từ đường
sau lọc; cần so vị trí, vận tốc thô, torque và solver giữa hai lượt cùng quỹ đạo.

Chạy bằng các lệnh ở phần cuối với tên log mới `circle_independent_kalman_run01.csv`.
Để thử không lọc: đổi `velocity_filter_type: none`, restart node MPC và dùng
tên log khác. Tham số lọc được nạp khi khởi tạo node.

## Tune lại từng khớp

Đợt tune coupled/no-Kalman elbow ngày 2026-10-28 được ghi ở
[ELBOW_RETUNE_20261028.md](results_new/mpc/ELBOW_RETUNE_20261028.md).
Phiên này dùng file coupled tường minh, khác mặc định independent bên dưới.

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

QI/QV có thể bằng 0; QE/R phải dương và RD không âm. Nếu gate vận tốc thô
liên tục báo chưa đứng yên dù vị trí ổn định, có thể thêm
`--stationarity-mode window`. Chế độ này xét mẫu mới liên tục trong 0.3s,
vận tốc trung bình <=0.05 rad/s và biên vị trí <=0.3° cho cả ba khớp;
không lọc tín hiệu đưa vào MPC. Nếu tư thế khớp giữ thay đổi trong precompute,
tuner yêu cầu chờ position ổn định rồi thử lại trước khi chuyển torque.

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

Tích hợp Kalman ngày 2026-10-25: build `gim_control` và `gim_arm_controller_mpc`
thành công; 183 test qua, gồm hồi quy solver/independent, bộ lọc, đường tín hiệu
vào QP, CSV/plot và bài đổi controller ROS với tay giả lập. Trọng số và mô hình
MPC được đối chiếu bằng nhau khi bật/tắt lọc. Chưa chạy hay đánh giá ổn định
vòng kín trên tay thật với bộ lọc; các bài kiểm tra này không chứng minh giảm rung.

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
  log_file:=results_new/mpc/circle_independent_kalman_run01.csv
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
ros2 run gim_control plot_joint_tracking results_new/mpc/circle_independent_kalman_run01.csv --show
```

Không ghi đè log; đổi run01 khi chạy lại. Muốn so cấu hình ghép thì dùng
`results_new/mpc/params_tuned.yaml` và tên CSV khác. Chỉ so RMS/MAX ở cùng pha
APPROACH/TRACK/RETURN, cùng thời lượng; theo dõi cả vận tốc, torque, solver và
thời gian tính toán. Khi quay lại SMC, switch smc-test rồi áp dụng đúng stash
đã lưu (xem `git stash list` trước), không pop nhầm stash khác.
## Chẩn đoán các thành phần torque phản hồi

Trên `mpc-test`, CSV mới ghi thêm các vector (Nm) cho từng khớp:

| Cột | Ý nghĩa |
|---|---|
| `tau_mpc_i_free` | Đóng góp của trạng thái tích phân sai số trong nghiệm QP chưa áp ràng buộc |
| `tau_mpc_q_free` | Đóng góp của sai số vị trí trong nghiệm QP chưa áp ràng buộc |
| `tau_mpc_qd_free` | Đóng góp của sai số vận tốc trong nghiệm QP chưa áp ràng buộc |
| `tau_mpc_prev_free` | Đóng góp của lệnh phản hồi trước đó qua cost thay đổi torque |
| `tau_mpc_fb_free` | Tổng bốn thành phần trên, tức bước điều khiển đầu của QP tự do |
| `tau_mpc_correction` | Phản hồi thực tế trừ nghiệm QP tự do: gồm ảnh hưởng ràng buộc torque/slew và sai số solver, projection hoặc fallback |
| `tau_mpc_total_clip` | Hiệu chỉnh do kẹp torque tổng sau khi cộng feedforward |

Quan hệ cộng kiểm tra được ở từng mẫu `CONTROL`:

```text
tau_fb = tau_mpc_i_free + tau_mpc_q_free + tau_mpc_qd_free
       + tau_mpc_prev_free + tau_mpc_correction
tau_controller = tau_ff + tau_fb + tau_mpc_total_clip
```

Đây là một cách quy chiếu để chẩn đoán MPC, không phải các nhánh PID độc lập.
Các thành phần `_free` được tính từ cùng Hessian/cost/mô hình của QP đang chạy:
`U_free = -H^-1 (F*x + P*u_previous)`, rồi lấy lệnh đầu tiên.
Khi có ràng buộc, phần đóng góp thực tế không có cách tách cộng duy nhất.
Vì vậy không được diễn giải riêng `_free` như torque thực sự do một nhánh phát
ra sau giới hạn, hoặc coi `tau_mpc_correction` chỉ do slew. Đối chiếu thêm
`solver_accepted`, `constraint_violation` và giới hạn tích phân.

Trạng thái tích phân dùng để tính torque là giá trị **trước** bước cập nhật
tích phân của tick; cột `integral` có sẵn ghi giá trị sau cập nhật/kẹp.
Heartbeat ở runner có thể tạo chênh lệch cỡ 1e-6 Nm giữa torque controller và
cột torque gửi. Mẫu giữ lệnh do thiếu trạng thái mới ghi NaN cho các thành phần
chẩn đoán, không giả là đã chạy lại MPC.

Với `precompute_prediction: true`, các ma trận gain chẩn đoán được tính trước
khi chạy quỹ đạo; không giải thêm QP trong control tick. Phần chẩn đoán không
thay đổi nghiệm solver, warm start, tích phân, gain hay giới hạn điều khiển.

Sau khi build `gim_control` và `gim_arm_controller_mpc`, restart node và ghi
CSV mới. `plot_joint_tracking` tự thêm các đường thành phần vào hình torque
riêng của từng khớp (`*_torque.png`); vị trí/sai số vị trí vẫn ở hình riêng.
Với `--show`, cuộn chuột để zoom và bấm đường trong chú giải để ẩn/hiện:

```bash
ros2 run gim_control plot_joint_tracking \
  results_new/mpc/fine_tune/circle_run04.csv --show
```

CSV cũ vẫn plot được, nhưng không chứa các thành phần mới. Các nhánh khác,
bao gồm `mpc-eso-test`, chưa được cập nhật bởi thay đổi này.
