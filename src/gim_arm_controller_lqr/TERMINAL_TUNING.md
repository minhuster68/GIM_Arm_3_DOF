# Tune LQR từng khớp: elbow → shoulder → base

Làm việc trong `/home/mirabo/GIM_Arm_3_DOF` trên branch `lqr-test`.
Chuyển thuật toán bằng branch trong cùng thư mục; bộ PID đã tune nằm trên `pid-test`.
Dùng `git switch pid-test` hoặc `git switch lqr-test` sau khi lưu thay đổi đang làm.
Sau khi đổi branch, build lại các package của thuật toán và source `install/setup.bash`.

Terminal mới: `tools/tune_lqr_terminal.sh`. Terminal quỹ đạo đầy đủ cũ
`tools/tune_lqr.py` vẫn được giữ để thử circle/R/A sau khi tune từng khớp.

## Luật điều khiển và tham số

```text
e = q - q_ref
x = [integral(e), e, qd - qd_ref]
tau_ff = inverse_dynamics(runtime URDF, q_ref, qd_ref, qdd_ref)
tau_ff_base *= base_ff_alpha
tau_fb_i = -Ki_i*integral(e_i) - Kp_i*e_i - Kd_i*(qd_i-qd_ref_i)
tau_raw = tau_ff + tau_fb
tau_cmd = clip_and_slew_limit(tau_raw)
```

Profile phần cứng dùng `feedback_model_mode: independent`. Mỗi khớp giải DARE
riêng trên [integral(e_i),e_i,ed_i], với quán tính M_ii. K có 3 hàng/9 cột,
nhưng mỗi hàng chỉ có 3 hệ số của chính khớp đó. FF vẫn dùng inverse dynamics
đầy đủ tại tham chiếu. Q phạt sai số tích phân, vị trí và vận tốc.
Đánh giá tracking dùng RMS và max sai số vị trí, không gộp sai số vận tốc.
Có tích phân nên đây là LQR/LQI.
K được tính trước theo HOME → góc thử → HOME ở 10 Hz và nội suy theo thời
gian quỹ đạo. Không giải Riccati trong vòng gửi torque.

Profile mặc định `config/lqr_joint_tuning.yaml` dùng `tau_scale=1` và
`base_ff_alpha=2.2`. Ban đầu Q giữ bộ đã tune coupled; tau_penalty_scale tăng từ512
lên8192 vì bộ independent với R cũ không qua kiểm tra robot ghép ZOH100Hz.
Elbow đã tune trên tay thật ngày 2026-10-24: QE=2000, QI=76800, QV=1, R=160,
slew=5 Nm/s; kiểm tra riêng 3–40°. Shoulder đã tune riêng 3–40°: QE=40000, QI=4800, QV=2, R=5.12, slew=20.
Base đã tune riêng +3° đến+40° và−10°: QE=6400,QI=2400,QV=1,R=10,slew=5,alpha=2.2.
Xem `results_new/lqr/base_independent_retune/README.md`.
Bộ trong `lqr_joint_tuning.yaml` chưa qua kiểm tra full-plant ba torque;
chưa dùng profile này chạy circle. Xem báo cáo shoulder trong results_new/lqr/shoulder_independent_retune/. Xem `results_new/lqr/elbow_independent_retune/README.md`.
Các snapshot trong
results_new/lqr/params_tuned.yaml giữ bộ coupled cũ để so sánh.

Các vector YAML theo thứ tự `[base, shoulder, elbow]`:

```text
QI = 1 / max_int_e²
QE = 1 / max_e²
QV = 1 / max_de²
R  = tau_penalty_scale / max_tau²
```

Baseline `max_de=[2,2,2]` rad/s cho `QV=[0.25,0.25,0.25]`.
Giảm max_de làm tăng QV; max_de là thang trọng số, không phải giới hạn cứng.

`qe`, `qi`, `qv`, `r` nhập trực tiếp phần tử đường chéo Q/R của khớp đang chọn;
terminal đổi sang các tham số Bryson tương đương để lưu YAML.
`max_tau` trong công thức R là thang chuẩn hóa trọng số, không đổi trần
torque. Trần lệnh vẫn là effort URDF × tau_scale: `[5,40,5]` Nm với profile này.
K phản hồi độc lập; chỉ output của khớp đang tune được gửi tới driver ở torque
mode. Hai khớp còn lại giữ position tại góc chụp khi switch. Mô hình cơ khí thật
vẫn có ghép khớp; cần kiểm chứng lại khi chạy cả ba khớp.

## Phạm vi kiểm tra ổn định khi tune đơn khớp

Terminal xác minh `torque_joint` từ URDF runtime rồi tự đặt
`isolated_joint_index` cho riêng lượt thử independent. Hai khớp khác giữ
position: mô hình kiểm tra dùng quán tính M_ii của khớp đang tune.
Nếu khớp giữ lệch quá 1° trong các pha chạy/giữ, bài thử dừng và trả position.
Đây là giả thiết khớp giữ đủ cứng, cần kiểm chứng lại khi đổi sang ba torque.

Tham số index chỉ ghi trong parameters.yaml/trial.json của lượt thử, không
được save vào profile trọng số. Quỹ đạo đầy đủ mặc định index=-1 và vẫn
kiểm tra robot ghép. Factory từ chối dùng index cô lập cho quỹ đạo không
phải diagnostic_hold, tránh dùng nhầm snapshot đơn khớp chạy circle.
Bộ tốt trong bài cô lập có thể không qua mô hình ghép: không xem kết quả
tune từng khớp là xác nhận chạy circle cả ba khớp.

## Chuẩn bị

Các terminal phần cứng và lệnh cần nạp môi trường LQR:

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
```

Build sau khi sửa source (đã build khi thêm terminal):

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  gim_arm_description gim_arm_hardware gim_control gim_arm_controller_lqr
source install/setup.bash
```

Nếu PID còn chạy effort, trả về position trước rồi dừng node PID:

```bash
ros2 control switch_controllers --strict --activate-asap \
  --deactivate gim_arm_effort_controller --activate gim_arm_group_controller
```

Nếu position đã active thì bỏ qua lệnh switch. Sau đó dừng launch phần cứng
cũ để mở plugin LQR mới có bàn giao torque liên tục.

## Bắt đầu elbow

Đặt/đỡ tay tại đúng tư thế zero trước startup. Terminal phần cứng:

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 set_zero_on_startup:=true \
  zero_method:=software torque_joint:=elbow
```

Terminal lệnh: chờ JTC active rồi về HOME. Chờ goal thành công trước test.

```bash
ros2 control list_controllers
ros2 action send_goal --feedback \
  /gim_arm_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [base_joint, shoulder_joint, elbow_joint], points: [{positions: [0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0], time_from_start: {sec: 8}}]}}"
```

Terminal tune tự source ROS/install và đặt domain 42:

```bash
cd /home/mirabo/GIM_Arm_3_DOF
bash tools/tune_lqr_terminal.sh --joint elbow --interface can0
```

Trong terminal tune, bắt đầu bằng bộ số baseline:

```text
show
test 3 6 3
```

`test 3 6 3` đi thêm 3 độ từ góc hiện tại trong 6 s, giữ 3 s, quay về
HOME trong 6 s rồi giữ thêm 3 s. K dựng cho đúng góc thử; URDF đọc từ
`/controller_manager`, kiểm đúng lựa chọn torque trước khi chuyển mode.

Trước quỹ đạo, PID driver bàn giao torque giữ đo được sang LQR trong 1 s.
LQR giữ HOME với tích phân. Terminal xác nhận đứng yên bằng cửa sổ mẫu mới
dài ít nhất 0.3 s: biên độ vị trí mỗi khớp ≤0.005 rad, vận tốc trung bình
từ vị trí và trung bình tín hiệu driver đều ≤0.05 rad/s; khoảng cách mẫu
không vượt 0.05 s. Vẫn yêu cầu sai lệch HOME
≤min(0.02,max_transition_error_rad) rad liên tục 0.3 s.
Điều này tránh chặn nhầm bởi một mẫu vận tốc nhiễu; vận tốc dùng trong
feedback vẫn là tín hiệu driver, chưa thêm LPF/Kalman.

Terminal còn nghe CAN thụ động: phải thấy frame chọn torque mode và sau đó
lệnh torque hữu hạn cho đúng node đang tune. Thiếu bằng chứng trong 0.5 s
thì trả position, dù ROS báo switch thành công. Đây là xác nhận frame gửi
trên bus, không phải ACK trạng thái bên trong driver.
Chờ ổn định tối đa 6 s sau bàn giao. Đổi bằng `--prestart-timeout 8` nếu cần;
không tự chốt lại HOME tại vị trí đã trôi.

Các lệnh đổi trọng số riêng khớp:

```text
qe 200
test 3 6 3
qi 50
test 3 6 3
qv 1
test 3 6 3
r 10
test 3 6 3
save
```

Đây là ví dụ cú pháp, không phải bộ trọng số đã tune. QE tăng hoặc R giảm
làm feedback vị trí mạnh hơn; QI tăng làm ưu tiên loại sai số tích lũy mạnh
hơn. QV phạt sai số vận tốc trong tối ưu. Thay một trọng số mỗi lần và
so sánh sai số vị trí, dao động, saturation.
`set QE QI QV R` đổi cả bốn cùng lúc, không phải Kp/Ki/Kd.
Cú pháp cũ `set QE QI R` vẫn dùng được và giữ nguyên QV hiện tại.
`slew VALUE` chỉnh tốc độ đổi torque khớp đó [Nm/s], giá trị >0.
`alpha VALUE` chỉ nhập khi chọn base; không nhân feedback -Kx.
Lệnh chỉnh số áp dụng vào test tiếp theo; `save` mới ghi YAML.

## Shoulder và base

Sau khi chọn được bộ elbow:

```text
save
next
```

Terminal chuyển shoulder. Dừng launch cũ, đặt/đỡ tay đúng zero và launch
lại với `torque_joint:=shoulder`; về HOME bằng goal trên, rồi `test 3 6 3`.
Giữa các lượt của cùng một khớp không cần khởi động lại phần cứng.

Lặp `save`, `next`, đổi launch sang `torque_joint:=base`, về HOME và tune base.
Để mở lại từ khớp đang tune dở: `--joint shoulder` hoặc `--joint base`.
Trong terminal chỉ có một khớp nhận torque; bài circle cả ba khớp cần được
đánh giá lại với các trọng số đã chọn sau khi hoàn thành thứ tự này.

## Chạy vòng tròn bằng bộ trọng số đã tune

**Trạng thái 2026-10-24:** giữ nguyên `config/lqr_joint_tuning.yaml` cho tune
cô lập. Bộ đó không đạt kiểm tra mô hình robot ghép. Circle dùng riêng
`config/lqr_circle_hardware.yaml`, dùng mô hình thiết kế `free_torque`,
giữ Q và tăng R 8 lần. Đã qua kiểm tra cả tay, lịch gain/nội suy, trễ thêm
10 ms và hoàn thành một vòng trên tay thật. Các guard ổn định vẫn bật.
TRACK RMS base/shoulder/elbow≈2.57°/1.94°/1.21°; chưa chốt tối ưu độ bám.
Xem `results_new/lqr/circle_readiness/README.md`.

Thoát terminal từng khớp bằng `quit`, chờ position active rồi dừng launch cũ.
Đặt/đỡ tay đúng zero và khởi động lại hardware với cả ba khớp torque:

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 set_zero_on_startup:=true \
  zero_method:=software torque_joint:=all
```

Terminal khác nạp ROS/install và domain 42 như phần Chuẩn bị. Chạy goal HOME
ở trên và chờ thành công, rồi mở terminal quỹ đạo đầy đủ:

```bash
python3 tools/tune_lqr.py \
  --params-file src/gim_arm_controller_lqr/config/lqr_circle_hardware.yaml \
  --trajectory-shape circle --log-dir results_new/lqr/circle_trials
```

Nhập `show`, rồi `test circle`. Terminal này nhận `test circle`, không nhận
cú pháp `test DEG MOVE HOLD` của terminal từng khớp. Bài chạy một vòng:
APPROACH 16 s → TRACK 27 s → RETURN 16 s → HOLD 3 s.
Ba khớp cùng dùng tau_ff + LQR, với Q/R, alpha, tau_scale và slew từ YAML.
Terminal đọc lựa chọn torque và thời gian bàn giao từ controller_manager;
gửi torque giữ HOME trước switch, chờ hết bàn giao rồi đứng yên 0.3 s với
tích phân hoạt động trước khi bắt đầu quỹ đạo. HOME ban đầu được giữ nguyên.
Nếu không ổn định sau 6 s, bài không chạy và terminal trả về position.

Kết thúc hoặc Ctrl-C trong test: tự trả về position trước khi đóng node,
ghi CSV và YAML tham số của lượt thử, rồi mở ba đồ thị tracking.
Log nằm trong `results_new/lqr/circle_trials/`. Đóng đồ thị để trở lại prompt.

## Log và ngắt bài thử

Mỗi lượt lưu riêng trong `results_new/lqr/joint_trials/lqr_<joint>_<timestamp>/`:

- `tracking.csv`: GRAVITY/APPROACH/TRACK/RETURN/HOLD, q/q_ref, tau_ff/tau_fb,
  các thành phần vị trí/tích phân/vận tốc, saturation và chỉ số gain.
- `model.urdf`: đúng URDF phần cứng dùng trong lượt đó.
- `parameters.yaml`, `trial.json`: tham số, HOME, góc đích và điều kiện giữ HOME.
- Ba PNG tracking cho cả khớp đang tune và hai khớp đang giữ.

Trong giây bàn giao, CSV tau là lệnh PC trước blend của plugin. Sau khi
bắt đầu APPROACH, blend đã kết thúc. Khi kết thúc/ABORT/Ctrl-C, terminal
chuyển về position trước khi dừng node torque và ghi log/mở đồ thị.
Nếu phản hồi switch chưa xác nhận, giữ node phát torque và thử trả về
position tiếp; không đóng publisher khi effort vẫn active.

`--no-show` chỉ lưu hình. Thử cú pháp không mở ROS/CAN hoặc ghi YAML:

```bash
bash tools/tune_lqr_terminal.sh --joint elbow --dry-run
```
