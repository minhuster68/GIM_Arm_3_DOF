# Tune cascade PID trên tay thật

Bài thử có thể cô lập một khớp: khớp đó dùng mô-men, hai khớp còn lại giữ
bằng position mode của motor. Một khớp đi theo quỹ đạo bậc 5 từ HOME
tới góc đích, giữ đích rồi về HOME; hai khớp còn lại có góc đặt bằng 0.
HOME được chụp từ `/joint_states`, vì vậy đưa tay về `[0,0,0]` trong chế độ
position trước khi khởi động bài thử. Không khởi động lại launch phần cứng
giữa các lần tune để giữ nguyên mốc zero phần mềm.
Có thể thử shoulder và elbow đồng thời với `shoulder_elbow`; base giữ bằng
position mode của driver. Hai khớp dùng cùng thời gian đi/giữ/về, với góc đích riêng.
Chọn `joint:=all` để cả ba khớp chạy cùng một quỹ đạo đi/giữ/về; đặt riêng
`base_deg`, `shoulder_deg`, `elbow_deg` và dùng phần cứng `torque_joint:=all`.

Trong mọi pha đi, giữ đích, quay về và giữ HOME:

```text
tau_ff = inverse_dynamics_URDF(q_ref, qd_ref, qdd_ref)
e_q = q_ref - q
qd_cmd = qd_ref + kpp * e_q
e_v = qd_cmd - qd
tau_p = kvp * e_v
tau_i = kvi * integral(e_v)
tau_fb = tau_p + tau_i
tau_cmd = clip(tau_ff + tau_fb, -tau_limit, tau_limit)
```

Feedforward gồm cả thành phần quán tính, vận tốc và trọng lực. PID hoạt động
liên tục cùng feedforward. Trước khi bắt đầu quỹ đạo, cùng cascade PID giữ
HOME với phần I được reset mỗi tick để tránh tích lũy khi effort controller
còn inactive. Sau khi bắt đầu, I được giữ liên tục qua các pha, kể cả HOLD_HOME.
Không cần dùng service hand-guiding cho bài thử này.

## Chuẩn bị

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select gim_control gim_arm_controller_pid --symlink-install
source install/setup.bash
```

Để tune elbow với base và shoulder giữ bằng motor, launch phần cứng cần chạy
với `torque_joint:=elbow`. Tham số này được đọc khi khởi tạo phần cứng,
không đổi bằng `ros2 param set` trong phiên đang chạy.

Đặt và kê đỡ tay tại đúng tư thế zero trước khi khởi động mới:

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 set_zero_on_startup:=true zero_method:=software \
  torque_joint:=elbow
```

Khi switch sang effort, base và shoulder vẫn ở position mode, chốt góc tại
thời điểm switch; chỉ elbow nhận `tau_ff + tau_fb`. Độ cứng phụ thuộc vòng
vị trí và khả năng motor, không tương đương khóa cơ khí. Node vẫn tính và
ghi vector mô-men đủ ba khớp từ mô hình; phần mô-men base/shoulder trong CSV
không được gửi thành lệnh torque cho hai driver đang giữ vị trí.

Nếu launch cũ dùng `torque_joint:=all`, chuyển về position, dừng node PID rồi
kê đỡ tay và khởi động lại launch phần cứng theo lệnh trên. Sau khi đã chọn
khớp, giữ launch phần cứng chạy giữa các lần sửa gain để giữ mốc zero.
Nếu node PID cũ còn chạy, chuyển controller về position rồi Ctrl-C node PID
cũ trước khi khởi động node mới.

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate gim_arm_group_controller
```

Nếu controller position đã active thì bỏ qua lệnh switch trên. Đưa tay về zero:

```bash
ros2 action send_goal \
  /gim_arm_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [base_joint, shoulder_joint, elbow_joint], points: [{positions: [0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0], time_from_start: {sec: 6}}]}}"
```

## Một lần thử elbow 30 độ

Terminal node PID:

```bash
ros2 launch gim_control pid_joint_tuning.launch.py \
  joint:=elbow target_deg:=30 \
  move_time:=6 hold_time:=3 return_time:=6 \
  log_file:=results/pid_elbow_30_run01.csv
```

Chờ log `WAIT -> READY_PID` với HOME gần `[0,0,0]`. Trong terminal khác:

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_group_controller \
  --activate gim_arm_effort_controller

ros2 param set /cascade_pid_controller autostart true
```

Trình tự: APPROACH 6 s (0 tới 30 độ) → TRACK giữ 3 s → RETURN 6 s → HOLD_HOME.
Sau khi về HOME, cùng cascade PID vẫn giữ tay; không tự chuyển sang bộ gain
giữ mềm khác. Khi kết thúc hoặc cần dừng, chuyển về position trước:

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate gim_arm_group_controller
```

Rồi Ctrl-C terminal node PID để ghi CSV và vẽ:

```bash
ros2 run gim_control plot_joint_tracking results/pid_elbow_30_run01.csv --show
```

## Thử đồng thời shoulder ID 1 và elbow ID 2

Dùng PID + inverse dynamics với gain hiện có trong YAML. Base ID 0 giữ
position tại góc chốt lúc switch, không nhận lệnh torque từ node PID.
Đây là giữ bằng motor, không phải khóa cơ khí. CSV vẫn ghi torque tính toán
của base; torque này không được áp dụng cho driver base.

Nếu hệ đang chạy effort, switch về position bằng lệnh ở phần trên, rồi dừng
node PID cũ. Kê đỡ tay, dừng launch phần cứng cũ để đổi lựa chọn torque.
Build và source lại workspace ở các terminal:

```bash
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
colcon build --packages-select gim_control gim_arm_controller_pid --symlink-install
source install/setup.bash
```

Terminal phần cứng: đặt tay ở đúng tư thế zero trước khi launch này thiết lập
lại mốc zero phần mềm. CAN vẫn dùng interface/bitrate đang kết nối được.

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 set_zero_on_startup:=true zero_method:=software \
  torque_joint:=shoulder_elbow
```

Chờ position controller active, đưa tay về zero bằng goal `[0,0,0]` ở phần
Chuẩn bị nếu cần. Terminal PID:

```bash
ros2 launch gim_control pid_joint_tuning.launch.py \
  joint:=shoulder_elbow shoulder_deg:=15 elbow_deg:=30 \
  move_time:=6 hold_time:=3 return_time:=6 \
  log_file:=results/pid_shoulder_elbow_run01.csv
```

`shoulder_deg` và `elbow_deg` là góc tuyệt đối theo mốc zero ROS, dùng riêng
cho bài thử cặp khớp; `target_deg` vẫn dành cho bài thử một khớp.
Launch từ chối bắt đầu nếu vận tốc đo của bất kỳ khớp nào vượt `0.05 rad/s`
khi bật autostart: ABORT và về gravity compensation, cần chuyển lại position
để kiểm tra. Điều kiện này phát hiện tay đang chuyển động lúc bắt đầu;
không thay thế việc kiểm tra ổn định của bộ gain.
Chờ `WAIT -> READY_PID` với HOME gần `[0,0,0]`. Terminal lệnh:

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_group_controller \
  --activate gim_arm_effort_controller
ros2 param set /cascade_pid_controller autostart true
```

Hai khớp đi đồng thời 6 s tới `[0,15,30]` độ, giữ 3 s, quay về HOME trong
6 s rồi giữ HOME bằng cùng cascade PID. Khi kết thúc, switch về position
trước khi Ctrl-C node PID để ghi CSV:

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate gim_arm_group_controller
```

Sau Ctrl-C terminal PID:

```bash
ros2 run gim_control plot_joint_tracking results/pid_shoulder_elbow_run01.csv --show
```

Xem đồ thị `q/qref`, sai số bám và `tau_ff/tau_fb` của shoulder và elbow;
đồ thị base dùng kiểm tra khả năng giữ vị trí. Đổi tên CSV cho mỗi lần thử.
Muốn lặp lại cùng cấu hình, giữ launch phần cứng chạy; chỉ chạy lại node PID.

### Profile gain thấp sau lần thử rung

Log `results/pid_shoulder_elbow_run02.csv` cho thấy tại đầu APPROACH,
shoulder đã có vận tốc `-0.833 rad/s`; feedback P yêu cầu `50.915 Nm`,
bị kẹp ở `14 Nm`. Sau `0.03 s`, sai số elbow là `0.1435 rad` và kích hoạt
ABORT (`max_transition_error_rad=0.10`). I của shoulder bằng 0 lúc đó.
Log cũ không ghi pha giữ trước quỹ đạo nên chưa xác định được thời điểm
rung bắt đầu. Gain quy đổi tương đương về đơn vị chưa đảm bảo ổn định
khi chạy vòng feedback trên PC.

File `config/pid_hardware_soft.yaml` là bộ thử gain thấp hơn, chưa được xác
nhận ổn định trên phần cứng: shoulder `[6,1.41,0]`, elbow `[8,0.20,0]`
theo thứ tự `kpp/kvp/kvi`. Feedforward, dấu CAN và trần mô-men giữ nguyên.
Profile gốc vẫn lưu bộ gain quy đổi để đối chiếu. Thử biên độ nhỏ hơn:

```bash
ros2 launch gim_control pid_joint_tuning.launch.py \
  joint:=shoulder_elbow shoulder_deg:=5 elbow_deg:=5 \
  move_time:=8 hold_time:=3 return_time:=8 \
  params_file:=src/gim_arm_controller_pid/config/pid_hardware_soft.yaml \
  log_file:=results/pid_shoulder_elbow_soft_run01.csv
```

Dùng cùng launch phần cứng `torque_joint:=shoulder_elbow`. Chuyển về position,
dừng PID cũ, đưa tay về HOME trước khi chạy node PID mới. Sau khi switch sang
effort, quan sát pha giữ trước quỹ đạo; chỉ bật autostart khi tay đã đứng yên.
Nếu rung khi giữ thì chuyển ngay lại position, dừng PID để lấy log, chưa chạy
quỹ đạo. CSV mới ghi cả pha GRAVITY/READY_PID với `q/qd`, `tau_ff`, `tau_fb`
và saturation; tên GRAVITY trong CSV là trạng thái chờ, vẫn dùng cascade PID
khi `cascade_hold=true`. Lệnh plot ở phần trên cũng đọc được log này.
Trước khi effort controller active, torque trong CSV là lệnh node tính và
publish, chưa phải torque được áp dụng xuống driver. Đối chiếu thời điểm
switch khi đọc pha chờ này.

## Thử đồng thời cả ba khớp

Dùng gain hiện có trong `pid_hardware_soft.yaml`, không thay gain khi đổi
bài thử. Quỹ đạo bậc 5 đi từ HOME tới `[5,5,5]` độ trong 8 s, giữ 3 s,
quay về HOME trong 8 s rồi giữ HOME. Các góc đích là góc tuyệt đối theo
mốc zero ROS, theo thứ tự `[base, shoulder, elbow]`.

Nếu đang chạy bài thử cô lập, chuyển về position trước, dừng PID cũ rồi kê
đỡ tay và dừng launch phần cứng cũ để đổi lựa chọn torque. Build lại:

```bash
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
colcon build --packages-select gim_control gim_arm_controller_pid --symlink-install
source install/setup.bash
```

Source workspace ở các terminal mới. Đặt và kê đỡ tay đúng tư thế zero
trước khi thiết lập lại zero phần mềm. Terminal phần cứng:

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 set_zero_on_startup:=true zero_method:=software \
  torque_joint:=all
```

Với URDF hiện tại cả ba khớp đều được phép vào torque; không có khớp giữ
bằng position mode khi effort controller active. Terminal PID:

```bash
ros2 launch gim_control pid_joint_tuning.launch.py \
  joint:=all base_deg:=5 shoulder_deg:=5 elbow_deg:=5 \
  move_time:=8 hold_time:=3 return_time:=8 \
  params_file:=src/gim_arm_controller_pid/config/pid_hardware_soft.yaml \
  log_file:=results/pid_all_5_run01.csv
```

Chờ `READY_PID` với HOME gần `[0,0,0]`. Terminal lệnh:

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_group_controller \
  --activate gim_arm_effort_controller
```

Quan sát giữ HOME trước; chỉ khi tay đứng yên, không rung rõ, mới chạy:

```bash
ros2 param set /cascade_pid_controller autostart true
```

Khi kết thúc hoặc rung, chuyển lại position trước khi Ctrl-C PID:

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate gim_arm_group_controller
```

Sau Ctrl-C PID, xem độ bám và `tau_ff/tau_fb` của từng khớp:

```bash
ros2 run gim_control plot_joint_tracking results/pid_all_5_run01.csv --show
```

Giữ cùng HOME, góc đích và thời gian khi so các lần thử. Đổi tên CSV cho
mỗi lần chạy. Kết quả tune riêng từng khớp chưa xác nhận đáp ứng khi ba khớp
cùng chuyển động và tác động động lực học lên nhau.

## Chạy lại vòng quét ban đầu

Launch `pid_sweep_hardware.launch.py` dùng lại `solve_waypoints` và
`SmoothSweep` của runner gốc; không dùng bài diagnostic giữ một góc.
Hình vòng quét lấy trực tiếp từ `gim_control/sweep_trajectory.py` và
`shapes.shoulder_sweep`, không thay đổi hình hoặc biên độ. Gain lấy từ
`pid_hardware_soft.yaml` hiện tại. PID và inverse dynamics hoạt động trong
các pha APPROACH, TRACK, RETURN và HOLD_HOME.

Mặc định HOME → điểm đầu trong 16 s → một vòng 27 s → HOME trong 16 s.
Thời gian đưa lên và đưa về tăng gấp đôi so với 8 s ban đầu: cùng vị trí,
vận tốc giảm một nửa và gia tốc giảm còn một phần tư. Tốc độ vòng quét giữ nguyên.
Điểm đầu trên URDF hiện tại khoảng `[42.97,48.30,43.38]` độ; biên độ khớp
trên cả vòng khoảng `[65.6,37.8,37.2]` độ. Đây là biên độ lớn hơn bài thử 5 độ.
Node giải IK và chạy kiểm tra quỹ đạo khi khởi tạo, tắt cache để đọc đúng
hình đang có trong source, chưa bật autostart.

Nếu phần cứng đang chạy với `torque_joint:=all`, giữ launch đó. Nếu còn ở
chế độ cô lập một khớp, chuyển về position, dừng PID và kê đỡ tay rồi khởi
động lại phần cứng theo lệnh `torque_joint:=all` ở mục thử ba khớp.
Trước khi đổi bài thử, chuyển controller về position và dừng PID cũ.
Đưa tay về cùng HOME trong position mode:

```bash
ros2 action send_goal \
  /gim_arm_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [base_joint, shoulder_joint, elbow_joint], points: [{positions: [0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0], time_from_start: {sec: 8}}]}}"
```

Build và source lại workspace để cài launch mới. Terminal PID:

```bash
ros2 launch gim_control pid_sweep_hardware.launch.py \
  params_file:=src/gim_arm_controller_pid/config/pid_hardware_soft.yaml \
  approach_time:=16 return_time:=16 \
  log_file:=results/pid_sweep_run01.csv
```

Chờ báo kiểm tra IK/quỹ đạo đạt và `WAIT -> READY_PID` với HOME đúng.
Terminal lệnh:

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_group_controller \
  --activate gim_arm_effort_controller
```

Khi giữ HOME đứng yên, không rung rõ, bắt đầu:

```bash
ros2 param set /cascade_pid_controller autostart true
```

Khi về HOME hoặc cần dừng, chuyển về position trước rồi Ctrl-C PID:

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate gim_arm_group_controller
```

Xem độ bám khớp và feedforward/feedback:

```bash
ros2 run gim_control plot_joint_tracking results/pid_sweep_run01.csv --show
```

Kiểm tra offline trên mô hình hiện tại: 90/90 điểm IK hội tụ, cách giới hạn
khớp ít nhất `0.266 rad`; tốc độ đỉnh vòng quét `[0.239,0.252,0.264] rad/s`.
Feedforward đỉnh theo khớp khoảng `[0.730,3.374,1.496] Nm`, nằm dưới trần
hiện tại `[1.75,14,1.75] Nm`. Elbow còn khoảng `0.25 Nm` dư địa feedback ở
điểm feedforward lớn nhất; theo dõi saturation khi thử thực tế. Các kiểm tra
mô hình và runner với state giả lập không xác nhận độ bám trên phần cứng.

## Chỉnh gain

File `config/pid_hardware_tuning.yaml` chứa các vector theo thứ tự
`[base, shoulder, elbow]`. Đây là gain khởi đầu cho bài thử ở 100 Hz, chưa
được tune trên tay thật. `tau_scale=0.35` đặt trần `[1.75,14,1.75] Nm` theo URDF;
không nhân feedforward với 0.35.

- `kpp`: vòng P vị trí, đơn vị 1/s.
- `kvp`: phần P của vòng vận tốc, đơn vị Nm/(rad/s).
- `kvi`: phần I của vòng vận tốc, đơn vị Nm/rad. Có thể đặt bằng 0 để bắt đầu
  tune phần P. Sau khi đáp ứng ổn, thêm I để giảm sai số tĩnh.
- `velocity_integral_limit`: giới hạn trạng thái tích phân; tổng mô-men còn
  có saturation và conditional anti-windup.

Mỗi lần thay gain của khớp đang thử, chuyển về position, dừng node PID, sửa
YAML và chạy lại với tên CSV mới. Gain được đọc khi khởi tạo; không dùng
`ros2 param set kpp/kvp/kvi` để tune trong phiên chạy này.
Nếu dùng bản YAML riêng, truyền `params_file:=/duong/dan/pid_tuning.yaml`.

`kvp` ảnh hưởng cả damping lẫn độ cứng tương đương, vì phần P của feedback là
`kvp*kpp*e_q + kvp*(qd_ref-qd)`. Đổi một gain mỗi lần để đánh giá tác động.

### Dùng gain đã tune trong driver làm điểm khởi đầu

Shoulder (CAN ID 1) trong profile hiện tại lấy từ bộ driver đã tune:
`pos_gain=10`, `vel_gain=0.75`, `vel_integrator_gain=1` (lần tune cuối trong CSV driver).
Elbow (CAN ID 2) lấy từ `pos_gain=25`, `vel_gain=0.6`, `vel_integrator_gain=10`.
Hai bộ có cùng cấu trúc cascade P/PI, nên có thể chuyển gain sau khi đổi đơn vị.
Phép đổi giả định driver dùng gain PI tạo torque theo quy ước
[ODrive](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.4/Firmware/odrive-interface.yaml)
và dùng đúng phép quy đổi CAN hiện tại của plugin.

Gọi `Nv` là `gear_ratio` dùng đổi góc khớp thành encoder rev, `Nt` là
`torque_gear_ratio` dùng đổi torque khớp thành torque CAN. Khi dấu vị trí và
dấu torque phù hợp như shoulder và elbow hiện tại:

```text
kpp_PC = pos_gain
kvp_PC = vel_gain * Nv * Nt / (2*pi)
kvi_PC = vel_integrator_gain * Nv * Nt / (2*pi)
```

Elbow có `Nv=8` (mặc định của plugin), `Nt=1`, nên hệ số là `8/(2*pi)`:

```text
kpp_PC = 25
kvp_PC = 0.7639437268
kvi_PC = 12.7323954474
```

Shoulder có `Nv=64`, `Nt=8`, nên hệ số là `512/(2*pi)`:

```text
kpp_PC = 10
kvp_PC = 61.1154981473
kvi_PC = 81.48733086305
```

Các gain shoulder lớn hơn do phép quy đổi encoder và torque trong cấu hình
CAN của khớp này. YAML có thứ tự `[base, shoulder, elbow]`; base (CAN ID 0)
tạm dùng cùng gain với elbow vì chưa tune riêng. Hai khớp này cùng `Nv=8`, `Nt=1`.
Không nhân thêm torque constant vì controller PC xuất Nm trực tiếp và phép đổi trên
giả định gain driver cũng xuất torque, không phải dòng điện.

Đây là tương đương phần feedback khi chưa bão hòa, cùng reference và state
tích phân tương ứng. Chu kỳ điều khiển PC 100 Hz, giới hạn tích phân,
anti-windup, velocity limiting của driver và feedforward URDF có thể tạo đáp
ứng khác. Vì vậy bộ số quy đổi là điểm khởi đầu để thử torque, chưa phải kết
quả đã được xác nhận ổn định trên tay thật. Giới hạn torque và dấu CAN không đổi.

CSV ghi `q/qref`, `qd/qdref`, `tau` đã kẹp, `tau_ff`, `tau_fb`, `tau_p`,
`tau_i`, `tau_raw` trước kẹp, trạng thái bão hòa và gain thực đang dùng.
Đồ thị mô-men hiển thị riêng các thành phần này. `tau_p/tau_i` là phần P/I
của vòng vận tốc; `kpp` thuộc vòng vị trí bên ngoài.

Nếu `saturated` xuất hiện nhiều, đánh giá trần mô-men, tốc độ quỹ đạo và gain
trước khi tăng I. Dòng stale/recovery trong CSV không có feedback PID mới,
nên các cột thành phần được đánh dấu NaN thay vì ghi lại giá trị cũ.

Để đổi khớp, chọn cùng tên ở cả launch phần cứng (`torque_joint:=base` hoặc
`torque_joint:=shoulder`) và launch PID (`joint:=base` hoặc `joint:=shoulder`).
Việc đổi `torque_joint` cần khởi động lại phần cứng và thiết lập lại mốc zero
ở đúng tư thế; chỉ sửa gain cùng một khớp thì không cần khởi động lại phần cứng.
Góc đích là góc tuyệt đối trong hệ zero ROS; node kiểm giới hạn URDF và margin
hiện có. Dấu và hệ số quy đổi CAN dùng nguyên cấu hình phần cứng hiện tại.
