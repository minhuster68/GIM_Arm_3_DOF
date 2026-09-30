# Kiến trúc bốn bộ điều khiển mô-men GIM Arm

Cả bốn thuật toán dùng chung một đường tín hiệu:

```text
/joint_states + common reference trajectory
                  |
                  v
      PID | LQR | MPC | SMC (exactly one)
                  |
                  v
   common safety/state machine/logger
                  |
                  v
/gim_arm_effort_controller/commands (joint Nm)
                  |
                  v
       ros2_control -> CAN -> plant
```

`gim_control` chịu trách nhiệm tạo quỹ đạo, các pha
gravity/approach/track/return/hold-home, kiểm tra trạng thái và giới hạn khớp,
kẹp mô-men và ghi CSV. Mỗi package thuật toán
chỉ khai tham số và cung cấp hàm `compute(...) -> tau`:

- `gim_arm_controller_pid`
- `gim_arm_controller_lqr`
- `gim_arm_controller_mpc`
- `gim_arm_controller_smc`

File quỹ đạo chung không gọi cứng một controller. Launch argument `algorithm`
chọn đúng một package/node theo bảng sau:

| `algorithm` | package | node ROS |
|---|---|---|
| `pid` | `gim_arm_controller_pid` | `/cascade_pid_controller` |
| `lqr` | `gim_arm_controller_lqr` | `/lqr_controller` |
| `mpc` | `gim_arm_controller_mpc` | `/mpc_controller` |
| `smc` | `gim_arm_controller_smc` | `/smc_controller` |

Build:

```bash
colcon build --symlink-install --packages-up-to \
  gim_arm_controller_pid gim_arm_controller_lqr \
  gim_arm_controller_mpc gim_arm_controller_smc
source install/setup.bash
```

Khởi động MuJoCo/vCAN và controller manager riêng. Effort controller được nạp
ở trạng thái inactive. Khi `/joint_states` đã ổn định, chọn đúng một thuật
toán:

```bash
ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=lqr tau_scale:=0.35 autostart:=false
```

Sau đó chuyển từ position sang effort mode:

```bash
ros2 control switch_controllers \
  --deactivate gim_arm_group_controller \
  --activate gim_arm_effort_controller
```

Sau khi switch, runner mặc định giữ HOME đã chụp trước switch. Clutch chung
cho cả bốn thuật toán được điều khiển bằng service:

```bash
# Clutch ON: gravity + damping, có thể kéo tay robot.
ros2 service call /gim_arm/set_hand_guiding \
  std_srvs/srv/SetBool "{data: true}"

# Clutch OFF: chụp q hiện tại và giữ bằng gravity-hold PID.
ros2 service call /gim_arm/set_hand_guiding \
  std_srvs/srv/SetBool "{data: false}"
```

Chỉ service clutch khi runner ở `GRAVITY_HOLD` hoặc `HOLD_HOME`; yêu cầu bị từ
chối trong APPROACH/TRACK/RETURN. Nếu `autostart=true` khi clutch còn ON,
runner giữ HAND_GUIDING và chưa chạy quỹ đạo. Thả clutch sẽ chốt vị trí mới,
sau đó quỹ đạo mới bắt đầu. Ví dụ bật quỹ đạo PID:

```bash
ros2 param set /cascade_pid_controller autostart true
```

Kịch bản chung của cả bốn thuật toán là:

```text
HOME (tư thế chụp trước switch, hoặc tư thế mới khi clutch OFF)
  -> APPROACH 5 s tới điểm đầu đường quét
  -> TRACK đúng 1 vòng (~27 s)
  -> RETURN 5 s về HOME
  -> HOLD_HOME
```

Các mode dùng chung quanh thuật toán bám quỹ đạo:

```text
GRAVITY_HOLD --clutch ON--> HAND_GUIDING: tau = G(q) - Kd_drag*qdot
HAND_GUIDING --clutch OFF-> GRAVITY_HOLD: chụp q_hold rồi giữ vòng kín
GRAVITY_HOLD --autostart--> APPROACH -> TRACK -> RETURN -> HOLD_HOME
```

`Kd_drag=[0.3, 1.0, 0.3] Nm/(rad/s)` mặc định tạo damping nhỏ trong lúc kéo.
Trong HAND_GUIDING, `q_hold` chạy theo state hiện tại và tích phân HOLD luôn
được xóa để không windup.

Đường quét dùng time-scaling minimum-jerk, nên vận tốc và gia tốc đều bằng 0
ở đầu/cuối vòng. `max_transition_error_rad` (mặc định 0.10 rad) áp cho APPROACH
và RETURN; `max_track_error_rad` chặt hơn (0.05 rad) chỉ áp cho TRACK/HOLD.

`HOLD_HOME` không dùng tiếp controller bám quỹ đạo. Runner xóa trạng thái tích
phân của controller rồi chuyển sang gravity-hold PID chung:

```text
tau = G(q) + hold_kp * e + hold_ki * integral(e) - hold_kd * qdot
hold_kp = [5, 12, 4] Nm/rad
hold_kd = [1, 3, 0.8] Nm/(rad/s)
hold_ki = [0.8, 2, 0.8] Nm/(rad*s)
|tau_i| <= [0.5, 1, 0.5] Nm
```

Nhờ vậy PID/LQR/MPC/SMC được so sánh trên cùng quỹ đạo, nhưng sau khi hoàn tất
đều về một bộ giữ HOME mềm và có damping, thay vì tiếp tục dùng gain bám quỹ
đạo có thể gây limit-cycle quanh điểm đứng yên. HOME được chụp ngay khi node
nhận state đầu tiên, lúc position controller còn active; thành phần I có
anti-windup và giới hạn mô-men riêng để xóa sai số tĩnh mà không tích lũy vô
hạn.

Runner đảo dấu một heartbeat mặc định `1e-6 Nm` trên lệnh base ở mỗi chu kỳ.
Mức này không có ý nghĩa cơ học, nhưng giúp hardware watchdog phân biệt một
lệnh GRAVITY/HOLD hợp lệ đang đứng yên với trường hợp node phát mô-men đã chết
và `forward_command_controller` giữ lại giá trị cuối.

## Gazebo Classic

Đường Gazebo dùng cùng URDF nguồn nhưng thay backend CAN bằng
`gazebo_ros2_control/GazeboSystem` lúc launch; vì vậy khối lượng, tâm khối,
inertia, giới hạn khớp và mesh không bị chép sang một URDF thứ hai. Chạy bằng:

```bash
ros2 launch gim_control gazebo_effort_control.launch.py
```

URDF runtime của Gazebo nâng `base_footprint` lên `0.300 m`. Giá trị này được
tính từ điểm thấp nhất của mesh ghế sau phép xoay cố định `0.073 rad`, nên chân
ghế nằm ngay trên mặt phẳng `z=0` (hở khoảng `1.9 mm`). URDF dùng cho phần cứng
thật không bị thay đổi.

Gazebo khởi động ở trạng thái pause, nạp `joint_state_broadcaster`, activate
physics tạm thời với gravity bằng 0 để controller manager có update tick,
activate `forward_position_controller`, nạp hai controller còn lại ở trạng
thái inactive, gửi lệnh HOME `[0,0,0]`, rồi mới bật gravity `-9.81 m/s²`.

Các STL SolidWorks vẫn được dùng để hiển thị, nhưng collision mesh được bỏ
khỏi URDF runtime của Gazebo. Các mesh lắp ráp có nhiều mặt lõm và giao nhau ở
ổ trục; dùng trực tiếp làm collision khiến ODE tự phát năng lượng dù gravity
bằng 0. Đây là mô phỏng điều khiển khớp, chưa phải mô phỏng va chạm với người
hoặc ghế.

### Chạy thử cascade PID trên Gazebo

Terminal 1:

```bash
cd ~/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gim_control gazebo_effort_control.launch.py
```

Terminal 2 (để `use_sim_time=false`; `/clock` của Gazebo Classic phát theo
cụm và làm timer mô-men bị giật nếu dùng làm clock điều khiển):

```bash
cd ~/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=pid control_hz:=2000.0 tau_scale:=0.50 \
  max_transition_error_rad:=0.35 max_track_error_rad:=0.10 \
  approach_time:=5.0 return_time:=5.0 loops:=1.0 \
  command_heartbeat_nm:=0.0 use_sim_time:=false autostart:=false \
  log_file:=/tmp/gim_pid.csv
```

Terminal 3, chỉ chạy khi terminal 2 đã báo `WAIT -> GRAVITY`:

```bash
cd ~/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 control switch_controllers \
  --deactivate forward_position_controller \
  --activate gim_arm_effort_controller
ros2 param set /cascade_pid_controller autostart true
```

Sau khi node báo `RETURN -> HOLD_HOME`, nhấn `Ctrl-C` ở terminal 2 để ghi
CSV. Vẽ quỹ đạo và sai số End-Effector bằng đúng điểm `tool_tip` trong
`matlab-sim/urdf_PID/setup_pid.m`:

```bash
ros2 run gim_control plot_ee_error /tmp/gim_pid.csv \
  --output /tmp/gim_pid_ee_error.png --show
```

Công cụ dùng offset MATLAB `[0.34349, 0.04674, 0.00607] m` từ frame
`lower_arm_link`. Hình kết quả gồm quỹ đạo EE 3D thực tế/tham chiếu và ba đồ
thị góc khớp `q1/q2/q3` thực tế/tham chiếu theo thời gian. Terminal đồng thời
in RMS/MAX của sai số EE cho toàn bài chạy và từng pha
APPROACH/TRACK/RETURN. CSV ghi `q` và `qref`, sau đó cả hai được đưa qua cùng
một FK, nên đồ thị đo sai số bám thực tế thay vì chỉ vẽ lại waypoint mong
muốn.

PID lấy nguyên gain liên tục của Simulink nên cần vòng physics/state/torque 2
kHz để kiểm tra trên Gazebo. Lần kiểm tra tích hợp ban đầu ở `tau_scale=0.50`
đã chạy đủ một vòng với sai số TRACK lớn nhất khoảng `0.0098 rad`, nhưng lần
ghi log tốc độ cao sau đó thấy spike vận tốc và chạm trần mô-men dù không ABORT;
không xem đó là nghiệm thu chất lượng bám. Gravity-hold PID
được kiểm tra riêng ở `tau_scale=0.50`: HOME chụp trước switch gần `[0,0,0]`,
sai số lớn nhất sau vài giây là `0.0013 rad` và tiếp tục giảm xuống dưới
`0.00018 rad`, không bão hòa mô-men. Tuy vậy PID bám quỹ đạo vẫn cần thiết kế
lại ở đúng 100 Hz của CAN trước khi chạy tay thật.

### Kết quả đối chiếu bốn bộ MATLAB trên Gazebo

Điều kiện kiểm tra: ODE 2 kHz, cùng inertial URDF, giới hạn khớp đang dùng,
`use_sim_time=false`, không có collision/contact. Đây là kết quả của profile
MATLAB nguyên gốc, chưa phải profile đã retune cho ROS/CAN.

| Bộ điều khiển | Điều kiện | Kết quả |
|---|---|---|
| PID MATLAB gốc | 2 kHz, `tau_scale=0.50` | Chạy đủ APPROACH/TRACK/RETURN, nhưng log sau cho thấy rung vận tốc/chạm trần mô-men; HOLD chuyển sang gravity-hold PID |
| LQR | 100 Hz, `tau_scale=0.50` | Node tự báo `|z|max=24.9131`; safety abort gần giới hạn khớp sau khi đóng vòng |
| MPC | 100 Hz, thử cả `tau_scale=0.50` và `1.0` | SLSQP thường vượt ngân sách 8 ms; safety abort trong APPROACH |
| SMC | 100 Hz và 2 kHz, thử đến `tau_scale=1.0` | Safety abort trong APPROACH; tăng tần số và trần mô-men không khắc phục |

Do đó không được hiểu việc bốn mô hình chạy trong Simulink là bốn bộ số có
thể chép thẳng sang controller rời rạc. Giữ các YAML hiện tại làm profile
`matlab_reference`; cần retune/thiết kế rời rạc cho từng thuật toán ở đúng nhịp
điều khiển mục tiêu. Profile PID 2 kHz dưới đây chưa giải quyết bài toán 100 Hz.

### PID profile riêng để giảm rung trong Gazebo

`pid.yaml` vẫn giữ gain MATLAB `[46,60,36] / [8,9.5,10] / [7.6,7.6,7.6]`
(Kpp/Kvp/Kvi). Chọn `pid_profile:=gazebo_smooth` để dùng profile thử nghiệm
`pid_gazebo_smooth.yaml`: `[18,24,14] / [2,3,2.5] / [1.5,1.5,1.5]`.
Launch tự chọn `control_hz=2000` và `tau_scale=0.50` cho profile này nếu không
truyền đè; vẫn cần kiểm tra các giá trị được in ra trước khi switch controller.

```bash
ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=pid pid_profile:=gazebo_smooth \
  control_hz:=2000.0 tau_scale:=0.50 \
  max_transition_error_rad:=0.35 max_track_error_rad:=0.10 \
  approach_time:=5.0 return_time:=5.0 loops:=1.0 \
  command_heartbeat_nm:=0.0 use_sim_time:=false autostart:=false \
  log_file:=/tmp/gim_pid_gazebo_smooth.csv
```

Sau khi node báo `WAIT -> GRAVITY`, activate effort controller như phần trên
rồi mới bật `autostart`. Trong một lượt Gazebo 2 kHz trên quỹ đạo đầy đủ,
max sai số TRACK `[0.236,0.620,0.205]` độ, max sai số vận tốc
`[0.201,0.052,0.032]` rad/s, mô-men đỉnh `[0.842,3.603,1.545]` Nm,
không chạm trần `[2.5,20,2.5]` Nm. Cùng profile trên đoạn ngắn 2 giây
ở 2 kHz cũng không chạm trần. Đây là kết quả mô phỏng, **không phải**
nghiệm thu trên phần cứng. Khi thử riêng đoạn ngắn ở 100 Hz, profile này
vẫn ABORT tại TRACK khoảng 1.29 s; không chuyển thẳng profile 2 kHz sang
CAN/robot thật. Cần thiết kế/tune ở đúng nhịp phần cứng và xác minh đường
phản hồi vận tốc trước khi chạy thật.

### Chạy LQR an toàn ban đầu trên Gazebo

Launch chung có hai profile LQR:

- `lqr_profile:=safe_100hz`: bộ TVLQR đã kiểm tra ổn định ZOH ở 100 Hz, dùng
  để bring-up trên Gazebo rồi mới tiến tới tay thật. Profile giải DARE trực
  tiếp trên mô hình ZOH tại chu kỳ điều khiển.
- `lqr_profile:=matlab_reference`: đúng trọng số CARE liên tục trong MATLAB;
  chỉ dùng để đối chiếu vì không ổn định khi áp trực tiếp bằng ZOH 100 Hz.

Sau khi Gazebo đã chạy ở terminal 1, mở terminal 2:

```bash
ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=lqr lqr_profile:=safe_100hz \
  control_hz:=200.0 lqr_recompute_every:=2 tau_scale:=1.0 \
  max_transition_error_rad:=0.35 max_track_error_rad:=0.10 \
  approach_time:=5.0 return_time:=5.0 loops:=1.0 \
  command_heartbeat_nm:=0.0 use_sim_time:=false autostart:=false
```

Sau khi node báo `WAIT -> GRAVITY`, switch ở terminal 3 rồi mới bắt đầu:

```bash
ros2 control switch_controllers \
  --deactivate forward_position_controller \
  --activate gim_arm_effort_controller
ros2 control list_controllers
ros2 param set /lqr_controller autostart true
```

Phải xác nhận `forward_position_controller` là `inactive` và
`gim_arm_effort_controller` là `active` trước khi bật `autostart`.

Trạng thái kiểm thử hiện tại: profile này khởi động đúng, DARE cho
`|z|max < 1`, hoàn thành APPROACH nhưng safety vẫn ngắt giữa TRACK khi lỗi
khớp tức thời đạt khoảng `0.106 rad > 0.10 rad`. Vì vậy lệnh trên dùng để tiếp
tục chẩn đoán/tune trong Gazebo, chưa được xem là profile LQR đã nghiệm thu và
tuyệt đối chưa chuyển sang tay thật. Không nới `max_track_error_rad` để che
spike; cần xử lý sai khác plant/độ trễ và kiểm lại vận tốc khớp trước.

### SMC rời rạc 100 Hz trên Gazebo

`smc.yaml` giữ nguyên profile MATLAB liên tục để đối chiếu. Trong lớp biên,
hệ số của reaching law là `alpha = ks + kr/phi`; profile gốc cho
`alpha=[1005,2737,780] 1/s`. Khi giữ lệnh gia tốc trong một chu kỳ 10 ms,
xấp xỉ computed-torque lý tưởng đã có bán kính phổ
`rho=[9.814,38.260,13.565] > 1`, phù hợp với hiện tượng bão hòa mô-men và
safety abort ngay đầu APPROACH. Tăng trần mô-men không sửa được mất ổn định
rời rạc này.

Profile `smc_gazebo_safe.yaml` dùng:

```text
lambda = [30, 35, 40]
ks     = [25, 30, 30]
kr     = [40, 100, 100]
phi    = [2, 5, 4]
```

Khi đó `alpha=[45,50,55] 1/s` và `rho=[0.7703,0.7427,0.7168]`. Factory sẽ
in các giá trị này khi khởi động và phát cảnh báo nếu bất kỳ `rho >= 1`.
Launch chọn profile an toàn bằng `smc_profile:=gazebo_safe` (mặc định cho
SMC), chạy ở 100 Hz và dùng `tau_scale=0.50` khi không truyền đè.

Hai lượt kiểm tra đầy đủ APPROACH/TRACK/RETURN/HOLD trên Gazebo đều hoàn tất:

| Tải tại tool tip | `tau_scale` | max sai số TRACK q1/q2/q3 | max sai số vận tốc | max bước mô-men |
|---|---:|---:|---:|---:|
| 0 kg | 0.50 | `[0.176,0.254,0.338]` độ | `[0.024,0.034,0.043]` rad/s | `[0.176,0.150,0.051]` Nm |
| 0.5 kg | 0.75 | `[0.641,0.686,2.472]` độ | `[0.031,0.036,0.048]` rad/s | `[0.254,0.236,0.084]` Nm |

Với tải 0.5 kg, riêng gravity ở elbow cần khoảng `3.52 Nm`, lớn hơn trần
`2.50 Nm` của `tau_scale=0.50`; vì vậy lượt có tải phải truyền
`tau_scale:=0.75`. Đây mới là xác minh trên Gazebo, chưa phải nghiệm thu trên
phần cứng thật.

Các YAML `matlab_reference` đặt `tau_scale: 1.0` để đúng trần MATLAB
`[5, 40, 5] Nm`. Launch chung hạ trần theo profile bring-up; riêng SMC
`gazebo_safe` mặc định dùng 0.50. Chỉ truyền `tau_scale:=1.0` để đối chiếu
toàn biên trên simulator trước khi có quy trình an toàn tương ứng trên tay
thật.

## Quan hệ với mô hình MATLAB

- Cascade PID, SMC và các trọng số/horizon MPC lấy từ repo `matlab-sim`.
- `lqr.yaml` dùng đúng bộ Bryson trong `setup_lqr.m`. Bộ này được thiết kế bằng
  CARE liên tục; khi áp trực tiếp bằng ZOH 100 Hz trong implementation ROS thì
  `|z|max` khoảng 24.9 (> 1). Vì vậy nó chỉ dùng để đối chiếu trên MuJoCo trước.
  Profile đã giảm gain được giữ riêng trong `lqr_safe_100hz.yaml`.
- MPC hiện giải QP cô đọng bằng SciPy/SLSQP với gradient giải tích. Benchmark
  cục bộ 100 chu kỳ ở `Np=40`, `Nc=10` cho median khoảng 6.1 ms, p99 khoảng
  7.5 ms. Biên thời gian ở 100 Hz vẫn mỏng, nên phải chạy hết quỹ đạo trên
  MuJoCo và kiểm worst-case trước khi dùng phần cứng.
