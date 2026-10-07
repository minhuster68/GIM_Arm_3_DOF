# GIM Arm 3 DOF — mô phỏng bám quỹ đạo trong Gazebo

Hướng dẫn chạy bốn thuật toán PID, LQR, MPC và SMC trên ROS 2 Humble với
Gazebo Classic. Mỗi thuật toán có lệnh chạy theo thứ tự trong 4 terminal.

| Thuật toán | Trạng thái hướng dẫn |
| --- | --- |
| [PID](#1-pid) | Có lệnh chạy trong 4 terminal |
| [LQR](#2-lqr) | Có lệnh chạy trong 4 terminal |
| [MPC](#3-mpc) | Có lệnh chạy trong 4 terminal |
| [SMC](#4-smc) | Có lệnh chạy trong 4 terminal |

## Chuẩn bị

Workspace dùng trong hướng dẫn: `/home/mirabo/GIM_Arm_3_DOF`.
Nếu đặt dự án ở thư mục khác, đổi đường dẫn trong các lệnh tương ứng.

Các dependency ROS/Gazebo cần được cài trước khi build. Với máy mới:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-pinocchio \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  python3-numpy \
  python3-scipy \
  python3-matplotlib
```

Build các package cần cho cả bốn thuật toán trước lần chạy đầu tiên hoặc sau khi sửa code:

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  gim_arm_description \
  gim_control \
  gim_arm_controller_pid \
  gim_arm_controller_lqr \
  gim_arm_controller_mpc \
  gim_arm_controller_smc

source install/setup.bash
```

## 1. PID

Các lệnh bên dưới chạy mô phỏng **không tải**, dùng profile
`pid_profile:=gazebo_smooth`. Mở 4 terminal và thực hiện theo thứ tự.
Mỗi terminal đều cần nạp môi trường bằng hai lệnh `source`.

### Terminal 1 — mở Gazebo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gim_control gazebo_effort_control.launch.py \
  gui:=true \
  verbose:=false \
  payload_mass_kg:=0.0
```

Giữ Terminal 1 chạy trong suốt quá trình mô phỏng.

### Terminal 2 — mở bộ điều khiển PID

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p results

ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=pid \
  pid_profile:=gazebo_smooth \
  tau_scale:=0.50 \
  autostart:=false \
  use_sim_time:=false \
  log_file:=/home/mirabo/GIM_Arm_3_DOF/results/gim_pid_run.csv
```

Chờ Terminal 2 báo `WAIT -> GRAVITY` trước khi chạy Terminal 3.
File CSV của lần chạy mới sẽ ghi đè file cùng tên của lần trước;
đổi `log_file` nếu cần lưu riêng từng lần chạy.

### Terminal 3 — bắt đầu bám quỹ đạo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service call /gim_arm/set_hand_guiding \
  std_srvs/srv/SetBool "{data: false}"

ros2 control switch_controllers \
  --strict \
  --activate-asap \
  --deactivate forward_position_controller \
  --activate gim_arm_effort_controller

sleep 5

ros2 param set /cascade_pid_controller autostart true
```

Chờ bộ điều khiển hoàn thành quỹ đạo, trở về HOME và báo `HOLD`
ở Terminal 2, rồi thực hiện các bước ở Terminal 4.

### Terminal 4 — kết thúc và vẽ kết quả

Chuyển về bộ điều khiển vị trí và gửi lệnh giữ HOME:

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control switch_controllers \
  --strict \
  --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate forward_position_controller

ros2 topic pub --once \
  /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  '{data: [0.0, 0.0, 0.0]}'
```

**Nhấn Ctrl+C ở Terminal 2 và chờ node dừng xong để ghi hoàn tất CSV.**
Sau đó quay lại Terminal 4 và chạy lệnh vẽ:

```bash
ros2 run gim_control plot_joint_tracking \
  /home/mirabo/GIM_Arm_3_DOF/results/gim_pid_run.csv \
  --show \
  --max-plot-points 30000
```

Đồ thị hiển thị góc khớp, sai số góc và mô-men của từng khớp.
Khi xem xong, nhấn Ctrl+C ở Terminal 1 để đóng Gazebo.

### Lỗi `Package 'gim_control' not found`

Nạp lại môi trường trong chính terminal đang báo lỗi:

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix gim_control
```

Nếu lệnh cuối in ra đường dẫn package, chạy lại lệnh bị lỗi.
Nếu chưa có `install/setup.bash`, thực hiện bước build trong mục Chuẩn bị.

## 2. LQR

Profile `lqr_profile:=gazebo_matlab` theo
[setup_lqr.m và mô hình Simulink](https://github.com/minhuster68/matlab-sim/tree/addec462971d946519cafa6cda5ebbcd18539da2/urdf_LQR)
của repo MATLAB, **nhánh `position-tracking`**, commit `addec462`.

- Mục tiêu chỉ phạt sai số vị trí và tích phân sai số: `Q_vel = [0,0,0]`.
  Profile này bỏ `max_de`, dùng `position_tracking_only: true` trong YAML.
- Vẫn giữ trạng thái `x = [tích phân(e), e, qd - qd_ref]`, với `e = q - q_ref`.
  Gain vận tốc do CARE tạo ra cung cấp damping, không thêm mục tiêu phạt vận tốc.
- Luật điều khiển: `tau = inverse_dynamics(q_ref, qd_ref, qdd_ref) - K*x`.
- Giải CARE tại từng điểm (frozen-time gain scheduling), tính trước trên
  các pha APPROACH, TRACK và RETURN;
  nội suy tuyến tính như `ts_K` trong Simulink.
- Đồng hồ gain đi cùng đồng hồ quỹ đạo ROS, dừng khi feedback cũ và quay
  lại đầu phần TRACK ở vòng quét tiếp theo.
- Vòng mô-men chạy ở 2 kHz; log chẩn đoán lưu ở 200 Hz để giảm tải.

Tham số nằm trong
[`lqr_gazebo_matlab.yaml`](src/gim_arm_controller_lqr/config/lqr_gazebo_matlab.yaml).
Các giới hạn Bryson gốc của MATLAB là `max_int_e=[10,20,10]`,
`max_e=[0.05,0.02,0.01]`, `max_tau=[5,40,5]`;
giới hạn tích phân là `0.5 rad*s`.

Profile Gazebo dùng **R nhân 16** (`tau_penalty_scale=16.0`) để giảm băng
thông phản hồi khi có trễ ROS. Luật phản hồi, Q và các giới hạn Bryson giữ
theo MATLAB. Có thể đổi R khi tune bằng launch argument
`lqr_tau_penalty_scale:=...`; argument này ghi đè giá trị trong YAML.

Gazebo dùng URDF và quỹ đạo ROS hiện tại. Không đảo dấu khớp 2 và 3 theo
file import MATLAB. Tải `payload_mass_kg` chỉ thêm vào plant Gazebo,
không thêm vào feedforward, cùng ý tưởng `compensatePayload=false`.
Gazebo gắn một vật có khối lượng và quán tính, còn MATLAB đặt lực trọng trường
ngoài tại tool tip; hai mô phỏng không có động lực học tải hoàn toàn giống nhau.

Các lệnh bên dưới chạy **không tải**. Trước khi chuyển từ PID sang LQR,
dừng node PID và Gazebo cũ bằng Ctrl+C rồi bắt đầu từ Terminal 1.

### Terminal 1 — mở Gazebo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gim_control gazebo_effort_control.launch.py \
  gui:=true \
  verbose:=false \
  payload_mass_kg:=0.0
```

Giữ Terminal 1 chạy trong suốt quá trình mô phỏng.

### Terminal 2 — mở bộ điều khiển LQR

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p results

ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=lqr \
  lqr_profile:=gazebo_matlab \
  tau_scale:=0.50 \
  autostart:=false \
  use_sim_time:=false \
  log_file:=/home/mirabo/GIM_Arm_3_DOF/results/gim_lqr_run.csv
```

Chờ Terminal 2 tính xong bảng gain (`Precompute xong ... gain`) và báo
`WAIT -> GRAVITY` trước khi chạy Terminal 3. Giữ position controller active
trong lúc tính bảng gain.
File CSV của lần chạy mới sẽ ghi đè file cùng tên của lần trước;
đổi `log_file` nếu cần lưu riêng từng lần chạy.

### Terminal 3 — bắt đầu bám quỹ đạo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service call /gim_arm/set_hand_guiding \
  std_srvs/srv/SetBool "{data: false}"

ros2 control switch_controllers \
  --strict \
  --activate-asap \
  --deactivate forward_position_controller \
  --activate gim_arm_effort_controller

sleep 5

ros2 param set /lqr_controller autostart true
```

Chờ bộ điều khiển hoàn thành quỹ đạo, trở về HOME và báo `HOLD`
ở Terminal 2, rồi thực hiện các bước ở Terminal 4.

### Terminal 4 — kết thúc và vẽ kết quả

Chuyển về bộ điều khiển vị trí và gửi lệnh giữ HOME:

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control switch_controllers \
  --strict \
  --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate forward_position_controller

ros2 topic pub --once \
  /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  '{data: [0.0, 0.0, 0.0]}'
```

**Nhấn Ctrl+C ở Terminal 2 và chờ node dừng xong để ghi hoàn tất CSV.**
Sau đó quay lại Terminal 4 và chạy lệnh vẽ:

```bash
ros2 run gim_control plot_joint_tracking \
  /home/mirabo/GIM_Arm_3_DOF/results/gim_lqr_run.csv \
  --show \
  --max-plot-points 30000
```

Đồ thị hiển thị góc khớp, sai số góc và mô-men của từng khớp.
Khi xem xong, nhấn Ctrl+C ở Terminal 1 để đóng Gazebo.

### Thử tải và kiểm tra tự động

Với tải 0,5 kg: đổi `payload_mass_kg:=0.5` ở Terminal 1 và
`tau_scale:=0.75` ở Terminal 2.

Để kiểm tra không tải và tải 0,5 kg trên Gazebo headless:

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/validate_lqr_gazebo.py
```

Script dùng ROS domain 74 và cổng Gazebo 11374, xuất kết quả riêng tại
`results/lqr_matlab_gazebo/`, rồi dừng các tiến trình mô phỏng của nó.

Đã kiểm tra ngày 06/10/2026 với **Q_vel = 0**, nguồn `position-tracking`:
cả không tải và tải 0,5 kg hoàn thành APPROACH → TRACK → RETURN → HOLD.
Ngưỡng dừng TRACK giữ ở `0.05 rad`. Sai số RMS pha TRACK,
thứ tự Base / Shoulder / Elbow:

| Tải | RMS sai số góc (độ) |
| --- | --- |
| 0 kg | 0.154 / 0.046 / 0.013 |
| 0,5 kg | 0.796 / 1.538 / 0.389 |

Đây là cấu hình khởi đầu để tune tiếp; tải chưa được bù trong mô hình.
Kết quả từ bản đọc nhầm nhánh `main` đã lưu riêng tại
`results/lqr_matlab_gazebo/main_reference/`.
Nhánh MATLAB `position-tracking` bắt đầu tune với `payloadMass=0`.

## 3. MPC

Profile `mpc_profile:=gazebo_position` theo
[`urdf_MPC/setup_mpc.m`, error_model.m và mô hình Simulink](https://github.com/minhuster68/matlab-sim/tree/addec462971d946519cafa6cda5ebbcd18539da2/urdf_MPC)
của **nhánh `position-tracking`**, commit `addec462`.

- Mục tiêu bám vị trí: `q_velocity=[0,0,0]`, giữ 9 trạng thái
  `x=[tích phân(e), e, qd-qd_ref]`, với `e=q-q_ref`.
  Vận tốc đo vẫn tham gia dự đoán động lực học và tạo damping.
- Chu kỳ `Ts=0.01 s` (100 Hz), prediction horizon `Np=40`, control horizon `Nc=10`.
  Sau bước thứ 10, mô-men phản hồi cuối được giữ đến cuối horizon.
- `tau = inverse_dynamics(q_ref, qd_ref, qdd_ref) + u_mpc`.
  Mô hình sai số lấy đạo hàm số của toàn bộ inverse dynamics, gồm `M(q)*qdd_ref`;
  rời rạc phần cơ học bằng ZOH, tích phân sai số bằng Forward Euler.
- Trọng số gốc: tích phân `[2,2,2]`, vị trí `[0.6271,1.5677,0.6271]`,
  mô-men `[0.0097,0.0097,0.0097]`, thay đổi mô-men `[0.03,0.03,0.03]`.
  ScaleFactor tương ứng là `0.25`, `0.05`, `0.10` cho ba nhóm trạng thái,
  và `[5,40,5]` cho mô-men. Hệ số trong hàm mục tiêu là
  **`(weight/ScaleFactor)^2`**, theo
  [công thức MPC Toolbox](https://www.mathworks.com/help/mpc/ug/optimization-problem.html).
- Dành miền mô-men phản hồi cố định cho cả quỹ đạo:
  `u_min=-tau_max-min(tau_ff)`, `u_max=tau_max-max(tau_ff)`.
  `torque_slew_rate=[100,200,100] Nm/s` giới hạn tốc độ thay đổi mô-men phản hồi.
  Tích phân không đặt trần (`integral_limit=0`), cùng cấu hình MATLAB.

Tham số nằm trong
[`mpc_gazebo_position.yaml`](src/gim_arm_controller_mpc/config/mpc_gazebo_position.yaml).
Gazebo dùng **R và Rdu nhân 10000** sau khi chuẩn hoá để giảm độ gắt phản hồi
với trễ ROS; Q và mục tiêu bám vị trí giữ theo nguồn. Launch argument
`mpc_torque_penalty_scale:=...` thay hệ số này khi tune.
Giá trị `1.0` khôi phục trọng số gốc; trọng số gốc đã chạm ngưỡng ABORT
trong thử nghiệm trên pipeline ROS/Gazebo này.

QP dùng bộ giải active-set với warm start, ràng buộc mô-men và slew-rate.
Các ma trận dự đoán được tính trước dọc APPROACH/TRACK/RETURN ở 100 Hz,
chọn mẫu gần nhất theo đồng hồ quỹ đạo, rồi giải QP từ trạng thái đo ở mỗi tick.
Mô hình tại mẫu được giữ cố định trong horizon, cùng cách adaptive MPC của MATLAB.
Tính trước mất khoảng 4 giây trên máy này; chưa activate effort trong thời gian đó.
Launch giới hạn BLAS ở một thread để giảm biến động thời gian tính toán.

Gazebo đọc trực tiếp góc và vận tốc khớp; MATLAB dùng observer mặc định của
MPC Toolbox. Mô hình ROS dùng URDF hiện tại, `use_armature=false` để khớp
quán tính plant Gazebo, và quỹ đạo ROS 5 s APPROACH + 27 s TRACK + 5 s RETURN.
Tải chỉ thêm vào plant, feedforward không bù tải (`compensatePayload=false`
trong MATLAB). Gazebo thêm cả khối lượng/quán tính của vật; MATLAB dùng lực
trọng trường ngoài tại tool tip. Kết quả hai môi trường vì vậy có thể khác nhau.

Các lệnh bên dưới chạy **không tải**. Dừng bộ điều khiển và Gazebo của lần
PID/LQR trước bằng Ctrl+C, rồi mở 4 terminal theo thứ tự.

### Terminal 1 — mở Gazebo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gim_control gazebo_effort_control.launch.py \
  gui:=true \
  verbose:=false \
  payload_mass_kg:=0.0
```

### Terminal 2 — mở bộ điều khiển MPC

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p results

ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=mpc \
  mpc_profile:=gazebo_position \
  mpc_torque_penalty_scale:=10000.0 \
  tau_scale:=1.0 \
  autostart:=false \
  use_sim_time:=false \
  log_file:=/home/mirabo/GIM_Arm_3_DOF/results/gim_mpc_run.csv
```

Chờ `Precompute xong 3701 mẫu tham chiếu MPC` và `WAIT -> GRAVITY`
ở Terminal 2 trước khi chạy Terminal 3. CSV mới ghi đè file cùng tên;
đổi `log_file` để lưu riêng từng lần chạy.

### Terminal 3 — bắt đầu bám quỹ đạo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service call /gim_arm/set_hand_guiding \
  std_srvs/srv/SetBool "{data: false}"

ros2 control switch_controllers \
  --strict \
  --activate-asap \
  --deactivate forward_position_controller \
  --activate gim_arm_effort_controller

sleep 5

ros2 param set /mpc_controller autostart true
```

Chờ Terminal 2 báo `RETURN -> HOLD_HOME`, rồi thực hiện Terminal 4.

### Terminal 4 — kết thúc và vẽ kết quả

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control switch_controllers \
  --strict \
  --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate forward_position_controller

ros2 topic pub --once \
  /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  '{data: [0.0, 0.0, 0.0]}'
```

**Nhấn Ctrl+C ở Terminal 2 và chờ node dừng xong để ghi hoàn tất CSV.**
Sau đó vẽ ở Terminal 4:

```bash
ros2 run gim_control plot_joint_tracking \
  /home/mirabo/GIM_Arm_3_DOF/results/gim_mpc_run.csv \
  --show \
  --max-plot-points 30000
```

Khi xem xong, nhấn Ctrl+C ở Terminal 1 để đóng Gazebo.

### Thử tải và kiểm tra tự động

Với tải 0,5 kg, đổi Terminal 1 thành `payload_mass_kg:=0.5`;
giữ `tau_scale:=1.0` và profile MPC như trên.

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/validate_mpc_gazebo.py
```

Script chạy headless trên ROS domain 75 và cổng Gazebo 11375,
xuất CSV, đồ thị và thống kê tại `results/mpc_position_gazebo/`,
rồi dừng các tiến trình mô phỏng của nó. Có thể đổi domain/cổng bằng
`--ros-domain-id` và `--gazebo-port`.

Không tải và tải 0,5 kg đã hoàn thành cả ba pha, giữ ngưỡng dừng TRACK
`0.05 rad` và APPROACH/RETURN `0.10 rad`. Xem số đo và trạng thái bộ giải
trong [`validation.json`](results/mpc_position_gazebo/validation.json).
Đã kiểm tra ngày 07/10/2026; RMS sai số góc pha TRACK theo thứ tự
Base / Shoulder / Elbow:

| Tải | RMS sai số góc (độ) |
| --- | --- |
| 0 kg | 0.059 / 0.075 / 0.147 |
| 0,5 kg | 0.465 / 0.486 / 1.025 |

Trong log của hai lần chạy, 100% nghiệm QP hội tụ và được chấp nhận,
vi phạm ràng buộc bằng 0; thời gian tính controller trung vị khoảng 0,5 ms,
lớn nhất khoảng 2,88 ms so với chu kỳ 10 ms.
Đây là cấu hình khởi đầu để tune tiếp; tải chưa được bù và sai số khớp cuối
khi có tải còn lớn hơn không tải.

## 4. SMC

Dùng profile SMC hiện có `smc_profile:=gazebo_safe`, chạy ở 100 Hz.
Giữ nguyên cấu trúc vòng kín và luật điều khiển trong
[`controller.py`](src/gim_arm_controller_smc/gim_arm_controller_smc/controller.py).
Tham số nằm trong
[`smc_gazebo_safe.yaml`](src/gim_arm_controller_smc/config/smc_gazebo_safe.yaml):

```yaml
lambda_gain: [30.0, 35.0, 40.0]
ks: [25.0, 30.0, 30.0]
kr: [40.0, 100.0, 100.0]
phi: [2.0, 5.0, 4.0]
```

Các lệnh bên dưới chạy **không tải**. Dừng node PID/LQR/MPC và Gazebo của
lần trước bằng Ctrl+C, rồi mở 4 terminal theo thứ tự.

### Terminal 1 — mở Gazebo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch gim_control gazebo_effort_control.launch.py \
  gui:=true \
  verbose:=false \
  payload_mass_kg:=0.0
```

### Terminal 2 — mở bộ điều khiển SMC

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p results

ros2 launch gim_control run_effort_algorithm.launch.py \
  algorithm:=smc \
  smc_profile:=gazebo_safe \
  tau_scale:=0.50 \
  autostart:=false \
  use_sim_time:=false \
  log_file:=/home/mirabo/GIM_Arm_3_DOF/results/gim_smc_run.csv
```

Chờ Terminal 2 báo `WAIT -> GRAVITY` trước khi chạy Terminal 3.
CSV mới ghi đè file cùng tên; đổi `log_file` để lưu riêng từng lần chạy.

### Terminal 3 — bắt đầu bám quỹ đạo

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service call /gim_arm/set_hand_guiding \
  std_srvs/srv/SetBool "{data: false}"

ros2 control switch_controllers \
  --strict \
  --activate-asap \
  --deactivate forward_position_controller \
  --activate gim_arm_effort_controller

sleep 5

ros2 param set /smc_controller autostart true
```

Chờ Terminal 2 báo `RETURN -> HOLD_HOME`, rồi thực hiện Terminal 4.

### Terminal 4 — kết thúc và vẽ 3 đồ thị output

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control switch_controllers \
  --strict \
  --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate forward_position_controller

ros2 topic pub --once \
  /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  '{data: [0.0, 0.0, 0.0]}'
```

**Nhấn Ctrl+C ở Terminal 2 và chờ node dừng xong để ghi hoàn tất CSV.**
Sau đó vẽ ở Terminal 4:

```bash
ros2 run gim_control plot_joint_tracking \
  /home/mirabo/GIM_Arm_3_DOF/results/gim_smc_run.csv \
  --show \
  --max-plot-points 30000
```

Lệnh này mở **3 cửa sổ đồ thị**, mỗi khớp một cửa sổ. Mỗi cửa sổ gồm góc
thực tế/mong muốn, sai số góc và mô-men điều khiển. Ba file được lưu là:

- `results/gim_smc_run_joints_q1_base.png`
- `results/gim_smc_run_joints_q2_shoulder.png`
- `results/gim_smc_run_joints_q3_elbow.png`

Khi xem xong, nhấn Ctrl+C ở Terminal 1 để đóng Gazebo.

### Thử tải 0,5 kg

Đổi `payload_mass_kg:=0.5` ở Terminal 1 và `tau_scale:=0.75` ở Terminal 2;
các lệnh còn lại giữ nguyên.
