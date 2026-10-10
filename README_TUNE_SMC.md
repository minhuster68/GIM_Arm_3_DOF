# Tune SMC trên tay thật — hướng dẫn bàn giao

Làm việc trong **`/home/mirabo/GIM_Arm_3_DOF`, branch `smc-test`**. Không cần
tạo worktree hay thư mục khác. Đây là bộ khởi đầu để tune, **chưa có bộ SMC
đã tune trên tay thật**. MPC được tạm giữ trên `mpc-test`; không đổi chuẩn hóa
Q/R của MPC trong lần chuẩn bị này.

## Đọc nhanh trước khi bắt đầu

- Tune theo thứ tự **elbow/id2 → shoulder/id1 → base/id0**.
- Mỗi lần chỉ một khớp nhận torque. Hai khớp kia được motor giữ position.
- Đặt/đỡ tay đúng tư thế zero **trước khi launch**. Software zero lấy tư thế
  lúc khởi động làm mốc; launch ở tư thế sai sẽ làm mọi góc 0 bị sai theo.
- Luôn bắt đầu bằng `test 3 6 3`. Khi chạy ổn mới tăng biên độ. Không chạy
  quỹ đạo cả ba khớp hoặc dùng `torque_joint:=all` trong quy trình này.
- `set` chỉ chỉnh trong bộ nhớ terminal. **`save` mới ghi YAML**. `quit`,
  `reset` hoặc mở terminal khác có thể làm mất bộ chưa lưu.
- Khi dừng test, chờ **`POSITION active; EFFORT inactive`** rồi mới dừng
  launch phần cứng. Đỡ tay khi ngắt điện, nhất là base có thể trôi về sau.

Hai terminal là đủ: Terminal 1 chạy phần cứng, Terminal 2 về HOME và tune.
Trong các ô `text` dưới đây, nhập tại prompt `smc[...]`, không nhập vào Bash.

## 1. Chuẩn bị workspace và build

Nếu còn tuner PID/LQR/MPC chạy, kết thúc bài thử và thoát tuner trước. Chờ
position active, rồi Ctrl-C launch phần cứng cũ. Không đổi branch khi robot
đang được node cũ điều khiển.

```bash
cd /home/mirabo/GIM_Arm_3_DOF
git switch smc-test
git branch --show-current
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  gim_arm_description gim_arm_hardware gim_control gim_arm_controller_smc
source install/setup.bash
```

Tên branch phải là `smc-test`; build phải kết thúc thành công. Nếu đổi branch
hoặc sửa controller, build lại và thoát/mở lại tuner để Python nạp code mới.

Để xem hướng dẫn terminal mà không mở ROS/CAN hoặc ghi YAML:

```bash
bash tools/tune_smc_terminal.sh --joint elbow --dry-run
```

Trong dry-run có thể thử `show`, `test 3 6 3`, `quit`. Dòng `DRY RUN` nghĩa là
robot không nhận lệnh từ tuner này.

## 2. Terminal 1 — CAN và khởi động elbow

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
ip -details link show can0
```

Nếu CAN chưa được mở, cấu hình theo bitrate motor đang dùng. Workspace này
ghi cấu hình mặc định **500000 bit/s** trong README phần cứng:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

Không cần làm lại ba lệnh này khi CAN đang hoạt động đúng. Xem thêm
[README phần cứng](src/gim_arm_hardware/README.md) nếu không có `can0` hoặc
bitrate của motor đã được đổi.

Đặt và đỡ tay đúng zero, rồi launch:

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 \
  set_zero_on_startup:=true \
  zero_method:=software \
  torque_joint:=elbow
```

Giữ Terminal 1 chạy. Chưa mở `smc_node` riêng và không gửi torque thủ công:
terminal tune sẽ tạo controller cho từng bài thử.

## 3. Terminal 2 — kiểm tra controller và về 0

```bash
cd /home/mirabo/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
ros2 control list_controllers
```

Cần thấy:

```text
joint_state_broadcaster       active
gim_arm_group_controller     active
gim_arm_effort_controller    inactive
```

Gửi lệnh HOME chỉ khi position controller đang active:

```bash
ros2 action send_goal --feedback \
  /gim_arm_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [base_joint, shoulder_joint, elbow_joint], points: [{positions: [0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0], time_from_start: {sec: 8}}]}}"
```

Chờ **`SUCCEEDED`**, tay đã đứng yên, rồi mở tuner:

```bash
bash tools/tune_smc_terminal.sh --joint elbow --interface can0
```

Script này tự source `/opt/ros/humble/setup.bash`, `install/setup.bash`, dùng
domain 42 và giới hạn BLAS một thread. Không cần bật `autostart` bằng lệnh
riêng. File mặc định:

```text
src/gim_arm_controller_smc/config/smc_joint_tuning.yaml
```

## 4. Bài thử đầu tiên

Tại prompt `smc[elbow/id2]>`:

```text
show
test 3 6 3
```

`test DEG MOVE HOLD` có nghĩa:

1. Chụp **HOME bằng góc đo trước khi chuyển mode**; DEG là độ dịch chuyển
   tương đối từ HOME, không phải góc đích tuyệt đối.
2. Phát torque giữ trước khi switch; bàn giao torque trong 1 giây.
3. Giữ HOME bằng SMC và chờ đứng yên liên tục 0,3 giây trước quỹ đạo.
4. Đi từ tư thế đo sau bàn giao tới `HOME + DEG` trong MOVE giây.
5. Giữ đích HOLD giây, về HOME gốc trong MOVE giây, giữ HOME HOLD giây.
6. Xác nhận chuyển về position, lưu log/ảnh và mở đồ thị.

Ví dụ `test 3 6 3`: đi thêm 3°, đi/về mỗi chiều 6 s, giữ mỗi đầu 3 s.
Nếu HOME là −6° thì đích của `test 10 5 3` là +4°, không phải +10°.
Đóng các cửa sổ đồ thị để terminal trở lại prompt.

Khi lượt nhỏ giữ được HOME, đi/về êm và không abort, thử tiếp:

```text
test 5 6 3
test 10 5 3
```

Chỉ chạy từng lệnh sau khi đã xem kết quả lượt trước. Muốn dừng bài thử,
Ctrl-C **Terminal 2** và chờ xác nhận position; không Ctrl-C Terminal 1 trước.

## 5. Chỉnh những số nào?

Thứ tự vector YAML luôn là **base, shoulder, elbow**. Tuner chỉ sửa phần tử
của khớp đang chọn. SMC dùng:

```text
e = q đo - q mong muốn
ed = qdot đo - qdot mong muốn
s = ed + lambda * e
a_fb = -lambda * ed - ks * s - kr * sat(s / phi)
tau_ff = inverse_dynamics(q đo, qdot đo, qdd mong muốn) từ URDF runtime
tau_fb_raw = M(q đo) * a_fb
tau = clip(tau_ff + slew_limited(tau_fb_raw), trần torque)
```

Trong bài một khớp, gia tốc hiệu chỉnh của hai khớp position được đặt bằng 0.
Feedforward base được nhân `base_ff_alpha`; feedback không nhân hệ số này.
SMC này **không có tích phân**, không nhập Kp/Ki/Kd hay Q/R vào tuner.

| Tham số | Đơn vị | Tác dụng khi tăng |
|---|---|---|
| `lambda` | 1/s | Tăng phản hồi theo sai số vị trí và một phần vận tốc |
| `ks` | 1/s | Kéo mặt trượt về 0 mạnh hơn; tăng cả độ cứng và damping |
| `kr` | rad/s² | Tăng thành phần chống nhiễu/ma sát; quá lớn có thể gây giật |
| `phi` | rad/s | Mở rộng lớp chuyển mềm; giảm chattering nhưng sửa sai mềm hơn |
| `slew` | Nm/s | Cho phép feedback torque tăng/giảm nhanh hơn |
| `alpha` | không thứ nguyên | Nhân feedforward riêng base; chỉ chỉnh khi chọn base |

Trong lớp biên, độ cứng gia tốc tương đương là
`lambda * (ks + kr/phi)`, damping là `lambda + ks + kr/phi`. Các tham số liên
quan nhau; nên đổi **một tham số mỗi lượt**, thường 10–20%, giữ HOME/DEG/MOVE/HOLD
để so sánh. Terminal từ chối bộ không ổn định trong xấp xỉ ZOH 100 Hz; kiểm tra
này không thay thế việc đánh giá tay thật và các giới hạn đo được.

Ví dụ cú pháp cho elbow; các số dưới đây minh họa cách nhập, chưa phải gain
đã tune:

```text
set 6 8 1 0.15
ks 9
test 3 6 3
show
```

`set LAMBDA KS KR PHI` chỉnh cả bốn số. Có thể dùng riêng `lambda`, `ks`,
`kr`, `phi`, `slew`. `kr=0` được phép; các số khác phải hữu hạn và >0.

| Quan sát | Bước chỉnh nên thử |
|---|---|
| Giữ có lệch, đứng–trượt, sai số kéo dài | Tăng nhẹ `kr` hoặc `ks`, mỗi lần một số |
| Torque rung/giật quanh đích | Tăng `phi` trước; kiểm tra vận tốc đo và slew |
| Đi trễ nhưng vẫn êm | Tăng nhẹ `lambda` hoặc `ks` |
| Vượt đích/dao động | Giảm mức tăng vừa thử; nếu vẫn rung, tăng `phi` |
| Trôi ngay khi switch và chưa có APPROACH | Sửa khả năng giữ HOME trước; không tăng biên độ hay nới ngưỡng abort |

## 6. Lưu bộ đã đạt và chuyển khớp

Khi đạt yêu cầu, nhập **trước khi thoát**:

```text
show
save
quit
```

Kiểm tra file đã lưu:

```bash
cat src/gim_arm_controller_smc/config/smc_joint_tuning.yaml
```

Mở lại bằng `--joint` đúng khớp và `show` để xác nhận số đọc từ YAML. `reset`
bỏ bộ chưa lưu để đọc lại YAML; `save` ghi file máy, không ghi flash motor.

Đổi sang **shoulder/id1**:

1. Đợi test trả về position; thoát tuner và Ctrl-C Terminal 1.
2. Đặt/đỡ tay đúng zero trước khi khởi động lại.
3. Terminal 1 dùng lệnh launch ở mục 2, đổi thành `torque_joint:=shoulder`.
4. Terminal 2 kiểm tra controller, chạy lệnh HOME ở mục 3, chờ SUCCEEDED.
5. Mở tuner và thử nhỏ:

```bash
bash tools/tune_smc_terminal.sh --joint shoulder --interface can0
```

```text
show
test 3 6 3
```

Đổi sang **base/id0** theo cùng trình tự, dùng `torque_joint:=base` và:

```bash
bash tools/tune_smc_terminal.sh --joint base --interface can0
```

Base có trọng lực gây trôi về sau. Bộ khởi đầu dùng `base_ff_alpha=2.2` như
bộ PID/LQR trước; đây chưa phải hiệu chỉnh SMC đã xác nhận. Chỉ ở prompt base
mới dùng được `alpha`, ví dụ `alpha 2.3`. Giữ cùng tư thế để đánh giá tác động.

Có thể dùng `save` rồi `next` để giữ cùng tuner và chuyển elbow → shoulder →
base, nhưng **vẫn phải restart launch** với khớp tương ứng và về HOME. `next`
không tự đổi mode motor; `quit`/mở lại như trên dễ kiểm tra hơn cho người mới.

## 7. Đọc kết quả và bàn giao

Mỗi bài thử tạo:

```text
results/smc_joint_trials/smc_<joint>_<ngày_giờ>/
  tracking.csv                 góc, vận tốc, torque, sai số/mặt trượt và timing
  parameters.yaml              chính xác bộ số dùng trong lượt đó
  model.urdf                   URDF đọc từ controller_manager đang chạy
  trial.json                   HOME, đích, thời gian và điều kiện trước quỹ đạo
  tracking_q1_base.png
  tracking_q2_shoulder.png
  tracking_q3_elbow.png
```

Chọn ảnh đúng khớp: elbow=q3, shoulder=q2, base=q1. Nhìn cả chiều đi, giữ đích,
chiều về và HOME cuối. RMS toàn ảnh có cả phần giữ trước quỹ đạo; cú trôi ban
đầu có thể làm RMS tổng tăng dù phần bám đã tốt. Kiểm tra torque bị kẹp và
`slew_limited`, không chỉ nhìn đường vị trí.

Cột `tau` là lệnh torque do controller tính, không phải mô-men đo từ motor.
Trong bài một khớp, plugin chỉ gửi torque cho khớp đang tune; hai khớp position
bỏ qua các giá trị torque tương ứng trong vector lệnh.

Mở lại ảnh/log khi đã đóng tuner:

```bash
ros2 run gim_control plot_joint_tracking \
  /home/mirabo/GIM_Arm_3_DOF/results/smc_joint_trials/smc_elbow_THAY_BANG_THU_MUC_THAT/tracking.csv \
  --show
```

Thay đúng đường dẫn; không dùng biến log rỗng hoặc đường dẫn `.png` cho lệnh
này. Ghi lại bộ số, DEG/MOVE/HOLD, HOME, thư mục kết quả và nhận xét của lượt
đạt. Lưu `save` sau mỗi khớp; cuối buổi gửi YAML cùng các thư mục lượt đạt.

## 8. Khi gặp lỗi

| Thông báo/hiện tượng | Cách xử lý |
|---|---|
| `Không xác minh được torque_mode_enable` / `chưa cô lập` | Build lại description/hardware, restart launch đúng `torque_joint`; tuner đọc URDF runtime, không đọc file URDF cũ để đoán mode |
| `Trước test: JTC phải active...` | Xem `ros2 control list_controllers`; cần position active và effort inactive |
| `Một node khác đang phát effort` | Dừng controller/tuner cũ, chờ trả về position rồi chạy lại |
| `Tay đang chuyển động` / `chưa giữ HOME ổn định` | Xem qdot và trôi HOME; về 0 trong position, xử lý gain giữ/FF trước khi thử quỹ đạo |
| `Tay trôi khỏi HOME` | Bài chưa chạy; terminal trả về position. Đường mong muốn nằm ngang là bình thường trong trường hợp này |
| `sai số bám >...` | Bài abort do lỗi vị trí; xem log pha nào, về HOME rồi giảm biên độ/đi chậm hoặc sửa gain |
| `gần giới hạn khớp` | Không tiếp tục test cùng hướng; kiểm tra HOME, zero và đích theo URDF |
| `/joint_states cũ` | Kiểm tra CAN/domain và node phần cứng; không nới timeout để chạy với dữ liệu cũ |
| `Bỏ qua control tick...` | Chưa có mẫu state mới; giữ torque trước và tạm dừng thời gian quỹ đạo. Nếu nhiều lần, kiểm tra timing/CAN |
| `compute >8 ms` | Cảnh báo thời gian ở chu kỳ 10 ms; quá thường xuyên gây trễ cập nhật. SMC không dùng solver QP như MPC |
| Bộ số không ổn định ZOH | Không được áp dụng; giảm lambda/ks/kr hoặc tăng phi, rồi `show` lại |
| `Saved:` nhưng mở lại không thấy số | Kiểm tra đường dẫn YAML trong `show`, branch và `--params-file`; mỗi thuật toán dùng file riêng |

Các giới hạn hiện tại: torque `[5,40,5] Nm`, error tracking/transition
0,1 rad (~5,73°), margin cữ 0,05 rad, state timeout 0,25 s. Trước quỹ đạo phải
đứng yên (`|qdot|≤0,05 rad/s`) và cách HOME không quá 0,02 rad liên tục 0,3 s
sau bàn giao. Không nới các ngưỡng này chỉ để bỏ qua trôi/rung.

Nếu terminal đang báo **`Đang giữ node mô-men; chờ chuyển về position`**, giữ
terminal đó chạy và kiểm tra controller từ terminal khác. Khi controller
manager vẫn phản hồi, có thể phục hồi bằng:

```bash
ros2 control switch_controllers \
  --activate gim_arm_group_controller \
  --deactivate gim_arm_effort_controller --strict
ros2 control list_controllers
```

Chỉ gửi HOME sau khi đã xác nhận position active. Nếu CAN/nguồn bị mất, đỡ
tay và xử lý phần cứng; lệnh ROS không thể giữ motor đã mất điện.

## 9. Kết thúc buổi tune

1. Kết thúc test hoặc Ctrl-C tuner, chờ position active.
2. `show`, `save`, `quit`.
3. Về 0 bằng lệnh HOME trong position, chờ SUCCEEDED.
4. Đỡ tay rồi Ctrl-C Terminal 1/tắt nguồn theo thao tác phần cứng.
5. Bàn giao YAML đã lưu và đường dẫn ảnh/CSV của từng khớp; ghi rõ khớp nào
   chưa tune xong. Không coi bộ khởi đầu trong YAML là bộ đã đạt.
