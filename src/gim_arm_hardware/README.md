# Khởi động tay thật và đặt zero encoder

Plugin dùng SocketCAN, CAN ID khớp theo URDF: base=0, shoulder=1, elbow=2.
Launch `origin_gim_arm_control.launch.py` nhận các tham số:

- `can_interface:=can0` chọn interface đã được cấu hình trên Linux.
- `torque_joint:=all` mặc định giữ cấu hình `torque_mode_enable` trong URDF.
  `torque_joint:=base`, `shoulder` hoặc `elbow` chỉ cho khớp được chọn vào
  torque mode; hai khớp còn lại giữ bằng position mode tại góc chụp lúc switch.
  `torque_joint:=shoulder_elbow` cho CAN ID 1 và 2 dùng torque đồng thời,
  còn base CAN ID 0 giữ bằng position mode tại góc chụp lúc switch.
  Đây là giữ bằng motor, độ cứng hữu hạn, không phải khóa cơ khí. Tham số đọc
  khi khởi tạo phần cứng; khi đổi khớp phải khởi động lại launch.
- `set_zero_on_startup:=false` là mặc định; `true` thiết lập mốc zero cả ba khớp
  một lần trong vòng đời của instance phần cứng. Deactivate/activate lại
  cùng instance không reset; khởi động lại process sẽ reset nếu vẫn bật tùy chọn.
- `zero_method:=can` là mặc định, thử reset encoder bằng `0x019` và xác minh
  vị trí phản hồi về 0. `zero_method:=software` chụp vị trí encoder hiện tại
  thành offset cho ROS, không gửi `Set_Linear_Count` và không đổi encoder driver.

Trong log thử tay thật ngày 03/10/2026, cả ba node đã ở closed-loop và nhận
150 mẫu encoder, nhưng node 1 vẫn báo 0.235048 rev, node 2 báo -2.74429 rev
sau `0x019`. Với firmware đang thử, dùng `software` để thiết lập mốc ROS.

## CAN

Khi adapter đã xuất hiện dưới dạng SocketCAN `can0`, cấu hình 500 kbit/s:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

Bitrate của adapter và tất cả driver phải giống nhau. Lệnh launch không đổi
bitrate của driver. Chỉ dùng `1000000` sau khi các driver cũng được cấu hình
1 Mbit/s. Nếu adapter chỉ xuất hiện dưới dạng cổng serial, cần cấu hình
firmware/SLCAN để có interface SocketCAN trước.

## Đặt zero khi khởi động

Đặt và kê đỡ tay ở đúng tư thế hình học `q=[0,0,0]` của URDF, giữ đứng yên
trong quá trình đặt zero. Đây là gán mốc cho tư thế hiện tại, không phải
chạy tìm điểm home hay hiệu chuẩn encoder/motor. Motor sẽ nhả lực trong
giai đoạn IDLE và xác minh với mô-men 0.

```bash
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select gim_arm_hardware gim_control --symlink-install
source install/setup.bash

ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 \
  set_zero_on_startup:=true \
  zero_method:=software
```

Trình tự thực thi:

1. Gửi IDLE cho cả ba trục, chờ heartbeat xác nhận từng trục.
2. Đặt chế độ torque passthrough và lệnh mô-men 0. Vào closed-loop và chờ
   heartbeat xác nhận cả ba trục đã vào trạng thái này.
3. Với `zero_method:=can`: gửi `Set_Linear_Count` (`0x019`), giá trị signed int32 bằng 0, sau khi
   driver đã khởi tạo encoder trong closed-loop. Yêu cầu heartbeat closed-loop
   và ba mẫu encoder liên tiếp
   với `|position| <= 0.01 rev`, `|velocity| <= 0.02 rev/s` cho từng motor.
   Các ngưỡng này ở đơn vị encoder driver, trước phép quy đổi sang góc khớp.
   Với `zero_method:=software`: yêu cầu ba mẫu encoder liên tiếp có vị trí,
   vận tốc hữu hạn và `|velocity| <= 0.02 rev/s`; chụp vị trí làm mốc.
   Plugin lưu `offset = position_rev / gear_ratio * 2π * direction` trong RAM,
   đọc góc khớp bằng `q = raw_joint_angle - offset` và gửi lệnh vị trí bằng
   cách cộng lại offset trước khi quy đổi sang rev. `q=0` vì vậy giữ đúng
   tư thế vừa chụp. Offset này không đổi URDF và không được lưu qua restart.
4. Chốt setpoint bằng vị trí vừa đo rồi bật chế độ position passthrough.
   `gim_arm_group_controller` hoạt động, effort controller vẫn inactive.

Trong lúc chờ closed-loop và xác minh zero, lệnh mô-men 0 được gửi lại mỗi
20 ms. Mỗi giai đoạn chờ có timeout 1.5 s. Nếu gửi lỗi, axis báo lỗi hoặc không xác minh
được zero, plugin trả lỗi activation và cố gửi IDLE cho tất cả trục. Khi bus lỗi,
không thể bảo đảm driver nhận được IDLE. Một số encoder có thể đã reset trước
khi lỗi xuất hiện; không có rollback.

`zero_offset_rad` phải bằng 0 ở cả ba khớp khi bật tùy chọn; plugin từ chối
khởi tạo nếu đang dùng offset khác 0 để tránh áp dụng hai mốc cùng lúc.

Khi timeout, log in trạng thái heartbeat, số mẫu encoder, số mẫu zero hợp lệ
liên tiếp, vị trí (rev) và vận tốc (rev/s) của từng node. `state=8` với vị trí
còn lệch 0 nghĩa là đã vào closed-loop nhưng chưa xác minh được mốc;
`encoder_samples=0` nghĩa là chưa nhận được frame encoder trong giai đoạn đó.
Không nới ngưỡng zero để bỏ qua một encoder chưa reset. Launch tự dừng các
node còn lại khi `ros2_control_node` thoát, tránh để spawner chờ vô hạn.

PDF SteadyWin rev1.3 liệt kê `Set_Linear_Count` trong bảng CAN và mô tả
`index_offset` cho user zero qua odrivetool. Payload int32 của lệnh CAN được
đối chiếu với [CANSimple của ODrive](https://github.com/odriverobotics/ODrive/blob/master/Firmware/communication/can/can_simple.cpp).
Tính tương thích của firmware GIM đang lắp cần xác minh trên tay thật.
Plugin không gửi Save_Configuration và không giả định zero còn giữ sau mất nguồn.

Chờ log `Software zero verified` (hoặc `Encoder zero verified` khi dùng `can`)
cho đủ ba khớp rồi kiểm tra:

```bash
ros2 topic echo /joint_states --once
ros2 control list_controllers
```

Để khởi động theo luồng vị trí hiện có, bỏ `set_zero_on_startup:=true`.

## Cô lập một khớp để tune mô-men

Ví dụ tune khớp 3, giữ khớp 1 và 2 bằng position mode:

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 set_zero_on_startup:=true zero_method:=software \
  torque_joint:=elbow
```

Sau khi đưa cả ba khớp về zero bằng JTC, khởi động launch PID với `joint:=elbow`
rồi switch từ `gim_arm_group_controller` sang `gim_arm_effort_controller`.
Log phần cứng sẽ báo `base_joint` và `shoulder_joint` `KHOA o che do VI TRI`.
Chỉ elbow nhận lệnh mô-men; lệnh mô-men hai khớp còn lại bị bỏ qua.
Position mode trước và sau bài thử vẫn điều khiển được cả ba khớp.

## Chỉnh gain bên trong driver bằng terminal

File `scripts/tune_motor_gains.py` dùng thư viện chuẩn Python và SocketCAN,
không cần odrivetool hay python-can. Cấu hình `can0` như ở trên rồi chạy:

```bash
cd /home/minh/git_gim_ws/GIM_Arm_3_DOF
python3 src/gim_arm_hardware/scripts/tune_motor_gains.py --interface can0 --joint elbow
```

Chương trình không tự gửi gain khi mở, không đổi mode, không đặt zero và không
reboot. Lệnh `test` gửi quỹ đạo qua JTC; các lệnh chỉnh gain chỉ gửi CAN gain.
Giữ launch phần cứng đang chạy với `gim_arm_group_controller` active.

Tên dùng trong terminal ánh xạ tới cấu hình driver:

| Lệnh | Cấu hình driver | CAN command |
|---|---|---|
| `kpp` | `controller.config.pos_gain` | `Set_Pos_Gain`, `0x01A` |
| `kvp` | `controller.config.vel_gain` | `Set_Vel_Gains`, `0x01B` |
| `kvi` | `controller.config.vel_integrator_gain` | `Set_Vel_Gains`, `0x01B` |

Command ID từ manual SteadyWin rev1.3 mục 4.1.2. Payload dùng float32
little-endian theo callback `set_pos_gain_callback` và `set_vel_gains_callback`
trong [ODrive CANSimple fw-v0.5.4](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.4/Firmware/communication/can/can_simple.cpp).
Tính tương thích của firmware đang lắp cần đánh giá qua đáp ứng motor;
giao thức này không có lệnh đọc lại gain hoặc ACK cho việc chỉnh gain.

Ví dụ cú pháp (các số minh họa, chưa tune cho tay thật):

```text
show
set 20 0.16 0
test 5 6 3
kpp 22
test 5 6 3
kvp 0.18
kvi 0.02
test 5 6 3
show
quit
```

### Chạy thử ngay sau khi chỉnh gain

Lệnh `test DEG [MOVE HOLD]` chạy khớp đang chọn theo quỹ đạo bậc 5:
đi thêm `DEG` độ từ góc hiện tại trong `MOVE` giây, giữ `HOLD` giây,
về góc đầu trong `MOVE` giây rồi giữ thêm `HOLD` giây. Mặc định `MOVE=6`,
`HOLD=3`. Ví dụ từ elbow ở 0 độ:

```text
test 30 6 3
```

Trình tự: 0 → 30 độ trong 6 s → giữ 3 s → về 0 trong 6 s → giữ 3 s.
Góc thử là **độ dịch chuyển tương đối**, không phải góc tuyệt đối.
Hai khớp còn lại giữ góc chụp từ `/joint_states`, không tự chuyển về zero.
Quỹ đạo dùng mốc và quy đổi của plugin phần cứng, không gửi trực tiếp CAN
setpoint cạnh tranh với controller đang chạy. Node CAN tùy chỉnh ngoài 0/1/2
chỉ chỉnh gain, không hỗ trợ `test` trên mô hình ba khớp hiện tại.

Lệnh `test` cần terminal đã source:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Nếu đang ở effort mode, chuyển về position trước khi thử:

```bash
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate gim_arm_group_controller
```

Lệnh thử kiểm controller active, mẫu joint state mới, giới hạn góc/margin
0.05 rad và vận tốc URDF trước khi gửi goal. Nhấn Ctrl-C trong bài thử để
gửi yêu cầu hủy goal rồi trở về terminal chỉnh gain. Khi thành công chương
trình in kết quả action, RMS/MAX sai số bám, và đường dẫn CSV trong
`results/motor_gain_trials`. CSV ghi góc thực tế và góc đặt từ action feedback
(radian) cùng gain đã gửi trong phiên. Nếu hủy/abort, CSV giữ các mẫu đã nhận.
Các CSV này có schema riêng, không dùng với `plot_joint_tracking` của bài
tune torque. Dùng `--log-dir` hoặc `--urdf` để chọn thư mục log/URDF riêng.

Sau mỗi bài thử có feedback, một cửa sổ đồ thị tự mở với ba phần: góc đặt và
góc thực tế của khớp đang tune, sai số góc, và sai lệch giữ của hai khớp còn lại.
Góc hiển thị bằng độ; tiêu đề ghi gain đã gửi trong phiên để so sánh các lượt.
PNG được lưu cùng tên với CSV. Đóng cửa sổ đồ thị để trở về terminal và chỉnh
gain tiếp. Bài thử bị hủy/abort vẫn vẽ các mẫu đã nhận; nếu không có feedback
thì không tạo đồ thị. Lệnh `test` không cần thêm tham số để bật đồ thị.

Đồ thị cần matplotlib và backend GUI trên máy có màn hình. Nếu môi trường
chọn backend không có GUI, chương trình lưu PNG và báo đường dẫn để mở.
Thêm `--no-show` khi chạy công cụ để chỉ lưu PNG, không bật cửa sổ.

Sau một bài thử, sửa gain, nhập lại `test`, so sánh đáp ứng; chỉ gõ `save`
khi đã chọn được gain muốn lưu vào driver.

`Set_Vel_Gains` gửi **cả kvp và kvi trong một frame**. Lúc mới mở công cụ,
gain hiện tại chưa biết; nhập `vel KVP KVI` hoặc `set KPP KVP KVI` trước khi
chỉnh riêng kvp/kvi. Công cụ dùng giá trị còn lại đã gửi trong phiên cho cùng
CAN node, không tự đoán hoặc đặt nó bằng zero. Không dùng công cụ khác chỉnh
gain song song, vì giá trị lưu trong phiên khi đó có thể đã cũ.

`joint base`, `joint shoulder`, `joint elbow` chọn node 0, 1, 2; `node ID` chọn
node tùy chỉnh. Đổi node không gửi CAN, và mỗi node có lịch sử gain riêng.
`show` chỉ là giá trị đã gửi qua socket, chưa xác minh driver đã áp dụng.
`set` dùng hai frame, không phải một giao dịch nguyên tử; nếu frame thứ hai
lỗi, giá trị kpp đã gửi vẫn được giữ trong lịch sử và lỗi được báo ở terminal.

Chỉ gõ `save` khi muốn gửi `Save_Configuration` (`0x01F`) cho node đang chọn.
Đây là lưu cấu hình driver, không chỉ một gain. Không có ACK xác minh lưu
flash; chương trình không tự lưu khi đổi gain hay khi thoát.

Các giá trị này ở đơn vị driver, giống nhập bằng odrivetool. Chúng độc lập
với `kpp/kvp/kvi` trong `pid_hardware_tuning.yaml` trên PC. Gain position/velocity
của driver dùng khi motor chạy các vòng này; ở torque mode, `tau_fb` vẫn do
bộ PID trên PC tính theo YAML.

Kiểm tra giao diện và frame mà không mở CAN:

```bash
python3 src/gim_arm_hardware/scripts/tune_motor_gains.py --joint elbow --dry-run
```

Sau khi build package, cũng có thể chạy:

```bash
ros2 run gim_arm_hardware tune_motor_gains.py --interface can0 --joint elbow
```
