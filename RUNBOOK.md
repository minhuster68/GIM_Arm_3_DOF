# RUNBOOK — chạy và tune GIM Arm 3DOF

Quy trình vận hành. Phần giải thích *vì sao* nằm ở
[kinematics_test/LQI_README.md](kinematics_test/LQI_README.md).

---

## 0. Build

```bash
cd ~/git_gim_ws/GIM_Arm_3_DOF
colcon build --symlink-install && source install/setup.bash
```

`--symlink-install` đáng dùng: sửa URDF / YAML / file Python là có hiệu lực ngay,
chỉ khi sửa C++ mới phải build lại.

Kiểm tra plugin đúng là bản mới (phải ra số > 0):

```bash
strings install/gim_arm_hardware/lib/libgim_arm_system_hardware.so | grep -c Feedforward
```

> `ls -la` trên file `.so` trong `install/` cho thấy ngày của **symlink**, không
> phải của thư viện. Đừng dùng nó để kết luận build cũ hay mới.

---

## 1. Chạy trên BUS ẢO trước (không cắm động cơ)

Bước này chạy **đúng plugin C++ thật, đúng mã hoá CAN thật, đúng mô hình cascade
của driver** — chỉ thay động cơ bằng MuJoCo. Sai đơn vị, sai dấu, sai cấu hình
đều lộ ra ở đây mà không hỏng gì.

```bash
sudo modprobe vcan
sudo ip link add dev can0 type vcan     # bỏ qua nếu can0 THẬT đang cắm
sudo ip link set up can0
```

### Cách nhanh: A/B tự động

```bash
./tools/ab_feedforward.sh can0
```

Script tự làm cả 6 bước cho từng cấu hình (sửa URDF → build → khởi động lại
driver → khởi động lại ros2_control → **đợi encoder có số thật** → chạy quỹ đạo)
rồi in bảng so sánh. Nó **từ chối chạy nếu `can0` không phải vcan**.

Kết quả tham chiếu đo được trên máy này (RMS, độ):

| khớp | FF tắt | FF bật | cải thiện |
|---|---|---|---|
| base_joint | 0.213 | 0.056 | 3.8× |
| shoulder_joint | 0.095 | 0.015 | 6.2× |
| elbow_joint | 0.054 | 0.018 | 3.0× |

Chạy lại sẽ lệch khoảng ±20% (kernel không realtime — có dòng cảnh báo
`Could not enable FIFO RT scheduling`). **Dùng cột RMS để kết luận**; cột
"lớn nhất" là thống kê một mẫu nên nhiễu, có lần nó còn xấu đi trong khi RMS
tốt lên rõ.

### Cách thủ công: 4 cửa sổ terminal

```bash
# T1 — giả lập 3 driver GIM6010-8
python3 src/gim_arm_control/gim_control/gim6010_mujoco_sim.py --can can0 --viewer

# T2 — stack ROS thật
ros2 launch gim_control origin_gim_arm_control.launch.py

# T3 — xem feedforward trên dây
ros2 run gim_control sniff_can_ff can0

# T4 — chạy quỹ đạo
ros2 run gim_control origin_draw_trajectory
```

**Luôn kiểm trước khi đo:**

```bash
ros2 topic echo /joint_states --once      # KHÔNG được có .nan
```

Nếu ra `.nan` thì driver chưa cấp encoder — khởi động lại **cả** T1 và T2. Số
NaN vẫn cho ra bảng sai số trông bình thường, rất dễ tưởng là kết quả thật.

Ở T3, `torque_ff` quy về khớp phải xấp xỉ `G(q)`. Đối chiếu:

```bash
cd kinematics_test && python3 -c "
from arm_dynamics import ArmDynamics
print(ArmDynamics('gim_arm.urdf').gravity([-0.0001, 0.0084, 0.0291]).round(4))"
```

(thay 3 số bằng cột "vị trí (rad)" mà sniffer in ra). Lệch trong 1 LSB là đúng:
LSB = 0.001 Nm phía rotor, tức 0.008 Nm ở base/elbow và 0.064 Nm ở vai.

---

## 2. Lên tay thật

Làm **tay**, từng bước, luôn có người cầm nút dừng. Đừng dùng
`ab_feedforward.sh` (nó tự khởi động lại controller và tự cho tay chạy).

### 2.0 ĐIỀU KIỆN TIÊN QUYẾT: điểm 0 của encoder

Cả 3 khớp đang để `zero_offset_rad = 0` (plugin in ra lúc khởi động: *"mặc
định, không khai trong URDF"*). `G(q)` được tính từ **góc khớp trong hệ URDF**.
Nếu điểm 0 vật lý của encoder không trùng tư thế 0 của URDF thì mô hình đang
tính trọng lực **cho một tư thế khác** — và sai theo kiểu càng xa điểm 0 càng
lệch, chứ không phải lệch một hằng số.

`velocity_feedforward` không cần mô hình nên không ảnh hưởng. Nhưng
`gravity_feedforward` **phụ thuộc hoàn toàn** vào việc này.

Quy trình đặt điểm 0 có sẵn ở `src/gim_arm_mujoco/README` mục 4. Đặt xong thì
điền số đọc được vào `<param name="zero_offset_rad">` của từng khớp.

### 2.1 Kiểm mô hình bằng số — làm TRƯỚC KHI cho người đeo vào

Cho tay **không tải** đứng yên (ros2_control đang giữ vị trí, KHÔNG chạy quỹ
đạo), rồi:

```bash
ros2 run gim_control check_gravity_model can0
```

Công cụ hỏi `Get_Torques` (0x01C) để lấy mô-men giữ THẬT, đọc encoder để biết
tư thế, rồi so với `G(q)`. Nó kiểm cùng lúc **bốn** thứ mà không thứ nào tự lộ
ra khi chạy bình thường: dấu mô-men, điểm 0 encoder, tỉ số truyền, và bản thân
mô hình khối lượng.

Làm ở **3–4 tư thế khác nhau** (vươn ra trước tải nặng, co lại tải nhẹ). Một tư
thế trùng có thể là ăn may; ba tư thế trùng thì mô hình đúng.

| Cột tỉ lệ | Nghĩa là |
|---|---|
| ≈ **+1.0** | mô hình đúng, và cột đó chỉ ra quy ước đơn vị của firmware |
| ≈ **−1.0** | mô hình đúng nhưng **ngược dấu** → đổi dấu `torque_ff_rotor` trong `send_position_command()` |
| lệch xa 1, **khác nhau giữa các khớp** | điểm 0 encoder chưa hiệu chỉnh (xem 2.0) |
| `nan` | khớp đó gần như không chịu tải ở tư thế này — đổi tư thế rồi đo lại |

> ODrive trả `Get_Torques` ở phía **rotor**, còn bản giả lập trong repo quy sẵn
> về phía khớp. Chưa rõ firmware GIM6010-8 theo quy ước nào, nên công cụ in cả
> hai cách hiểu và để phép so tự chỉ ra cách đúng.

Trong lúc bring-up siết trần lại cho chặt:

```xml
<param name="max_torque_ff_rotor_nm">0.1</param>
```

Trọng lực thật lớn nhất chỉ ~0.057 Nm phía rotor nên 0.1 vẫn dư gấp đôi, mà
chặn được mọi trường hợp mô hình tính ra số vô lý. Nới về 0.625 sau khi tin.

### 2.2 Lấy chuẩn rồi bật dần

Mỗi lần đổi tham số: sửa `gim_arm.urdf` → `colcon build --packages-select
gim_arm_description` → **khởi động lại launch** (tham số chỉ đọc lúc `on_init`)
→ chạy `origin_draw_trajectory` → ghi bảng.

| Lần | velocity_ff | gravity_ff | Chờ đợi gì |
|---|---|---|---|
| 1 | false | false | chuẩn để so |
| 2 | **true** | false | RMS giảm mạnh nhất ở đây |
| 3 | true | **true** | giảm thêm ít hơn |

---

## 3. Đọc bảng để biết chỉnh gì

Bảng in ra 3 cột. **Cột "lệch TB" là cột phân loại vấn đề**, đọc nó trước.

| Triệu chứng | Nghĩa là | Làm gì |
|---|---|---|
| Lệch TB lớn, cùng dấu ở 1 khớp | sai số **có hệ thống** | kiểm `zero_offset_rad` khớp đó; kiểm dấu `torque_ff` |
| Lệch TB ≈ 0, RMS lớn, max/RMS ≈ 2–3 | **trễ bám** | bật `velocity_feedforward` |
| Lệch TB ≈ 0, RMS lớn, max/RMS > 5 | **dao động / nhiễu** | feedforward vô ích, phải chỉnh gain |
| Chỉ một khớp xấu hẳn | riêng khớp đó | ma sát, hoặc `gear_ratio`/`direction` sai |

Trên bus ảo, lệch TB ≈ 0 ở **cả hai** cấu hình. Nghĩa là sai số có hệ thống chưa
bao giờ là võng trọng lực — khâu tích phân vận tốc của ODrive đã khử nó. Cái tồn
tại là **trễ bám**, đúng thứ `vel_ff` xử lý. Trên tay thật có ma sát và rơ hộp
số thì cột lệch TB có thể khác hẳn — đó chính là thông tin cần.

### Chỉ chỉnh gain khi lệch TB đã ≈ 0 mà RMS vẫn lớn

Ba hệ số cascade nằm trong driver, sửa bằng `odrivetool`:

```python
odrv0.axis0.controller.config.pos_gain               # mặc định 20
odrv0.axis0.controller.config.vel_gain               # 0.16
odrv0.axis0.controller.config.vel_integrator_gain    # 0.32
odrv0.axis0.controller.config.input_filter_bandwidth # KIỂM giá trị này
```

Giờ `vel_ff` đã gánh phần vận tốc nên `pos_gain` không còn phải cao để giảm trễ
— **hạ xuống được cho êm hơn mà không mất độ bám**, điều trước đây không làm
được. Đổi một hệ số một lần, đo lại, ghi bảng.

`input_filter_bandwidth`: mô phỏng cho thấy `input_mode = 3` với bandwidth 2.0
làm sai số **xấu đi 22 lần**. Bạn từng đổi `input_mode` 3→1 mà thấy gần như
không khác — nếu bandwidth trên driver đã được đặt cao thì đó là lời giải thích.
Cứ giữ `input_mode = 1`.

> **Trọng số LQR (`q_int`, `q_pos`, `q_vel`, `r`) KHÔNG dùng ở đây.** Bước hiện
> tại chỉ dùng `G(q)` từ mô hình Lagrange làm feedforward; phần phản hồi vẫn là
> cascade của driver. Mục 4 của `LQI_README.md` chỉ áp dụng khi chuyển sang chế
> độ mô-men (`control_mode = 1`).

---

## 4. Sự cố hay gặp

| Hiện tượng | Nguyên nhân |
|---|---|
| `Unable to parse the value of parameter robot_description as yaml` | launch thiếu `ParameterValue(..., value_type=str)`. Đã sửa; nếu tái diễn là đang chạy launch file cũ. |
| `/joint_states` ra `.nan` | driver chưa cấp encoder. Khởi động lại **cả** driver lẫn ros2_control. |
| Controller không activate được | plugin cũ chưa có interface `velocity`. Build lại `gim_arm_hardware`. |
| `torque_ff = 0` ở mọi khớp | `gravity_feedforward` chưa `true`, hoặc chưa build lại `gim_arm_description`. Sniffer tự cảnh báo. |
| Bảng sai số ra RMS lớn mà lệch TB ≈ 0, max giống nhau ở cả 3 khớp | lệch trục thời gian khi đo, không phải lỗi điều khiển. |
