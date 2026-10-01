# BÁO CÁO TỔNG HỢP BỐN BỘ ĐIỀU KHIỂN GIM ARM 3-DOF

**PID – LQR – MPC – SMC trên ROS 2 Humble và Gazebo Classic**  
**Ngày tổng hợp:** 30/09/2026

## 1. Phạm vi và kết luận

Báo cáo này tổng hợp toàn bộ quá trình chuyển bốn bộ điều khiển từ mô hình
MATLAB sang kiến trúc ROS 2/Gazebo dùng chung cho tay máy GIM Arm 3-DOF. Nội
dung phân biệt rõ lỗi của thuật toán, lỗi rời rạc hóa, lỗi mô phỏng và lỗi vận
hành controller.

Hiện tại cả bốn thuật toán đều đã hoàn thành chu trình:

```text
HOME -> APPROACH -> TRACK -> RETURN -> HOLD_HOME
```

trên Gazebo trong các cấu hình đã kiểm chứng. Kết quả này chưa phải nghiệm thu
trên robot thật. PID đang cần vòng 2 kHz; LQR dùng torque/HOLD loop 200 Hz;
MPC còn worst-case tính toán vượt 10 ms; SMC chưa được xác minh với trễ CAN,
nhiễu encoder và sai mô hình thực.

## 2. Kiến trúc cuối cùng

```text
                     GIM ARM 3 DOF
                           |
          +----------------+----------------+
          |                                 |
    URDF + Gazebo                    Runner dùng chung
    ODE + payload                effort_controller_node
          |                                 |
          |               +-----------------+-----------------+
          |               |                 |                 |
          |           Quỹ đạo            Safety          CSV / plot
          |               |                 |                 |
          +---------------+-----------------+-----------------+
                                          |
                         +----------------+----------------+
                         |                |                |
                        PID              LQR              MPC              SMC
                         |                |                |                |
                         +----------------+----------------+----------------+
                                          |
                              torque khớp [Nm]
                                          |
                              ros2_control / plant
```

Runner dùng chung quản lý `/joint_states`, quỹ đạo, các phase, safety, giới
hạn mô-men và logger. Mỗi package thuật toán chỉ khai tham số và triển khai:

```python
compute(q, qd, q_ref, qd_ref, qdd_ref, dt) -> tau
```

Luồng vận hành an toàn:

```text
Gazebo khởi động với gravity = 0
  -> position controller giữ HOME
  -> node thuật toán nhận state và báo WAIT -> GRAVITY
  -> switch position -> effort
  -> bật autostart
  -> APPROACH -> TRACK -> RETURN -> HOLD_HOME
  -> switch effort -> position trước khi dừng node
```

## 3. Các vấn đề chung và cách xử lý

| Vấn đề | Triệu chứng / nguyên nhân | Cách xử lý |
|---|---|---|
| Code phân tán | LQR thực nằm ngoài package, còn wrapper và LQI cũ | Chuyển toàn bộ LQR vào package riêng, xóa symlink và code cũ |
| Khó so sánh thuật toán | Mỗi controller có thể tự quản lý quỹ đạo/safety khác nhau | Tạo runner, quỹ đạo, safety và logger dùng chung |
| Switch gây rung/rơi | Effort active khi nguồn torque chưa sẵn sàng | Effort load inactive; chờ `WAIT -> GRAVITY`; switch rồi mới bật autostart |
| Hand guiding bị hiểu nhầm | Gravity compensation không giữ cứng vị trí | `true` để kéo tay; `false` chụp vị trí hiện tại và giữ vòng kín |
| Xung lực giả trong Gazebo | Collision mesh SolidWorks lõm và giao nhau | Chỉ bỏ collision khỏi URDF runtime của Gazebo |
| Ghế xuyên mặt đất | Mesh thấp hơn mặt phẳng z=0 | Nâng `base_footprint` thêm 0,300 m |
| Robot rơi khi load | Gravity bật trước khi position controller giữ HOME | Gravity ban đầu bằng 0; gửi HOME xong mới bật -9,81 m/s² |
| Timer không đều | `/clock` Gazebo Classic phát theo cụm | Dùng `use_sim_time=false` cho node torque |
| State cũ gây cú giật | Timer tính nhiều lần trên cùng `/joint_states` trong khi reference tiến | Giữ torque cũ, đóng băng trajectory và phục hồi có damping |
| Plot không tìm thấy CSV | Sai tên/path hoặc node chưa tạo xong file | Dùng path tuyệt đối trong `results/`, đóng node sạch trước khi plot |
| Thử payload khó quản lý | Phải sửa URDF cho mỗi tải | Sinh payload runtime bằng `payload_mass_kg` tại tool tip |

### 3.1 Bảo vệ khi `/joint_states` bị gián đoạn

Một scheduler stall có thể làm timer callback chạy trước các JointState đang
xếp hàng. Nếu tiếp tục tính controller trên state cũ trong khi reference vẫn
tiến, feedback, tích phân và MPC warm-start đều bị lệch.

Logic mới:

```text
Không có state mới
  -> giữ torque trước
  -> đóng băng phase_elapsed
  -> nếu mất state >= 30 ms
  -> gravity + PD damping trong 0,25 s
  -> reset integrator / warm-start
  -> tiếp tục đúng điểm quỹ đạo cũ
```

Phép thử tiêm gián đoạn xác nhận: chỉ giữ torque mà không phục hồi vẫn có thể
ABORT lúc state trở lại; thêm 0,25 s damped recovery thì controller tiếp tục
quỹ đạo bình thường.

## 4. Cascade PID

### 4.1 Luật điều khiển

```text
e_q       = q_ref - q
qdot_cmd  = qdot_ref + Kpp * e_q
e_qdot    = qdot_cmd - qdot
tau       = inverse_dynamics(reference)
            + Kt * (Kvp * e_qdot + Kvi * integral(e_qdot))
```

### 4.2 Vấn đề

Gain MATLAB gốc `Kpp=[46,60,36]`, `Kvp=[8,9.5,10]`,
`Kvi=[7.6,7.6,7.6]` có lúc bám góc đẹp nhưng vận tốc rung và mô-men chạm trần.
Profile này chạy được trong một số lượt không có nghĩa tín hiệu đủ sạch. Khi
thử ở 100 Hz, profile smooth cũng từng ABORT khoảng 1,29 s sau khi vào TRACK.

PID còn kích mode số của ODE giống LQR khi Quick solver chỉ dùng 50
iterations, chứng minh một phần rung nằm ở plant mô phỏng chứ không thuộc
riêng một thuật toán.

### 4.3 Cách sửa

- Giữ `pid.yaml` làm profile MATLAB tham chiếu.
- Tạo `pid_gazebo_smooth.yaml` với `Kpp=[18,24,14]`, `Kvp=[2,3,2.5]`,
  `Kvi=[1.5,1.5,1.5]`.
- Chạy PID smooth ở 2 kHz trên Gazebo.
- Thêm anti-windup khi torque bão hòa.
- Sau RETURN dùng gravity-hold PID mềm chung thay vì cascade PID cứng.
- Tăng ODE Quick solver lên 200 iterations.

### 4.4 Kết quả không tải trong TRACK

| Chỉ tiêu | q1 | q2 | q3 |
|---|---:|---:|---:|
| RMS sai số góc | 0,054° | 0,278° | 0,069° |
| Max sai số góc | 0,217° | 0,701° | 0,208° |
| Max sai số vận tốc | 0,042 | 0,060 | 0,039 rad/s |
| Max mô-men | 0,769 | 3,625 | 1,542 Nm |

PID đạt trên Gazebo 2 kHz nhưng chưa thể chuyển nguyên cấu hình sang hardware
100 Hz.

## 5. LQR

### 5.1 Profile CARE MATLAB không phù hợp ZOH 100 Hz

Gain MATLAB được thiết kế bằng CARE liên tục. Khi giữ torque bằng ZOH 10 ms,
kiểm tra cho `|z|max khoảng 24,9 > 1`, tức profile thực sự phân kỳ trong
implementation rời rạc.

Cách sửa:

- Tạo `lqr_safe_100hz.yaml` dùng DARE trên mô hình ZOH.
- Tăng `tau_penalty_scale` lên 512 để giảm băng thông feedback.
- Ghép trọng số tích phân với giới hạn integral của runner.
- Kiểm tra bán kính phổ trước khi chạy.
- Dùng gravity tại vị trí đo được trong profile an toàn.

Tên `safe_100hz` là tên lịch sử: torque/HOLD loop hiện chạy 200 Hz, còn bảng
gain cập nhật mỗi hai chu kỳ nên schedule vẫn ở 100 Hz.

### 5.2 Tránh giải Riccati trong callback

Toàn bộ bảng gain được precompute khi position controller còn giữ robot.
Runtime chỉ tra gain theo `[q_ref, qd_ref]` bằng KD-tree. Cách này tránh giải
DARE trong callback 5 ms và tránh chọn sai gain khi wall time bị trễ.

### 5.3 Cú giật khoảng giây 17

Các giả thuyết từng được thử gồm đổi gain schedule, torque clipping,
friction, armature, gravity feedforward, CARE/DARE và sai vận tốc. Giữ cố định
K_121 hoặc K_122 vẫn lỗi; vận tốc bất thường xuất hiện trước torque clipping.

Nguyên nhân đo được là ODE Quick solver với 50 iterations chưa hội tụ đủ cho
mô hình có tỉ số khối lượng/quán tính lớn:

```text
physics 2 kHz -> mode vận tốc 191,3 Hz
physics 4 kHz -> mode vận tốc 388,4 Hz
```

Tần số gần gấp đôi theo bước tích phân chứng minh đây là mode số khóa theo
solver, không phải mode cơ học thật hoặc cực LQR. Khi mode đã bắt đầu, vận tốc
shoulder vượt giới hạn URDF; cơ chế cắt lực của Gazebo làm rung nặng thêm.

Sửa bền vững là giữ physics 2 kHz và tăng ODE Quick solver từ 50 lên 200
iterations. Không dùng low-pass để che rung và không nới safety threshold.

### 5.4 Rung ngay sau switch

Đây là lỗi độc lập với mode 191 Hz. Gravity/HOLD loop 100 Hz không đủ ổn định
cho các khớp quán tính thấp. Sau khi chuyển torque/HOLD lên 200 Hz, chụp HOME
trước switch và bắt đầu APPROACH từ state thực, GRAVITY giữ ổn định và LQR
hoàn thành toàn quỹ đạo.

### 5.5 Kết quả không tải trong TRACK

| Chỉ tiêu | q1 | q2 | q3 |
|---|---:|---:|---:|
| RMS sai số góc | 0,019° | 0,024° | 0,031° |
| Max sai số góc | 0,120° | 0,161° | 0,217° |
| Max sai số vận tốc | 0,031 | 0,040 | 0,046 rad/s |
| Max mô-men | 0,780 | 3,631 | 1,547 Nm |

LQR hiện cho sai số bám tốt nhất trong các log không tải, nhưng torque loop
200 Hz chưa khớp hardware 100 Hz.

## 6. MPC

### 6.1 Cấu trúc

MPC dùng `Np=40`, `Nc=10`, `dt=0,01 s`. Mỗi chu kỳ linearize inverse
dynamics quanh reference, dự báo 0,4 s, giải QP có giới hạn tổng torque và
slew-rate, rồi chỉ phát move đầu tiên.

### 6.2 SLSQP quá sát deadline

SLSQP từng cần khoảng 6-10 ms trong chu kỳ 10 ms. Khi solver hoặc scheduler
chậm hơn bình thường, callback trễ và hệ có thể ABORT.

Cách sửa:

- Thêm ADMM cho QP 30 biến.
- Dùng Cholesky cho hệ tuyến tính nhỏ.
- Warm-start từ nghiệm chu kỳ trước.
- Tự cân bằng `rho`.
- Project causal để torque và slew luôn khả thi.
- Nếu candidate không tốt hơn warm-start thì giữ nghiệm khả thi cũ.

### 6.3 Sửa mô hình tuyến tính hóa

Linearization cũ chỉ vi phân gravity và Coriolis, bỏ qua ảnh hưởng của
`d(M(q) * qdd_ref)/dq` khi reference chuyển động. Bản mới vi phân toàn bộ
`inverse_dynamics(q_ref, qd_ref, qdd_ref)` theo q và qdot.

### 6.4 Khả năng khử tải

Profile Gazebo tăng `q_integral` từ `[2,2,2]` lên `[128,128,128]` để khử tải
không biết trước trong horizon ngắn. `mpc.yaml` vẫn giữ bộ MATLAB để đối chiếu;
`mpc_gazebo_safe.yaml` dùng ADMM.

### 6.5 Cú giật khi đang TRACK

Đây là cơ chế stale-state của runner, khác mode 191 Hz của ODE. Timer đã từng
tính lại MPC nhiều lần trên cùng state trong khi trajectory tiến. Bảo vệ state
sequence và damped recovery đã loại cơ chế này trong lượt kiểm chứng sau.

### 6.6 Kết quả không tải trong TRACK

| Chỉ tiêu | q1 | q2 | q3 |
|---|---:|---:|---:|
| RMS sai số góc | 0,044° | 0,108° | 0,156° |
| Max sai số góc | 0,273° | 0,333° | 0,622° |
| Max sai số vận tốc | 0,077 | 0,032 | 0,062 rad/s |
| Max mô-men | 0,904 | 3,643 | 1,543 Nm |

Thời gian compute trong log mới: median khoảng 2,05 ms, p99 khoảng 3,13 ms,
max khoảng 10,06 ms.

### 6.7 Kết quả với payload 0,5 kg

```text
Max sai số TRACK = [0,644; 0,417; 1,045] độ
Max torque       = [1,074; 6,360; 3,533] Nm
Compute p99      = 5,31 ms
Compute max      = 18,26 ms
```

QP sau projection vẫn thỏa constraint, nhưng dưới payload nhiều chu kỳ ADMM
chưa đạt tolerance chặt trong 100 iterations. MPC đã hoàn thành Gazebo nhưng
chưa đạt tiêu chí hard-real-time cho robot thật.

## 7. SMC

### 7.1 Luật điều khiển

```text
e       = q - q_ref
edot    = qdot - qdot_ref
s       = edot + lambda * e
sat     = clip(s / phi, -1, 1)
qdd_cmd = qdd_ref - lambda * edot - ks * s - kr * sat
tau     = inverse_dynamics(q, qdot, qdd_cmd)
```

### 7.2 Profile MATLAB mất ổn định sau rời rạc hóa

Profile gốc:

```text
lambda = [15, 85, 155]
ks     = [5, 137, 180]
kr     = [30, 130, 90]
phi    = [0,03; 0,05; 0,15]
```

Trong lớp biên, `alpha = ks + kr/phi = [1005,2737,780] 1/s`. Với ZOH 10 ms,
bán kính phổ xấp xỉ `[9,814; 38,260; 13,565]`, lớn hơn 1 ở cả ba khớp. Vì vậy
tăng trần mô-men không thể sửa mất ổn định này.

### 7.3 Cách sửa

Profile rời rạc 100 Hz đã chọn:

```text
lambda = [30, 35, 40]
ks     = [25, 30, 30]
kr     = [40, 100, 100]
phi    = [2, 5, 4]
```

Khi đó `alpha=[45,50,55]` và `rho=[0,7703;0,7427;0,7168] < 1`.

- Tạo `smc_gazebo_safe.yaml`, giữ `smc.yaml` làm MATLAB reference.
- Factory tính và in `rho_ZOH_ideal` khi khởi động.
- Cảnh báo rõ nếu `rho >= 1`.
- Dùng HOLD damping mềm để tránh kích vận tốc lúc switch.

### 7.4 Kết quả không tải trong TRACK

| Chỉ tiêu | q1 | q2 | q3 |
|---|---:|---:|---:|
| RMS sai số góc | 0,072° | 0,180° | 0,223° |
| Max sai số góc | 0,168° | 0,255° | 0,338° |
| Max sai số vận tốc | 0,058 | 0,034 | 0,043 rad/s |
| Max mô-men | 0,808 | 3,622 | 1,535 Nm |

### 7.5 Kết quả payload 0,5 kg

```text
tau_scale        = 0,75
Max sai số TRACK = [0,641; 0,686; 2,472] độ
Max torque       = [1,074; 6,374; 3,524] Nm
```

Elbow cần khoảng 3,52 Nm để cân bằng tải, nên `tau_scale=0,50`, tương đương
trần elbow 2,50 Nm, chắc chắn không đủ.

## 8. So sánh kết quả không tải

Các lượt chạy không hoàn toàn cùng control frequency, vì vậy bảng chỉ dùng để
nhìn tổng quan, không phải xếp hạng tuyệt đối.

| Controller | Tần số | Max sai số TRACK q1/q2/q3 | Max sai số vận tốc | Max torque |
|---|---:|---:|---:|---:|
| PID smooth | 2000 Hz | [0,217; 0,701; 0,208]° | [0,042; 0,060; 0,039] | [0,769; 3,625; 1,542] Nm |
| LQR safe | 200 Hz, gain 100 Hz | [0,120; 0,161; 0,217]° | [0,031; 0,040; 0,046] | [0,780; 3,631; 1,547] Nm |
| MPC ADMM | 100 Hz | [0,273; 0,333; 0,622]° | [0,077; 0,032; 0,062] | [0,904; 3,643; 1,543] Nm |
| SMC safe | 100 Hz | [0,168; 0,255; 0,338]° | [0,058; 0,034; 0,043] | [0,808; 3,622; 1,535] Nm |

Nhận xét:

- LQR bám góc tốt nhất trong log không tải hiện tại.
- SMC là profile 100 Hz gọn, nhanh và ổn định trên Gazebo.
- MPC xử lý constraint rõ ràng nhưng còn worst-case deadline.
- PID đơn giản, ổn định nhưng profile hiện tại phụ thuộc vòng 2 kHz.

## 9. Phân loại các nguồn gây giật

| Hiện tượng | Nguyên nhân đã xác định |
|---|---|
| LQR giật lặp lại gần giây 17 | Mode số ODE Quick solver 50 iterations |
| Rung ngay sau switch position sang effort | Gravity/HOLD 100 Hz quá chậm và chuyển reference chưa mượt |
| PID/LQR/MPC đôi lúc giật sau scheduler stall | Timer dùng lại state cũ trong khi reference tiếp tục tiến |
| SMC bật lên là phân kỳ nhanh | Gain MATLAB không ổn định sau rời rạc hóa 100 Hz |
| Gắn tải 0,5 kg làm elbow tụt | Torque gravity cần lớn hơn giới hạn `tau_scale=0,50` |

Không có một lỗi chung duy nhất cho mọi cú giật. Quá trình chẩn đoán đã tìm
thấy ít nhất bốn cơ chế độc lập: solver vật lý, tần số HOLD/switch, độ tươi của
state và gain rời rạc của thuật toán.

## 10. Trạng thái hiện tại và bước tiếp theo

| Thành phần | Trạng thái |
|---|---|
| Gazebo plant | Đã ổn định cho bài toán động lực học khớp |
| Controller switching | Đã có quy trình an toàn |
| Quỹ đạo/safety/logger dùng chung | Đã hoàn thành |
| Payload runtime | Đã hoạt động |
| PID trên Gazebo | Đạt ở 2 kHz |
| LQR trên Gazebo | Đạt với torque loop 200 Hz |
| MPC trên Gazebo | Đạt; còn worst-case compute và hội tụ ADMM |
| SMC trên Gazebo | Đạt ở 100 Hz, không tải và tải 0,5 kg |
| Robot thật 100 Hz | Chưa nghiệm thu |
| Chạy với người | Chưa được phép |

Trước khi chuyển sang hardware cần thực hiện lại theo đúng vòng CAN 100 Hz,
đo latency state-to-command, xác nhận watchdog và emergency stop, giảm dần
torque limit, thử không tải không có người, sau đó mới tăng payload từng bước.

## 11. Tài liệu và file chính

- `CONTROLLER_ARCHITECTURE.md`: kiến trúc runner và bốn package.
- `LQR_JERK_DIAGNOSIS_20260928.md`: bằng chứng chẩn đoán mode ODE 191 Hz.
- `src/gim_arm_control/gim_control/effort_controller_node.py`: state machine,
  safety, stale-state recovery và logger.
- `src/gim_arm_control/gim_control/controller_api.py`: giao diện chung.
- `src/gim_arm_control/worlds/gim_arm_zero_gravity.world`: ODE 2 kHz,
  Quick solver 200 iterations.
- `src/gim_arm_controller_pid/`: Cascade PID.
- `src/gim_arm_controller_lqr/`: TVLQR/DARE.
- `src/gim_arm_controller_mpc/`: MPC QP, SLSQP và ADMM.
- `src/gim_arm_controller_smc/`: SMC và kiểm tra ổn định ZOH.
- `results/`: CSV và đồ thị của các lượt chạy đã lưu.

---

**Ghi chú an toàn:** Các kết quả trong báo cáo là kết quả mô phỏng Gazebo
Classic. Collision mesh đã được bỏ trong URDF runtime, do đó đây là mô phỏng
động lực học khớp, không phải mô phỏng tương tác an toàn với người hoặc ghế.
