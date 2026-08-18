# Bộ điều khiển LQI cho GIM Arm 3DOF

Tài liệu này giải thích bộ LQI mới: đã làm gì, dựa trên cơ sở nào, kết quả đo
được ra sao, và cách chỉnh (tune).

**Không có gì của bộ PID cũ bị xoá hay sửa.** Bộ PID (cascade của ODrive) vẫn
nguyên vẹn trong `gim6010_mujoco_sim.py` và `gim_arm_system.cpp`; file so sánh
`import` thẳng lớp gốc đó chứ không chép lại, nên số liệu "PID" trong bảng đúng
là bộ đang chạy trên tay máy.

---

## 1. Các file mới

| File | Việc |
|---|---|
| `arm_dynamics.py` | Mô hình Lagrange `M(q)q̈ + C(q,q̇)q̇ + G(q) = τ` lấy từ Pinocchio đọc URDF, cộng thêm quán tính rotor + ma sát. Có hàm tự kiểm chứng với MuJoCo. |
| `lqi_controller.py` | Bộ LQI: tuyến tính hoá phản hồi + LQR có khâu tích phân, giải Riccati bằng `scipy`. |
| `compare_pid_lqi.py` | Bàn so sánh PID cascade với LQI: cùng vật lý, cùng quỹ đạo, in bảng + vẽ đồ thị. |
| `mit_mode.py` | Rã luật LQI thành 5 trường của `Mit_Control` (0x008), có mô phỏng lượng tử hoá 12 bit y như frame thật. |
| `compare_architectures.py` | So 6 kiến trúc điều khiển để **chọn** cách đưa lên tay thật. |
| `test_failsafe.py` | Mô phỏng máy chủ chết giữa chừng, đo tay trôi bao nhiêu ở từng chế độ. |
| `LQI_README.md` | Tài liệu này. |

Chạy nhanh:

```bash
cd kinematics_test
python3 arm_dynamics.py           # tự kiểm mô hình với MuJoCo
python3 lqi_controller.py         # tự kiểm bộ điều khiển + bảng ảnh hưởng trọng số
python3 compare_pid_lqi.py        # PID vs LQI, xuất compare_pid_lqi.png
python3 compare_architectures.py  # bảng chọn kiến trúc để lên tay thật
python3 test_failsafe.py          # máy chủ chết thì tay làm gì
```

---

## 2. Vì sao phải có mô hình Lagrange trước

Tay máy là hệ **phi tuyến**, còn LQR/LQI là lý thuyết cho hệ **tuyến tính**.
Không thể áp thẳng LQR lên tay máy. Cách nối hai thứ đó là chia làm 2 tầng.

### Tầng 1 — tuyến tính hoá phản hồi (computed torque)

Phương trình Lagrange của tay máy:

```
M(q) q̈ + C(q,q̇) q̇ + G(q) = τ
```

Nếu chọn mô-men theo công thức:

```
τ = M(q)·u + C(q,q̇)q̇ + G(q)
```

thì thay vào phương trình trên, **toàn bộ phần phi tuyến triệt tiêu**, còn lại
đúng:

```
q̈ = u
```

Nghĩa là sau tầng 1, mỗi khớp chỉ còn là một **khâu tích phân kép**, và 3 khớp
**tách rời hẳn nhau** — không còn xen kênh, không còn Coriolis, không còn
trọng lực.

Đây chính là chỗ `M`, `C`, `G` được dùng, và cũng là chỗ LQI hơn PID về bản
chất: quán tính hiệu dụng của `base_joint` thay đổi **5.56 lần** trên vùng làm
việc (đo được, in ra khi chạy `arm_dynamics.py`). PID buộc phải dùng **một** bộ
hệ số cố định cho cả dải đó — chỉnh cứng cho tư thế nặng thì tư thế nhẹ sẽ vọt
lố, chỉnh mềm cho tư thế nhẹ thì tư thế nặng bám không kịp. LQI tính lại `M(q)`
mỗi chu kỳ nên không phải đánh đổi.

### M, C, G lấy từ Pinocchio — và đã kiểm chứng

Đúng như bạn nghĩ, `M`/`C`/`G` lấy thẳng từ URDF qua Pinocchio, không dẫn công
thức bằng tay:

| Ký hiệu Lagrange | Hàm Pinocchio |
|---|---|
| `M(q)` | `pin.crba` |
| `C(q,q̇)q̇ + G(q)` | `pin.nonLinearEffects` |
| `G(q)` | `pin.computeGeneralizedGravity` |
| `τ = M q̈ + C q̇ + G` | `pin.rnea` |

Lý do dùng thư viện: CRBA/RNEA **chính là** phương trình Lagrange được tổ chức
lại cho máy tính, chạy O(n) thay vì O(n³) và không sai dấu khi khai triển tay.
Khối lượng / tâm khối / ma trận quán tính từng link đã có sẵn trong URDF (xuất
từ CAD).

Nhưng tôi không tin thư viện suông. `arm_dynamics.py` có hàm kiểm chứng so `M`
và `C q̇ + G` của Pinocchio với `mj_fullM` và `qfrc_bias` của MuJoCo trên 20 cấu
hình ngẫu nhiên. Hai thư viện cài đặt hoàn toàn độc lập nhau:

```
sai lệch M lớn nhất      = 8.451e-10   (M cỡ 0.03..0.16)
sai lệch (Cq̇+G) lớn nhất = 5.304e-10   (bias cỡ 0.5..2 Nm)
```

### Ba thứ URDF không có, phải thêm tay

1. **`armature = J_rotor · N²`** — quán tính rotor phản chiếu qua hộp số. Ở
   `shoulder` (N=64) phần này là 0.0108 kg·m², bằng **7%** quán tính link của
   chính khớp đó. Bỏ qua là mô hình sai 7% ở khớp nặng nhất, sai theo kiểu cộng
   thẳng vào đường chéo `M` nên đi trực tiếp vào mô-men tính ra.
2. **Ma sát** nhớt + khô của hộp số — lấy đúng số trong `gim6010_mujoco_sim.py`.
3. **Giới hạn mô-men** — đọc từ `<limit effort>` của URDF (5/40/5 Nm).

> Điểm yếu đã biết: hai hệ số ma sát trong repo đang ghi rõ là *"SỐ ĐẶT TẠM,
> chưa hiệu chỉnh từ log CAN thật"*. Đây là phần yếu nhất của mô hình — và
> cũng chính là lý do cần khâu tích phân.

### Tầng 2 — LQI trên hệ đã tuyến tính hoá

Đặt `e = q - q_ref` và `u = q̈_ref + v` (phần `q̈_ref` là feedforward của quỹ
đạo) thì `ë = v`. Thêm biến tích phân `xi = ∫e dt`:

```
x = [xi, e, ė]ᵀ          ẋ = A x + B v

A = [[0,1,0],            B = [0,
     [0,0,1],                 0,
     [0,0,0]]                 1]
```

LQR cực tiểu hoá `J = ∫ (xᵀQx + r·v²) dt`, nghiệm là `v = -Kx` với `K` giải từ
phương trình Riccati đại số (`scipy.linalg.solve_continuous_are`).

Vì sau tầng 1 ba khớp đã tách rời và **giống hệt nhau**, bài toán 9 trạng thái
rã thành 3 bài toán 3 trạng thái giống nhau → chỉ cần giải Riccati một lần.

### Vì sao LQI chứ không phải LQR

LQR thuần cho **sai số bám xác lập khác 0** mỗi khi có mô-men mà mô hình không
biết. Ở tay máy này thành phần đó có thật và không nhỏ: ma sát khô ước lượng
0.256 Nm ở shoulder, với hệ số chưa hiệu chỉnh. Khâu tích phân `xi` triệt tiêu
đúng loại sai số này. Giá phải trả: thêm 1 cực (dễ vọt lố hơn) và phải chống
bão hoà tích phân.

**Chống bão hoà (anti-windup):** khi mô-men đã chạm giới hạn 5/40/5 Nm mà sai số
vẫn cùng chiều thì **ngừng tích luỹ** `xi`. Không có bước này, `xi` phình lên
trong lúc bão hoà rồi đẩy tay vọt qua bên kia khi thoát bão hoà. Thêm một lớp
kẹp cứng `|xi| ≤ i_limit` phòng trường hợp kẹt cơ khí.

---

## 3. Kết quả đo

Điều kiện giữ giống nhau tuyệt đối cho cả hai bộ: cùng `ArmPhysics` (MuJoCo +
armature + ma sát nhớt + ma sát khô), cùng bước thời gian 1/2000 s, cùng quỹ
đạo `sweep_trajectory.py`, cùng giới hạn mô-men, cùng tư thế xuất phát. Đo trên
**vòng cuối** để loại quá độ khởi động.

Chỗ cố ý khác nhau, vì đó là bản chất hai kiến trúc: PID cascade nhận lệnh **vị
trí** và chạy trong driver ở tần số cao (2 kHz trong mô phỏng, 8 kHz trên driver
thật), còn LQI tính **mô-men** trên máy chủ ở 100 Hz (đúng `update_rate` trong
`controllers.yaml`). LQI **bị thiệt 20 lần về tần số vòng lặp**.

| Chỉ số (vòng cuối) | PID cascade | LQI 100 Hz | LQI 1 kHz |
|---|---|---|---|
| **Sai số đầu tay RMS (mm)** | **2.066** | **0.106** | 0.106 |
| Sai số đầu tay lớn nhất (mm) | 2.356 | 0.408 | 0.401 |
| Sai số `base_joint` RMS (độ) | 0.187 | 0.009 | 0.009 |
| Sai số `shoulder_joint` RMS (độ) | 0.089 | 0.026 | 0.026 |
| Sai số `elbow_joint` RMS (độ) | 0.050 | 0.038 | 0.037 |
| Mô-men RMS vai (Nm) | 3.323 | 3.320 | 3.320 |
| Mô-men đỉnh vai (Nm) | 4.216 | 3.656 | 3.656 |
| Thời gian bão hoà mô-men | 0% | 0% | 0% |

**Sai số bám đầu tay giảm 19.4 lần** (2.066 mm → 0.106 mm).

Ba điều đáng chú ý hơn con số tổng:

1. **Mô-men RMS gần như y hệt nhau** (3.320 so với 3.323 Nm ở vai). LQI không
   "ăn gian" bằng cách đạp mô-men mạnh hơn — nó dùng đúng chừng đó năng lượng
   nhưng đặt đúng chỗ, đúng lúc. Mô-men đỉnh còn *thấp hơn* 13%.
2. **LQI ở 1 kHz gần như không hơn LQI ở 100 Hz** (0.106 so với 0.106 mm). Đây
   là phép thử tôi cố tình dựng để trả lời: phần hơn đến từ **mô hình**, không
   phải từ tần số. Nghĩa là chạy LQI trong `ros2_control` ở 100 Hz là đủ, không
   cần vòng điều khiển thời gian thực tần số cao.
3. **Dạng sai số khác hẳn nhau** (xem `compare_pid_lqi.png`): PID cho đường
   cong trơn bám theo hình quỹ đạo — đó là **trễ bám + võng do trọng lực**, tức
   sai số có hệ thống. LQI cho sai số dạng bậc thang phẳng — đó là **dính-trượt
   do ma sát khô**, khâu tích phân dồn lên tới khi bứt ra.

### Một phát hiện ngược với trực giác: bù ma sát làm HẠI

Ban đầu tôi để `friction_ff=True` (bù ma sát bằng mô hình). Kết quả đo:

| Cấu hình bù ma sát | Sai số đầu tay RMS |
|---|---|
| **Tắt hoàn toàn** | **0.1064 mm** |
| Bật, `smooth_eps=0.02` | 0.1559 mm (xấu hơn 47%) |
| Bật, `smooth_eps=0.1` | 0.1379 mm |
| Bật, `smooth_eps=0.5` | 0.1012 mm (hơn 5%) |

Bật với `eps` nhỏ không chỉ tệ hơn về số — đồ thị hiện rõ **dao động ~2 Hz biên
độ ±0.1 độ** ở shoulder và elbow, tức tay **rung**.

Nguyên nhân: `eps = 0.02 rad/s` trong khi quỹ đạo chỉ chạy tối đa 0.118 rad/s,
nên `tanh(q̇/eps)` gần như thành hàm dấu — mỗi lần `q̇` đổi dấu, mô-men bù nhảy
±0.256 Nm ở vai. Bù quá tay như vậy đóng vai trò **giảm chấn âm** và đẻ ra chu
trình giới hạn. Với `eps=0.5` thì `tanh` gần như tuyến tính trong dải tốc độ
thật, hoá ra chỉ còn là bù nhớt nhẹ — hết rung, nhưng cái lợi 5% không đáng để
gánh thêm rủi ro trên thiết bị đeo lên người, nhất là khi hệ số ma sát vẫn chưa
hiệu chỉnh thật.

**Mặc định đã đổi thành `friction_ff=False`** — để khâu tích phân gánh ma sát.
Nếu vẫn muốn bật, giữ `smooth_eps` lớn hơn ít nhất 4 lần tốc độ khớp lớn nhất
của quỹ đạo.

---

## 4. Cách chỉnh (tune)

Đa thức đặc trưng vòng kín sau tầng 1 là:

```
s³ + k_v·s² + k_e·s + k_i = 0
```

Có hai đường chỉnh, dùng đường nào cũng được.

### Đường (a): theo băng thông — trực giác hơn, nên bắt đầu từ đây

```python
LqiController(dyn, bandwidth=8.0)     # đặt cả 3 cực trùng nhau tại -8
```

`gains_from_bandwidth(w)` cho `k_v = 3w`, `k_e = 3w²`, `k_i = w³` (ba cực trùng
tại `-w`, đáp ứng tới hạn, không dao động).

Chọn `w` thế nào:

- **`w` phải nhỏ hơn nhiều tần số vòng điều khiển.** Vòng `ros2_control` chạy
  100 Hz = 628 rad/s → giữ `w ≤ ~1/20` của nó, tức `w ≤ 30 rad/s`.
- **Nhiễu vận tốc encoder bị nhân với `k_v = 3w`** → `w` lớn nghĩa là mô-men
  rung. Trên thiết bị đeo lên người, rung là thứ cấm.
- Quy trình: bắt đầu `w = 8 rad/s` (~1.3 Hz), tăng dần tới khi sai số bám đủ
  nhỏ hoặc bắt đầu thấy rung/kêu, rồi **lùi lại 30%**.

### Đường (b): theo trọng số Q, R — đúng bài LQR

```python
LqiController(dyn, weights=LqiWeights(q_int=6e6, q_pos=1.5e5, q_vel=1e3, r=1.0))
```

| Tăng | Tác dụng |
|---|---|
| `q_pos` | ưu tiên bám vị trí, cứng hơn |
| `q_vel` | ưu tiên êm, giảm vọt lố, chậm hơn |
| `q_int` | khử sai số xác lập nhanh hơn, nhưng dễ vọt lố + windup |
| `r` | tiết kiệm mô-men, mềm hơn |

**Chỉ tỉ lệ `Q/r` có ý nghĩa** — nhân cả 4 số cho cùng một hằng số thì `K` không
đổi.

Bộ mặc định `(6e6, 1.5e5, 1e3, 1)` không phải số bốc đại: quét lưới `Q` rồi
chọn bộ cho cực vòng kín `[-28.8, -10.0, -8.5]` — toàn cực **thực** (không dao
động, không vọt lố), cực chậm nhất 8.5 rad/s ≈ 1.35 Hz, tức chỉ bằng 1/74 tần
số lấy mẫu 100 Hz.

### Đọc kết quả chỉnh

```python
print(ctrl.describe(q_nominal=q0))
```

in ra `K`, cực vòng kín, **cực chậm nhất** và thời gian xác lập ước lượng, cùng
với hệ số PID **tương đương** tại một tư thế (nhân `K` với `M_ii`) — dùng để đối
chiếu trực tiếp với bộ PID cascade.

> Cảnh báo khi đọc số: hàm `equivalent_bandwidth()` chỉ phụ thuộc `k_i`, nên hai
> bộ hệ số cùng `k_i` sẽ ra cùng giá trị dù đáp ứng khác hẳn. Muốn biết nhanh
> hay chậm thì xem `dominant_pole()` — cực chậm nhất mới quyết định thời gian
> xác lập (`t_xác_lập ≈ 4 / cực_chậm_nhất`).

### Thử nhanh một bộ hệ số

```bash
python3 compare_pid_lqi.py --bandwidth 12      # cứng hơn mặc định
python3 compare_pid_lqi.py --bandwidth 5       # mềm hơn
python3 compare_pid_lqi.py --friction-ff       # bật lại bù ma sát để tự kiểm
python3 compare_pid_lqi.py --loops 3           # chạy 3 vòng, đo vòng 3
```

---

## 5. Đưa lên tay thật — dùng control_mode / input_mode nào

### Bảng quyết định

Đo trên cùng quỹ đạo, cùng vật lý, cùng giới hạn mô-men (`compare_architectures.py`),
kèm kết quả thử **máy chủ chết lúc t=5s** (`test_failsafe.py`):

| Kiến trúc | Lệnh CAN | control / input mode | Sai số đầu tay | Máy chủ chết → tay trôi |
|---|---|---|---|---|
| Vị trí thuần — **đang chạy** | 0x00C | 3 / 1 | 2.066 mm | **0.2 cm** |
| Vị trí, POS_FILTER | 0x00C | 3 / 3 | 45.520 mm | 0.2 cm |
| Vị trí + `vel_ff` | 0x00C | 3 / 1 | 0.437 mm | 0.2 cm |
| **Vị trí + `vel_ff` + `torque_ff`=G(q)** | 0x00C | 3 / 1 | **0.266 mm** | **0.2 cm** |
| MIT mode + LQI | 0x008 | 3 / 9 | 0.503 mm | **0.4 cm** |
| Mô-men thuần + LQI | 0x00E | **1 / 1** | **0.106 mm** | **47–88 cm** |

### Khuyến nghị: đi từng bước, đừng nhảy thẳng sang mô-men

**Bước 1 — điền 2 trường đang bỏ trống của `Set_Input_Pos`.** `send_position_command()`
đang gửi `vel_ff = 0; torque_ff = 0`. Điền `vel_ff = q̇_ref` và
`torque_ff = G(q_ref)/(gear_ratio·direction)` là **tốt hơn 7.8 lần**, không đổi
control mode, không mất lưới an toàn, và JTC đã có sẵn `q̇_ref` trong quỹ đạo.
Đây là thứ nên làm trước tiên vì gần như không có rủi ro.

**Bước 2 — nếu cần chính xác hơn nữa:** mô-men thuần, `control_mode = 1`
(TORQUE_CONTROL), `input_mode = 1` (PASSTHROUGH), gửi `Set_Input_Torque` (0x00E,
float32, **đơn vị phía rotor** — chia `gear_ratio·direction`). Được 19.4 lần,
nhưng **bắt buộc** phải thiết kế lưới an toàn thay thế trước (xem mục 6).

Không khuyến nghị MIT mode cho tay này: nó vừa kém chính xác hơn cách bước 1
(0.503 so với 0.266 mm) vừa phức tạp hơn. Lý do kém: trường `kd` chỉ tới
5.0 Nm·s/rad trong khi `base_joint` cần 4.10–5.79 → **56.7% số điểm bị cắt `kd`**,
và mô-men chỉ 12 bit (0.195 Nm/LSB ở khớp vai, so với 0.064 Nm/LSB của
`torque_ff` trong 0x00C). MIT mode vẫn đáng giá nếu sau này cần điều khiển
**trở kháng** (impedance) cho tương tác với người — lúc đó `kp`/`kd` gửi theo
thời gian thực mới là thứ cần.

### Vì sao mô-men thuần lại rơi tự do, dù phương trình đã có G(q)

Không phải vì thiếu G(q) trong công thức, mà vì **ai giữ vòng lặp sống**:

- **Chế độ vị trí:** vòng chống trọng lực nằm **trong driver**, chạy 8 kHz, độc
  lập với PC. PC chết → driver vẫn giữ setpoint cuối → tay đứng yên (0.2 cm).
  Bạn nói đúng: ở chế độ này **không cần** bù trọng lực tường minh. Nhưng đó
  cũng chính là lý do nó kém chính xác — cascade chỉ sinh mô-men **từ sai số**,
  nên phải *có sai số trước* rồi mới sửa. 2.066 mm đó phần lớn là trễ bám, không
  phải võng trọng lực.
- **Chế độ mô-men:** driver áp đúng con số bạn gửi, không hơn. G(q) chỉ tồn tại
  **chừng nào phần mềm của bạn còn tính và còn gửi, mỗi chu kỳ**. PC treo →
  driver giữ mô-men cuối, mà mô-men đó tính cho *tư thế lúc đó*; tay rời khỏi tư
  thế ấy thì mô-men không còn đúng nữa → trôi 47 cm. Watchdog nhảy → mô-men = 0
  → sập hoàn toàn 88 cm.

Có một nguồn sai G(q) thứ hai, quan trọng với thiết bị đeo: **URDF không mô hình
hoá cánh tay người**. G(q) lớn nhất đo được là 3.64 Nm ở vai cho tay máy trần.
Một cẳng tay + bàn tay người cỡ 1.5 kg ở tầm 0.3 m là thêm ~4.4 Nm — tức tải
chưa mô hình hoá có thể **lớn hơn cả** phần đã mô hình hoá. Ở chế độ vị trí,
feedback nuốt trọn sai số này. Ở chế độ mô-men, nó là độ lệch thường trực mà chỉ
khâu tích phân gỡ được — và khâu tích phân cần vòng lặp còn sống.

### Vì sao đổi input_mode 3 → 1 mà độ trễ gần như không đổi

Vì độ trễ chính **không đến từ bộ lọc**. Nó là đặc tính cấu trúc của vòng P vị trí:
để khớp chạy ở tốc độ `q̇`, vòng P phải duy trì một sai số thường trực

```
e_khớp = q̇ / pos_gain
```

(triệt tiêu hết `gear_ratio` — công thức đúng cho cả 3 khớp). Với
`pos_gain = 20` và `q̇` lớn nhất 0.109 rad/s ở base: `e = 0.31°`, khớp với 0.29°
đo được. **Chỉ feedforward mới xoá được sai số này** — đó là lý do `vel_ff` cải
thiện 4.7 lần trong khi đổi input_mode thì không.

Nhưng có một chỗ **không khớp** giữa mô phỏng và cái bạn quan sát, cần kiểm tra
trên phần cứng: mô phỏng cho thấy `input_mode = 3` phải **xấu hơn 22 lần**
(45.5 mm so với 2.07 mm), chứ không phải "gần như không đổi". Bộ lọc POS_FILTER
với `bandwidth = 2.0 rad/s` gây trễ `2/bw · v` — rất lớn. Bạn thấy không đổi thì
nhiều khả năng là một trong hai:

1. `input_filter_bandwidth` trên driver của bạn **không phải 2.0** (đây là tham
   số lưu trong driver, mô phỏng dùng mặc định của ODrive). Kiểm bằng
   `odrv0.axis0.controller.config.input_filter_bandwidth` — nếu nó đã được đặt
   cao (vài chục) thì bộ lọc gần như trong suốt và quan sát của bạn là đúng.
2. Lệnh `Set_Controller_Mode` không thực sự có tác dụng lúc đó.

Dù là lý do nào, kết luận thực dụng vẫn giữ nguyên: **giữ `input_mode = 1`**, và
độ trễ còn lại phải xử lý bằng `vel_ff`, không phải bằng đổi mode.

---

## 6. Nếu chọn chế độ mô-men — những thứ phải làm trước

1. **Watchdog của driver** (`axis.config.enable_watchdog`, `watchdog_timeout`) —
   bắt buộc bật, và plugin phải nuôi nó mỗi chu kỳ. Nhưng nhớ: watchdog nhảy là
   mô-men về 0, tức **tay sập** (88 cm). Watchdog bảo vệ khỏi mô-men sai, không
   bảo vệ khỏi rơi.
2. **Chế độ dự phòng về vị trí:** khi phát hiện quá hạn, thay vì để rơi, hãy gửi
   `Set_Controller_Mode(3, 1)` + `Set_Input_Pos(vị trí hiện tại)` để driver tự
   giữ. Đây là thứ phải viết trong plugin C++, không có sẵn.
3. **`enable_torque_mode_vel_limit`** — ODrive giảm mô-men khi tốc độ tiến gần
   `vel_limit`. Giữ bật; đây là phanh chống chạy loạn.
4. **Giới hạn mô-men theo tư thế trong phần mềm**, chặt hơn 5/40/5 Nm của URDF.
5. **Mô hình hoá tải cánh tay người** hoặc để khâu tích phân ước lượng nó, và
   giới hạn tốc độ tăng của khâu tích phân lúc khởi động.
6. **Thêm `<command_interface name="effort"/>`** vào URDF và nhánh gửi 0x00E vào
   plugin — giao thức đã có sẵn `CmdId::SetInputTorque` trong
   `gim6010_can_protocol.hpp`.

> Toàn bộ số liệu trong tài liệu này là **mô phỏng**, chưa có tải cánh tay người
> và với 2 hệ số ma sát chưa hiệu chỉnh từ log CAN thật. Coi đây là thứ tự ưu
> tiên và bậc độ lớn, không phải con số nghiệm thu.

---

## 7. Đã cài lên phần cứng — bước 1 (feedforward ở chế độ vị trí)

Đã triển khai xong. **Vẫn giữ `control_mode = 3` (vị trí), `input_mode = 1`** —
driver giữ nguyên vòng vị trí của nó, nên lưới an toàn còn nguyên (PC chết → tay
trôi 0.2 cm).

### Các file đã sửa

| File | Sửa gì |
|---|---|
| `gim6010_can_protocol.hpp` | Thêm `pack_set_input_pos()` đóng gói đủ 3 trường + `encode_milli_i16()` (int16 thang 0.001, có bão hoà). |
| `gim_arm_system.hpp/.cpp` | Thêm command interface `velocity`; nạp mô hình Pinocchio từ URDF; tính `G(q_lệnh)` mỗi chu kỳ `write()`; quy đổi khớp→rotor và kẹp trần. |
| `gim_arm.urdf` | Thêm `<command_interface name="velocity"/>` cho 3 khớp; bật 2 tham số feedforward. |
| `controllers.yaml` | Thêm `velocity` vào `command_interfaces` của JTC. |
| `CMakeLists.txt`, `package.xml` | Thêm phụ thuộc `pinocchio`, `ament_index_cpp`. |

### Bật / tắt

Nằm trong `gim_arm.urdf`, khối `<ros2_control><hardware>`:

```xml
<param name="velocity_feedforward">true</param>
<param name="gravity_feedforward">true</param>
```

**Mặc định trong code C++ là `false` cả hai** — cố ý: build lại plugin không
được âm thầm đổi hành vi của thiết bị đang đeo trên tay người. URDF là nơi bật,
và bật ở đó thì nhìn thấy được. Tắt để so sánh A/B chỉ cần đổi `true` → `false`
rồi `colcon build --packages-select gim_arm_description`, **không cần build lại
C++**.

Trần an toàn `max_torque_ff_rotor_nm` mặc định 0.625 Nm (định mức phía rotor).
Trọng lực thật lớn nhất trên quỹ đạo chỉ ~0.057 Nm rotor nên trần này rất rộng;
siết lại được bằng `<param name="max_torque_ff_rotor_nm">`.

### Đã kiểm chứng những gì

1. **Biên dịch sạch** cả 4 package (`colcon build`, Release).
2. **G(q) trong C++ khớp Python từng chữ số** — mà bản Python đã khớp MuJoCo tới
   5e-10. Chuỗi URDF → Pinocchio C++ → `G(q)` đúng.
3. **Vòng quy đổi đơn vị khớp ↔ rotor đúng cả hai chiều**, kiểm cho cả 3 khớp
   gồm `shoulder` (gear 64) và `elbow` (`invert_direction=true`): đóng gói frame
   rồi giải mã đúng như driver thì ra lại giá trị ban đầu trong phạm vi 1 LSB.
   Đây là chỗ dễ sai nhất — nhân thay vì chia `gear_ratio` là shoulder lệch 64
   lần, quên `direction` là feedforward đẩy ngược chống lại vòng vị trí.
4. **Tương thích ngược:** plugin chỉ xuất interface `velocity` khi URDF có khai;
   URDF cũ chạy y như trước. `G(q)` chỉ tính khi **cả 3** lệnh khớp hợp lệ.

### Chưa kiểm được ở đây, phải làm trên tay thật

- Chạy thật với `vcan` + `gim6010_mujoco_sim.py` để xem frame CAN có đúng
  `Vel_FF`/`Torque_FF` không (cần `sudo` dựng vcan).
- **Dấu của `Torque_FF`.** Quy đổi đã kiểm bằng số, nhưng quy ước dấu mô-men của
  *firmware* thì chỉ tay thật mới trả lời. Cách thử an toàn: cho tay đứng yên
  một tư thế, bật `gravity_feedforward`, xem dòng điện (0x014 `Get_Iq`) **giảm**
  hay **tăng**. Giảm là đúng dấu — feedforward đang gánh bớt cho vòng vị trí.
  Tăng là ngược dấu, đổi dấu `torque_ff_rotor` trong `send_position_command()`.
  Làm bước này **trước khi** cho người đeo vào.
- **Tải cánh tay người chưa có trong mô hình** (mục 5). `G(q)` hiện chỉ tính cho
  tay máy trần.
