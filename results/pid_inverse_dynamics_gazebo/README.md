# PID với feedforward nghịch động lực học — 06/10/2026

Quỹ đạo giữ đủ `q_ref`, `qd_ref`, `qdd_ref`. Luật điều khiển trong các pha
APPROACH, TRACK, RETURN của profile `gazebo_smooth`:

```text
e      = q_ref - q
de/dt  = qd_ref - qd
tau_ff = M(q_ref)*qdd_ref + C(q_ref,qd_ref)*qd_ref + G(q_ref)
tau    = tau_ff + Kp*e + Ki*integral(e) + Kd*de/dt
```

`tau_ff` gọi `ArmDynamics.inverse_dynamics(q_ref, qd_ref, qdd_ref)`, bao gồm
quán tính rotor phản chiếu đã có trong mô hình. Đã đối chiếu hàm này với tổng
`M @ qdd + C*qd + G` trên mô hình URDF: sai khác lớn nhất khoảng 1,78e-15 Nm.
Gazebo dùng effort interface và nhận tổng mô-men từ controller.

Tiêu chí đánh giá chỉ là RMS và max sai số vị trí. Đồ thị mỗi khớp gồm vị trí,
sai số vị trí, mô-men; vận tốc/gia tốc vẫn được dùng trong điều khiển.

| Thông số | Base / shoulder / elbow |
|---|---|
| Kp (Nm/rad) | 16.92 / 33.84 / 16.45 |
| Ki (Nm/(rad*s)) | 12.69 / 16.92 / 9.87 |
| Kd (Nm*s/rad) | 0.94 / 1.41 / 1.175 |
| Giới hạn tích phân (rad*s) | 0.3 / 0.3 / 0.3 |
| Tần số điều khiển | 2000 Hz |

Không tải dùng `tau_scale=0.50`; tải 0,5 kg dùng `tau_scale=0.75`.
Tải được gắn tại tool trong Gazebo; mô hình feedforward vẫn là URDF gốc.
Hai lần chạy đều đạt `HOLD` sau APPROACH 5 s, TRACK một vòng 27 s và
RETURN 5 s. Mỗi CSV có 74.003 mẫu. Dùng Gazebo Classic headless, ODE
200 iterations, physics step 0,5 ms và `use_sim_time=false`.

| Trường hợp | RMS vị trí TRACK (độ) | Max sai số TRACK (độ) |
|---|---|---|
| Không tải | 0.056 / 0.277 / 0.065 | 0.230 / 0.718 / 0.198 |
| Tải 0,5 kg | 0.303 / 0.351 / 0.294 | 0.733 / 0.989 / 1.514 |

| Trường hợp | RMS vị trí toàn bộ ba pha (độ) | Max sai số toàn bộ ba pha (độ) | Max mô-men (Nm) |
|---|---|---|---|
| Không tải | 0.051 / 0.261 / 0.063 | 0.230 / 0.718 / 0.198 | 0.769 / 3.628 / 1.545 |
| Tải 0,5 kg | 0.364 / 0.759 / 0.986 | 0.866 / 2.258 / 3.164 | 1.023 / 6.318 / 3.549 |

Các trường hợp giữ nguyên ngưỡng dừng sai số: 0,05 rad ở TRACK và 0,10 rad
ở APPROACH/RETURN. Mô-men và tích phân được kẹp, có anti-windup.
Ảnh dùng sai số `q-q_ref`; controller dùng `e=q_ref-q`. RMS và max trị
tuyệt đối không phụ thuộc quy ước dấu này.

| Trường hợp | CSV | Base | Shoulder | Elbow |
|---|---|---|---|---|
| Không tải | [noload.csv](noload.csv) | [q1](noload_joints_q1_base.png) | [q2](noload_joints_q2_shoulder.png) | [q3](noload_joints_q3_elbow.png) |
| Tải 0,5 kg | [payload_0p5.csv](payload_0p5.csv) | [q1](payload_0p5_joints_q1_base.png) | [q2](payload_0p5_joints_q2_shoulder.png) | [q3](payload_0p5_joints_q3_elbow.png) |

Build hai package thành công. Bảy regression tests đạt, gồm kiểm tra tổng
mô-men PID với feedforward, giới hạn tích phân/reset/anti-windup, đạo hàm sai
số, tác động của gia tốc tham chiếu và feedforward dùng đủ ba tín hiệu
tham chiếu.

[Thông tin từng lần chạy](validation.json) · [Log kiểm tra](validation.log).

Chạy lại từ workspace sau khi build và source ROS/workspace:

```bash
python3 tools/validate_pid_gazebo.py
```

Script dùng ROS domain 73/cổng Gazebo 11373, tự dừng mô phỏng sau khi kiểm tra.
[Hướng dẫn chạy thủ công](../../src/README.md).
