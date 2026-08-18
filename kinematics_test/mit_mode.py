"""
mit_mode.py — chạy LQI qua Mit_Control (0x008) thay vì Set_Input_Torque (0x00E).

===========================================================================
VÌ SAO MIT MODE MỚI LÀ ĐƯỜNG ĐÚNG CHO THIẾT BỊ ĐEO
===========================================================================
Mit_Control gửi 1 frame gồm 5 trường: (p_des, v_des, kp, kd, τ_ff), và firmware
tính NGAY TRONG DRIVER, ở 8 kHz:

        τ = kp·(p_des - p) + kd·(v_des - v) + τ_ff

Đối chiếu với luật LQI:

        τ = M(q)·(q̈_ref - k_i∫e - k_e·e - k_v·ė) + C(q,q̇)q̇ + G(q)
          = [M q̈_ref + Cq̇ + G - M k_i ∫e]  -  M k_e (q - q_ref)  -  M k_v (q̇ - q̇_ref)
            └────────── τ_ff ──────────┘     └── kp ──┘             └── kd ──┘

Tức LQI RÃ ĐÚNG THÀNH 3 TRƯỜNG của Mit_Control, không phải gán ép:
    p_des = q_ref,  v_des = q̇_ref
    kp    = M_ii(q)·k_e          (hệ số này ĐỔI THEO TƯ THẾ, không cố định)
    kd    = M_ii(q)·k_v
    τ_ff  = M q̈_ref + Cq̇ + G - M k_i ∫e

Được cả 3 thứ cùng lúc, không phải chọn 1:
  1) Phần feedforward theo mô hình Lagrange -> hết trễ bám và võng trọng lực.
  2) Phần phản hồi chạy ở 8 KHZ TRONG DRIVER, không phải 100 Hz trên PC.
  3) VÀ QUAN TRỌNG NHẤT: nếu PC chết, driver vẫn còn kp/kd/p_des của lệnh cuối
     -> nó tiếp tục GIỮ TAY như một vòng vị trí. Không rơi. Đây đúng là lưới an
     toàn mà chế độ mô-men thuần (0x00E) không có.

===========================================================================
HAI CÁI BẪY ĐƠN VỊ -- SAI CHỖ NÀY LÀ HỎNG PHẦN CỨNG
===========================================================================
1) Mit_Control dùng đơn vị PHÍA TRỤC RA, KHÔNG phải phía rotor (manual 3.1.6 &
   4.1.2, header ghi rõ 2 lần). Khác hẳn Set_Input_Pos/Set_Input_Torque vốn
   dùng đơn vị rotor. Không được tự nhân/chia tỉ số truyền cho 0x008 -- firmware
   đã làm rồi.
2) NHƯNG "trục ra" của firmware chỉ là sau hộp số NỘI BỘ 8:1. Hộp giảm tốc
   NGOÀI 8:1 của shoulder thì firmware KHÔNG BIẾT. Nên hệ số quy đổi còn lại là
        n_ngoài = gear_ratio_tổng / 8
   tức 1 cho base/elbow, và 8 cho shoulder. Bỏ sót chỗ này là shoulder sai đúng
   8 lần -- lệch kiểu đó trên tay đang đeo vào người là tai nạn.

Quy đổi đầy đủ (d = direction ±1 bù chiều lắp):
        p_mit = q_khớp · n_ngoài · d          τ_mit = τ_khớp / (n_ngoài · d)
        v_mit = q̇_khớp · n_ngoài · d          kp_mit = kp_khớp / n_ngoài²
                                              kd_mit = kd_khớp / n_ngoài²

===========================================================================
LƯỢNG TỬ HOÁ -- 12 BIT, KHÔNG PHẢI FLOAT
===========================================================================
Mit_Control nhét cả 5 trường vào 8 byte, nên độ phân giải thô hơn float32 của
0x00E rất nhiều (số lấy từ chính header):
    p  : 16 bit trên ±12.5 rad   -> 3.8e-4 rad
    v  : 12 bit trên ±65 rad/s   -> 0.032 rad/s   <- thô so với quỹ đạo 0.1 rad/s
    kp : 12 bit trên 0..500      -> 0.122 Nm/rad
    kd : 12 bit trên 0..5        -> 0.0012 Nm·s/rad
    τ  : 12 bit trên ±50 Nm      -> 0.0244 Nm phía trục ra
Với shoulder (n_ngoài=8) thì 0.0244 Nm đó thành 0.195 Nm ở KHỚP. Đây là lý do
file này mô phỏng lượng tử hoá y hệt encode_range/decode_range trong header
C++, thay vì giả vờ truyền số thực -- để con số so sánh là con số thật.

===========================================================================
HAI GIỚI HẠN ĐO ĐƯỢC -- ĐỌC TRƯỚC KHI CHỌN KIẾN TRÚC NÀY
===========================================================================
1) TRƯỜNG kd CHỈ TỚI 5.0 Nm·s/rad, VÀ THẾ LÀ KHÔNG ĐỦ. base_joint cần kd trong
   khoảng 4.10..5.79 -> **56.7% số điểm trên quỹ đạo bị CẮT kd**. Đây là giới
   hạn của ĐỊNH DẠNG FRAME (12 bit trên dải 0..5), không phải chuyện chỉnh
   tham số, nên không thể lách bằng cách đổi trọng số LQR mà không hy sinh
   chất lượng. Nguyên nhân sâu xa: n_ngoài = 1 ở base/elbow nên kd không được
   chia nhỏ như ở shoulder (chia 64).
2) Kết quả bám thực đo (2 vòng, đo vòng cuối), sai số đầu tay RMS:
       vị trí thuần (đang chạy)                 2.0657 mm
       vị trí + vel_ff + torque_ff = G(q)       0.2656 mm
       MIT mode + LQI (lượng tử 12 bit)         0.5025 mm
       mô-men thuần 0x00E + LQI (float32)       0.1064 mm
   Tức MIT mode THUA cách đơn giản hơn nhiều là điền 2 trường feedforward của
   Set_Input_Pos. Lý do: torque_ff của 0x00C là int16 thang 0.001 Nm PHÍA
   ROTOR -> ở khớp shoulder là 0.064 Nm/LSB, mịn gấp 3 lần 0.195 Nm/LSB của
   MIT; cộng thêm chuyện kd bị cắt ở trên.
   Cái MIT mode đổi lại được là AN TOÀN: máy chủ chết lúc t=5s, sau 3s tay chỉ
   trôi 0.4 cm (chế độ mô-men thuần trôi 47..88 cm). Xem test_failsafe.py.
"""

import numpy as np

# Dải mã hoá -- PHẢI khớp đúng gim6010_can_protocol.hpp
POS_MIN, POS_MAX, POS_BITS = -12.5, 12.5, 16
VEL_MIN, VEL_MAX, VEL_BITS = -65.0, 65.0, 12
KP_MIN, KP_MAX, KP_BITS = 0.0, 500.0, 12
KD_MIN, KD_MAX, KD_BITS = 0.0, 5.0, 12
TRQ_MIN, TRQ_MAX, TRQ_BITS = -50.0, 50.0, 12

# Mô-men tối đa driver cấp được, quy về phía trục ra hộp số nội bộ:
# 0.625 Nm phía rotor × 8 = 5 Nm.
MIT_TORQUE_LIMIT_NM = 5.0


def encode_range(x, lo, hi, bits):
    """Bản Python của encode_range() trong header C++."""
    x = np.clip(x, lo, hi)
    scale = float((1 << bits) - 1)
    return np.floor((x - lo) * scale / (hi - lo)).astype(np.int64)


def decode_range(x_int, lo, hi, bits):
    scale = float((1 << bits) - 1)
    return x_int * (hi - lo) / scale + lo


def quantize(x, lo, hi, bits):
    """Cho ra đúng giá trị mà firmware sẽ THẤY sau khi đi qua 8 byte."""
    return decode_range(encode_range(x, lo, hi, bits), lo, hi, bits)


class MitAxis:
    """Mô hình phía DRIVER của Mit_Control: giữ nguyên 5 trường của lệnh cuối
    cùng nhận được, và tính lại mô-men mỗi tick của firmware (8 kHz thật)."""

    def __init__(self, cfg):
        self.cfg = cfg
        # n_ngoài: phần tỉ số truyền mà firmware KHÔNG biết (xem docstring)
        self.n_ext = cfg.gear_ratio / 8.0
        self.d = cfg.direction
        self.p_des = self.v_des = self.kp = self.kd = self.tau_ff = 0.0

    # --- quy đổi khớp <-> MIT ---
    def joint_to_mit_pos(self, q):
        return (q + self.cfg.zero_offset) * self.n_ext * self.d

    def mit_to_joint_pos(self, p):
        return p / (self.n_ext * self.d) - self.cfg.zero_offset

    def set_command(self, q_ref, qd_ref, kp_joint, kd_joint, tau_ff_joint):
        """Đóng gói 1 frame Mit_Control, có lượng tử hoá y như trên bus thật."""
        self.p_des = quantize(self.joint_to_mit_pos(q_ref), POS_MIN, POS_MAX, POS_BITS)
        self.v_des = quantize(qd_ref * self.n_ext * self.d, VEL_MIN, VEL_MAX, VEL_BITS)
        self.kp = quantize(kp_joint / self.n_ext ** 2, KP_MIN, KP_MAX, KP_BITS)
        self.kd = quantize(kd_joint / self.n_ext ** 2, KD_MIN, KD_MAX, KD_BITS)
        self.tau_ff = quantize(tau_ff_joint / (self.n_ext * self.d),
                               TRQ_MIN, TRQ_MAX, TRQ_BITS)

    def update(self, q, qd):
        """Một tick của firmware. Trả về mô-men TẠI KHỚP (Nm)."""
        p = self.joint_to_mit_pos(q)
        v = qd * self.n_ext * self.d
        tau_mit = self.kp * (self.p_des - p) + self.kd * (self.v_des - v) + self.tau_ff
        tau_mit = float(np.clip(tau_mit, -MIT_TORQUE_LIMIT_NM, MIT_TORQUE_LIMIT_NM))
        return tau_mit * self.n_ext * self.d


def run_mit(sim, urdf, ref, dyn, duration, ctrl_hz=100.0, fail_at=None):
    """Chạy LQI qua Mit_Control trong mô phỏng.

    fail_at: nếu đặt (giây), mô phỏng MÁY CHỦ CHẾT tại thời điểm đó -- ngừng
    gửi frame mới, driver tiếp tục dùng kp/kd/p_des/τ_ff của lệnh cuối cùng.
    Trả về (log, q_lúc_hỏng, q_cuối)."""
    from lqi_controller import LqiController, LqiWeights

    axes = sim.parse_axes_from_urdf(urdf)
    physics = sim.ArmPhysics(urdf, axes)
    dt_phys = 1.0 / sim.CONTROL_HZ
    per = max(1, int(round(sim.CONTROL_HZ / ctrl_hz)))
    q0 = ref.at(0.0)[0]
    physics.reset(q0)
    drivers = [MitAxis(c) for c in axes]
    ctrl = LqiController(dyn, weights=LqiWeights())

    n = int(duration / dt_phys)
    n_fail = int(fail_at / dt_phys) if fail_at else n + 1
    log = {k: np.zeros((n, 3)) for k in ("q", "qd", "q_ref", "tau")}
    log["t"] = np.zeros(n)
    q_fail = None

    for k in range(n):
        t = k * dt_phys
        q = np.array([physics.joint_state(i)[0] for i in range(3)])
        qd = np.array([physics.joint_state(i)[1] for i in range(3)])
        q_ref, qd_ref, qdd_ref = ref.at(t)
        if k == n_fail:
            q_fail = q.copy()
        if k % per == 0 and k < n_fail:
            kp, kd, tau_ff = lqi_to_mit(dyn, ctrl, q, qd, q_ref, qd_ref, qdd_ref,
                                        per * dt_phys)
            for i, d in enumerate(drivers):
                d.set_command(q_ref[i], qd_ref[i], kp[i], kd[i], tau_ff[i])
        tau = np.array([d.update(q[i], qd[i]) for i, d in enumerate(drivers)])
        log["t"][k] = t
        log["q"][k], log["qd"][k], log["q_ref"][k], log["tau"][k] = q, qd, q_ref, tau
        physics.step(tau)
    q_end = np.array([physics.joint_state(i)[0] for i in range(3)])
    return log, q_fail, q_end


def lqi_to_mit(dyn, controller, q, qd, q_ref, qd_ref, qdd_ref, dt):
    """Rã luật LQI thành 5 trường của Mit_Control (xem khai triển ở docstring).

    Trả về (kp_khớp, kd_khớp, tau_ff_khớp), tất cả ở phía KHỚP -- MitAxis lo
    phần quy đổi và lượng tử hoá."""
    M = dyn.mass_matrix(q)
    e = q - q_ref
    k_i, k_e, k_v = controller.K[:, 0], controller.K[:, 1], controller.K[:, 2]

    kp_joint = np.diag(M) * k_e
    kd_joint = np.diag(M) * k_v
    # Phần feedforward: mô hình + khâu tích phân. Phần tỉ lệ/vi phân đã nằm ở
    # kp/kd rồi nên KHÔNG được cộng lại vào đây (cộng 2 lần = gấp đôi độ cứng).
    tau_ff = (M @ qdd_ref + dyn.nonlinear(q, qd)
              - np.diag(M) * k_i * controller.integral)

    # Cập nhật khâu tích phân + chống windup, dùng chung quy tắc với LqiController
    tau_total = tau_ff - kp_joint * e - kd_joint * (qd - qd_ref)
    tau_sat = np.clip(tau_total, -controller.tau_limit, controller.tau_limit)
    pushing = (np.abs(tau_total - tau_sat) > 1e-12) & (np.sign(tau_total) == np.sign(-e))
    controller.integral = np.where(pushing, controller.integral,
                                   controller.integral + e * dt)
    controller.integral = np.clip(controller.integral,
                                  -controller.i_limit, controller.i_limit)
    return kp_joint, kd_joint, tau_ff
