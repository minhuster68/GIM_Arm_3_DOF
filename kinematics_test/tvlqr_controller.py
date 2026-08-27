"""
tvlqr_controller.py — LQR BIẾN THIÊN THEO THỜI GIAN (time-varying LQR + tích
phân) cho GIM Arm 3DOF. Cổng của bộ điều khiển bên repo matlab
(matlab-for-GIM-arm-3-dof/setup_lqr.m) sang ROS 2, thay cho lqi_controller.py.

Đặt tại kinematics_test/ (file THẬT), gim_control/ chỉ là symlink -- để 1 bản duy
nhất, không trôi lệch.

===========================================================================
KHÁC GÌ lqi_controller.py (bộ cũ)
===========================================================================
Hai bộ KHÔNG phải cùng một luật viết ở hai nơi, mà là HAI KIẾN TRÚC khác nhau:

  lqi_controller.py (CŨ) -- tuyến tính hoá phản hồi + LQI hằng số
      τ = M(q)·(q̈_ref - K·x) + C(q,q̇)q̇ + G(q),  K giải Riccati 3x3 MỘT LẦN
    Khử sạch phi tuyến trước, rồi điều khiển 3 khâu tích phân kép rời nhau.
    K không đổi theo tư thế; độ cứng hiệu dụng tự đổi vì nhân lại M(q).

  tvlqr_controller.py (BỘ NÀY) -- tuyến tính hoá cục bộ + LQR dọc quỹ đạo
      τ = τ_ff(q_ref,q̇_ref,q̈_ref) - K(q_ref,q̇_ref)·x,  K giải Riccati 9x9
                                                          LẠI Ở TỪNG ĐIỂM
    Không khử phi tuyến. Thay vào đó tuyến tính hoá phương trình SAI SỐ quanh
    điểm làm việc rồi giải LQR đúng cho hệ 9 trạng thái CÓ XEN KÊNH giữa 3 khớp
    (M⁻¹ đầy, không chéo hoá), cộng feedforward nghịch động lực học.

===========================================================================
TOÁN -- giống setup_lqr.m từng dòng
===========================================================================
Trạng thái sai số (9 chiều), e = q - q_ref:

    x = [∫e dt ; e ; ė]

Tuyến tính hoá M q̈ + C(q,q̇)q̇ + G(q) = τ quanh (q_ref, q̇_ref) bằng SAI PHÂN
TRUNG TÂM (bước fd_step, mặc định 1e-5 -- đúng `delta` của setup_lqr.m):

    Astiff = ∂G/∂q            (matlab: gravityTorque   -> arm_dynamics.gravity)
    Adamp  = ∂(C(q,q̇)q̇)/∂q̇   (matlab: velocityProduct -> arm_dynamics.coriolis)

    A = [ 0        I        0      ]        B = [ 0    ]
        [ 0        0        I      ]            [ 0    ]
        [ 0   -M⁻¹Astiff  -M⁻¹Adamp]            [ M⁻¹  ]

    K = R⁻¹BᵀP,  P nghiệm Riccati liên tục (CARE) của (A, B, Q, R)
    τ = τ_ff - K x

Q, R theo luật Bryson (matlab):
    Q = diag(1/max_int_e², 1/max_e², 1/max_de²)
    R = diag(1/max_tau²) * tau_penalty_scale

LƯU Ý ĐƠN VỊ: ở bộ CŨ đầu vào u là GIA TỐC (τ = M·u), nên K có đơn vị gia tốc.
Ở bộ NÀY B = [0;0;M⁻¹] nên đầu vào u CHÍNH LÀ MÔ-MEN và K có đơn vị Nm. Đừng so
trực tiếp hai bảng số K với nhau, và đừng dùng lại công thức kiểm i_limit của bộ
cũ (K[:,0]*i_limit*M_ii) -- ở đây dùng integral_kick().

===========================================================================
BA CHỖ CỐ Ý LÀM KHÁC setup_lqr.m -- đều có lý do đo được
===========================================================================
1) KHÔNG bê K_array tĩnh từ matlab.mat, cũng KHÔNG lập bảng K(t) theo thời gian.
   Thay vào đó tính K TẠI CHỖ từ (q_ref, q̇_ref) mỗi chu kỳ.
   . setup_lqr.m lập bảng theo t_full rồi để Simulink nội suy theo THỜI GIAN.
     Cách đó buộc quỹ đạo lúc chạy phải trùng khít quỹ đạo lúc lập bảng; lệch
     pha thì K đúng số nhưng sai thời điểm -- tệ hơn dùng K sai hẳn, và im lặng.
     Ở ROS thì lqi_node còn có pha APPROACH (đa thức bậc 5) và HOLD, vốn KHÔNG
     có trong bảng, nên bảng theo thời gian không phủ hết.
   . Lập theo (q_ref, q̇_ref) là ánh xạ 1-1 với điểm làm việc, không cần đồng bộ
     thời gian, và tự phủ mọi pha của máy trạng thái.
   . Dùng q_ref chứ KHÔNG dùng q đo được: q_ref sạch nhiễu, nên K trơn theo thời
     gian. Lập theo q đo được thì nhiễu encoder đi thẳng vào hệ số điều khiển.
   . ĐO ĐƯỢC trên máy này: A,B + Riccati 9x9 mất 0.52 ms/chu kỳ, tức 5.2% ngân
     sách 10 ms của vòng 100 Hz. Không cần lập bảng trước.
     (recompute_every > 1 để giảm tải nếu cần; K được giữ giữa các lần tính.)

2) TRỌNG SỐ MẶC ĐỊNH KHÁC matlab. Bộ số của setup_lqr.m
       max_int_e = [100, 200, 100],  max_e = 0.1,  max_de = 2,  max_tau = [5,40,5]
   KHÔNG dùng được cho vòng 100 Hz của ROS. Đã kiểm bằng số trên chính URDF này,
   40 tư thế ngẫu nhiên, rời rạc hoá ZOH ở dt = 0.01 s:
       bán kính phổ rời rạc |z|max = 18.06   -> PHÂN KỲ (cần < 1)
       cực liên tục nhanh nhất |s|max = 1735 rad/s, gấp 2.8 lần cả tần số
       Nyquist của vòng (314 rad/s)
   Lý do matlab không lộ ra lỗi này: Simulink chạy solver liên tục bước rất nhỏ,
   nên |s| = 1735 rad/s vẫn giải được. Vòng ros2_control thì cố định 100 Hz.
   Thêm nữa max_int_e = [100, 200, 100] rad·s làm trọng số tích phân còn
   1e-4..2.5e-5 -> ba cực tích phân nằm ở -0.0005 rad/s, tức KHÂU TÍCH PHÂN
   COI NHƯ KHÔNG CÓ (thời gian xác lập 8000 s). "LQI" khi đó chỉ là LQR.
   ---- tau_penalty_scale: quét trên chính URDF này, plant = mô hình + ma sát,
        controller 100 Hz ZOH, bám đúng quỹ đạo của sweep_trajectory.py:
            R x    |z|max   |s|max (rad/s)
             128    1.193       192      <- PHÂN KỲ khi rời rạc hoá
             256    0.969       134
             512    0.953        94      <= MẶC ĐỊNH
            1024    0.958        65
            2048    0.979        35      (bằng băng thông bộ LQI cũ)
        Chọn 512 chứ không phải 256: 256 chỉ cách ranh giới phân kỳ 1.5 lần theo
        R (1.23 lần theo hệ số) -- không đủ dư địa cho sai mô hình + trễ + nhiễu
        encoder trên tay thật. Ở 512, |s|max = 94 rad/s = 1/3.3 Nyquist của vòng.

   ---- max_int_e: chỗ này KHÔNG được chỉnh trên mô phỏng "mô hình hoàn hảo".
        Với plant = đúng mô hình thì khâu tích phân chỉ thêm trễ, quét kiểu đó sẽ
        kết luận sai là "bỏ tích phân đi". Phải đưa vào SAI MÔ HÌNH THẬT ĐÃ ĐO:
        gravity_scale trên tay thật là 1.1 ở shoulder và 0.8 ở elbow, tức mô hình
        trọng lực lệch 10-25%. Đặt plant = 1.10 * G(q) rồi quét (R x512, sai số
        ĐẦU TAY RMS, mm):
          max_int_e   |z|max  |s|max  cực chậm  G sai 10%  G sai 25%  +trễ 1ck  +nhiễu q̇
              0.001    0.942    95.4     5.67      0.087      0.096     0.089     0.416
              0.002    0.953    93.6     4.57      0.163      0.181     0.166     0.478  <= MẶC ĐỊNH
              0.005    0.964    93.0     3.53      0.352      8.650     0.356     0.578
              0.020    0.974    92.9     2.58     13.288         --        --        --
           matlab(100) 1.000       --       --     39.650         --        --        --
        Chọn 0.002 chứ KHÔNG phải 0.001 dù 0.001 bám tốt hơn 2 lần: ở 0.001 thì
        RIÊNG khâu tích phân đã ra lệnh được 1.22 / 11.83 / 1.39 Nm khi ∫e chạm
        kẹp i_limit = 0.004, tức 70-85% toàn bộ trần mô-men (1.75/14/1.75 ở
        tau_scale 0.35) -- vượt ngưỡng cảnh báo của lqi_node.check_i_limit. Ở
        0.002 là 0.61 / 5.91 / 0.70 Nm, vừa dưới ngưỡng. Xem integral_kick().

   ---- ĐỐI CHIẾU với bộ CŨ, cùng phép thử, cùng i_limit 0.004 (đầu tay RMS, mm):
                              G sai 10%  G sai 25%  +trễ 1ck  +nhiễu q̇
          TVLQR (mặc định)        0.163      0.181     0.166     0.478
          LqiController cũ        0.495      1.046     0.500     0.626
        Tức bộ mới bám tốt hơn 3 lần khi có sai mô hình. NHƯNG với plant = mô
        hình HOÀN HẢO thì ngược lại, bộ cũ tốt hơn (0.758 mm so với 1.33 mm ở
        R x512, và bộ cũ chỉ dùng |s|max 28.8 rad/s so với 94). Nói cách khác:
        lợi thế của TVLQR ở đây đến từ trọng số tích phân được chỉnh lại cho
        đúng sai mô hình thật, không phải từ bản thân kiến trúc.
   Muốn chạy đúng số matlab để đối chiếu thì đặt
       max_int_e=[100,200,100], tau_penalty_scale=1.0, require_discrete_stable=False
   và CHỈ chạy trong mô phỏng.

3) G(q) lấy ở TƯ THẾ ĐO ĐƯỢC, không phải ở q_ref (gravity_at_measured=True).
   setup_lqr.m/Simulink cộng feedforward hoàn toàn theo q_ref. Nhưng lqi_node.py
   hiệu chỉnh mô hình trọng lực bằng cách TRỪ LẠI (1-α)·G(q) với q ĐO ĐƯỢC. Nếu
   feedforward chỉ chứa G(q_ref) thì phép trừ đó không còn khử đúng, còn dư
   G(q_ref) - G(q); với sai số bám cho phép 0.05 rad thì phần dư tới ~0.25 Nm ở
   shoulder, cỡ 7% của |G| đỉnh -- cùng cỡ với chính lượng mà α đang sửa (9-25%).
   Đặt gravity_at_measured=False để chạy đúng kiểu matlab.

===========================================================================
CHỐNG BÃO HOÀ TÍCH PHÂN
===========================================================================
Giống bộ cũ: mô-men đã chạm trần mà sai số vẫn cùng chiều đẩy -> NGỪNG tích luỹ.
Thêm kẹp cứng |∫e| <= i_limit. Trần mặc định lấy <limit effort> của URDF; trên
tay thật lqi_node truyền vào trần đã nhân tau_scale.

Chạy tự kiểm tra:  python3 tvlqr_controller.py
"""

from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import expm, solve_continuous_are

_Z3 = np.zeros((3, 3))
_I3 = np.eye(3)

# Bước sai phân trung tâm để lấy Astiff/Adamp. Bằng `delta` của setup_lqr.m.
FD_STEP = 1e-5

# Bộ trọng số mặc định -- KHÔNG phải số của matlab, xem mục 2 của docstring.
DEFAULT_MAX_INT_E = (0.002, 0.002, 0.002)   # rad·s   -- sai số tích luỹ chịu được
DEFAULT_MAX_E = (0.1, 0.1, 0.1)             # rad     -- như matlab
DEFAULT_MAX_DE = (2.0, 2.0, 2.0)            # rad/s   -- như matlab
DEFAULT_TAU_PENALTY_SCALE = 512.0           # nhân vào R; matlab tương đương 1.0

# Bộ số ĐÚNG NHƯ matlab, để đối chiếu trong mô phỏng.
MATLAB_MAX_INT_E = (100.0, 200.0, 100.0)
MATLAB_TAU_PENALTY_SCALE = 1.0


@dataclass
class TvlqrWeights:
    """Trọng số Bryson. Mỗi trường là mảng 3 phần tử (riêng từng khớp) hoặc số
    vô hướng (dùng chung). max_tau=None -> lấy <limit effort> của URDF, đúng như
    setup_lqr.m dùng [5, 40, 5]."""

    max_int_e: tuple = DEFAULT_MAX_INT_E
    max_e: tuple = DEFAULT_MAX_E
    max_de: tuple = DEFAULT_MAX_DE
    max_tau: tuple = None
    tau_penalty_scale: float = DEFAULT_TAU_PENALTY_SCALE

    def _v(self, x, n):
        a = np.asarray(x, dtype=float)
        return np.full(n, float(a)) if a.ndim == 0 else a.astype(float)

    def QR(self, tau_max, n: int = 3):
        """Q (3n x 3n) và R (n x n). Thứ tự trạng thái [∫e, e, ė] -- như matlab."""
        mie = self._v(self.max_int_e, n)
        me = self._v(self.max_e, n)
        mde = self._v(self.max_de, n)
        mt = self._v(tau_max if self.max_tau is None else self.max_tau, n)
        if np.any(mie <= 0) or np.any(me <= 0) or np.any(mde <= 0) or np.any(mt <= 0):
            raise ValueError("mọi max_* phải > 0 (luật Bryson lấy 1/max²)")
        Q = np.diag(np.concatenate([1.0 / mie**2, 1.0 / me**2, 1.0 / mde**2]))
        R = np.diag(float(self.tau_penalty_scale) / mt**2)
        return Q, R


class TvlqrController:
    """LQR biến thiên + tích phân. Mọi đại lượng ở PHÍA KHỚP (rad, Nm).

    Cùng giao diện compute(q, qd, q_ref, qd_ref, qdd_ref, dt) -> tau như
    LqiController, nên lqi_node.py chỉ phải đổi chỗ khởi tạo."""

    def __init__(
        self,
        dynamics,
        weights: TvlqrWeights = None,
        i_limit: float = 0.004,
        tau_limit=None,
        control_hz: float = 100.0,
        gravity_at_measured: bool = True,
        recompute_every: int = 1,
        fd_step: float = FD_STEP,
        require_discrete_stable: bool = True,
    ):
        self.dyn = dynamics
        self.n = int(dynamics.nq)
        self.weights = weights or TvlqrWeights()
        self.Q, self.R = self.weights.QR(dynamics.tau_max, self.n)
        self.i_limit = float(i_limit)
        self.tau_limit = (np.asarray(dynamics.tau_max, dtype=float)
                          if tau_limit is None else np.asarray(tau_limit, dtype=float))
        self.control_hz = float(control_hz)
        self.gravity_at_measured = bool(gravity_at_measured)
        self.recompute_every = max(1, int(recompute_every))
        self.fd_step = float(fd_step)
        self.K = self.gain(*self._nominal())
        self.reset()
        if require_discrete_stable:
            rep = self.stability_report()
            if not rep["stable_discrete"]:
                raise ValueError(
                    "Bộ trọng số này PHÂN KỲ khi rời rạc hoá ở "
                    f"{self.control_hz:g} Hz: bán kính phổ |z|max = "
                    f"{rep['spectral_radius']:.3f} (cần < 1), cực liên tục nhanh "
                    f"nhất {rep['fastest_pole']:.0f} rad/s so với Nyquist "
                    f"{np.pi*self.control_hz:.0f} rad/s.\n"
                    "Cách sửa: tăng tau_penalty_scale (mô-men đắt hơn -> hệ số "
                    "nhỏ hơn) hoặc tăng control_hz. Đặt "
                    "require_discrete_stable=False CHỈ khi chạy mô phỏng."
                )

    # ------------------------------------------------------------------
    def _nominal(self):
        return (self.dyn.q_min + self.dyn.q_max) / 2.0, np.zeros(self.n)

    def reset(self):
        self.integral = np.zeros(self.n)
        self.last = {}
        self._cycle = 0

    # ------------------------------------------------------------------
    def linearize(self, q_ref, qd_ref):
        """(A, B) của hệ sai số 9 trạng thái tại điểm làm việc. Sai phân trung
        tâm hai phía -- KHÔNG phải một phía: một phía có sai số bậc h (1e-5) còn
        hai phía bậc h² (1e-10), mà Astiff đi thẳng vào A nên sai số đó nhân
        trực tiếp vào hệ số điều khiển."""
        q = np.asarray(q_ref, dtype=float)
        qd = np.asarray(qd_ref, dtype=float)
        n, h = self.n, self.fd_step
        Minv = np.linalg.inv(self.dyn.mass_matrix(q))

        Astiff = np.empty((n, n))
        Adamp = np.empty((n, n))
        for i in range(n):
            qp, qm = q.copy(), q.copy()
            qp[i] += h
            qm[i] -= h
            Astiff[:, i] = (self.dyn.gravity(qp) - self.dyn.gravity(qm)) / (2.0 * h)

            vp, vm = qd.copy(), qd.copy()
            vp[i] += h
            vm[i] -= h
            Adamp[:, i] = (self.dyn.coriolis(q, vp) - self.dyn.coriolis(q, vm)) / (2.0 * h)

        A = np.block([[_Z3, _I3, _Z3],
                      [_Z3, _Z3, _I3],
                      [_Z3, -Minv @ Astiff, -Minv @ Adamp]])
        B = np.vstack([_Z3, _Z3, Minv])
        return A, B

    def gain(self, q_ref, qd_ref) -> np.ndarray:
        """K (n x 3n) tại điểm làm việc. Cột 0..n-1 nhân ∫e, n..2n-1 nhân e,
        2n..3n-1 nhân ė."""
        A, B = self.linearize(q_ref, qd_ref)
        P = solve_continuous_are(A, B, self.Q, self.R)
        return np.linalg.solve(self.R, B.T @ P)

    # ------------------------------------------------------------------
    def compute(self, q, qd, q_ref, qd_ref, qdd_ref, dt: float) -> np.ndarray:
        """Một chu kỳ điều khiển. Trả về mô-men khớp (Nm) đã kẹp giới hạn."""
        q = np.asarray(q, dtype=float)
        qd = np.asarray(qd, dtype=float)
        q_ref = np.asarray(q_ref, dtype=float)
        qd_ref = np.asarray(qd_ref, dtype=float)

        if self._cycle % self.recompute_every == 0:
            self.K = self.gain(q_ref, qd_ref)
        self._cycle += 1

        e = q - q_ref
        ed = qd - qd_ref
        x = np.concatenate([self.integral, e, ed])

        # feedforward nghịch động lực học TRÊN QUỸ ĐẠO THAM CHIẾU
        tau_ff = self.dyn.inverse_dynamics(q_ref, qd_ref, qdd_ref)
        if self.gravity_at_measured:
            # đổi G(q_ref) -> G(q); xem mục 3 docstring
            tau_ff = tau_ff - self.dyn.gravity(q_ref) + self.dyn.gravity(q)

        tau = tau_ff - self.K @ x
        tau_sat = np.clip(tau, -self.tau_limit, self.tau_limit)

        # chống windup: chạm trần VÀ vẫn đang đẩy cùng chiều -> ngừng tích luỹ
        saturated = np.abs(tau - tau_sat) > 1e-12
        pushing_further = saturated & (np.sign(tau) == np.sign(-e))
        self.integral = np.where(pushing_further, self.integral,
                                 self.integral + e * dt)
        self.integral = np.clip(self.integral, -self.i_limit, self.i_limit)

        self.last = dict(e=e, ed=ed, tau_ff=tau_ff, tau_raw=tau, tau=tau_sat,
                         integral=self.integral.copy(), saturated=saturated,
                         K=self.K.copy())
        return tau_sat

    # ------------------------------------------------------------------
    def integral_kick(self, i_limit: float = None) -> np.ndarray:
        """Mô-men (Nm) mà RIÊNG khâu tích phân có thể ra lệnh khi ∫e chạm kẹp.

        Thay cho công thức K[:,0]*i_limit*M_ii của bộ cũ, vốn sai đơn vị ở đây:
        K của bộ này đã là Nm/(rad·s), không phải (rad/s²)/(rad·s). Lấy chuẩn L1
        theo 3 cột tích phân, tức trường hợp xấu nhất khi cả 3 thành phần ∫e
        chạm kẹp cùng chiều có hại."""
        il = self.i_limit if i_limit is None else float(i_limit)
        n = self.n
        worst = np.zeros(n)
        for q in (self.dyn.q_min, self.dyn.q_max, (self.dyn.q_min + self.dyn.q_max) / 2):
            K = self.gain(q, np.zeros(n))
            worst = np.maximum(worst, np.abs(K[:, :n]).sum(axis=1) * il)
        return worst

    # ------------------------------------------------------------------
    def stability_report(self, samples: int = 24, seed: int = 0) -> dict:
        """Kiểm hệ SAI SỐ vòng kín trên nhiều điểm làm việc, cả liên tục lẫn
        RỜI RẠC HOÁ ZOH ở control_hz.

        Vì sao phải kiểm bản rời rạc: LQR liên tục luôn cho hệ ổn định về mặt
        toán học, nhưng cài đặt thật giữ mô-men không đổi suốt 1 chu kỳ (ZOH).
        Cực liên tục nhanh hơn ~1/dt thì bản rời rạc PHÂN KỲ dù bản liên tục
        hoàn hảo. Đây chính là chỗ bộ trọng số của setup_lqr.m vỡ ở 100 Hz."""
        dt = 1.0 / self.control_hz
        rng = np.random.default_rng(seed)
        qs = np.vstack([(self.dyn.q_min + self.dyn.q_max) / 2.0,
                        self.dyn.q_min, self.dyn.q_max,
                        rng.uniform(self.dyn.q_min, self.dyn.q_max,
                                    size=(max(0, samples - 3), self.n))])
        rho = 0.0
        fast = 0.0
        slow = np.inf
        for q in qs:
            A, B = self.linearize(q, np.zeros(self.n))
            P = solve_continuous_are(A, B, self.Q, self.R)
            K = np.linalg.solve(self.R, B.T @ P)
            ev = np.linalg.eigvals(A - B @ K)
            fast = max(fast, float(np.abs(ev).max()))
            slow = min(slow, float(np.abs(ev.real).min()))
            m, nb = B.shape[1], A.shape[0]
            E = expm(np.block([[A, B], [np.zeros((m, nb + m))]]) * dt)
            Ad, Bd = E[:nb, :nb], E[:nb, nb:]
            rho = max(rho, float(np.abs(np.linalg.eigvals(Ad - Bd @ K)).max()))
        return dict(spectral_radius=rho, stable_discrete=rho < 1.0,
                    fastest_pole=fast, slowest_pole=slow,
                    nyquist=np.pi * self.control_hz,
                    settling_time=(4.0 / slow if slow > 0 else np.inf))

    # ------------------------------------------------------------------
    def describe(self, q_nominal=None) -> str:
        w = self.weights
        rep = self.stability_report()
        lines = [
            "Bộ điều khiển: TVLQR (LQR biến thiên 9 trạng thái + tích phân), "
            "kiến trúc theo setup_lqr.m",
            f"  trọng số Bryson: max_int_e={np.round(w._v(w.max_int_e, self.n), 4)} rad·s  "
            f"max_e={np.round(w._v(w.max_e, self.n), 4)} rad  "
            f"max_de={np.round(w._v(w.max_de, self.n), 3)} rad/s",
            f"  max_tau={np.round(w._v(w.max_tau if w.max_tau is not None else self.dyn.tau_max, self.n), 3)} Nm"
            f"  x tau_penalty_scale={w.tau_penalty_scale:g}",
            f"  feedforward: inverse_dynamics(q_ref) "
            f"{'+ G(q_đo) - G(q_ref)' if self.gravity_at_measured else '(G lấy ở q_ref, đúng kiểu matlab)'}",
            f"  K tính lại mỗi {self.recompute_every} chu kỳ, lập theo (q_ref, q̇_ref)",
            f"  trần mô-men: {np.round(self.tau_limit, 3)} Nm   |∫e| <= {self.i_limit:g} rad·s",
            "",
            f"  ỔN ĐỊNH ({self.control_hz:g} Hz, ZOH): |z|max = {rep['spectral_radius']:.4f}"
            f"  -> {'ỔN ĐỊNH' if rep['stable_discrete'] else 'PHÂN KỲ'}",
            f"  cực liên tục: nhanh nhất {rep['fastest_pole']:.1f} rad/s "
            f"(Nyquist vòng {rep['nyquist']:.0f} rad/s), "
            f"chậm nhất {rep['slowest_pole']:.3f} rad/s "
            f"-> t_xác_lập ≈ {rep['settling_time']:.2f} s",
            f"  mô-men riêng khâu tích phân khi ∫e chạm kẹp: "
            f"{np.round(self.integral_kick(), 3)} Nm",
        ]
        q0 = self._nominal()[0] if q_nominal is None else np.asarray(q_nominal, float)
        K = self.gain(q0, np.zeros(self.n))
        lines.append(f"  K tại q = {np.round(q0, 3)} (Nm trên mỗi đơn vị trạng thái):")
        for i, name in enumerate(self.dyn.joint_names):
            lines.append(
                f"    {name:<15} ∫e:{np.array2string(K[i, :self.n], precision=3)}"
                f"  e:{np.array2string(K[i, self.n:2*self.n], precision=2)}"
                f"  ė:{np.array2string(K[i, 2*self.n:], precision=2)}")
        lines.append("    (K đổi theo tư thế -- đây chỉ là 1 điểm; xem biên độ ở "
                     "self-test của file này)")
        return "\n".join(lines)


# ======================================================================
# Tự kiểm tra: mô phỏng vòng kín trên CHÍNH mô hình, so mấy bộ trọng số
# ======================================================================
def _simulate(dyn, ctrl, ref, duration, ctrl_hz=100.0, sim_sub=10):
    """Vòng kín: controller chạy ở ctrl_hz (ZOH), plant tích phân ở
    ctrl_hz*sim_sub bằng Runge-Kutta 4, có cả ma sát của mô hình.
    Trả về (t, q, q_ref, tau)."""
    dt_c = 1.0 / ctrl_hz
    dt_s = dt_c / sim_sub
    q, qd = ref(0.0)[0].copy(), ref(0.0)[1].copy()
    ts, qs, qrs, taus = [], [], [], []

    def acc(q_, qd_, tau_):
        rhs = tau_ - dyn.nonlinear(q_, qd_) - dyn.friction(qd_)
        return np.linalg.solve(dyn.mass_matrix(q_), rhs)

    n_steps = int(round(duration / dt_c))
    for k in range(n_steps):
        t = k * dt_c
        qr, qdr, qddr = ref(t)
        tau = ctrl.compute(q, qd, qr, qdr, qddr, dt_c)
        ts.append(t); qs.append(q.copy()); qrs.append(qr.copy()); taus.append(tau.copy())
        for _ in range(sim_sub):                       # ZOH: tau giữ nguyên
            k1v = acc(q, qd, tau)
            k2v = acc(q + 0.5*dt_s*qd, qd + 0.5*dt_s*k1v, tau)
            k3v = acc(q + 0.5*dt_s*(qd + 0.5*dt_s*k1v), qd + 0.5*dt_s*k2v, tau)
            k4v = acc(q + dt_s*(qd + 0.5*dt_s*k2v), qd + dt_s*k3v, tau)
            q = q + dt_s*(qd + dt_s*(k1v + k2v + k3v)/6.0)
            qd = qd + dt_s*(k1v + 2*k2v + 2*k3v + k4v)/6.0
        if not np.all(np.isfinite(q)) or np.abs(q).max() > 1e3:
            break
    return (np.array(ts), np.array(qs), np.array(qrs), np.array(taus))


if __name__ == "__main__":
    import sys

    from arm_dynamics import ArmDynamics

    urdf = sys.argv[1] if len(sys.argv) > 1 else "gim_arm.urdf"
    dyn = ArmDynamics(urdf)
    n = dyn.nq
    q0 = (dyn.q_min + dyn.q_max) / 2.0

    print("=" * 78)
    print("1) Astiff/Adamp có đúng là đạo hàm của G và C q̇ không")
    print("=" * 78)
    ctrl = TvlqrController(dyn)
    A, B = ctrl.linearize(q0, np.zeros(n))
    Minv = np.linalg.inv(dyn.mass_matrix(q0))
    # kiểm gián tiếp: -M⁻¹Astiff phải bằng d(q̈)/dq quanh điểm cân bằng tĩnh
    h = 1e-6
    num = np.empty((n, n))
    for i in range(n):
        qp, qm = q0.copy(), q0.copy()
        qp[i] += h
        qm[i] -= h
        num[:, i] = -(Minv @ (dyn.gravity(qp) - dyn.gravity(qm)) / (2 * h))
    print(f"  |(-M⁻¹∂G/∂q) khối A[6:9,3:6] - sai phân độc lập| = "
          f"{np.abs(A[2*n:, n:2*n] - num).max():.3e}")
    print(f"  B[6:9] - M⁻¹ = {np.abs(B[2*n:] - Minv).max():.3e}  (phải là 0)")
    print(f"  A[0:3,3:6] = I? {np.allclose(A[:n, n:2*n], np.eye(n))}   "
          f"A[3:6,6:9] = I? {np.allclose(A[n:2*n, 2*n:], np.eye(n))}")
    print()

    print("=" * 78)
    print("2) Bộ trọng số mặc định")
    print("=" * 78)
    print(ctrl.describe(q_nominal=q0))
    print()

    print("=" * 78)
    print("3) Bộ trọng số ĐÚNG NHƯ matlab -- ở 100 Hz thì sao")
    print("=" * 78)
    wm = TvlqrWeights(max_int_e=MATLAB_MAX_INT_E,
                      tau_penalty_scale=MATLAB_TAU_PENALTY_SCALE)
    try:
        TvlqrController(dyn, weights=wm)
        print("  (khởi tạo được -- không chặn)")
    except ValueError as ex:
        print("  BỊ CHẶN, đúng như mong đợi:")
        for ln in str(ex).splitlines():
            print("   ", ln)
    print()

    print("=" * 78)
    print("4) Mô phỏng vòng kín: bám bước 0.10 rad ở cả 3 khớp, 100 Hz")
    print("=" * 78)

    class _Step:
        def __init__(self, q_start, amp, t_on=0.5):
            self.q0 = np.asarray(q_start, float)
            self.amp = np.asarray(amp, float)
            self.t_on = float(t_on)

        def at(self, t):
            z = np.zeros_like(self.q0)
            qr = self.q0 + (self.amp if t >= self.t_on else 0.0)
            return qr, z, z.copy()

    variants = [
        ("mặc định (int 0.002, R x512)", TvlqrWeights()),
        ("int 0.001 (bám hơn, quá kick)", TvlqrWeights(max_int_e=(0.001,)*3)),
        ("R x2048 (mềm hơn, ~băng thông bộ cũ)", TvlqrWeights(tau_penalty_scale=2048.0)),
        ("không tích phân hữu ích (int matlab)",
         TvlqrWeights(max_int_e=MATLAB_MAX_INT_E)),
    ]
    print(f"  {'bộ trọng số':<38} {'|z|max':>7} {'e_RMS(mrad)':>12} "
          f"{'e_cuối(mrad)':>13} {'|τ|đỉnh(Nm)':>26}")
    step = _Step(q0, np.full(n, 0.10))
    for label, w in variants:
        try:
            c = TvlqrController(dyn, weights=w, i_limit=0.05,
                                require_discrete_stable=False)
        except ValueError as ex:
            print(f"  {label:<38} bị chặn: {ex}")
            continue
        rep = c.stability_report(samples=6)
        t, q, qr, tau = _simulate(dyn, c, step.at, duration=6.0)
        if len(t) < 500:
            print(f"  {label:<38} {rep['spectral_radius']:7.3f}  PHÂN KỲ trong mô phỏng")
            continue
        err = q - qr
        m = t >= 1.0                       # bỏ 0.5 s trước bước + 0.5 s đầu
        print(f"  {label:<38} {rep['spectral_radius']:7.3f} "
              f"{1000*np.sqrt((err[m]**2).mean()):12.3f} "
              f"{1000*np.abs(err[-1]).max():13.3f} "
              f"{str(np.round(np.abs(tau).max(axis=0), 2)):>26}")
    print()
    print("  e_cuối = sai số xác lập lớn nhất trên 3 khớp ở cuối 6 s -- đây là chỗ")
    print("  khâu tích phân có tác dụng hay không hiện ra rõ nhất.")
