"""
lqi_controller.py — bộ điều khiển LQI (LQR + tích phân) cho GIM Arm 3DOF,
dựng trên mô hình Lagrange trong arm_dynamics.py.

===========================================================================
Ý TƯỞNG: 2 TẦNG, ĐỪNG LẪN
===========================================================================
Tay máy là hệ PHI TUYẾN (M phụ thuộc q, có Coriolis, có trọng lực), mà LQR/LQI
là lý thuyết cho hệ TUYẾN TÍNH. Nối 2 thứ đó bằng 2 tầng:

  Tầng 1 -- TUYẾN TÍNH HOÁ PHẢN HỒI (computed torque):
      τ = M(q)·u + C(q,q̇)q̇ + G(q)   [+ τ_masat(q̇) nếu bật, MẶC ĐỊNH TẮT]
    Thay vào phương trình Lagrange M q̈ + C q̇ + G + τ_ms = τ thì mọi thứ phi
    tuyến triệt tiêu, còn đúng:
      q̈ = u
    Tức là sau tầng 1, mỗi khớp chỉ còn là một KHÂU TÍCH PHÂN KÉP, và 3 khớp
    tách rời hẳn nhau (không còn xen kênh). Đây là chỗ M, C, G của Lagrange
    được dùng -- và cũng là chỗ LQI ăn đứt PID: PID phải dùng MỘT bộ hệ số cố
    định cho cả dải quán tính thay đổi 5.6 lần (đo được ở base_joint, xem
    arm_dynamics.py), còn ở đây M(q) được tính lại mỗi chu kỳ.

  Tầng 2 -- LQI trên hệ tuyến tính đã có:
    Đặt e = q - q_ref, và u = q̈_ref + v (q̈_ref là gia tốc mong muốn của quỹ
    đạo, tức phần feedforward), thì ë = v. Thêm biến tích phân xi = ∫e dt:

        x = [xi, e, ė]ᵀ        ẋ = A x + B v
        A = [[0,1,0],          B = [0,
             [0,0,1],               0,
             [0,0,0]]               1]

    LQR cực tiểu hoá  J = ∫ (xᵀQx + r·v²) dt  ->  v = -K x = -(k_i·xi + k_e·e + k_v·ė)
    K giải từ phương trình Riccati đại số (CARE): K = R⁻¹BᵀP.

    Vì sau tầng 1 ba khớp đã tách rời và giống hệt nhau, bài toán 9 trạng thái
    rã thành 3 bài toán 3 trạng thái GIỐNG NHAU -> chỉ cần giải CARE 3x3 một
    lần. (Nếu đặt Q khác nhau cho từng khớp thì giải 3 lần, code dưới hỗ trợ.)

===========================================================================
VÌ SAO LQI CHỨ KHÔNG PHẢI LQR
===========================================================================
LQR thuần (không có xi) cho sai số bám XÁC LẬP KHÁC 0 mỗi khi có thành phần
mô-men mà mô hình không biết. Ở tay máy này thành phần đó có thật và không
nhỏ: ma sát khô của hộp số (ước lượng 0.256 Nm ở shoulder) mà 2 hệ số ma sát
trong repo đang ghi rõ là "SỐ ĐẶT TẠM, chưa hiệu chỉnh từ log CAN thật".
Khâu tích phân xi triệt tiêu đúng loại sai số này -- đó là lý do chọn LQI.
Giá phải trả: thêm 1 cực -> dễ vọt lố hơn, và phải chống bão hoà tích phân
(xem chống windup bên dưới).

VÌ SAO MẶC ĐỊNH **TẮT** BÙ MA SÁT (friction_ff=False) -- đây là kết quả ĐO,
không phải lựa chọn theo cảm tính. Chạy compare_pid_lqi.py với 4 cấu hình,
sai số đầu tay RMS trên 1 vòng:
      tắt hoàn toàn                    0.1064 mm
      bật, smooth_eps=0.02             0.1559 mm   <- XẤU HƠN 47%
      bật, smooth_eps=0.1              0.1379 mm
      bật, smooth_eps=0.5              0.1012 mm   (hơn được 5%)
Bật với eps nhỏ không chỉ tệ hơn về số: đồ thị hiện rõ dao động ~2 Hz biên độ
±0.1 độ ở shoulder/elbow, tức tay RUNG. Nguyên nhân: eps=0.02 rad/s trong khi
quỹ đạo chỉ chạy tối đa 0.118 rad/s, nên tanh(q̇/eps) gần như thành hàm dấu ->
mỗi lần q̇ đổi dấu, mô-men bù nhảy ±0.256 Nm ở shoulder. Bù quá tay như vậy
đóng vai trò GIẢM CHẤN ÂM và đẻ ra chu trình giới hạn (limit cycle). Với
eps=0.5 thì tanh gần như tuyến tính trong dải tốc độ thật -> hoá ra chỉ còn là
bù nhớt nhẹ, hết rung, nhưng cái lợi 5% không đáng để gánh thêm rủi ro trên
thiết bị đeo lên người, nhất là khi 2 hệ số ma sát vẫn chưa hiệu chỉnh thật.
Kết luận: để khâu tích phân gánh ma sát. Nếu vẫn muốn bật, giữ smooth_eps lớn
hơn ít nhất 4 lần tốc độ khớp lớn nhất của quỹ đạo.

===========================================================================
CÁCH CHỈNH (TUNE)
===========================================================================
Đa thức đặc trưng vòng kín sau tầng 1 là:
        s³ + k_v·s² + k_e·s + k_i = 0
nên có 2 đường chỉnh, dùng đường nào cũng được:

 (a) Theo BĂNG THÔNG (trực giác hơn, khuyến nghị bắt đầu từ đây):
     gains_from_bandwidth(w) đặt cả 3 cực trùng nhau tại -w:
        k_v = 3w,  k_e = 3w²,  k_i = w³
     w là băng thông vòng kín (rad/s). Chọn w thế nào:
       - w PHẢI nhỏ hơn nhiều tần số vòng điều khiển. Vòng ros2_control chạy
         100 Hz = 628 rad/s -> giữ w <= ~1/20 của nó, tức w <= 30 rad/s.
       - Nhiễu vận tốc từ encoder bị nhân với k_v = 3w, nên w lớn = mô-men
         rung. Trên thiết bị đeo lên người, rung là thứ cấm.
       - Bắt đầu w = 8 rad/s (~1.3 Hz), tăng dần tới khi sai số bám đủ nhỏ
         hoặc bắt đầu thấy rung/kêu, rồi lùi lại 30%.
 (b) Theo TRỌNG SỐ Q, R (đúng bài LQR):
       q_pos lớn  -> ưu tiên bám vị trí, cứng hơn
       q_vel lớn  -> ưu tiên êm, giảm vọt lố, chậm hơn
       q_int lớn  -> khử sai số xác lập nhanh hơn, nhưng dễ vọt lố + windup
       r lớn      -> tiết kiệm mô-men, mềm hơn
     Chỉ TỈ LỆ Q/r có ý nghĩa, nhân cả Q và r cho cùng một số thì K không đổi.
     describe() in ra cực vòng kín + cực chậm nhất của bộ trọng số, đối chiếu
     trực tiếp được với w ở cách (a).

CHỐNG BÃO HOÀ TÍCH PHÂN (anti-windup): khi mô-men đã chạm giới hạn URDF
(5/40/5 Nm) mà sai số vẫn cùng chiều thì NGỪNG tích luỹ xi. Không có bước này,
xi phình lên trong lúc bão hoà rồi đẩy tay vọt qua bên kia khi thoát bão hoà.
Thêm 1 lớp kẹp cứng |xi| <= i_limit phòng trường hợp kẹt cơ khí.

Chạy tự kiểm tra:  python3 lqi_controller.py
"""

from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_continuous_are

# Hệ tuyến tính SAU khi đã tuyến tính hoá phản hồi: [xi, e, ė], vào là v = ë
A_LIN = np.array([[0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0],
                  [0.0, 0.0, 0.0]])
B_LIN = np.array([[0.0], [0.0], [1.0]])


@dataclass
class LqiWeights:
    """Trọng số LQR. Mỗi trường là số vô hướng (dùng chung cho 3 khớp) hoặc
    mảng 3 phần tử (riêng từng khớp).

    Mặc định dưới đây KHÔNG phải số bốc đại: quét lưới Q rồi chọn bộ cho cực
    vòng kín [-28.8, -10.0, -8.5] -- toàn cực THỰC (không dao động, không vọt
    lố) và cực chậm nhất 8.5 rad/s ~ 1.35 Hz. Chọn 8.5 rad/s vì vòng
    ros2_control chạy 100 Hz = 628 rad/s, tức băng thông chỉ bằng 1/74 tần số
    lấy mẫu -- rất an toàn về mặt rời rạc hoá, và đủ chậm để nhiễu vận tốc
    encoder không bị khuếch đại thành rung trên thiết bị đeo lên người.
    Chỉ TỈ LỆ Q/r có ý nghĩa: nhân cả 4 số cho cùng một hằng số thì K không đổi.
    """
    q_int: float = 6.0e6     # phạt ∫e  -> khử sai số xác lập
    q_pos: float = 1.5e5     # phạt e   -> độ cứng bám vị trí
    q_vel: float = 1.0e3     # phạt ė   -> độ êm
    r: float = 1.0           # phạt gia tốc điều khiển -> tiết kiệm mô-men

    def as_arrays(self, n: int = 3):
        out = []
        for field in (self.q_int, self.q_pos, self.q_vel, self.r):
            arr = np.asarray(field, dtype=float)
            out.append(np.full(n, float(arr)) if arr.ndim == 0 else arr.astype(float))
        return out


def lqi_gain_single(q_int: float, q_pos: float, q_vel: float, r: float) -> np.ndarray:
    """Giải CARE cho MỘT khớp. Trả về K = [k_i, k_e, k_v]."""
    Q = np.diag([q_int, q_pos, q_vel])
    R = np.array([[r]])
    P = solve_continuous_are(A_LIN, B_LIN, Q, R)
    return (np.linalg.solve(R, B_LIN.T @ P)).ravel()


def lqi_gains(weights: LqiWeights, n: int = 3) -> np.ndarray:
    """K cho cả n khớp, dạng (n, 3)."""
    qi, qp, qv, r = weights.as_arrays(n)
    return np.array([lqi_gain_single(qi[i], qp[i], qv[i], r[i]) for i in range(n)])


def gains_from_bandwidth(omega: float) -> np.ndarray:
    """Đặt cả 3 cực vòng kín trùng nhau tại -omega (đáp ứng tới hạn, không vọt
    lố dao động). Trả về K = [k_i, k_e, k_v] = [w³, 3w², 3w]."""
    w = float(omega)
    return np.array([w ** 3, 3.0 * w ** 2, 3.0 * w])


def closed_loop_poles(K) -> np.ndarray:
    """Nghiệm của s³ + k_v s² + k_e s + k_i = 0."""
    k_i, k_e, k_v = K
    return np.roots([1.0, k_v, k_e, k_i])


def equivalent_bandwidth(K) -> float:
    """Trung bình nhân độ lớn 3 cực (= k_i^(1/3)). Với hệ số sinh từ
    gains_from_bandwidth(w) thì trả lại đúng w.

    LƯU Ý khi đọc số này: nó chỉ phụ thuộc k_i, nên hai bộ hệ số có cùng k_i
    sẽ ra cùng một giá trị dù đáp ứng khác hẳn nhau. Muốn biết đáp ứng NHANH
    hay CHẬM thì phải xem dominant_pole() -- cực chậm nhất mới là cái quyết
    định thời gian xác lập."""
    return float(np.abs(closed_loop_poles(K)).prod() ** (1.0 / 3.0))


def dominant_pole(K) -> float:
    """|phần thực| của cực CHẬM NHẤT -- đây mới là băng thông thực dụng.
    Thời gian xác lập 2% xấp xỉ 4 / dominant_pole."""
    return float(np.abs(np.real(closed_loop_poles(K))).min())


class LqiController:
    """LQI + tuyến tính hoá phản hồi. Mọi đại lượng ở phía KHỚP (rad, Nm)."""

    def __init__(
        self,
        dynamics,
        weights: LqiWeights = None,
        bandwidth: float = None,
        friction_ff: bool = False,
        i_limit: float = 0.5,
        tau_limit=None,
    ):
        """
        dynamics   : ArmDynamics
        weights    : trọng số LQR (bỏ qua nếu truyền bandwidth)
        bandwidth  : nếu đặt, dùng gán cực thay cho LQR (xem cách chỉnh (a))
        friction_ff: có bù ma sát ước lượng hay không. MẶC ĐỊNH TẮT -- đo
                     được là bật vào thì xấu hơn và gây rung, xem docstring
                     đầu file. Bật lên kèm smooth_eps lớn nếu muốn thử lại.
        i_limit    : kẹp cứng |∫e| (rad·s)
        tau_limit  : giới hạn mô-men; mặc định lấy <limit effort> của URDF
        """
        self.dyn = dynamics
        self.n = dynamics.nq
        if bandwidth is not None:
            self.K = np.tile(gains_from_bandwidth(bandwidth), (self.n, 1))
            self.weights = None
            self.bandwidth = float(bandwidth)
        else:
            self.weights = weights or LqiWeights()
            self.K = lqi_gains(self.weights, self.n)
            self.bandwidth = None
        self.friction_ff = friction_ff
        self.i_limit = float(i_limit)
        self.tau_limit = (np.asarray(dynamics.tau_max, dtype=float)
                          if tau_limit is None else np.asarray(tau_limit, dtype=float))
        self.reset()

    def reset(self):
        self.integral = np.zeros(self.n)
        self.last = {}

    def compute(self, q, qd, q_ref, qd_ref, qdd_ref, dt: float) -> np.ndarray:
        """Một chu kỳ điều khiển. Trả về mô-men khớp (Nm) đã kẹp giới hạn."""
        q = np.asarray(q, dtype=float)
        qd = np.asarray(qd, dtype=float)
        e = q - np.asarray(q_ref, dtype=float)
        ed = qd - np.asarray(qd_ref, dtype=float)

        # --- Tầng 2: luật LQI trên hệ đã tuyến tính hoá ---
        # v = -(k_i·∫e + k_e·e + k_v·ė). Dấu trừ vì e = q - q_ref.
        v = -(self.K[:, 0] * self.integral + self.K[:, 1] * e + self.K[:, 2] * ed)
        u = np.asarray(qdd_ref, dtype=float) + v      # gia tốc khớp mong muốn

        # --- Tầng 1: tuyến tính hoá phản hồi (computed torque) ---
        # Dùng q, q̇ ĐO ĐƯỢC (không phải q_ref) -- đây là bản computed-torque
        # đúng nghĩa; nếu dùng q_ref thì thành feedforward thuần và sẽ mất tác
        # dụng khử phi tuyến ngay khi tay bị lệch khỏi quỹ đạo.
        tau = self.dyn.mass_matrix(q) @ u + self.dyn.nonlinear(q, qd)
        if self.friction_ff:
            tau = tau + self.dyn.friction(qd)

        tau_sat = np.clip(tau, -self.tau_limit, self.tau_limit)

        # --- Chống windup: chỉ tích luỹ khi KHÔNG bão hoà theo chiều đang đẩy ---
        saturated = np.abs(tau - tau_sat) > 1e-12
        pushing_further = saturated & (np.sign(tau) == np.sign(-e))
        self.integral = np.where(
            pushing_further, self.integral, self.integral + e * dt
        )
        self.integral = np.clip(self.integral, -self.i_limit, self.i_limit)

        self.last = dict(e=e, ed=ed, v=v, u=u, tau_raw=tau, tau=tau_sat,
                         integral=self.integral.copy(), saturated=saturated)
        return tau_sat

    # ------------------------------------------------------------------
    def describe(self, q_nominal=None) -> str:
        """In hệ số, cực vòng kín, và hệ số PID TƯƠNG ĐƯƠNG tại một tư thế --
        để đối chiếu trực tiếp với bộ PID cascade đang chạy."""
        lines = []
        if self.bandwidth is not None:
            lines.append(f"Nguồn hệ số: gán cực, băng thông w = {self.bandwidth:g} rad/s "
                         f"({self.bandwidth/(2*np.pi):.2f} Hz)")
        else:
            w = self.weights
            lines.append(f"Nguồn hệ số: LQR, Q = diag(q_int={w.q_int}, q_pos={w.q_pos}, "
                         f"q_vel={w.q_vel}), R = {w.r}")
        for i, name in enumerate(self.dyn.joint_names):
            k_i, k_e, k_v = self.K[i]
            poles = closed_loop_poles(self.K[i])
            lines.append(
                f"  {name:<15} k_i={k_i:9.2f}  k_e={k_e:8.2f}  k_v={k_v:7.2f}"
                f"  | cực: {np.array2string(poles, precision=2)}"
                f"  | cực chậm nhất={dominant_pole(self.K[i]):.2f} rad/s"
                f"  | t_xác_lập≈{4.0 / dominant_pole(self.K[i]):.2f}s"
            )
        if q_nominal is not None:
            M = self.dyn.mass_matrix(q_nominal)
            lines.append(f"  Hệ số PID tương đương tại q = {np.round(q_nominal, 3)} "
                         f"(nhân với M_ii, đơn vị Nm):")
            for i, name in enumerate(self.dyn.joint_names):
                k_i, k_e, k_v = self.K[i] * M[i, i]
                lines.append(f"    {name:<15} Kp={k_e:8.3f} Nm/rad   "
                             f"Kd={k_v:7.3f} Nm/(rad/s)   Ki={k_i:8.3f} Nm/(rad·s)")
            lines.append("    (LQI KHÔNG dùng bộ số cố định này -- nó nhân lại M(q) mỗi "
                         "chu kỳ, nên hệ số hiệu dụng tự đổi theo tư thế.)")
        return "\n".join(lines)


if __name__ == "__main__":
    import sys

    from arm_dynamics import ArmDynamics

    urdf = sys.argv[1] if len(sys.argv) > 1 else "gim_arm.urdf"
    dyn = ArmDynamics(urdf)
    q0 = (dyn.q_min + dyn.q_max) / 2

    print("=" * 78)
    print("1) LQR: cùng một Q/R, xem hệ số và cực vòng kín")
    print("=" * 78)
    ctrl = LqiController(dyn, weights=LqiWeights())
    print(ctrl.describe(q_nominal=q0))
    print()

    print("=" * 78)
    print("2) Gán cực theo băng thông: kiểm k = [w³, 3w², 3w] và cực trùng tại -w")
    print("=" * 78)
    for w in (4.0, 8.0, 16.0):
        K = gains_from_bandwidth(w)
        poles = closed_loop_poles(K)
        print(f"  w={w:5.1f} rad/s -> K={np.round(K, 2)}  cực={np.round(poles, 3)}  "
              f"cực chậm nhất={dominant_pole(K):.3f}")
    print()

    print("=" * 78)
    print("3) Trọng số ảnh hưởng thế nào (mỗi dòng đổi ĐÚNG 1 trọng số)")
    print("=" * 78)
    base = LqiWeights()
    variants = [
        ("mặc định", base),
        ("q_pos x10  (bám cứng hơn)", LqiWeights(base.q_int, base.q_pos * 10, base.q_vel, base.r)),
        ("q_vel x10  (êm hơn)", LqiWeights(base.q_int, base.q_pos, base.q_vel * 10, base.r)),
        ("q_int x10  (khử sai số xác lập nhanh)", LqiWeights(base.q_int * 10, base.q_pos, base.q_vel, base.r)),
        ("r x100     (tiết kiệm mô-men)", LqiWeights(base.q_int, base.q_pos, base.q_vel, base.r * 100)),
    ]
    print(f"  {'bộ trọng số':<40} {'k_i':>9} {'k_e':>8} {'k_v':>7} {'cực chậm':>7}")
    for label, w in variants:
        K = lqi_gain_single(w.q_int, w.q_pos, w.q_vel, w.r)
        print(f"  {label:<40} {K[0]:9.1f} {K[1]:8.1f} {K[2]:7.2f} "
              f"{dominant_pole(K):7.2f}")
    print()

    print("=" * 78)
    print("4) Kiểm tính đúng: tuyến tính hoá phản hồi có thật sự cho q̈ = u không")
    print("=" * 78)
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(200):
        q = rng.uniform(dyn.q_min, dyn.q_max)
        qd = rng.uniform(-1.0, 1.0, size=3)
        u_want = rng.uniform(-2.0, 2.0, size=3)
        tau = dyn.mass_matrix(q) @ u_want + dyn.nonlinear(q, qd) + dyn.friction(qd)
        qdd = dyn.forward_dynamics(q, qd, tau)      # giải ngược lại từ τ
        worst = max(worst, np.abs(qdd - u_want).max())
    print(f"  |q̈ thực tế - u mong muốn| lớn nhất trên 200 mẫu = {worst:.3e}")
    print("  =>", "ĐÚNG: tầng 1 biến hệ phi tuyến thành 3 khâu tích phân kép độc lập"
          if worst < 1e-9 else "SAI: xem lại mass_matrix/nonlinear/friction")
