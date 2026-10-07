"""
sweep_trajectory.py — ĐỊNH NGHĨA DUY NHẤT của quỹ đạo quét trước mặt người
đeo, dùng chung cho đường position và runner mô-men PID/LQR/MPC/SMC.

Để chung 1 chỗ vì đây là quỹ đạo chạy trên thiết bị ĐEO VÀO NGƯỜI: nếu tham
số bị chép ra 2-3 nơi rồi sửa lệch nhau, cái đã kiểm chứng trong mô phỏng sẽ
không còn là cái chạy trên tay thật.

Vì sao quét theo GÓC chứ không phải ellipse phẳng: đã quét FK toàn dải khớp
-- vùng với được của tay 3 DOF này là 1 LỚP VỎ CẦU quanh tâm vai
(r ~ 0.39..0.60 m), không phải khối đặc. Ellipse phẳng biên độ lớn luôn chọc
thủng vỏ ở 2 đầu trục dài (đã thử: mọi ellipse phẳng bán trục >= 7cm trong
vùng phía trước đều fail IK). Xem shapes.shoulder_sweep().
"""

import numpy as np
from scipy.interpolate import CubicSpline

try:
    from gim_control.shapes import shoulder_sweep, discretize
except ImportError:
    from shapes import shoulder_sweep, discretize


SHOULDER_PIVOT = (0.031381, -0.532211, 0.611271)

TOOL_OFFSET = (0.4031, 0.049, -0.029)

SWEEP = dict(
    pivot=SHOULDER_PIVOT,
    radius=0.52,         # độ vươn tay (m) -- lùi từ 0.56 về giữa vỏ cầu tầm
                         # với (0.41..0.59) nên còn dư địa cả 2 phía, đó là chỗ
                         # cond(J) tốt nhất và cũng là chỗ khuỷu duỗi thoải mái
    az_center_deg=8.0,   # tâm quét ngang, 0 = thẳng trước mặt, + = sang phải.
                         # Kéo từ 30 về 8: giới hạn base_joint mới cắt mất phía
                         # phải, dư địa còn lại nằm ở phía trong (az âm)
    el_center_deg=-12.0, # hạ tâm quét thêm 4 độ (~3.2 cm tại bán kính 0.52 m)
    az_amp_deg=30.4,     # biên độ ngang -> quét az -22..+38 độ
    el_amp_deg=13.6,     # biên độ dọc  -> quét el -22..+6 độ
    radius_amp=0.04,     # "thở" độ vươn +-4cm -> KHUỶU cũng có biên độ thật
                         # (37.5 độ thay vì 12.3). Không có nó thì khuỷu gần
                         # như đứng yên, tập vai xong khuỷu vẫn cứng.
)

N_POINTS = 90        # số điểm 1 vòng
DT = 0.3             # giây giữa 2 điểm -> 1 vòng ~27s
TRANSITION_TIME = 5.0  # giây để đi êm từ tư thế hiện tại về điểm đầu quỹ đạo
RETURN_TIME = 5.0      # giây để đi êm từ cuối quỹ đạo về lại tư thế ban đầu

# Ngưỡng an toàn khi chạy trên tay thật
MAX_ERR_MM = 0.1
MAX_COND = 15.0
MIN_JOINT_MARGIN_RAD = 0.05

BODY_BOXES = {
    # thân + đầu: mặt trước ngực ~10cm trước tâm khớp vai
    "thân/đầu": dict(x=(-0.28, 0.34), y=(-0.85, -0.42), z=(0.28, 0.95)),
    # đùi nằm trên mặt ghế, từ hông ra tới đầu gối (~mép trước ghế)
    "đùi/ghế": dict(x=(-0.32, 0.38), y=(-0.85, -0.19), z=(0.10, 0.32)),
    # cẳng chân buông thẳng xuống từ đầu gối
    "cẳng chân": dict(x=(-0.32, 0.38), y=(-0.26, -0.12), z=(0.00, 0.30)),
}
MIN_BODY_CLEARANCE_M = 0.08

MAX_JOINT_SPEED_FRACTION = 0.25


class SmoothJointProfile:
    """Profile khớp một vòng, khởi hành và kết thúc với qdot=qddot=0.

    Spline tuần hoàn giữ hình dạng đường đi qua các waypoint IK. Đa thức
    minimum-jerk bậc 5 làm time-scaling cho pha chạy, nhờ đó vận tốc không còn
    là hệ quả ngầm của các điểm position: q, qdot và qddot đều được định nghĩa
    giải tích và đồng bộ từ cùng một nguồn.
    """

    def __init__(self, q_way, dt_way=DT):
        q_way = np.asarray(q_way, dtype=float)
        if q_way.ndim != 2 or len(q_way) < 3:
            raise ValueError("q_way phải có dạng (N, số_khớp), với N >= 3")
        if dt_way <= 0:
            raise ValueError("dt_way phải > 0")

        self.n = len(q_way)
        self.dt_way = float(dt_way)
        self.duration = self.n * self.dt_way
        phase_way = np.arange(self.n + 1, dtype=float)
        q_closed = np.vstack([q_way, q_way[:1]])
        self.sp = CubicSpline(
            phase_way, q_closed, axis=0, bc_type="periodic")

    def at(self, t):
        """Trả về (q, qdot, qddot) tại thời gian t, t có thể là scalar/array."""
        ta = np.asarray(t, dtype=float)
        u = np.clip(ta / self.duration, 0.0, 1.0)

        # h(0)=0, h(1)=1 và hdot=hddot=0 ở cả hai đầu.
        h = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
        hd = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / self.duration
        hdd = (60.0 * u - 180.0 * u**2 + 120.0 * u**3) / self.duration**2

        phase = self.n * h
        phase_dot = self.n * hd
        phase_ddot = self.n * hdd
        # phase=n ở đúng điểm cuối tương đương phase=0 của spline tuần hoàn.
        phase_wrapped = np.mod(phase, self.n)

        q = self.sp(phase_wrapped)
        q_phase = self.sp(phase_wrapped, 1)
        q_phase2 = self.sp(phase_wrapped, 2)
        if ta.ndim == 0:
            qd = q_phase * phase_dot
            qdd = q_phase2 * phase_dot**2 + q_phase * phase_ddot
        else:
            qd = q_phase * phase_dot[..., None]
            qdd = (q_phase2 * phase_dot[..., None]**2
                   + q_phase * phase_ddot[..., None])
        return q, qd, qdd

    def sample(self, sample_dt=DT):
        """Lấy mẫu đều và luôn bao gồm chính xác điểm kết thúc."""
        if sample_dt <= 0:
            raise ValueError("sample_dt phải > 0")
        count = max(1, int(np.ceil(self.duration / sample_dt)))
        t = np.linspace(0.0, self.duration, count + 1)
        q, qd, qdd = self.at(t)
        return t, q, qd, qdd


def build_positions(n_points: int = N_POINTS):
    """Danh sách điểm (x,y,z) khép kín của quỹ đạo."""
    return discretize(shoulder_sweep(**SWEEP), n_points=n_points, close_loop=True)


def solve(kin, positions=None):
    """Giải IK cả vòng. Chạy 2 lượt: lượt 2 gieo bằng nghiệm điểm cuối để chỗ
    khép kín (điểm cuối -> điểm đầu) cũng liền mạch, không giật."""
    if positions is None:
        positions = build_positions()
    results = kin.solve_trajectory(positions)
    results = kin.solve_trajectory(positions, q_init=results[-1].q)
    return positions, results


def body_clearance(positions):
    """Khoảng hở nhỏ nhất từ đầu tay tới từng hộp bao người/ghế.

    Trả về dict {tên_hộp: khoảng_hở_m}. Khoảng cách điểm-tới-hộp theo trục
    (0 khi điểm nằm TRONG hộp), đúng nghĩa "còn cách bao nhiêu thì chạm".

    Chỉ kiểm ĐẦU TAY, không kiểm cả cánh tay: link cánh tay bám sát tay người
    đeo nên nó ở đâu là do vai người quyết định, không phải chỗ quỹ đạo tự do
    đi vào. Đầu tay mới là phần vươn ra xa và có thể quật vào người."""
    P = np.array(positions)
    out = {}
    for name, box in BODY_BOXES.items():
        d2 = np.zeros(len(P))
        for i, axis in enumerate("xyz"):
            lo_b, hi_b = box[axis]
            d2 += np.maximum(np.maximum(lo_b - P[:, i], P[:, i] - hi_b), 0.0) ** 2
        out[name] = float(np.sqrt(d2).min())
    return out


def safety_report(kin, positions, results, dt: float = DT, profile=None):
    """Kiểm tra quỹ đạo trước khi cho chạy. Trả về (ok, các_dòng_báo_cáo).

    Kiểm cả 4 thứ chứ không chỉ 'IK hội tụ': hội tụ mà nằm sát giới hạn khớp
    hoặc sát singularity thì tay thật vẫn có thể giật/kẹt."""
    P = np.array(positions)
    qs = np.array([r.q for r in results])
    lo, hi = kin.model.lowerPositionLimit, kin.model.upperPositionLimit

    n_bad = sum(not r.converged for r in results)
    err_mm = max(r.position_error_m for r in results) * 1000
    conds = np.array([np.linalg.cond(kin.jacobian(q)[:3, :]) for q in qs])
    margin = float(min((qs - lo).min(), (hi - qs).min()))
    # Kiểm đúng profile sẽ gửi; mặc định là vòng quét minimum-jerk.
    # Lấy mẫu dày 100 lần mỗi khoảng waypoint để không bỏ sót đỉnh qdot.
    if profile is None:
        profile = SmoothJointProfile(qs, dt)
    check_t = np.linspace(0.0, profile.duration, len(qs) * 100 + 1)
    _, qd_check, qdd_check = profile.at(check_t)
    speeds = np.abs(qd_check).max(axis=0)
    accels = np.abs(qdd_check).max(axis=0)
    vel_limit = np.asarray(kin.model.velocityLimit, dtype=float)
    allowed = MAX_JOINT_SPEED_FRACTION * vel_limit
    used_frac = speeds / vel_limit
    clearances = body_clearance(P)

    lines = [
        f"Kích thước: rộng {(P[:,0].max()-P[:,0].min())*100:.0f}cm (trái-phải) x "
        f"cao {(P[:,2].max()-P[:,2].min())*100:.0f}cm x "
        f"sâu {(P[:,1].max()-P[:,1].min())*100:.0f}cm (ra-vào)",
        f"  x[{P[:,0].min():.3f}, {P[:,0].max():.3f}] "
        f"y[{P[:,1].min():.3f}, {P[:,1].max():.3f}] "
        f"z[{P[:,2].min():.3f}, {P[:,2].max():.3f}]  "
        f"(mặt ngồi z~0.145, mép trước ghế y=-0.215)",
        f"IK: {len(results)-n_bad}/{len(results)} điểm hội tụ, sai số lớn nhất {err_mm:.5f}mm",
        f"  cond(J) lớn nhất {conds.max():.1f} (ngưỡng {MAX_COND}) | "
        f"cách giới hạn khớp gần nhất {margin:.3f} rad (ngưỡng {MIN_JOINT_MARGIN_RAD})",
        f"  biên độ mỗi khớp (độ): {np.degrees(qs.max(axis=0)-qs.min(axis=0)).round(1)} "
        f"-> {kin.joint_names}",
        f"  tốc độ đỉnh profile (rad/s, duration={profile.duration:g}s): {speeds.round(3)} | "
        f"trần URDF: {vel_limit.round(3)} | "
        f"dùng {(used_frac*100).round(1)}% (cho phép "
        f"{MAX_JOINT_SPEED_FRACTION*100:.0f}%)",
        f"  gia tốc đỉnh profile (rad/s^2): {accels.round(3)}",
        f"  hở người ngồi + ghế (ngưỡng {MIN_BODY_CLEARANCE_M*100:.0f}cm): "
        + " | ".join(f"{k} {v*100:.1f}cm" for k, v in clearances.items()),
    ]

    fails = []
    if n_bad:
        fails.append(f"{n_bad} điểm IK không hội tụ")
    if err_mm > MAX_ERR_MM:
        fails.append(f"sai số vị trí {err_mm:.3f}mm > {MAX_ERR_MM}mm")
    if conds.max() > MAX_COND:
        fails.append(f"cond(J) {conds.max():.1f} > {MAX_COND} (quá gần singularity)")
    if margin < MIN_JOINT_MARGIN_RAD:
        fails.append(f"chỉ cách giới hạn khớp {margin:.3f} rad < {MIN_JOINT_MARGIN_RAD}")
    for name, c in clearances.items():
        if c < MIN_BODY_CLEARANCE_M:
            fails.append(
                f"đầu tay chỉ cách '{name}' {c*100:.1f}cm < "
                f"{MIN_BODY_CLEARANCE_M*100:.0f}cm"
            )
    over = np.where(speeds > allowed)[0]
    for i in over:
        fails.append(
            f"khớp {kin.joint_names[i]} chạy {speeds[i]:.3f} rad/s, vượt "
            f"{MAX_JOINT_SPEED_FRACTION*100:.0f}% trần {vel_limit[i]:.3f} rad/s "
            f"-- tăng dt hoặc giảm biên độ"
        )

    if fails:
        lines.append("KHÔNG ĐẠT: " + "; ".join(fails))
    return (not fails), lines
