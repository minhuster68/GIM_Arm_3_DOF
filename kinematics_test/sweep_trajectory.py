"""
sweep_trajectory.py — ĐỊNH NGHĨA DUY NHẤT của quỹ đạo quét trước mặt người
đeo, dùng chung cho cả mô phỏng MuJoCo (test_sweep_mujoco.py) lẫn robot thật
(draw_trajectory.py / origin_draw_trajectory.py).

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

from shapes import shoulder_sweep, discretize

# Tâm quay vai trong hệ world, đọc thẳng từ mô hình (data.xanchor của
# base_joint trong MuJoCo), không ước lượng bằng mắt.
SHOULDER_PIVOT = (0.031, -0.538, 0.606)

TOOL_OFFSET = (0.4031, 0.049, -0.029)

# Tham số dưới đây là kết quả quét tìm kiếm có ràng buộc, không phải số ước
# lượng: tối đa hoá biên độ với điều kiện cond(J) < 10.5, cách giới hạn khớp
# >= 0.1 rad, và đầu tay hở >= 8cm so với hộp bao ghế + người ngồi.
# Kết quả: 42cm rộng x 27cm cao x 25cm sâu, tay vươn 0.56m (gần hết tầm với
# 0.60m -> dáng tay duỗi thẳng ra trước).
SWEEP = dict(
    pivot=SHOULDER_PIVOT,
    radius=0.56,        # độ vươn tay (m)
    az_center_deg=30.0,  # tâm quét ngang, 0 = thẳng trước mặt, + = sang phải
    el_center_deg=-8.0,  # tâm quét dọc, - = thấp hơn vai
    az_amp_deg=26.0,     # biên độ ngang -> quét az 4..56 độ
    el_amp_deg=14.0,     # biên độ dọc  -> quét el -22..+6 độ
)

N_POINTS = 90        # số điểm 1 vòng
DT = 0.3             # giây giữa 2 điểm -> 1 vòng ~27s
TRANSITION_TIME = 5.0  # giây để đi êm từ tư thế hiện tại về điểm đầu quỹ đạo

# Ngưỡng an toàn khi chạy trên tay thật
MAX_ERR_MM = 0.1
MAX_COND = 15.0
MIN_JOINT_MARGIN_RAD = 0.05

# Tốc độ khớp tối đa cho phép, tính theo TỈ LỆ so với <limit velocity> của
# từng khớp trong URDF (base/elbow 15.708 rad/s, shoulder chỉ 1.963 rad/s vì
# gear 64). So theo tỉ lệ chứ không theo 1 con số chung, vì 3 khớp có trần
# phần cứng lệch nhau tới 8 lần: một con số chung sẽ vừa quá lỏng cho
# base/elbow vừa quá chặt cho shoulder.
MAX_JOINT_SPEED_FRACTION = 0.25


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


def safety_report(kin, positions, results, dt: float = DT):
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
    # tốc độ TỪNG khớp: chênh lệch góc lớn nhất giữa 2 điểm liền kề / dt
    speeds = np.abs(np.diff(np.vstack([qs, qs[:1]]), axis=0)).max(axis=0) / dt
    vel_limit = np.asarray(kin.model.velocityLimit, dtype=float)
    allowed = MAX_JOINT_SPEED_FRACTION * vel_limit
    used_frac = speeds / vel_limit

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
        f"  tốc độ khớp (rad/s, dt={dt}s): {speeds.round(3)} | "
        f"trần URDF: {vel_limit.round(3)} | "
        f"dùng {(used_frac*100).round(1)}% (cho phép "
        f"{MAX_JOINT_SPEED_FRACTION*100:.0f}%)",
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
