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

from gim_control.shapes import shoulder_sweep, discretize

# Tâm quay vai trong hệ world, đọc thẳng từ mô hình (data.xanchor của
# base_joint trong MuJoCo), không ước lượng bằng mắt.
SHOULDER_PIVOT = (0.031, -0.538, 0.606)

TOOL_OFFSET = (0.4031, 0.049, -0.029)

# ĐO LẠI GIỚI HẠN KHỚP 19/08/2026 -> QUỸ ĐẠO CŨ KHÔNG CÒN DÙNG ĐƯỢC.
# Giới hạn thật đo bằng encoder (xem <limit> trong gim_arm.urdf) chặt hơn số
# cũ rất nhiều ở base_joint: trần 1.0226 rad thay vì 1.57. Bộ tham số cũ
# (radius 0.56, az_center 30, az_amp 26) đẩy base_joint lên đúng 1.0226 --
# CHẠM TRẦN, margin 0.000 rad, 8/90 điểm IK không hội tụ, sai số 9.9mm.
# Đừng khôi phục lại bộ số đó.
#
# Bộ tham số dưới đây là kết quả quét tìm kiếm có ràng buộc trên giới hạn MỚI,
# không phải số ước lượng. Ràng buộc:
#   - IK hội tụ toàn bộ, sai số <= 0.1mm
#   - cond(J) < 10.5 (tránh singularity)
#   - cách giới hạn khớp >= 0.10 rad
#   - đầu tay hở >= 8cm so với hộp bao người ngồi + ghế (BODY_BOXES bên dưới)
#   - tầm vận động vai nằm trong vùng KHÔNG ĐAU cho người đeo:
#       el <= +10 độ  -> không nâng tay cao hơn vai (tránh chạm mỏm cùng vai)
#       el >= -28 độ  -> hạ xuống vừa phải, không thúc vào đùi
#       az  -30..+46 độ -> khép/dang ngang trong tầm sinh lý
# Bản quét đầu tiên cho ra 68cm rộng -- xem trong MuJoCo thấy TO QUÁ so với nhu
# cầu thật, nên đã thu biên độ góc còn 80% (az_amp 38->30.4, el_amp 17->13.6).
# radius_amp GIỮ NGUYÊN 0.04 chứ không thu theo: nó không làm quỹ đạo trông to
# hơn (chiều sâu 8.4cm so với 8.2cm nếu thu) nhưng giữ được biên độ khuỷu 36.8
# độ thay vì tụt còn 30 độ. Thu cả 3 thì mất biên độ khuỷu mà chẳng nhỏ thêm.
# Kết quả: 56cm rộng x 22cm cao x 8cm sâu, cond 8.4, margin 0.218 rad,
# hở đùi/ghế 14.1cm. Biên độ khớp: base 64.6 / shoulder 37.4 / elbow 36.8 độ
# (bộ cũ chỉ 52.8 / 19.7 / 12.3 -- vẫn rộng hơn ở CẢ BA khớp dù đã thu 20%).
SWEEP = dict(
    pivot=SHOULDER_PIVOT,
    radius=0.52,         # độ vươn tay (m) -- lùi từ 0.56 về giữa vỏ cầu tầm
                         # với (0.41..0.59) nên còn dư địa cả 2 phía, đó là chỗ
                         # cond(J) tốt nhất và cũng là chỗ khuỷu duỗi thoải mái
    az_center_deg=8.0,   # tâm quét ngang, 0 = thẳng trước mặt, + = sang phải.
                         # Kéo từ 30 về 8: giới hạn base_joint mới cắt mất phía
                         # phải, dư địa còn lại nằm ở phía trong (az âm)
    el_center_deg=-8.0,  # tâm quét dọc, - = thấp hơn vai
    az_amp_deg=30.4,     # biên độ ngang -> quét az -22..+38 độ
    el_amp_deg=13.6,     # biên độ dọc  -> quét el -22..+6 độ
    radius_amp=0.04,     # "thở" độ vươn +-4cm -> KHUỶU cũng có biên độ thật
                         # (37.5 độ thay vì 12.3). Không có nó thì khuỷu gần
                         # như đứng yên, tập vai xong khuỷu vẫn cứng.
)

N_POINTS = 90        # số điểm 1 vòng
DT = 0.3             # giây giữa 2 điểm -> 1 vòng ~27s
TRANSITION_TIME = 5.0  # giây để đi êm từ tư thế hiện tại về điểm đầu quỹ đạo

# Ngưỡng an toàn khi chạy trên tay thật
MAX_ERR_MM = 0.1
MAX_COND = 15.0
MIN_JOINT_MARGIN_RAD = 0.05

# Hộp bao NGƯỜI NGỒI + GHẾ trong hệ world (m). TRƯỚC ĐÂY chỉ tồn tại dưới dạng
# comment "đầu tay hở >= 8cm so với hộp bao ghế + người ngồi" -- tức là KHÔNG hề
# được kiểm, ai sửa tham số SWEEP cũng không có gì chặn lại. Đưa thành số thật ở
# đây để safety_report() kiểm được, vì đây là ràng buộc duy nhất mà IK/cond/
# giới hạn khớp đều không nhìn thấy: quỹ đạo hoàn toàn hợp lệ về mặt động học
# vẫn có thể đập thẳng vào mặt hoặc vào đùi người ngồi.
#
# Dựng từ 3 mốc đã có sẵn trong repo, không phải đo người thật:
#   vai (0.031, -0.538, 0.606) | mặt ngồi z = 0.145 | mép trước ghế y = -0.215
# Người ngồi quay mặt theo +Y. NẾU ĐỔI GHẾ hoặc đổi vóc người thì sửa lại đây
# TRƯỚC khi chạy, đừng nới ngưỡng MIN_BODY_CLEARANCE_M.
BODY_BOXES = {
    # thân + đầu: mặt trước ngực ~10cm trước tâm khớp vai
    "thân/đầu": dict(x=(-0.28, 0.34), y=(-0.85, -0.42), z=(0.28, 0.95)),
    # đùi nằm trên mặt ghế, từ hông ra tới đầu gối (~mép trước ghế)
    "đùi/ghế": dict(x=(-0.32, 0.38), y=(-0.85, -0.19), z=(0.10, 0.32)),
    # cẳng chân buông thẳng xuống từ đầu gối
    "cẳng chân": dict(x=(-0.32, 0.38), y=(-0.26, -0.12), z=(0.00, 0.30)),
}
MIN_BODY_CLEARANCE_M = 0.08

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
        f"  tốc độ khớp (rad/s, dt={dt}s): {speeds.round(3)} | "
        f"trần URDF: {vel_limit.round(3)} | "
        f"dùng {(used_frac*100).round(1)}% (cho phép "
        f"{MAX_JOINT_SPEED_FRACTION*100:.0f}%)",
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
