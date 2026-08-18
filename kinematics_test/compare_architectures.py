"""
compare_architectures.py — so sánh 6 KIẾN TRÚC ĐIỀU KHIỂN trên cùng quỹ đạo,
cùng vật lý, cùng giới hạn mô-men. Đây là bảng để CHỌN cách đưa LQI lên tay
thật, không phải chỉ để xem LQI hơn PID bao nhiêu.

Mỗi dòng khác nhau ở 3 thứ: dùng lệnh CAN nào, vòng phản hồi chạy Ở ĐÂU, và
nếu máy chủ chết thì tay máy làm gì.

Chạy: python3 compare_architectures.py
"""
import numpy as np

from arm_dynamics import ArmDynamics
from gim_arm_kinematics import GimArmKinematics
from lqi_controller import LqiController, LqiWeights
from mit_mode import run_mit
import sweep_trajectory as traj
from compare_pid_lqi import (Reference, load_hardware_sim, run_cascade, run_lqi,
                             metrics, URDF, CTRL_HZ_ROS)

kin = GimArmKinematics(URDF, tool_offset_xyz=traj.TOOL_OFFSET)
_, ik = traj.solve(kin)
ref = Reference(np.array([r.q for r in ik]), traj.DT)
sim, dyn = load_hardware_sim(), ArmDynamics(URDF)
D = 2 * ref.period

rows = []
for label, note, kw in [
    ("vị trí, input_mode=1", "ĐANG CHẠY", dict(input_mode=1)),
    ("vị trí, input_mode=3 (POS_FILTER)", "bộ lọc setpoint bw=2", dict(input_mode=3)),
    ("vị trí + vel_ff", "1 dòng C++", dict(input_mode=1, vel_ff=True)),
    ("vị trí + vel_ff + torque_ff=G(q)", "+ Lagrange", dict(input_mode=1, vel_ff=True,
                                                            torque_ff="gravity")),
]:
    m = metrics(run_cascade(sim, URDF, ref, D, dyn=dyn, **kw), kin, ref.period, dyn.tau_max)
    rows.append((label, note, m))

log, _, _ = run_mit(sim, URDF, ref, dyn, D, CTRL_HZ_ROS)
rows.append(("MIT mode 0x008 + LQI", "lượng tử 12 bit",
             metrics(log, kin, ref.period, dyn.tau_max)))

ctrl = LqiController(dyn, weights=LqiWeights())
rows.append(("mô-men thuần 0x00E + LQI", "float32, KHÔNG lưới an toàn",
             metrics(run_lqi(sim, URDF, ref, dyn, ctrl, D, CTRL_HZ_ROS),
                     kin, ref.period, dyn.tau_max)))

base = rows[0][2]["ee_rms_mm"]
print(f"\n{'kiến trúc':<36} {'ghi chú':<28} {'EE RMS':>9} {'EE max':>8} {'so với nay':>11}")
print("-" * 96)
for label, note, m in rows:
    print(f"{label:<36} {note:<28} {m['ee_rms_mm']:8.3f}mm {m['ee_max_mm']:7.3f}mm "
          f"{base/m['ee_rms_mm']:9.1f}x")
