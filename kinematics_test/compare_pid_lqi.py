"""
compare_pid_lqi.py — so sánh SÒNG PHẲNG bộ PID đang chạy với bộ LQI mới.

KHÔNG sửa và KHÔNG thay thế gì của bộ PID cũ. Bộ PID ở đây là bộ THẬT: import
thẳng lớp OdriveAxis trong gim6010_mujoco_sim.py (bản dựng lại cascade của
ODrive: lọc setpoint -> P vị trí -> PI vận tốc -> giới hạn mô-men), với đúng
hệ số mặc định mà driver đang dùng. Nhờ vậy "PID" trong bảng kết quả đúng là
cái đang chạy trên tay máy, không phải một bộ PID sách vở dựng lại cho có.

ĐIỀU KIỆN SO SÁNH GIỮ GIỐNG NHAU TUYỆT ĐỐI cho cả 2 bộ:
  - cùng đối tượng vật lý: lớp ArmPhysics của chính file sim đó (MuJoCo +
    armature + ma sát nhớt + ma sát khô), cùng bước thời gian 1/2000 s
  - cùng quỹ đạo tham chiếu (sweep_trajectory.py) và cùng cách nội suy
  - cùng giới hạn mô-men khớp 5/40/5 Nm
  - cùng tư thế xuất phát (đúng điểm đầu quỹ đạo)
  - đo trên VÒNG CUỐI, bỏ vòng đầu để loại quá độ khởi động

CHỖ CỐ Ý KHÁC NHAU -- và phải khác, vì đó là bản chất 2 kiến trúc:
  - PID cascade nhận lệnh VỊ TRÍ, chạy trong driver ở tần số cao (2 kHz trong
    mô phỏng, 8 kHz trên driver thật). Máy chủ chỉ gửi vị trí xuống 100 Hz.
  - LQI tính MÔ-MEN, chạy trên máy chủ ở 100 Hz (đúng update_rate của
    controller_manager trong controllers.yaml).
  Nói cách khác LQI bị thiệt 20 lần về tần số vòng lặp, nhưng được lợi vì biết
  mô hình động lực học. Chạy thêm cấu hình LQI ở 1 kHz để tách bạch xem phần
  hơn/kém đến từ MÔ HÌNH hay từ TẦN SỐ.

Chạy:  python3 compare_pid_lqi.py            (mặc định 2 vòng, đo vòng 2)
       python3 compare_pid_lqi.py --loops 3
       python3 compare_pid_lqi.py --bandwidth 12   (đổi độ cứng LQI)
"""

import argparse
import importlib.util
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

from arm_dynamics import ArmDynamics
from gim_arm_kinematics import GimArmKinematics
from lqi_controller import LqiController, LqiWeights
import sweep_trajectory as traj

URDF = "gim_arm.urdf"
SIM_REL_PATH = "../src/gim_arm_control/gim_control/gim6010_mujoco_sim.py"
CTRL_HZ_ROS = 100.0     # update_rate của controller_manager (controllers.yaml)


def load_hardware_sim():
    """Nạp gim6010_mujoco_sim.py theo ĐƯỜNG DẪN (nó nằm trong package ROS
    gim_control, không nằm trên sys.path của thư mục này). Import theo đường
    dẫn thay vì chép code sang đây để bộ PID dùng để so sánh luôn là bản gốc,
    không có nguy cơ hai bản trôi lệch nhau."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), SIM_REL_PATH)
    spec = importlib.util.spec_from_file_location("gim6010_mujoco_sim", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["gim6010_mujoco_sim"] = mod
    spec.loader.exec_module(mod)
    return mod


class Reference:
    """Quỹ đạo tham chiếu liên tục theo thời gian, tuần hoàn.

    90 điểm IK rời rạc cách nhau 0.3s được nội suy bằng spline bậc 3 TUẦN HOÀN
    (bc_type='periodic') -> có sẵn q_ref, q̇_ref, q̈_ref giải tích tại mọi thời
    điểm, và nối vòng liền mạch cả về vận tốc lẫn gia tốc ở chỗ khép kín.
    LQI cần q̈_ref cho phần feedforward; lấy sai phân hữu hạn trên lưới 0.3s sẽ
    ra gia tốc đầy răng cưa, nên buộc phải nội suy trơn."""

    def __init__(self, q_waypoints: np.ndarray, dt_waypoint: float):
        n = len(q_waypoints)
        self.period = n * dt_waypoint
        ts = np.arange(n + 1) * dt_waypoint
        qs = np.vstack([q_waypoints, q_waypoints[:1]])   # khép vòng
        self.spline = CubicSpline(ts, qs, axis=0, bc_type="periodic")

    def at(self, t: float):
        tt = t % self.period
        return (self.spline(tt), self.spline(tt, 1), self.spline(tt, 2))


def run_lqi(sim, urdf, ref, dyn, controller, duration, ctrl_hz):
    """Chạy LQI: tính mô-men trên máy chủ ở ctrl_hz, giữ nguyên (ZOH) giữa các
    chu kỳ, vật lý vẫn chạy ở 2 kHz."""
    axes = sim.parse_axes_from_urdf(urdf)
    physics = sim.ArmPhysics(urdf, axes)
    dt_phys = 1.0 / sim.CONTROL_HZ
    steps_per_ctrl = max(1, int(round(sim.CONTROL_HZ / ctrl_hz)))
    dt_ctrl = steps_per_ctrl * dt_phys

    q0, _, _ = ref.at(0.0)
    physics.reset(q0)
    controller.reset()

    n = int(duration / dt_phys)
    log = {k: np.zeros((n, 3)) for k in ("q", "qd", "q_ref", "tau")}
    log["t"] = np.zeros(n)
    tau = np.zeros(3)

    for k in range(n):
        t = k * dt_phys
        q = np.array([physics.joint_state(i)[0] for i in range(3)])
        qd = np.array([physics.joint_state(i)[1] for i in range(3)])
        q_ref, qd_ref, qdd_ref = ref.at(t)

        if k % steps_per_ctrl == 0:
            tau = controller.compute(q, qd, q_ref, qd_ref, qdd_ref, dt_ctrl)

        log["t"][k] = t
        log["q"][k], log["qd"][k], log["q_ref"][k], log["tau"][k] = q, qd, q_ref, tau
        physics.step(tau)
        if not physics.healthy():
            raise RuntimeError(f"Mô phỏng nổ (NaN) ở bước {k}, t={t:.3f}s")
    return log


def _q16(x, scale=1000.0):
    """Lượng tử hoá đúng như trường int16 của Set_Input_Pos: vel_ff và
    torque_ff đi trên bus dưới dạng int16 thang 0.001 (rev/s, Nm phía rotor).
    Không mô phỏng bước này thì feedforward trông đẹp hơn thực tế."""
    return float(np.clip(round(x * scale), -32768, 32767)) / scale


def run_cascade(sim, urdf, ref, duration, cmd_hz=CTRL_HZ_ROS, input_mode=1,
                vel_ff=False, torque_ff=None, dyn=None):
    """Chạy bộ PID cascade THẬT của ODrive: máy chủ gửi lệnh vị trí xuống
    cmd_hz (đúng như JTC), driver chạy vòng cascade mỗi bước vật lý.

    input_mode : 1 = passthrough, 3 = POS_FILTER (bộ lọc setpoint bậc 2)
    vel_ff     : có điền trường vel_ff của 0x00C bằng q̇_ref hay không
    torque_ff  : None | 'gravity' (G(q_ref)) | 'full' (nghịch động lực học đầy
                 đủ M q̈_ref + C q̇_ref + G) -- điền vào trường torque_ff
    Cả 2 trường này plugin hiện đang gửi 0 (xem gim_arm_system.cpp)."""
    axes = sim.parse_axes_from_urdf(urdf)
    physics = sim.ArmPhysics(urdf, axes)
    dt_phys = 1.0 / sim.CONTROL_HZ
    steps_per_cmd = max(1, int(round(sim.CONTROL_HZ / cmd_hz)))

    q0, _, _ = ref.at(0.0)
    physics.reset(q0)
    drivers = []
    for i, cfg in enumerate(axes):
        drv = sim.OdriveAxis(cfg)
        # Đúng như gim_arm_system.cpp on_activate(): control_mode=3 (vị trí),
        # input_mode=1 (passthrough). Lưu ý comment trong file C++ ghi
        # "input_mode=3" nhưng code truyền 1 -- ở đây theo CODE, tức theo cái
        # thật sự chạy trên phần cứng.
        drv.control_mode = 3
        drv.input_mode = input_mode
        drv.arm(q0[i])
        drivers.append(drv)

    n = int(duration / dt_phys)
    log = {k: np.zeros((n, 3)) for k in ("q", "qd", "q_ref", "tau")}
    log["t"] = np.zeros(n)

    for k in range(n):
        t = k * dt_phys
        q = np.array([physics.joint_state(i)[0] for i in range(3)])
        qd = np.array([physics.joint_state(i)[1] for i in range(3)])
        q_ref, qd_ref, qdd_ref = ref.at(t)

        if k % steps_per_cmd == 0:
            if torque_ff == "gravity":
                tau_ff = dyn.gravity(q_ref)
            elif torque_ff == "full":
                tau_ff = dyn.inverse_dynamics(q_ref, qd_ref, qdd_ref)
            else:
                tau_ff = np.zeros(3)
            for i, drv in enumerate(drivers):
                cfg = drv.cfg
                drv.input_pos = drv.joint_rad_to_rotor_rev(q_ref[i])
                # rad/s ở khớp -> rev/s ở rotor, cùng phép đổi như vị trí
                drv.input_vel = _q16(
                    qd_ref[i] * cfg.direction / (2.0 * np.pi) * cfg.gear_ratio
                ) if vel_ff else 0.0
                # Nm ở khớp -> Nm ở rotor: chia gear_ratio và direction
                drv.input_torque = _q16(
                    tau_ff[i] / (cfg.gear_ratio * cfg.direction)
                ) if torque_ff else 0.0

        tau = np.array([drv.update(q[i], qd[i], dt_phys) for i, drv in enumerate(drivers)])
        log["t"][k] = t
        log["q"][k], log["qd"][k], log["q_ref"][k], log["tau"][k] = q, qd, q_ref, tau
        physics.step(tau)
        if not physics.healthy():
            raise RuntimeError(f"Mô phỏng nổ (NaN) ở bước {k}, t={t:.3f}s")
    return log


def metrics(log, kin, period, tau_max):
    """Chỉ số bám, tính TRÊN VÒNG CUỐI (bỏ quá độ khởi động)."""
    t = log["t"]
    mask = t >= (t[-1] - period)
    e = log["q"][mask] - log["q_ref"][mask]

    ee_err = np.array([
        np.linalg.norm(kin.fk_position(q) - kin.fk_position(qr))
        for q, qr in zip(log["q"][mask], log["q_ref"][mask])
    ])
    tau = log["tau"][mask]
    return dict(
        joint_rms_deg=np.degrees(np.sqrt((e ** 2).mean(axis=0))),
        joint_max_deg=np.degrees(np.abs(e).max(axis=0)),
        ee_rms_mm=np.sqrt((ee_err ** 2).mean()) * 1000,
        ee_max_mm=ee_err.max() * 1000,
        tau_rms=np.sqrt((tau ** 2).mean(axis=0)),
        tau_peak=np.abs(tau).max(axis=0),
        sat_percent=100.0 * (np.abs(tau) >= 0.999 * tau_max).mean(axis=0),
    )


def print_table(results, joint_names):
    w = 26
    names = list(results.keys())
    print("\n" + "=" * (w + 18 * len(names)))
    print("KẾT QUẢ SO SÁNH (đo trên vòng cuối)".center(w + 18 * len(names)))
    print("=" * (w + 18 * len(names)))
    header = f"{'chỉ số':<{w}}" + "".join(f"{n:>18}" for n in names)
    print(header)
    print("-" * len(header))

    def row(label, fn, fmt="{:>18.4f}"):
        print(f"{label:<{w}}" + "".join(fmt.format(fn(results[n])) for n in names))

    print("SAI SỐ BÁM ĐẦU TAY (quan trọng nhất)")
    row("  RMS (mm)", lambda m: m["ee_rms_mm"])
    row("  lớn nhất (mm)", lambda m: m["ee_max_mm"])
    print("SAI SỐ BÁM TỪNG KHỚP -- RMS (độ)")
    for i, jn in enumerate(joint_names):
        row(f"  {jn}", lambda m, i=i: m["joint_rms_deg"][i])
    print("SAI SỐ BÁM TỪNG KHỚP -- lớn nhất (độ)")
    for i, jn in enumerate(joint_names):
        row(f"  {jn}", lambda m, i=i: m["joint_max_deg"][i])
    print("MÔ-MEN RMS (Nm)")
    for i, jn in enumerate(joint_names):
        row(f"  {jn}", lambda m, i=i: m["tau_rms"][i])
    print("MÔ-MEN ĐỈNH (Nm)")
    for i, jn in enumerate(joint_names):
        row(f"  {jn}", lambda m, i=i: m["tau_peak"][i])
    print("THỜI GIAN BÃO HOÀ MÔ-MEN (%)")
    for i, jn in enumerate(joint_names):
        row(f"  {jn}", lambda m, i=i: m["sat_percent"][i])
    print("=" * len(header))


def make_plots(logs, kin, period, joint_names, out_png):
    colors = {"PID cascade (đang dùng)": "tab:orange",
              "LQI 100Hz": "tab:blue", "LQI 1kHz": "tab:green"}
    fig, axes = plt.subplots(4, 1, figsize=(12, 13), sharex=True)

    first = next(iter(logs.values()))
    t_end = first["t"][-1]
    mask = first["t"] >= (t_end - period)
    t_plot = first["t"][mask] - (t_end - period)

    for i in range(3):
        for name, log in logs.items():
            m = log["t"] >= (log["t"][-1] - period)
            err = np.degrees(log["q"][m][:, i] - log["q_ref"][m][:, i])
            axes[i].plot(t_plot, err, label=name,
                         color=colors.get(name), linewidth=1.2)
        axes[i].axhline(0, color="k", linewidth=0.6, alpha=0.4)
        axes[i].set_ylabel(f"sai số {joint_names[i]}\n(độ)")
        axes[i].grid(alpha=0.3)
        axes[i].legend(loc="upper right", fontsize=8)

    for name, log in logs.items():
        m = log["t"] >= (log["t"][-1] - period)
        ee = np.array([np.linalg.norm(kin.fk_position(q) - kin.fk_position(qr)) * 1000
                       for q, qr in zip(log["q"][m], log["q_ref"][m])])
        axes[3].plot(t_plot, ee, label=name, color=colors.get(name), linewidth=1.2)
    axes[3].set_ylabel("sai số đầu tay\n(mm)")
    axes[3].set_xlabel("thời gian trong 1 vòng (s)")
    axes[3].grid(alpha=0.3)
    axes[3].legend(loc="upper right", fontsize=8)

    fig.suptitle("PID cascade (ODrive) vs LQI computed-torque -- cùng quỹ đạo, cùng vật lý")
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    print(f"\nĐã lưu đồ thị: {out_png}")

    # Đồ thị mô-men riêng
    fig2, ax2 = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for i in range(3):
        for name, log in logs.items():
            m = log["t"] >= (log["t"][-1] - period)
            ax2[i].plot(t_plot, log["tau"][m][:, i], label=name,
                        color=colors.get(name), linewidth=1.0)
        ax2[i].set_ylabel(f"τ {joint_names[i]}\n(Nm)")
        ax2[i].grid(alpha=0.3)
        ax2[i].legend(loc="upper right", fontsize=8)
    ax2[-1].set_xlabel("thời gian trong 1 vòng (s)")
    fig2.suptitle("Mô-men khớp")
    fig2.tight_layout()
    out2 = out_png.replace(".png", "_torque.png")
    fig2.savefig(out2, dpi=110)
    print(f"Đã lưu đồ thị: {out2}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--loops", type=int, default=2, help="số vòng chạy (đo vòng cuối)")
    ap.add_argument("--bandwidth", type=float, default=None,
                    help="dùng gán cực theo băng thông (rad/s) thay cho trọng số LQR")
    ap.add_argument("--friction-ff", action="store_true",
                    help="BẬT bù ma sát của LQI (mặc định tắt -- đo được là bật "
                         "vào thì xấu hơn và gây rung, xem lqi_controller.py)")
    ap.add_argument("--out", default="compare_pid_lqi.png")
    args = ap.parse_args()

    print("Đang dựng quỹ đạo tham chiếu (IK 2 lượt trên 90 điểm)...")
    kin = GimArmKinematics(URDF, tool_offset_xyz=traj.TOOL_OFFSET)
    positions, ik = traj.solve(kin)
    ok, lines = traj.safety_report(kin, positions, ik)
    if not ok:
        print("\n".join(lines))
        print("DỪNG: quỹ đạo không đạt ngưỡng an toàn.")
        return 1
    ref = Reference(np.array([r.q for r in ik]), traj.DT)
    print(f"  chu kỳ 1 vòng = {ref.period:.1f}s, chạy {args.loops} vòng")

    sim = load_hardware_sim()
    dyn = ArmDynamics(URDF)
    duration = args.loops * ref.period

    ctrl = LqiController(dyn, weights=LqiWeights(), bandwidth=args.bandwidth,
                         friction_ff=args.friction_ff)
    print("\nBộ điều khiển LQI:")
    print(ctrl.describe(q_nominal=ref.at(0.0)[0]))
    print("  bù ma sát:", "BẬT" if args.friction_ff
          else "TẮT (mặc định) -- khâu tích phân tự gánh ma sát")

    logs = {}
    print("\nChạy PID cascade (bộ đang dùng)...")
    logs["PID cascade (đang dùng)"] = run_cascade(sim, URDF, ref, duration)
    print("Chạy LQI ở 100 Hz (đúng update_rate của ros2_control)...")
    logs["LQI 100Hz"] = run_lqi(sim, URDF, ref, dyn, ctrl, duration, CTRL_HZ_ROS)
    print("Chạy LQI ở 1 kHz (để tách phần hơn do MÔ HÌNH và phần do TẦN SỐ)...")
    logs["LQI 1kHz"] = run_lqi(sim, URDF, ref, dyn, ctrl, duration, 1000.0)

    results = {name: metrics(log, kin, ref.period, dyn.tau_max)
               for name, log in logs.items()}
    print_table(results, dyn.joint_names)

    base = results["PID cascade (đang dùng)"]["ee_rms_mm"]
    print("\nMức cải thiện sai số đầu tay so với PID cascade:")
    for name, m in results.items():
        if name.startswith("PID"):
            continue
        print(f"  {name:<12}: {base:.3f}mm -> {m['ee_rms_mm']:.3f}mm  "
              f"(giảm {100*(1-m['ee_rms_mm']/base):.1f}%, tốt hơn {base/m['ee_rms_mm']:.1f} lần)")

    make_plots(logs, kin, ref.period, dyn.joint_names, args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
