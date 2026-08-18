"""
test_failsafe.py — trả lời bằng mô phỏng: CHUYỆN GÌ XẢY RA KHI MÁY CHỦ CHẾT
giữa lúc tay đang chạy, ở chế độ vị trí so với chế độ mô-men.

Đây không phải câu hỏi lý thuyết. Vòng điều khiển LQI chạy trên PC (node ROS,
100 Hz). PC treo / node crash / dây CAN rớt / vòng lặp trễ -- đều là chuyện
xảy ra thật. Câu hỏi là tay máy ĐANG ĐEO TRÊN TAY NGƯỜI sẽ làm gì lúc đó.

Bốn kịch bản, cùng một thời điểm hỏng t = 5s:
  A. Chế độ VỊ TRÍ, máy chủ ngừng gửi Set_Input_Pos -> driver vẫn giữ vòng vị
     trí của nó ở 8 kHz với setpoint cuối cùng.
  B. Chế độ MÔ-MEN, máy chủ ngừng gửi Set_Input_Torque, KHÔNG bật watchdog ->
     driver giữ nguyên mô-men cuối cùng.
  C. Chế độ MÔ-MEN, watchdog của driver hết giờ -> axis về IDLE, mô-men = 0.
  D. MIT mode (0x008), driver giữ kp/kd/p_des của frame cuối -> vẫn còn một
     vòng PD chạy trong driver ở 8 kHz.
"""
import numpy as np
from arm_dynamics import ArmDynamics
from gim_arm_kinematics import GimArmKinematics
from lqi_controller import LqiController, LqiWeights
import sweep_trajectory as traj
from mit_mode import run_mit
from compare_pid_lqi import Reference, load_hardware_sim, URDF

T_FAIL, T_AFTER = 5.0, 3.0

kin = GimArmKinematics(URDF, tool_offset_xyz=traj.TOOL_OFFSET)
_, ik = traj.solve(kin)
ref = Reference(np.array([r.q for r in ik]), traj.DT)
sim, dyn = load_hardware_sim(), ArmDynamics(URDF)
axes = sim.parse_axes_from_urdf(URDF)
dt = 1.0 / sim.CONTROL_HZ
n_fail, n_total = int(T_FAIL / dt), int((T_FAIL + T_AFTER) / dt)


def run(mode):
    physics = sim.ArmPhysics(URDF, axes)
    q0 = ref.at(0.0)[0]
    physics.reset(q0)
    drivers = [sim.OdriveAxis(c) for c in axes]
    for i, d in enumerate(drivers):
        d.control_mode, d.input_mode = 3, 1
        d.arm(q0[i])
    ctrl = LqiController(dyn, weights=LqiWeights())
    tau = np.zeros(3)
    q_at_fail = None
    for k in range(n_total):
        t = k * dt
        q = np.array([physics.joint_state(i)[0] for i in range(3)])
        qd = np.array([physics.joint_state(i)[1] for i in range(3)])
        alive = k < n_fail
        if k == n_fail:
            q_at_fail = q.copy()
        if mode == "position":
            if alive and k % 20 == 0:
                for i, d in enumerate(drivers):
                    d.input_pos = d.joint_rad_to_rotor_rev(ref.at(t)[0][i])
            tau = np.array([d.update(q[i], qd[i], dt) for i, d in enumerate(drivers)])
        elif mode == "torque_hold":
            if alive and k % 20 == 0:
                tau = ctrl.compute(q, qd, *ref.at(t), 0.01)
        elif mode == "torque_watchdog":
            if alive:
                if k % 20 == 0:
                    tau = ctrl.compute(q, qd, *ref.at(t), 0.01)
            else:
                tau = np.zeros(3)
        physics.step(tau)
    q_end = np.array([physics.joint_state(i)[0] for i in range(3)])
    return q_at_fail, q_end


print(f"Hỏng tại t={T_FAIL}s, đo độ trôi sau {T_AFTER}s\n")
print(f"{'kịch bản':<46} {'trôi từng khớp (độ)':>26} {'đầu tay trôi':>14}")
print("-" * 90)
scenarios = [("position", "A. VỊ TRÍ, máy chủ ngừng gửi lệnh"),
             ("torque_hold", "B. MÔ-MEN, giữ mô-men cuối (không watchdog)"),
             ("torque_watchdog", "C. MÔ-MEN, watchdog -> IDLE, mô-men = 0"),
             ("mit", "D. MIT MODE, driver giữ kp/kd/p_des cuối")]
for mode, label in scenarios:
    if mode == "mit":
        _, q_f, q_e = run_mit(sim, URDF, ref, dyn, T_FAIL + T_AFTER, fail_at=T_FAIL)
    else:
        q_f, q_e = run(mode)
    drift_deg = np.degrees(q_e - q_f)
    ee = np.linalg.norm(kin.fk_position(q_e) - kin.fk_position(q_f)) * 100
    print(f"{label:<46} {np.array2string(drift_deg, precision=1, floatmode='fixed'):>26} "
          f"{ee:11.1f} cm")
