"""
test_sweep_mujoco.py — xem BẰNG MẮT quỹ đạo quét rộng trước mặt người ngồi
trong MuJoCo (thay cho vòng tròn O bán kính 5cm ở test_draw_mujoco.py).

Tham số quỹ đạo nằm ở sweep_trajectory.py -- CÙNG file mà draw_trajectory.py
dùng để chạy tay thật, nên cái xem ở đây đúng là cái sẽ chạy trên tay thật.

Chạy: python3 test_sweep_mujoco.py     (đóng cửa sổ viewer để dừng)
"""

import time

import mujoco
import mujoco.viewer

from mujoco_env import MujocoEnv
from gim_arm_kinematics import GimArmKinematics
import sweep_trajectory as traj

URDF = "gim_arm.urdf"


def build_trajectory():
    """Trả về (kin, positions, results) và in báo cáo an toàn."""
    kin = GimArmKinematics(URDF, tool_offset_xyz=traj.TOOL_OFFSET)
    positions, results = traj.solve(kin)
    ok, lines = traj.safety_report(kin, positions, results)
    for line in lines:
        print(line)
    return kin, positions, results, ok


def main():
    kin, positions, results, ok = build_trajectory()
    if not ok:
        print("DỪNG: quỹ đạo chưa đạt ngưỡng an toàn -- không mở viewer.")
        return

    env = MujocoEnv(URDF)
    env.reset(qpos=results[0].q)

    print("\nĐang mở viewer... quan sát tay quét ellipse trước mặt (lặp liên tục).")
    print("Đóng cửa sổ viewer để dừng chương trình.")
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        idx = 0
        while viewer.is_running():
            step_start = time.time()
            env.set_qpos_direct(results[idx % len(results)].q)
            viewer.sync()
            idx += 1
            # ~25 điểm/giây: 90 điểm -> 1 vòng ~3.6s, đủ chậm để nhìn rõ.
            # Tay THẬT chạy chậm hơn nhiều (dt=0.3s/điểm -> 27s/vòng).
            elapsed = time.time() - step_start
            if elapsed < 1.0 / 25.0:
                time.sleep(1.0 / 25.0 - elapsed)


if __name__ == "__main__":
    main()
