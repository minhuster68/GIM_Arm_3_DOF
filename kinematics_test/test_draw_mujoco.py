"""
test_draw_mujoco.py — task #10: xem BẰNG MẮT tay máy vẽ chữ O trong MuJoCo,
không chỉ tin vào số liệu "không NaN" như Test 3 của mujoco_env.py.

Cách chạy: đặt cùng thư mục với mujoco_env.py, gim_arm_kinematics.py,
shapes.py, gim_arm.urdf, meshes/, rồi chạy: python3 test_draw_mujoco.py
Cửa sổ viewer sẽ mở, tay máy lặp lại vẽ chữ O liên tục. Đóng cửa sổ để dừng.
"""

import time

import mujoco
import mujoco.viewer
import numpy as np

from mujoco_env import MujocoEnv
from shapes import letter_o, discretize
from gim_arm_kinematics import GimArmKinematics


def main():
    kin = GimArmKinematics("gim_arm.urdf", tool_offset_xyz=(0.4031, 0.049, -0.029))
    path_o = letter_o(center=(-0.2486, 0.2), radius=0.05, plane="x", plane_value=0.1)
    positions = discretize(path_o, n_points=60, close_loop=True)
    results = kin.solve_trajectory(positions)

    n_bad = sum(not r.converged for r in results)
    if n_bad > 0:
        print(f"CẢNH BÁO: {n_bad}/{len(results)} điểm không hội tụ -- kiểm tra lại trước khi xem.")
        return
    print(f"IK hội tụ đủ {len(results)}/{len(results)} điểm, sai số lớn nhất "
          f"{max(r.position_error_m for r in results)*1000:.5f}mm -- an toàn để xem.")

    env = MujocoEnv("gim_arm.urdf")
    env.reset(qpos=results[0].q)

    print("Đang mở viewer... quan sát tay máy vẽ chữ O (lặp lại liên tục).")
    print("Đóng cửa sổ viewer để dừng chương trình.")
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        idx = 0
        while viewer.is_running():
            step_start = time.time()

            q = results[idx % len(results)].q
            env.set_qpos_direct(q)  # phát lại vị trí thuần, chưa cần động lực học
            viewer.sync()

            idx += 1
            # ~15 điểm/giây -- đủ chậm để nhìn rõ hình dạng, không giật cục
            elapsed = time.time() - step_start
            if elapsed < 1.0 / 15.0:
                time.sleep(1.0 / 15.0 - elapsed)


if __name__ == "__main__":
    main()