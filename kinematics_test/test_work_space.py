"""
plot_workspace_cloud.py — vẽ đám mây điểm 3D của không gian thao tác đầu bút
(end-effector), quét ngẫu nhiên không gian góc khớp, tô màu theo điều kiện
Jacobian (xanh = tốt, đỏ = gần singular).

Cách dùng: đặt cùng thư mục với gim_arm_kinematics.py + gim_arm.urdf, chạy:
    python3 plot_workspace_cloud.py
Cửa sổ 3D mở lên, dùng chuột kéo để xoay xem từ mọi góc.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 -- cần import để bật chế độ 3D

from gim_arm_kinematics import GimArmKinematics


def sample_workspace(kin, n_samples=6000, cond_threshold=15.0, seed=0):
    """Quét ngẫu nhiên n_samples cấu hình khớp, trả về (points, conds) --
    vị trí (x,y,z) và condition number tương ứng của từng điểm."""
    rng = np.random.default_rng(seed)
    points = []
    conds = []
    for _ in range(n_samples):
        q = rng.uniform(kin.model.lowerPositionLimit, kin.model.upperPositionLimit)
        pos = kin.fk_position(q)
        J = kin.jacobian(q)[:3, :]
        cond = np.linalg.cond(J)
        points.append(pos)
        conds.append(min(cond, 100))  # cắt trần để không làm hỏng thang màu vì vài điểm ~vô cực
    return np.array(points), np.array(conds)


if __name__ == "__main__":
    kin = GimArmKinematics("gim_arm.urdf", tool_offset_xyz=(0.4031, 0.049, -0.029))

    print("Đang quét không gian làm việc (có thể mất vài giây)...")
    points, conds = sample_workspace(kin, n_samples=6000)

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")

    scatter = ax.scatter(
        points[:, 0], points[:, 1], points[:, 2],
        c=conds, cmap="RdYlGn_r", s=4, alpha=0.6, vmin=1, vmax=30,
    )
    cbar = fig.colorbar(scatter, ax=ax, shrink=0.6, pad=0.1)
    cbar.set_label("Condition number (thấp = tốt, cao = gần singular)")

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title(f"Đám mây điểm không gian thao tác đầu bút ({len(points)} điểm)")

    n_good = np.sum(conds < 15)
    print(f"Số điểm tốt (cond<15): {n_good}/{len(points)} ({100*n_good/len(points):.1f}%)")
    print("Kéo chuột để xoay, cuộn để zoom. Đóng cửa sổ để thoát.")

    plt.tight_layout()
    plt.show()