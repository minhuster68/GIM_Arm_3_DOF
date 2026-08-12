"""
scan_workspace.py — quét không gian làm việc, tìm vùng đủ tốt (không gần
singularity) để chọn đặt bảng vẽ (task #4 trong list).

Cách dùng: đặt cùng thư mục với gim_arm_kinematics.py + gim_arm.urdf, chạy:
    python3 scan_workspace.py
"""

import numpy as np
from gim_arm_kinematics import GimArmKinematics


def scan_plane(kin, axis: str, value: float, thickness=0.02, n_samples=20000, cond_threshold=15.0):
    """Quét workspace, giữ lại điểm nằm trong 1 lát mỏng quanh mặt phẳng
    vuông góc trục `axis` (một trong 'x','y','z') tại toạ độ `value`.
    Trả về mảng 2 cột (2 toạ độ còn lại) của các điểm đủ tốt."""
    axis_idx = {"x": 0, "y": 1, "z": 2}[axis]
    other_idx = [i for i in range(3) if i != axis_idx]

    np.random.seed(0)
    good_pts = []
    for _ in range(n_samples):
        q = np.random.uniform(kin.model.lowerPositionLimit, kin.model.upperPositionLimit)
        pos = kin.fk_position(q)
        if abs(pos[axis_idx] - value) > thickness:
            continue
        J = kin.jacobian(q)[:3, :]
        if np.linalg.cond(J) < cond_threshold:
            good_pts.append(pos[other_idx])
    return np.array(good_pts)


if __name__ == "__main__":
    kin = GimArmKinematics("gim_arm.urdf", tool_offset_xyz=(0.4031, 0.049, -0.029))

    # Thử 1 dải mặt phẳng x=const -- đổi "axis"/danh sách giá trị nếu muốn
    # dò mặt phẳng theo hướng khác (vd đặt bảng nằm ngang thì dò theo "z").
    print("=== Quét các mặt phẳng x=const ===")
    for x_try in [0.1, 0.2, 0.3, 0.4, 0.5]:
        pts = scan_plane(kin, "x", x_try)
        if len(pts) < 10:
            print(f"x={x_try}: quá ít điểm tốt ({len(pts)}) -- bỏ qua")
            continue
        y_range = pts[:, 0].max() - pts[:, 0].min()
        z_range = pts[:, 1].max() - pts[:, 1].min()
        print(
            f"x={x_try}: {len(pts)} điểm tốt | "
            f"y=[{pts[:,0].min():.3f}, {pts[:,0].max():.3f}] (rộng {y_range*100:.1f}cm) | "
            f"z=[{pts[:,1].min():.3f}, {pts[:,1].max():.3f}] (rộng {z_range*100:.1f}cm)"
        )

    print()
    print("LƯU Ý: đây là bounding box (biên ngoài), không đảm bảo đặc kín 100%")
    print("bên trong. Trước khi chốt vùng vẽ thật, nên kiểm tra thêm vài điểm")
    print("CỤ THỂ nằm rải rác trong vùng định chọn bằng kin.check_singularity(q)")
    print("để chắc chắn không có 'lỗ hổng' singular nằm giữa vùng.")