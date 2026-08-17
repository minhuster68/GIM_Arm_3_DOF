"""
gim_arm_kinematics.py — FK / Jacobian / IK cho GIM Arm 3DOF, dùng Pinocchio.

Cài đặt (chọn 1 trong 2 cách):
    pip install pin --break-system-packages      # tên package pip là "pin", KHÔNG phải "pinocchio"
    sudo apt install ros-humble-pinocchio        # nếu muốn qua apt, tích hợp sẵn ROS 2

Lưu ý quan trọng đã kiểm chứng bằng test thật (không chỉ suy đoán):
- Tay chỉ có 3 DOF -> không đủ bậc tự do để ép cả vị trí lẫn hướng end-effector
  cùng lúc (6 phương trình/3 ẩn, over-constrained). IK ở đây CHỈ giải vị trí
  (x,y,z) -- đúng bản chất bài toán vẽ hình, hướng bút "đi theo" tự nhiên.
- q=[0,0,0] là 1 điểm khởi đầu XẤU cho IK: base_joint có limit [0, 1.57] nên
  0 nằm ĐÚNG biên giới hạn, dễ khiến solver bị kẹt. Mặc định dùng điểm giữa
  dải góc mỗi khớp làm điểm khởi đầu, hoặc truyền q_init=nghiệm điểm trước
  (khi giải cả 1 quỹ đạo liên tiếp -- xem hàm solve_trajectory()).
"""

from dataclasses import dataclass

import numpy as np
import pinocchio as pin


@dataclass
class IKResult:
    q: np.ndarray
    converged: bool
    iterations: int
    position_error_m: float


class GimArmKinematics:
    def __init__(
        self,
        urdf_path: str,
        end_effector_frame: str = "lower_arm_link",
        tool_offset_xyz=(0.0, 0.0, 0.0),
    ):
        """
        tool_offset_xyz: độ lệch (m) từ gốc frame `end_effector_frame` (theo
        URDF, thường nằm ĐÚNG tại điểm nối của elbow_joint) tới điểm bút thật.

        QUAN TRỌNG -- đã phát hiện bằng test thật: nếu để tool_offset_xyz mặc
        định (0,0,0), điểm tính FK/IK trùng đúng trục quay của elbow_joint,
        khiến elbow KHÔNG hề làm điểm đó di chuyển (Jacobian vị trí mất hạng,
        rank 2/3 ở MỌI cấu hình, không giải IK được). Bắt buộc phải đo và
        điền đúng khoảng cách thật từ điểm nối elbow tới đầu bút trước khi
        dùng để sinh quỹ đạo vẽ hình.
        """
        self.model = pin.buildModelFromUrdf(urdf_path)
        parent_frame_id = self.model.getFrameId(end_effector_frame)
        if parent_frame_id >= len(self.model.frames):
            raise ValueError(
                f"Không tìm thấy frame '{end_effector_frame}' trong URDF. "
                f"Kiểm tra lại tên link end-effector."
            )
        parent_joint = self.model.frames[parent_frame_id].parentJoint
        offset = pin.SE3(np.eye(3), np.array(tool_offset_xyz, dtype=float))
        placement = self.model.frames[parent_frame_id].placement * offset
        tool_frame = pin.Frame("pen_tip", parent_joint, parent_frame_id, placement, pin.OP_FRAME)
        self.ee_frame_id = self.model.addFrame(tool_frame)

        self.data = self.model.createData()
        # Thứ tự q khớp đúng với model.names[1:] (bỏ 'universe' ở đầu)
        self.joint_names = list(self.model.names)[1:]
        self.mid_q = (self.model.lowerPositionLimit + self.model.upperPositionLimit) / 2.0

    def fk(self, q) -> pin.SE3:
        """FK đầy đủ (vị trí + hướng) của end-effector. q: mảng góc khớp (rad),
        đúng thứ tự self.joint_names (base_joint, shoulder_joint, elbow_joint)."""
        q = np.asarray(q, dtype=float)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.ee_frame_id].copy()

    def fk_position(self, q) -> np.ndarray:
        """Chỉ lấy vị trí (x,y,z), tiện dùng cho việc vẽ hình."""
        return self.fk(q).translation.copy()

    def jacobian(self, q, frame=pin.LOCAL_WORLD_ALIGNED) -> np.ndarray:
        """Jacobian 6xN tại end-effector: 3 hàng đầu = vận tốc dài (m/s),
        3 hàng sau = vận tốc góc (rad/s). Mặc định biểu diễn theo trục world
        (LOCAL_WORLD_ALIGNED) -- khớp trực tiếp với fk_position()."""
        q = np.asarray(q, dtype=float)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return pin.computeFrameJacobian(self.model, self.data, q, self.ee_frame_id, frame)

    def ik_position(
        self,
        target_pos,
        q_init=None,
        max_iter: int = 200,
        eps: float = 1e-6,
        damp: float = 1e-8,
        dt: float = 1.0,
    ) -> IKResult:
        """Giải IK CHỈ theo vị trí (x,y,z) -- xem lý do ở docstring module.
        Newton-Raphson có damping (Levenberg-Marquardt nhẹ) trên 3 hàng vị trí
        của Jacobian. Hội tụ rất nhanh (thường 3-10 vòng lặp) trừ khi gần
        singularity (Jacobian mất hạng -- xem check_singularity())."""
        q = self.mid_q.copy() if q_init is None else np.array(q_init, dtype=float)
        target_pos = np.asarray(target_pos, dtype=float)

        for i in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            current_pos = self.data.oMf[self.ee_frame_id].translation
            err = target_pos - current_pos
            err_norm = float(np.linalg.norm(err))
            if err_norm < eps:
                return IKResult(q=q, converged=True, iterations=i, position_error_m=err_norm)

            J6 = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_frame_id, pin.LOCAL_WORLD_ALIGNED
            )
            Jp = J6[:3, :]
            JJt = Jp @ Jp.T + damp * np.eye(3)
            v = Jp.T @ np.linalg.solve(JJt, err)
            q = pin.integrate(self.model, q, v * dt)
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)

        final_err = float(np.linalg.norm(target_pos - self.fk_position(q)))
        return IKResult(q=q, converged=False, iterations=max_iter, position_error_m=final_err)

    def seed_from_scan(self, target_pos, n_grid: int = 12) -> np.ndarray:
        """Chọn điểm khởi đầu q cho IK bằng cách quét lưới thô toàn dải khớp và
        lấy cấu hình có FK gần target nhất.

        Vì sao cần (đã gặp thật, không phải phòng xa): ik_position() dùng
        Newton + np.clip vào giới hạn khớp. Nếu điểm khởi đầu ở "phía sai",
        bước Newton đẩy q ra ngoài giới hạn -> clip dán q vào ĐÚNG góc của hộp
        giới hạn và mọi vòng lặp sau đều bị dán lại y chỗ đó (err đứng im hàng
        trăm mm dù Jacobian không hề singular). Quỹ đạo vươn xa ra trước mặt
        rơi vào bẫy này khi khởi đầu từ mid_q. Lưới FK được cache lại nên chỉ
        tốn thời gian ở lần gọi đầu tiên."""
        cache = getattr(self, "_scan_cache", None)
        if cache is None or cache[0] != n_grid:
            axes = [np.linspace(self.model.lowerPositionLimit[i],
                                self.model.upperPositionLimit[i], n_grid)
                    for i in range(self.model.nq)]
            qs = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, self.model.nq)
            pts = np.array([self.fk_position(q) for q in qs])
            cache = (n_grid, qs, pts)
            self._scan_cache = cache
        _, qs, pts = cache
        return qs[np.linalg.norm(pts - np.asarray(target_pos, dtype=float), axis=1).argmin()].copy()

    def check_singularity(self, q, threshold: float = 50.0) -> bool:
        """True nếu Jacobian vị trí gần mất hạng (condition number vượt ngưỡng)
        -- vùng này IK sẽ hội tụ chậm/không ổn định, nên tránh khi chọn vùng vẽ."""
        J6 = self.jacobian(q)
        cond = np.linalg.cond(J6[:3, :])
        return cond > threshold

    def solve_trajectory(self, positions: list, q_init=None) -> list:
        """Giải IK cho 1 chuỗi điểm (x,y,z) liên tiếp -- dùng nghiệm điểm
        trước làm điểm khởi đầu cho điểm sau (warm start), vừa nhanh vừa ổn
        định hơn nhiều so với luôn bắt đầu từ 1 điểm cố định.

        Điểm ĐẦU TIÊN (khi không truyền q_init) được gieo bằng seed_from_scan()
        thay vì mid_q, và bất kỳ điểm nào không hội tụ đều được giải lại bằng
        hạt giống quét -- xử lý đúng cái bẫy "kẹt ở góc giới hạn khớp" mô tả
        trong seed_from_scan()."""
        results = []
        if q_init is None:
            q_current = self.seed_from_scan(positions[0]) if len(positions) else self.mid_q.copy()
        else:
            q_current = np.array(q_init, dtype=float)
        for pos in positions:
            res = self.ik_position(pos, q_init=q_current)
            if not res.converged:
                retry = self.ik_position(pos, q_init=self.seed_from_scan(pos))
                if retry.position_error_m < res.position_error_m:
                    res = retry
            results.append(res)
            q_current = res.q  # warm start cho điểm tiếp theo
        return results


if __name__ == "__main__":
    # Demo/self-test nhanh khi chạy trực tiếp file này.
    import sys

    urdf_path = sys.argv[1] if len(sys.argv) > 1 else "gim_arm.urdf"
    # TODO: thay bằng khoảng cách THẬT đo được từ điểm nối elbow_joint tới đầu bút.
    # 0.2m theo trục X cục bộ chỉ là số ví dụ để chạy demo, KHÔNG phải số thật.
    kin = GimArmKinematics(urdf_path, tool_offset_xyz=(0.2, 0.0, 0.0))

    print("Khớp (đúng thứ tự q):", kin.joint_names)
    print("Điểm giữa dải góc (rad):", kin.mid_q.round(4))

    np.random.seed(0)
    n_ok = 0
    n_test = 20
    for _ in range(n_test):
        q_true = np.random.uniform(kin.model.lowerPositionLimit, kin.model.upperPositionLimit)
        target = kin.fk_position(q_true)
        res = kin.ik_position(target)
        pos_now = kin.fk_position(res.q)
        err_mm = np.linalg.norm(pos_now - target) * 1000
        n_ok += err_mm < 0.5
    print(f"Round-trip IK(FK(q)): {n_ok}/{n_test} đạt sai số dưới 0.5mm")