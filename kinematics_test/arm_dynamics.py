"""
arm_dynamics.py — mô hình động lực học LAGRANGE của GIM Arm 3DOF:

    M(q) q̈ + C(q,q̇) q̇ + G(q) + τ_ms = τ

M, C, G lấy từ Pinocchio đọc thẳng URDF (không tự dẫn công thức Lagrange bằng
tay). Lý do dùng thư viện chứ không dẫn tay: Pinocchio cài đặt đúng thuật toán
CRBA/RNEA -- vốn CHÍNH LÀ phương trình Lagrange được tổ chức lại cho máy tính
(RNEA = Newton-Euler đệ quy, cho ra cùng kết quả với Lagrange nhưng O(n) thay
vì O(n^3) và không sai dấu khi khai triển tay). Khối lượng / tâm khối / ma trận
quán tính của từng link đã có sẵn trong URDF (xuất từ CAD), nên tự viết lại
công thức chỉ thêm chỗ để sai.

Ánh xạ sang ký hiệu Lagrange:
    M(q)      = pin.crba(model, data, q)          (ma trận quán tính, = ∂²T/∂q̇²)
    C(q,q̇)q̇  = pin.nonLinearEffects(...) - G(q)   (Coriolis + ly tâm)
    G(q)      = pin.computeGeneralizedGravity(...) (= ∂U/∂q)
    τ         = pin.rnea(model, data, q, q̇, q̈)    (nghịch động lực học đầy đủ)

BA THỨ URDF KHÔNG CÓ, phải thêm tay ở đây (nếu bỏ qua thì mô hình sai ở đúng
những khớp có hộp số lớn):
  1) armature = J_rotor * N^2 -- quán tính rotor phản chiếu qua hộp số. Với
     shoulder N=64 thì J_rotor*N^2 = 0.0108 kg·m², bằng 7% quán tính link của
     chính khớp đó (0.149) -- bỏ qua là mô hình sai 7% ở khớp nặng nhất, và
     sai theo kiểu cộng thẳng vào ĐƯỜNG CHÉO của M nên đi trực tiếp vào mô-men
     tính ra. Ở base/elbow (N=8) phần này chỉ 0.00017, cỡ 0.4%. Cộng thẳng vào M ở đây thay vì gán model.armature, để không phụ
     thuộc vào chuyện phiên bản Pinocchio nào có áp dụng model.armature trong
     crba/rnea (đã kiểm: xem verify_against_mujoco()).
  2) ma sát nhớt + ma sát khô của hộp số. LẤY ĐÚNG số trong
     gim6010_mujoco_sim.py để mô hình và mô phỏng dùng cùng một bộ số.
     CẢNH BÁO: 2 số này trong sim đang ghi rõ là "SỐ ĐẶT TẠM, chưa hiệu chỉnh
     từ log CAN thật" -- nên phần bù ma sát ở đây là phần YẾU NHẤT của mô hình,
     và cũng chính là lý do cần thành phần tích phân (I trong LQI).
  3) giới hạn mô-men: đọc từ <limit effort> của URDF (5/40/5 Nm).

Kiểm chứng: verify_against_mujoco() so M và (C q̇ + G) của Pinocchio với
mj_fullM và qfrc_bias của MuJoCo ở các cấu hình ngẫu nhiên. Hai thư viện này
cài đặt độc lập nhau, nên trùng khớp tới ~1e-12 là bằng chứng mô hình Lagrange
dùng cho LQI đúng với đúng cái vật lý mà mô phỏng đang chạy.

Chạy tự kiểm tra:  python3 arm_dynamics.py
"""

import numpy as np
import pinocchio as pin

# Giữ ĐÚNG bằng các số trong mujoco_env.py và gim6010_mujoco_sim.py.
ROTOR_INERTIA_KGM2 = 26.3e-7          # 26.3 g·cm² (datasheet GIM6010-8)
GEAR_RATIOS = {"base_joint": 8.0, "shoulder_joint": 64.0, "elbow_joint": 8.0}
VISCOUS_PER_N2 = 2.0e-5               # Nm/(rad/s) trên mỗi đơn vị N²  (SỐ ĐẶT TẠM)
DRY_FRICTION_PER_N = 0.004            # Nm trên mỗi đơn vị N            (SỐ ĐẶT TẠM)


class ArmDynamics:
    """Mô hình Lagrange + truyền động. Mọi đại lượng ở PHÍA KHỚP (rad, Nm),
    không phải phía rotor -- quy đổi sang rotor là việc của tầng CAN."""

    def __init__(self, urdf_path: str, use_armature: bool = True):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.joint_names = list(self.model.names)[1:]
        self.nq = self.model.nq

        n = np.array([GEAR_RATIOS[j] for j in self.joint_names], dtype=float)
        self.gear_ratios = n
        self.armature = ROTOR_INERTIA_KGM2 * n ** 2 if use_armature else np.zeros_like(n)
        self.viscous = VISCOUS_PER_N2 * n ** 2
        self.dry_friction = DRY_FRICTION_PER_N * n

        self.tau_max = np.asarray(self.model.effortLimit, dtype=float).copy()
        self.vel_max = np.asarray(self.model.velocityLimit, dtype=float).copy()
        self.q_min = np.asarray(self.model.lowerPositionLimit, dtype=float).copy()
        self.q_max = np.asarray(self.model.upperPositionLimit, dtype=float).copy()

    # ------------------------------------------------------------------
    # Các khối của phương trình Lagrange
    # ------------------------------------------------------------------
    def mass_matrix(self, q) -> np.ndarray:
        """M(q), đã cộng quán tính rotor phản chiếu qua hộp số."""
        M = pin.crba(self.model, self.data, np.asarray(q, dtype=float)).copy()
        M[np.diag_indices_from(M)] += self.armature
        return M

    def gravity(self, q) -> np.ndarray:
        """G(q) = ∂U/∂q -- mô-men cần để GIỮ tay ở tư thế q (chống trọng lực)."""
        return pin.computeGeneralizedGravity(
            self.model, self.data, np.asarray(q, dtype=float)
        ).copy()

    def nonlinear(self, q, qd) -> np.ndarray:
        """C(q,q̇)q̇ + G(q) -- gộp Coriolis/ly tâm và trọng lực (bias force)."""
        return pin.nonLinearEffects(
            self.model, self.data, np.asarray(q, dtype=float), np.asarray(qd, dtype=float)
        ).copy()

    def coriolis(self, q, qd) -> np.ndarray:
        """Chỉ riêng C(q,q̇)q̇, tách trọng lực ra -- tiện để xem số."""
        return self.nonlinear(q, qd) - self.gravity(q)

    def inverse_dynamics(self, q, qd, qdd) -> np.ndarray:
        """τ = M(q)q̈ + C(q,q̇)q̇ + G(q). CHƯA gồm ma sát (xem friction())."""
        qdd = np.asarray(qdd, dtype=float)
        tau = pin.rnea(
            self.model, self.data,
            np.asarray(q, dtype=float), np.asarray(qd, dtype=float), qdd,
        ).copy()
        return tau + self.armature * qdd      # rnea không tính armature

    def friction(self, qd, smooth_eps: float = 0.02) -> np.ndarray:
        """Mô-men ma sát ƯỚC LƯỢNG khi khớp đang chạy ở tốc độ qd.

        Dùng tanh(qd/eps) thay cho sign(qd) vì phần bù ma sát khô mà dùng sign
        sẽ đảo dấu đột ngột mỗi lần qd đi qua 0 -> tự gây dao động tần số cao
        (chattering) quanh điểm dừng. eps = 0.02 rad/s là vùng chuyển mềm."""
        qd = np.asarray(qd, dtype=float)
        return self.viscous * qd + self.dry_friction * np.tanh(qd / smooth_eps)

    def forward_dynamics(self, q, qd, tau) -> np.ndarray:
        """q̈ = M⁻¹(τ - Cq̇ - G - ma sát). Dùng để kiểm tra mô hình, không dùng
        trong vòng điều khiển."""
        rhs = np.asarray(tau, dtype=float) - self.nonlinear(q, qd) - self.friction(qd)
        return np.linalg.solve(self.mass_matrix(q), rhs)

    # ------------------------------------------------------------------
    # Kiểm chứng độc lập
    # ------------------------------------------------------------------
    def verify_against_mujoco(self, urdf_path: str, n_samples: int = 20, seed: int = 0):
        """So M và (Cq̇+G) của Pinocchio với mj_fullM và qfrc_bias của MuJoCo.

        Hai thư viện cài đặt hoàn toàn độc lập (Pinocchio: CRBA/RNEA trên
        Lagrange; MuJoCo: solver riêng của nó) -> khớp nhau nghĩa là mô hình
        Lagrange dùng cho LQI đúng bằng vật lý mà mô phỏng chạy.

        Nạp MuJoCo THẲNG từ URDF, KHÔNG qua MujocoEnv: MujocoEnv có bước
        mj_saveLastXML rồi nạp lại, mà XML ghi số dạng text nên mất chữ số
        (đo được: sai lệch nhảy từ 1e-9 lên 8e-8 ở M và 6e-6 ở bias chỉ vì
        vòng lưu/nạp đó). Ở đây cần kiểm MÔ HÌNH, không phải kiểm bộ ghi XML.

        Phần dư ~1e-9 còn lại là do MuJoCo lưu quán tính link dưới dạng riêng
        của nó (quán tính chéo hoá + quaternion khung), không phải sai mô hình.
        Trả về (sai_số_M_lớn_nhất, sai_số_bias_lớn_nhất)."""
        import os
        import re
        import tempfile

        import mujoco

        with open(urdf_path, "r", encoding="utf-8") as f:
            content = re.sub(r"package://[^/]+/meshes/", "meshes/", f.read())
        # để cùng thư mục với URDF gốc thì đường dẫn mesh tương đối mới đúng
        tmp = tempfile.NamedTemporaryFile(
            "w", suffix=".urdf", delete=False,
            dir=os.path.dirname(os.path.abspath(urdf_path)), encoding="utf-8",
        )
        try:
            tmp.write(content)
            tmp.close()
            model = mujoco.MjModel.from_xml_path(tmp.name)
        finally:
            os.unlink(tmp.name)
        data = mujoco.MjData(model)

        mj_dof = np.array([
            int(model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])
            for name in self.joint_names
        ])

        rng = np.random.default_rng(seed)
        err_M = err_bias = 0.0
        for _ in range(n_samples):
            q = rng.uniform(self.q_min, self.q_max)
            qd = rng.uniform(-1.0, 1.0, size=self.nq)

            data.qpos[:] = q
            data.qvel[:] = qd
            mujoco.mj_forward(model, data)
            # mujoco >=3.11: mj_fullM nhận (model, data, dst), tự lấy ma trận
            # khối lượng dạng thưa trong data.M rồi bung ra dạng đặc.
            M_mj = np.zeros((model.nv, model.nv))
            mujoco.mj_fullM(model, data, M_mj)
            M_mj = M_mj[np.ix_(mj_dof, mj_dof)]
            bias_mj = data.qfrc_bias[mj_dof].copy()

            # MuJoCo nạp thẳng từ URDF không có armature (mujoco_env mới là chỗ
            # thêm vào), nên trừ phần rotor ra cho cùng hệ quy chiếu.
            M_pin = self.mass_matrix(q)
            M_pin[np.diag_indices_from(M_pin)] -= self.armature
            err_M = max(err_M, np.abs(M_pin - M_mj).max())
            err_bias = max(err_bias, np.abs(self.nonlinear(q, qd) - bias_mj).max())
        return err_M, err_bias


if __name__ == "__main__":
    import sys

    urdf = sys.argv[1] if len(sys.argv) > 1 else "gim_arm.urdf"
    dyn = ArmDynamics(urdf)

    print("Khớp:", dyn.joint_names)
    print("gear_ratio:", dyn.gear_ratios)
    print("armature = J_rotor*N² (kg·m²):", dyn.armature)
    print("giới hạn mô-men từ URDF (Nm):", dyn.tau_max)
    print("ma sát nhớt (Nm/(rad/s)):", dyn.viscous)
    print("ma sát khô (Nm):", dyn.dry_friction)
    print()

    q0 = (dyn.q_min + dyn.q_max) / 2
    M = dyn.mass_matrix(q0)
    print("Tại tư thế giữa dải khớp q =", q0.round(4))
    print("M(q) =\n", M.round(6))
    print("đường chéo M, phần link / phần rotor:",
          (np.diag(M) - dyn.armature).round(6), "/", dyn.armature.round(6))
    print("G(q) (Nm) =", dyn.gravity(q0).round(4))
    print("C(q,q̇)q̇ tại q̇=[1,1,1] (Nm) =", dyn.coriolis(q0, np.ones(3)).round(4))
    print()

    # Quán tính hiệu dụng thay đổi bao nhiêu trên toàn vùng làm việc -> đây
    # chính là lý do PID hệ số cố định không thể tối ưu ở mọi tư thế.
    rng = np.random.default_rng(0)
    diags = np.array([np.diag(dyn.mass_matrix(rng.uniform(dyn.q_min, dyn.q_max)))
                      for _ in range(500)])
    print("M_ii nhỏ nhất :", diags.min(axis=0).round(5))
    print("M_ii lớn nhất :", diags.max(axis=0).round(5))
    print("tỉ lệ max/min :", (diags.max(axis=0) / diags.min(axis=0)).round(2),
          "<- PID hệ số cố định phải chấp nhận cùng 1 bộ gain cho cả dải này")
    print()

    err_M, err_bias = dyn.verify_against_mujoco(urdf)
    print("Kiểm chứng với MuJoCo (20 cấu hình ngẫu nhiên, nạp thẳng từ URDF):")
    print(f"  sai lệch M lớn nhất      = {err_M:.3e}  (M cỡ 0.03..0.16)")
    print(f"  sai lệch (Cq̇+G) lớn nhất = {err_bias:.3e}  (bias cỡ 0.5..2 Nm)")
    ok = err_M < 1e-8 and err_bias < 1e-8
    print("  =>", "KHỚP -- mô hình Lagrange đúng bằng vật lý mô phỏng"
          if ok else "LỆCH -- phải tìm nguyên nhân trước khi dùng cho LQI")
