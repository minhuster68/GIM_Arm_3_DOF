"""
mujoco_env.py — môi trường MuJoCo thuần Python. Load MJCF, step simulation,
điều khiển qua data.ctrl[...] (mô-men thật, Nm) hoặc trực tiếp qua data.qpos.

Task #9 (Setup Actuator & System Dynamics) đã tích hợp vào đây:
- armature = rotor_inertia * gear_ratio^2 (quán tính rotor phản ánh qua hộp
  số) -- dùng ĐÚNG gear_ratio riêng từng khớp (8 cho base/elbow, 64 cho
  shoulder vì có hộp giảm tốc ngoài) -- khớp chính xác với gear_ratios_ đã
  dùng trong gim_arm_hardware/gim_arm_system.cpp, không phải số bừa.
- ctrlrange = giới hạn mô-men THẬT tại khớp, theo datasheet GIM6010-8 (rated
  5Nm ở tỉ số 8:1 gốc của motor). Với shoulder có thêm hộp ngoài 8:1, mô-men
  khả dụng THẬT tại khớp cuối là 5 * (64/8) = 40Nm. URDF trước đây khai 5Nm
  cho cả 3 khớp (thiếu 8 lần ở shoulder); đã sửa lại đúng 5/40/5 Nm nên
  ctrlrange tính ở đây giờ KHỚP với <limit effort> trong URDF.
- ctrl ở đây đại diện cho MÔ-MEN (Nm) tại khớp, KHÔNG phải dòng điện -- nên
  actuator "gear" (hệ số scale ctrl->force của MuJoCo) để =1 (pass-through).

CHƯA làm trong file này (để dành #11, cần dữ liệu log CAN thật):
- damping/frictionloss của <joint> -- vẫn để mặc định 0, chưa tune.
"""

import re

import mujoco
import numpy as np

JOINT_NAMES = ["base_joint", "shoulder_joint", "elbow_joint"]

# Datasheet GIM6010-8 (xác nhận qua nhiều nguồn, khớp nhau tuyệt đối):
# rated 5Nm / stall 11Nm (ở tỉ số 8:1 gốc), rotor inertia 26.3 g*cm^2.
ROTOR_INERTIA_KGM2 = 26.3e-7  # 26.3 g*cm^2 -> kg*m^2
RATED_TORQUE_AT_8TO1_NM = 5.0

# gear_ratio TỔNG mỗi khớp -- PHẢI khớp đúng gear_ratios_ trong
# gim_arm_hardware/gim_arm_system.cpp (base/elbow=8 mặc định, shoulder=64
# vì có thêm hộp giảm tốc ngoài 8:1).
GEAR_RATIOS = {"base_joint": 8.0, "shoulder_joint": 64.0, "elbow_joint": 8.0}


class MujocoEnv:
    def __init__(self, urdf_path: str, fixed_urdf_path: str = "gim_arm_mujoco.urdf"):
        # Sửa package:// (quy ước ROS, MuJoCo không hiểu) -- bắt buộc mỗi lần
        # đồng bộ URDF mới.
        with open(urdf_path, "r", encoding="utf-8") as f:
            content = f.read()
        fixed = re.sub(r"package://[^/]+/meshes/", "meshes/", content)
        with open(fixed_urdf_path, "w", encoding="utf-8") as f:
            f.write(fixed)

        base_model = mujoco.MjModel.from_xml_path(fixed_urdf_path)
        mujoco.mj_saveLastXML("_tmp_gim_arm.xml", base_model)
        with open("_tmp_gim_arm.xml", "r", encoding="utf-8") as f:
            xml = f.read()

        # 1) Thêm armature vào từng <joint>, đúng gear_ratio riêng của khớp đó.
        for name, gear_ratio in GEAR_RATIOS.items():
            armature = ROTOR_INERTIA_KGM2 * gear_ratio ** 2
            xml = re.sub(
                rf'(<joint name="{name}"[^>]*)/>',
                rf'\1 armature="{armature:.8f}"/>',
                xml,
            )

        # 2) Thêm actuator: ctrl = mô-men thật (Nm) tại khớp. gear=1 (pass-
        # through, KHÔNG phải tỉ số truyền cơ khí -- đừng nhầm 2 khái niệm).
        actuator_lines = []
        for name, gear_ratio in GEAR_RATIOS.items():
            rated_at_joint = RATED_TORQUE_AT_8TO1_NM * (gear_ratio / 8.0)
            actuator_lines.append(
                f'    <motor name="{name}_motor" joint="{name}" gear="1" '
                f'ctrlrange="-{rated_at_joint:.1f} {rated_at_joint:.1f}"/>'
            )
        actuator_block = "\n  <actuator>\n" + "\n".join(actuator_lines) + "\n  </actuator>\n</mujoco>"
        xml_with_actuator = xml.replace("</mujoco>", actuator_block)

        self.model = mujoco.MjModel.from_xml_string(xml_with_actuator)
        self.data = mujoco.MjData(self.model)
        self._renderer = None

    def reset(self, qpos=None):
        mujoco.mj_resetData(self.model, self.data)
        if qpos is not None:
            self.data.qpos[:] = qpos
        mujoco.mj_forward(self.model, self.data)

    def step(self, ctrl=None):
        """Bước mô phỏng động lực học thật. ctrl = mô-men (Nm) mong muốn tại
        mỗi khớp -- giờ đã đúng đơn vị/giới hạn thật (khác placeholder gear=1
        không tính vật lý trước đây)."""
        if ctrl is not None:
            self.data.ctrl[:] = ctrl
        mujoco.mj_step(self.model, self.data)

    def set_qpos_direct(self, qpos):
        """Đặt thẳng vị trí khớp (bỏ qua động lực học/actuator) -- dùng để
        kiểm tra nhanh 1 quỹ đạo IK, không cần lo lực/mô-men."""
        self.data.qpos[:] = qpos
        mujoco.mj_forward(self.model, self.data)

    def get_qpos(self):
        return self.data.qpos.copy()

    def is_healthy(self) -> bool:
        return bool(np.all(np.isfinite(self.data.qpos)) and np.all(np.isfinite(self.data.qvel)))

    def render(self, width=640, height=480):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)
        self._renderer.update_scene(self.data)
        return self._renderer.render()


if __name__ == "__main__":
    env = MujocoEnv("gim_arm.urdf")
    print(f"Model nạp OK: nq={env.model.nq}, nu={env.model.nu}")
    for i in range(env.model.njnt):
        name = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_JOINT, i)
        print(f"  {name}: armature={env.model.dof_armature[i]:.6f} kg*m^2, "
              f"ctrlrange={env.model.actuator_ctrlrange[i]}")
    print()

    # Test 1: step với ctrl=0, vài nghìn step, kiểm tra không NaN/nổ
    env.reset()
    n_steps = 5000
    max_qpos_seen = 0.0
    for i in range(n_steps):
        env.step(ctrl=np.zeros(env.model.nu))
        if not env.is_healthy():
            print(f"LỖI: NaN/Inf xuất hiện ở step {i}")
            break
        max_qpos_seen = max(max_qpos_seen, np.max(np.abs(env.data.qpos)))
    else:
        print(f"Test 1 (ctrl=0, {n_steps} step): ỔN ĐỊNH, |qpos| lớn nhất gặp = {max_qpos_seen:.4f} rad")

    # Test 2: phát lại quỹ đạo qua set_qpos_direct
    env.reset()
    ok = True
    for t in np.linspace(0, 2 * np.pi, 200):
        # elbow -0.3 nam ngoai gioi han that (elbow >= -0.2336 sau khi do lai
        # bang encoder 19/08/2026) -> doi ve 0.95, giua dai lam viec cua quy dao quet.
        q = np.array([0.3 + 0.1 * np.sin(t), 0.5 + 0.1 * np.cos(t), 0.95])
        env.set_qpos_direct(q)
        if not env.is_healthy():
            ok = False
            break
    print(f"Test 2 (phát lại quỹ đạo qua set_qpos_direct, 200 điểm): {'ỔN ĐỊNH' if ok else 'LỖI NaN'}")

    # Test 3: phát lại chữ O thật từ shapes.py + IK
    from shapes import letter_o, discretize
    from gim_arm_kinematics import GimArmKinematics

    kin = GimArmKinematics("gim_arm.urdf", tool_offset_xyz=(0.4031, 0.049, -0.029))
    path_o = letter_o(center=(-0.2486, 0.2), radius=0.05, plane="x", plane_value=0.1)
    positions = discretize(path_o, n_points=60, close_loop=True)
    results = kin.solve_trajectory(positions)

    env.reset()
    ok3 = True
    for r in results:
        env.set_qpos_direct(r.q)
        if not env.is_healthy():
            ok3 = False
            break
    print(f"Test 3 (phát lại chữ O thật từ shapes.py + IK): {'ỔN ĐỊNH' if ok3 else 'LỖI NaN'}")

    # Test 4 (MỚI): dùng actuator thật (ctrl=mô-men) để giữ tay chống trọng
    # lực tại 1 tư thế -- kiểm tra armature/ctrlrange mới không gây NaN khi
    # thật sự dùng động lực học (khác Test 1 chỉ test ctrl=0).
    env.reset(qpos=[0.4, 0.5, 0.95])   # elbow -0.4 cu nam ngoai gioi han that
    ok4 = True
    for i in range(2000):
        # mô-men bừa trong giới hạn, chỉ để kiểm tra ổn định số học, CHƯA
        # phải gravity compensation thật (việc đó cần Inverse Dynamics, #16-17)
        ctrl = np.array([0.5, 5.0, 0.5])
        env.step(ctrl=ctrl)
        if not env.is_healthy():
            ok4 = False
            print(f"LỖI: NaN ở step {i} khi test Test 4")
            break
    print(f"Test 4 (step động lực học thật với ctrl khác 0, 2000 step): {'ỔN ĐỊNH' if ok4 else 'LỖI NaN'}")