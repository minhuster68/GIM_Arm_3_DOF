#!/usr/bin/env python3
"""
gim6010_mujoco_sim.py — giả lập PHẦN CỨNG THẬT của GIM Arm 3DOF: 3 driver
GIM6010-8 + bus CAN, với động lực học do MuJoCo tính.

Nó nói đúng giao thức CAN mà src/gim_arm_hardware/ đang dùng, nên plugin
ros2_control C++ KHÔNG cần sửa một dòng nào: cứ trỏ can_interface vào một
interface CAN ảo là toàn bộ pipeline (MoveIt -> JTC -> hardware -> CAN ->
driver -> encoder -> /joint_states) chạy y như có tay máy thật.

    ┌─ ros2 launch gim_control gim_arm_control.launch.py   (không đổi gì)
    │        │  0x00C Set_Input_Pos  (rev, phía rotor)
    │        v
    │   [ file này ]  ──> cascade PID của ODrive ──> mô-men khớp ──> MuJoCo
    │        │  0x009 Get_Encoder_Estimates  (rev, rev/s)
    └────────┘

CÀI ĐẶT
    pip install mujoco --break-system-packages
    (không cần python-can, file này dùng socket AF_CAN của stdlib)

DỰNG CAN ẢO — đặt tên đúng "can0" để khỏi phải sửa URDF
    sudo modprobe vcan
    sudo ip link add dev can0 type vcan     # bỏ qua nếu can0 thật đang cắm
    sudo ip link set up can0

CHẠY
    # 1) tự kiểm tra, không cần CAN, in ra overshoot/settling của step response
    python3 gim6010_mujoco_sim.py --selftest

    # 2) giả lập đầy đủ, mở cửa sổ MuJoCo xem tay máy chuyển động
    python3 gim6010_mujoco_sim.py --can can0 --viewer

    # rồi ở terminal khác, chạy y như thật:
    ros2 launch gim_control gim_arm_control.launch.py
    ros2 run gim_control draw_trajectory

NHỮNG THỨ ĐƯỢC GIẢ LẬP ĐÚNG
  - CAN ID = (node_id << 5) | cmd_id, standard frame 11 bit
  - 0x007 Set_Axis_State, 0x00B Set_Controller_Mode, 0x00C Set_Input_Pos,
    0x018 Clear_Errors, 0x01A Set_Pos_Gain, 0x01B Set_Vel_Gains,
    0x01F Save_Configuration
  - 0x001 Heartbeat, 0x009 Get_Encoder_Estimates (10ms), 0x014 Get_Iq,
    0x017 Get_Bus_Voltage_Current, 0x01C Get_Torques
  - Đơn vị phía rotor (rev, rev/s) — đúng như 0x00C/0x009 thật, KHÔNG phải
    phía trục ra. Quy đổi rotor<->khớp lấy gear_ratio / invert_direction /
    zero_offset_rad đọc thẳng từ khối <ros2_control> trong URDF, nên không
    bao giờ lệch với gim_arm_system.cpp.
  - Cascade điều khiển của ODrive: bộ lọc input_mode=3 (POS_FILTER bậc 2)
    -> P vị trí -> PI vận tốc -> giới hạn mô-men.
  - Quán tính rotor phản chiếu qua hộp số (armature = J_rotor * N^2), giới
    hạn góc khớp, trọng lực, ma sát nhớt + ma sát khô.
  - Quirk thật của driver: lúc IDLE, Get_Encoder_Estimates phát toàn 0 (đúng
    hiện tượng đã xác nhận bằng candump 2026-08-05), nên đoạn "đua để chốt
    setpoint" trong on_activate() được thử đúng như trên máy thật.

NHỮNG THỨ *KHÔNG* ĐƯỢC GIẢ LẬP — đọc kỹ trước khi tin số liệu
  - Vòng dòng điện (FOC), cogging, tiếng ù, độ đàn hồi/backlash hộp số, giới
    hạn nhiệt, nhiễu encoder, trễ và mất frame trên bus CAN.
  - Vòng điều khiển ở đây chạy 2 kHz (xem CONTROL_HZ), driver thật 8 kHz. Sim
    do đó CHỊU ĐƯỢC gain cao hơn thực tế. Đừng bê gain tune trong sim xuống
    máy thật rồi tin ngay — dùng sim để kiểm pipeline, quỹ đạo, IK, vùng
    làm việc và bắt lỗi logic, còn gain cuối cùng vẫn phải tune trên phần
    cứng thật.
  - Ma sát và giới hạn mô-men bên dưới là số ĐẶT TẠM, chưa hiệu chỉnh theo
    log CAN thật. Xem khối THÔNG SỐ PHẦN CỨNG.
"""

import argparse
import math
import os
import re
import socket
import struct
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

# ─────────────────── THÔNG SỐ PHẦN CỨNG (sửa ở đây) ───────────────────

CONTROL_HZ = 2000.0          # tần số cascade + bước vật lý MuJoCo
FEEDBACK_PERIOD_S = 0.010    # chu kỳ broadcast 0x009, mặc định driver là 10ms
HEARTBEAT_PERIOD_S = 0.100

# Datasheet GIM6010-8. Giữ khớp với kinematics_test/mujoco_env.py.
ROTOR_INERTIA_KGM2 = 26.3e-7     # 26.3 g*cm^2
# Mô-men giới hạn PHÍA ROTOR. 5 Nm định mức tại trục ra / 8 (hộp số nội bộ).
# Quy về khớp sẽ là 0.625*8 = 5 Nm cho base/elbow, 0.625*64 = 40 Nm cho
# shoulder — khớp với <limit effort> trong URDF ở base/elbow.
ROTOR_TORQUE_LIM_NM = 0.625
VEL_LIMIT_REV_S = 20.0           # controller.config.vel_limit, phía rotor

# Gain mặc định = mặc định gốc của ODrive. Đổi bằng cờ dòng lệnh, hoặc gửi
# 0x01A / 0x01B lúc đang chạy y như với driver thật.
DEFAULT_POS_GAIN = 20.0          # (rev/s)/rev
DEFAULT_VEL_GAIN = 0.16          # Nm/(rev/s)
DEFAULT_VEL_INTEGRATOR_GAIN = 0.32
DEFAULT_INPUT_FILTER_BW = 2.0    # input_mode=3 dùng số này

# SỐ ĐẶT TẠM — chưa hiệu chỉnh từ log thật. Ma sát quy về phía khớp.
# Tỉ lệ N^2 cho ma sát nhớt và N cho ma sát khô là cách xấp xỉ thô ma sát
# hộp số quy đổi về trục ra.
VISCOUS_PER_N2 = 2.0e-5          # Nm/(rad/s) trên mỗi đơn vị N^2
DRY_FRICTION_PER_N = 0.004       # Nm trên mỗi đơn vị N

BUS_VOLTAGE_V = 24.0
TORQUE_CONSTANT_NM_PER_A = 8.23 / 12.3   # theo README của repo

# ─────────────────────── giao thức CAN ───────────────────────

CMD_HEARTBEAT = 0x001
CMD_ESTOP = 0x002
CMD_GET_ERROR = 0x003
CMD_SET_AXIS_STATE = 0x007
CMD_GET_ENCODER_ESTIMATES = 0x009
CMD_SET_CONTROLLER_MODE = 0x00B
CMD_SET_INPUT_POS = 0x00C
CMD_SET_INPUT_VEL = 0x00D
CMD_SET_INPUT_TORQUE = 0x00E
CMD_GET_IQ = 0x014
CMD_REBOOT = 0x016
CMD_GET_BUS_VOLTAGE_CURRENT = 0x017
CMD_CLEAR_ERRORS = 0x018
CMD_SET_POS_GAIN = 0x01A
CMD_SET_VEL_GAINS = 0x01B
CMD_GET_TORQUES = 0x01C
CMD_SAVE_CONFIGURATION = 0x01F

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

CAN_SFF_MASK = 0x7FF
CAN_RTR_FLAG = 0x40000000
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


def make_can_id(node_id: int, cmd_id: int) -> int:
    return (node_id << 5) | cmd_id


class CanBus:
    """Bọc socket AF_CAN thô — cùng cách mà socketcan_bus.hpp đang làm."""

    def __init__(self, interface: str):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))
        self.sock.setblocking(False)

    def send(self, can_id: int, data: bytes) -> None:
        payload = data.ljust(8, b"\x00")
        frame = struct.pack(CAN_FRAME_FMT, can_id, len(data), payload)
        try:
            self.sock.send(frame)
        except OSError:
            pass  # buffer đầy: bỏ frame, đúng như bus thật khi nghẽn

    def receive_all(self):
        out = []
        while True:
            try:
                raw = self.sock.recv(CAN_FRAME_SIZE)
            except (BlockingIOError, InterruptedError):
                return out
            if len(raw) < CAN_FRAME_SIZE:
                return out
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
            out.append((can_id, dlc, data[:dlc]))

    def close(self):
        self.sock.close()


# ─────────────────────── đọc cấu hình từ URDF ───────────────────────

class AxisConfig:
    def __init__(self, node_id, joint_name, gear_ratio, direction, zero_offset):
        self.node_id = int(node_id)
        self.joint_name = joint_name
        self.gear_ratio = float(gear_ratio)
        self.direction = float(direction)
        self.zero_offset = float(zero_offset)

    def __repr__(self):
        return (f"node {self.node_id} -> {self.joint_name} "
                f"(gear {self.gear_ratio:g}, dir {self.direction:+.0f}, "
                f"offset {self.zero_offset:.4f} rad)")


def parse_axes_from_urdf(urdf_path: str):
    """Lấy node_id / gear_ratio / invert_direction / zero_offset_rad từ khối
    <ros2_control> — cùng nguồn sự thật mà on_init() của plugin C++ đọc, nên
    sim không thể lệch cấu hình với phần cứng."""
    root = ET.parse(urdf_path).getroot()
    block = root.find("ros2_control")
    if block is None:
        raise RuntimeError(f"Không thấy khối <ros2_control> trong {urdf_path}")

    axes = []
    for joint in block.findall("joint"):
        params = {p.get("name"): (p.text or "").strip() for p in joint.findall("param")}
        if "can_node_id" not in params:
            raise RuntimeError(f"Khớp '{joint.get('name')}' thiếu can_node_id")
        axes.append(AxisConfig(
            node_id=params["can_node_id"],
            joint_name=joint.get("name"),
            gear_ratio=params.get("gear_ratio", 8.0),
            direction=-1.0 if params.get("invert_direction") == "true" else 1.0,
            zero_offset=params.get("zero_offset_rad", 0.0),
        ))
    return axes


def resolve_mesh_paths(urdf_path: str) -> str:
    """MuJoCo không hiểu package:// của ROS. Đổi sang đường dẫn tuyệt đối rồi
    ghi ra file tạm, để không phải đứng đúng thư mục mới chạy được."""
    with open(urdf_path, encoding="utf-8") as f:
        content = f.read()

    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))

    def find_mesh_dir(pkg: str):
        try:
            from ament_index_python.packages import get_package_share_directory
            cand = os.path.join(get_package_share_directory(pkg), "meshes")
            if os.path.isdir(cand):
                return cand
        except Exception:
            pass
        probe = urdf_dir
        for _ in range(4):
            cand = os.path.join(probe, "meshes")
            if os.path.isdir(cand):
                return cand
            probe = os.path.dirname(probe)
        raise RuntimeError(
            f"Không tìm được thư mục meshes cho package '{pkg}'. "
            f"Đã dò ament index và các thư mục cha của {urdf_dir}.")

    def repl(m):
        return find_mesh_dir(m.group(1)) + "/"

    content = re.sub(r"package://([^/]+)/meshes/", repl, content)
    tmp = tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False, encoding="utf-8")
    tmp.write(content)
    tmp.close()
    return tmp.name


# ─────────────────────── mô hình MuJoCo ───────────────────────

class ArmPhysics:
    """MuJoCo + các thông số truyền động. Mô-men đưa vào qua qfrc_applied nên
    không cần vá XML để thêm <actuator>."""

    def __init__(self, urdf_path: str, axes):
        fixed = resolve_mesh_paths(urdf_path)
        try:
            self.model = mujoco.MjModel.from_xml_path(fixed)
        finally:
            os.unlink(fixed)

        self.model.opt.timestep = 1.0 / CONTROL_HZ
        # Tắt tiếp xúc: tay máy đứng một mình, mà convex hull của các mesh này
        # chồng nhau sẵn ở gốc nên bật lên là nổ ngay.
        self.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)

        self.qadr, self.vadr = [], []
        for ax in axes:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, ax.joint_name)
            if jid < 0:
                raise RuntimeError(f"URDF không có khớp '{ax.joint_name}'")
            qa = int(self.model.jnt_qposadr[jid])
            va = int(self.model.jnt_dofadr[jid])
            self.qadr.append(qa)
            self.vadr.append(va)

            n = ax.gear_ratio
            self.model.dof_armature[va] = ROTOR_INERTIA_KGM2 * n * n
            self.model.dof_damping[va] = VISCOUS_PER_N2 * n * n
            self.model.dof_frictionloss[va] = DRY_FRICTION_PER_N * n

        self.data = mujoco.MjData(self.model)

    def reset(self, qpos_rad):
        mujoco.mj_resetData(self.model, self.data)
        for qa, v in zip(self.qadr, qpos_rad):
            self.data.qpos[qa] = v
        mujoco.mj_forward(self.model, self.data)

    def joint_state(self, i):
        return float(self.data.qpos[self.qadr[i]]), float(self.data.qvel[self.vadr[i]])

    def step(self, joint_torques_nm):
        self.data.qfrc_applied[:] = 0.0
        for va, tau in zip(self.vadr, joint_torques_nm):
            self.data.qfrc_applied[va] = tau
        mujoco.mj_step(self.model, self.data)

    def healthy(self) -> bool:
        return bool(np.all(np.isfinite(self.data.qpos)) and np.all(np.isfinite(self.data.qvel)))


# ─────────────────────── cascade PID của ODrive ───────────────────────

class OdriveAxis:
    """Bản dựng lại vòng điều khiển của ODrive, làm việc hoàn toàn ở ĐƠN VỊ
    PHÍA ROTOR (rev, rev/s, Nm tại rotor) — đúng như driver thật."""

    def __init__(self, cfg: AxisConfig):
        self.cfg = cfg
        self.state = AXIS_STATE_IDLE
        self.control_mode = 3      # position
        self.input_mode = 1        # passthrough; plugin sẽ đổi sang 3
        self.pos_gain = DEFAULT_POS_GAIN
        self.vel_gain = DEFAULT_VEL_GAIN
        self.vel_integrator_gain = DEFAULT_VEL_INTEGRATOR_GAIN
        self.input_filter_bandwidth = DEFAULT_INPUT_FILTER_BW

        self.input_pos = 0.0       # rev, do 0x00C đặt
        self.input_vel = 0.0
        self.input_torque = 0.0
        self.pos_setpoint = 0.0    # rev, đầu ra bộ lọc
        self.vel_setpoint = 0.0
        self.vel_integrator_torque = 0.0
        self.torque_setpoint = 0.0
        self.torque_measured = 0.0
        self.first_command = True

    # --- quy đổi rotor <-> khớp, đảo đúng phép của gim_arm_system.cpp ---
    # joint_rad ở đây là góc khớp theo URDF (đúng cái MuJoCo mô phỏng).
    # zero_offset_rad được cộng vào để ra "rad thô" mà encoder driver thấy,
    # y như send_position_command() làm trước khi nhân gear_ratio.
    def joint_rad_to_rotor_rev(self, joint_rad: float) -> float:
        raw = joint_rad + self.cfg.zero_offset
        return raw * self.cfg.direction / (2.0 * math.pi) * self.cfg.gear_ratio

    def rotor_rev_to_joint_rad(self, rev: float) -> float:
        raw = rev / self.cfg.gear_ratio * 2.0 * math.pi * self.cfg.direction
        return raw - self.cfg.zero_offset

    def arm(self, joint_rad: float):
        """Vào closed loop: chốt setpoint tại vị trí hiện tại để không giật."""
        here = self.joint_rad_to_rotor_rev(joint_rad)
        self.state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.input_pos = here
        self.pos_setpoint = here
        self.vel_setpoint = 0.0
        self.vel_integrator_torque = 0.0
        self.first_command = True

    def disarm(self):
        self.state = AXIS_STATE_IDLE
        self.vel_integrator_torque = 0.0
        self.torque_setpoint = 0.0
        self.torque_measured = 0.0

    def update(self, joint_rad: float, joint_vel_rad_s: float, dt: float) -> float:
        """Chạy một bước cascade. Trả về mô-men TẠI KHỚP (Nm)."""
        if self.state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            self.torque_setpoint = 0.0
            self.torque_measured = 0.0
            return 0.0

        pos_est = self.joint_rad_to_rotor_rev(joint_rad)
        vel_est = joint_vel_rad_s * self.cfg.direction / (2.0 * math.pi) * self.cfg.gear_ratio

        # 1) bộ lọc setpoint. input_mode 3 = POS_FILTER, bậc 2 tắt dần tới hạn,
        #    hệ số đúng theo update_filter_gains() của ODrive.
        if self.input_mode == 3:
            ki = 2.0 * self.input_filter_bandwidth
            kp = 0.25 * ki * ki
            accel = kp * (self.input_pos - self.pos_setpoint) + ki * (self.input_vel - self.vel_setpoint)
            self.vel_setpoint += dt * accel
            self.pos_setpoint += dt * self.vel_setpoint
        else:
            self.pos_setpoint = self.input_pos
            self.vel_setpoint = self.input_vel

        # 2) P vị trí
        vel_cmd = (self.pos_setpoint - pos_est) * self.pos_gain + self.vel_setpoint
        vel_cmd = max(-VEL_LIMIT_REV_S, min(VEL_LIMIT_REV_S, vel_cmd))

        # 3) PI vận tốc
        vel_err = vel_cmd - vel_est
        torque = self.vel_gain * vel_err + self.vel_integrator_torque + self.input_torque
        self.vel_integrator_torque += self.vel_integrator_gain * dt * vel_err
        self.vel_integrator_torque = max(-ROTOR_TORQUE_LIM_NM,
                                         min(ROTOR_TORQUE_LIM_NM, self.vel_integrator_torque))

        # 4) giới hạn mô-men (thay cho giới hạn dòng của vòng FOC)
        self.torque_setpoint = torque
        torque = max(-ROTOR_TORQUE_LIM_NM, min(ROTOR_TORQUE_LIM_NM, torque))
        self.torque_measured = torque

        # Mô-men đổi hệ bằng CÙNG hệ số như vị trí: rotor_rad = joint_rad *
        # direction * gear_ratio, nên tau_joint = tau_rotor * direction *
        # gear_ratio. Thiếu `direction` là khớp nào có invert_direction=true
        # sẽ bị đẩy ngược, chạy thẳng vào biên rồi bão hoà mô-men.
        return torque * self.cfg.gear_ratio * self.cfg.direction


# ─────────────────────── giả lập 3 driver trên bus ───────────────────────

class GimArmHardwareSim:
    def __init__(self, urdf_path, can_interface=None, idle_reports_zero=True,
                 initial_q=None, gains=None):
        self.axes_cfg = parse_axes_from_urdf(urdf_path)
        self.physics = ArmPhysics(urdf_path, self.axes_cfg)
        self.drivers = [OdriveAxis(c) for c in self.axes_cfg]
        self.idle_reports_zero = idle_reports_zero
        self.bus = CanBus(can_interface) if can_interface else None

        if gains:
            for d in self.drivers:
                d.pos_gain, d.vel_gain, d.vel_integrator_gain = gains

        q0 = initial_q if initial_q is not None else [0.0] * len(self.drivers)
        self.physics.reset(q0)
        self._t_feedback = 0.0
        self._t_heartbeat = 0.0

    # ---------------- nhận lệnh từ bus ----------------
    def handle_frame(self, can_id, dlc, data):
        std_id = can_id & CAN_SFF_MASK
        node_id, cmd_id = (std_id >> 5) & 0x3F, std_id & 0x1F
        idx = next((i for i, c in enumerate(self.axes_cfg) if c.node_id == node_id), None)
        if idx is None:
            return
        drv = self.drivers[idx]
        is_rtr = bool(can_id & CAN_RTR_FLAG)

        if is_rtr:
            self.send_telemetry(idx, only=cmd_id)
            return

        if cmd_id == CMD_SET_AXIS_STATE and dlc >= 4:
            requested = struct.unpack_from("<I", data, 0)[0]
            if requested == AXIS_STATE_CLOSED_LOOP_CONTROL:
                joint_rad, _ = self.physics.joint_state(idx)
                drv.arm(joint_rad)
                print(f"[node {node_id}] -> CLOSED_LOOP tại {joint_rad:+.4f} rad")
            elif requested == AXIS_STATE_IDLE:
                drv.disarm()
                print(f"[node {node_id}] -> IDLE")

        elif cmd_id == CMD_SET_CONTROLLER_MODE and dlc >= 8:
            drv.control_mode, drv.input_mode = struct.unpack_from("<II", data, 0)
            print(f"[node {node_id}] control_mode={drv.control_mode} "
                  f"input_mode={drv.input_mode}")

        elif cmd_id == CMD_SET_INPUT_POS and dlc >= 8:
            pos, vff, tff = struct.unpack_from("<fhh", data, 0)
            drv.input_pos = float(pos)
            drv.input_vel = vff / 1000.0      # int16 thang 0.001 rev/s
            drv.input_torque = tff / 1000.0
            if drv.first_command:
                # Lệnh đầu tiên thường tới sau khi đã vào closed loop: kéo
                # thẳng bộ lọc tới đó, tránh cú nhảy giả không có trên máy thật.
                drv.pos_setpoint = float(pos)
                drv.first_command = False

        elif cmd_id == CMD_SET_INPUT_VEL and dlc >= 8:
            drv.input_vel, drv.input_torque = struct.unpack_from("<ff", data, 0)

        elif cmd_id == CMD_SET_INPUT_TORQUE and dlc >= 4:
            drv.input_torque = struct.unpack_from("<f", data, 0)[0]

        elif cmd_id == CMD_SET_POS_GAIN and dlc >= 4:
            drv.pos_gain = struct.unpack_from("<f", data, 0)[0]
            print(f"[node {node_id}] pos_gain = {drv.pos_gain}")

        elif cmd_id == CMD_SET_VEL_GAINS and dlc >= 8:
            drv.vel_gain, drv.vel_integrator_gain = struct.unpack_from("<ff", data, 0)
            print(f"[node {node_id}] vel_gain = {drv.vel_gain}, "
                  f"vel_integrator_gain = {drv.vel_integrator_gain}")

        elif cmd_id in (CMD_CLEAR_ERRORS, CMD_SAVE_CONFIGURATION):
            pass  # sim không có lỗi và không có flash

        elif cmd_id in (CMD_ESTOP, CMD_REBOOT):
            drv.disarm()
            print(f"[node {node_id}] estop/reboot -> IDLE")

    # ---------------- phát telemetry ----------------
    def send_telemetry(self, idx, only=None):
        if self.bus is None:
            return
        drv, cfg = self.drivers[idx], self.axes_cfg[idx]
        joint_rad, joint_vel = self.physics.joint_state(idx)

        if only in (None, CMD_GET_ENCODER_ESTIMATES):
            if drv.state != AXIS_STATE_CLOSED_LOOP_CONTROL and self.idle_reports_zero:
                pos_rev, vel_rev = 0.0, 0.0
            else:
                pos_rev = drv.joint_rad_to_rotor_rev(joint_rad)
                vel_rev = joint_vel * cfg.direction / (2.0 * math.pi) * cfg.gear_ratio
            self.bus.send(make_can_id(cfg.node_id, CMD_GET_ENCODER_ESTIMATES),
                          struct.pack("<ff", pos_rev, vel_rev))

        if only == CMD_GET_TORQUES:
            self.bus.send(make_can_id(cfg.node_id, CMD_GET_TORQUES),
                          struct.pack("<ff",
                                      drv.torque_setpoint * cfg.gear_ratio * cfg.direction,
                                      drv.torque_measured * cfg.gear_ratio * cfg.direction))
        if only == CMD_GET_IQ:
            iq = drv.torque_measured / TORQUE_CONSTANT_NM_PER_A
            self.bus.send(make_can_id(cfg.node_id, CMD_GET_IQ),
                          struct.pack("<ff", iq, iq))
        if only == CMD_GET_BUS_VOLTAGE_CURRENT:
            self.bus.send(make_can_id(cfg.node_id, CMD_GET_BUS_VOLTAGE_CURRENT),
                          struct.pack("<ff", BUS_VOLTAGE_V, 0.5))

    def send_heartbeat(self):
        if self.bus is None:
            return
        for drv, cfg in zip(self.drivers, self.axes_cfg):
            self.bus.send(make_can_id(cfg.node_id, CMD_HEARTBEAT),
                          struct.pack("<IBBBB", 0, drv.state, 0, 0, 0))

    # ---------------- vòng lặp chính ----------------
    def tick(self, dt):
        if self.bus is not None:
            for frame in self.bus.receive_all():
                self.handle_frame(*frame)

        torques = []
        for i, drv in enumerate(self.drivers):
            joint_rad, joint_vel = self.physics.joint_state(i)
            torques.append(drv.update(joint_rad, joint_vel, dt))
        self.physics.step(torques)

        t = self.physics.data.time
        if t - self._t_feedback >= FEEDBACK_PERIOD_S:
            self._t_feedback = t
            for i in range(len(self.drivers)):
                self.send_telemetry(i)
        if t - self._t_heartbeat >= HEARTBEAT_PERIOD_S:
            self._t_heartbeat = t
            self.send_heartbeat()

    def run(self, use_viewer=False, realtime=1.0):
        dt = self.physics.model.opt.timestep
        print(f"\nĐang giả lập {len(self.drivers)} driver ở {CONTROL_HZ:.0f} Hz. Ctrl-C để dừng.\n")

        viewer_ctx = None
        if use_viewer:
            import mujoco.viewer
            viewer_ctx = mujoco.viewer.launch_passive(self.physics.model, self.physics.data)

        wall0 = time.perf_counter()
        try:
            with viewer_ctx if viewer_ctx else _NullCtx() as viewer:
                last_sync = 0.0
                while True:
                    if viewer is not None and not viewer.is_running():
                        break
                    self.tick(dt)
                    if not self.physics.healthy():
                        print("NaN trong qpos/qvel — gain quá cao hoặc mô hình nổ. Dừng.")
                        break
                    sim_t = self.physics.data.time
                    if viewer is not None and sim_t - last_sync >= 1 / 60:
                        last_sync = sim_t
                        viewer.sync()
                    lag = sim_t / realtime - (time.perf_counter() - wall0)
                    if lag > 0.001:
                        time.sleep(lag)
        except KeyboardInterrupt:
            print("\nĐã dừng.")
        finally:
            for d in self.drivers:
                d.disarm()
            if self.bus:
                self.bus.close()


class _NullCtx:
    def __enter__(self):
        return None

    def __exit__(self, *a):
        return False


# ─────────────────────── self-test không cần CAN ───────────────────────

def selftest(urdf_path, gains, node_id, step_rev, duration, input_mode):
    """Bơm một bước nhảy vị trí thẳng vào driver ảo rồi đo overshoot/settling
    — cùng chỉ số mà kinematics_test/step_response_test.py in ra trên máy
    thật, nên hai bên so được với nhau."""
    sim = GimArmHardwareSim(urdf_path, can_interface=None, gains=gains)
    print("Cấu hình đọc từ URDF:")
    for c in sim.axes_cfg:
        print("  ", c)

    idx = next((i for i, c in enumerate(sim.axes_cfg) if c.node_id == node_id), None)
    if idx is None:
        print(f"Không có node_id {node_id} trong URDF.")
        return 1
    drv = sim.drivers[idx]
    drv.input_mode = input_mode
    print(f"\nKhớp thử: {sim.axes_cfg[idx].joint_name} (node {node_id}), "
          f"input_mode={input_mode}, "
          f"pos_gain={drv.pos_gain} vel_gain={drv.vel_gain} "
          f"vel_integrator_gain={drv.vel_integrator_gain}")

    dt = sim.physics.model.opt.timestep
    for _ in range(int(0.5 / dt)):          # để tay rơi/ổn định dưới trọng lực
        sim.tick(dt)

    for i, d in enumerate(sim.drivers):     # bật cả 3 khớp để có tải thật
        d.arm(sim.physics.joint_state(i)[0])
    for _ in range(int(1.0 / dt)):
        sim.tick(dt)

    start_rev = drv.joint_rad_to_rotor_rev(sim.physics.joint_state(idx)[0])
    target_rev = start_rev + step_rev
    drv.input_pos = target_rev

    ts, pos = [], []
    t0 = sim.physics.data.time
    while sim.physics.data.time - t0 < duration:
        sim.tick(dt)
        ts.append(sim.physics.data.time - t0)
        pos.append(drv.joint_rad_to_rotor_rev(sim.physics.joint_state(idx)[0]))
    ts, pos = np.array(ts), np.array(pos)

    step = target_rev - start_rev
    peak = pos.max() if step > 0 else pos.min()
    overshoot = max(0.0, (peak - target_rev) / step * 100) if step > 0 else \
                max(0.0, (target_rev - peak) / (-step) * 100)
    band = 0.02 * abs(step)
    settled = np.abs(pos - target_rev) <= band
    settling = next((ts[i] for i in range(len(ts)) if settled[i:].all()), None)
    final_err_deg = abs(pos[-1] - target_rev) / sim.axes_cfg[idx].gear_ratio * 360

    print(f"\nBước nhảy {step_rev:+.3f} rev phía rotor "
          f"(= {abs(step_rev) / sim.axes_cfg[idx].gear_ratio * 360:.2f} deg tại khớp)")
    print(f"  overshoot        : {overshoot:.1f} %")
    print(f"  settling (±2%)   : {f'{settling:.3f} s' if settling is not None else 'chưa ổn định'}")
    print(f"  sai số cuối       : {final_err_deg:.3f} deg tại khớp")
    print(f"  vật lý ổn định    : {'có' if sim.physics.healthy() else 'KHÔNG (NaN)'}")
    print("\nNhắc lại: vòng ở đây chạy "
          f"{CONTROL_HZ:.0f} Hz, driver thật 8 kHz — dùng số này để so tương "
          "đối giữa các bộ gain, đừng bê thẳng xuống phần cứng.")
    return 0


# ─────────────────────── CLI ───────────────────────

def default_urdf():
    for cand in ("src/gim_arm_description/urdf/gim_arm.urdf",
                 "../gim_arm_description/urdf/gim_arm.urdf",
                 "gim_arm.urdf"):
        if os.path.isfile(cand):
            return cand
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("gim_arm_description"),
                            "urdf", "gim_arm.urdf")
    except Exception:
        return "gim_arm.urdf"


def main():
    p = argparse.ArgumentParser(
        description="Giả lập 3 driver GIM6010-8 + bus CAN, động lực học bằng MuJoCo.")
    p.add_argument("--urdf", default=default_urdf(), help="đường dẫn gim_arm.urdf")
    p.add_argument("--can", default=None,
                   help="tên interface CAN, vd can0 hoặc vcan0. Bỏ trống = không mở CAN.")
    p.add_argument("--viewer", action="store_true", help="mở cửa sổ MuJoCo")
    p.add_argument("--realtime", type=float, default=1.0,
                   help="hệ số tốc độ, 1.0 = thời gian thực, 0.25 = chậm 4 lần")
    p.add_argument("--no-idle-zero", action="store_true",
                   help="phát vị trí thật cả khi IDLE (máy thật phát 0)")
    p.add_argument("--pos-gain", type=float, default=DEFAULT_POS_GAIN)
    p.add_argument("--vel-gain", type=float, default=DEFAULT_VEL_GAIN)
    p.add_argument("--vi-gain", type=float, default=DEFAULT_VEL_INTEGRATOR_GAIN)
    p.add_argument("--selftest", action="store_true", help="chạy step response, không cần CAN")
    p.add_argument("--node", type=int, default=0, help="node_id để thử step response")
    p.add_argument("--step", type=float, default=0.5, help="bước nhảy, rev phía rotor")
    p.add_argument("--duration", type=float, default=2.0, help="thời gian đo, giây")
    p.add_argument("--input-mode", type=int, default=1,
                   help="1 = passthrough (thấy đáp ứng thật), 3 = POS_FILTER như plugin dùng")
    args = p.parse_args()

    if not os.path.isfile(args.urdf):
        print(f"Không thấy URDF: {args.urdf}\nDùng --urdf để chỉ đúng đường dẫn.")
        return 1

    gains = (args.pos_gain, args.vel_gain, args.vi_gain)
    if args.selftest:
        return selftest(args.urdf, gains, args.node, args.step,
                        args.duration, args.input_mode)

    if args.can is None:
        print("Thiếu --can. Ví dụ: --can can0. Hoặc dùng --selftest để thử offline.")
        return 1

    try:
        sim = GimArmHardwareSim(args.urdf, args.can,
                                idle_reports_zero=not args.no_idle_zero,
                                gains=gains)
    except OSError as e:
        print(f"Không mở được interface CAN '{args.can}': {e}\n"
              f"Dựng CAN ảo:\n"
              f"  sudo modprobe vcan\n"
              f"  sudo ip link add dev {args.can} type vcan\n"
              f"  sudo ip link set up {args.can}")
        return 1

    print(f"Đã mở CAN '{args.can}'. Cấu hình đọc từ URDF:")
    for c in sim.axes_cfg:
        print("  ", c)
    sim.run(use_viewer=args.viewer, realtime=args.realtime)
    return 0


if __name__ == "__main__":
    sys.exit(main())