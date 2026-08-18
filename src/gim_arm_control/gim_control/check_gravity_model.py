#!/usr/bin/env python3
"""
check_gravity_model.py — so MÔ-MEN GIỮ THẬT của tay máy với G(q) mà mô hình
Lagrange dự đoán, tại đúng tư thế tay đang đứng.

Đây là phép kiểm quan trọng nhất trước khi bật gravity_feedforward trên tay
thật, vì nó kiểm CÙNG LÚC bốn thứ mà không thứ nào tự lộ ra khi chạy bình
thường:

  1) DẤU mô-men. Sai dấu thì feedforward đẩy ngược, chống lại chính vòng vị
     trí -- tay vẫn chạy đúng (vòng vị trí thắng) nhưng ăn hết dự trữ mô-men.
  2) ĐIỂM 0 CỦA ENCODER. G(q) tính từ góc khớp trong hệ URDF. Nếu điểm 0 vật
     lý của encoder không trùng tư thế 0 của URDF thì G(q) đang tính cho MỘT
     TƯ THẾ KHÁC. Cả 3 khớp đang để zero_offset_rad = 0 -- chưa hiệu chỉnh.
  3) TỈ SỐ TRUYỀN. Lệch 8 lần ở shoulder lộ ra ngay ở cột tỉ lệ.
  4) BẢN THÂN MÔ HÌNH: khối lượng/tâm khối trong URDF có đúng tay thật không.

Cách chạy: cho tay ĐỨNG YÊN ở một tư thế (ros2_control đang giữ vị trí, KHÔNG
chạy quỹ đạo), rồi:

    ros2 run gim_control check_gravity_model can0
    python3 check_gravity_model.py can0 --urdf /duong/dan/gim_arm.urdf

Làm ở 3-4 tư thế khác nhau, càng khác nhau càng tốt (vươn ra trước tải nặng,
co lại tải nhẹ). Một tư thế trùng khớp có thể là ăn may; ba tư thế trùng thì
mô hình đúng.

LƯU Ý VỀ ĐƠN VỊ: ODrive trả Get_Torques ở PHÍA ROTOR, nhưng bản giả lập trong
repo lại quy sẵn về phía khớp. Không chắc firmware GIM6010-8 theo quy ước nào,
nên công cụ này in CẢ HAI cách hiểu và để phép so với G(q) tự chỉ ra cách nào
đúng -- cột nào có tỉ lệ ~1.0 thì đó là quy ước của firmware bạn.
"""

import argparse
import os
import socket
import struct
import sys
import time

import numpy as np

CMD_GET_ENCODER_ESTIMATES = 0x009
CMD_GET_TORQUES = 0x01C
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_SFF_MASK = 0x7FF
CAN_RTR_FLAG = 0x40000000
TWO_PI = 6.283185307179586


def load_helpers(urdf_path):
    """parse_axes_from_urdf từ bản giả lập, ArmDynamics từ kinematics_test --
    dùng lại chứ không chép, để không lệch nguồn sự thật."""
    import importlib.util

    here = os.path.dirname(os.path.abspath(__file__))

    def load(name, path):
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
        return mod

    try:
        from gim_control.gim6010_mujoco_sim import parse_axes_from_urdf
    except ImportError:
        parse_axes_from_urdf = load(
            "_gim_sim", os.path.join(here, "gim6010_mujoco_sim.py")).parse_axes_from_urdf

    # ArmDynamics nằm ở kinematics_test/ (ngoài package ROS)
    cand = [os.path.join(here, "arm_dynamics.py"),
            os.path.join(here, "../../../kinematics_test/arm_dynamics.py")]
    for p in cand:
        if os.path.exists(p):
            return parse_axes_from_urdf, load("_arm_dyn", os.path.abspath(p)).ArmDynamics
    raise RuntimeError(
        "Không tìm thấy arm_dynamics.py. Chạy từ thư mục kinematics_test, hoặc "
        "chép arm_dynamics.py vào gim_control/.")


def default_urdf():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("gim_arm_description"),
                            "urdf", "gim_arm.urdf")
    except Exception:
        return "gim_arm.urdf"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("interface", nargs="?", default="can0")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--samples", type=int, default=30,
                    help="số mẫu lấy trung bình (mặc định 30, ~3 giây)")
    args = ap.parse_args()

    urdf = args.urdf or default_urdf()
    parse_axes_from_urdf, ArmDynamics = load_helpers(urdf)
    axes = parse_axes_from_urdf(urdf)
    by_node = {ax.node_id: ax for ax in axes}
    dyn = ArmDynamics(urdf)

    # Thứ tự khớp của mô hình (Pinocchio) -> node_id tương ứng
    order = []
    for name in dyn.joint_names:
        match = [ax for ax in axes if ax.joint_name == name]
        if not match:
            print(f"URDF: khớp '{name}' không có trong khối <ros2_control>.")
            return 1
        order.append(match[0])

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((args.interface,))
    except OSError as e:
        print(f"Không mở được '{args.interface}': {e}")
        return 1
    sock.settimeout(0.5)

    def request(node_id, cmd):
        can_id = ((node_id << 5) | cmd) | CAN_RTR_FLAG
        sock.send(struct.pack(CAN_FRAME_FMT, can_id, 0, b"\x00" * 8))

    pos = {ax.node_id: [] for ax in axes}
    trq = {ax.node_id: [] for ax in axes}

    print(f"Đang đo trên {args.interface}, {args.samples} mẫu. "
          f"GIỮ TAY ĐỨNG YÊN, đừng chạy quỹ đạo.\n")
    deadline = time.time() + args.samples * 0.1 + 5.0
    while min(len(v) for v in trq.values()) < args.samples and time.time() < deadline:
        for ax in axes:
            request(ax.node_id, CMD_GET_TORQUES)
        t_end = time.time() + 0.1
        while time.time() < t_end:
            try:
                raw = sock.recv(CAN_FRAME_SIZE)
            except socket.timeout:
                break
            if len(raw) < CAN_FRAME_SIZE:
                continue
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
            if can_id & CAN_RTR_FLAG or dlc < 8:
                continue
            std_id = can_id & CAN_SFF_MASK
            node_id, cmd = (std_id >> 5) & 0x3F, std_id & 0x1F
            if node_id not in by_node:
                continue
            if cmd == CMD_GET_ENCODER_ESTIMATES:
                pos[node_id].append(struct.unpack_from("<ff", data, 0)[0])
            elif cmd == CMD_GET_TORQUES:
                trq[node_id].append(struct.unpack_from("<ff", data, 0)[1])
    sock.close()

    missing = [by_node[n].joint_name for n, v in trq.items() if len(v) < 3]
    if missing:
        print(f"Không nhận đủ Get_Torques (0x01C) từ: {', '.join(missing)}")
        print("Driver có thể không trả lời RTR cho lệnh này. Thay bằng cách đọc")
        print("Get_Iq (0x014) rồi nhân torque_constant, hoặc đọc bằng odrivetool.")
        return 1

    q = np.array([
        np.mean(pos[ax.node_id]) / ax.gear_ratio * TWO_PI * ax.direction - ax.zero_offset
        if pos[ax.node_id] else 0.0
        for ax in order
    ])
    tau_meas_raw = np.array([np.mean(trq[ax.node_id]) for ax in order])
    gear_dir = np.array([ax.gear_ratio * ax.direction for ax in order])
    g_pred = dyn.gravity(q)

    print(f"Tư thế đo (rad):  {q.round(4)}")
    if not any(pos.values()):
        print("  (CẢNH BÁO: không bắt được frame encoder 0x009, q có thể sai)")
    print()
    print(f"{'khớp':<16}{'G(q) dự đoán':>14}{'đo được (thô)':>15}"
          f"{'× gear·dir':>13}{'tỉ lệ thô':>11}{'tỉ lệ ×gear':>13}")
    print("-" * 82)
    for i, ax in enumerate(order):
        raw = tau_meas_raw[i]
        scaled = raw * gear_dir[i]
        r1 = raw / g_pred[i] if abs(g_pred[i]) > 1e-3 else float("nan")
        r2 = scaled / g_pred[i] if abs(g_pred[i]) > 1e-3 else float("nan")
        print(f"{ax.joint_name:<16}{g_pred[i]:14.4f}{raw:15.4f}{scaled:13.4f}"
              f"{r1:11.2f}{r2:13.2f}")
    print("-" * 82)
    print("Đọc bảng:")
    print("  - Cột tỉ lệ nào gần +1.0 -> đó là quy ước đơn vị của firmware, và")
    print("    mô hình ĐÚNG ở tư thế này. Lặp lại ở 2-3 tư thế khác để chắc.")
    print("  - Tỉ lệ gần -1.0  -> mô hình đúng nhưng NGƯỢC DẤU: đổi dấu")
    print("    torque_ff_rotor trong send_position_command() của gim_arm_system.cpp.")
    print("  - Tỉ lệ lệch xa 1 và KHÁC NHAU giữa các khớp -> nhiều khả năng điểm 0")
    print("    encoder chưa hiệu chỉnh (zero_offset_rad đang = 0 cho cả 3 khớp),")
    print("    nên G(q) đang được tính cho một tư thế khác tư thế thật.")
    print("  - Khớp nào |G(q)| rất nhỏ thì tỉ lệ vô nghĩa; đổi tư thế để khớp đó")
    print("    chịu tải rồi đo lại.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
