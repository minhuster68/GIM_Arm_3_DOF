#!/usr/bin/env python3
"""
sniff_can_ff.py — nghe bus CAN và giải mã frame Set_Input_Pos (0x00C) để XEM
TẬN MẮT hai trường feedforward có thật sự đi ra dây hay không, và với độ lớn /
dấu nào.

Vì sao cần: `vel_ff` và `torque_ff` nằm ở byte 4..7 của frame, dạng int16 thang
0.001 PHÍA ROTOR. Nhìn candump thô thì chỉ thấy 8 byte hex, không biết quy về
khớp là bao nhiêu Nm. Công cụ này quy đổi ngược về ĐƠN VỊ KHỚP (rad, rad/s, Nm)
bằng đúng gear_ratio / direction / zero_offset đọc từ URDF, nên số in ra so
thẳng được với /joint_states và với G(q) mà mô hình tính.

Chạy được cả 2 kiểu:
    ros2 run gim_control sniff_can_ff can0
    python3 sniff_can_ff.py can0 --urdf /duong/dan/gim_arm.urdf

Dùng cho 3 việc trong lúc bring-up:
  1) Xác nhận feedforward ĐÃ BẬT: torque_ff/vel_ff khác 0 (nếu vẫn 0 thì tham
     số trong URDF chưa có tác dụng, hoặc controller chưa claim interface
     velocity).
  2) Xác nhận ĐỘ LỚN hợp lý: mô-men quy về khớp phải xấp xỉ G(q) mà
     kinematics_test/arm_dynamics.py in ra ở cùng tư thế.
  3) Xác nhận KHÔNG BỊ KẸP TRẦN: cột "kẹp" phải là 0%. Nếu khác 0 nghĩa là
     max_torque_ff_rotor_nm quá chặt hoặc mô hình đang tính ra số vô lý.
"""

import argparse
import os
import socket
import struct
import sys
from collections import defaultdict

CMD_SET_INPUT_POS = 0x00C
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_SFF_MASK = 0x7FF
TWO_PI = 6.283185307179586


def load_axes(urdf_path):
    """Dùng lại parse_axes_from_urdf của gim6010_mujoco_sim -- cùng một nguồn
    sự thật với plugin C++, không chép lại logic đọc URDF."""
    try:
        from gim_control.gim6010_mujoco_sim import parse_axes_from_urdf
    except ImportError:
        import importlib.util
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "gim6010_mujoco_sim.py")
        spec = importlib.util.spec_from_file_location("_gim_sim", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        parse_axes_from_urdf = mod.parse_axes_from_urdf
    return {ax.node_id: ax for ax in parse_axes_from_urdf(urdf_path)}


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
    ap.add_argument("--every", type=int, default=100,
                    help="in 1 dòng mỗi N frame của mỗi khớp (mặc định 100 "
                         "-> khoảng 1 dòng/giây khi controller chạy 100 Hz)")
    ap.add_argument("--torque-limit", type=float, default=0.625,
                    help="trần torque_ff phía rotor đang đặt trong URDF, chỉ "
                         "dùng để đếm % bị kẹp")
    args = ap.parse_args()

    axes = load_axes(args.urdf or default_urdf())
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((args.interface,))
    except OSError as e:
        print(f"Không mở được '{args.interface}': {e}")
        print("  bus thật:  sudo ip link set can0 up type can bitrate 500000")
        print("  bus ảo  :  sudo modprobe vcan && sudo ip link add dev can0 type vcan"
              " && sudo ip link set up can0")
        return 1

    print(f"Đang nghe {args.interface}, giải mã Set_Input_Pos (0x00C). Ctrl-C để dừng.\n")
    print(f"{'khớp':<16}{'vị trí (rad)':>14}{'vel_ff (rad/s)':>16}"
          f"{'torque_ff (Nm)':>16}{'= rotor (Nm)':>14}{'kẹp trần':>10}")
    print("-" * 86)

    count = defaultdict(int)
    clipped = defaultdict(int)
    peak = defaultdict(float)
    try:
        while True:
            raw = sock.recv(CAN_FRAME_SIZE)
            if len(raw) < CAN_FRAME_SIZE:
                continue
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
            std_id = can_id & CAN_SFF_MASK
            if (std_id & 0x1F) != CMD_SET_INPUT_POS or dlc < 8:
                continue
            node_id = (std_id >> 5) & 0x3F
            ax = axes.get(node_id)
            if ax is None:
                continue

            pos_rev, vel_i, trq_i = struct.unpack_from("<fhh", data, 0)
            vel_rotor = vel_i / 1000.0        # rev/s phía rotor
            trq_rotor = trq_i / 1000.0        # Nm phía rotor

            # Quy về khớp bằng ĐÚNG phép của gim_arm_system.cpp (đảo ngược lại)
            pos_joint = pos_rev / ax.gear_ratio * TWO_PI * ax.direction - ax.zero_offset
            vel_joint = vel_rotor / ax.gear_ratio * TWO_PI * ax.direction
            trq_joint = trq_rotor * ax.gear_ratio * ax.direction

            count[node_id] += 1
            peak[node_id] = max(peak[node_id], abs(trq_joint))
            if abs(trq_rotor) >= 0.999 * args.torque_limit:
                clipped[node_id] += 1

            if count[node_id] % args.every == 0:
                pct = 100.0 * clipped[node_id] / count[node_id]
                print(f"{ax.joint_name:<16}{pos_joint:14.4f}{vel_joint:16.4f}"
                      f"{trq_joint:16.4f}{trq_rotor:14.4f}{pct:9.1f}%")
    except KeyboardInterrupt:
        print("\n\nTổng kết:")
        for node_id, n in sorted(count.items()):
            ax = axes[node_id]
            pct = 100.0 * clipped[node_id] / n
            print(f"  {ax.joint_name:<16} {n:6d} frame | torque_ff lớn nhất "
                  f"{peak[node_id]:7.4f} Nm tại khớp | kẹp trần {pct:.1f}%")
        if all(p == 0.0 for p in peak.values()):
            print("\n  CẢNH BÁO: torque_ff = 0 ở MỌI khớp -> bù trọng lực CHƯA có tác dụng.")
            print("  Kiểm: <param name=\"gravity_feedforward\">true</param> trong URDF,")
            print("  và đã colcon build + source lại chưa.")
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
