#!/usr/bin/env python3
"""
read_joints.py — đọc góc khớp trực tiếp từ CAN. KHÔNG cần ROS.

    python3 read_joints.py --can can0

Vì sao cần: driver GIM6010-8 chỉ điền dữ liệu thật vào Get_Encoder_Estimates
(0x009) SAU khi vào closed loop; ở IDLE nó phát 0. Nên muốn đọc góc trong lúc
làm phép thử mô-men (ros2_control đã tắt) thì phải đọc CAN thô.

Quy đổi lấy đúng công thức trong read() của gim_arm_system.cpp:

    q = (pos_rev / gear_ratio) * 2π * direction − zero_offset

Ba con số dưới đây phải khớp <ros2_control> trong gim_arm.urdf. Sửa URDF thì
sửa cả đây, nếu không hai bên báo hai góc khác nhau và bạn sẽ đuổi một con bug
không tồn tại.
"""

import argparse
import math
import socket
import struct
import sys
import time

# (tên, node_id, gear_ratio, direction, zero_offset_rad) -- khớp gim_arm.urdf
JOINTS = [
    ("base_joint",     0,  8.0, -1.0, 0.0),   # invert_direction = true
    ("shoulder_joint", 1, 64.0, +1.0, 0.0),   # gear_ratio = 64.0
    ("elbow_joint",    2,  8.0, -1.0, 0.0),   # invert_direction = true
]

CMD_HEARTBEAT = 0x001
CMD_GET_ENCODER = 0x009
CMD_GET_TORQUES = 0x01C
CAN_RTR_FLAG = 0x40000000

AXIS_STATES = {0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQ", 3: "FULL_CALIB",
               4: "MOTOR_CALIB", 6: "ENCODER_INDEX", 7: "ENCODER_OFFSET",
               8: "CLOSED_LOOP", 9: "LOCKIN", 10: "ENCODER_DIR_FIND",
               11: "HOMING", 12: "ENCODER_HALL_POL", 13: "ENCODER_HALL_PHASE"}


def open_can(ifname):
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        s.bind((ifname,))
    except OSError as e:
        sys.exit(f"Không mở được '{ifname}': {e}\n"
                 f"Kiểm: ip link show {ifname}   (phải thấy state UP)")
    s.settimeout(0.5)
    return s


def read_state(sock, duration=0.5):
    """Rút frame trong `duration` giây, trả về góc khớp + mô-men + trạng thái."""
    pos, vel, state, err = {}, {}, {}, {}
    trq_set, trq_meas = {}, {}

    # Hỏi Get_Torques bằng frame RTR. Driver không phát tuần hoàn lệnh này.
    for _, node, _, _, _ in JOINTS:
        cid = ((node << 5) | CMD_GET_TORQUES) | CAN_RTR_FLAG
        try:
            sock.send(struct.pack("<IBBBB", cid, 8, 0, 0, 0) + b"\x00" * 8)
        except OSError:
            pass

    t_end = time.time() + duration
    while time.time() < t_end:
        try:
            frame = sock.recv(16)
        except socket.timeout:
            break
        can_id_full, dlc = struct.unpack_from("<IB", frame, 0)
        # SocketCAN bật loopback mặc định -> chính frame RTR mình vừa gửi cũng
        # quay lại, với 8 byte 0. Không lọc là đọc nhầm thành mô-men bằng 0.
        if can_id_full & CAN_RTR_FLAG:
            continue
        can_id = can_id_full & 0x7FF
        data = frame[8:8 + dlc]
        node, cmd = can_id >> 5, can_id & 0x1F
        if cmd == CMD_GET_ENCODER and dlc >= 8:
            pos[node], vel[node] = struct.unpack_from("<ff", data, 0)
        elif cmd == CMD_GET_TORQUES and dlc >= 8:
            trq_set[node], trq_meas[node] = struct.unpack_from("<ff", data, 0)
        elif cmd == CMD_HEARTBEAT and dlc >= 5:
            err[node] = struct.unpack_from("<I", data, 0)[0]
            state[node] = data[4]

    out = []
    for name, node, gear, direction, off in JOINTS:
        if node in pos:
            q = (pos[node] / gear) * 2.0 * math.pi * direction - off
            qd = (vel[node] / gear) * 2.0 * math.pi * direction
        else:
            q = qd = float("nan")
        out.append(dict(name=name, node=node, q=q, qd=qd,
                        pos_rev=pos.get(node, float("nan")),
                        t_set=trq_set.get(node, float("nan")),
                        t_meas=trq_meas.get(node, float("nan")),
                        gear=gear,
                        state=state.get(node), err=err.get(node)))
    return out


def fmt(rows):
    lines = [f"{'khớp':<16}{'q (rad)':>10}{'q (độ)':>9}{'q̇':>9}"
             f"{'τ đặt':>10}{'τ đo':>10}{'% ĐM':>7}{'τ khớp':>10}"
             f"{'trạng thái':>14}{'lỗi':>9}"]
    for r in rows:
        st = AXIS_STATES.get(r["state"], "—" if r["state"] is None else str(r["state"]))
        e = "—" if r["err"] is None else (f"0x{r['err']:X}" if r["err"] else "0")
        tm = r["t_meas"]
        pct = abs(tm) / 0.625 * 100 if tm == tm else float("nan")
        lines.append(f"{r['name']:<16}{r['q']:>10.4f}{math.degrees(r['q']):>9.2f}"
                     f"{r['qd']:>9.4f}{r['t_set']:>10.4f}{tm:>10.4f}{pct:>6.0f}%"
                     f"{tm * r['gear']:>10.3f}{st:>14}{e:>9}")
    lines.append("")
    lines.append("  τ đặt / τ đo: Nm PHÍA ROTOR, đọc thẳng từ Get_Torques (0x01C).")
    lines.append("  % ĐM: so với 0.625 Nm định mức phía rotor.")
    lines.append("  τ khớp: τ đo × gear_ratio (chưa nhân direction).")
    lines.append("  Ở trạng thái IDLE, động cơ KHÔNG có dòng -> τ đo phải ≈ 0.")
    lines.append("  Nếu IDLE mà τ đo vẫn lớn thì trường này KHÔNG phải mô-men đo được.")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--can", default="can0")
    ap.add_argument("--once", action="store_true", help="in 1 lần rồi thoát")
    args = ap.parse_args()

    sock = open_can(args.can)
    if args.once:
        print(fmt(read_state(sock, 0.6)))
        return

    print("Ctrl-C để dừng.\n")
    try:
        while True:
            rows = read_state(sock, 0.3)
            print("\033[H\033[J" + fmt(rows), flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()