#!/usr/bin/env python3
"""
set_zero.py — BUÔNG cả 3 khớp ở chế độ mô-men 0 Nm rồi đọc encoder. KHÔNG cần ROS.

    # TẮT ros2_control trước (Ctrl-C launch), rồi:
    python3 tools/set_zero.py --can can0

===========================================================================
VÌ SAO PHẢI CÓ CÔNG CỤ RIÊNG
===========================================================================
Ba cách "hiển nhiên" đều KHÔNG dùng được:

  1) Tắt động cơ (IDLE) rồi đọc encoder.
     KHÔNG được: driver GIM6010-8 chỉ điền dữ liệu thật vào
     Get_Encoder_Estimates (0x009) SAU khi vào CLOSED_LOOP; ở IDLE nó phát 0.

  2) Chạy lqi_node với gravity_scale = [0,0,0] để nó phát 0 Nm.
     KHÔNG được: gim_arm_system.cpp command_stale() so BIT-IDENTICAL. Phát 0.0
     không đổi 51 chu kỳ (0.51 s) là plugin coi nguồn phát đã chết và TỤT VỀ BÙ
     TRỌNG LỰC -> tay bị giữ lên, đúng cái ta không muốn.

  3) torque_sign_test.py --tau 0.
     Gần được, nhưng nó chạy MỘT khớp mỗi lần và có chốt hành trình 0.25 rad.
     Tư thế nghỉ là cân bằng của CẢ CÁNH TAY: buông từng khớp một cho ra tư thế
     khác với buông cả ba cùng lúc.

Nên script này nói CAN trực tiếp, đặt CẢ BA node vào CLOSED_LOOP + control_mode
= 1 (mô-men) + Set_Input_Torque = 0, chờ tay lắng, rồi đọc.

===========================================================================
AN TOÀN
===========================================================================
  . Khối `finally` LUÔN chạy (kể cả Ctrl-C, kể cả exception): mô-men 0 -> IDLE
    -> control_mode về 3 (vị trí) cho cả 3 node. Đây là điểm khác quan trọng
    nhất so với gõ cansend bằng tay.
  . TAY SẼ RƠI ngay khi vào chế độ này. Phải tháo khỏi người, đặt lên bàn có kê
    đỡ, và ĐỠ TAY BẰNG TAY trong lúc chạy.
  . --max-seconds ngắt sau một khoảng cố định dù có lắng hay không.

===========================================================================
CON SỐ IN RA DÙNG THẾ NÀO
===========================================================================
q_raw in ra là góc khớp trong không gian URDF với zero_offset_rad = 0, tức
    q_raw = (pos_rev / gear_ratio) * 2π * direction

Nó CHƯA phải giá trị điền vào <param name="zero_offset_rad">, TRỪ KHI bạn chấp
nhận định nghĩa "q = 0 là tư thế nghỉ vật lý". Nhưng q = 0 trong gim_arm.urdf
(từ 27/08/2026) là tư thế HÌNH HỌC thẳng đứng, KHÁC tư thế thế năng nhỏ nhất.
Xem RUNBOOK mục 2.0 để biết cách bù chênh đó bằng một phép đo góc nghiêng link.
"""

import argparse
import math
import socket
import struct
import sys
import time

# Phải khớp <ros2_control> trong gim_arm.urdf. Sửa URDF thì sửa cả đây.
JOINTS = [
    ("base_joint",     0,  8.0, -1.0),   # invert_direction = true
    ("shoulder_joint", 1, 64.0, +1.0),   # gear_ratio = 64.0
    ("elbow_joint",    2,  8.0, -1.0),   # invert_direction = true
]

CMD = dict(heartbeat=0x001, set_axis_state=0x007, get_encoder=0x009,
           set_controller_mode=0x00B, set_input_torque=0x00E, clear_errors=0x018)
AXIS_IDLE, AXIS_CLOSED_LOOP = 1, 8
MODE_TORQUE, MODE_POSITION = 1, 3


class Bus:
    def __init__(self, ifname):
        self.s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.s.bind((ifname,))
        except OSError as e:
            sys.exit(f"Không mở được '{ifname}': {e}\n"
                     f"Kiểm: ip link show {ifname}  (phải thấy state UP)")
        self.s.settimeout(0.05)

    def send(self, node, cmd, data=b""):
        can_id = (node << 5) | cmd
        payload = data.ljust(8, b"\x00")
        self.s.send(struct.pack("<IBBBB", can_id, 8, 0, 0, 0) + payload)

    def drain(self, duration=0.1):
        """Rút mọi frame trong `duration` giây. Trả về dict node -> (pos,vel,state,err)."""
        got = {}
        t_end = time.time() + duration
        while time.time() < t_end:
            try:
                frame = self.s.recv(16)
            except socket.timeout:
                continue
            cid, dlc = struct.unpack_from("<IB", frame, 0)
            cid &= 0x7FF
            node, c = (cid >> 5), cid & 0x1F
            d = frame[8:8 + dlc]
            cur = got.setdefault(node, [None, None, None, None])
            if c == CMD["get_encoder"] and dlc >= 8:
                cur[0], cur[1] = struct.unpack_from("<ff", d, 0)
            elif c == CMD["heartbeat"] and dlc >= 5:
                cur[3] = struct.unpack_from("<I", d, 0)[0]
                cur[2] = d[4]
        return got


def to_joint(pos_rev, gear, direction):
    return float("nan") if pos_rev is None else (pos_rev / gear) * 2.0 * math.pi * direction


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--can", default="can0")
    ap.add_argument("--settle-vel", type=float, default=0.01,
                    help="ngưỡng |q̇| coi là đã lắng, rad/s phía khớp")
    ap.add_argument("--settle-hold", type=float, default=1.5,
                    help="phải dưới ngưỡng liên tục bao nhiêu giây")
    ap.add_argument("--max-seconds", type=float, default=25.0,
                    help="ngắt sau khoảng này dù chưa lắng")
    ap.add_argument("--yes", action="store_true", help="bỏ bước hỏi xác nhận")
    args = ap.parse_args()

    print(__doc__.split("===")[0].strip())
    print()
    print("!! TAY SẼ RƠI ngay khi vào chế độ mô-men 0 Nm.")
    print("!! Tháo khỏi người, đặt lên bàn có kê đỡ, ĐỠ TAY BẰNG TAY.")
    print("!! ros2_control phải ĐANG TẮT (Ctrl-C cái launch).")
    if not args.yes:
        if input("\nĐã đúng cả 3 điều trên? gõ 'ok' để chạy: ").strip().lower() != "ok":
            sys.exit("Huỷ.")

    bus = Bus(args.can)
    nodes = [j[1] for j in JOINTS]
    try:
        for _, node, _, _ in JOINTS:
            bus.send(node, CMD["clear_errors"])
            time.sleep(0.03)
            bus.send(node, CMD["set_controller_mode"], struct.pack("<II", MODE_TORQUE, 1))
            time.sleep(0.03)
            bus.send(node, CMD["set_axis_state"], struct.pack("<II", AXIS_CLOSED_LOOP, 0))
            time.sleep(0.05)
        time.sleep(0.4)

        got = bus.drain(0.5)
        missing = [n for n in nodes if n not in got or got[n][0] is None]
        if missing:
            print(f"  !! Không nhận Get_Encoder_Estimates từ node {missing}. "
                  f"Bus đúng chưa? node_id đúng chưa?")
            return 1
        bad = [(n, got[n][2], got[n][3]) for n in nodes if got[n][2] != AXIS_CLOSED_LOOP]
        if bad:
            for n, st, er in bad:
                print(f"  !! node {n} không vào CLOSED_LOOP (state={st}, error=0x{er or 0:X})")
            return 1
        print("\n  Cả 3 trục đã CLOSED_LOOP, mô-men 0 Nm. Đang chờ tay lắng...")

        t0 = time.time()
        calm_since = None
        last = {}
        while time.time() - t0 < args.max_seconds:
            for _, node, _, _ in JOINTS:
                bus.send(node, CMD["set_input_torque"], struct.pack("<f", 0.0))
            got = bus.drain(0.08)
            vels, poss = [], {}
            for name, node, gear, direction in JOINTS:
                if node in got and got[node][0] is not None:
                    last[node] = got[node]
                if node not in last:
                    vels = None
                    break
                poss[node] = to_joint(last[node][0], gear, direction)
                vels.append(abs(to_joint(last[node][1], gear, direction)))
            if vels is None:
                continue
            calm = max(vels) < args.settle_vel
            calm_since = (calm_since or time.time()) if calm else None
            print(f"\r  t={time.time()-t0:5.1f}s  q = "
                  + "  ".join(f"{poss[n]:+.4f}" for n in nodes)
                  + f"   |q̇|max = {max(vels):.4f}"
                  + ("  [lắng]" if calm else "        "), end="", flush=True)
            if calm_since and time.time() - calm_since >= args.settle_hold:
                print("\n  -> đã lắng.")
                break
        else:
            print(f"\n  !! Hết {args.max_seconds:g}s mà chưa lắng. Số dưới đây là tư thế "
                  f"lúc ngắt, ĐỪNG dùng làm mốc.")

        print()
        print("  " + "=" * 64)
        print(f"  {'khớp':<16}{'q_raw (rad)':>14}{'q_raw (độ)':>13}{'|q̇| (rad/s)':>14}")
        for name, node, gear, direction in JOINTS:
            q = to_joint(last[node][0], gear, direction)
            qd = to_joint(last[node][1], gear, direction)
            print(f"  {name:<16}{q:>14.5f}{math.degrees(q):>13.2f}{abs(qd):>14.5f}")
        print("  " + "=" * 64)
        print()
        print("  q_raw CHƯA phải zero_offset_rad. Xem RUNBOOK mục 2.0:")
        print("  tư thế nghỉ vật lý KHÁC tư thế q=0 hình học của gim_arm.urdf,")
        print("  phải bù chênh bằng một phép đo góc nghiêng link.")
        return 0

    finally:
        # LUÔN chạy: kể cả Ctrl-C, kể cả exception.
        for _, node, _, _ in JOINTS:
            bus.send(node, CMD["set_input_torque"], struct.pack("<f", 0.0))
            time.sleep(0.02)
            bus.send(node, CMD["set_axis_state"], struct.pack("<II", AXIS_IDLE, 0))
            time.sleep(0.02)
            bus.send(node, CMD["set_controller_mode"], struct.pack("<II", MODE_POSITION, 1))
            time.sleep(0.02)
        print("  [dọn dẹp] cả 3 node: mô-men 0 -> IDLE -> control_mode = 3 (vị trí)")


if __name__ == "__main__":
    sys.exit(main())
