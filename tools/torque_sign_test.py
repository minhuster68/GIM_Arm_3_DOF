#!/usr/bin/env python3
"""
torque_sign_test.py — đo QUY ƯỚC DẤU MÔ-MEN của firmware. KHÔNG cần ROS.

    # TẮT ros2_control trước (Ctrl-C launch), rồi:
    python3 torque_sign_test.py --can can0 --joint base_joint

===========================================================================
VÌ SAO CẦN PHÉP THỬ NÀY
===========================================================================
`invert_direction` KHÔNG phải chỗ chưa chắc chắn -- nó đã nằm đúng trong
    τ_rotor = τ_khớp / (gear_ratio × direction)
Chỗ chưa chắc chắn là: firmware coi Input_Torque DƯƠNG là chiều encoder TĂNG
hay GIẢM. ODrive gốc thì tăng; đây là bản fork.

Ở chế độ vị trí, dấu sai chỉ là nhiễu tải -- vòng P của driver gánh, tay vẫn đi
đúng chiều. Ở chế độ mô-men KHÔNG có vòng nào trong driver, nên dấu sai đảo dấu
cả k_e lẫn k_v cùng lúc:
    đúng:  ë + k_v·ė + k_e·e = 0  ->  cực  −7.5 ± 4.33j   (ổn định)
    sai :  ë − k_v·ė − k_e·e = 0  ->  cực  +18.96, −3.96  (phân kỳ)
Cực dương 18.96 rad/s: sai số gấp 10 lần sau 121 ms. Nó tự khuếch đại.

===========================================================================
SCRIPT NÀY LÀM GÌ ĐỂ AN TOÀN
===========================================================================
1) Mô-men phát ra trong ĐÚNG `--hold` giây (mặc định 0.8 s) rồi tự tắt.
2) Khối `finally` LUÔN chạy: mô-men 0 -> IDLE -> control_mode về 3 (vị trí).
   Kể cả khi bạn Ctrl-C, kể cả khi script gặp lỗi. Đây là điểm khác quan trọng
   nhất so với gõ cansend bằng tay: gõ tay mà quên bước cuối là tay bị buông
   lỏng ở chế độ mô-men, không ai giữ.
3) Chốt hành trình: vượt `--max-travel` rad là ngắt mô-men ngay lập tức.
4) Đòi bạn xác nhận từng khớp, và nhắc nắm chặt link trước khi phát.
"""

import argparse
import math
import socket
import struct
import sys
import time

JOINTS = {
    "base_joint":     dict(node=0, gear=8.0,  direction=-1.0, offset=0.0),
    "shoulder_joint": dict(node=1, gear=64.0, direction=+1.0, offset=0.0),
    "elbow_joint":    dict(node=2, gear=8.0,  direction=-1.0, offset=0.0),
}

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
        self.s.settimeout(0.3)

    def send(self, node, cmd, data=b""):
        can_id = (node << 5) | cmd
        payload = data.ljust(8, b"\x00")
        self.s.send(struct.pack("<IBBBB", can_id, 8, 0, 0, 0) + payload)

    def drain(self, node, duration=0.4):
        """Trả về (pos_rev, vel_rev_s, axis_state, axis_error) mới nhất."""
        pos = vel = st = er = None
        t_end = time.time() + duration
        while time.time() < t_end:
            try:
                frame = self.s.recv(16)
            except socket.timeout:
                break
            cid, dlc = struct.unpack_from("<IB", frame, 0)
            cid &= 0x7FF
            if (cid >> 5) != node:
                continue
            d = frame[8:8 + dlc]
            c = cid & 0x1F
            if c == CMD["get_encoder"] and dlc >= 8:
                pos, vel = struct.unpack_from("<ff", d, 0)
            elif c == CMD["heartbeat"] and dlc >= 5:
                er = struct.unpack_from("<I", d, 0)[0]
                st = d[4]
        return pos, vel, st, er


def to_joint(pos_rev, cfg):
    if pos_rev is None:
        return float("nan")
    return (pos_rev / cfg["gear"]) * 2.0 * math.pi * cfg["direction"] - cfg["offset"]


def run_one(bus, name, cfg, tau_rotor, hold, max_travel):
    node = cfg["node"]
    results = {}
    try:
        bus.send(node, CMD["clear_errors"])
        time.sleep(0.05)
        bus.send(node, CMD["set_controller_mode"],
                 struct.pack("<II", MODE_TORQUE, 1))
        time.sleep(0.05)
        bus.send(node, CMD["set_axis_state"], struct.pack("<II", AXIS_CLOSED_LOOP, 0))
        time.sleep(0.4)

        pos, _, st, er = bus.drain(node, 0.5)
        if pos is None:
            print("  !! Không nhận được Get_Encoder_Estimates. Bus có đúng node_id?")
            return None
        if st != AXIS_CLOSED_LOOP:
            print(f"  !! Trục không vào CLOSED_LOOP (state={st}, error=0x{er or 0:X}). Dừng.")
            return None
        print(f"  Trục đã CLOSED_LOOP. q ban đầu = {to_joint(pos, cfg):+.4f} rad")

        for sign, label in ((+1.0, "DƯƠNG"), (-1.0, "ÂM")):
            input(f"\n  >> NẮM CHẶT link '{name}'. Enter để phát mô-men {label} "
                  f"({sign*tau_rotor:+.3f} Nm rotor = {sign*tau_rotor*cfg['gear']:+.2f} Nm khớp), "
                  f"{hold:g}s: ")
            p0, _, _, _ = bus.drain(node, 0.3)
            q0 = to_joint(p0, cfg)
            payload = struct.pack("<f", sign * tau_rotor)
            t_end = time.time() + hold
            q_last = q0
            tripped = False
            while time.time() < t_end:
                bus.send(node, CMD["set_input_torque"], payload)
                p, _, _, _ = bus.drain(node, 0.05)
                if p is not None:
                    q_last = to_joint(p, cfg)
                    if abs(q_last - q0) > max_travel:
                        tripped = True
                        break
            bus.send(node, CMD["set_input_torque"], struct.pack("<f", 0.0))
            dq = q_last - q0
            results[sign] = dq
            flag = "  [CHỐT HÀNH TRÌNH ngắt sớm]" if tripped else ""
            print(f"     q: {q0:+.4f} -> {q_last:+.4f}   Δq = {dq:+.4f} rad "
                  f"({math.degrees(dq):+.2f}°){flag}")
        return results

    finally:
        # LUÔN chạy: kể cả Ctrl-C, kể cả exception.
        bus.send(node, CMD["set_input_torque"], struct.pack("<f", 0.0))
        time.sleep(0.05)
        bus.send(node, CMD["set_axis_state"], struct.pack("<II", AXIS_IDLE, 0))
        time.sleep(0.05)
        bus.send(node, CMD["set_controller_mode"],
                 struct.pack("<II", MODE_POSITION, 1))
        print(f"  [dọn dẹp] node {node}: mô-men 0 -> IDLE -> control_mode = 3 (vị trí)")


def verdict(name, res):
    if not res or +1.0 not in res or -1.0 not in res:
        return f"{name:<16} KHÔNG KẾT LUẬN ĐƯỢC"
    dp, dm = res[+1.0], res[-1.0]
    if abs(dp) < 0.01 and abs(dm) < 0.01:
        return (f"{name:<16} KHÔNG NHÚC NHÍCH (|Δq| < 0.01 rad cả hai chiều)\n"
                f"{'':<16}   -> tăng --tau, hoặc khớp đang bị kẹp/chạm cữ")
    if dp > 0.01 and dm < -0.01:
        return f"{name:<16} THUẬN   -> giữ torque_sign = 1"
    if dp < -0.01 and dm > 0.01:
        return f"{name:<16} NGƯỢC   -> đổi torque_sign = -1 trong gim_arm.urdf"
    return (f"{name:<16} MƠ HỒ (Δq+ = {dp:+.4f}, Δq− = {dm:+.4f})\n"
            f"{'':<16}   -> hai chiều không đối xứng: trọng lực hoặc ma sát đang trội.\n"
            f"{'':<16}      Đưa khớp về tư thế mà trọng lực nhỏ rồi thử lại, "
            f"hoặc tăng --tau.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--can", default="can0")
    ap.add_argument("--joint", action="append", choices=list(JOINTS),
                    help="lặp lại để thử nhiều khớp. Bỏ trống = cả 3.")
    ap.add_argument("--tau", type=float, default=0.15,
                    help="Nm PHÍA ROTOR. 0.15 thắng được trọng lực đỉnh "
                         "(0.148 base / 0.064 shoulder / 0.190 elbow).")
    ap.add_argument("--hold", type=float, default=0.8, help="giây")
    ap.add_argument("--max-travel", type=float, default=0.25,
                    help="rad; vượt là ngắt mô-men ngay")
    args = ap.parse_args()

    names = args.joint or list(JOINTS)
    if args.tau > 0.4:
        sys.exit(f"--tau {args.tau} quá lớn. Trần rotor định mức là 0.625 Nm; "
                 "phép thử này không cần quá 0.4.")

    print("=" * 74)
    print("ĐO DẤU MÔ-MEN. Yêu cầu: ros2_control ĐÃ TẮT (nó phát 0x00C đè lệnh).")
    print(f"τ = ±{args.tau:.3f} Nm rotor, giữ {args.hold:g}s, chốt hành trình "
          f"{args.max_travel:g} rad")
    print("=" * 74)

    bus = Bus(args.can)
    out = {}
    for name in names:
        print(f"\n--- {name} (node {JOINTS[name]['node']}) ---")
        out[name] = run_one(bus, name, JOINTS[name], args.tau,
                            args.hold, args.max_travel)

    print("\n" + "=" * 74)
    print("KẾT LUẬN")
    print("=" * 74)
    for name in names:
        print(verdict(name, out[name]))
    print()
    print("Δq DƯƠNG khi τ dương  =  THUẬN  =  torque_sign 1")
    print("Δq ÂM     khi τ dương  =  NGƯỢC  =  torque_sign -1")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[Ctrl-C] khối finally đã dọn dẹp từng node đã chạm tới.")