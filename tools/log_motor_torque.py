#!/usr/bin/env python3
"""Log GIM Get_Torques replies alongside a ROS tracking trial.

Only Get_Torques RTR requests are sent. This script does not change motor
mode, gains, limits, or torque commands. CAN reports driver-side torque;
joint-side torque uses torque_gear_ratio and torque_sign from the URDF,
not the encoder's total gear_ratio.
"""

import argparse
import csv
import math
from pathlib import Path
import socket
import struct
import time
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
GET_TORQUES = 0x1C
JOINTS = {name: name + '_joint' for name in ('base', 'shoulder', 'elbow')}


def joint_config(path, joint):
    root = ET.parse(path).getroot()
    node = root.find(f"ros2_control/joint[@name='{JOINTS[joint]}']")
    if node is None:
        raise ValueError(f'URDF thiếu ros2_control/{JOINTS[joint]}')
    params = {p.get('name'): p.text for p in node.findall('param')}
    node_id = int(params['can_node_id'])
    ratio = float(params.get('torque_gear_ratio') or params['gear_ratio'])
    sign = float(params.get('torque_sign', '1.0'))
    if not 0 <= node_id <= 63 or not math.isfinite(ratio) or ratio <= 0:
        raise ValueError('can_node_id/torque_gear_ratio không hợp lệ')
    if sign not in (-1.0, 1.0):
        raise ValueError('torque_sign phải là -1 hoặc 1')
    return node_id, ratio * sign


def decode_reply(frame, response_id):
    if len(frame) != 16:
        return None
    can_id, dlc = struct.unpack_from('=IB', frame)
    # Exact match rejects RTR loopback, extended frames and error frames.
    if can_id != response_id or dlc != 8:
        return None
    values = struct.unpack_from('<ff', frame, 8)
    return values if all(math.isfinite(v) for v in values) else None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--can', default='can0')
    parser.add_argument('--joint', choices=JOINTS, default='shoulder')
    parser.add_argument('--hz', type=float, default=20.0)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--urdf', type=Path,
                        default=ROOT / 'src/gim_arm_description/urdf/gim_arm.urdf')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if args.output is None:
        args.output = ROOT / f'results/{args.joint}_motor_torque.csv'
    if not math.isfinite(args.hz) or not 0 < args.hz <= 50:
        parser.error('--hz phải trong (0, 50]')
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration phải hữu hạn và > 0')
    try:
        node_id, conversion = joint_config(args.urdf, args.joint)
    except (OSError, ET.ParseError, KeyError, ValueError) as error:
        parser.error(str(error))
    response_id = (node_id << 5) | GET_TORQUES
    request = struct.pack('=IB3x8s', response_id | socket.CAN_RTR_FLAG, 8, b'\0' * 8)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    try:
        with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
            mask = (socket.CAN_SFF_MASK | socket.CAN_RTR_FLAG
                    | socket.CAN_EFF_FLAG | socket.CAN_ERR_FLAG)
            sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                            struct.pack('=II', response_id, mask))
            sock.bind((args.can,))
            with args.output.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(['t_wall', 'joint', 'node_id',
                                 'tau_setpoint_driver_nm', 'tau_measured_driver_nm',
                                 'tau_setpoint_joint_nm', 'tau_measured_joint_nm'])
                stream.flush()
                print(f'Đọc {args.joint}, ID{node_id}, {args.hz:g} Hz; '
                      f'quy đổi torque ×{conversion:g}.', flush=True)
                print(f'CSV: {args.output.resolve()}', flush=True)
                print('Bắt đầu bài test ở terminal tuner; Ctrl+C để kết thúc log.',
                      flush=True)
                period = 1.0 / args.hz
                started = next_request = time.monotonic()
                deadline = started + args.duration
                try:
                    while time.monotonic() < deadline:
                        now = time.monotonic()
                        if now >= next_request:
                            sock.send(request)
                            next_request = now + period
                        timeout = min(next_request, deadline) - time.monotonic()
                        if timeout <= 0:
                            continue
                        sock.settimeout(timeout)
                        try:
                            frame = sock.recv(16)
                        except socket.timeout:
                            continue
                        received_wall = time.time()
                        values = decode_reply(frame, response_id)
                        if values is None:
                            continue
                        writer.writerow([received_wall, args.joint, node_id,
                                         *values, *(v * conversion for v in values)])
                        stream.flush()
                        count += 1
                except KeyboardInterrupt:
                    pass
    except OSError as error:
        print(f'Lỗi CAN/file: {error}')
        return 1
    print(f'Đã ghi {count} mẫu torque: {args.output.resolve()}')
    if count == 0:
        print('Không có phản hồi Get_Torques; chưa có dữ liệu để kết luận torque đo.')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
