"""Read-only SocketCAN checks for the selected-joint tuning handoff."""

import math
import socket
import struct
import time
import xml.etree.ElementTree as ET


class CanHandoffMonitor:
    """Observe fresh holding feedback and outgoing mode/torque frames.

    This socket never sends commands. Seeing outgoing frames verifies the host
    handoff, not acknowledgement of the driver's internal control mode.
    """

    def __init__(self, interface, description, joint_name):
        joint = ET.fromstring(description).find(
            f"ros2_control[@name='GimArmSystem']/joint[@name='{joint_name}']")
        if joint is None:
            raise ValueError(f'Không tìm thấy cấu hình CAN cho {joint_name}')
        params = {p.attrib['name']: p.text for p in joint.findall('param')}
        self.node_id = int(params['can_node_id'])
        self.gear_ratio = float(params.get('gear_ratio', '8.0'))
        if not math.isfinite(self.gear_ratio) or self.gear_ratio <= 0:
            raise ValueError('gear_ratio CAN phải hữu hạn và > 0')
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.sock.bind((interface,))
            self.sock.setblocking(False)
        except BaseException:
            self.sock.close()
            raise
        self.velocity = None
        self.count = 0
        self.stamp = None
        self.mode_seen = False
        self.torque_seen = False
        self.discard_pending()

    def close(self):
        self.sock.close()

    def discard_pending(self):
        while True:
            try:
                self.sock.recv(16)
            except BlockingIOError:
                return

    def ingest(self, frame, now):
        can_id, length, data = struct.unpack('=IB3x8s', frame)
        if can_id & 0xE0000000 or (can_id >> 5) != self.node_id:
            return
        command = can_id & 0x1F
        if command == 0x09 and length >= 8:
            _, velocity_rev_s = struct.unpack('<ff', data)
            self.velocity = velocity_rev_s * 2 * math.pi / self.gear_ratio
        elif command == 0x1C and length >= 8:
            _, measured = struct.unpack('<ff', data)
            if (self.velocity is None or not math.isfinite(self.velocity)
                    or abs(self.velocity) > 0.1 or not math.isfinite(measured)):
                self.count, self.stamp = 0, None
                return
            if self.stamp is not None and now - self.stamp > 0.2:
                self.count = 0
            self.count += 1
            self.stamp = now
        elif command == 0x0B and length >= 8:
            mode, input_mode = struct.unpack('<II', data)
            self.mode_seen = mode == 1 and input_mode == 1
        elif command == 0x0E and length >= 4:
            self.torque_seen = math.isfinite(struct.unpack('<f', data[:4])[0])

    def poll(self):
        while True:
            try:
                frame = self.sock.recv(16)
            except BlockingIOError:
                return
            if len(frame) == 16:
                self.ingest(frame, time.monotonic())

    def feedback_ready(self):
        now = time.monotonic()
        return self.count >= 3 and self.stamp is not None and 0 <= now - self.stamp <= 0.2

    def begin_switch(self):
        # Do not let frames from an earlier trial validate this handoff.
        self.discard_pending()
        self.mode_seen = self.torque_seen = False

    def switch_seen(self):
        return self.mode_seen and self.torque_seen
