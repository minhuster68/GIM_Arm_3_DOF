"""Passive evidence of host mode/torque frames; never transmits CAN commands."""
import math
import socket
import struct


class CanTorqueModeProbe:
    def __init__(self, interface, node_id, socket_factory=socket.socket):
        self.mode_id = (node_id << 5) | 0x0B
        self.torque_id = (node_id << 5) | 0x0E
        self.mode_seen = False
        self.torque_seen = False
        self.socket = socket_factory(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            mask = 0x7FF | socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG
            filters = b''.join(struct.pack('=II', ident, mask)
                               for ident in (self.mode_id, self.torque_id))
            self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            self.socket.bind((interface,))
            self.socket.setblocking(False)
        except Exception:
            self.socket.close()
            raise

    def poll(self):
        while True:
            try:
                frame = self.socket.recv(16)
            except BlockingIOError:
                break
            if len(frame) != 16:
                continue
            ident, size, data = struct.unpack('=IB3x8s', frame)
            if ident & (socket.CAN_RTR_FLAG | socket.CAN_EFF_FLAG | socket.CAN_ERR_FLAG):
                continue
            if ident == self.mode_id and size == 8:
                self.mode_seen = struct.unpack('<II', data) == (1, 1)
                self.torque_seen = False
            elif ident == self.torque_id and size >= 4 and self.mode_seen:
                self.torque_seen = math.isfinite(struct.unpack('<f', data[:4])[0])
        return self.mode_seen and self.torque_seen

    def close(self):
        self.socket.close()
