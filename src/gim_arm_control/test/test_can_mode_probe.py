"""Passive mode evidence: no real socket or outgoing CAN in these tests."""
import socket
import struct

from gim_control.can_mode_probe import CanTorqueModeProbe


class FakeSocket:
    def __init__(self, *args): self.frames=[]; self.closed=False
    def setsockopt(self, *args): pass
    def bind(self, *args): pass
    def setblocking(self, *args): pass
    def close(self): self.closed=True
    def recv(self, size):
        if not self.frames: raise BlockingIOError()
        return self.frames.pop(0)


def frame(ident, data):
    return struct.pack('=IB3x8s',ident,len(data),data.ljust(8,b'\0'))


def test_requires_matching_mode_then_finite_torque_in_order():
    probe=CanTorqueModeProbe('mock0',2,FakeSocket)
    probe.socket.frames=[frame(0x4e,struct.pack('<f',.2)),
                         frame(0x2b,struct.pack('<II',1,1))]
    assert not probe.poll()
    probe.socket.frames=[frame(0x4b,struct.pack('<II',1,1)),
                         frame(0x4e,struct.pack('<f',float('nan')))]
    assert not probe.poll()
    probe.socket.frames=[frame(0x4e,struct.pack('<f',.2))]
    assert probe.poll()
    probe.socket.frames=[frame(0x4b,struct.pack('<II',3,1))]
    assert not probe.poll()
    probe.close(); assert probe.socket.closed


def test_remote_error_and_short_frames_do_not_confirm_mode():
    probe=CanTorqueModeProbe('mock0',2,FakeSocket)
    probe.socket.frames=[b'bad',frame(0x4b|socket.CAN_RTR_FLAG,struct.pack('<II',1,1)),
                         frame(0x4b|socket.CAN_ERR_FLAG,struct.pack('<II',1,1)),
                         frame(0x4e,struct.pack('<f',.2))]
    assert not probe.poll()
