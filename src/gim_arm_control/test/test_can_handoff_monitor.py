"""Handoff checks reject stale feedback and false-positive ROS switches."""

import math
import struct
from unittest.mock import patch

from gim_control.can_handoff_monitor import CanHandoffMonitor


def monitor():
    result = CanHandoffMonitor.__new__(CanHandoffMonitor)
    result.node_id, result.gear_ratio = 2, 8.0
    result.velocity, result.count, result.stamp = None, 0, None
    result.mode_seen = result.torque_seen = False
    return result


def frame(command, payload, node=2, flags=0):
    return struct.pack('=IB3x8s', (node << 5) | command | flags, len(payload), payload)


def encoder(velocity):
    return frame(9, struct.pack('<ff', 0, velocity * 8 / (2 * math.pi)))


def torque():
    return frame(28, struct.pack('<ff', 0, 0.2))


def test_requires_three_fresh_feedback_samples_and_low_velocity():
    m = monitor()
    m.ingest(encoder(0.02), 1.0)
    with patch('gim_control.can_handoff_monitor.time.monotonic', return_value=1.11):
        for now in (1.0, 1.05):
            m.ingest(torque(), now)
        assert not m.feedback_ready()
        m.ingest(torque(), 1.1)
        assert m.feedback_ready()
    with patch('gim_control.can_handoff_monitor.time.monotonic', return_value=1.31):
        assert not m.feedback_ready()
    m.ingest(encoder(0.2), 1.32)
    m.ingest(torque(), 1.33)
    assert m.count == 0


def test_gap_nonfinite_feedback_and_missing_velocity_reset_readiness():
    m = monitor()
    m.ingest(torque(), 1)
    assert m.count == 0
    m.ingest(encoder(0.0), 1)
    for t in (1, 1.05, 1.1):
        m.ingest(torque(), t)
    m.ingest(torque(), 1.4)
    assert m.count == 1
    m.ingest(frame(28, struct.pack('<ff', 0, float('nan'))), 1.45)
    assert m.count == 0


def test_requires_both_selected_motor_mode_and_finite_torque_command():
    m = monitor()
    mode = struct.pack('<II', 1, 1)
    m.ingest(frame(11, mode, node=1), 1)
    m.ingest(frame(11, mode, flags=0x40000000), 1)
    assert not m.mode_seen
    m.ingest(frame(11, mode), 1)
    assert not m.switch_seen()
    m.ingest(frame(14, struct.pack('<f', float('nan'))), 1)
    assert not m.switch_seen()
    m.ingest(frame(14, struct.pack('<f', 0.1)), 1)
    assert m.switch_seen()
    m.ingest(frame(11, struct.pack('<II', 3, 1)), 1)
    assert not m.switch_seen()


def test_begin_switch_discards_frames_from_previous_trial():
    m = monitor()
    m.mode_seen = m.torque_seen = True
    with patch.object(m, 'discard_pending') as discard:
        m.begin_switch()
    discard.assert_called_once()
    assert not m.switch_seen()
