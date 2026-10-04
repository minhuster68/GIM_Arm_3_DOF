"""Exercise the tuning runner without sending commands to any robot."""

import csv
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from gim_arm_controller_pid.factory import CascadePidFactory
from gim_control.effort_controller_node import (
    ABORT, APPROACH, GRAVITY, HOLD, RETURN, TRACK, EffortControllerNode,
)
from gim_control.plot_ee_error import load_log


class TestTuningRunner(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.log_file = str(Path(self.directory.name) / 'tuning.csv')
        rclpy.init(args=[
            '--ros-args', '-p', 'diagnostic_hold:=true', '-p', 'cascade_hold:=true',
            '-p', 'diagnostic_q3_deg:=30.0', '-p', 'approach_time:=0.03',
            '-p', 'return_time:=0.03', '-p', 'diagnostic_hold_time:=0.03',
            '-p', 'start_velocity_limit_rad_s:=0.05',
            '-p', 'control_hz:=100.0', '-p', 'log_file:=' + self.log_file])
        self.node = EffortControllerNode(CascadePidFactory())

        def capture(tau):
            self.node.last_command = np.asarray(tau).copy()
            self.node.last_publish_wall_ns = time.time_ns()

        self.publish_patch = patch.object(self.node, '_publish', side_effect=capture)
        self.publish_patch.start()

    def tearDown(self):
        self.publish_patch.stop()
        self.node.destroy_node()
        rclpy.shutdown()
        self.directory.cleanup()

    def state(self, q, qd=None):
        msg = JointState()
        msg.name = self.node.joint_names
        msg.position = list(np.asarray(q, dtype=float))
        msg.velocity = list(np.zeros(3) if qd is None else np.asarray(qd, dtype=float))
        self.node._on_state(msg)

    def test_entire_move_hold_return_uses_cascade_and_preserves_integral_at_home(self):
        node = self.node
        self.state(np.zeros(3))
        node._tick()
        self.assertEqual(node.phase, GRAVITY)
        node.set_parameters([Parameter('autostart', value=True)])
        phases = set()
        preserved = np.array([0.01, 0.02, 0.03])
        for _ in range(30):
            if node.phase == GRAVITY:
                q, qd = np.zeros(3), np.zeros(3)
            else:
                q, qd, _ = node._reference_at_phase(node.phase_elapsed)
            if node.phase == RETURN and node.phase_elapsed >= node.return_time:
                node.controller.integral = preserved.copy()
            self.state(q, qd)
            node._tick()
            phases.add(node.phase)
            if node.phase == HOLD:
                break
        self.assertEqual(phases, {APPROACH, TRACK, RETURN, HOLD})
        np.testing.assert_allclose(node.controller.integral, preserved)
        np.testing.assert_allclose(node.controller.last['tau_i'], node.controller.kvi * preserved)
        node.dump()
        with open(self.log_file, newline='') as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(rows[-1]['phase'], HOLD)
        for row in rows:
            self.assertNotIn(None, row)
            for name in node.joint_names:
                self.assertAlmostEqual(float(row['tau_ff_' + name]) + float(row['tau_fb_' + name]),
                                       float(row['tau_raw_' + name]))
        log = load_log(self.log_file)
        self.assertIn('tau_fb', log)
        self.assertIn('tau_ff', log)

    def test_stale_sample_is_logged_without_claiming_new_feedback_was_computed(self):
        node = self.node
        self.state(np.zeros(3))
        node._tick()
        node.set_parameters([Parameter('autostart', value=True)])
        self.state(np.zeros(3))
        node._tick()
        node._tick()
        node.dump()
        with open(self.log_file, newline='') as stream:
            rows = list(csv.DictReader(stream))
        row = rows[-1]
        self.assertEqual(row['event'], 'STALE_STATE_HOLD')
        self.assertTrue(np.isnan(float(row['tau_fb_elbow_joint'])))
        self.assertTrue(np.isfinite(float(row['tau_elbow_joint'])))
        load_log(self.log_file)

    def test_pretrajectory_hold_logs_velocity_feedback_without_accumulating_integral(self):
        node = self.node
        self.state(np.zeros(3))
        node._tick()
        node.controller.integral[:] = 0.25
        self.state(np.zeros(3), [0.0, -0.02, 0.03])
        node._tick()
        self.assertEqual(node.phase, GRAVITY)
        np.testing.assert_array_equal(node.controller.integral, np.zeros(3))
        node.dump()
        with open(self.log_file, newline='') as stream:
            rows = list(csv.DictReader(stream))
        row = rows[-1]
        self.assertEqual(row['phase'], GRAVITY)
        self.assertEqual(row['event'], 'CONTROL')
        for index, name in enumerate(node.joint_names):
            self.assertAlmostEqual(
                float(row['tau_p_' + name]), -node.controller.kvp[index] * node.qd[index])
            self.assertEqual(float(row['tau_i_' + name]), 0)
            self.assertEqual(float(row['integral_' + name]), 0)
        load_log(self.log_file)

    def test_moving_arm_cannot_start_trajectory(self):
        node = self.node
        self.state(np.zeros(3))
        node._tick()
        node.set_parameters([Parameter('autostart', value=True)])
        self.state(np.zeros(3), [0.0, -0.83, 0.0])
        node._tick()
        self.assertEqual(node.phase, ABORT)
        np.testing.assert_allclose(node.last_command, node._gravity_torque())
        self.assertFalse(any(row[1] == APPROACH for row in node.rows))


if __name__ == '__main__':
    unittest.main()
