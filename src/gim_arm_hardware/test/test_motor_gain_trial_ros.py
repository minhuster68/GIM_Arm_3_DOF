"""Exercise trial goals and cancellation with an isolated mock ROS arm."""

import csv
import importlib.util
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'tune_motor_gains.py'
SPEC = importlib.util.spec_from_file_location('motor_gain_tuner', SCRIPT)
tuner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tuner)


class TestRosMotorGainTrial(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.context = rclpy.context.Context()
        rclpy.init(args=[], context=self.context)
        self.node = rclpy.create_node('mock_arm', context=self.context)
        self.active = True
        self.received = []
        self.cancelled = threading.Event()
        self.state_publisher = self.node.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.node.create_timer(0.01, self.publish_state)
        self.service = self.node.create_service(
            ListControllers, '/controller_manager/list_controllers', self.list_controllers)
        self.action = ActionServer(
            self.node, FollowJointTrajectory,
            '/gim_arm_group_controller/follow_joint_trajectory', self.execute_goal,
            cancel_callback=lambda request: CancelResponse.ACCEPT)
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.trial = tuner.RosJointTrial(
            'gim_arm_group_controller', self.directory.name, show_plot=False)

    def tearDown(self):
        self.trial.close()
        self.executor.shutdown()
        self.thread.join(timeout=3.0)
        self.action.destroy()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        self.directory.cleanup()

    def publish_state(self):
        message = JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = list(tuner.JOINT_NAMES)
        message.position = [0.03, 0.05, 0.10]
        message.velocity = [0.0] * 3
        self.state_publisher.publish(message)

    def list_controllers(self, request, response):
        controller = ControllerState()
        controller.name = 'gim_arm_group_controller'
        controller.state = 'active' if self.active else 'inactive'
        response.controller = [controller]
        return response

    def execute_goal(self, handle):
        self.received.append(handle.request)
        for point in handle.request.trajectory.points:
            if handle.is_cancel_requested:
                handle.canceled()
                self.cancelled.set()
                return FollowJointTrajectory.Result()
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = handle.request.trajectory.joint_names
            feedback.desired = point
            feedback.actual.positions = list(point.positions)
            feedback.actual.positions[2] += 0.001
            handle.publish_feedback(feedback)
            time.sleep(0.03)
        handle.succeed()
        return FollowJointTrajectory.Result(error_code=0, error_string='mock success')

    def test_trial_uses_measured_start_holds_other_joints_and_logs_feedback(self):
        self.trial.run(2, 30.0, 0.1, 0.02, dict(kpp=20.0, kvp=0.16, kvi=0.0))
        self.assertEqual(len(self.received), 1)
        goal = self.received[0]
        self.assertEqual(goal.trajectory.joint_names, list(tuner.JOINT_NAMES))
        self.assertEqual(len(goal.trajectory.points), 5)
        for point in goal.trajectory.points:
            self.assertEqual(list(point.positions[:2]), [0.03, 0.05])
            self.assertEqual(list(point.velocities), [0.0] * 3)
            self.assertEqual(list(point.accelerations), [0.0] * 3)
        self.assertAlmostEqual(goal.trajectory.points[1].positions[2], 0.1 + math.radians(30))
        self.assertEqual(list(goal.trajectory.points[-1].positions), [0.03, 0.05, 0.10])
        paths = list(Path(self.directory.name).glob('*.csv'))
        self.assertEqual(len(paths), 1)
        with paths[0].open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        self.assertGreaterEqual(len(rows), 1)
        error = float(rows[0]['q_elbow_joint']) - float(rows[0]['qref_elbow_joint'])
        self.assertAlmostEqual(error, 0.001)
        self.assertEqual(float(rows[0]['kpp']), 20.0)
        self.assertTrue(paths[0].with_suffix('.png').is_file())

    def test_inactive_controller_and_out_of_limits_never_send_goal(self):
        self.active = False
        with self.assertRaisesRegex(RuntimeError, 'chưa active'):
            self.trial.run(2, 30, 6, 3, dict(kpp=None, kvp=None, kvi=None))
        self.active = True
        with self.assertRaisesRegex(ValueError, 'giới hạn'):
            self.trial.run(2, 200, 6, 3, dict(kpp=None, kvp=None, kvi=None))
        self.assertEqual(self.received, [])

    def test_ctrl_c_cancels_accepted_goal_and_writes_partial_log(self):
        wait = self.trial.wait
        calls = 0

        def interrupt_result(future, timeout):
            nonlocal calls
            calls += 1
            if calls == 3:
                raise KeyboardInterrupt
            return wait(future, timeout)

        with patch.object(self.trial, 'wait', side_effect=interrupt_result):
            with self.assertRaises(KeyboardInterrupt):
                self.trial.run(2, 30, 0.1, 0.02, dict(kpp=20.0, kvp=0.16, kvi=0.0))
        self.assertTrue(self.cancelled.wait(timeout=2.0))
        self.assertEqual(len(list(Path(self.directory.name).glob('*.csv'))), 1)


if __name__ == '__main__':
    unittest.main()
