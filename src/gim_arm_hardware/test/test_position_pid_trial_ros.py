"""Run the PC PID terminal against a mock ROS arm, never CAN hardware."""

import csv
import importlib.util
from pathlib import Path
import threading
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

from gim_control.hardware_description import configure_hardware_description


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'src/gim_arm_hardware/scripts/tune_position_pid.py'
SPEC = importlib.util.spec_from_file_location('position_pid_ros_trial', SCRIPT)
tuner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tuner)


class TestPositionPidRosTrial(unittest.TestCase):
    def setUp(self):
        import tempfile
        self.directory = tempfile.TemporaryDirectory()
        self.trial = tuner.RosPositionTrial(self.directory.name, show_plot=False)
        self.context = rclpy.context.Context()
        rclpy.init(args=[], context=self.context)
        self.node = rclpy.create_node('mock_pid_arm', context=self.context)
        source = (ROOT / 'src/gim_arm_description/urdf/gim_arm.urdf').read_text()
        self.stale_description = source
        self.description = configure_hardware_description(source, torque_joint='elbow')
        self.description_requests = 0
        self.stale_description_requests = 0
        self.effort_active = False
        self.switches = []
        self.publisher = self.node.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.node.create_timer(0.01, self.publish_state)
        self.node.create_service(
            ListControllers, '/controller_manager/list_controllers', self.list_controllers)
        self.node.create_service(
            SwitchController, '/controller_manager/switch_controller', self.switch_controller)
        self.node.create_service(
            GetParameters, '/controller_manager/get_parameters', self.robot_description)
        self.node.create_service(
            GetParameters, '/robot_state_publisher/get_parameters', self.stale_robot_description)
        self.executor = MultiThreadedExecutor(num_threads=3, context=self.context)
        self.executor.add_node(self.node)
        self.stop_spinning = threading.Event()
        self.thread = threading.Thread(target=self.spin_mock_arm, daemon=True)
        self.thread.start()
        self.parameters = tuner.load_config(
            ROOT / 'src/gim_arm_controller_pid/config/pid_hardware_soft.yaml')[
                'cascade_pid_controller']['ros__parameters']

    def tearDown(self):
        self.stop_spinning.set()
        self.thread.join(timeout=3.0)
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        self.directory.cleanup()

    def spin_mock_arm(self):
        while not self.stop_spinning.is_set():
            self.executor.spin_once(timeout_sec=0.05)

    def publish_state(self):
        q, qd = np.array([0.03, 0.05, 0.1]), np.zeros(3)
        pid = self.trial.pid
        if pid is not None and self.effort_active and pid.phase in (
                'APPROACH', 'TRACK', 'RETURN', 'HOLD'):
            q, qd, _ = pid._reference_at_phase(pid.phase_elapsed)
        message = JointState()
        message.name = list(tuner.JOINT_NAMES)
        message.position = [float(value) for value in q]
        message.velocity = [float(value) for value in qd]
        message.header.stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(message)

    def list_controllers(self, request, response):
        for name, active in (
                ('gim_arm_group_controller', not self.effort_active),
                ('gim_arm_effort_controller', self.effort_active)):
            state = ControllerState(name=name, state='active' if active else 'inactive')
            response.controller.append(state)
        return response

    def switch_controller(self, request, response):
        self.switches.append(request)
        self.effort_active = 'gim_arm_effort_controller' in request.activate_controllers
        response.ok = True
        return response

    def robot_description(self, request, response):
        self.description_requests += 1
        response.values = [ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=self.description)]
        return response

    def stale_robot_description(self, request, response):
        self.stale_description_requests += 1
        response.values = [ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=self.stale_description)]
        return response

    def test_trial_adds_urdf_feedforward_runs_pid_and_holds_two_measured_references(self):
        self.trial.run(2, 1, 0.15, 0.04, self.parameters)
        self.assertEqual(self.description_requests, 1)
        self.assertEqual(self.stale_description_requests, 0)
        self.assertFalse(self.effort_active)
        self.assertEqual(len(self.switches), 2)
        self.assertEqual(list(self.switches[0].activate_controllers), ['gim_arm_effort_controller'])
        self.assertEqual(list(self.switches[1].activate_controllers), ['gim_arm_group_controller'])
        path = next(Path(self.directory.name).glob('*/tracking.csv'))
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertTrue({'APPROACH', 'TRACK', 'RETURN', 'HOLD'} <= {row['phase'] for row in rows})
        for row in rows:
            self.assertNotIn(None, row)
            self.assertAlmostEqual(float(row['qref_base_joint']), 0.03)
            self.assertAlmostEqual(float(row['qref_shoulder_joint']), 0.05)
            if row['event'] == 'CONTROL':
                for joint in tuner.JOINT_NAMES:
                    self.assertAlmostEqual(
                        float(row['tau_raw_' + joint]),
                        float(row['tau_ff_' + joint]) + float(row['tau_fb_' + joint]))
                    self.assertAlmostEqual(
                        float(row['tau_fb_' + joint]),
                        sum(float(row[prefix + joint]) for prefix in ('tau_p_', 'tau_i_', 'tau_d_')))
        self.assertGreater(max(float(row['qref_elbow_joint']) for row in rows), 0.1)
        self.assertTrue(path.with_name('model.urdf').is_file())
        self.assertEqual(path.with_name('model.urdf').read_text(), self.description)
        from gim_control.arm_dynamics import ArmDynamics
        dynamics = ArmDynamics(str(path.with_name('model.urdf')))
        track = next(row for row in rows if row['phase'] == 'TRACK' and row['event'] == 'CONTROL')
        expected_ff = dynamics.inverse_dynamics(
            [float(track['qref_' + joint]) for joint in tuner.JOINT_NAMES],
            np.zeros(3), np.zeros(3))
        np.testing.assert_allclose(
            [float(track['tau_ff_' + joint]) for joint in tuner.JOINT_NAMES], expected_ff)
        self.assertTrue(path.with_name('tracking_q3_elbow.png').is_file())

    def test_wrong_hardware_selection_never_activates_effort(self):
        with self.assertRaisesRegex(RuntimeError, 'chưa cô lập'):
            self.trial.run(1, 1, 0.15, 0.04, self.parameters)
        self.assertEqual(self.switches, [])
        self.assertIsNone(self.trial.pid)

    def test_ctrl_c_during_pid_motion_returns_to_position(self):
        from gim_control.effort_controller_node import EffortControllerNode
        original = EffortControllerNode._tick

        def interrupt_motion(node):
            original(node)
            if node.phase == 'TRACK':
                raise KeyboardInterrupt()

        with patch.object(EffortControllerNode, '_tick', interrupt_motion):
            with self.assertRaises(KeyboardInterrupt):
                self.trial.run(2, 1, 0.15, 0.04, self.parameters)
        self.assertFalse(self.effort_active)
        self.assertEqual(len(self.switches), 2)
        self.assertEqual(list(self.switches[-1].activate_controllers), ['gim_arm_group_controller'])
        self.assertTrue(list(Path(self.directory.name).glob('*/tracking.csv')))
