"""Run selected-joint MPC against a mock ROS arm, never CAN hardware."""

import csv
import json
import importlib.util
from pathlib import Path
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
import yaml
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from gim_control.hardware_description import configure_hardware_description


ROOT = Path(__file__).resolve().parents[3]
from gim_control import mpc_joint_trial as tuner


class TestMpcJointRosTrial(unittest.TestCase):
    def setUp(self):
        import tempfile
        self.directory = tempfile.TemporaryDirectory()
        self.trial = tuner.RosMpcJointTrial(self.directory.name, show_plot=False)
        self.context = rclpy.context.Context()
        rclpy.init(args=[], context=self.context)
        self.node = rclpy.create_node('mock_mpc_arm', context=self.context)
        source = (ROOT / 'src/gim_arm_description/urdf/gim_arm.urdf').read_text()
        self.stale_description = source
        self.description = configure_hardware_description(source, torque_joint='elbow')
        self.description_requests = 0
        self.stale_description_requests = 0
        self.effort_active = False
        self.switches = []
        self.prestart_velocity = 0.0
        self.prestart_drift = 0.0
        self.prestart_duration = 0.0
        self.activation_time = None
        self.commands_before_effort = []
        self.node.create_subscription(
            Float64MultiArray, '/gim_arm_effort_controller/commands',
            self.receive_command, 10)
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
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.stop_spinning = threading.Event()
        self.thread = threading.Thread(target=self.spin_mock_arm, daemon=True)
        self.thread.start()
        self.parameters = yaml.safe_load((
            ROOT / 'src/gim_arm_controller_mpc/config/mpc_joint_tuning.yaml').read_text())[
                'mpc_controller']['ros__parameters']

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
        mpc = self.trial.node
        if (mpc is not None and self.effort_active and mpc.phase == 'GRAVITY'
                and time.monotonic() - self.activation_time < self.prestart_duration):
            qd[0] = self.prestart_velocity
            q[0] += self.prestart_drift
        if mpc is not None and self.effort_active and mpc.phase in (
                'APPROACH', 'TRACK', 'RETURN', 'HOLD'):
            q, qd, _ = mpc._reference_at_phase(mpc.phase_elapsed)
        message = JointState()
        message.name = list(tuner.JOINT_NAMES)
        message.position = [float(value) for value in q]
        message.velocity = [float(value) for value in qd]
        message.header.stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(message)

    def receive_command(self, message):
        if not self.effort_active:
            self.commands_before_effort.append(list(message.data))

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
        if self.effort_active:
            self.activation_time = time.monotonic()
            self.activation_wall_time = time.time()
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

    def test_trial_adds_urdf_feedforward_runs_mpc_and_holds_two_measured_references(self):
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
        # Prediction matrices must follow the actual target, not the initial hold.
        times, terms = self.trial.node.controller.prediction_schedule
        self.assertGreater(times[-1], 0)
        self.assertGreater(max(np.linalg.norm(h - terms[0][0]) for h, f in terms), 1e-5)
        for row in rows:
            self.assertNotIn(None, row)
            self.assertAlmostEqual(float(row['qref_base_joint']), 0.03)
            self.assertAlmostEqual(float(row['qref_shoulder_joint']), 0.05)
            if row['event'] == 'CONTROL':
                for joint in tuner.JOINT_NAMES:
                    self.assertAlmostEqual(
                        float(row['tau_raw_' + joint]),
                        float(row['tau_ff_' + joint]) + float(row['tau_fb_' + joint]))
                    self.assertAlmostEqual(float(row['tau_fb_' + joint]),
                                           float(row['tau_feedback_' + joint]))
                    self.assertEqual(row['solver_accepted'], '1')
        self.assertGreater(max(float(row['qref_elbow_joint']) for row in rows), 0.1)
        self.assertTrue(path.with_name('model.urdf').is_file())
        self.assertEqual(path.with_name('model.urdf').read_text(), self.description)
        from gim_control.arm_dynamics import ArmDynamics
        dynamics = ArmDynamics(str(path.with_name('model.urdf')))
        track = next(row for row in rows if row['phase'] == 'TRACK' and row['event'] == 'CONTROL')
        expected_ff = dynamics.inverse_dynamics(
            [float(track['qref_' + joint]) for joint in tuner.JOINT_NAMES],
            np.zeros(3), np.zeros(3))
        expected_ff[0] *= self.parameters['base_ff_alpha']
        np.testing.assert_allclose(
            [float(track['tau_ff_' + joint]) for joint in tuner.JOINT_NAMES], expected_ff)
        self.assertTrue(path.with_name('tracking_q3_elbow.png').is_file())

    def test_wrong_hardware_selection_never_activates_effort(self):
        with self.assertRaisesRegex(RuntimeError, 'chưa cô lập'):
            self.trial.run(1, 1, 0.15, 0.04, self.parameters)
        self.assertEqual(self.switches, [])
        self.assertIsNone(self.trial.node)

    def test_base_handoff_keeps_home_mpc_active_until_transient_velocity_settles(self):
        self.description = configure_hardware_description(self.stale_description, torque_joint='base')
        self.parameters['base_ff_alpha'] = 1.5
        self.prestart_velocity = -0.15
        self.prestart_drift = -0.005
        self.prestart_duration = 0.15
        self.trial.run(0, 1, 0.15, 0.04, self.parameters)
        self.assertTrue(self.commands_before_effort)
        self.assertTrue(any(abs(command[0]) > 0.1 for command in self.commands_before_effort))
        path = next(Path(self.directory.name).glob('*/tracking.csv'))
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        prep = [row for row in rows if row['phase'] == 'GRAVITY' and row['event'] == 'CONTROL']
        self.assertTrue(any(abs(float(row['qd_base_joint'])) > 0.05 for row in prep))
        self.assertTrue(any(abs(float(row['tau_fb_base_joint'])) > 0 for row in prep))
        self.assertTrue(any(abs(float(row['integral_base_joint'])) > 0 for row in prep))
        first_move = next(row for row in rows if row['phase'] == 'APPROACH')
        self.assertGreaterEqual(float(first_move['t_wall']) - self.activation_wall_time, 1.3)
        self.assertLessEqual(abs(float(first_move['qd_base_joint'])), 0.05)
        self.assertAlmostEqual(float(first_move['qref_base_joint']), 0.03)
        from gim_control.arm_dynamics import ArmDynamics
        dynamics = ArmDynamics(str(path.with_name('model.urdf')))
        for row in rows:
            if row['event'] != 'CONTROL' or row['phase'] not in ('GRAVITY', 'TRACK', 'HOLD'):
                continue
            expected_ff = dynamics.inverse_dynamics(
                [float(row['qref_' + joint]) for joint in tuner.JOINT_NAMES],
                [float(row['qdref_' + joint]) for joint in tuner.JOINT_NAMES],
                np.zeros(3))
            expected_ff[0] *= 1.5
            np.testing.assert_allclose(
                [float(row['tau_ff_' + joint]) for joint in tuner.JOINT_NAMES], expected_ff)
        metadata = json.loads(path.with_name('trial.json').read_text())
        self.assertEqual(metadata['base_ff_alpha'], 1.5)
        saved = yaml.safe_load(path.with_name('parameters.yaml').read_text())
        self.assertEqual(saved['mpc_controller']['ros__parameters']['base_ff_alpha'], 1.5)
        self.assertFalse(self.effort_active)

    def test_persistent_prestart_motion_returns_to_position_without_starting_trajectory(self):
        self.prestart_velocity = -0.15
        self.prestart_duration = 10.0
        with patch.object(self.trial, 'prestart_timeout', 0.25):
            with self.assertRaisesRegex(RuntimeError, 'chưa giữ HOME ổn định'):
                self.trial.run(2, 1, 0.15, 0.04, self.parameters)
        self.assertFalse(self.effort_active)
        self.assertEqual(len(self.switches), 2)
        path = next(Path(self.directory.name).glob('*/tracking.csv'))
        with path.open() as stream:
            self.assertEqual({row['phase'] for row in csv.DictReader(stream)}, {'GRAVITY'})

    def test_prestart_drift_past_position_limit_returns_to_position(self):
        self.prestart_drift = -0.15
        self.prestart_duration = 10.0
        with self.assertRaisesRegex(RuntimeError, 'trôi khỏi HOME'):
            self.trial.run(2, 1, 0.15, 0.04, self.parameters)
        self.assertFalse(self.effort_active)
        self.assertEqual(len(self.switches), 2)

    def test_ctrl_c_during_mpc_motion_returns_to_position(self):
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
