"""Check direct torque output and preserve the tuned cascade responses."""

from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from gim_arm_controller_pid.controller import CascadePidController
from gim_arm_controller_pid.factory import CascadePidFactory


class _Dynamics:
    nq = 3
    tau_max = np.array([1.0, 2.0, 1.0])

    def inverse_dynamics(self, q, qd, qdd):
        return np.array([0.2, 0.4, -0.1]) + 0.1 * q + 0.05 * qd + qdd


class _Node:
    def __init__(self, overrides=None):
        self.parameters = {}
        self.overrides = overrides or {}

    def declare_parameter(self, name, default):
        self.parameters[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])


class TestCascadePidController(unittest.TestCase):
    def test_ros_entry_point_uses_package_factory(self):
        from gim_arm_controller_pid import node

        with patch.object(node, 'run_controller') as run_controller:
            node.main()

        run_controller.assert_called_once()
        self.assertIsInstance(run_controller.call_args.args[0], CascadePidFactory)

    def test_velocity_pi_outputs_torque_and_adds_feedforward(self):
        dynamics = _Dynamics()
        controller = CascadePidController(
            dynamics, kpp=[2, 3, 4], kvp=[0.5, 0.6, 0.7], kvi=[1, 2, 3])
        q = np.array([0.1, 0.2, 0.3])
        q_ref = q + 0.01
        qd = np.array([0.03, 0.02, 0.01])
        qd_ref = np.array([0.04, 0.05, 0.06])
        qdd_ref = np.zeros(3)
        initial_integral = np.array([0.01, 0.02, 0.03])
        controller.integral = initial_integral.copy()

        torque = controller.compute(q, qd, q_ref, qd_ref, qdd_ref, 0.01)

        error = qd_ref + controller.kpp * (q_ref - q) - qd
        expected_fb = controller.kvp * error + controller.kvi * initial_integral
        np.testing.assert_allclose(controller.last['tau_fb'], expected_fb)
        np.testing.assert_allclose(
            torque, dynamics.inverse_dynamics(q_ref, qd_ref, qdd_ref) + expected_fb)
        self.assertNotIn('current_command', controller.last)

    def test_profiles_preserve_legacy_torque_and_anti_windup(self):
        config_dir = Path(__file__).resolve().parents[1] / 'config'
        profiles = (
            ('pid.yaml', [8.0, 9.5, 10.0], [7.6, 7.6, 7.6]),
            ('pid_gazebo_smooth.yaml', [2.0, 3.0, 2.5], [1.5, 1.5, 1.5]),
        )
        for filename, old_kvp, old_kvi in profiles:
            with (config_dir / filename).open() as stream:
                params = yaml.safe_load(stream)['cascade_pid_controller']['ros__parameters']
            for integral_limit in (0.0, 0.025):
                with self.subTest(profile=filename, integral_limit=integral_limit):
                    params['velocity_integral_limit'] = integral_limit
                    node = _Node(params)
                    factory = CascadePidFactory()
                    factory.declare_parameters(node)
                    self.assertNotIn('torque_constant', node.parameters)
                    dynamics = _Dynamics()
                    controller = factory.build(node, dynamics, dynamics.tau_max, 100)
                    old_integral = np.zeros(3)
                    rng = np.random.default_rng(42)
                    saturated_count = 0
                    unsaturated_count = 0
                    for _ in range(500):
                        q, qd, q_ref, qd_ref, qdd_ref = rng.normal(0, 0.1, (5, 3))
                        dt = rng.uniform(0.001, 0.02)
                        error = qd_ref + controller.kpp * (q_ref - q) - qd
                        raw = dynamics.inverse_dynamics(q_ref, qd_ref, qdd_ref)
                        raw += 0.47 * (np.array(old_kvp) * error
                                       + np.array(old_kvi) * old_integral)
                        expected = np.clip(raw, -dynamics.tau_max, dynamics.tau_max)
                        saturated = np.abs(raw - expected) > 1e-12
                        drives_back = np.sign(0.47 * np.array(old_kvi) * error) != np.sign(raw)
                        old_integral += np.where(~saturated | drives_back, error * dt, 0)
                        if integral_limit > 0:
                            old_integral = np.clip(old_integral, -integral_limit, integral_limit)

                        actual = controller.compute(q, qd, q_ref, qd_ref, qdd_ref, dt)

                        np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=1e-12)
                        np.testing.assert_allclose(controller.integral, old_integral, atol=1e-12)
                        np.testing.assert_array_equal(controller.last['saturated'], saturated)
                        saturated_count += np.count_nonzero(saturated)
                        unsaturated_count += np.count_nonzero(~saturated)
                    self.assertGreater(saturated_count, 0)
                    self.assertGreater(unsaturated_count, 0)
                    controller.reset()
                    np.testing.assert_array_equal(controller.integral, np.zeros(3))

    def test_factory_defaults_match_nominal_profile(self):
        node = _Node()
        factory = CascadePidFactory()
        factory.declare_parameters(node)
        controller = factory.build(node, _Dynamics(), _Dynamics.tau_max, 100)
        config = Path(__file__).resolve().parents[1] / 'config' / 'pid.yaml'
        with config.open() as stream:
            params = yaml.safe_load(stream)['cascade_pid_controller']['ros__parameters']
        for name in ('kpp', 'kvp', 'kvi'):
            np.testing.assert_array_equal(getattr(controller, name), params[name])

    def test_tuning_can_disable_integral_without_disabling_feedforward(self):
        node = _Node({'kvi': [0.0, 0.0, 0.0]})
        factory = CascadePidFactory()
        factory.declare_parameters(node)
        controller = factory.build(node, _Dynamics(), _Dynamics.tau_max, 100)
        zero = np.zeros(3)
        tau = controller.compute(zero, zero, zero, zero, zero, 0.01)
        np.testing.assert_allclose(tau, _Dynamics().inverse_dynamics(zero, zero, zero))
        np.testing.assert_array_equal(controller.last['tau_i'], zero)
        with self.assertRaises(ValueError):
            node.parameters['kvi'] = [-0.1, 0.0, 0.0]
            factory.build(node, _Dynamics(), _Dynamics.tau_max, 100)

    def test_joint_motion_has_dynamic_feedforward_independent_of_tuned_gain(self):
        from gim_control.reference_trajectory import Quintic

        reference = Quintic(np.zeros(3), np.radians([0.0, 0.0, 30.0]), 6.0)
        q_ref, qd_ref, qdd_ref = reference.at(1.5)
        controllers = [CascadePidController(
            _Dynamics(), kpp=[2, 3, gain], kvp=[0.5, 0.6, 0.7], kvi=[0, 0, 0])
            for gain in (4, 8)]
        for controller in controllers:
            controller.compute(q_ref - [0, 0, 0.01], qd_ref,
                               q_ref, qd_ref, qdd_ref, 0.01)
            np.testing.assert_allclose(
                controller.last['tau_fb'], controller.last['tau_p'] + controller.last['tau_i'])
        np.testing.assert_allclose(controllers[0].last['tau_ff'], controllers[1].last['tau_ff'])
        self.assertGreater(controllers[1].last['tau_fb'][2], controllers[0].last['tau_fb'][2])
        static_ff = _Dynamics().inverse_dynamics(q_ref, np.zeros(3), np.zeros(3))
        self.assertGreater(abs(controllers[0].last['tau_ff'][2] - static_ff[2]), 0.001)
        np.testing.assert_allclose(reference.at(0)[0], np.zeros(3))
        np.testing.assert_allclose(reference.at(6)[0], np.radians([0, 0, 30]))
        for t in (0, 6):
            np.testing.assert_allclose(reference.at(t)[1], np.zeros(3))
            np.testing.assert_allclose(reference.at(t)[2], np.zeros(3))

    def test_hardware_profile_matches_driver_feedback_after_can_unit_conversion(self):
        root = Path(__file__).resolve().parents[2]
        urdf = ET.parse(root / 'gim_arm_description/urdf/gim_arm.urdf').getroot()
        with (root / 'gim_arm_controller_pid/config/pid_hardware_tuning.yaml').open() as stream:
            params = yaml.safe_load(stream)['cascade_pid_controller']['ros__parameters']
        cases = ((1, 'shoulder_joint', 10.0, 0.75, 1.0),
                 (2, 'elbow_joint', 25.0, 0.6, 10.0))
        for index, name, pos_gain, vel_gain, integrator_gain in cases:
            with self.subTest(joint=name):
                joint = urdf.find(f"ros2_control/joint[@name='{name}']")
                calibration = {param.get('name'): param.text for param in joint.findall('param')}
                velocity_ratio = float(calibration.get('gear_ratio', 8.0))
                torque_ratio = float(calibration.get('torque_gear_ratio', velocity_ratio))
                direction = -1 if calibration.get('invert_direction') == 'true' else 1
                torque_sign = float(calibration.get('torque_sign', 1.0))
                node = _Node(params)
                factory = CascadePidFactory()
                factory.declare_parameters(node)
                controller = factory.build(node, _Dynamics(), _Dynamics.tau_max, 100)
                q, qref, qd, qdref = np.zeros((4, 3))
                q[index] = 0.1
                qref[index] = 0.1001
                qd[index] = 0.001
                qdref[index] = 0.002
                integral = 0.0001
                controller.integral[index] = integral
                controller.compute(q, qd, qref, qdref, np.zeros(3), 0.01)

                encoder_scale = direction * velocity_ratio / (2 * np.pi)
                driver_velocity_error = encoder_scale * (
                    qdref[index] + pos_gain * (qref[index] - q[index]) - qd[index])
                driver_integral = encoder_scale * integral
                driver_feedback = (
                    vel_gain * driver_velocity_error + integrator_gain * driver_integral)
                pc_feedback_on_can = torque_sign * controller.last['tau_fb'][index] / torque_ratio
                self.assertAlmostEqual(pc_feedback_on_can, driver_feedback, places=9)
                self.assertEqual(controller.kpp[index], pos_gain)
                self.assertFalse(controller.last['saturated'][index])

    def test_soft_profile_avoids_shoulder_saturation_on_recorded_startup_states(self):
        path = Path(__file__).resolve().parents[1] / 'config/pid_hardware_soft.yaml'
        with path.open() as stream:
            params = yaml.safe_load(stream)['cascade_pid_controller']['ros__parameters']
        node = _Node(params)
        factory = CascadePidFactory()
        factory.declare_parameters(node)
        controller = factory.build(node, _Dynamics(), np.array([1.75, 14.0, 1.75]), 100)
        qref = np.array([0.0, 0.0112362934, -0.1325180012])
        snapshots = (
            (qref, [0.0, -0.8330908844, -0.9242833458]),
            ([0.0, -0.0013264512, -0.0954324427], [0.0, -1.2798777692, 3.7690391072]),
            ([0.0, -0.0116692356, -0.0349677212], [0.0, -0.9594870065, 6.0849905702]),
        )
        for q, qd in snapshots:
            controller.compute(q, qd, qref, np.zeros(3), np.zeros(3), 0.01)
            self.assertLess(abs(controller.last['tau_fb'][1]), 3.0)
            self.assertFalse(controller.last['saturated'][1])
            np.testing.assert_array_equal(controller.last['tau_i'][1:], np.zeros(2))
            np.testing.assert_allclose(
                controller.last['tau_ff'], _Dynamics().inverse_dynamics(
                    qref, np.zeros(3), np.zeros(3)))


if __name__ == '__main__':
    unittest.main()
