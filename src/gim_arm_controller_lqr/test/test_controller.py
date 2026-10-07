"""Smoke tests for the package-owned LQR implementation."""

import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from gim_arm_controller_lqr.controller import LqrController, LqrWeights


class _Dynamics:
    nq = 3
    q_min = np.full(3, -1.0)
    q_max = np.full(3, 1.0)
    tau_max = np.array([5.0, 40.0, 5.0])
    joint_names = ("joint_1", "joint_2", "joint_3")

    def mass_matrix(self, _q):
        return np.diag([1.0, 2.0, 1.5])

    def gravity(self, _q):
        return np.zeros(3)

    def coriolis(self, _q, _qd):
        return np.zeros(3)

    def inverse_dynamics(self, q, qd, qdd):
        del q, qd
        return self.mass_matrix(None) @ qdd


class TestLqrController(unittest.TestCase):
    def test_matlab_gains_are_zoh_stable_at_2khz_but_not_100hz_on_robot(self):
        from gim_control.arm_dynamics import ArmDynamics

        root = Path(__file__).resolve().parents[3]
        dynamics = ArmDynamics(str(root / "src/gim_arm_description/urdf/gim_arm.urdf"))
        weights = LqrWeights(
            max_int_e=(10.0, 20.0, 10.0), max_e=(0.05, 0.02, 0.01),
            max_de=None, position_tracking_only=True, tau_penalty_scale=1.0)
        fast = LqrController(dynamics, weights=weights, control_hz=2000.0)
        self.assertTrue(fast.stability_report(samples=6)["stable_discrete"])
        with self.assertRaisesRegex(ValueError, "PHÂN KỲ"):
            LqrController(dynamics, weights=weights, control_hz=100.0)

    def test_runner_maps_loop_and_return_to_the_correct_gain_section(self):
        from gim_control.effort_controller_node import EffortControllerNode

        runner = SimpleNamespace(
            phase="APPROACH", approach_time=5.0,
            trajectory=SimpleNamespace(duration=27.0))
        time_at = lambda t: EffortControllerNode._gain_schedule_time(runner, t)
        self.assertEqual(time_at(2.0), 2.0)
        runner.phase = "TRACK"
        self.assertEqual(time_at(0.0), 5.0)
        self.assertEqual(time_at(10.0), 15.0)
        self.assertEqual(time_at(37.0), 15.0)
        runner.phase = "RETURN"
        self.assertEqual(time_at(0.0), 32.0)
        self.assertEqual(time_at(5.0), 37.0)

    def test_runner_precomputes_all_phases_without_duplicate_time_knots(self):
        from gim_control.effort_controller_node import EffortControllerNode
        from gim_control.reference_trajectory import Quintic

        zero = np.zeros(3)
        for mode, expected_count in (("trajectory_time", 5), ("reference_state", 7)):
            with self.subTest(mode=mode):
                controller = self._matlab_controller(
                    gain_schedule_mode=mode, gain_schedule_hz=2.0)
                runner = SimpleNamespace(
                    controller=controller,
                    trajectory=Quintic(np.full(3, 0.1), np.full(3, 0.2), 1.0),
                    approach_time=0.5, return_time=0.5,
                    fixed_track_gain_index=-1,
                    get_logger=lambda: SimpleNamespace(info=lambda _: None))
                self.assertTrue(EffortControllerNode._prepare_controller_schedule(runner, zero))
                self.assertEqual(len(controller._gain_schedule_gains), expected_count)
                if mode == "trajectory_time":
                    np.testing.assert_allclose(
                        controller._gain_schedule_times, [0.0, 0.5, 1.0, 1.5, 2.0])
                else:
                    self.assertIsNone(controller._gain_schedule_times)

    def _matlab_controller(self, **overrides):
        options = dict(
            weights=LqrWeights(
                max_int_e=(10.0, 20.0, 10.0),
                max_e=(0.05, 0.02, 0.01),
                max_de=None,
                position_tracking_only=True,
                tau_penalty_scale=1.0),
            control_hz=2000.0,
            gravity_at_measured=False,
            i_limit=0.5,
            gain_schedule_hz=20.0,
            gain_schedule_mode="trajectory_time")
        options.update(overrides)
        return LqrController(_Dynamics(), **options)

    def test_matlab_bryson_weights_and_torque_sign(self):
        controller = self._matlab_controller()
        np.testing.assert_allclose(
            np.diag(controller.Q),
            [0.01, 0.0025, 0.01, 400.0, 2500.0, 10000.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(np.diag(controller.R), [0.04, 1.0/1600.0, 0.04])
        zero = np.zeros(3)
        displacement = np.array([0.001, 0.0, 0.0])
        torque = controller.compute(displacement, zero, zero, zero, zero, 0.0005)
        self.assertLess(torque[0], 0.0)

    def test_zero_velocity_cost_keeps_damping_feedback(self):
        controller = self._matlab_controller()
        zero = np.zeros(3)
        torque = controller.compute(
            zero, np.array([0.01, 0.0, 0.0]), zero, zero, zero, 0.0005)
        self.assertLess(torque[0], 0.0)
        self.assertGreater(np.linalg.norm(controller.K[:, 6:]), 0.0)
        self.assertIsNone(controller.weights.max_de)

    def test_position_cost_does_not_read_legacy_max_de(self):
        weights = LqrWeights(position_tracking_only=True, max_de=None)
        expected_q, expected_r = weights.QR(_Dynamics.tau_max)
        weights.max_de = (-1.0, 0.0, float("nan"))
        q, r = weights.QR(_Dynamics.tau_max)
        np.testing.assert_allclose(q, expected_q)
        np.testing.assert_allclose(r, expected_r)

    def test_legacy_profiles_keep_positive_velocity_cost(self):
        q, _ = LqrWeights(max_de=(2.0, 4.0, 5.0)).QR(_Dynamics.tau_max)
        np.testing.assert_allclose(np.diag(q)[-3:], [0.25, 0.0625, 0.04])

    def test_position_profile_does_not_expose_max_de_parameter(self):
        from gim_arm_controller_lqr.factory import LqrFactory

        for position_only in (True, False):
            parameters = {}

            def declare(name, default):
                value = position_only if name == "position_tracking_only" else default
                parameters[name] = value
                return SimpleNamespace(value=value)

            LqrFactory().declare_parameters(SimpleNamespace(declare_parameter=declare))
            self.assertEqual("max_de" in parameters, not position_only)

    def test_gain_interpolation_uses_reference_clock_without_online_solve(self):
        controller = self._matlab_controller()
        q = np.array([[0.0, 0.0, 0.0], [0.2, 0.1, -0.1]])
        qd = np.zeros_like(q)
        controller.precompute_gain_schedule(q, qd, [0.0, 2.0])
        gains = controller._gain_schedule_gains.copy()
        # Give the endpoints distinct gains to expose interpolation versus
        # nearest-neighbour lookup even for this constant-mass test plant.
        controller._gain_schedule_gains[1] = gains[1] * 2.0
        expected_midpoint = (gains[0] + gains[1] * 2.0) / 2.0
        zero = np.zeros(3)
        with patch.object(controller, "gain", side_effect=AssertionError("online Riccati")):
            controller.set_reference_time(1.0)
            controller.compute(zero, zero, zero, zero, zero, 0.0005)
            np.testing.assert_allclose(controller.K, expected_midpoint)
            # Repeated callbacks during a paused reference use the same gain.
            controller.compute(zero, zero, q[1], zero, zero, 0.0005)
            np.testing.assert_allclose(controller.K, expected_midpoint)
            for t, expected in ((-1.0, gains[0]), (3.0, gains[1]*2.0)):
                controller.set_reference_time(t)
                controller.compute(zero, zero, zero, zero, zero, 0.0005)
                np.testing.assert_allclose(controller.K, expected)

    def test_time_schedule_rejects_duplicate_or_missing_timestamps(self):
        controller = self._matlab_controller()
        q = np.zeros((2, 3))
        for times in (None, [0.0], [0.0, 0.0], [1.0, 0.0], [0.0, np.nan]):
            with self.subTest(times=times), self.assertRaises(ValueError):
                controller.precompute_gain_schedule(q, q, times)

    def test_fixed_diagnostic_gain_overrides_interpolation(self):
        controller = self._matlab_controller()
        q = np.zeros((2, 3))
        controller.precompute_gain_schedule(q, q, [0.0, 1.0])
        controller._gain_schedule_gains[1] *= 2.0
        controller.set_reference_time(0.0)
        controller.set_fixed_gain_index(1)
        zero = np.zeros(3)
        controller.compute(zero, zero, zero, zero, zero, 0.0005)
        np.testing.assert_allclose(controller.K, controller._gain_schedule_gains[1])

    def test_compute_returns_zero_at_zero_equilibrium(self):
        controller = LqrController(
            _Dynamics(),
            weights=LqrWeights(tau_penalty_scale=512.0),
            control_hz=100.0,
            require_discrete_stable=False,
        )
        zero = np.zeros(3)

        torque = controller.compute(zero, zero, zero, zero, zero, 0.01)

        np.testing.assert_allclose(torque, zero, atol=1.0e-12)
        self.assertEqual(torque.shape, (3,))
        self.assertIn("K", controller.last)


if __name__ == "__main__":
    unittest.main()
