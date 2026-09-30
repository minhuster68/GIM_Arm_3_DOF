"""Smoke tests for the package-owned LQR implementation."""

import unittest

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
