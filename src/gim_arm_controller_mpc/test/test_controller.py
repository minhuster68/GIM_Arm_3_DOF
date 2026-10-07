"""Check MATLAB cost scaling, prediction and constrained feedback behavior."""

from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
import yaml

from gim_arm_controller_mpc.controller import MpcController
from gim_arm_controller_mpc.factory import MpcFactory


class _Dynamics:
    nq = 3
    tau_max = np.array([5.0, 40.0, 5.0])

    def mass_matrix(self, q):
        return np.diag([1.0, 2.0, 1.5])

    def inverse_dynamics(self, q, qd, qdd):
        return self.mass_matrix(q) @ qdd + np.asarray(q)


class _Parameters:
    def __init__(self, overrides):
        self.values = dict(overrides)

    def declare_parameter(self, name, default):
        self.values.setdefault(name, default)
        return self.get_parameter(name)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.values[name])


class TestMpcPositionTracking(unittest.TestCase):
    def controller(self, **overrides):
        path = Path(__file__).resolve().parents[1] / "config/mpc_gazebo_position.yaml"
        values = yaml.safe_load(path.read_text())["mpc_controller"]["ros__parameters"]
        values.update(torque_penalty_scale=1.0, precompute_prediction=False)
        values.update(overrides)
        node = _Parameters(values)
        factory = MpcFactory()
        factory.declare_parameters(node)
        return factory.build(node, _Dynamics(), _Dynamics.tau_max, 100.0)

    def test_factory_matches_squared_matlab_weights_and_allows_zero_velocity(self):
        c = self.controller()
        np.testing.assert_allclose(np.diag(c.Q), [
            64.0, 64.0, 64.0,
            (0.6271/0.05)**2, (1.5677/0.05)**2, (0.6271/0.05)**2,
            0.0, 0.0, 0.0])
        np.testing.assert_allclose(np.diag(c.R), (0.0097/_Dynamics.tau_max)**2)
        np.testing.assert_allclose(np.diag(c.Rd), (0.03/_Dynamics.tau_max)**2)

    def test_legacy_cost_convention_is_preserved(self):
        c = self.controller(weight_convention="legacy_linear")
        np.testing.assert_allclose(np.diag(c.Q)[:3], np.full(3, 2.0/0.25**2))

    def test_gazebo_penalty_multiplies_both_input_costs_without_velocity_cost(self):
        source = self.controller()
        gazebo = self.controller(torque_penalty_scale=10000.0)
        np.testing.assert_allclose(gazebo.Q, source.Q)
        np.testing.assert_allclose(gazebo.R, 10000.0*source.R)
        np.testing.assert_allclose(gazebo.Rd, 10000.0*source.Rd)
        for invalid in (0.0, -1.0, float("nan")):
            with self.assertRaises(ValueError):
                self.controller(torque_penalty_scale=invalid)

    def test_cached_prediction_matches_online_qp_at_reference_samples(self):
        cached = self.controller(precompute_prediction=True, torque_penalty_scale=10000.0)
        online = self.controller(torque_penalty_scale=10000.0)
        q = np.array([[0.0, 0.0, 0.0], [0.1, -0.2, 0.1]])
        qd = q*0.1
        qdd = q*0.2
        for c in (cached, online):
            c.prepare_reference(q, qd, qdd, [0.0, 1.0])
        for k in (0, 1, 0):
            cached.set_reference_time(float(k))
            actual = cached.compute(q[k]+0.001, qd[k], q[k], qd[k], qdd[k], 0.01)
            expected = online.compute(q[k]+0.001, qd[k], q[k], qd[k], qdd[k], 0.01)
            np.testing.assert_allclose(actual, expected, atol=1e-8)
        cached.reset()
        self.assertIsNotNone(cached.prediction_schedule)

    def test_discretization_matches_zoh_mechanics_and_euler_integral(self):
        c = self.controller()
        zero = np.zeros(3)
        a, b = c._linearize(zero, zero, zero)
        np.testing.assert_allclose(a[:3, :3], np.eye(3))
        np.testing.assert_allclose(a[:3, 3:6], 0.01*np.eye(3))
        np.testing.assert_allclose(a[:3, 6:], 0.0)
        np.testing.assert_allclose(b[:3], 0.0)
        # Each joint is an independent harmonic oscillator: qdd=-q/m+u/m.
        mass = np.diag(_Dynamics().mass_matrix(zero))
        for j in range(3):
            w = 1.0/np.sqrt(mass[j])
            self.assertAlmostEqual(a[3+j, 3+j], np.cos(w*0.01), places=12)
            self.assertAlmostEqual(a[3+j, 6+j], np.sin(w*0.01)/w, places=12)
            self.assertAlmostEqual(b[3+j, j], 1.0-np.cos(w*0.01), places=12)
            self.assertAlmostEqual(b[6+j, j], w*np.sin(w*0.01), places=12)

    def prepare(self, c):
        q = np.array([[-0.5, -1.0, -0.25], [0.75, 0.5, 0.4]])
        zeros = np.zeros_like(q)
        c.prepare_reference(q, zeros, zeros, [0.0, 1.0])
        return q

    def test_global_feedback_bounds_protect_every_feedforward_extreme(self):
        c = self.controller()
        q = self.prepare(c)
        np.testing.assert_allclose(c.feedback_min, -c.tau_limit-q.min(axis=0))
        np.testing.assert_allclose(c.feedback_max, c.tau_limit-q.max(axis=0))
        for ff in (q[0], q[1], q.mean(axis=0)):
            for feedback in (c.feedback_min, c.feedback_max):
                self.assertTrue(np.all(np.abs(ff+feedback) <= c.tau_limit+1e-12))
        c.reset()
        self.assertIsNotNone(c.feedback_min)

    def test_preparation_rejects_feedforward_without_feedback_headroom(self):
        c = self.controller()
        q = np.array([[0.0, 0.0, 0.0], [6.0, 0.0, 0.0]])
        with self.assertRaisesRegex(ValueError, "miền phản hồi"):
            c.prepare_reference(q, np.zeros_like(q), np.zeros_like(q), [0.0, 1.0])

    def test_condensed_qp_equals_direct_rollout_cost_with_move_blocking(self):
        c = self.controller(prediction_horizon=5, control_horizon=2)
        c.previous_feedback = np.array([0.1, -0.2, 0.05])
        rng = np.random.default_rng(12)
        state = rng.normal(scale=0.01, size=9)
        u = rng.normal(scale=0.2, size=(2, 3))
        zero = np.zeros(3)
        a, b = c._linearize(zero, zero, zero)
        h, f = c._qp_terms(state, a, b)

        def rollout(sequence):
            x = state.copy()
            previous = c.previous_feedback.copy()
            total = 0.0
            for k in range(c.np):
                current = sequence[min(k, c.nc-1)]
                x = a @ x + b @ current
                du = current-previous
                total += x @ c.Q @ x + current @ c.R @ current + du @ c.Rd @ du
                previous = current
            return total

        predicted = c._quadratic_cost(u.ravel(), h, f)
        self.assertAlmostEqual(predicted, rollout(u)-rollout(np.zeros_like(u)), places=10)

    def test_admm_feedback_respects_global_torque_and_slew(self):
        c = self.controller(qp_solver="admm", admm_iterations=300)
        self.prepare(c)
        zero = np.zeros(3)
        previous = c.previous_feedback.copy()
        torque = c.compute(np.array([0.03, -0.02, 0.01]), zero, zero, zero, zero, 0.01)
        self.assertTrue(c.last["solver_accepted"])
        self.assertLess(c.last["constraint_violation"], 1e-7)
        self.assertTrue(np.all(np.abs(torque) <= c.tau_limit))
        self.assertTrue(np.all(np.abs(c.previous_feedback-previous) <= c.slew*0.01+1e-12))
        self.assertLess(torque[0], 0.0)

    def test_admm_solution_agrees_with_slsqp_on_constrained_position_problem(self):
        admm = self.controller(qp_solver="admm", admm_iterations=1000, admm_tolerance=1e-7)
        slsqp = self.controller(qp_solver="slsqp", max_solver_iterations=300)
        self.prepare(admm)
        self.prepare(slsqp)
        zero = np.zeros(3)
        state_q = np.array([0.03, -0.02, 0.01])
        ta = admm.compute(state_q, zero, zero, zero, zero, 0.01)
        ts = slsqp.compute(state_q, zero, zero, zero, zero, 0.01)
        self.assertTrue(admm.last["solver_accepted"])
        self.assertTrue(slsqp.last["solver_accepted"])
        np.testing.assert_allclose(ta, ts, atol=5e-3, rtol=1e-2)

    def test_active_set_matches_slsqp_with_torque_rate_constraints(self):
        active = self.controller(qp_solver="active_set")
        slsqp = self.controller(qp_solver="slsqp", max_solver_iterations=300)
        self.prepare(active)
        self.prepare(slsqp)
        zero = np.zeros(3)
        state_q = np.array([0.03, -0.02, 0.01])
        ta = active.compute(state_q, zero, zero, zero, zero, 0.01)
        ts = slsqp.compute(state_q, zero, zero, zero, zero, 0.01)
        self.assertTrue(active.last["solver_success"])
        self.assertTrue(active.last["solver_accepted"])
        self.assertLess(active.last["constraint_violation"], 1e-7)
        np.testing.assert_allclose(ta, ts, atol=5e-3, rtol=1e-2)

    def test_active_set_solves_ill_conditioned_robot_qp_when_bounds_are_inactive(self):
        from gim_control.arm_dynamics import ArmDynamics

        root = Path(__file__).resolve().parents[3]
        c = self.controller()
        c.dyn = ArmDynamics(str(root / "src/gim_arm_description/urdf/gim_arm.urdf"),
                            use_armature=False)
        zero = np.zeros(3)
        a, b = c._linearize(zero, zero, zero)
        h, f = c._qp_terms(np.r_[zero, [1e-5, -2e-5, 1e-5], zero], a, b)
        expected = np.linalg.solve(h, -f)
        lower = np.full(60, -10.0)
        upper = np.full(60, 10.0)
        actual, _, success = c._solve_active_set(h, f, np.zeros(30), lower, upper)
        self.assertTrue(success)
        np.testing.assert_allclose(actual, expected, atol=1e-6, rtol=1e-4)


if __name__ == "__main__":
    unittest.main()
