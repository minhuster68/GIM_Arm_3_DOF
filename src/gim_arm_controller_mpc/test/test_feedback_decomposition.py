"""QP attribution is exact bookkeeping and does not alter motor commands."""
from unittest.mock import patch

import numpy as np
import pytest

from gim_arm_controller_mpc.controller import MpcController


class Dynamics:
    nq = 3
    joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint']
    tau_max = np.ones(3) * 5.0

    def mass_matrix(self, q):
        return np.diag([1., 2., 1.5])

    def inverse_dynamics(self, q, qd, qdd):
        return self.mass_matrix(q) @ qdd + np.asarray(q)


def controller(cached=False, mode='independent', constrained=False):
    c = MpcController(
        Dynamics(), 100., 12, 4, [2.] * 3, [3.] * 3, [1.] * 3,
        [.01] * 3, [.02] * 3, [.5 if constrained else 1.e6] * 3,
        tau_limit=np.ones(3) * (.02 if constrained else 5.),
        qp_solver='active_set', max_iterations=100,
        precompute_prediction=cached, feedback_model_mode=mode)
    zero = np.zeros(3)
    if cached:
        c.prepare_reference(np.zeros((2, 3)), np.zeros((2, 3)),
                            np.zeros((2, 3)), [0., .1])
    return c, zero


@pytest.mark.parametrize('cached', [False, True])
@pytest.mark.parametrize('mode', ['independent', 'coupled'])
@pytest.mark.parametrize('constrained', [False, True])
def test_parts_match_independent_free_qp_and_actual_feedback(cached, mode, constrained):
    c, zero = controller(cached, mode, constrained)
    c.integral = np.array([.001, -.002, .003])
    c.previous_feedback = np.array([.0002, -.0001, .0003])
    q = np.array([.002, -.003, .001]) * (100 if constrained else 1)
    qd = np.array([.001, -.001, .003])
    old_integral, old_previous = c.integral.copy(), c.previous_feedback.copy()
    a, b = c._linearize(zero, zero, zero)
    h, f = c._prediction_terms(a, b)
    state = np.r_[old_integral, q, qd]
    reference_free = -np.linalg.solve(h, f @ state + c._previous_gradient @ old_previous)[:3]
    tau = c.compute(q, qd, zero, zero, zero, .01)
    last = c.last
    names = ['tau_mpc_i_free', 'tau_mpc_q_free', 'tau_mpc_qd_free', 'tau_mpc_prev_free']
    np.testing.assert_allclose(sum(last[k] for k in names), reference_free, atol=1e-10)
    np.testing.assert_allclose(last['tau_mpc_fb_free'], reference_free, atol=1e-10)
    np.testing.assert_allclose(reference_free + last['tau_mpc_correction'],
                               last['tau_feedback'], atol=1e-10)
    np.testing.assert_allclose(last['tau_ff'] + last['tau_feedback']
                               + last['tau_mpc_total_clip'], tau, atol=1e-10)
    if constrained:
        assert np.max(abs(last['tau_mpc_correction'])) > .01
        assert np.all(abs(last['tau_feedback'] - old_previous) <= .005 + 1e-9)
    else:
        np.testing.assert_allclose(last['tau_mpc_correction'], 0., atol=1e-8)
    # The integral contribution uses the state given to the solver, before
    # this tick's integration/clipping (last['integral'] is the updated state).
    gain = -np.linalg.solve(h, f)[:3]
    np.testing.assert_allclose(last['tau_mpc_i_free'], gain[:, :3] @ old_integral)


def test_cached_controller_does_no_extra_gain_solve_in_control_tick():
    c, zero = controller(cached=True)
    with patch.object(c, '_feedback_decomposition_gains', side_effect=AssertionError('live factorization')):
        c.compute(zero, zero, zero, zero, zero, .01)
        c.reset()
        c.compute(zero, zero, zero, zero, zero, .01)


def test_diagnostic_values_do_not_change_torque_solver_history_or_integral():
    actual, zero = controller(cached=True, constrained=True)
    untouched, _ = controller(cached=True, constrained=True)
    with patch.object(untouched, '_feedback_decomposition', return_value={'tau_mpc_correction': np.ones(3)*1.e9}):
        for step in range(8):
            q = np.array([.1, -.2, .3]) * (1 + step / 10)
            tau = actual.compute(q, zero, zero, zero, zero, .01)
            expected = untouched.compute(q, zero, zero, zero, zero, .01)
            np.testing.assert_array_equal(tau, expected)
            np.testing.assert_array_equal(actual.warm, untouched.warm)
            np.testing.assert_array_equal(actual.integral, untouched.integral)
            assert actual.last['solver_accepted'] == untouched.last['solver_accepted']
