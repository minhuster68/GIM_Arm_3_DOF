"""Check per-joint error models, QP isolation and full feedforward."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from scipy.linalg import expm

from gim_arm_controller_mpc.factory import MpcFactory


class CoupledDynamics:
    nq = 3
    joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint']
    tau_max = np.array([5., 40., 5.])
    mass = np.array([[.05, .01, -.005], [.01, .16, .02], [-.005, .02, .04]])
    stiffness = np.array([[1., .7, -.2], [.7, 2., .3], [-.2, .3, 1.5]])
    damping = np.array([[.05, .02, 0.], [.02, .2, .03], [0., .03, .04]])

    def mass_matrix(self, q):
        return self.mass

    def inverse_dynamics(self, q, qd, qdd):
        return self.mass @ qdd + self.stiffness @ q + self.damping @ qd


def controller(**overrides):
    path = Path(__file__).resolve().parents[1] / 'config/mpc_circle_independent_feedback.yaml'
    p = yaml.safe_load(path.read_text())['mpc_controller']['ros__parameters']
    p.update(prediction_horizon=12, control_horizon=4,
             feedback_bounds_mode='instantaneous', precompute_prediction=False)
    p.update(overrides)
    node = SimpleNamespace(
        declare_parameter=lambda name, default: p.setdefault(name, default),
        get_parameter=lambda name: SimpleNamespace(value=p[name]))
    factory = MpcFactory()
    factory.declare_parameters(node)
    return factory.build(node, CoupledDynamics(), CoupledDynamics.tau_max, 100.)


def test_diagonal_error_model_is_separable_and_uses_inverse_of_diagonal_mass():
    c = controller()
    q, v, a = np.array([.03, .04, .05]), np.ones(3)*.01, np.ones(3)*.02
    ad, bd = c._linearize(q, v, a)
    h, f = c._prediction_terms(ad, bd)
    for j in range(3):
        states = np.array([j, 3+j, 6+j])
        other_states = np.setdiff1d(np.arange(9), states)
        moves = np.arange(c.nc)*3+j
        other_moves = np.setdiff1d(np.arange(c.nc*3), moves)
        np.testing.assert_allclose(ad[np.ix_(states, other_states)], 0., atol=1e-14)
        np.testing.assert_allclose(bd[np.ix_(states, np.setdiff1d(np.arange(3), [j]))], 0., atol=1e-14)
        np.testing.assert_allclose(h[np.ix_(moves, other_moves)], 0., atol=1e-12)
        np.testing.assert_allclose(f[np.ix_(moves, other_states)], 0., atol=1e-12)
        m = c.dyn.mass[j, j]
        continuous = np.array([[0., 1., 0.],
                               [-c.dyn.stiffness[j, j]/m, -c.dyn.damping[j, j]/m, 1./m],
                               [0., 0., 0.]])
        expected = expm(continuous*c.dt)
        mech = [3+j, 6+j]
        np.testing.assert_allclose(ad[np.ix_(mech, mech)], expected[:2, :2], atol=1e-11)
        np.testing.assert_allclose(bd[mech, j], expected[:2, 2], atol=1e-11)


@pytest.mark.parametrize('solver', ['active_set', 'admm', 'slsqp'])
@pytest.mark.parametrize('selected', [0, 1, 2])
def test_other_joint_errors_cannot_change_selected_feedback_or_warm_start(solver, selected):
    baseline, changed = controller(qp_solver=solver), controller(qp_solver=solver)
    qr, vr, ar = np.array([.03, .04, .05]), np.array([.01, .02, -.01]), np.array([.02, -.01, .03])
    other = np.arange(3) != selected
    for _ in range(6):
        q, v = qr.copy(), vr.copy()
        q[selected] += .02
        qp, vp = q.copy(), v.copy()
        qp[other] += [.4, -.4]
        vp[other] += [.2, -.2]
        previous = changed.previous_feedback.copy()
        baseline.compute(q, v, qr, vr, ar, .01)
        tau = changed.compute(qp, vp, qr, vr, ar, .01)
        expected_ff = changed.dyn.inverse_dynamics(qr, vr, ar)
        expected_ff[0] *= changed.base_ff_alpha
        np.testing.assert_allclose(changed.last['tau_ff'], expected_ff)
        np.testing.assert_allclose(changed.previous_feedback[selected], baseline.previous_feedback[selected], atol=1e-10)
        np.testing.assert_allclose(changed.warm[:, selected], baseline.warm[:, selected], atol=1e-10)
        np.testing.assert_allclose(changed.integral[selected], baseline.integral[selected], atol=1e-14)
        # SLSQP feasibility uses the controller's 1e-7 acceptance tolerance.
        assert np.all(abs(changed.previous_feedback-previous) <= changed.slew*changed.dt+1e-7)
        assert np.all(abs(tau) <= changed.tau_limit+1e-10)
        assert changed.last['constraint_violation'] <= 1e-7


def test_rejected_joint_candidate_does_not_force_other_joints_to_fallback():
    baseline, failed = controller(), controller()
    solve = failed._solve_active_set
    calls = []

    def reject_shoulder(*args, **kwargs):
        j = len(calls) % 3
        calls.append(j)
        if j == 1:
            return np.full(failed.nc, np.nan), 1, False
        return solve(*args, **kwargs)

    failed._solve_active_set = reject_shoulder
    zero = np.zeros(3)
    for _ in range(3):
        q = np.array([.03, -.02, .01])
        baseline.compute(q, zero, zero, zero, zero, .01)
        failed.compute(q, zero, zero, zero, zero, .01)
        np.testing.assert_allclose(failed.previous_feedback[[0, 2]], baseline.previous_feedback[[0, 2]])
        np.testing.assert_array_equal(failed.last['solver_accepted_joints'], [True, False, True])
        assert not failed.last['solver_accepted']
        assert np.all(np.isfinite(failed.last['tau']))


def test_cached_independent_predictions_match_online_and_keep_global_ff_bounds():
    cached = controller(precompute_prediction=True, feedback_bounds_mode='trajectory_global')
    online = controller(feedback_bounds_mode='trajectory_global')
    q = np.array([[.01, .02, .03], [.02, .03, .04]])
    v, a = q*.1, q*.2
    for c in (cached, online):
        c.prepare_reference(q, v, a, [0., 1.])
    for k in [0, 1, 0]:
        cached.set_reference_time(float(k))
        actual = cached.compute(q[k]+.01, v[k], q[k], v[k], a[k], .01)
        expected = online.compute(q[k]+.01, v[k], q[k], v[k], a[k], .01)
        np.testing.assert_allclose(actual, expected, atol=1e-10)
        assert np.all(abs(actual) <= cached.tau_limit+1e-10)


def test_default_mode_preserves_coupled_response_and_bad_mode_is_rejected():
    full = controller(feedback_model_mode='coupled')
    zero = np.zeros(3)
    full.compute([0., .1, 0.], zero, zero, zero, zero, .01)
    assert abs(full.previous_feedback[2]) > 1e-5
    with pytest.raises(ValueError, match='feedback_model_mode'):
        controller(feedback_model_mode='typo')


@pytest.mark.parametrize('bad', [0., -1., np.nan, np.inf])
def test_independent_model_rejects_bad_diagonal_mass(bad):
    c = controller()
    c.dyn = CoupledDynamics()
    c.dyn.mass = c.dyn.mass.copy()
    c.dyn.mass[2, 2] = bad
    with pytest.raises(ValueError, match='M_ii'):
        c._linearize(np.zeros(3), np.zeros(3), np.zeros(3))
