"""Verify computed-torque law, held-joint isolation and output limits."""
from types import SimpleNamespace

import numpy as np
import pytest

from gim_arm_controller_smc.controller import SmcController
from gim_arm_controller_smc.factory import SmcFactory


class Dynamics:
    nq = 3
    joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint']
    tau_max = np.array([5., 40., 5.])
    matrix = np.array([[.05, .01, -.005], [.01, .16, .02], [-.005, .02, .04]])

    def mass_matrix(self, q):
        return self.matrix

    def inverse_dynamics(self, q, qd, qdd):
        return self.matrix @ qdd + np.array([.3, -.2, .1])


def controller(**kwargs):
    return SmcController(Dynamics(), [6, 8, 6], [6, 8, 6], [1, 2, 1], [.2, .3, .2],
                         **kwargs)


def test_default_law_matches_original_computed_torque_and_ff_fb_sum():
    c = controller()
    q = np.array([.1, .2, -.1]); qd = np.array([.03, -.02, .01])
    qref = np.array([.08, .22, -.12]); vref = np.zeros(3); aref = np.array([.2, -.1, .1])
    s = qd + c.lambda_gain * (q - qref)
    acc = aref - c.lambda_gain * qd - c.ks*s - c.kr*np.clip(s/c.phi, -1, 1)
    expected = np.clip(c.dyn.inverse_dynamics(q, qd, acc), -c.tau_limit, c.tau_limit)
    np.testing.assert_allclose(c.compute(q, qd, qref, vref, aref, .01), expected)
    np.testing.assert_allclose(c.last['tau_raw'], c.last['tau_ff'] + c.last['tau_fb'])


def test_position_held_joint_errors_do_not_drive_selected_feedback():
    c = controller(controlled_joint='shoulder')
    q = np.array([0., .02, 0.]); v = np.array([0., .03, 0.])
    c.compute(q, v, np.zeros(3), np.zeros(3), np.zeros(3), .01)
    feedback = c.last['tau_fb'].copy()
    q[[0, 2]] = [.1, -.1]; v[[0, 2]] = [.2, -.2]
    c.compute(q, v, np.zeros(3), np.zeros(3), np.zeros(3), .01)
    np.testing.assert_allclose(c.last['tau_fb'], feedback)
    np.testing.assert_allclose(c.last['virtual_acceleration'][[0, 2]], 0)


def test_alpha_only_scales_base_feedforward_and_slew_limits_feedback_steps():
    c = controller(base_ff_alpha=2.2, controlled_joint='elbow', torque_slew_rate=[5, 40, 5])
    previous = np.zeros(3)
    limited = []
    for _ in range(5):
        c.compute([0, 0, .1], np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), .01)
        np.testing.assert_allclose(c.last['tau_ff'], [.66, -.2, .1])
        assert np.all(abs(c.last['tau_fb']-previous) <= np.array([.05, .4, .05])+1e-12)
        limited.append(c.last['slew_limited'][2])
        previous = c.last['tau_fb'].copy()
    assert limited[0]
    assert not limited[-1]
    c.reset()
    np.testing.assert_allclose(c.previous_feedback, 0)


def test_hard_total_torque_limit_wins_even_with_slew():
    c = controller(tau_limit=[.05, .05, .05], torque_slew_rate=[5, 40, 5])
    out = c.compute([1, 1, 1], np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), .01)
    assert np.all(abs(out) <= .05)
    assert c.last['saturated'].any()


def test_factory_rejects_unstable_sampled_gains_before_activation():
    p = {'lambda_gain':[15,85,155], 'ks':[5,137,180], 'kr':[30,130,90],
         'phi':[.03,.05,.15], 'base_ff_alpha':1., 'controlled_joint':'elbow',
         'torque_slew_rate':[5,40,5]}
    node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=p[name]))
    with pytest.raises(ValueError, match='không ổn định'):
        SmcFactory().build(node, Dynamics(), Dynamics.tau_max, 100.)
