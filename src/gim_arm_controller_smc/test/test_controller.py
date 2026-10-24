"""Verify computed-torque law, held-joint isolation and output limits."""
from types import SimpleNamespace

import numpy as np
import pytest

from gim_arm_controller_smc.controller import SmcController
from gim_arm_controller_smc.factory import SmcFactory
from gim_arm_controller_smc.super_twisting import SuperTwistingController


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


def test_default_law_matches_reference_feedforward_and_measured_feedback_sum():
    c = controller()
    q = np.array([.1, .2, -.1]); qd = np.array([.03, -.02, .01])
    qref = np.array([.08, .22, -.12]); vref = np.zeros(3); aref = np.array([.2, -.1, .1])
    s = qd + c.lambda_gain * (q - qref)
    acc = -c.lambda_gain * qd - c.ks*s - c.kr*np.clip(s/c.phi, -1, 1)
    expected = np.clip(c.dyn.inverse_dynamics(qref, vref, aref)
                       + c.dyn.mass_matrix(q) @ acc, -c.tau_limit, c.tau_limit)
    np.testing.assert_allclose(c.compute(q, qd, qref, vref, aref, .01), expected)
    np.testing.assert_allclose(c.last['tau_raw'], c.last['tau_ff'] + c.last['tau_fb'])


@pytest.mark.parametrize('variant', ['saturation', 'super_twisting'])
@pytest.mark.parametrize('selected', ['all', 'elbow'])
def test_reference_ff_is_independent_of_measurement_and_filter_while_fb_uses_measured_mass(variant, selected):
    class StateDependentDynamics(Dynamics):
        def mass_matrix(self, q):
            return self.matrix + np.diag(.1 + .05 * np.sin(np.asarray(q)))

        def inverse_dynamics(self, q, qd, qdd):
            return (self.mass_matrix(q) @ qdd + np.sin(q)
                    + .3 * np.asarray(qd) * np.abs(qd))

    dyn = StateDependentDynamics()
    common = dict(lambda_gain=[2., 3., 4.], controlled_joint=selected,
                  base_ff_alpha=2.46, tau_limit=[100.] * 3)
    if variant == 'super_twisting':
        c = SuperTwistingController(dyn, k1=[1.] * 3, k2=[1.] * 3,
                                   z_limit=[10.] * 3,
                                   velocity_kalman_enabled=[False, False, True], **common)
    else:
        c = SmcController(dyn, ks=[1.] * 3, kr=[.1] * 3, phi=[.2] * 3, **common)
    qr = np.array([-.1, .2, .1]); vr = np.array([.02, .03, .04]); ar = np.array([.1, .2, -.1])
    expected_ff = dyn.inverse_dynamics(qr, vr, np.where(c.active, ar, 0.))
    expected_ff[0] *= 2.46
    feedback = []
    for q, velocity in (([.3, .4, .8], [.7, -.4, 1.]), ([-.3, .5, .9], [-.5, .6, -1.])):
        c.compute(q, velocity, qr, vr, ar, .01)
        np.testing.assert_allclose(c.last['tau_ff'], expected_ff)
        used_velocity = c.last.get('qd_filtered', np.asarray(velocity))
        ed = used_velocity - vr
        np.testing.assert_allclose(c.last['error_rate'], ed)
        np.testing.assert_allclose(c.last['error'], np.asarray(q) - qr)
        if variant == 'super_twisting':
            acceleration = (-c.lambda_gain * ed - c.k1 * np.sqrt(abs(c.last['surface']))
                            * np.sign(c.last['surface']) + c.last['integral'])
        else:
            acceleration = -c.lambda_gain * ed - c.ks*c.last['surface'] - c.kr*c.last['boundary']
        np.testing.assert_allclose(c.last['tau_feedback_raw'],
                                   dyn.mass_matrix(q) @ np.where(c.active, acceleration, 0.))
        feedback.append(c.last['tau_fb'].copy())
    assert not np.allclose(feedback[0], feedback[1])


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


def test_offdiagonal_feedback_can_be_reduced_for_elbow_only():
    q = np.array([.1, -.2, .15])
    zero = np.zeros(3)
    full = controller()
    scaled = controller(feedback_offdiag_scale=[1., 1., .5])
    full.compute(q, zero, zero, zero, zero, .01)
    scaled.compute(q, zero, zero, zero, zero, .01)
    np.testing.assert_allclose(scaled.last['tau_feedback_raw'][:2],
                               full.last['tau_feedback_raw'][:2])
    np.testing.assert_allclose(
        scaled.last['tau_feedback_raw'][2],
        full.last['tau_feedback_self'][2] + .5 * full.last['tau_feedback_cross'][2])
    np.testing.assert_allclose(full.last['tau_feedback_raw'],
                               full.last['tau_feedback_self'] +
                               full.last['tau_feedback_cross'])
    with pytest.raises(ValueError, match='feedback_offdiag_scale'):
        controller(feedback_offdiag_scale=[1., 1., 1.1])


def test_factory_rejects_unstable_sampled_gains_before_activation():
    p = {'lambda_gain':[15,85,155], 'ks':[5,137,180], 'kr':[30,130,90],
         'phi':[.03,.05,.15], 'base_ff_alpha':1.,
         'controlled_joint':'elbow',
         'torque_slew_rate':[5,40,5], 'feedback_offdiag_scale':[1,1,1],
         'smc_variant':'saturation'}
    node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=p[name]))
    with pytest.raises(ValueError, match='không ổn định'):
        SmcFactory().build(node, Dynamics(), Dynamics.tau_max, 100.)
