"""Check STA state timing, torque constraints and disturbed-plant tracking."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from gim_arm_controller_smc.factory import SmcFactory
from gim_arm_controller_smc.super_twisting import SuperTwistingController
from gim_arm_controller_smc.velocity_kalman import VelocityKalman2


class Dynamics:
    nq = 3
    joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint']
    tau_max = np.full(3, 100.)
    matrix = np.array([[1., .2, -.1], [.2, 2., .3], [-.1, .3, 1.]])

    def mass_matrix(self, q):
        return self.matrix

    def inverse_dynamics(self, q, qd, qdd):
        return self.matrix @ qdd + np.array([.3, -.2, .1])

    def gravity(self, q):
        return np.array([.3, -.2, .1])


def controller(**kwargs):
    return SuperTwistingController(Dynamics(), [4., 5., 6.], [3., 4., 5.],
                                   [2., 3., 4.], [10., 10., 10.], **kwargs)


def test_computed_torque_and_explicit_euler_state_timing():
    c = controller()
    q = np.array([.1, -.2, .05]); v = np.array([.02, -.01, .03])
    qr = np.array([.03, .01, -.02]); vr = np.array([.01, .02, -.01])
    ar = np.array([.2, -.1, .1]); dt = .007
    s = v - vr + c.lambda_gain * (q - qr)
    a = -c.lambda_gain * (v - vr) - c.k1 * np.sqrt(abs(s)) * np.sign(s)
    np.testing.assert_allclose(c.compute(q, v, qr, vr, ar, dt),
                               c.dyn.inverse_dynamics(qr, vr, ar) + c.dyn.mass_matrix(q) @ a)
    np.testing.assert_allclose(c.last['integral'], 0)
    np.testing.assert_allclose(c.z, -c.k2 * np.sign(s) * dt)
    a += c.z
    np.testing.assert_allclose(c.compute(q, v, qr, vr, ar, dt),
                               c.dyn.inverse_dynamics(qr, vr, ar) + c.dyn.mass_matrix(q) @ a)
    c.reset()
    np.testing.assert_allclose(c.z, 0)
    np.testing.assert_allclose(c.previous_feedback, 0)
    assert c.sampled_spectral_radius() is None


def test_held_joints_do_not_accumulate_sta_state():
    c = controller(controlled_joint='elbow')
    c.compute([.2, -.3, .1], [.1, .2, .03], np.zeros(3), np.zeros(3), np.ones(3), .01)
    np.testing.assert_allclose(c.z[:2], 0)
    np.testing.assert_allclose(c.last['virtual_acceleration'][:2], 0)


@pytest.mark.parametrize('selected', ['all', 'elbow'])
@pytest.mark.parametrize('scales', [[1., .8, .5], [0., 0., 0.]])
@pytest.mark.parametrize('constraints', [{}, {'torque_slew_rate': [.1, .2, .3]},
                                       {'tau_limit': [.4, .4, .4]}])
def test_torque_components_include_coupling_current_integral_and_constraints(selected, scales, constraints):
    c = controller(controlled_joint=selected, feedback_offdiag_scale=scales,
                   friction_ff_nm=[0., 0., .1], **constraints)
    q = np.array([.1, -.2, .05]); velocity = np.array([.02, -.01, .03])
    zero = np.zeros(3)
    effective = np.diag(np.diag(c.dyn.matrix)) + c.feedback_offdiag_scale[:, None] * (
        c.dyn.matrix - np.diag(np.diag(c.dyn.matrix)))
    for _ in range(8):
        before = c.z.copy()
        tau = c.compute(q, velocity, zero, zero, zero, .01)
        np.testing.assert_allclose(c.last['tau_lambda'], effective @ np.where(
            c.active, -c.lambda_gain * c.last['error_rate'], 0.))
        np.testing.assert_allclose(c.last['tau_k1'], effective @ np.where(
            c.active, -c.k1 * np.sqrt(abs(c.last['surface'])) * np.sign(c.last['surface']), 0.))
        np.testing.assert_allclose(c.last['tau_k2'], effective @ np.where(c.active, before, 0.))
        components = sum(c.last[key] for key in ('tau_lambda', 'tau_k1', 'tau_k2'))
        np.testing.assert_allclose(components, c.last['tau_feedback_raw'], atol=1e-14)
        np.testing.assert_allclose(components + c.last['tau_slew'], c.last['tau_fb'], atol=1e-14)
        correction = tau - c.last['tau_raw']
        np.testing.assert_allclose(c.last['tau_ff'] + components + c.last['tau_slew']
                                   + correction, tau, atol=1e-14)
        np.testing.assert_allclose(c.last['tau_feedback_self'] + c.last['tau_feedback_cross'],
                                   c.last['tau_feedback_raw'], atol=1e-14)
        if not np.any(scales):
            np.testing.assert_allclose(c.last['tau_feedback_cross'], 0., atol=1e-14)


@pytest.mark.parametrize('output_index', [0, 1, 2])
def test_independent_feedback_and_integrator_ignore_other_joint_errors_and_clipping(output_index):
    limits = np.full(3, .6)
    limits[output_index] = 5.
    kwargs = dict(feedback_offdiag_scale=[0., 0., 0.], tau_limit=limits,
                  torque_slew_rate=[6., 16., 1.5])
    baseline, perturbed = controller(**kwargs), controller(**kwargs)
    zero = np.zeros(3)
    qr = np.array([.03, -.02, .01])
    ar = np.array([.2, -.1, .1])
    q = qr.copy()
    q[output_index] += .001
    other = np.arange(3) != output_index
    changed = q.copy()
    changed[other] += [5., -5.]
    clipped_other = False
    for _ in range(60):
        baseline.compute(q, zero, qr, zero, ar, .01)
        perturbed.compute(changed, zero, qr, zero, ar, .01)
        # Non-diagonal M remains in reference FF, even though FB is diagonal.
        np.testing.assert_allclose(perturbed.last['tau_ff'], perturbed.dyn.inverse_dynamics(qr, zero, ar))
        for key in ('tau_ff', 'tau_fb', 'tau_lambda', 'tau_k1', 'tau_k2',
                    'tau_slew', 'tau', 'integral', 'integral_blocked'):
            np.testing.assert_allclose(perturbed.last[key][output_index],
                                       baseline.last[key][output_index], atol=1e-14)
        np.testing.assert_allclose(perturbed.z[output_index], baseline.z[output_index])
        np.testing.assert_allclose(perturbed.last['tau_feedback_cross'], 0., atol=1e-14)
        clipped_other |= bool(perturbed.last['saturated'][other].any())
    assert clipped_other
    assert abs(perturbed.z[output_index]) > 0.


def test_circle_profile_factory_selects_independent_feedback():
    params = {}
    node = SimpleNamespace(
        declare_parameter=lambda name, value: params.setdefault(name, value),
        get_parameter=lambda name: SimpleNamespace(value=params[name]))
    factory = SmcFactory()
    factory.declare_parameters(node)
    path = Path(__file__).resolve().parents[1] / 'config/smc_circle_from_terminal_tuning.yaml'
    params.update(yaml.safe_load(path.read_text())['smc_controller']['ros__parameters'])
    c = factory.build(node, Dynamics(), Dynamics.tau_max, params['control_hz'])
    assert c.controlled_joint == 'all'
    np.testing.assert_array_equal(c.feedback_offdiag_scale, 0.)
    assert 'diag(M(q_measured))' in c.describe()
    zero = np.zeros(3)
    c.compute([.01, -.02, .03], zero, zero, zero, zero, .01)
    np.testing.assert_allclose(c.last['tau_feedback_cross'], 0., atol=1e-14)


@pytest.mark.parametrize('limits', [dict(tau_limit=[.4, .4, .4]),
                                  dict(torque_slew_rate=[.1, .1, .1])])
def test_constraints_block_integral_growth(limits):
    c = controller(**limits)
    for _ in range(500):
        tau = c.compute(np.ones(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), .01)
        assert np.all(abs(tau) <= c.tau_limit)
        assert np.all(abs(c.z) <= c.z_limit)
    np.testing.assert_allclose(c.z, 0)
    assert c.last['integral_blocked'].all()


@pytest.mark.parametrize('dt', [0., -.01, float('nan'), float('inf')])
def test_bad_timestep_leaves_state_unchanged(dt):
    c = controller()
    with pytest.raises(ValueError, match='dt'):
        c.compute(np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), dt)
    np.testing.assert_allclose(c.z, 0)


def test_velocity_lpf_is_per_joint_and_resets_between_trials():
    c = controller(velocity_lpf_hz=[0., 0., 3.])
    zero = np.zeros(3)
    noisy = np.array([.2, -.3, .4])
    c.compute(zero, noisy, zero, zero, zero, .01)
    alpha = 1. - np.exp(-2. * np.pi * 3. * .01)
    np.testing.assert_allclose(c.last['qd_filtered'], [.2, -.3, alpha * .4])
    c.compute(zero, zero, zero, zero, zero, .01)
    np.testing.assert_allclose(c.last['qd_filtered'], [0., 0., alpha * .4 * (1. - alpha)])
    c.reset()
    np.testing.assert_allclose(c.filtered_qd, 0.)
    with pytest.raises(ValueError, match='velocity_lpf_hz'):
        controller(velocity_lpf_hz=[0., -1., 3.])


def test_kalman_velocity_reaches_feedback_and_preserves_raw_and_reference():
    c = controller(velocity_kalman_enabled=[False, False, True])
    baseline = controller()
    zero = np.zeros(3)
    q = np.array([.1, -.1, .05])
    reference_velocity = np.array([.1, -.1, .02])
    for stamp, raw in ((1_000_000_000, .1), (1_020_000_000, -.1)):
        velocity = np.array([.2, -.3, raw])
        before, reference_before = velocity.copy(), reference_velocity.copy()
        c.set_velocity_sample_time(stamp)
        tau = c.compute(q, velocity, zero, reference_velocity, zero, .01)
        expected = baseline.compute(q, c.last['qd_filtered'], zero, reference_velocity, zero, .01)
        np.testing.assert_allclose(tau, expected)
        np.testing.assert_array_equal(velocity, before)
        np.testing.assert_array_equal(reference_velocity, reference_before)
        np.testing.assert_array_equal(c.last['qd_filtered'][:2], velocity[:2])
    assert abs(c.last['qd_filtered'][2] - velocity[2]) > .01
    c.reset()
    c.set_velocity_sample_time(2_000_000_000)
    c.compute(zero, [.1, .2, .8], zero, zero, zero, .01)
    np.testing.assert_array_equal(c.last['qd_filtered'], [.1, .2, .8])


def test_kalman_uses_consumed_state_time_and_ignores_duplicate_sample():
    c = controller(velocity_kalman_enabled=[False, False, True])
    f = VelocityKalman2(3, [False, False, True])
    zero = np.zeros(3)
    for stamp, sample, sample_dt in ((1_000_000_000, .1, .01), (1_030_000_000, -.1, .03)):
        c.set_velocity_sample_time(stamp)
        c.compute(zero, [0., 0., sample], zero, zero, zero, .01)
        np.testing.assert_allclose(c.last['qd_filtered'], f.update([0., 0., sample], sample_dt))
    covariance = c.velocity_kalman.covariance.copy()
    filtered = c.last['qd_filtered'].copy()
    c.compute(zero, [0., 0., -.1], zero, zero, zero, .01)
    np.testing.assert_array_equal(c.velocity_kalman.covariance, covariance)
    np.testing.assert_array_equal(c.last['qd_filtered'], filtered)
    for stamp in (2_000_000_000, 500_000_000):
        c.set_velocity_sample_time(stamp)  # gap, then clock rewind
        c.compute(zero, [0., 0., .5], zero, zero, zero, .01)
        assert c.last['qd_filtered'][2] == .5
        assert c.velocity_kalman.state[2, 1] == 0.


def test_kalman_and_lpf_cannot_be_stacked_on_same_joint():
    with pytest.raises(ValueError, match='LPF và Kalman'):
        controller(velocity_kalman_enabled=[False, False, True], velocity_lpf_hz=[0., 0., 3.])


def test_reference_direction_friction_feedforward_is_smooth_and_joint_scoped():
    base = controller(controlled_joint='elbow')
    boosted = controller(controlled_joint='elbow', friction_ff_nm=[0., 0., .2],
                         friction_transition_rad_s=.005)
    zero = np.zeros(3)
    for velocity in (0., .005, -.005):
        reference_velocity = np.array([0., 0., velocity])
        base.reset(); boosted.reset()
        tau_base = base.compute(zero, zero, zero, reference_velocity, zero, .01)
        tau_boosted = boosted.compute(zero, zero, zero, reference_velocity, zero, .01)
        expected = .2 * np.tanh(velocity / .005)
        np.testing.assert_allclose(tau_boosted - tau_base, [0., 0., expected])
        np.testing.assert_allclose(boosted.last['tau_friction'], [0., 0., expected])
        np.testing.assert_allclose(boosted.last['tau_ff'] + boosted.last['tau_fb'], tau_boosted)
    with pytest.raises(ValueError, match='friction_transition_rad_s'):
        controller(friction_transition_rad_s=0.)


def test_factory_loads_sta_profile_and_rejects_bad_gains():
    class Node:
        def __init__(self):
            self.params = {}

        def declare_parameter(self, name, value):
            self.params[name] = value

        def get_parameter(self, name):
            return SimpleNamespace(value=self.params[name])

    node = Node()
    factory = SmcFactory()
    factory.declare_parameters(node)
    config = Path(__file__).resolve().parents[1] / 'config/smc_super_twisting_gazebo.yaml'
    node.params.update(yaml.safe_load(config.read_text())['smc_controller']['ros__parameters'])
    c = factory.build(node, Dynamics(), Dynamics.tau_max, node.params['control_hz'])
    assert isinstance(c, SuperTwistingController)
    assert c.dt == .001
    assert not c.velocity_kalman.enabled.any()
    node.params.update(velocity_kalman_enabled=[False, False, True],
                       velocity_kalman_jerk_psd=[.2, .3, .4],
                       velocity_kalman_measurement_variance=[.001, .002, .003])
    c = factory.build(node, Dynamics(), Dynamics.tau_max, node.params['control_hz'])
    np.testing.assert_array_equal(c.velocity_kalman.enabled, [False, False, True])
    np.testing.assert_allclose(c.velocity_kalman.jerk_psd, [.2, .3, .4])
    np.testing.assert_allclose(c.velocity_kalman.measurement_variance, [.001, .002, .003])
    node.params['k1'] = [float('nan'), 1., 1.]
    with pytest.raises(ValueError, match='k1'):
        factory.build(node, Dynamics(), Dynamics.tau_max, 100.)
    node.params['smc_variant'] = 'typo'
    with pytest.raises(ValueError, match='smc_variant'):
        factory.build(node, Dynamics(), Dynamics.tau_max, 100.)


def test_constant_matched_disturbance_converges_with_sampled_sta():
    c = controller()
    q = np.array([.1, -.08, .06]); v = np.zeros(3); zero = np.zeros(3)
    disturbance = np.array([.3, -.4, .2])  # acceleration disturbance
    tail = []
    for k in range(3000):
        tau = c.compute(q, v, zero, zero, zero, .01)
        acceleration = np.linalg.solve(c.dyn.matrix, tau - [.3, -.2, .1]) + disturbance
        q += .01*v + .5*.01**2*acceleration
        v += .01*acceleration
        if k > 2500:
            tail.append(q.copy())
    assert np.max(np.abs(tail)) < .002
    np.testing.assert_allclose(c.z, -disturbance, atol=.15)
