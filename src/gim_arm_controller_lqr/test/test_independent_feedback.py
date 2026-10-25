"""Feedback isolation on a plant with nonzero inertia/stiffness cross terms."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest
import yaml
from scipy.linalg import solve_continuous_are, solve_discrete_are

from gim_arm_controller_lqr.controller import LqrController, LqrWeights


class CoupledDynamics:
    nq = 3
    q_min = np.full(3, -1.)
    q_max = np.full(3, 1.)
    tau_max = np.array([5., 40., 5.])
    joint_names = ('base_joint', 'shoulder_joint', 'elbow_joint')
    mass = np.array([[1., .3, .1], [.3, 2., .4], [.1, .4, 1.5]])
    stiffness = np.array([[2., .4, .2], [.4, 3., .5], [.2, .5, 1.]])
    damping = np.array([[.2, .03, .01], [.03, .3, .04], [.01, .04, .15]])

    def mass_matrix(self, q):
        return self.mass

    def gravity(self, q):
        return self.stiffness @ q

    def coriolis(self, q, qd):
        return self.damping @ qd

    def inverse_dynamics(self, q, qd, qdd):
        return self.mass @ qdd + self.gravity(q) + self.coriolis(q, qd)


def controller(discrete=True, **kwargs):
    return LqrController(CoupledDynamics(), feedback_model_mode='independent',
                         use_discrete_lqr=discrete, gravity_at_measured=False,
                         weights=kwargs.pop('weights', LqrWeights(
                             max_int_e=(.1, .2, .3), max_e=(.05, .06, .07))),
                         **kwargs)


@pytest.mark.parametrize('discrete', [False, True])
@pytest.mark.parametrize('design', ['position_held', 'free_torque'])
def test_each_gain_equals_its_three_state_scalar_input_lqr(discrete, design):
    c = controller(discrete, independent_design_model=design)
    A, B = c.linearize(np.zeros(3), np.zeros(3))
    if design == 'position_held':
        inverse = np.diag(1 / np.diag(c.dyn.mass))
        stiffness, damping = c.dyn.stiffness, c.dyn.damping
    else:
        inverse = np.linalg.inv(c.dyn.mass)
        stiffness, damping = c.dyn.stiffness, c.dyn.damping
    np.testing.assert_allclose(B[6:], np.diag(np.diag(inverse)))
    mask = np.ones((3, 9), dtype=bool)
    for i in range(3):
        columns = [i, 3+i, 6+i]
        mask[i, columns] = False
        a = np.array([[0., 1., 0.], [0., 0., 1.],
                      [0., -(inverse @ stiffness)[i, i],
                       -(inverse @ damping)[i, i]]])
        b = np.array([[0.], [0.], [inverse[i, i]]])
        q = c.Q[np.ix_(columns, columns)]
        r = c.R[i:i+1, i:i+1]
        if discrete:
            ad, bd = c._zoh(a, b)
            p = solve_discrete_are(ad, bd, q/100., r/100.)
            expected = np.linalg.solve(r/100. + bd.T @ p @ bd, bd.T @ p @ ad)
        else:
            p = solve_continuous_are(a, b, q, r)
            expected = np.linalg.solve(r, b.T @ p)
        np.testing.assert_allclose(c.K[i, columns], expected[0], rtol=1e-9)
    np.testing.assert_array_equal(c.K[mask], 0.)


@pytest.mark.parametrize('schedule', ['online', 'trajectory_time', 'reference_state'])
@pytest.mark.parametrize('design', ['position_held', 'free_torque'])
def test_other_joint_errors_and_integrals_cannot_change_selected_feedback(schedule, design):
    controllers = [controller(gain_schedule_mode=(
        'reference_state' if schedule == 'online' else schedule),
        independent_design_model=design) for _ in range(2)]
    zero = np.zeros(3)
    reference = np.array([.1, .2, .3])
    qd_ref = np.array([.01, .02, .03])
    qdd = np.array([.1, -.2, .3])
    if schedule != 'online':
        for c in controllers:
            c.precompute_gain_schedule(np.array([reference, reference+.01]),
                                       np.array([qd_ref, qd_ref]), [0., 1.])
            c.set_reference_time(.4)
    for selected in range(3):
        for c in controllers:
            c.reset()
        for _ in range(4):
            controllers[0].compute(reference+.001, qd_ref+.002,
                                   reference, qd_ref, qdd, .01)
            q = reference+.02
            qd = qd_ref-.03
            q[selected], qd[selected] = reference[selected]+.001, qd_ref[selected]+.002
            controllers[1].compute(q, qd, reference, qd_ref, qdd, .01)
            for key in ('tau_fb', 'tau_p', 'tau_i', 'tau_d', 'tau_raw', 'tau'):
                assert controllers[0].last[key][selected] == pytest.approx(
                    controllers[1].last[key][selected], abs=1e-12)
            for c in controllers:
                np.testing.assert_allclose(c.last['tau_ff'],
                    c.dyn.inverse_dynamics(reference, qd_ref, qdd))
                np.testing.assert_allclose(c.last['tau_raw'],
                    c.last['tau_ff'] + c.last['tau_p'] + c.last['tau_i'] + c.last['tau_d'])


def test_other_joint_weights_do_not_change_elbow_gain():
    a = controller()
    b = controller(weights=LqrWeights(max_int_e=(.001, .9, .3),
                                      max_e=(.5, .6, .07)))
    np.testing.assert_allclose(a.K[2], b.K[2], atol=1e-12)


def test_legacy_coupled_default_remains_a_full_gain():
    c = LqrController(CoupledDynamics(), use_discrete_lqr=True)
    A, B = c.linearize(np.zeros(3), np.zeros(3))
    np.testing.assert_allclose(B[6:], np.linalg.inv(c.dyn.mass))
    assert c.feedback_model_mode == 'coupled'
    assert abs(c.K[0, 4]) > 1e-5


def test_saturation_slew_and_integral_limits_remain_per_joint():
    c = controller(tau_limit=[.2, .3, .4], tau_rate_limit=[1., 2., 3.], i_limit=.004)
    zero = np.zeros(3)
    first = c.compute(np.ones(3), zero, zero, zero, zero, .01)
    second = c.compute(-np.ones(3), zero, zero, zero, zero, .01)
    assert np.all(abs(first) <= c.tau_limit + 1e-12)
    assert np.all(abs(second) <= c.tau_limit + 1e-12)
    assert np.all(abs(second-first) <= c.tau_rate_limit*.01 + 1e-12)
    assert np.all(abs(c.integral) <= c.i_limit)


def test_mode_rejects_unknown_value():
    with pytest.raises(ValueError, match='feedback_model_mode'):
        LqrController(CoupledDynamics(), feedback_model_mode='diagonalize_later')


@pytest.mark.parametrize('options', [
    {'independent_design_model': 'unknown'},
    {'independent_design_model': 'free_torque', 'isolated_joint_index': 1},
])
def test_free_design_cannot_be_misused_for_position_held_trials(options):
    with pytest.raises(ValueError, match='independent_design_model'):
        controller(**options)


def test_unstable_gain_schedule_is_rejected_before_installing_table():
    c = controller(gain_schedule_mode='trajectory_time')
    with patch.object(c, 'gain', return_value=c.K*1000):
        with pytest.raises(ValueError, match='mô hình robot ghép ZOH'):
            c.precompute_gain_schedule(np.zeros((2, 3)), np.zeros((2, 3)), [0., 1.])
    assert c._gain_schedule_gains is None


def test_old_coupled_tuning_is_rejected_if_full_plant_is_unstable():
    from gim_control.arm_dynamics import ArmDynamics
    root = Path(__file__).resolve().parents[3]
    dyn = ArmDynamics(str(root/'src/gim_arm_description/urdf/gim_arm.urdf'))
    old = LqrWeights(max_int_e=(.040824829046386304, .05773502691896257, .05773502691896257),
                     max_e=(.025, 1/30, .022360679774997897),
                     max_de=(1., 1/np.sqrt(2), 1.),
                     max_tau=(7.155417527999327, 40., 7.155417527999327))
    diagnostic = LqrController(dyn, weights=old, feedback_model_mode='independent',
                              use_discrete_lqr=True, require_discrete_stable=False)
    report = diagnostic.stability_report(samples=24)
    assert report['design_spectral_radius'] < 1
    assert report['plant_spectral_radius'] > 1
    with pytest.raises(ValueError, match='PHÂN KỲ'):
        LqrController(dyn, weights=old, feedback_model_mode='independent',
                      use_discrete_lqr=True)
    isolated = LqrController(dyn, weights=old, feedback_model_mode='independent',
                             use_discrete_lqr=True, isolated_joint_index=1,
                             gain_schedule_mode='trajectory_time')
    assert isolated.stability_report()['stable_discrete']
    assert 'cô lập 1' in isolated.stability_report()['validation_scope']
    isolated.precompute_gain_schedule(np.array([[0., 0., 0.], [0., .5, 0.]]),
                                     np.zeros((2, 3)), [0., 1.])


def test_isolated_validation_uses_constrained_inertia_not_free_coupled_inverse():
    c = controller(isolated_joint_index=1)
    q, qd = np.zeros(3), np.zeros(3)
    i = [1, 4, 7]
    A, B = c.linearize(q, qd)
    Ad, Bd = c._zoh(A[np.ix_(i, i)], B[i, 1:2])
    expected = np.max(abs(np.linalg.eigvals(Ad-Bd@c.K[1:2, i])))
    assert c._plant_spectral_radius(q, qd, c.K) == pytest.approx(expected)


@pytest.mark.parametrize('index', [-2, 3, 1.5])
def test_isolated_index_rejects_invalid_values(index):
    with pytest.raises(ValueError, match='isolated_joint_index'):
        controller(isolated_joint_index=index)


def test_full_trajectory_cannot_load_isolated_validation_from_trial_snapshot():
    from gim_arm_controller_lqr.factory import LqrFactory
    values = {'isolated_joint_index': 1, 'diagnostic_hold': False}
    node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=values[name]))
    with pytest.raises(ValueError, match='quỹ đạo đầy đủ'):
        LqrFactory().build(node, CoupledDynamics(), [5, 40, 5], 100)


@pytest.mark.parametrize('isolated_index', [-1, 0, 1, 2])
@pytest.mark.parametrize('profile', ['lqr_joint_tuning.yaml', 'lqr_circle_hardware.yaml'])
def test_tuned_hardware_profile_validates_the_requested_plant(isolated_index, profile):
    from gim_control.arm_dynamics import ArmDynamics
    from gim_arm_controller_lqr.factory import LqrFactory
    root = Path(__file__).resolve().parents[3]
    source = root/'src/gim_arm_controller_lqr/config'/profile
    parameters = yaml.safe_load(source.read_text())['lqr_controller']['ros__parameters']
    assert parameters['require_discrete_stable']
    assert 'isolated_joint_index' not in parameters
    parameters.update(isolated_joint_index=isolated_index, diagnostic_hold=True)

    def declare(name, default):
        parameters.setdefault(name, default)
        return SimpleNamespace(value=parameters[name])

    node = SimpleNamespace(declare_parameter=declare,
                           get_parameter=lambda n: SimpleNamespace(value=parameters[n]))
    factory = LqrFactory()
    factory.declare_parameters(node)
    dyn = ArmDynamics(str(root/'src/gim_arm_description/urdf/gim_arm.urdf'))
    if isolated_index >= 0 and profile == 'lqr_circle_hardware.yaml':
        with pytest.raises(ValueError, match='free_torque'):
            factory.build(node, dyn, dyn.tau_max*parameters['tau_scale'], parameters['control_hz'])
        return
    if isolated_index == -1 and profile == 'lqr_joint_tuning.yaml':
        with pytest.raises(ValueError, match='PHÂN KỲ'):
            factory.build(node, dyn, dyn.tau_max*parameters['tau_scale'], parameters['control_hz'])
        return
    c = factory.build(node, dyn, dyn.tau_max*parameters['tau_scale'], parameters['control_hz'])
    assert c.feedback_model_mode == 'independent'
    report = c.stability_report(samples=8)
    assert report['stable_discrete']
    assert report['plant_spectral_radius'] < 1
    assert report['design_spectral_radius'] < 1
    assert (f'cô lập {isolated_index}' if isolated_index >= 0 else 'robot ghép') in report['validation_scope']
    assert 'feedback_model_mode=independent' in c.describe()
