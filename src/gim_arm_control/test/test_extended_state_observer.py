"""Discrete LESO equations, disturbance recovery, timing and actuator limits."""
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from gim_control.extended_state_observer import DiscreteLeso, EsoCompensator
from gim_control.eso_runtime import EsoModeGate, EsoRuntime, hardware_all_torque


class Dynamics:
    nq = 3
    mass = np.array([[.1, .02, 0.], [.02, .2, .01], [0., .01, .08]])
    bias = np.array([.1, .2, .05])

    def mass_matrix(self, q):
        return self.mass

    def inverse_dynamics(self, q, v, a):
        return self.mass @ a + self.bias


def compensator(**kw):
    return EsoCompensator(Dynamics(), [5., 40., 5.], [15.]*3, [1.]*3,
                          [.5, 2., .3], [.5, 2., .3], **kw)


@pytest.mark.parametrize('dt', [.001, .01, .017])
def test_matlab_gain_formula_places_three_observer_poles_at_exp_minus_omega_dt(dt):
    o = DiscreteLeso([10., 15., 20.])
    l1, l2, l3 = o.gains(dt)
    f = np.array([[1., dt, .5*dt*dt], [0., 1., dt], [0., 0., 1.]])
    for i, w in enumerate(o.bandwidth):
        correction = np.eye(3) - np.outer([l1[i], l2[i], l3[i]], [1., 0., 0.])
        np.testing.assert_allclose(np.poly(correction @ f), np.poly([np.exp(-w*dt)]*3), atol=1e-13)


@pytest.mark.parametrize('dt', [.001, .01])
def test_recovers_three_constant_disturbance_accelerations_from_position_only(dt):
    o = DiscreteLeso([15.]*3)
    disturbance, nominal = np.array([1., -.5, .3]), np.array([.2, .1, -.1])
    o.seed(np.zeros(3))
    for k in range(1, int(4/dt)+1):
        q = .5*(nominal+disturbance)*(k*dt)**2
        qhat, vhat, dhat = o.update(q, nominal, dt)
    np.testing.assert_allclose(qhat, q, atol=1e-10)
    np.testing.assert_allclose(vhat, 4*(nominal+disturbance), atol=1e-9)
    np.testing.assert_allclose(dhat, disturbance, atol=1e-8)


def test_recorded_nominal_acceleration_uses_final_applied_torque_and_estimated_velocity():
    c = compensator()
    c.prepare([0.]*3, [0.]*3, 1000000000, 1, True)
    torque = np.array([.3, -.2, .1])
    c.record_applied(np.zeros(3), torque, 1)
    np.testing.assert_allclose(c.nominal_a_previous,
                               np.linalg.solve(c.dyn.mass, torque-c.dyn.bias))
    c.record_applied(np.zeros(3), [99.]*3, 2)  # unrelated/unconsumed state
    np.testing.assert_allclose(c.nominal_a_previous,
                               np.linalg.solve(c.dyn.mass, torque-c.dyn.bias))


def test_nominal_scale_matches_empirical_feedforward_and_avoids_double_counting_base_bias():
    scale = np.array([2.2, 1., 1.])
    c = compensator(nominal_input_scale=scale)
    c.prepare([0.]*3, [0.]*3, 1000000000, 1, True)
    desired_acceleration = np.array([.1, -.2, .3])
    calibrated_ff = scale * c.dyn.inverse_dynamics(np.zeros(3), np.zeros(3), desired_acceleration)
    c.record_applied(np.zeros(3), calibrated_ff, 1)
    np.testing.assert_allclose(c.nominal_a_previous, desired_acceleration, atol=1e-15)


def test_gate_warmup_duplicate_long_gap_and_rewind_reset_without_learning_ignored_commands():
    c = compensator()
    zero = np.zeros(3)
    for k in range(152):
        c.prepare(zero, zero, 1000000000+k*10000000, k, True)
        c.record_applied(zero, c.dyn.bias, k)
        if k < 150:
            np.testing.assert_array_equal(c.compensation, zero)
    assert c.status == 'ACTIVE'
    state = c.observer.d_hat.copy()
    c.prepare([1.]*3, zero, 2510000000, 151, True)
    np.testing.assert_array_equal(c.observer.d_hat, state)
    for stamp in (3000000000, 1000000000):
        c.prepare([.1]*3, [.2]*3, stamp, stamp, True)
        assert c.status == 'WARMUP'
        np.testing.assert_array_equal(c.observer.v_hat, [.2]*3)
        np.testing.assert_array_equal(c.observer.d_hat, zero)
    c.prepare(zero, zero, 1100000000, 300, False)
    assert c.status == 'WAIT_EFFORT_ALL' and c.observer.q_hat is None
    np.testing.assert_array_equal(c.compensation, zero)


def test_three_axis_disturbance_compensation_has_correct_sign_mass_mapping_and_bounded_rate():
    c = compensator(ramp_s=.1)
    disturbance = np.array([1., -.5, .3])
    q = np.zeros(3)
    for k in range(500):
        t = k*.01
        q = .5*disturbance*t*t
        old = c.compensation.copy()
        c.prepare(q, disturbance*t, 1000000000+k*10000000, k, True)
        c.record_applied(q, c.dyn.bias, k)  # nominal acceleration zero
        assert np.all(np.abs(c.compensation-old) <= c.comp_rate*.01+1e-12)
    np.testing.assert_allclose(c.tau_dist_hat, c.dyn.mass @ disturbance, atol=1e-7)
    np.testing.assert_allclose(c.compensation, -c.dyn.mass @ disturbance, atol=1e-7)


def test_final_torque_and_slew_limits_and_logged_decomposition():
    c = compensator()
    c.status = 'ACTIVE'
    c.compensation[:] = [.5, -2., .3]
    previous = np.array([4.9, -39.9, 4.9])
    applied = c.apply([5., -40., 5.], previous, [5., 20., 5.], .01)
    assert np.all(np.abs(applied) <= c.limit)
    assert np.all(np.abs(applied-previous) <= np.array([5., 20., 5.])*.01+1e-12)
    np.testing.assert_allclose(applied, c.last['tau_pre_eso']+c.last['tau_eso']
                              +c.last['tau_eso_actuator_adjustment'])


def description(all_torque=True):
    return ('<robot><ros2_control><hardware><param name="torque_handoff_duration_s">1.2</param>'
            '</hardware>'+''.join(f'<joint name="{j}_joint"><param name="torque_mode_enable">'
            f'{str(all_torque or j=="elbow").lower()}</param></joint>'
            for j in ('base', 'shoulder', 'elbow'))+'</ros2_control></robot>')


def test_hybrid_position_joints_and_unconfirmed_or_stale_controller_modes_cannot_enable_eso():
    names = ['base_joint', 'shoulder_joint', 'elbow_joint']
    assert hardware_all_torque(description(), names) == (True, 1.2)
    assert hardware_all_torque(description(False), names) == (False, 1.2)
    gate = EsoModeGate.__new__(EsoModeGate)
    gate.node = SimpleNamespace(joint_names=names)
    gate.effort_active = gate.all_torque = False
    gate.mode_stamp = gate.description_stamp = 0.
    def mode(effort, position):
        return SimpleNamespace(result=lambda: SimpleNamespace(controller=[
            SimpleNamespace(name='gim_arm_effort_controller', state=effort),
            SimpleNamespace(name='gim_arm_group_controller', state=position)]))
    with patch('gim_control.eso_runtime.time.monotonic', return_value=10.):
        gate._mode_done(mode('inactive', 'active'))
        gate._description_done(SimpleNamespace(result=lambda: SimpleNamespace(values=[
            SimpleNamespace(string_value=description())])))
        assert not gate.allowed
        gate._mode_done(mode('active', 'inactive'))
        assert gate.allowed
    with patch('gim_control.eso_runtime.time.monotonic', return_value=10.6):
        assert not gate.allowed


@pytest.mark.parametrize('value', [0., -1., np.nan, np.inf])
def test_invalid_bandwidth_rejected(value):
    with pytest.raises(ValueError):
        DiscreteLeso([15., value, 15.])
