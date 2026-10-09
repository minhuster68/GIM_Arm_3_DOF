"""Regression checks for reference inverse dynamics plus position-error PID."""

import numpy as np

from gim_arm_controller_pid.controller import PositionPidController


class ReferenceDynamics:
    nq = 3
    tau_max = np.array([10.0, 10.0, 10.0])

    def gravity(self, q):
        return np.array([0.0, 2.0, 1.0]) + 0.1 * np.asarray(q)

    mass = np.array([2.0, 3.0, 4.0])

    def inverse_dynamics(self, q, qd, qdd):
        return (self.mass * np.asarray(qdd)
                + 0.2 * np.asarray(qd) ** 2 + self.gravity(q))


def controller(**overrides):
    parameters = dict(
        kp=[4.0, 5.0, 6.0], ki=[1.0, 2.0, 3.0],
        kd=[0.5, 1.0, 1.5], integral_limit=[0.1, 0.1, 0.1])
    parameters.update(overrides)
    return PositionPidController(ReferenceDynamics(), **parameters)


def test_pid_torque_is_feedforward_plus_position_p_i_and_error_derivative():
    pid = controller()
    q = np.array([0.1, 0.2, 0.3])
    qd = np.array([0.2, -0.1, 0.3])
    qref = q + 0.05
    qdref = np.array([0.3, -0.2, 0.4])
    qddref = np.array([-0.1, 0.2, 0.05])
    pid.compute(q, qd, qref, qdref, qddref, 0.1)
    integral = pid.integral.copy()
    torque = pid.compute(q, qd, qref, qdref, qddref, 0.1)
    np.testing.assert_allclose(
        torque, pid.dyn.inverse_dynamics(qref, qdref, qddref)
        + pid.kp * (qref - q)
        + pid.ki * integral + pid.kd * (qdref - qd))


def test_integral_accumulates_position_error_and_is_bounded():
    pid = controller()
    for _ in range(100):
        pid.compute(np.zeros(3), np.ones(3), np.array([0.2, -0.3, 0.4]),
                    np.ones(3), np.zeros(3), 0.01)
    np.testing.assert_allclose(pid.integral, [0.1, -0.1, 0.1])
    pid.reset()
    np.testing.assert_array_equal(pid.integral, np.zeros(3))


def test_saturation_freezes_integral_but_allows_unwinding():
    pid = controller(tau_limit=[0.1, 0.1, 0.1])
    torque = pid.compute(np.zeros(3), np.zeros(3), np.ones(3),
                         np.zeros(3), np.zeros(3), 0.01)
    np.testing.assert_allclose(torque, [0.1, 0.1, 0.1])
    np.testing.assert_array_equal(pid.integral, np.zeros(3))
    # Negative position error must unwind I even when gravity alone
    # still saturates the positive torque at the shoulder and elbow.
    pid.compute(np.zeros(3), np.zeros(3), np.full(3, -0.01),
                np.zeros(3), np.zeros(3), 0.01)
    pid.integral[:] = 0.1
    pid.compute(np.zeros(3), np.zeros(3), np.full(3, -0.01),
                np.zeros(3), np.zeros(3), 0.01)
    assert np.all(pid.integral < 0.1)


def test_derivative_is_zero_for_constant_error_during_motion():
    pid = controller(ki=[0.0, 0.0, 0.0])
    q = np.array([0.1, 0.2, 0.3])
    qd = np.array([0.2, -0.1, 0.3])
    zero = np.zeros(3)
    torque = pid.compute(q, qd, q, qd, zero, 0.01)
    np.testing.assert_allclose(torque, pid.dyn.inverse_dynamics(q, qd, zero))
    q = q + qd * 0.01
    torque = pid.compute(q, qd, q, qd, zero, 0.01)
    np.testing.assert_allclose(torque, pid.dyn.inverse_dynamics(q, qd, zero))
    np.testing.assert_array_equal(pid.last["tau_derivative"], np.zeros(3))
    np.testing.assert_array_equal(pid.integral, np.zeros(3))


def test_derivative_responds_to_reference_motion_and_opposes_actual_motion():
    pid = controller(ki=[0.0, 0.0, 0.0])
    zero = np.zeros(3)
    velocity = np.array([0.1, -0.2, 0.3])
    pid.compute(zero, zero, zero, velocity, zero, 0.01)
    np.testing.assert_allclose(pid.last["tau_derivative"], pid.kd * velocity)
    pid.compute(zero, velocity, zero, zero, zero, 0.01)
    np.testing.assert_allclose(pid.last["tau_derivative"], -pid.kd * velocity)
    pid.reset()
    np.testing.assert_array_equal(pid.integral, zero)


def test_reference_acceleration_changes_torque_by_mass_times_acceleration():
    pid = controller(ki=[0.0, 0.0, 0.0])
    q = np.array([0.1, 0.2, 0.3])
    zero = np.zeros(3)
    first = pid.compute(q, zero, q, zero, zero, 0.01)
    acceleration = np.array([0.1, -0.2, 0.3])
    second = pid.compute(q, zero, q, zero, acceleration, 0.01)
    np.testing.assert_allclose(second - first, pid.dyn.mass * acceleration)


def test_feedforward_uses_all_reference_signals_and_not_measured_state():
    pid = controller(ki=[0.0, 0.0, 0.0])
    q = np.array([0.1, 0.2, 0.3])
    qd = np.array([0.2, -0.1, 0.3])
    qref = np.array([0.15, 0.25, 0.35])
    qdref = np.array([0.3, -0.2, 0.4])
    qddref = np.array([0.1, -0.2, 0.3])
    expected = pid.dyn.inverse_dynamics(qref, qdref, qddref)
    pid.compute(q, qd, qref, qdref, qddref, 0.01)
    np.testing.assert_allclose(pid.last["tau_ff"], expected)
    pid.compute(q + 0.02, qd * 0.5, qref, qdref, qddref, 0.01)
    np.testing.assert_allclose(pid.last["tau_ff"], expected)


def test_alpha_scales_only_base_feedforward_and_preserves_feedback_and_model():
    original = controller()
    scaled = controller(base_ff_alpha=1.7)
    q = np.array([0.1, 0.2, 0.3])
    qd = np.array([0.2, -0.1, 0.3])
    qref = q + 0.05
    qdref = qd + 0.01
    qddref = np.array([0.1, -0.2, 0.3])
    for _ in range(2):
        original.compute(q, qd, qref, qdref, qddref, 0.01)
        scaled.compute(q, qd, qref, qdref, qddref, 0.01)
    expected = original.last['tau_ff'] * np.array([1.7, 1.0, 1.0])
    np.testing.assert_allclose(scaled.last['tau_ff'], expected)
    np.testing.assert_allclose(scaled.last['tau_ff_urdf'], original.last['tau_ff'])
    for term in ('tau_fb', 'tau_p', 'tau_i', 'tau_d', 'integral'):
        np.testing.assert_allclose(scaled.last[term], original.last[term])
    np.testing.assert_allclose(scaled.last['tau_raw'], expected + original.last['tau_fb'])


def test_alpha_respects_torque_limit_and_saturation_antiwindup():
    pid = controller(base_ff_alpha=100.0, tau_limit=[0.5, 10.0, 10.0])
    zero = np.zeros(3)
    pid.compute(zero, zero, np.array([0.1, 0.0, 0.0]), zero, zero, 0.01)
    assert pid.last['tau_ff'][0] > 0.5
    assert pid.last['tau'][0] == 0.5
    assert pid.last['saturated'][0]
    assert pid.integral[0] == 0.0


def test_alpha_identifies_base_by_joint_name_when_order_changes():
    dynamics = ReferenceDynamics()
    dynamics.joint_names = ['shoulder_joint', 'base_joint', 'elbow_joint']
    pid = PositionPidController(
        dynamics, kp=[0.0] * 3, ki=[0.0] * 3, kd=[0.0] * 3,
        integral_limit=[0.1] * 3, base_ff_alpha=2.0)
    zero = np.zeros(3)
    torque = pid.compute(zero, zero, zero, zero, zero, 0.01)
    np.testing.assert_allclose(torque, [0.0, 4.0, 1.0])
