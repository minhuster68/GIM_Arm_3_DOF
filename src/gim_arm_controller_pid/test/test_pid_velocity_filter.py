"""PID velocity filtering preserves the position loop, feedforward and raw gate."""
import csv
from pathlib import Path
from types import SimpleNamespace
import time
from unittest.mock import patch

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
import yaml

from gim_arm_controller_pid.factory import PositionPidFactory
from gim_control.arm_dynamics import ArmDynamics
from gim_control.effort_controller_node import EffortControllerNode
from gim_control.plot_ee_error import load_log
from gim_control.reference_trajectory import Hold


ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / 'src/gim_arm_controller_pid/config/pid_hardware_soft.yaml'


def parameters():
    return yaml.safe_load(PROFILE.read_text())['cascade_pid_controller']['ros__parameters']


def build(values):
    node = SimpleNamespace(
        declare_parameter=lambda n, v: values.setdefault(n, v),
        get_parameter=lambda n: SimpleNamespace(value=values[n]))
    factory = PositionPidFactory()
    factory.declare_parameters(node)
    dyn = ArmDynamics(str(ROOT / 'src/gim_arm_description/urdf/gim_arm.urdf'))
    return factory.build(node, dyn, dyn.tau_max, 100.)


def test_only_derivative_changes_when_measured_velocity_is_filtered():
    filtered = build(parameters())
    raw = build(dict(parameters(), velocity_filter_type='none'))
    for name in ('kp', 'ki', 'kd', 'tau_limit', 'integral_limit'):
        np.testing.assert_array_equal(getattr(filtered, name), getattr(raw, name))
    assert raw.velocity_filter is None
    f = filtered.velocity_filter
    f.update([0., 0., 0.], 1.)
    measured = np.array([.06, -.04, .08])
    estimate = f.update(measured, 1.01)
    zero = np.zeros(3)
    qr, vr, ar = np.full(3, .01), np.full(3, .005), np.full(3, .02)
    for controller, velocity in ((filtered, estimate), (raw, measured)):
        controller.integral[:] = [.01, .02, .03]
        controller.compute(zero, velocity, qr, vr, ar, .01)
    for name in ('tau_ff', 'tau_p', 'tau_i'):
        np.testing.assert_array_equal(filtered.last[name], raw.last[name])
    np.testing.assert_allclose(filtered.last['tau_d'], filtered.kd * (vr - estimate))
    np.testing.assert_allclose(filtered.last['tau_d'] - raw.last['tau_d'],
                               filtered.kd * (measured - estimate))
    saved = f.state.copy()
    filtered.reset()
    np.testing.assert_array_equal(f.state, saved)


@pytest.mark.parametrize('overrides', [
    {'velocity_filter_type': 'typo'},
    {'velocity_kalman_measurement_std': [.03, 0., .04]},
    {'velocity_kalman_jerk_std': [180., -1., 240.]},
    {'velocity_kalman_reset_gap_s': .001},
])
def test_invalid_filter_parameters_are_rejected(overrides):
    with pytest.raises(ValueError):
        build(dict(parameters(), **overrides))


def test_node_filters_all_three_derivatives_logs_both_signals_and_keeps_raw_start_gate(tmp_path):
    values = dict(parameters(), urdf_file=str(ROOT / 'src/gim_arm_description/urdf/gim_arm.urdf'),
                  cache_file='', log_file=str(tmp_path / 'pid.csv'),
                  start_velocity_limit_rad_s=.05)
    rclpy.init(args=[])
    node = None
    try:
        node = EffortControllerNode(PositionPidFactory(), parameter_overrides=[
            Parameter(k, value=v) for k, v in values.items()])
        node.home_q = np.zeros(3)
        node.reference = Hold(node.home_q)
        node.phase = 'APPROACH'
        node.controller_schedule_ready = True

        def state(nsec, velocity):
            msg = JointState()
            msg.name, msg.position, msg.velocity = node.joint_names, [0.]*3, velocity
            msg.header.stamp.sec, msg.header.stamp.nanosec = 1, nsec
            node._on_state(msg)

        state(0, [0.]*3)
        state(10000000, [.06]*3)
        assert np.all(node.qd < .05) and np.all(node.qd_raw > .05)
        assert not node._start_velocity_is_safe()
        timestamp = node.velocity_filter.timestamp

        def capture(torque):
            node.last_command = np.asarray(torque).copy()
            node.last_publish_wall_ns = time.time_ns()

        with patch.object(node, '_publish', side_effect=capture):
            node._tick()
            np.testing.assert_array_equal(node.controller.last['error_rate'], -node.qd)
            np.testing.assert_allclose(node.controller.last['tau_d'], -node.controller.kd * node.qd)
            assert np.all(np.abs(node.last_command) <= node.tau_limit)
            node._tick()  # No fresh state: freeze the observer and controller.
        assert node.velocity_filter.timestamp == timestamp
        node.dump()
        log = load_log(values['log_file'])
        np.testing.assert_array_equal(log['qd_raw'][0], [.06]*3)
        np.testing.assert_array_equal(log['qd_filtered'][0], log['qd'][0])
        np.testing.assert_allclose(log['tau_fb'][0],
            log['tau_p'][0] + log['tau_i'][0] + log['tau_d'][0])
        with open(values['log_file']) as stream:
            rows = list(csv.DictReader(stream))
        assert all(None not in row and all(v is not None for v in row.values()) for row in rows)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
