"""Exercise the actual MPC measurement/control/logging path without CAN."""
import csv
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
import yaml

from gim_arm_controller_mpc.factory import MpcFactory
from gim_control.arm_dynamics import ArmDynamics
from gim_control.effort_controller_node import EffortControllerNode
from gim_control.plot_ee_error import load_log
from gim_control.reference_trajectory import Hold


ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / 'src/gim_arm_controller_mpc/config/mpc_circle_independent_feedback.yaml'


def parameters():
    return yaml.safe_load(PROFILE.read_text())['mpc_controller']['ros__parameters']


def build(values):
    node = SimpleNamespace(
        declare_parameter=lambda n, v: values.setdefault(n, v),
        get_parameter=lambda n: SimpleNamespace(value=values[n]))
    factory = MpcFactory()
    factory.declare_parameters(node)
    dyn = ArmDynamics(str(ROOT / 'src/gim_arm_description/urdf/gim_arm.urdf'))
    return factory.build(node, dyn, dyn.tau_max, 100.)


def test_filter_preserves_tuned_costs_models_feedforward_and_constraints():
    filtered = build(parameters())
    raw = build(dict(parameters(), velocity_filter_type='none'))
    for name in ('Q', 'R', 'Rd', 'slew', 'tau_limit'):
        np.testing.assert_array_equal(getattr(filtered, name), getattr(raw, name))
    assert filtered.feedback_model_mode == raw.feedback_model_mode == 'independent'
    assert (filtered.np, filtered.nc) == (raw.np, raw.nc) == (40, 10)
    assert filtered.velocity_filter is not None and raw.velocity_filter is None
    q, v, acc = np.array([.1, .2, .3]), np.array([.02, .03, .04]), np.zeros(3)
    np.testing.assert_array_equal(filtered._feedforward(q, v, acc), raw._feedforward(q, v, acc))
    for a, b in zip(filtered._linearize(q, v, acc), raw._linearize(q, v, acc)):
        np.testing.assert_array_equal(a, b)
    f = filtered.velocity_filter
    f.update([.1, .2, .3], 1.)
    state = f.state.copy()
    filtered.reset()
    np.testing.assert_array_equal(f.state, state)


@pytest.mark.parametrize('overrides', [
    {'velocity_filter_type': 'typo'},
    {'velocity_kalman_measurement_std': [.03, 0., .04]},
    {'velocity_kalman_jerk_std': [180., -1., 240.]},
    {'velocity_kalman_reset_gap_s': .001},
])
def test_invalid_filter_settings_fail_before_control(overrides):
    with pytest.raises(ValueError):
        build(dict(parameters(), **overrides))


def test_filtered_velocity_enters_qp_raw_velocity_stays_in_gate_and_csv(tmp_path):
    values = dict(parameters(), urdf_file=str(ROOT / 'src/gim_arm_description/urdf/gim_arm.urdf'),
                  cache_file='', log_file=str(tmp_path / 'mpc.csv'),
                  prediction_horizon=12, control_horizon=4,
                  feedback_bounds_mode='instantaneous', precompute_prediction=False)
    rclpy.init(args=[])
    node = None
    try:
        node = EffortControllerNode(MpcFactory(), parameter_overrides=[
            Parameter(k, value=v) for k, v in values.items()])
        node.home_q = np.zeros(3)
        node.reference = Hold(node.home_q)
        node.phase = 'APPROACH'
        node.controller_schedule_ready = True

        def state(nanosec, velocity):
            msg = JointState()
            msg.name = node.joint_names
            msg.position = [0.] * 3
            msg.velocity = velocity
            msg.header.stamp.sec = 1
            msg.header.stamp.nanosec = nanosec
            node._on_state(msg)

        state(0, [0.] * 3)
        state(10000000, [.06] * 3)
        assert np.all(node.qd < .05) and np.all(node.qd_raw > .05)
        assert not node._start_velocity_is_safe()
        timestamp = node.velocity_filter.timestamp
        def capture(torque):
            node.last_command = np.asarray(torque).copy()
            node.last_publish_wall_ns = time.time_ns()

        with patch.object(node, '_publish', side_effect=capture) as publish:
            node._tick()
            assert node.phase == 'APPROACH'
            np.testing.assert_array_equal(node.controller.last['error_rate'], node.qd)
            assert node.controller.last['solver_accepted']
            assert np.all(np.abs(publish.call_args.args[0]) <= node.tau_limit)
            node._tick()  # No fresh JointState: never update/filter the same measurement again.
        assert node.velocity_filter.timestamp == timestamp
        node.dump()
        log = load_log(values['log_file'])
        np.testing.assert_array_equal(log['qd_raw'][0], [.06] * 3)
        np.testing.assert_array_equal(log['qd_filtered'][0], log['qd'][0])
        assert np.any(log['qd_raw'] != log['qd_filtered'])
        rows = list(csv.DictReader(open(values['log_file'])))
        assert all(None not in row and all(v is not None for v in row.values()) for row in rows)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
