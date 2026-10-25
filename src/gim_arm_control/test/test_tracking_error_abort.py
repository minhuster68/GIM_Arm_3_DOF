"""Tracking-error bypass preserves the other guards, without CAN hardware."""

from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from gim_control.effort_controller_node import EffortControllerNode
from gim_control.reference_trajectory import Hold


class DummyController:
    def __init__(self):
        self.reset()

    def reset(self):
        self.integral = np.zeros(3)
        self.last = {'solver_accepted': True}

    def describe(self, q_nominal=None):
        return 'Test controller: exercises shared guards without motor commands'

    def compute(self, *args):
        return np.full(3, 1000.0)


class DummyFactory:
    def __init__(self, algorithm):
        self.algorithm_name = algorithm

    def declare_parameters(self, node):
        pass

    def build(self, *args):
        return DummyController()


@pytest.fixture(params=['mpc'])
def arm(request):
    root = Path(__file__).resolve().parents[3]
    rclpy.init(args=[])
    node = None
    try:
        node = EffortControllerNode(DummyFactory(request.param), parameter_overrides=[
            Parameter('urdf_file', value=str(
                root / 'src/gim_arm_description/urdf/gim_arm.urdf')),
            Parameter('diagnostic_hold', value=True),
            Parameter('algorithm_hold', value=True),
            Parameter('cascade_hold', value=request.param == 'cascade_pid'),
            Parameter('max_track_error_rad', value=0.1),
            Parameter('max_transition_error_rad', value=0.1),
        ])
        node.home_q = np.zeros(3)
        node.reference = Hold(node.home_q)
        node.phase = 'APPROACH'
        node.phase_elapsed = 0.25
        node.controller.last = {'solver_accepted': True}
        with patch.object(node, '_publish') as publish, patch.object(
                node.controller, 'compute', return_value=np.full(3, 1000.0)) as compute:
            yield node, publish, compute
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def state(node, q=None):
    msg = JointState()
    msg.name = node.joint_names
    msg.position = list(q if q is not None else [0.0, 0.2, 0.0])
    msg.velocity = [0.0, 0.0, 0.0]
    msg.header.stamp = node.get_clock().now().to_msg()
    node._on_state(msg)


@pytest.mark.parametrize('phase', ['APPROACH', 'TRACK', 'RETURN', 'HOLD'])
@pytest.mark.parametrize('enabled', [True, False])
def test_large_error_can_be_bypassed_without_bypassing_torque_limits(arm, phase, enabled):
    node, publish, compute = arm
    node.phase = phase
    node.set_parameters([Parameter('abort_on_tracking_error', value=enabled)])
    state(node)
    node._tick()
    if enabled:
        assert node.phase == 'ABORT'
        compute.assert_not_called()
    else:
        assert node.phase == phase
        compute.assert_called_once()
        torque = publish.call_args.args[0]
        np.testing.assert_allclose(torque, node.tau_limit)


def test_abort_is_disabled_by_default_and_can_be_restored_while_running(arm):
    node, _, compute = arm
    assert node.get_parameter('abort_on_tracking_error').value is False
    node.set_parameters([Parameter('abort_on_tracking_error', value=False)])
    state(node)
    node._tick()
    assert node.phase == 'APPROACH'
    compute.reset_mock()
    node.set_parameters([Parameter('abort_on_tracking_error', value=True)])
    state(node)
    node._tick()
    assert node.phase == 'ABORT'
    compute.assert_not_called()


@pytest.mark.parametrize('guard', ['joint_limit', 'stale_state'])
def test_other_guards_remain_active_when_tracking_abort_is_disabled(arm, guard):
    node, _, compute = arm
    node.set_parameters([Parameter('abort_on_tracking_error', value=False)])
    q = np.zeros(3)
    if guard == 'joint_limit':
        q[1] = node.dynamics.q_max[1] - node.joint_margin / 2.0
    state(node, q)
    if guard == 'stale_state':
        node.state_stamp = node.get_clock().now() - Duration(seconds=0.5)
    node._tick()
    assert node.phase == 'ABORT'
    compute.assert_not_called()
