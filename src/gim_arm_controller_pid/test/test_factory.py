"""Check that ordinary PID startup selects a single position loop."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from gim_arm_controller_pid.controller import (
    PositionPidController,
)
from gim_arm_controller_pid.factory import CascadePidFactory


class ParameterNode:
    def __init__(self, overrides=None):
        self.parameters = {}
        self.overrides = overrides or {}

    def declare_parameter(self, name, default):
        self.parameters[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])


def build(overrides=None):
    node = ParameterNode(overrides)
    factory = CascadePidFactory()
    factory.declare_parameters(node)
    dynamics = SimpleNamespace(nq=3, tau_max=np.ones(3))
    return factory.build(node, dynamics, dynamics.tau_max, 2000)


def test_default_factory_builds_position_pid():
    assert isinstance(build(), PositionPidController)


@pytest.mark.parametrize('filename, expected', [
    ('pid_gazebo_smooth.yaml', PositionPidController),
    ('pid.yaml', PositionPidController),
    ('pid_hardware_soft.yaml', PositionPidController),
    ('pid_hardware_tuning.yaml', PositionPidController),
])
def test_profiles_choose_their_control_law_explicitly(filename, expected):
    config = Path(__file__).resolve().parents[1] / 'config' / filename
    parameters = yaml.safe_load(config.read_text())[
        'cascade_pid_controller']['ros__parameters']
    assert isinstance(build(parameters), expected)


def test_position_pid_accepts_zero_integral_and_derivative_gains():
    pid = build({'position_ki': [0.0] * 3, 'position_kd': [0.0] * 3})
    np.testing.assert_array_equal(pid.ki, np.zeros(3))
    np.testing.assert_array_equal(pid.kd, np.zeros(3))


def test_launch_defaults_to_position_profile_at_gazebo_rate():
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    path = (Path(__file__).resolve().parents[2] / 'gim_arm_control' /
            'launch/run_effort_algorithm.launch.py')
    spec = importlib.util.spec_from_file_location('pid_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations['algorithm'] = 'pid'
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['pid_profile'] == 'gazebo_smooth'
    assert float(context.launch_configurations['control_hz']) == 2000.0
    assert float(context.launch_configurations['tau_scale']) == 0.50


def test_factory_rejects_invalid_position_gains():
    for parameter in ('position_kp', 'position_ki', 'position_kd'):
        with pytest.raises(ValueError):
            build({parameter: [-1.0, 0.0, 0.0]})
    with pytest.raises(ValueError):
        build({'position_integral_limit': [0.0, 0.1, 0.1]})


def test_ros_entry_point_builds_position_pid_without_cascade_parameters():
    from unittest.mock import patch
    from gim_arm_controller_pid import node
    from gim_arm_controller_pid.factory import PositionPidFactory

    parameters = ParameterNode()
    factory = PositionPidFactory()
    factory.declare_parameters(parameters)
    assert set(parameters.parameters) == {
        'position_kp', 'position_ki', 'position_kd', 'position_integral_limit'}
    with patch.object(node, 'run_controller') as run:
        node.main()
    assert isinstance(run.call_args.args[0], PositionPidFactory)
