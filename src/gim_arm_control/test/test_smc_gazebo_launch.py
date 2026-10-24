"""Check profile routing, 1 ms timing and Gazebo's simulated clock."""

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.utilities import evaluate_parameters, normalize_parameters


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('profile,filename,hz,scale', [
    ('super_twisting', 'smc_super_twisting_gazebo.yaml', 1000., 1.),
    ('gazebo_safe', 'smc_gazebo_safe.yaml', 100., .5),
    ('matlab_reference', 'smc.yaml', 100., .35),
])
def test_smc_profile_routes_frequency_and_torque(profile, filename, hz, scale, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'effort_launch', ROOT / 'launch/run_effort_algorithm.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    captured = []

    def capture(**kwargs):
        captured.append(kwargs)
        return OpaqueFunction(function=lambda context: [])

    monkeypatch.setattr(module, 'Node', capture)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(
        algorithm='smc', smc_profile=profile, use_sim_time='true')
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    active = [node for node in captured if node['condition'].evaluate(context)]
    assert len(active) == 1
    node = active[0]
    assert node['executable'] == 'smc_node'
    config = node['parameters'][0].perform(context)
    assert Path(config).name == filename
    overrides, = evaluate_parameters(context, normalize_parameters(node['parameters'][1:]))
    assert overrides['control_hz'] == hz
    assert overrides['tau_scale'] == scale
    assert overrides['use_sim_time'] is True
    if profile == 'super_twisting':
        params = yaml.safe_load(Path(config).read_text())['smc_controller']['ros__parameters']
        assert params['smc_variant'] == 'super_twisting'
        assert params['control_hz'] == 1 / .001
        assert params['lambda_gain'] == [15., 85., 155.]
        assert params['k1'] == [15., 30., 25.]
        assert params['k2'] == [30., 100., 80.]


def test_clock_can_drive_one_millisecond_controller():
    params = yaml.safe_load((ROOT / 'config/gazebo_clock.yaml').read_text())
    assert params['gazebo']['ros__parameters']['publish_rate'] >= 1000.
