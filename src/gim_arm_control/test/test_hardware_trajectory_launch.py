"""Validate hardware launch routing and startup defaults without CAN."""

from pathlib import Path

from launch import LaunchContext
import pytest
import yaml

from gim_control import hardware_trajectory_launch as hardware


@pytest.mark.parametrize('algorithm', ['mpc'])
@pytest.mark.parametrize('shape', ['circle', 'r', 'a'])
def test_profiles_and_shapes_route_without_changing_gains(tmp_path, monkeypatch, algorithm, shape):
    source = Path(__file__).resolve().parents[2]
    monkeypatch.setattr(hardware, 'get_package_share_directory', lambda package: str(source / package))
    monkeypatch.setattr(hardware, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update({
        'algorithm': algorithm, 'trajectory_shape': shape, 'params_file': '',
        'approach_time': '16', 'return_time': '16', 'log_file': str(tmp_path / 'test.csv'),
    })
    node, = hardware._launch_setup(context)
    path, overrides = node['parameters']
    params = yaml.safe_load(Path(path).read_text())[f'{algorithm}_controller']['ros__parameters']
    assert node['package'] == f'gim_arm_controller_{algorithm}'
    assert params['control_hz'] == 100.0
    assert params['tau_scale'] == 0.35
    assert params['algorithm_hold']
    assert overrides['trajectory_shape'] == shape
    assert overrides['algorithm_hold'] and not overrides['cascade_hold']
    assert not overrides['autostart'] and not overrides['use_sim_time']
    assert overrides['start_velocity_limit_rad_s'] == 0.05
    assert not {'control_hz', 'tau_scale', 'lambda_gain', 'max_e', 'max_de',
                'q_position', 'q_velocity', 'output_scale_velocity'}.intersection(overrides)
    if algorithm == 'lqr':
        assert params['position_tracking_only'] is False and params['use_discrete_lqr']
        assert all(v > 0 for v in params['max_de'])
    if algorithm == 'mpc':
        assert all(v > 0 for v in params['q_velocity'])


@pytest.mark.parametrize('options', [
    {'algorithm': 'pid'}, {'trajectory_shape': 'unknown'},
    {'approach_time': '0'}, {'return_time': 'nan'},
    {'params_file': '/missing/profile.yaml'},
])
def test_invalid_launch_options_are_rejected(tmp_path, options):
    context = LaunchContext()
    valid_params = Path(__file__).resolve().parents[2] / (
        'gim_arm_controller_mpc/config/mpc_hardware_soft.yaml')
    context.launch_configurations.update({
        'algorithm': 'mpc', 'trajectory_shape': 'circle',
        'params_file': str(valid_params), 'approach_time': '16',
        'return_time': '16', 'log_file': str(tmp_path / 'test.csv'), **options,
    })
    with pytest.raises(ValueError):
        hardware._launch_setup(context)
