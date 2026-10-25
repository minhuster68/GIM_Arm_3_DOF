"""Validate hardware launch routing and startup defaults without CAN."""

from pathlib import Path

from launch import LaunchContext
import pytest
import yaml

from gim_control import hardware_trajectory_launch as hardware


@pytest.mark.parametrize('algorithm', ['smc'])
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
    assert params['tau_scale'] == (1.0 if algorithm == 'smc' else 0.35)
    if algorithm == 'smc':
        assert Path(path).name == 'smc_circle_from_terminal_tuning.yaml'
        assert params['feedback_offdiag_scale'] == [0., 0., 0.]
    assert params['algorithm_hold']
    assert overrides['trajectory_shape'] == shape
    assert overrides['algorithm_hold'] and not overrides['cascade_hold']
    assert not overrides['autostart'] and not overrides['use_sim_time']
    assert overrides['start_velocity_limit_rad_s'] == 0.05
    assert not {'control_hz', 'tau_scale', 'lambda_gain', 'max_e', 'q_position'}.intersection(overrides)
    if algorithm == 'lqr':
        assert params['position_tracking_only'] and params['use_discrete_lqr']
    if algorithm == 'mpc':
        assert params['q_velocity'] == [0.0, 0.0, 0.0]


@pytest.mark.parametrize('options', [
    {'algorithm': 'pid'}, {'trajectory_shape': 'unknown'},
    {'approach_time': '0'}, {'return_time': 'nan'},
    {'params_file': '/missing/profile.yaml'},
])
def test_invalid_launch_options_are_rejected(tmp_path, options):
    context = LaunchContext()
    valid_params = Path(__file__).resolve().parents[2] / (
        'gim_arm_controller_smc/config/smc_circle_from_terminal_tuning.yaml')
    context.launch_configurations.update({
        'algorithm': 'smc', 'trajectory_shape': 'circle',
        'params_file': str(valid_params), 'approach_time': '16',
        'return_time': '16', 'log_file': str(tmp_path / 'test.csv'), **options,
    })
    with pytest.raises(ValueError):
        hardware._launch_setup(context)


def test_smc_circle_rejects_single_joint_profile(tmp_path):
    params_file = tmp_path / 'single_joint.yaml'
    params_file.write_text(
        'smc_controller:\n  ros__parameters:\n    controlled_joint: elbow\n')
    context = LaunchContext()
    context.launch_configurations.update({
        'algorithm': 'smc', 'trajectory_shape': 'circle',
        'params_file': str(params_file), 'approach_time': '16',
        'return_time': '16', 'log_file': str(tmp_path / 'run.csv'),
    })
    with pytest.raises(ValueError, match='controlled_joint: all'):
        hardware._launch_setup(context)
