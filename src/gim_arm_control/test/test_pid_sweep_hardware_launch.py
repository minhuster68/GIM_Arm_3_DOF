"""Check the original sweep is selected with the user's hardware gains."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
import numpy as np
import pytest

from gim_control.reference_trajectory import Quintic


@pytest.fixture
def sweep_launch():
    path = Path(__file__).resolve().parents[1] / 'launch/pid_sweep_hardware.launch.py'
    spec = importlib.util.spec_from_file_location('pid_sweep_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(tmp_path, **options):
    params = Path(__file__).resolve().parents[2] / (
        'gim_arm_controller_pid/config/pid_hardware_soft.yaml')
    value = LaunchContext()
    value.launch_configurations.update({
        'params_file': str(params), 'approach_time': '16.0', 'return_time': '16.0',
        'log_file': str(tmp_path / 'logs' / 'sweep.csv'), **options,
    })
    return value


def test_original_sweep_uses_yaml_without_overriding_gains(sweep_launch, tmp_path, monkeypatch):
    monkeypatch.setattr(sweep_launch, 'Node', lambda **kwargs: kwargs)
    actions = sweep_launch._launch_setup(context(tmp_path))
    params_file, overrides = actions[0]['parameters']
    assert params_file.endswith('config/pid_hardware_soft.yaml')
    assert not overrides['diagnostic_hold'] and not overrides['diagnostic_segment']
    assert not overrides['autostart'] and not overrides['use_sim_time']
    assert overrides['cascade_hold']
    assert overrides['loops'] == 1.0
    assert overrides['approach_time'] == 16.0
    assert overrides['return_time'] == 16.0
    assert overrides['start_velocity_limit_rad_s'] == 0.05
    assert overrides['cache_file'] == ''
    assert not {'kpp', 'kvp', 'kvi', 'tau_scale', 'control_hz'}.intersection(overrides)
    assert (tmp_path / 'logs').is_dir()


def test_slower_transitions_keep_positions_and_halve_velocity(sweep_launch, tmp_path, monkeypatch):
    monkeypatch.setattr(sweep_launch, 'Node', lambda **kwargs: kwargs)
    overrides = sweep_launch._launch_setup(context(tmp_path))[0]['parameters'][1]
    home = np.zeros(3)
    sweep_start = np.radians([42.97, 48.30, 43.38])
    for name, start, end in (
            ('approach_time', home, sweep_start), ('return_time', sweep_start, home)):
        original = Quintic(start, end, 8.0)
        slower = Quintic(start, end, overrides[name])
        for fraction in np.linspace(0, 1, 33):
            q_old, qd_old, qdd_old = original.at(fraction * 8.0)
            q_new, qd_new, qdd_new = slower.at(fraction * overrides[name])
            np.testing.assert_allclose(q_new, q_old)
            np.testing.assert_allclose(qd_new, 0.5 * qd_old)
            np.testing.assert_allclose(qdd_new, 0.25 * qdd_old)


@pytest.mark.parametrize('shape', ['circle', 'r', 'a'])
def test_shape_selection_keeps_existing_pid_gains(sweep_launch, tmp_path, monkeypatch, shape):
    monkeypatch.setattr(sweep_launch, 'Node', lambda **kwargs: kwargs)
    overrides = sweep_launch._launch_setup(
        context(tmp_path, trajectory_shape=shape))[0]['parameters'][1]
    assert overrides['trajectory_shape'] == shape
    assert overrides['cascade_hold']
    assert not {'kpp', 'kvp', 'kvi', 'tau_scale', 'control_hz'}.intersection(overrides)


@pytest.mark.parametrize('options', [
    {'approach_time': '0'}, {'approach_time': 'nan'},
    {'return_time': '-1'}, {'return_time': 'inf'},
    {'params_file': '/missing/gains.yaml'},
    {'trajectory_shape': 'unknown'},
])
def test_invalid_options_fail_before_node_creation(sweep_launch, tmp_path, options):
    with pytest.raises(ValueError):
        sweep_launch._launch_setup(context(tmp_path, **options))
