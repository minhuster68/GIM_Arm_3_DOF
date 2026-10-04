"""Verify isolated and paired tuning options without launching controllers."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
import numpy as np
import pytest

from gim_control.reference_trajectory import Quintic


@pytest.fixture
def tuning_launch():
    path = Path(__file__).resolve().parents[1] / 'launch/pid_joint_tuning.launch.py'
    spec = importlib.util.spec_from_file_location('pid_tuning_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(tmp_path, **options):
    params = Path(__file__).resolve().parents[2] / (
        'gim_arm_controller_pid/config/pid_hardware_tuning.yaml')
    value = LaunchContext()
    value.launch_configurations.update({
        'joint': 'elbow', 'target_deg': '30.0', 'move_time': '6.0',
        'shoulder_deg': '15.0', 'elbow_deg': '30.0',
        'hold_time': '3.0', 'return_time': '6.0', 'params_file': str(params),
        'log_file': str(tmp_path / 'logs' / 'tuning.csv'), **options,
    })
    return value


@pytest.mark.parametrize('joint,index', [('base', 0), ('shoulder', 1), ('elbow', 2)])
def test_only_selected_joint_moves_and_cascade_hold_is_enabled(
        tuning_launch, tmp_path, monkeypatch, joint, index):
    monkeypatch.setattr(tuning_launch, 'Node', lambda **kwargs: kwargs)
    actions = tuning_launch._launch_setup(context(tmp_path, joint=joint))
    options = actions[0]['parameters'][1]
    target = [options[f'diagnostic_q{i}_deg'] for i in (1, 2, 3)]
    expected = [0.0, 0.0, 0.0]
    expected[index] = 30.0
    assert target == expected
    assert options['diagnostic_hold'] and options['cascade_hold']
    assert not options['autostart']
    assert not options['use_sim_time']
    assert options['approach_time'] == 6.0
    assert options['diagnostic_hold_time'] == 3.0
    assert options['return_time'] == 6.0


def test_paired_targets_move_together_and_leave_base_reference_fixed(
        tuning_launch, tmp_path, monkeypatch):
    monkeypatch.setattr(tuning_launch, 'Node', lambda **kwargs: kwargs)
    actions = tuning_launch._launch_setup(context(
        tmp_path, joint='shoulder_elbow', shoulder_deg='12', elbow_deg='24'))
    options = actions[0]['parameters'][1]
    target = np.radians([options[f'diagnostic_q{i}_deg'] for i in (1, 2, 3)])
    np.testing.assert_allclose(target, np.radians([0, 12, 24]))
    outbound = Quintic(np.zeros(3), target, options['approach_time'])
    returning = Quintic(target, np.zeros(3), options['return_time'])
    for reference in (outbound, returning):
        for time in np.linspace(0, 6, 13):
            q, qd, qdd = reference.at(time)
            for vector in (q, qd, qdd):
                assert vector[0] == 0
                assert vector[2] == pytest.approx(2 * vector[1])
    assert options['cascade_hold'] and options['diagnostic_hold']
    assert options['start_velocity_limit_rad_s'] == 0.05
    assert not options['autostart']


@pytest.mark.parametrize('options', [
    {'target_deg': 'nan'}, {'move_time': '0'}, {'return_time': '-1'},
    {'hold_time': 'inf'}, {'joint': 'wrong'}, {'params_file': '/missing/tuning.yaml'},
    {'joint': 'shoulder_elbow', 'shoulder_deg': 'nan'},
    {'joint': 'shoulder_elbow', 'elbow_deg': 'inf'},
])
def test_invalid_options_fail_before_creating_a_controller(tuning_launch, tmp_path, options):
    with pytest.raises(ValueError):
        tuning_launch._launch_setup(context(tmp_path, **options))
