"""Verify terminal isolation, staged tuning and YAML persistence without a robot."""

import copy
import importlib.util
from pathlib import Path
from unittest.mock import Mock, patch

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'src/gim_arm_hardware/scripts/tune_position_pid.py'
SPEC = importlib.util.spec_from_file_location('position_pid_terminal', SCRIPT)
tuner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tuner)


@pytest.fixture
def terminal(tmp_path):
    source = ROOT / 'src/gim_arm_controller_pid/config/pid_hardware_soft.yaml'
    path = tmp_path / 'gains.yaml'
    path.write_text(source.read_text())
    return tuner.PositionPidTerminal(path, log_dir=tmp_path / 'logs', show_plot=False)


def test_tuning_moves_from_joint_three_to_two_to_one_and_preserves_other_gains(terminal):
    original = copy.deepcopy(terminal.parameters)
    assert terminal.selected == 2
    terminal.execute('set 8 0.1 0.9')
    for key in tuner.GAIN_KEYS.values():
        assert terminal.parameters[key][:2] == original[key][:2]
    terminal.execute('next')
    assert terminal.selected == 1
    terminal.execute('kp 30')
    terminal.execute('next')
    assert terminal.selected == 0
    terminal.execute('kd 1.2')
    terminal.execute('next')
    assert terminal.selected == 0
    terminal.execute('save')
    restored = tuner.load_config(terminal.params_file)['cascade_pid_controller']['ros__parameters']
    assert restored['position_kp'] == [20.0, 30.0, 8.0]
    assert restored['position_ki'] == [2.0, 1.5, 0.1]
    assert restored['position_kd'] == [1.2, 0.7, 0.9]


@pytest.mark.parametrize('command', [
    'set 5 nan 1', 'kp -1', 'ki inf', 'kd 1 extra', 'set 1 2',
    'test nan', 'test 5 0', 'test 5 6 -1', 'test 5 inf',
    'test 5 1e300', 'test 5 1e-15', 'vel 1 2',
])
def test_invalid_input_never_starts_ros_or_changes_gains(terminal, command):
    original = copy.deepcopy(terminal.parameters)
    terminal.trial = Mock()
    with pytest.raises(ValueError):
        terminal.execute(command)
    terminal.trial.run.assert_not_called()
    assert terminal.parameters == original


def test_test_uses_current_position_gains_and_not_driver_frames(terminal):
    terminal.trial = Mock()
    terminal.execute('set 9 0.2 1')
    terminal.execute('test 5 8 2')
    selected, degrees, move, hold, config = terminal.trial.run.call_args.args
    assert (selected, degrees, move, hold) == (2, 5.0, 8.0, 2.0)
    assert config['position_kp'][2] == 9.0
    assert config['position_ki'][2] == 0.2
    assert config['position_kd'][2] == 1.0


def test_dry_run_never_opens_ros_or_writes_yaml(terminal):
    original = terminal.params_file.read_text()
    terminal.dry_run = True
    terminal.trial = Mock()
    terminal.execute('kp 8')
    terminal.execute('test 5')
    terminal.execute('save')
    terminal.trial.run.assert_not_called()
    assert terminal.params_file.read_text() == original


def test_existing_terminal_entry_point_defaults_to_pc_pid(terminal, capsys):
    path = SCRIPT.with_name('tune_motor_gains.py')
    spec = importlib.util.spec_from_file_location('motor_terminal', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch.object(module, 'SocketCanSender') as sender, patch(
            'builtins.input', side_effect=['set 8 0 1', 'test 5', 'next', 'next', 'quit']):
        assert module.main(['--dry-run', '--params-file', str(terminal.params_file)]) == 0
    sender.assert_not_called()
    output = capsys.readouterr().out
    assert 'tau_ff(URDF)' in output
    assert 'Khâu 3:' in output and 'Khâu 2:' in output and 'Khâu 1:' in output


def test_two_fixed_joints_keep_their_measured_reference():
    start = [0.03, 0.05, 0.1]
    result = tuner.trial_target(start, 2, 5, 6, [-1] * 3, [1] * 3, [2] * 3)
    assert result[:2] == start[:2]
    assert result[2] > start[2]
    assert start == [0.03, 0.05, 0.1]


def test_trial_rejects_joint_limit_and_excess_velocity():
    with pytest.raises(ValueError, match='giới hạn'):
        tuner.trial_target([0.0] * 3, 2, 90, 6, [-1] * 3, [1] * 3, [2] * 3)
    with pytest.raises(ValueError, match='vận tốc'):
        tuner.trial_target([0.0] * 3, 2, 5, 0.001, [-1] * 3, [1] * 3, [2] * 3)


def test_runtime_urdf_must_enable_only_the_selected_torque_joint():
    from gim_control.hardware_description import configure_hardware_description
    description = (ROOT / 'src/gim_arm_description/urdf/gim_arm.urdf').read_text()
    for selected, joint in enumerate(tuner.JOINTS):
        configured = configure_hardware_description(description, torque_joint=joint)
        tuner.check_torque_selection(configured, selected)
        with pytest.raises(RuntimeError, match='chưa cô lập'):
            tuner.check_torque_selection(configured, (selected + 1) % 3)
    paired = configure_hardware_description(description, torque_joint='shoulder_elbow')
    with pytest.raises(RuntimeError, match='chưa cô lập'):
        tuner.check_torque_selection(paired, 2)
