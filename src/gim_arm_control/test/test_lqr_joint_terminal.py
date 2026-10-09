"""Selected-joint Q/R editing and staging, without ROS/CAN."""

from copy import deepcopy
from pathlib import Path
from unittest.mock import Mock

import pytest
import yaml

from gim_control.lqr_joint_tuner import LqrJointTerminal, validate_joint_profile

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def terminal(tmp_path):
    source = ROOT / 'src/gim_arm_controller_lqr/config/lqr_joint_tuning.yaml'
    params = tmp_path / 'lqr.yaml'
    params.write_text(source.read_text())
    return LqrJointTerminal(params, show_plot=False, log_dir=tmp_path / 'results')


def test_costs_edit_only_selected_joint_and_save_bryson_equivalent(terminal):
    before = deepcopy(terminal.parameters)
    terminal.execute('set 400 100 10')
    assert terminal.costs() == pytest.approx((400, 100, 10))
    for key in ('max_e', 'max_int_e', 'max_tau'):
        assert terminal.parameters[key][:2] == before[key][:2]
    assert terminal.parameters['tau_scale'] == before['tau_scale']
    assert terminal.parameters['max_tau'][2] != before['max_tau'][2]
    terminal.execute('save')
    saved = LqrJointTerminal(terminal.session.params_file, show_plot=False)
    assert saved.costs() == pytest.approx((400, 100, 10))


def test_velocity_cost_is_selected_joint_only_and_survives_save(terminal, capsys):
    before = deepcopy(terminal.parameters)
    terminal.execute('qv 4')
    assert terminal.parameters['max_de'] == [2.0, 2.0, 0.5]
    for key in ('max_e', 'max_int_e', 'max_tau'):
        assert terminal.parameters[key] == before[key]
    terminal.execute('set 400 100 1 10')
    assert terminal.costs() == pytest.approx((400, 100, 10))
    assert terminal.parameters['max_de'] == [2.0, 2.0, 1.0]
    terminal.execute('set 225 25 20')  # Old syntax preserves QV.
    assert terminal.parameters['max_de'][2] == 1.0
    terminal.execute('save')
    saved = LqrJointTerminal(terminal.session.params_file, show_plot=False)
    saved.show()
    assert saved.parameters['max_de'] == [2.0, 2.0, 1.0]
    assert 'QV=1' in capsys.readouterr().out
    terminal.execute('next')
    terminal.execute('max_de 0.5')
    assert terminal.parameters['max_de'] == [2.0, 0.5, 1.0]


def test_stages_elbow_shoulder_base_and_preserves_other_costs(terminal):
    terminal.execute('qe 225')
    terminal.execute('next')
    assert terminal.selected == 1
    terminal.execute('qi 16')
    terminal.execute('next')
    assert terminal.selected == 0
    terminal.execute('r 30')
    terminal.execute('alpha 1.8')
    terminal.execute('next')
    assert terminal.selected == 0
    assert terminal.parameters['base_ff_alpha'] == 1.8
    assert 1 / terminal.parameters['max_e'][2]**2 == pytest.approx(225)
    assert 1 / terminal.parameters['max_int_e'][1]**2 == pytest.approx(16)


@pytest.mark.parametrize('command', [
    'set 100 nan 20', 'qe -1', 'qi 0', 'r inf', 'set 1 2', 'alpha 2',
    'slew 0', 'test nan', 'test 5 0', 'test 5 inf', 'test 5 6 -1',
    'max_e 1e300', 'max_int_e 1e-300', 'max_tau 1e-300',
    'qv 0', 'qv nan', 'qv -1', 'max_de 1e300', 'max_de 1e-300',
    'set 100 25 nan 10', 'set 100 25 0 10',
])
def test_invalid_commands_preserve_state_and_never_start_trial(terminal, command):
    before = deepcopy(terminal.parameters)
    terminal.trial = Mock()
    with pytest.raises(ValueError):
        terminal.execute(command)
    assert terminal.parameters == before
    terminal.trial.run.assert_not_called()


def test_test_passes_current_profile_without_writing_original(terminal):
    before = terminal.session.params_file.read_text()
    terminal.trial = Mock()
    terminal.execute('qe 400')
    terminal.execute('qv 4')
    terminal.execute('test 3 6 3')
    selected, degrees, move, hold, params = terminal.trial.run.call_args.args
    assert (selected, degrees, move, hold) == (2, 3.0, 6.0, 3.0)
    assert params['max_e'][2] == 0.05
    assert params['max_de'][2] == 0.5
    assert terminal.session.params_file.read_text() == before
    terminal.execute('qe 100')
    assert params['max_e'][2] == 0.05  # Snapshot is not mutated by later edits.


def test_dry_run_never_saves_or_opens_ros(terminal):
    before = terminal.session.params_file.read_text()
    terminal.dry_run = True
    terminal.trial = Mock()
    terminal.execute('qe 200')
    terminal.execute('test 3')
    terminal.execute('save')
    assert terminal.session.params_file.read_text() == before
    terminal.trial.run.assert_not_called()


@pytest.mark.parametrize('field,value', [
    ('control_hz', 2000), ('use_discrete_lqr', False),
    ('require_discrete_stable', False), ('tau_scale', 2),
    ('base_ff_alpha', float('nan')), ('gain_schedule_hz', 0),
    ('gain_schedule_mode', 'reference_state'),
    ('position_tracking_only', True),
])
def test_reject_incompatible_hardware_profile(terminal, field, value):
    params = deepcopy(terminal.parameters)
    params[field] = value
    with pytest.raises(ValueError):
        validate_joint_profile(params)
