"""Selected-joint staging, stability rejection and explicit save without CAN."""
from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from gim_control.smc_joint_tuner import SmcJointTerminal

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def terminal(tmp_path):
    path = tmp_path / 'profile.yaml'
    # Keep coverage of the legacy saturation terminal without retaining a
    # second, obsolete hardware profile in the shipped config directory.
    document = yaml.safe_load((ROOT / 'src/gim_arm_controller_smc/config/smc_circle_from_terminal_tuning.yaml').read_text())
    document['smc_controller']['ros__parameters'].update(
        smc_variant='saturation', lambda_gain=[15.7, 20., 18.],
        ks=[65., 44., 30.], kr=[3., 4.2, 1.], phi=[.2, .15, .5])
    path.write_text(yaml.safe_dump(document))
    return SmcJointTerminal(path, dry_run=True)


@pytest.fixture
def sta_terminal(tmp_path):
    path = tmp_path / 'sta.yaml'
    path.write_text((ROOT /
        'src/gim_arm_controller_smc/config/smc_circle_from_terminal_tuning.yaml').read_text())
    return SmcJointTerminal(path, dry_run=True)


def test_gains_only_change_selected_joint_and_save_is_explicit(terminal):
    before = deepcopy(terminal.parameters)
    contents = terminal.session.params_file.read_text()
    terminal.execute('set 7 8 1.2 0.2')
    for name in ('lambda_gain', 'ks', 'kr', 'phi'):
        assert terminal.parameters[name][:2] == before[name][:2]
    assert terminal.parameters['lambda_gain'][2] == 7
    assert terminal.session.params_file.read_text() == contents
    terminal.dry_run = False
    terminal.execute('save')
    restored = SmcJointTerminal(terminal.session.params_file, dry_run=True)
    assert restored.parameters == terminal.parameters
    terminal.execute('lambda 8')
    terminal.execute('reset')
    assert terminal.parameters['lambda_gain'][2] == 7


def test_order_slew_and_base_only_alpha(terminal):
    before = deepcopy(terminal.parameters['torque_slew_rate'])
    terminal.execute('slew 6')
    assert terminal.parameters['torque_slew_rate'] == before[:2] + [6]
    with pytest.raises(ValueError, match='alpha'):
        terminal.execute('alpha 2')
    terminal.execute('next')
    assert terminal.selected == 1
    terminal.execute('next')
    assert terminal.selected == 0
    terminal.execute('alpha 2.3')
    assert terminal.parameters['base_ff_alpha'] == 2.3


@pytest.mark.parametrize('command', ['lambda nan', 'ks 0', 'kr -1', 'phi 0',
                                    'slew inf', 'set 155 180 90 0.001'])
def test_bad_or_unstable_commands_do_not_mutate_profile(terminal, command):
    before = deepcopy(terminal.parameters)
    with pytest.raises(ValueError):
        terminal.execute(command)
    assert terminal.parameters == before


def test_dry_run_never_uses_ros_or_writes_yaml(terminal, monkeypatch):
    contents = terminal.session.params_file.read_text()
    monkeypatch.setattr(terminal.trial, 'run', lambda *args: pytest.fail('opened ROS'))
    terminal.execute('test 3 6 3')
    terminal.execute('save')
    assert terminal.session.params_file.read_text() == contents


def test_sta_edits_selected_joint_save_and_reset(sta_terminal):
    terminal = sta_terminal
    before = deepcopy(terminal.parameters)
    contents = terminal.session.params_file.read_text()
    terminal.execute('set 5 3.5 1.2')
    terminal.execute('k1 4')
    terminal.execute('k2 1.5')
    terminal.execute('zlimit 25')
    for name in ('lambda_gain', 'k1', 'k2', 'z_limit'):
        assert terminal.parameters[name][:2] == before[name][:2]
    assert terminal.parameters['lambda_gain'][2] == 5
    assert terminal.parameters['k1'][2] == 4
    assert terminal.parameters['k2'][2] == 1.5
    assert terminal.parameters['z_limit'][2] == 25
    assert terminal.session.params_file.read_text() == contents
    terminal.dry_run = False
    terminal.execute('save')
    restored = SmcJointTerminal(terminal.session.params_file, dry_run=True)
    assert restored.parameters == terminal.parameters
    terminal.execute('lambda 6')
    terminal.execute('reset')
    assert terminal.parameters['lambda_gain'][2] == 5
    assert terminal.parameters['smc_variant'] == 'super_twisting'


@pytest.mark.parametrize('command', ['set 4 3 1 0.2', 'set 4 3', 'k1 nan', 'k2 0',
                                    'zlimit -1', 'ks 8', 'phi 0.2'])
def test_sta_bad_or_legacy_commands_do_not_mutate_profile(sta_terminal, command):
    before = deepcopy(sta_terminal.parameters)
    with pytest.raises(ValueError):
        sta_terminal.execute(command)
    assert sta_terminal.parameters == before


def test_sta_dry_run_help_and_trial_preserve_variant(sta_terminal, monkeypatch, capsys):
    contents = sta_terminal.session.params_file.read_text()
    sta_terminal.execute('help')
    sta_terminal.execute('show')
    output = capsys.readouterr().out
    assert 'set LAMBDA K1 K2' in output
    assert 'set LAMBDA KS KR PHI' not in output
    assert 'super-twisting' in output
    assert 'rho_ZOH_ideal=' not in output
    monkeypatch.setattr(sta_terminal.trial, 'run', lambda *args: pytest.fail('opened ROS'))
    sta_terminal.execute('test 3 6 3')
    sta_terminal.execute('save')
    assert sta_terminal.session.params_file.read_text() == contents
    received = []
    monkeypatch.setattr(sta_terminal.trial, 'run', lambda *args: received.append(args))
    sta_terminal.dry_run = False
    sta_terminal.execute('k1 3.25')
    sta_terminal.execute('test 3 6 3')
    selected, degrees, move, hold, params = received[0]
    assert (selected, degrees, move, hold) == (2, 3., 6., 3.)
    assert params['smc_variant'] == 'super_twisting'
    assert params['k1'][2] == 3.25
    params['k1'][2] = 99.
    assert sta_terminal.parameters['k1'][2] == 3.25


def test_sta_next_selects_shoulder_then_base(sta_terminal):
    expected = list(sta_terminal.parameters['k1'])
    expected[1] = 4.5
    sta_terminal.execute('next')
    sta_terminal.execute('k1 4.5')
    assert sta_terminal.selected == 1
    assert sta_terminal.parameters['k1'] == expected
    sta_terminal.execute('next')
    sta_terminal.execute('set 3.5 3.25 1.1')
    assert sta_terminal.selected == 0
    assert sta_terminal.parameters['lambda_gain'][0] == 3.5


def test_kalman_profile_survives_load_save_and_trial_without_ros(tmp_path, monkeypatch, capsys):
    path = tmp_path / 'kalman.yaml'
    document = yaml.safe_load((ROOT / 'src/gim_arm_controller_smc/config/smc_circle_from_terminal_tuning.yaml').read_text())
    params = document['smc_controller']['ros__parameters']
    params.update(velocity_kalman_enabled=[False, False, True],
                  velocity_kalman_jerk_psd=[1, 1, 1],
                  velocity_kalman_measurement_variance=[.0004] * 3)
    path.write_text(yaml.safe_dump(document))
    terminal = SmcJointTerminal(path)
    terminal.execute('show')
    assert 'Kalman [v, a]' in capsys.readouterr().out
    terminal.execute('save')
    terminal.execute('reset')
    received = []
    monkeypatch.setattr(terminal.trial, 'run', lambda *args: received.append(args[-1]))
    terminal.execute('test 3 6 3')
    assert received[0]['velocity_kalman_enabled'] == [False, False, True]
    assert all(type(value) is float for value in received[0]['velocity_kalman_jerk_psd'])
    bad = yaml.safe_load(path.read_text())
    bad['smc_controller']['ros__parameters']['velocity_lpf_hz'] = [0., 0., 3.]
    path.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError, match='LPF và Kalman'):
        SmcJointTerminal(path)


def test_integer_tau_scale_load_save_reset_and_trial_use_ros_double(tmp_path, monkeypatch):
    source = ROOT / 'src/gim_arm_controller_smc/config/smc_circle_from_terminal_tuning.yaml'
    document = yaml.safe_load(source.read_text())
    params = document['smc_controller']['ros__parameters']
    params['tau_scale'] = 1
    for name in ('lambda_gain', 'k1', 'k2', 'z_limit'):
        params[name] = [3, 4, 5]
    path = tmp_path / 'integers.yaml'
    path.write_text(yaml.safe_dump(document))
    terminal = SmcJointTerminal(path, joint='shoulder')
    received = []
    monkeypatch.setattr(terminal.trial, 'run', lambda *args: received.append(args))
    assert type(terminal.parameters['tau_scale']) is float
    terminal.execute('set 6 6 12')
    terminal.execute('slew 8')
    terminal.execute('save')
    saved = yaml.safe_load(path.read_text())['smc_controller']['ros__parameters']
    assert saved['tau_scale'] == 1.0 and type(saved['tau_scale']) is float
    assert type(saved['joint_state_qos_depth']) is int
    terminal.execute('reset')
    terminal.execute('test 10 5 2')
    selected, degrees, move, hold, trial_params = received[0]
    assert (selected, degrees, move, hold) == (1, 10., 5., 2.)
    assert type(trial_params['tau_scale']) is float
    assert all(type(v) is float for name in ('lambda_gain','k1','k2','z_limit')
               for v in trial_params[name])
