"""Selected-joint staging, stability rejection and explicit save without CAN."""
from copy import deepcopy
from pathlib import Path

import pytest

from gim_control.smc_joint_tuner import SmcJointTerminal

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def terminal(tmp_path):
    path = tmp_path / 'profile.yaml'
    path.write_text((ROOT / 'src/gim_arm_controller_smc/config/smc_joint_tuning.yaml').read_text())
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
    terminal.execute('slew 6')
    assert terminal.parameters['torque_slew_rate'] == [5, 40, 6]
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
