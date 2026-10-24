"""Check staged physical Q/R/Rd costs and per-joint isolation without CAN."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from gim_control.mpc_joint_tuner import MpcJointTerminal
from gim_arm_controller_mpc.factory import MpcFactory

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def terminal(tmp_path):
    path = tmp_path / 'profile.yaml'
    path.write_text((ROOT / 'src/gim_arm_controller_mpc/config/mpc_joint_tuning.yaml').read_text())
    return MpcJointTerminal(path, dry_run=True)


def test_terminal_costs_reach_actual_controller_and_other_joints_stay_unchanged(terminal):
    before = deepcopy(terminal.parameters)
    terminal.execute('set 1600 600 1 10 0.3')
    assert terminal.costs() == pytest.approx((1600, 600, 1, 10, .3))
    for key, values in before.items():
        if isinstance(values, list):
            assert terminal.parameters[key][:2] == values[:2]
    params = terminal.parameters
    class Parameters:
        def get_parameter(self, name):
            return SimpleNamespace(value=params[name])
    from gim_control.arm_dynamics import ArmDynamics
    dynamics = ArmDynamics(str(ROOT / 'src/gim_arm_description/urdf/gim_arm.urdf'))
    c = MpcFactory().build(Parameters(), dynamics, dynamics.tau_max, 100)
    assert np.diag(c.Q)[[2, 5, 8]] == pytest.approx((600, 1600, 1))
    assert c.R[2, 2] == pytest.approx(10)
    assert c.Rd[2, 2] == pytest.approx(.3)


def test_set_four_costs_keeps_rd_and_save_is_explicit(terminal):
    before = terminal.session.params_file.read_text()
    rd = terminal.costs()[-1]
    terminal.execute('set 200 100 1 2')
    assert terminal.costs()[-1] == pytest.approx(rd)
    assert terminal.session.params_file.read_text() == before
    terminal.dry_run = False
    terminal.execute('save')
    restored = MpcJointTerminal(terminal.session.params_file, dry_run=True)
    assert restored.costs() == pytest.approx(terminal.costs())


def test_stage_order_slew_alpha_and_horizons(terminal):
    other_slew = terminal.parameters['torque_slew_rate'][:2]
    terminal.execute('slew 6')
    assert terminal.parameters['torque_slew_rate'] == other_slew + [6]
    with pytest.raises(ValueError, match='alpha'):
        terminal.execute('alpha 2.3')
    terminal.execute('next')
    assert terminal.selected == 1
    terminal.execute('next')
    assert terminal.selected == 0
    terminal.execute('alpha 2.3')
    terminal.execute('np 50')
    terminal.execute('nc 12')
    assert terminal.parameters['base_ff_alpha'] == 2.3
    assert terminal.parameters['prediction_horizon'] == 50
    assert terminal.parameters['control_horizon'] == 12
    with pytest.raises(ValueError):
        terminal.execute('np 5')
    assert terminal.parameters['prediction_horizon'] == 50


@pytest.mark.parametrize('command', ['qe nan', 'qv -1', 'r 0', 'rd -1',
                                    'set 1 2 0 4', 'slew inf', 'nc 2.5'])
def test_bad_commands_do_not_mutate_profile(terminal, command):
    before = deepcopy(terminal.parameters)
    with pytest.raises(ValueError):
        terminal.execute(command)
    assert terminal.parameters == before


def test_dry_run_never_connects_to_ros_or_saves(terminal, monkeypatch):
    before = terminal.session.params_file.read_text()
    monkeypatch.setattr(terminal.trial, 'run', lambda *args: pytest.fail('dry-run used ROS'))
    terminal.execute('test 3 6 3')
    terminal.execute('save')
    assert terminal.session.params_file.read_text() == before


def test_independent_mode_survives_edits_save_and_trial(tmp_path, monkeypatch, capsys):
    path = tmp_path / 'independent.yaml'
    source = ROOT / 'src/gim_arm_controller_mpc/config/mpc_circle_independent_feedback.yaml'
    path.write_text(source.read_text())
    terminal = MpcJointTerminal(path)
    terminal.execute('show')
    assert 'feedback_model_mode=independent' in capsys.readouterr().out
    terminal.execute('set 500 256 0.04 0.037636 0.36')
    terminal.execute('save')
    restored = MpcJointTerminal(path)
    assert restored.parameters['feedback_model_mode'] == 'independent'
    received = []
    monkeypatch.setattr(restored.trial, 'run', lambda *args: received.append(args[-1]))
    restored.execute('test 3 6 2')
    assert received[0]['feedback_model_mode'] == 'independent'
    assert restored.costs() == pytest.approx((500, 256, .04, .037636, .36))


def test_bad_feedback_mode_is_rejected_before_opening_ros(tmp_path):
    source = ROOT / 'src/gim_arm_controller_mpc/config/mpc_circle_independent_feedback.yaml'
    document = yaml.safe_load(source.read_text())
    document['mpc_controller']['ros__parameters']['feedback_model_mode'] = 'typo'
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='feedback_model_mode'):
        MpcJointTerminal(path)
