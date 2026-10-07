"""Terminal edits and hardware trial ownership/cleanup, without CAN."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from gim_control import algorithm_tuner as tuner

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(params=['lqr', 'mpc', 'smc'])
def session(request, tmp_path):
    algorithm = request.param
    source = ROOT / f'src/gim_arm_controller_{algorithm}/config/{algorithm}_hardware_soft.yaml'
    config = tmp_path / 'profile.yaml'
    config.write_text(source.read_text())
    return tuner.TuningSession(algorithm, config)


def test_edit_is_staged_and_save_is_explicit(session, tmp_path):
    before = session.params_file.read_text()
    name = next(iter(tuner.FIELDS[session.algorithm]))
    session.set_value(name, ['0.3', '0.4', '0.5'])
    assert session.params_file.read_text() == before
    target = tmp_path / 'chosen.yaml'
    session.save(target)
    saved = yaml.safe_load(target.read_text())
    assert saved[f'{session.algorithm}_controller']['ros__parameters'][name] == [0.3, 0.4, 0.5]
    session.reset()
    assert session.params[name] != [0.3, 0.4, 0.5]


@pytest.mark.parametrize('values', [['nan', '1', '1'], ['inf', '1', '1'],
                                   ['-1', '1', '1'], ['0', '1', '1'], ['1']])
def test_reject_invalid_gain_without_mutation(session, values):
    before = deepcopy(session.document)
    name = next(name for name, spec in tuner.FIELDS[session.algorithm].items() if spec[1])
    with pytest.raises(ValueError):
        session.set_value(name, values)
    assert session.document == before


def test_safety_and_position_objective_are_not_editable(session):
    for name in ('control_hz', 'tau_scale', 'max_track_error_rad', 'q_velocity',
                 'position_tracking_only', 'autostart', 'command_topic'):
        with pytest.raises(ValueError):
            session.set_value(name, ['1'])


def test_snapshot_has_full_trajectory_and_keeps_staged_profile(session, tmp_path):
    original = deepcopy(session.document)
    for shape in ('circle', 'r', 'a'):
        session.shape = shape
        snapshot = session.trial_document(tmp_path / 'trial.csv', tmp_path / 'robot.urdf')
        params = snapshot[f'{session.algorithm}_controller']['ros__parameters']
        assert params['trajectory_shape'] == shape
        assert params['algorithm_hold'] is True
        assert params['autostart'] is False
        assert params['approach_time'] == params['return_time'] == 16.0
        assert params['loops'] == 1.0
    assert session.document == original


def test_dry_run_does_not_call_ros_or_write_trial(session, tmp_path, monkeypatch):
    def unexpected(*args):
        pytest.fail('Dry run accessed ROS')
    monkeypatch.setattr(tuner, 'run_ros_trial', unexpected)
    tuner.test_session(session, ROOT, tmp_path / 'logs', dry_run=True)
    assert not (tmp_path / 'logs').exists()


def test_horizon_validation_is_transactional(tmp_path):
    source = ROOT / 'src/gim_arm_controller_mpc/config/mpc_hardware_soft.yaml'
    session = tuner.TuningSession('mpc', source)
    before = deepcopy(session.document)
    for name, value in [('control_horizon', '41'), ('prediction_horizon', '9'),
                        ('control_horizon', '2.5'), ('prediction_horizon', '201')]:
        with pytest.raises(ValueError):
            session.set_value(name, [value])
        assert session.document == before


@pytest.mark.parametrize('algorithm,field,value', [
    ('lqr', 'position_tracking_only', False), ('mpc', 'q_velocity', [1.0, 0.0, 0.0])])
def test_reject_velocity_objective_from_custom_yaml(tmp_path, algorithm, field, value):
    source = ROOT / f'src/gim_arm_controller_{algorithm}/config/{algorithm}_hardware_soft.yaml'
    document = yaml.safe_load(source.read_text())
    document[f'{algorithm}_controller']['ros__parameters'][field] = value
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        tuner.TuningSession(algorithm, path)


@pytest.mark.parametrize('states', [
    {}, {tuner.POSITION: 'inactive', tuner.EFFORT: 'active'},
    {tuner.POSITION: 'active', tuner.EFFORT: 'active'}])
def test_start_requires_exclusive_position_controller(states):
    with pytest.raises(RuntimeError):
        tuner.require_position(states)


def test_restore_retries_uncertain_switch_and_waits_for_confirmation():
    calls = []
    class Manager:
        executor = SimpleNamespace(spin_once=lambda **kw: calls.append('spin'))
        polls = 0
        def states(self):
            self.polls += 1
            if self.polls >= 3:
                return {tuner.POSITION: 'active', tuner.EFFORT: 'inactive'}
            return {tuner.POSITION: 'inactive', tuner.EFFORT: 'active'}
        def switch(self, activate, deactivate):
            calls.append((activate, deactivate))
            if self.polls == 1:
                raise RuntimeError('response timeout; may have switched remotely')
    tuner.restore_position(Manager())
    assert calls.count((tuner.POSITION, tuner.EFFORT)) == 2
    assert 'spin' in calls


@pytest.mark.parametrize('failure', ['none', 'abort', 'interrupt', 'switch_timeout', 'not_ready'])
def test_ros_trial_restores_before_dump_and_destroy(tmp_path, monkeypatch, failure):
    # Real ROS context/Node; fake feedback and manager services, never a CAN driver.
    import rclpy.executors
    import rclpy.node
    from gim_control import effort_controller_node
    events = []
    class FakeNode(rclpy.node.Node):
        def __init__(self, factory):
            super().__init__('smc_controller')
            self.phase = 'ABORT' if failure == 'not_ready' else 'GRAVITY'
            self.qd = [0.0, 0.0, 0.0]
            self.start_velocity_limit = 0.05
            self.controller = SimpleNamespace(sampled_spectral_radius=lambda: __import__('numpy').array([0.5]*3))
        def set_parameters(self, params):
            events.append('start')
            self.phase = 'ABORT' if failure == 'abort' else 'HOLD'
            if failure == 'interrupt':
                raise KeyboardInterrupt
            return [SimpleNamespace(successful=True)]
        def dump(self):
            events.append('dump')
        def destroy_node(self):
            events.append('destroy')
            return super().destroy_node()
    class FakeExecutor:
        def add_node(self, node):
            pass
        def spin_once(self, **kw):
            pass
        def shutdown(self):
            pass
    class FakeManager:
        def __init__(self, *args):
            self.effort = False
        def states(self):
            return {tuner.POSITION: 'inactive' if self.effort else 'active',
                    tuner.EFFORT: 'active' if self.effort else 'inactive'}
        def switch(self, activate, deactivate):
            events.append(activate)
            self.effort = activate == tuner.EFFORT
            if failure == 'switch_timeout' and self.effort:
                raise RuntimeError('timeout after activation')
    clock = iter(range(1000))
    monkeypatch.setattr(tuner.time, 'monotonic', lambda: next(clock))
    monkeypatch.setattr(tuner, 'ControllerManager', FakeManager)
    monkeypatch.setattr(rclpy.executors, 'SingleThreadedExecutor', FakeExecutor)
    monkeypatch.setattr(rclpy.node.Node, 'count_publishers', lambda *args: 0)
    monkeypatch.setattr(effort_controller_node, 'EffortControllerNode', FakeNode)
    path = tmp_path / 'params.yaml'
    path.write_text('smc_controller:\n  ros__parameters: {}\n')
    if failure == 'none':
        tuner.run_ros_trial('smc', path)
    else:
        with pytest.raises(KeyboardInterrupt if failure == 'interrupt' else RuntimeError):
            tuner.run_ros_trial('smc', path)
    if failure == 'not_ready':
        assert tuner.EFFORT not in events
    else:
        assert events.index(tuner.POSITION) < events.index('dump') < events.index('destroy')


def test_staged_gains_rebuild_actual_controller_and_schedule(session, tmp_path):
    import importlib
    import numpy as np
    import rclpy
    from gim_control.effort_controller_node import EffortControllerNode
    algorithm = session.algorithm
    name, values, shape = {
        'lqr': ('max_e', ['0.08']*3, 'r'),
        'mpc': ('q_position', ['0.7', '1.7', '0.7'], 'a'),
        'smc': ('lambda_gain', ['3.1', '4.1', '4.1'], 'circle'),
    }[algorithm]
    session.set_value(name, values)
    session.shape = shape
    snapshot = tuner.atomic_yaml(tmp_path / 'trial.yaml', session.trial_document(
        tmp_path / 'trial.csv', ROOT / 'src/gim_arm_description/urdf/gim_arm.urdf'))
    module = importlib.import_module(f'gim_arm_controller_{algorithm}.factory')
    factory = getattr(module, {'lqr': 'LqrFactory', 'mpc': 'MpcFactory', 'smc': 'SmcFactory'}[algorithm])
    rclpy.init(args=['--ros-args', '--params-file', str(snapshot)])
    node = None
    try:
        node = EffortControllerNode(factory())
        assert node.get_parameter(name).value == [float(v) for v in values]
        node._prepare_controller_schedule(np.zeros(3))
        q, qd, qdd = node.trajectory.at(13.5)
        assert np.isfinite([q, qd, qdd]).all()
        if hasattr(node.controller, 'set_reference_time'):
            node.controller.set_reference_time(29.5)
        torque = node.controller.compute(q + 0.001, qd, q, qd, qdd, node.dt_nom)
        assert np.isfinite(torque).all()
        assert np.all(np.abs(torque) <= node.tau_limit + 1e-8)
        if algorithm == 'lqr':
            np.testing.assert_allclose(node.controller.weights.max_e, [0.08]*3)
        elif algorithm == 'mpc':
            assert node.controller.last['solver_accepted']
        else:
            np.testing.assert_allclose(node.controller.lambda_gain, [3.1, 4.1, 4.1])
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
