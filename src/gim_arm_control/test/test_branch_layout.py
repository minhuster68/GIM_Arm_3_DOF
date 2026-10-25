"""Source/launch isolation for a branch containing one controller, without CAN."""
import ast
from importlib import import_module, util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

ROOT = Path(__file__).resolve().parents[3]
PACKAGES = list((ROOT / 'src').glob('gim_arm_controller_*/package.xml'))
ALGORITHM = PACKAGES[0].parent.name.removeprefix('gim_arm_controller_')


def test_only_one_controller_and_no_foreign_imports():
    assert len(PACKAGES) == 1
    assert ALGORITHM in ('pid', 'lqr', 'mpc', 'smc')
    for folder in (ROOT / 'src/gim_arm_control', PACKAGES[0].parent, ROOT / 'tools'):
        for path in folder.rglob('*.py'):
            if not path.exists():
                continue
            for node in ast.walk(ast.parse(path.read_text())):
                names = ([node.module] if isinstance(node, ast.ImportFrom)
                         else [a.name for a in node.names] if isinstance(node, ast.Import) else [])
                for name in names:
                    if name and name.startswith('gim_arm_controller_'):
                        assert name.split('.')[0] == 'gim_arm_controller_' + ALGORITHM, path


def test_generic_launch_exposes_only_local_algorithm(monkeypatch):
    path = ROOT / 'src/gim_arm_control/launch/run_effort_algorithm.launch.py'
    spec = util.spec_from_file_location('branch_effort_launch', path)
    module = util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert list(module.ALGORITHMS) == [ALGORITHM]
    captured = []
    monkeypatch.setattr(module, 'Node', lambda **kw: captured.append(kw) or DeclareLaunchArgument('captured', default_value=''))
    monkeypatch.setattr(module, 'FindPackageShare', lambda name: str(ROOT / 'src' / name))
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert len(captured) == 1
    node = captured[0]
    assert node['package'] == 'gim_arm_controller_' + ALGORITHM
    assert Path(node['parameters'][0].perform(context)).is_file()
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    overrides = evaluate_parameters(context, normalize_parameters(node['parameters'][1:]))
    assert all(isinstance(params['control_hz'], float) and params['control_hz'] > 0 for params in overrides if 'control_hz' in params)


@pytest.mark.skipif(ALGORITHM == 'pid', reason='PID has its own sweep launch')
def test_hardware_dispatcher_only_routes_local_algorithm(tmp_path, monkeypatch):
    hardware = import_module('gim_control.hardware_trajectory_launch')
    assert list(hardware.ALGORITHMS) == [ALGORITHM]
    monkeypatch.setattr(hardware, 'get_package_share_directory', lambda name: str(ROOT / 'src' / name))
    monkeypatch.setattr(hardware, 'Node', lambda **kwargs: kwargs)
    for shape in ('circle', 'r', 'a'):
        context = LaunchContext()
        context.launch_configurations.update(
            algorithm=ALGORITHM, trajectory_shape=shape, params_file='',
            approach_time='16', return_time='16', log_file=str(tmp_path / 'trial.csv'))
        node, = hardware._launch_setup(context)
        assert node['package'] == 'gim_arm_controller_' + ALGORITHM
        assert Path(node['parameters'][0]).is_file()
        assert node['parameters'][1]['autostart'] is False
    for other in {'pid', 'lqr', 'mpc', 'smc'} - {ALGORITHM}:
        context.launch_configurations['algorithm'] = other
        with pytest.raises(ValueError):
            hardware._launch_setup(context)
