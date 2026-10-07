"""Interactive PC-controller tuning; ROS/CAN is used only by explicit `test`."""

import argparse
from copy import deepcopy
from datetime import datetime
import importlib
import math
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import time
from uuid import uuid4

import yaml


# (length, strictly positive, integer). Vectors follow base, shoulder, elbow.
FIELDS = {
    'lqr': {
        'max_int_e': (3, True, False), 'max_e': (3, True, False),
        'max_tau': (3, True, False), 'tau_penalty_scale': (1, True, False),
        'integral_limit': (1, True, False),
        'max_tau_rate_nm_s': (3, True, False),
    },
    'mpc': {
        'q_integral': (3, False, False), 'q_position': (3, True, False),
        'r_input': (3, True, False), 'r_rate': (3, False, False),
        'output_scale_integral': (3, True, False),
        'output_scale_position': (3, True, False),
        'torque_penalty_scale': (1, True, False),
        'integral_limit': (1, True, False), 'torque_slew_rate': (3, True, False),
        'prediction_horizon': (1, True, True), 'control_horizon': (1, True, True),
    },
    'smc': {
        'lambda_gain': (3, True, False), 'ks': (3, True, False),
        'kr': (3, False, False), 'phi': (3, True, False),
    },
}
HELP = """show                         bộ số hiện tại (vector: base shoulder elbow)
set PARAM VALUE [VALUE VALUE] chỉnh tham số; áp dụng khi test tiếp theo
shape circle|r|a              chọn quỹ đạo
reset                        đọc lại bộ số từ file YAML
save [FILE.yaml]             lưu YAML trên máy; không ghi flash motor
test [circle|r|a]             một vòng: APPROACH -> TRACK -> RETURN -> HOLD
help
quit
Ctrl+C trong test: dừng bài thử, chuyển về position, ghi CSV.
Đóng cửa sổ đồ thị để quay lại terminal tune.
"""
POSITION = 'gim_arm_group_controller'
EFFORT = 'gim_arm_effort_controller'
COMMAND_TOPIC = '/gim_arm_effort_controller/commands'


def atomic_yaml(path, document):
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent,
                                         prefix='.' + path.name, delete=False) as stream:
            temporary = stream.name
            yaml.safe_dump(document, stream, sort_keys=False)
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)
    return path


class TuningSession:
    def __init__(self, algorithm, params_file, shape='circle'):
        self.algorithm = algorithm
        self.params_file = Path(params_file).expanduser().resolve()
        self.shape = shape
        self.reset()

    @property
    def params(self):
        return self.document[f'{self.algorithm}_controller']['ros__parameters']

    def reset(self):
        document = yaml.safe_load(self.params_file.read_text())
        self.validate(document)
        self.document = document

    def validate(self, document):
        try:
            params = document[f'{self.algorithm}_controller']['ros__parameters']
            for name in FIELDS[self.algorithm]:
                self.parse_value(name, params[name] if isinstance(params[name], list)
                                 else [params[name]])
            if self.algorithm == 'lqr' and params.get('position_tracking_only') is not True:
                raise ValueError('position_tracking_only phải là true')
            if self.algorithm == 'mpc':
                if params.get('q_velocity') != [0.0, 0.0, 0.0]:
                    raise ValueError('q_velocity phải là [0, 0, 0]')
                if params['control_horizon'] > params['prediction_horizon']:
                    raise ValueError('control_horizon phải <= prediction_horizon')
        except (KeyError, TypeError) as error:
            raise ValueError('YAML thiếu tham số của ' + self.algorithm) from error

    def parse_value(self, name, words):
        if name not in FIELDS[self.algorithm]:
            raise ValueError('Tham số được chỉnh: ' + ', '.join(FIELDS[self.algorithm]))
        length, positive, integer = FIELDS[self.algorithm][name]
        if len(words) != length:
            raise ValueError(f'{name}: cần {length} giá trị')
        values = [float(word) for word in words]
        if any(not math.isfinite(v) or v < 0 or (positive and v == 0) for v in values):
            raise ValueError(f'{name}: cần số hữu hạn ' + ('> 0' if positive else '>= 0'))
        if integer:
            if any(v != int(v) or v > 200 for v in values):
                raise ValueError('Horizon phải là số nguyên trong [1, 200]')
            values = [int(v) for v in values]
        return values[0] if length == 1 else values

    def set_value(self, name, words):
        value = self.parse_value(name, words)
        candidate = deepcopy(self.document)
        candidate[f'{self.algorithm}_controller']['ros__parameters'][name] = value
        self.validate(candidate)
        self.document = candidate

    def trial_document(self, log_file, urdf):
        document = deepcopy(self.document)
        document[f'{self.algorithm}_controller']['ros__parameters'].update({
            'autostart': False, 'use_sim_time': False, 'algorithm_hold': True,
            'cascade_hold': False, 'diagnostic_hold': False, 'diagnostic_segment': False,
            'trajectory_shape': self.shape, 'loops': 1.0,
            'approach_time': 16.0, 'return_time': 16.0, 'cache_file': '',
            'start_velocity_limit_rad_s': 0.05, 'log_file': str(log_file),
            'urdf_file': str(urdf), 'command_topic': COMMAND_TOPIC,
        })
        return document

    def show(self):
        print(f'{self.algorithm.upper()} | shape={self.shape} | YAML={self.params_file}')
        for name in FIELDS[self.algorithm]:
            print(f'  {name}: {self.params[name]}')

    def save(self, path=None):
        self.validate(self.document)
        target = atomic_yaml(path or self.params_file, self.document)
        print(f'Saved: {target}')


def require_position(states):
    if states.get(POSITION) != 'active' or states.get(EFFORT) != 'inactive':
        raise RuntimeError(f'Cần {POSITION}=active và {EFFORT}=inactive; hiện tại {states}')


class ControllerManager:
    """Keep servicing effort callbacks while waiting for manager responses."""
    def __init__(self, node, executor):
        from controller_manager_msgs.srv import ListControllers, SwitchController
        self.executor = executor
        self.list_client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.switch_client = node.create_client(SwitchController, '/controller_manager/switch_controller')
        self.list_type, self.switch_type = ListControllers, SwitchController

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('Không có controller_manager; mở hardware ở Terminal 1 trước')
        future = client.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        if not future.done():
            # A timed-out switch may still finish remotely: the caller must restore.
            raise RuntimeError('controller_manager không phản hồi trong 5s')
        if future.exception():
            raise RuntimeError(str(future.exception()))
        return future.result()

    def states(self):
        result = self.call(self.list_client, self.list_type.Request())
        return {c.name: c.state for c in result.controller}

    def switch(self, activate, deactivate):
        request = self.switch_type.Request()
        request.activate_controllers = [activate]
        request.deactivate_controllers = [deactivate]
        request.strictness = request.STRICT
        request.activate_asap = True
        request.timeout.sec = 3
        if not self.call(self.switch_client, request).ok:
            raise RuntimeError('Switch controller thất bại')


def restore_position(manager):
    """Never tear down the torque publisher before confirming effort is inactive."""
    while True:
        try:
            states = manager.states()
            if states.get(POSITION) == 'active' and states.get(EFFORT) == 'inactive':
                print('POSITION active; EFFORT inactive')
                return
            manager.switch(POSITION, EFFORT)
        except (RuntimeError, KeyboardInterrupt) as error:
            print(f'Đang giữ node mô-men; chờ chuyển về position: {error}', flush=True)
            # Keep its feedback/holding loop alive during retries or manual recovery.
            until = time.monotonic() + 1.0
            while time.monotonic() < until:
                try:
                    manager.executor.spin_once(timeout_sec=0.01)
                except KeyboardInterrupt:
                    pass


def run_ros_trial(algorithm, params_file):
    # Lazy imports: dry-run and parameter editing work without ROS or hardware.
    import numpy as np
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from gim_control.effort_controller_node import EffortControllerNode

    rclpy.init(args=['--ros-args', '--params-file', str(params_file)],
                signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor()
    admin = Node('algorithm_tuner')
    executor.add_node(admin)
    manager = ControllerManager(admin, executor)
    node = None
    attempted_switch = False
    try:
        require_position(manager.states())
        # Allow DDS discovery before checking for a competing torque publisher.
        discovery_end = time.monotonic() + 1.0
        while time.monotonic() < discovery_end:
            executor.spin_once(timeout_sec=0.05)
        if admin.count_publishers(COMMAND_TOPIC):
            raise RuntimeError('Đã có node phát effort; tắt Terminal 2 cũ trước khi tune')
        module = importlib.import_module(f'gim_arm_controller_{algorithm}.factory')
        factory = getattr(module, {'lqr': 'LqrFactory', 'mpc': 'MpcFactory', 'smc': 'SmcFactory'}[algorithm])
        node = EffortControllerNode(factory())
        executor.add_node(node)
        if algorithm == 'smc' and np.any(node.controller.sampled_spectral_radius() >= 1.0):
            raise RuntimeError('Bộ gain SMC không đạt kiểm tra ổn định ZOH ở 100 Hz')
        deadline = time.monotonic() + 180.0
        while node.phase != 'GRAVITY':
            executor.spin_once(timeout_sec=0.01)
            if node.phase == 'ABORT' or time.monotonic() > deadline:
                raise RuntimeError('Không READY; kiểm tra joint_states, giới hạn khớp và precompute')
        if np.max(np.abs(node.qd)) > node.start_velocity_limit:
            raise RuntimeError('Tay chưa đứng yên; chưa chuyển effort')
        require_position(manager.states())
        attempted_switch = True  # Set before the request: a timeout is not a cancellation.
        manager.switch(EFFORT, POSITION)
        states = manager.states()
        if states.get(EFFORT) != 'active' or states.get(POSITION) != 'inactive':
            raise RuntimeError('Chưa xác nhận effort active')
        results = node.set_parameters([Parameter('autostart', value=True)])
        if not all(result.successful for result in results):
            raise RuntimeError('Không bật được autostart')
        deadline = time.monotonic() + 120.0
        while node.phase != 'HOLD':
            executor.spin_once(timeout_sec=0.01)
            if node.phase == 'ABORT' or time.monotonic() > deadline:
                raise RuntimeError('Bài thử ABORT hoặc quá thời gian; xem log phía trên')
        until = time.monotonic() + 3.0
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=0.01)
        print('Trial complete')
    finally:
        if attempted_switch:
            restore_position(manager)
        try:
            if node is not None:
                node.dump()
        finally:
            executor.shutdown()
            if node is not None:
                node.destroy_node()
            admin.destroy_node()
            rclpy.shutdown()


def test_session(session, root, log_dir, dry_run=False, no_show=False):
    run_id = datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid4().hex[:6]
    log_file = log_dir / f'{session.algorithm}_{session.shape}_{run_id}.csv'
    document = session.trial_document(log_file, root / 'src/gim_arm_description/urdf/gim_arm.urdf')
    if dry_run:
        print(f'DRY RUN: {session.algorithm}/{session.shape}; q, qd, qdd; 16s + 27s + 16s + 3s HOLD')
        print('Không kết nối ROS/CAN, không switch controller, không tạo CSV.')
        print(yaml.safe_dump(document, sort_keys=False))
        return
    snapshot = atomic_yaml(log_file.with_suffix('.yaml'), document)
    try:
        run_ros_trial(session.algorithm, snapshot)
    except KeyboardInterrupt:
        print('Đã dừng bài thử.')
    except Exception as error:
        print(f'Test error: {error}')
    if log_file.exists():
        command = ['ros2', 'run', 'gim_control', 'plot_joint_tracking', str(log_file)]
        if not no_show:
            command.append('--show')
        subprocess.run(command, check=False)
    print(f'Tham số bài thử: {snapshot}')


def main(algorithm, root):
    root = Path(root).resolve()
    parser = argparse.ArgumentParser(description=f'Tune {algorithm.upper()} từ terminal')
    parser.add_argument('--params-file', type=Path, default=root /
                        f'src/gim_arm_controller_{algorithm}/config/{algorithm}_hardware_soft.yaml')
    parser.add_argument('--trajectory-shape', choices=('circle', 'r', 'a'), default='circle')
    parser.add_argument('--log-dir', type=Path, default=root / f'results/tune_{algorithm}')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--no-show', action='store_true')
    args = parser.parse_args()
    try:
        session = TuningSession(algorithm, args.params_file, args.trajectory_shape)
    except (ValueError, OSError, yaml.YAMLError) as error:
        parser.error(str(error))
    args.log_dir = args.log_dir.expanduser().resolve()
    print(HELP)
    session.show()
    while True:
        try:
            words = shlex.split(input(f'{algorithm}> '))
            if not words:
                continue
            command, values = words[0].lower(), words[1:]
            if command in ('quit', 'exit') and not values:
                return
            if command == 'help' and not values:
                print(HELP)
            elif command == 'show' and not values:
                session.show()
            elif command == 'set' and len(values) >= 2:
                session.set_value(values[0], values[1:])
                print(f'{values[0]} = {session.params[values[0]]} (test tiếp theo)')
            elif command == 'reset' and not values:
                session.reset()
                session.show()
            elif command == 'save' and len(values) <= 1:
                session.save(values[0] if values else None)
            elif command in ('shape', 'test') and len(values) <= 1:
                if command == 'shape' and not values:
                    raise ValueError('shape circle|r|a')
                if values:
                    if values[0] not in ('circle', 'r', 'a'):
                        raise ValueError('Quỹ đạo: circle|r|a')
                    session.shape = values[0]
                if command == 'test':
                    test_session(session, root, args.log_dir, args.dry_run, args.no_show)
                else:
                    print(f'shape = {session.shape}')
            else:
                raise ValueError('Lệnh không hợp lệ; gõ help')
        except (EOFError, KeyboardInterrupt):
            print('\nKết thúc tune; dùng save để lưu bộ số trước khi quit.')
            return
        except (ValueError, OSError, yaml.YAMLError) as error:
            print(f'Error: {error}')
