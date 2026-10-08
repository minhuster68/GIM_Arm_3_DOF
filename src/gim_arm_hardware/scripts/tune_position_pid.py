#!/usr/bin/env python3
"""Tune one position PID joint with URDF feedforward, elbow -> shoulder -> base."""

import argparse
import copy
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shlex
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from zoneinfo import ZoneInfo

import yaml


JOINTS = ('base', 'shoulder', 'elbow')
JOINT_NAMES = tuple(name + '_joint' for name in JOINTS)
GAIN_KEYS = dict(kp='position_kp', ki='position_ki', kd='position_kd')
HELP = """PID một vòng trên PC: tau = tau_ff(URDF) + tau_p + tau_i + tau_d.
Thứ tự: khâu 3 elbow -> khâu 2 shoulder -> khâu 1 base.
  show                   Xem Kp/Ki/Kd và YAML đang dùng
  set KP KI KD           Chỉnh ba gain của riêng khớp đang chọn
  kp VALUE / ki VALUE / kd VALUE
  test DEG [MOVE HOLD]   Đi thêm DEG độ, giữ, về điểm đầu; mặc định 6 / 3 s
  save                   Lưu gain ba khớp vào YAML, không ghi flash motor
  next                   Chuyển 3 -> 2 -> 1 và in lệnh launch phần cứng cần dùng
  help / quit

Set chỉ đổi gain trong terminal; mỗi test dựng PID mới với gain vừa nhập.
Hai khớp còn lại giữ bằng position mode tại góc chụp khi switch sang effort.
Khi đổi khớp, cần khởi động lại launch phần cứng với torque_joint tương ứng.
Test kiểm lựa chọn đó từ robot_description trước khi chuyển sang effort.
Sau test/Ctrl-C, trả về position rồi mới lưu CSV và mở đồ thị.
"""


def checked_gain(value):
    value = float(value)
    if not math.isfinite(value) or value < 0:
        raise ValueError('Gain phải hữu hạn và >= 0')
    return value


def default_params_file():
    source = Path(__file__).resolve().parents[2] / (
        'gim_arm_controller_pid/config/pid_hardware_soft.yaml')
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('gim_arm_controller_pid')) / (
        'config/pid_hardware_soft.yaml')


def load_config(path):
    config = yaml.safe_load(Path(path).read_text())
    parameters = config['cascade_pid_controller']['ros__parameters']
    for key in (*GAIN_KEYS.values(), 'position_integral_limit'):
        values = parameters[key]
        if not isinstance(values, list) or len(values) != 3:
            raise ValueError(f'{key} phải là vector [base, shoulder, elbow]')
        normalized = [checked_gain(value) for value in values]
        for value in normalized:
            if value == 0 and key == 'position_integral_limit':
                raise ValueError('position_integral_limit phải > 0')
        parameters[key] = normalized
    return config


def checked_trial(degrees, move, hold):
    values = [float(degrees), float(move), float(hold)]
    if not all(math.isfinite(value) for value in values) or min(values[1:]) <= 0:
        raise ValueError('DEG phải hữu hạn; MOVE và HOLD phải hữu hạn và > 0')
    if 2 * (values[1] + values[2]) >= 2**31 or min(values[1:]) < 1.0e-9:
        raise ValueError('MOVE/HOLD nằm ngoài phạm vi thời gian của bài thử')
    return values


def check_torque_selection(description, selected):
    root = ET.fromstring(description)
    control = root.find("ros2_control[@name='GimArmSystem']")
    if control is None:
        raise ValueError('robot_description thiếu GimArmSystem')
    enabled = []
    for name in JOINT_NAMES:
        param = control.find(f"joint[@name='{name}']/param[@name='torque_mode_enable']")
        if param is None or (param.text or '').strip().lower() not in ('true', 'false'):
            raise ValueError(f'Không xác minh được torque_mode_enable của {name}')
        enabled.append(param.text.strip().lower() == 'true')
    if enabled != [index == selected for index in range(3)]:
        raise RuntimeError(
            f'Phần cứng chưa cô lập {JOINTS[selected]}. Khởi động lại với '
            f'torque_joint:={JOINTS[selected]}; hai khớp còn lại phải giữ position.')


def trial_target(start, selected, degrees, move, lower, upper, velocity):
    target = list(start)
    target[selected] += math.radians(degrees)
    for pose in (start, target):
        for index, value in enumerate(pose):
            if not math.isfinite(value) or not lower[index] + 0.05 <= value <= upper[index] - 0.05:
                raise ValueError(f'{JOINT_NAMES[index]} gần/vượt giới hạn URDF')
    if 1.875 * abs(math.radians(degrees)) / move > velocity[selected]:
        raise ValueError('Quỹ đạo vượt vận tốc URDF; tăng MOVE')
    return target


class RosPositionTrial:
    """Run the existing effort PID runner, with only one hardware torque joint."""

    def __init__(self, log_dir, show_plot=True):
        self.log_dir = Path(log_dir)
        self.show_plot = show_plot
        self.pid = None

    def run(self, selected, degrees, move, hold, parameters):
        import rclpy
        from controller_manager_msgs.srv import ListControllers, SwitchController
        from rcl_interfaces.srv import GetParameters
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from gim_arm_controller_pid.factory import PositionPidFactory
        from gim_control.effort_controller_node import EffortControllerNode
        from gim_control.plot_ee_error import load_log, plot, print_metrics
        from gim_control.reference_trajectory import TimedHold

        self.pid = None
        if rclpy.ok():
            raise RuntimeError('Terminal cần một ROS context riêng, không chạy trong node khác')
        rclpy.init(args=[])
        executor = SingleThreadedExecutor()
        probe = rclpy.create_node('position_pid_trial')
        executor.add_node(probe)
        listing = probe.create_client(ListControllers, '/controller_manager/list_controllers')
        switching = probe.create_client(SwitchController, '/controller_manager/switch_controller')
        description_client = probe.create_client(
            GetParameters, '/controller_manager/get_parameters')
        state = {}

        def on_state(message):
            try:
                indices = [message.name.index(name) for name in JOINT_NAMES]
                q = [float(message.position[i]) for i in indices]
                qd = [float(message.velocity[i]) for i in indices]
            except (ValueError, IndexError):
                return
            if all(math.isfinite(value) for value in q + qd):
                state.update(q=q, qd=qd, received=time.monotonic())

        probe.create_subscription(JointState, '/joint_states', on_state, qos_profile_sensor_data)

        def call(client, request, timeout=5.0):
            deadline = time.monotonic() + timeout
            while not client.service_is_ready():
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'Chưa có service {client.srv_name}')
                executor.spin_once(timeout_sec=0.01)
            future = client.call_async(request)
            while not future.done():
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'Hết thời gian chờ {client.srv_name}')
                executor.spin_once(timeout_sec=0.01)
            return future.result()

        def switch(effort):
            request = SwitchController.Request()
            position_name = 'gim_arm_group_controller'
            effort_name = 'gim_arm_effort_controller'
            if not effort:
                states = {item.name: item.state for item in call(
                    listing, ListControllers.Request()).controller}
                if states.get(position_name) == 'active' and states.get(effort_name) == 'inactive':
                    return
            request.activate_controllers = [effort_name if effort else position_name]
            request.deactivate_controllers = [position_name if effort else effort_name]
            request.strictness = SwitchController.Request.STRICT
            request.activate_asap = True
            request.timeout.sec = 5
            if not call(switching, request, 8.0).ok:
                raise RuntimeError('Không chuyển được controller')

        active_effort = False
        path = None
        try:
            controllers = call(listing, ListControllers.Request())
            states = {item.name: item.state for item in controllers.controller}
            if (states.get('gim_arm_group_controller') != 'active'
                    or states.get('gim_arm_effort_controller') != 'inactive'):
                raise RuntimeError('Trước test: JTC phải active và effort phải inactive')
            request = GetParameters.Request(names=['robot_description'])
            descriptions = call(description_client, request).values
            if len(descriptions) != 1 or not descriptions[0].string_value:
                raise RuntimeError('Chưa đọc được robot_description từ /controller_manager')
            description = descriptions[0].string_value
            check_torque_selection(description, selected)
            if probe.count_publishers('/gim_arm_effort_controller/commands'):
                raise RuntimeError('Một node khác đang phát effort; dừng node PID cũ trước test')
            deadline = time.monotonic() + 3.0
            while not state or time.monotonic() - state['received'] > 0.25:
                if time.monotonic() >= deadline:
                    raise RuntimeError('Chưa có joint state mới với đủ góc/vận tốc')
                executor.spin_once(timeout_sec=0.01)
            if max(abs(value) for value in state['qd']) > 0.05:
                raise RuntimeError('Tay đang chuyển động; giữ yên trước test')

            stamp = datetime.now(ZoneInfo('Asia/Ho_Chi_Minh')).strftime('%Y%m%d_%H%M%S_%f')
            directory = self.log_dir / f'pid_{JOINTS[selected]}_{stamp}'
            directory.mkdir(parents=True)
            path = directory / 'tracking.csv'
            urdf = directory / 'model.urdf'
            urdf.write_text(description)
            config = copy.deepcopy(parameters)
            config.update(
                urdf_file=str(urdf.resolve()), log_file=str(path.resolve()),
                command_topic='/gim_arm_effort_controller/commands',
                use_sim_time=False, autostart=False, cascade_hold=True,
                gravity_scale=[1.0, 1.0, 1.0],
                diagnostic_hold=True, diagnostic_segment=False,
                approach_time=move, return_time=move, diagnostic_hold_time=hold,
                start_velocity_limit_rad_s=0.05, command_heartbeat_nm=1.0e-6)
            # Construct with a hold at the measured pose; update the target
            # after WAIT captures the exact HOME from a fresh state sample.
            for index in range(3):
                config[f'diagnostic_q{index + 1}_deg'] = math.degrees(state['q'][index])
            self.pid = EffortControllerNode(
                PositionPidFactory(),
                parameter_overrides=[Parameter(name, value=value) for name, value in config.items()])
            executor.add_node(self.pid)
            deadline = time.monotonic() + 3.0
            while self.pid.phase == 'WAIT':
                if time.monotonic() >= deadline:
                    raise RuntimeError('PID chưa nhận được joint states')
                executor.spin_once(timeout_sec=0.01)
            target = trial_target(
                self.pid.home_q, selected, degrees, move,
                self.pid.dynamics.q_min, self.pid.dynamics.q_max, self.pid.dynamics.vel_max)
            self.pid.trajectory = TimedHold(target, hold)
            for index, value in enumerate(target):
                config[f'diagnostic_q{index + 1}_deg'] = math.degrees(value)
            self.pid.set_parameters([
                Parameter(f'diagnostic_q{index + 1}_deg', value=math.degrees(value))
                for index, value in enumerate(target)])
            (directory / 'parameters.yaml').write_text(yaml.safe_dump({
                'cascade_pid_controller': {'ros__parameters': config}}, sort_keys=False))
            (directory / 'trial.json').write_text(json.dumps({
                'joint': JOINTS[selected], 'home_rad': self.pid.home_q.tolist(),
                'target_rad': target, 'move_s': move, 'hold_s': hold,
                'tau_ff': 'inverse_dynamics(runtime URDF, q_ref, qd_ref, qdd_ref)',
                'tau_fb': 'Kp*e_q + Ki*integral(e_q) + Kd*(qd_ref-qd)',
            }, indent=2) + '\n')
            print(f'Test {JOINTS[selected]}: đi thêm {degrees:g} độ, '
                  f'đi/về {move:g}s, giữ {hold:g}s; tau_ff + tau_fb.')
            # A failed/timed-out activation may still have changed the manager;
            # mark the cleanup obligation before requesting the switch.
            active_effort = True
            switch(True)
            self.pid.set_parameters([Parameter('autostart', value=True)])
            deadline = time.monotonic() + 5 * (2 * move + 2 * hold) + 15.0
            while self.pid.phase not in ('HOLD', 'ABORT'):
                if time.monotonic() >= deadline:
                    raise RuntimeError('Bài thử chưa hoàn thành trong thời gian chờ')
                executor.spin_once(timeout_sec=0.01)
            if self.pid.phase == 'ABORT':
                raise RuntimeError('PID ABORT; xem CSV và lỗi trong log')
            stop_hold = self.pid.phase_elapsed + hold
            while self.pid.phase_elapsed < stop_hold:
                if self.pid.phase == 'ABORT' or time.monotonic() >= deadline:
                    raise RuntimeError('Bài thử dừng trong pha HOLD_HOME')
                executor.spin_once(timeout_sec=0.01)
        finally:
            try:
                if active_effort:
                    switch(False)
                    print('Đã trả về position; hai khớp cố định giữ bằng motor.')
            finally:
                if self.pid is not None:
                    self.pid.dump()
                    executor.remove_node(self.pid)
                    self.pid.destroy_node()
                executor.remove_node(probe)
                executor.shutdown()
                probe.destroy_node()
                rclpy.shutdown()
                if path is not None and path.is_file():
                    print(f'CSV và URDF của lượt thử: {path.parent.resolve()}')
                    log = load_log(path)
                    print_metrics(log)
                    plot(log, str(path.with_suffix('.png')), self.show_plot, 30000)


class PositionPidTerminal:
    def __init__(self, params_file, joint='elbow', log_dir='results/position_pid_trials',
                 show_plot=True, dry_run=False, interface='can0'):
        self.params_file = Path(params_file).expanduser().resolve()
        self.config = load_config(self.params_file)
        self.selected = JOINTS.index(joint)
        self.dry_run = dry_run
        self.interface = interface
        self.trial = RosPositionTrial(log_dir, show_plot)

    @property
    def parameters(self):
        return self.config['cascade_pid_controller']['ros__parameters']

    def show(self):
        gains = {name: self.parameters[key][self.selected] for name, key in GAIN_KEYS.items()}
        print(f'Khâu {self.selected + 1}: {JOINTS[self.selected]}, '
              f'Kp={gains["kp"]:g}, Ki={gains["ki"]:g}, Kd={gains["kd"]:g}')
        print(f'YAML lưu bằng lệnh save: {self.params_file}')

    def stage(self):
        self.show()
        print('Launch phần cứng cho khớp này (đưa tay về đúng zero trước startup):')
        print('ros2 launch gim_control origin_gim_arm_control.launch.py '
              f'can_interface:={self.interface} set_zero_on_startup:=true '
              f'zero_method:=software torque_joint:={JOINTS[self.selected]}')
        print('Giữ launch chạy khi tune cùng khớp; đổi khớp mới khởi động lại launch.')

    def execute(self, line):
        words = shlex.split(line)
        if not words:
            return True
        command, *args = words
        if command in ('quit', 'exit') and not args:
            return False
        if command in ('help', 'show') and not args:
            print(HELP) if command == 'help' else self.show()
        elif command == 'set' and len(args) == 3:
            values = [checked_gain(value) for value in args]
            for key, value in zip(GAIN_KEYS.values(), values):
                self.parameters[key][self.selected] = value
            self.show()
        elif command in GAIN_KEYS and len(args) == 1:
            self.parameters[GAIN_KEYS[command]][self.selected] = checked_gain(args[0])
            self.show()
        elif command == 'next' and not args:
            if self.selected == 0:
                print('Đã tới khâu 1; thứ tự 3 -> 2 -> 1 hoàn tất.')
            else:
                self.selected -= 1
                self.stage()
        elif command == 'save' and not args:
            if self.dry_run:
                print(f'DRY RUN: save {self.params_file}; không ghi YAML.')
                return True
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(
                        mode='w', dir=self.params_file.parent, delete=False) as stream:
                    temporary = stream.name
                    yaml.safe_dump(self.config, stream, sort_keys=False)
                os.replace(temporary, self.params_file)
            finally:
                if temporary is not None and os.path.exists(temporary):
                    os.unlink(temporary)
            print(f'Đã lưu gain PID trên PC: {self.params_file}')
        elif command == 'test' and 1 <= len(args) <= 3:
            degrees, move, hold = checked_trial(
                args[0], args[1] if len(args) >= 2 else 6, args[2] if len(args) >= 3 else 3)
            if self.dry_run:
                print(f'DRY RUN: khâu {self.selected + 1}, {degrees:g} độ, '
                      f'MOVE={move:g}, HOLD={hold:g}; không mở ROS/CAN.')
            else:
                self.trial.run(self.selected, degrees, move, hold, copy.deepcopy(self.parameters))
        else:
            raise ValueError('Sai lệnh; dùng kp/ki/kd hoặc set KP KI KD. Gõ help để xem.')
        return True

    def run(self):
        print(HELP)
        self.stage()
        while True:
            try:
                line = input(f'pid[khâu {self.selected + 1}/{JOINTS[self.selected]}]> ')
            except (EOFError, KeyboardInterrupt):
                print('\nThoát terminal; gain chỉ lưu khi đã gõ save.')
                return
            try:
                if not self.execute(line):
                    return
            except KeyboardInterrupt:
                print('\nĐã ngắt bài thử; kiểm tra thông báo trả về position ở trên.')
            except (ValueError, OSError, RuntimeError) as error:
                print(f'Lỗi: {error}')


def main(argv=None):
    try:
        import readline
        readline.set_history_length(200)
    except ImportError:
        pass
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params-file', type=Path)
    parser.add_argument('--joint', choices=JOINTS, default='elbow')
    parser.add_argument('--interface', '--can', default='can0', help='Tên CAN cho lệnh launch')
    parser.add_argument('--log-dir', default='results/position_pid_trials')
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    try:
        terminal = PositionPidTerminal(
            args.params_file or default_params_file(), args.joint, args.log_dir,
            not args.no_show, args.dry_run, args.interface)
        terminal.run()
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f'Lỗi cấu hình: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
