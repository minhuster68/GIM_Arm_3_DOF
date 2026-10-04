#!/usr/bin/env python3
"""Interactive GIM6010 driver gain tuning through Linux SocketCAN.

Command IDs come from SteadyWin GIM6010-8 manual rev1.3, section 4.1.2.
Little-endian float32 fields match ODrive CANSimple fw-v0.5.4 callbacks:
https://github.com/odriverobotics/ODrive/blob/fw-v0.5.4/Firmware/communication/can/can_simple.cpp
These are motor driver gains, independent of the PC cascade PID gains.
"""

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import shlex
import socket
import struct
import sys
import time
import xml.etree.ElementTree as ET


JOINT_NODES = {'base': 0, 'shoulder': 1, 'elbow': 2}
SET_POS_GAIN = 0x1A
SET_VEL_GAINS = 0x1B
SAVE_CONFIGURATION = 0x1F
CAN_FRAME = struct.Struct('=IB3x8s')
JOINT_NAMES = tuple(name + '_joint' for name in JOINT_NODES)
HELP = """Lệnh terminal (gain bên trong driver, không phải PID trên PC):
  show                  Xem các gain đã gửi trong phiên này
  set KPP KVP KVI       Gửi cả ba gain, ví dụ: set 20 0.16 0
  kpp VALUE             Chỉnh pos_gain
  vel KVP KVI           Gửi cặp vel_gain / vel_integrator_gain
  kvp VALUE             Chỉnh vel_gain, giữ kvi đã gửi trước đó
  kvi VALUE             Chỉnh vel_integrator_gain, giữ kvp đã gửi trước đó
  test DEG [MOVE HOLD]  Đi thêm DEG độ, giữ, về và giữ lại; mặc định 6 / 3 s
  joint base|shoulder|elbow   Chọn khớp (CAN node 0 / 1 / 2)
  node ID               Chọn CAN node 0..63
  save                  Gửi Save_Configuration cho driver đang chọn
  help                  Hiện hướng dẫn
  quit                  Thoát; giữ nguyên gain đã gửi

Ban đầu gain chưa biết. Dùng 'vel KVP KVI' trước khi chỉnh riêng kvp/kvi.
'show' là lịch sử gửi, không phải đọc lại gain từ motor; CAN không có gain ACK.
'save' ghi cấu hình driver vào flash; chương trình không tự save.
Chỉ lệnh test gửi quỹ đạo qua ROS position controller; không tự đổi mode/zero.
Ctrl-C trong bài thử gửi hủy goal. Gain CAN độc lập với quỹ đạo ROS.
Sau mỗi test có dữ liệu, đồ thị được lưu PNG và tự mở; đóng cửa sổ để tune tiếp.
"""


def load_joint_limits(path=None):
    if path is None:
        source = Path(__file__).resolve().parents[2] / 'gim_arm_description/urdf/gim_arm.urdf'
        if source.is_file():
            path = source
        else:
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory('gim_arm_description')) / 'urdf/gim_arm.urdf'
    root = ET.parse(path).getroot()
    limits = []
    for name in JOINT_NAMES:
        limit = root.find(f"joint[@name='{name}']/limit")
        if limit is None:
            raise ValueError(f'URDF thiếu giới hạn {name}')
        limits.append((float(limit.get('lower')), float(limit.get('upper')),
                       float(limit.get('velocity'))))
    return limits


def trial_plan(start, selected, degrees, move, hold, limits):
    """Build an out/hold/back/hold trial, leaving other references unchanged."""
    if selected not in range(3) or len(start) != 3 or len(limits) != 3:
        raise ValueError('Bài thử cần đúng ba khớp base / shoulder / elbow')
    if not all(math.isfinite(value) for value in (*start, degrees, move, hold)):
        raise ValueError('Góc và thời gian phải hữu hạn')
    if move <= 0 or hold <= 0:
        raise ValueError('MOVE và HOLD phải > 0')
    if 2 * (move + hold) >= 2**31 or min(round(move * 1e9), round(hold * 1e9)) < 1:
        raise ValueError('Thời gian phải biểu diễn được bằng ROS Duration')
    target = list(start)
    target[selected] += math.radians(degrees)
    for pose in (start, target):
        for name, value, limit in zip(JOINT_NAMES, pose, limits):
            lower, upper = limit[:2]
            if not lower + 0.05 <= value <= upper - 0.05:
                raise ValueError(f'{name}: {math.degrees(value):.3f} độ gần/vượt giới hạn URDF')
    if len(limits[selected]) >= 3:
        peak_velocity = 1.875 * abs(math.radians(degrees)) / move
        if peak_velocity > limits[selected][2]:
            raise ValueError('Quỹ đạo vượt giới hạn vận tốc URDF; tăng MOVE')
    return [(0.0, list(start)), (move, target), (move + hold, target),
            (2 * move + hold, list(start)), (2 * move + 2 * hold, list(start))]


def plot_trial(rows, selected, gains, csv_path, move, hold, show=True):
    """Save one figure per trial and show it after the motion has finished."""
    if not rows:
        print('Chưa có mẫu feedback nên không vẽ đồ thị cho lượt thử này.')
        return None
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    times = [row[0] for row in rows]
    actual = [math.degrees(row[1 + selected]) for row in rows]
    desired = [math.degrees(row[4 + selected]) for row in rows]
    errors = [q - qref for q, qref in zip(actual, desired)]
    rms = math.sqrt(sum(error**2 for error in errors) / len(errors))
    maximum = max(abs(error) for error in errors)
    gain_label = ', '.join(f'{name}={value:.6g}' if value is not None else f'{name}=?'
                           for name, value in gains.items())
    path = Path(csv_path).with_suffix('.png')
    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, constrained_layout=True)
    try:
        figure.suptitle(f'{JOINT_NAMES[selected]} — {gain_label}\n{Path(csv_path).stem}')
        manager = figure.canvas.manager
        if manager is not None:
            manager.set_window_title(f'Tune {JOINT_NAMES[selected]} — {Path(csv_path).stem}')
        axes[0].plot(times, desired, '--', color='tab:red', label='Góc đặt')
        axes[0].plot(times, actual, color='tab:blue', label='Góc thực tế')
        axes[0].set_ylabel('Góc (độ)')
        axes[0].set_title('Đáp ứng khớp đang tune')
        axes[0].legend()
        axes[1].plot(times, errors, color='tab:orange',
                     label=f'RMS={rms:.3f}°, MAX={maximum:.3f}°')
        axes[1].axhline(0, color='black', linewidth=0.8)
        axes[1].set_ylabel('Sai số (độ)')
        axes[1].set_title('Góc thực tế − góc đặt')
        axes[1].legend()
        for index, name in enumerate(JOINT_NAMES):
            if index != selected:
                drift = [math.degrees(row[1 + index] - row[4 + index]) for row in rows]
                axes[2].plot(times, drift, label=name)
        axes[2].axhline(0, color='black', linewidth=0.8)
        axes[2].set_ylabel('Sai lệch giữ (độ)')
        axes[2].set_title('Hai khớp đang giữ vị trí')
        axes[2].legend()
        axes[2].set_xlabel('Thời gian từ lúc gửi goal (s)')
        for axis in axes:
            axis.grid(alpha=0.3)
            for boundary in (move, move + hold, 2 * move + hold):
                if times[0] <= boundary <= times[-1]:
                    axis.axvline(boundary, linestyle=':', color='0.5', linewidth=0.8)
        figure.savefig(path, dpi=150)
        print(f'Đã lưu đồ thị: {path.resolve()}')
        if show:
            static_backends = ('agg', 'pdf', 'ps', 'svg', 'cairo', 'template')
            if matplotlib.get_backend().lower() in static_backends:
                print('Môi trường này không mở được cửa sổ đồ thị; mở file PNG để xem.')
            else:
                print('Đóng cửa sổ đồ thị để trở về terminal chỉnh gain.')
                plt.show(block=True)
    finally:
        plt.close(figure)
    return path


class RosJointTrial:
    """Lazy ROS action client; never sends direct CAN position setpoints."""

    def __init__(self, controller, log_dir, urdf=None, show_plot=True):
        try:
            import rclpy
            from controller_manager_msgs.srv import ListControllers
            from control_msgs.action import FollowJointTrajectory
            from rclpy.action import ActionClient
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import JointState
        except ImportError as error:
            raise RuntimeError('Lệnh test cần source ROS Humble và install/setup.bash') from error
        self.ros = rclpy
        self.context = rclpy.context.Context()
        self.ros.init(args=[], context=self.context)
        self.node = self.ros.create_node('motor_gain_trial', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.controller = controller.strip('/')
        self.client = ActionClient(self.node, FollowJointTrajectory,
                                   f'/{self.controller}/follow_joint_trajectory')
        self.controllers = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.controller_request = ListControllers.Request
        self.goal_type = FollowJointTrajectory.Goal
        self.subscription = self.node.create_subscription(
            JointState, '/joint_states', self.on_state, qos_profile_sensor_data)
        self.log_dir = Path(log_dir)
        self.urdf = urdf
        self.show_plot = show_plot
        self.start = None
        self.rows = []
        self.started = 0.0

    def close(self):
        self.client.destroy()
        self.executor.shutdown()
        self.node.destroy_node()
        self.ros.shutdown(context=self.context)

    def wait(self, future, timeout):
        self.executor.spin_until_future_complete(future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('Hết thời gian chờ phản hồi ROS')
        return future.result()

    def on_state(self, message):
        try:
            start = [float(message.position[message.name.index(name)]) for name in JOINT_NAMES]
        except (ValueError, IndexError):
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        now = self.node.get_clock().now().nanoseconds * 1e-9
        if stamp > 0 and (now - stamp > 0.5 or stamp - now > 0.5):
            return
        if all(math.isfinite(value) for value in start):
            self.start = start

    def on_feedback(self, message):
        feedback = message.feedback
        try:
            indices = [feedback.joint_names.index(name) for name in JOINT_NAMES]
            actual = [feedback.actual.positions[i] for i in indices]
            desired = [feedback.desired.positions[i] for i in indices]
        except (ValueError, IndexError):
            return
        if all(math.isfinite(value) for value in actual + desired):
            self.rows.append([time.monotonic() - self.started, *actual, *desired])

    def run(self, selected, degrees, move, hold, gains):
        from action_msgs.msg import GoalStatus
        from trajectory_msgs.msg import JointTrajectoryPoint

        if not self.controllers.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('Chưa có controller_manager; hãy chạy launch phần cứng')
        response = self.wait(self.controllers.call_async(self.controller_request()), 3.0)
        if not any(c.name == self.controller and c.state == 'active' for c in response.controller):
            raise RuntimeError(f'{self.controller} chưa active; chuyển về position trước khi test')
        self.start = None
        deadline = time.monotonic() + 3.0
        while self.start is None and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.1)
        if self.start is None:
            raise RuntimeError('Chưa nhận được /joint_states mới và đủ ba khớp')
        plan = trial_plan(self.start, selected, degrees, move, hold,
                          load_joint_limits(self.urdf))
        if not self.client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError('Không kết nối được action FollowJointTrajectory')
        # Ensure log creation succeeds before asking the robot to move.
        self.log_dir.mkdir(parents=True, exist_ok=True)
        path = self.log_dir / ('motor_' + JOINT_NAMES[selected] + '_'
                               + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.csv')
        with path.open('w', newline='') as stream:
            stream.write('')
        goal = self.goal_type()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        for seconds, pose in plan:
            point = JointTrajectoryPoint()
            point.positions = pose
            point.velocities = [0.0] * 3
            point.accelerations = [0.0] * 3
            ns = round(seconds * 1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
            goal.trajectory.points.append(point)
        target = plan[1][1]
        print(f'Test {JOINT_NAMES[selected]}: {math.degrees(self.start[selected]):.3f} '
              f'-> {math.degrees(target[selected]):.3f} độ -> về điểm đầu.')
        print('Hai khớp còn lại giữ góc ban đầu. Ctrl-C để gửi hủy goal.')
        self.rows = []
        self.started = time.monotonic()
        handle = None
        pending = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        try:
            handle = self.wait(pending, 5.0)
            if not handle.accepted:
                raise RuntimeError('Controller từ chối goal thử')
            result = self.wait(handle.get_result_async(), plan[-1][0] + 10.0)
            print(f'Kết quả: status={result.status}, error_code={result.result.error_code}, '
                  f'{result.result.error_string}')
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
                raise RuntimeError('Bài thử chưa thành công; xem kết quả controller ở trên')
        except (KeyboardInterrupt, RuntimeError):
            if handle is None:
                # Resolve a pending acceptance before cancelling the same goal.
                try:
                    handle = self.wait(pending, 3.0)
                except RuntimeError:
                    print('Chưa nhận được goal handle; không xác minh được việc hủy goal.')
            if handle is not None and handle.accepted:
                try:
                    cancelled = self.wait(handle.cancel_goal_async(), 3.0)
                    print('Đã nhận hủy goal.' if cancelled.goals_canceling
                          else 'Controller không xác nhận hủy goal; kiểm tra trạng thái tay.')
                except RuntimeError:
                    print('Không nhận được phản hồi hủy goal; kiểm tra trạng thái tay.')
            raise
        finally:
            header = ['t_s'] + ['q_' + name for name in JOINT_NAMES]
            header += ['qref_' + name for name in JOINT_NAMES] + ['kpp', 'kvp', 'kvi']
            with path.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(header)
                for row in self.rows:
                    writer.writerow([*row, gains['kpp'], gains['kvp'], gains['kvi']])
            print(f'CSV góc thực tế / góc đặt: {path.resolve()} ({len(self.rows)} mẫu)')
            if self.rows:
                errors = [math.degrees(row[1 + selected] - row[4 + selected])
                          for row in self.rows]
                rms = math.sqrt(sum(error**2 for error in errors) / len(errors))
                print(f'Sai số bám {JOINT_NAMES[selected]}: RMS={rms:.3f} độ, '
                      f'MAX={max(abs(error) for error in errors):.3f} độ')
            try:
                plot_trial(self.rows, selected, gains, path, move, hold, show=self.show_plot)
            except KeyboardInterrupt:
                print('Đã ngắt xem đồ thị; CSV đã được lưu.')
            except Exception as error:
                print(f'Không vẽ được đồ thị: {error}. CSV đã được lưu tại {path.resolve()}')


def checked_node(value):
    """Reject node IDs that cannot fit in CANSimple's six-bit node field."""
    node = int(value)
    if not 0 <= node <= 63:
        raise ValueError('CAN node phải nằm trong 0..63')
    return node


def checked_gain(value):
    """Validate and return the actual float32 value that will be sent."""
    gain = float(value)
    if not math.isfinite(gain) or gain < 0:
        raise ValueError('Gain phải hữu hạn và >= 0')
    try:
        encoded = struct.pack('<f', gain)
    except (OverflowError, struct.error) as error:
        raise ValueError('Gain vượt phạm vi float32') from error
    rounded = struct.unpack('<f', encoded)[0]
    if not math.isfinite(rounded) or (gain > 0 and rounded == 0):
        raise ValueError('Gain vượt phạm vi float32')
    return rounded


def make_frame(node, command, payload=b''):
    """Pack a classic standard-ID Linux can_frame, preserving CAN DLC."""
    node = checked_node(node)
    if len(payload) > 8:
        raise ValueError('Classic CAN chỉ hỗ trợ tối đa 8 byte')
    arbitration_id = (node << 5) | command
    return CAN_FRAME.pack(arbitration_id, len(payload), payload.ljust(8, b'\x00'))


class SocketCanSender:
    def __init__(self, interface):
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.socket.settimeout(1.0)
            # This tool only transmits; telemetry continues to other CAN sockets.
            self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, b'')
            self.socket.bind((interface,))
        except OSError:
            self.socket.close()
            raise

    def send(self, node, command, payload):
        frame = make_frame(node, command, payload)
        if self.socket.send(frame) != len(frame):
            raise OSError('Không gửi được toàn bộ CAN frame')

    def close(self):
        self.socket.close()


class DryRunSender:
    def send(self, node, command, payload):
        make_frame(node, command, payload)
        print(f'DRY RUN: CAN ID=0x{(node << 5) | command:03X} '
              f'DLC={len(payload)} DATA={payload.hex(" ")}')

    def close(self):
        pass


class GainTerminal:
    def __init__(self, sender, node, controller='gim_arm_group_controller',
                 log_dir='results/motor_gain_trials', urdf=None, show_plot=True):
        self.sender = sender
        self.node = checked_node(node)
        self.history = {}
        self.trial = None
        self.controller = controller
        self.log_dir = log_dir
        self.urdf = urdf
        self.show_plot = show_plot

    def close(self):
        if self.trial is not None:
            self.trial.close()

    def test(self, args):
        if self.node not in JOINT_NODES.values():
            raise ValueError('test chỉ hỗ trợ CAN node 0 / 1 / 2 của tay hiện tại')
        degrees = float(args[0])
        move = float(args[1]) if len(args) >= 2 else 6.0
        hold = float(args[2]) if len(args) >= 3 else 3.0
        # Validate before loading ROS or contacting any controller.
        trial_plan([0.0] * 3, self.node, 0.0, move, hold,
                   [(-math.inf, math.inf)] * 3)
        if not math.isfinite(degrees):
            raise ValueError('Góc thử phải hữu hạn')
        if isinstance(self.sender, DryRunSender):
            print(f'DRY RUN test: {JOINT_NAMES[self.node]}, delta={degrees:g} độ, '
                  f'move={move:g}s, hold={hold:g}s; không gửi ROS goal.')
            return
        if self.trial is None:
            self.trial = RosJointTrial(self.controller, self.log_dir, self.urdf, self.show_plot)
        try:
            self.trial.run(self.node, degrees, move, hold, dict(self.gains))
        except KeyboardInterrupt:
            print('Đã ngắt bài thử; trở về terminal chỉnh gain.')

    @property
    def gains(self):
        return self.history.setdefault(self.node, dict(kpp=None, kvp=None, kvi=None))

    def show(self):
        names = [joint for joint, node in JOINT_NODES.items() if node == self.node]
        label = names[0] if names else 'custom'
        values = ', '.join(f'{name}={value:.7g}' if value is not None else f'{name}=?'
                           for name, value in self.gains.items())
        print(f'CAN node {self.node} ({label}): {values}')
        print('Giá trị đã gửi trong phiên này; chưa có đọc lại/ACK từ driver.')

    def set_position(self, gain):
        self.sender.send(self.node, SET_POS_GAIN, struct.pack('<f', gain))
        self.gains['kpp'] = gain
        print(f'Đã gửi node {self.node}: kpp={gain:.7g} (pos_gain)')

    def set_velocity(self, kvp, kvi):
        self.sender.send(self.node, SET_VEL_GAINS, struct.pack('<ff', kvp, kvi))
        self.gains.update(kvp=kvp, kvi=kvi)
        print(f'Đã gửi node {self.node}: kvp={kvp:.7g}, kvi={kvi:.7g}')

    def execute(self, line):
        """Process one user command; return False only for quit/exit."""
        parts = shlex.split(line)
        if not parts:
            return True
        command, args = parts[0].lower(), parts[1:]
        if command in ('quit', 'exit') and not args:
            return False
        if command == 'help' and not args:
            print(HELP)
        elif command == 'show' and not args:
            self.show()
        elif command == 'joint' and len(args) == 1:
            if args[0] not in JOINT_NODES:
                raise ValueError('joint phải là base, shoulder hoặc elbow')
            self.node = JOINT_NODES[args[0]]
            self.show()
        elif command == 'node' and len(args) == 1:
            self.node = checked_node(args[0])
            self.show()
        elif command == 'set' and len(args) == 3:
            # Validate all values before sending either of the two frames.
            kpp, kvp, kvi = [checked_gain(value) for value in args]
            self.set_position(kpp)
            self.set_velocity(kvp, kvi)
        elif command == 'kpp' and len(args) == 1:
            self.set_position(checked_gain(args[0]))
        elif command == 'test' and 1 <= len(args) <= 3:
            self.test(args)
        elif command == 'vel' and len(args) == 2:
            kvp, kvi = [checked_gain(value) for value in args]
            self.set_velocity(kvp, kvi)
        elif command in ('kvp', 'kvi') and len(args) == 1:
            value = checked_gain(args[0])
            partner = 'kvi' if command == 'kvp' else 'kvp'
            if self.gains[partner] is None:
                raise ValueError(f'Chưa biết {partner}; dùng vel KVP KVI để gửi cặp ban đầu')
            values = dict(self.gains)
            values[command] = value
            self.set_velocity(values['kvp'], values['kvi'])
        elif command == 'save' and not args:
            self.sender.send(self.node, SAVE_CONFIGURATION, b'')
            print(f'Đã gửi Save_Configuration cho node {self.node}; chưa có ACK lưu flash.')
        else:
            raise ValueError('Sai lệnh hoặc số tham số. Gõ help để xem cách dùng.')
        return True

    def run(self):
        print(HELP)
        self.show()
        while True:
            try:
                line = input(f'gain[node={self.node}]> ')
                if not self.execute(line):
                    return
            except (EOFError, KeyboardInterrupt):
                print('\nThoát; không đổi mode và không tự lưu flash.')
                return
            except (ValueError, OSError, RuntimeError) as error:
                print(f'Lỗi: {error}')


def main(argv=None):
    try:
        import readline
        readline.set_history_length(200)
    except ImportError:
        pass
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', '--can', default='can0', help='SocketCAN interface')
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--joint', choices=tuple(JOINT_NODES), help='Mặc định elbow')
    selection.add_argument('--node', type=checked_node, help='CAN node ID 0..63')
    parser.add_argument('--dry-run', action='store_true', help='Chỉ in frame, không mở CAN')
    parser.add_argument('--controller', default='gim_arm_group_controller',
                        help='Position controller dùng cho lệnh test')
    parser.add_argument('--log-dir', default='results/motor_gain_trials')
    parser.add_argument('--urdf', help='URDF dùng kiểm giới hạn bài thử')
    parser.add_argument('--no-show', action='store_true',
                        help='Vẫn lưu PNG sau test nhưng không mở cửa sổ đồ thị')
    args = parser.parse_args(argv)
    node = args.node if args.node is not None else JOINT_NODES[args.joint or 'elbow']
    try:
        sender = DryRunSender() if args.dry_run else SocketCanSender(args.interface)
    except OSError as error:
        print(f'Không mở được {args.interface}: {error}', file=sys.stderr)
        return 1
    terminal = GainTerminal(sender, node, args.controller, args.log_dir, args.urdf,
                            show_plot=not args.no_show)
    try:
        if args.dry_run:
            print('DRY RUN: không gửi gì tới motor.')
        else:
            print(f'Đã mở {args.interface}. Gain chỉ được gửi khi bạn nhập lệnh.')
        terminal.run()
    finally:
        terminal.close()
        sender.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
