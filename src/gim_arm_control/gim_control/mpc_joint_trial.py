"""Selected-joint MPC trials with runtime URDF and continuous torque handoff.

The handoff/state guards follow the position PID terminal on pid-test.
Only explicit run() activates effort; importing this module never uses ROS/CAN.
"""

import copy
from datetime import datetime
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
from zoneinfo import ZoneInfo

import yaml

JOINTS = ('base', 'shoulder', 'elbow')
JOINT_NAMES = tuple(name + '_joint' for name in JOINTS)
START_VELOCITY_LIMIT_RAD_S = 0.05
PRESTART_STABLE_TIME_S = 0.3


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


class RosMpcJointTrial:
    """Run the existing effort MPC runner, with only one hardware torque joint."""

    def __init__(self, log_dir, show_plot=True, prestart_timeout=6.0):
        self.log_dir = Path(log_dir)
        self.show_plot = show_plot
        self.prestart_timeout = float(prestart_timeout)
        if not math.isfinite(self.prestart_timeout) or self.prestart_timeout <= 0:
            raise ValueError('prestart_timeout phải hữu hạn và > 0')
        self.node = None

    def wait_for_stable_hold(self, executor, handoff_duration=0.0):
        """Keep the original HOME under MPC before starting the trajectory."""
        node = self.node
        handoff_end = time.monotonic() + handoff_duration
        deadline = handoff_end + self.prestart_timeout
        stable_since = None
        previous_seq = node.state_seq
        last_fresh = time.monotonic()
        position_limit = min(0.02, node.max_transition_error)
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            if node.phase == 'ABORT':
                raise RuntimeError('MPC ABORT khi giữ HOME trước quỹ đạo; xem CSV')
            if node.state_seq == previous_seq:
                if time.monotonic() - last_fresh > 0.05:
                    stable_since = None
                continue
            now = time.monotonic()
            if now - last_fresh > 0.05:
                stable_since = None
            last_fresh = now
            previous_seq = node.state_seq
            error = max(abs(float(q - home)) for q, home in zip(node.q, node.home_q))
            if error > node.max_transition_error:
                raise RuntimeError(
                    f'Tay trôi khỏi HOME {math.degrees(error):.2f} độ khi chuyển sang torque; '
                    'trả về position, chưa chạy quỹ đạo')
            if now < handoff_end:
                stable_since = None
                continue
            still = max(abs(float(value)) for value in node.qd) <= START_VELOCITY_LIMIT_RAD_S
            if still and error <= position_limit:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= PRESTART_STABLE_TIME_S:
                    return
            else:
                stable_since = None
        values = ', '.join(
            f'{name}: qd={float(velocity):.3f} rad/s, '
            f'e_HOME={math.degrees(float(q - home)):.2f} độ'
            for name, velocity, q, home in zip(JOINTS, node.qd, node.q, node.home_q))
        raise RuntimeError(
            f'MPC chưa giữ HOME ổn định sau {self.prestart_timeout:g}s; {values}. '
            'Dừng bài thử và trả về position, chưa chạy quỹ đạo')

    def run(self, selected, degrees, move, hold, parameters):
        import rclpy
        from controller_manager_msgs.srv import ListControllers, SwitchController
        from rcl_interfaces.srv import GetParameters
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from gim_arm_controller_mpc.factory import MpcFactory
        from gim_control.effort_controller_node import EffortControllerNode
        from gim_control.plot_ee_error import load_log, plot, print_metrics
        from gim_control.reference_trajectory import TimedHold

        self.node = None
        if rclpy.ok():
            raise RuntimeError('Terminal cần một ROS context riêng, không chạy trong node khác')
        rclpy.init(args=[])
        executor = SingleThreadedExecutor()
        probe = rclpy.create_node('mpc_joint_trial')
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
            handoff_param = ET.fromstring(description).find(
                "ros2_control/hardware/param[@name='torque_handoff_duration_s']")
            handoff_duration = float(handoff_param.text) if handoff_param is not None else 1.0
            if not math.isfinite(handoff_duration) or not 0 < handoff_duration <= 5.0:
                raise RuntimeError('torque_handoff_duration_s phải trong (0, 5] giây')
            if probe.count_publishers('/gim_arm_effort_controller/commands'):
                raise RuntimeError('Một node khác đang phát effort; dừng node MPC cũ trước test')
            deadline = time.monotonic() + 3.0
            while not state or time.monotonic() - state['received'] > 0.25:
                if time.monotonic() >= deadline:
                    raise RuntimeError('Chưa có joint state mới với đủ góc/vận tốc')
                executor.spin_once(timeout_sec=0.01)
            if max(abs(value) for value in state['qd']) > 0.1:
                raise RuntimeError('Tay đang chuyển động; giữ yên trước test')

            stamp = datetime.now(ZoneInfo('Asia/Ho_Chi_Minh')).strftime('%Y%m%d_%H%M%S_%f')
            directory = self.log_dir / f'mpc_{JOINTS[selected]}_{stamp}'
            directory.mkdir(parents=True)
            path = directory / 'tracking.csv'
            urdf = directory / 'model.urdf'
            urdf.write_text(description)
            config = copy.deepcopy(parameters)
            config.update(
                urdf_file=str(urdf.resolve()), log_file=str(path.resolve()),
                command_topic='/gim_arm_effort_controller/commands',
                use_sim_time=False, autostart=False, cascade_hold=False, algorithm_hold=True,
                gravity_scale=[1.0, 1.0, 1.0],
                diagnostic_hold=True, diagnostic_segment=False,
                approach_time=move, return_time=move, diagnostic_hold_time=hold,
                start_velocity_limit_rad_s=START_VELOCITY_LIMIT_RAD_S,
                command_heartbeat_nm=1.0e-6)
            # Construct with a hold at the measured pose; update the target
            # after WAIT captures the exact HOME from a fresh state sample.
            for index in range(3):
                config[f'diagnostic_q{index + 1}_deg'] = math.degrees(state['q'][index])
            self.node = EffortControllerNode(
                MpcFactory(),
                parameter_overrides=[Parameter(name, value=value) for name, value in config.items()])
            executor.add_node(self.node)
            deadline = time.monotonic() + 180.0
            while self.node.phase == 'WAIT':
                if time.monotonic() >= deadline:
                    raise RuntimeError('MPC chưa nhận được joint states')
                executor.spin_once(timeout_sec=0.01)
            target = trial_target(
                self.node.home_q, selected, degrees, move,
                self.node.dynamics.q_min, self.node.dynamics.q_max, self.node.dynamics.vel_max)
            self.node.trajectory = TimedHold(target, hold)
            for index, value in enumerate(target):
                config[f'diagnostic_q{index + 1}_deg'] = math.degrees(value)
            self.node.set_parameters([
                Parameter(f'diagnostic_q{index + 1}_deg', value=math.degrees(value))
                for index, value in enumerate(target)])
            # WAIT initially prepared a hold at the measured pose. Rebuild
            # prediction matrices/bounds for the target before effort activation.
            self.node._prepare_controller_schedule(self.node.home_q)
            self.node.controller.reset()
            # Precomputation blocks callbacks: require a fresh state afterward.
            self.node.q = self.node.qd = self.node.state_stamp = None
            self.node.previous_tick = None
            deadline = time.monotonic() + 3.0
            while self.node.q is None:
                if time.monotonic() >= deadline:
                    raise RuntimeError('Không có state mới sau precompute MPC')
                executor.spin_once(timeout_sec=0.01)
            (directory / 'parameters.yaml').write_text(yaml.safe_dump({
                'mpc_controller': {'ros__parameters': config}}, sort_keys=False))
            (directory / 'trial.json').write_text(json.dumps({
                'joint': JOINTS[selected], 'home_rad': self.node.home_q.tolist(),
                'target_rad': target, 'move_s': move, 'hold_s': hold,
                'tau_ff': 'inverse_dynamics(runtime URDF, q_ref, qd_ref, qdd_ref); '
                          'base component multiplied by base_ff_alpha',
                'base_ff_alpha': self.node.controller.base_ff_alpha,
                'tau_fb': 'first input of constrained MPC QP for [integral(e), e, qd-qd_ref]',
                'prestart_hold': {
                    'hardware_handoff_duration_s': handoff_duration,
                    'stable_time_s': PRESTART_STABLE_TIME_S,
                    'timeout_s': self.prestart_timeout,
                    'velocity_limit_rad_s': START_VELOCITY_LIMIT_RAD_S,
                    'position_limit_rad': min(0.02, self.node.max_transition_error),
                },
            }, indent=2) + '\n')
            print(f'Test {JOINTS[selected]}: đi thêm {degrees:g} độ, '
                  f'đi/về {move:g}s, giữ {hold:g}s; tau_ff + tau_fb.')
            # Establish the command connection while position mode still holds
            # the arm. Send the current HOME MPC + gravity command before the
            # mode handoff instead of enabling the trajectory immediately.
            deadline = time.monotonic() + 3.0
            while self.node.publisher.get_subscription_count() == 0:
                if time.monotonic() >= deadline:
                    raise RuntimeError('Effort controller chưa nhận topic commands')
                executor.spin_once(timeout_sec=0.01)
            prime_until = time.monotonic() + 0.05
            while time.monotonic() < prime_until:
                executor.spin_once(timeout_sec=0.01)
            if self.node.phase == 'ABORT':
                raise RuntimeError('MPC ABORT trước khi chuyển sang torque; xem CSV')
            self.node._publish(self.node._hold_torque(self.node.dt_nom, integrate=False))
            # A failed/timed-out activation may still have changed the manager;
            # mark the cleanup obligation before requesting the switch.
            active_effort = True
            switch(True)
            # Position mode is now inactive: allow this same MPC's I term to
            # correct gravity/model mismatch while HOME stays fixed. Nothing
            # accumulated while the driver was holding position.
            self.node.manual_hold_latched = True
            print(f'Bàn giao torque trong {handoff_duration:g}s, rồi giữ HOME '
                  'bằng tau_ff + MPC; chờ tay ổn định 0.3s trước quỹ đạo.')
            self.wait_for_stable_hold(executor, handoff_duration)
            print('HOME đã ổn định; bắt đầu quỹ đạo.')
            self.node.set_parameters([Parameter('autostart', value=True)])
            deadline = time.monotonic() + 5 * (2 * move + 2 * hold) + 15.0
            while self.node.phase not in ('HOLD', 'ABORT'):
                if time.monotonic() >= deadline:
                    raise RuntimeError('Bài thử chưa hoàn thành trong thời gian chờ')
                executor.spin_once(timeout_sec=0.01)
            if self.node.phase == 'ABORT':
                raise RuntimeError('MPC ABORT; xem CSV và lỗi trong log')
            stop_hold = self.node.phase_elapsed + hold
            while self.node.phase_elapsed < stop_hold:
                if self.node.phase == 'ABORT' or time.monotonic() >= deadline:
                    raise RuntimeError('Bài thử dừng trong pha HOLD_HOME')
                executor.spin_once(timeout_sec=0.01)
        finally:
            try:
                if active_effort:
                    from types import SimpleNamespace
                    from gim_control.algorithm_tuner import restore_position
                    restore_position(SimpleNamespace(
                        executor=executor,
                        states=lambda: {item.name: item.state for item in call(
                            listing, ListControllers.Request()).controller},
                        switch=lambda activate, deactivate: switch(False)))
                    print('Đã trả về position; hai khớp cố định giữ bằng motor.')
            finally:
                if self.node is not None:
                    self.node.dump()
                    executor.remove_node(self.node)
                    self.node.destroy_node()
                executor.remove_node(probe)
                executor.shutdown()
                probe.destroy_node()
                rclpy.shutdown()
                if path is not None and path.is_file():
                    print(f'CSV và URDF của lượt thử: {path.parent.resolve()}')
                    log = load_log(path)
                    print_metrics(log)
                    plot(log, str(path.with_suffix('.png')), self.show_plot, 30000)
