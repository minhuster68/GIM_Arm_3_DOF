#!/usr/bin/env python3
"""Guarded SMC hold or small elbow out-and-back at a position reached in position mode.

This diagnostic requires a hardware launch with handoff_shoulder_sign:=-1.
It restores the position controller in finally.
"""

import argparse
from collections import deque
from datetime import datetime
import math
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
from zoneinfo import ZoneInfo

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/gim_arm_control'))
sys.path.insert(0, str(ROOT / 'src/gim_arm_controller_smc'))
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'


def require_single_hardware_launch():
    launches = []
    managers = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv = (entry / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if any(b'origin_gim_arm_control.launch.py' in arg for arg in argv):
            launches.append(entry.name)
        if any(arg.endswith(b'/ros2_control_node') for arg in argv):
            managers.append(entry.name)
    if len(launches) != 1 or len(managers) != 1:
        raise RuntimeError(
            'Cần đúng một launch phần cứng và một ros2_control_node; '
            f'launch PID={launches}, manager PID={managers}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--q-deg', nargs=3, type=float, default=[25., 35., 40.])
    parser.add_argument('--seconds', type=float, default=2.0)
    parser.add_argument('--elbow-excursion-deg', type=float, default=0.0,
                        help='Đi elbow thêm góc này rồi về tư thế giữa; mặc định chỉ giữ tĩnh')
    parser.add_argument('--move-seconds', type=float, default=4.0,
                        help='Thời gian mỗi chiều của quỹ đạo elbow')
    parser.add_argument('--elbow-gravity-scale', type=float, default=1.0,
                        help='Hệ số thử riêng cho G(q) của elbow; mặc định 1')
    parser.add_argument('--params-file', type=Path, default=ROOT /
                        'src/gim_arm_controller_smc/config/'
                        'smc_circle_from_terminal_tuning.yaml')
    parser.add_argument('--log-dir', type=Path, default=ROOT /
                        'results_new/smc_super_twisting/coupled_diagnostic')
    args = parser.parse_args()
    target = np.radians(args.q_deg)
    if (not np.all(np.isfinite(target)) or not math.isfinite(args.seconds)
            or not 1.2 <= args.seconds <= 3.0):
        parser.error('q-deg phải hữu hạn; seconds phải trong [1.2, 3.0]')
    if (not math.isfinite(args.elbow_gravity_scale)
            or not 0.5 <= args.elbow_gravity_scale <= 1.0):
        parser.error('elbow-gravity-scale phải trong [0.5, 1.0]')
    if (not math.isfinite(args.elbow_excursion_deg)
            or not 0.0 <= args.elbow_excursion_deg <= 10.0
            or not math.isfinite(args.move_seconds)
            or not 3.0 <= args.move_seconds <= 10.0):
        parser.error('elbow-excursion-deg trong [0,10], move-seconds trong [3,10]')
    profile = yaml.safe_load(args.params_file.read_text())['smc_controller']['ros__parameters']
    if (profile.get('smc_variant') != 'super_twisting'
            or profile.get('controlled_joint') != 'all'
            or profile.get('gravity_scale', [1.0, 1.0, 1.0]) != [1.0, 1.0, 1.0]):
        parser.error('Cần profile SMC all với dấu trọng lực URDF gốc')

    import rclpy
    from controller_manager_msgs.srv import ListControllers, SwitchController
    from rcl_interfaces.srv import GetParameters
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from gim_arm_controller_smc.factory import SmcFactory
    from gim_control.effort_controller_node import EffortControllerNode
    from gim_control.reference_trajectory import Quintic
    from gim_control.smc_joint_trial import stable_samples

    class ElbowOutAndBack:
        def __init__(self, middle, excursion_rad, leg_time):
            end = middle.copy()
            end[2] += excursion_rad
            self.outbound = Quintic(middle, end, leg_time)
            self.inbound = Quintic(end, middle, leg_time)
            self.leg_time = leg_time
            self.duration = 2.0 * leg_time

        def at(self, t):
            if t <= self.leg_time:
                return self.outbound.at(t)
            return self.inbound.at(t - self.leg_time)

    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    probe = rclpy.create_node('smc_handoff_hold_probe')
    executor.add_node(probe)
    listing = probe.create_client(ListControllers, '/controller_manager/list_controllers')
    switching = probe.create_client(SwitchController, '/controller_manager/switch_controller')
    description_client = probe.create_client(
        GetParameters, '/controller_manager/get_parameters')
    node = None
    switch_attempted = False
    result = 'chưa bắt đầu'
    path = None

    def call(client, request, timeout=6.0):
        deadline = time.monotonic() + timeout
        while not client.service_is_ready():
            if time.monotonic() >= deadline:
                raise RuntimeError(f'Không có service {client.srv_name}')
            executor.spin_once(timeout_sec=0.005)
        future = client.call_async(request)
        while not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError(f'Hết hạn service {client.srv_name}')
            executor.spin_once(timeout_sec=0.005)
        return future.result()

    def states():
        return {item.name: item.state for item in call(
            listing, ListControllers.Request()).controller}

    def switch(effort):
        wanted = 'gim_arm_effort_controller' if effort else 'gim_arm_group_controller'
        other = 'gim_arm_group_controller' if effort else 'gim_arm_effort_controller'
        current = states()
        if current.get(wanted) == 'active' and current.get(other) == 'inactive':
            return
        request = SwitchController.Request()
        request.activate_controllers = [wanted]
        request.deactivate_controllers = [other]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 5
        if not call(switching, request, timeout=8.0).ok:
            raise RuntimeError('Chuyển controller thất bại')

    try:
        require_single_hardware_launch()
        current = states()
        if (current.get('gim_arm_group_controller') != 'active'
                or current.get('gim_arm_effort_controller') != 'inactive'):
            raise RuntimeError('Cần position active và effort inactive')
        response = call(description_client,
                        GetParameters.Request(names=['robot_description']))
        description = response.values[0].string_value
        control = ET.fromstring(description).find("ros2_control[@name='GimArmSystem']")
        if control is None:
            raise RuntimeError('Runtime URDF thiếu GimArmSystem')
        for name in ('base_joint', 'shoulder_joint', 'elbow_joint'):
            joint = control.find(f"joint[@name='{name}']")
            if joint is None:
                raise RuntimeError(f'Runtime URDF thiếu {name}')
            enabled = joint.find("param[@name='torque_mode_enable']")
            flag = (enabled.text or '').strip().lower() if enabled is not None else 'true'
            if flag in ('false', '0'):
                raise RuntimeError(f'{name} chưa bật torque_mode_enable')
        shoulder = control.find("joint[@name='shoulder_joint']")
        sign = shoulder.find("param[@name='torque_handoff_feedback_sign']")
        if sign is None or (sign.text or '').strip() != '-1':
            raise RuntimeError('Launch phần cứng chưa dùng handoff_shoulder_sign:=-1')
        if probe.count_publishers('/gim_arm_effort_controller/commands'):
            raise RuntimeError('Một node khác đang phát effort')

        stamp = datetime.now(ZoneInfo('Asia/Ho_Chi_Minh')).strftime('%Y%m%d_%H%M%S_%f')
        kind = 'smc_elbow_loop_trial' if args.elbow_excursion_deg else 'smc_handoff_seed_trial'
        directory = args.log_dir / f'{kind}_{stamp}'
        directory.mkdir(parents=True)
        path = directory / 'tracking.csv'
        urdf = directory / 'model.urdf'
        urdf.write_text(description)
        config = dict(profile)
        config.update(
            urdf_file=str(urdf.resolve()), log_file=str(path.resolve()),
            command_topic='/gim_arm_effort_controller/commands',
            use_sim_time=False, autostart=False, algorithm_hold=True,
            gravity_scale=[1.0, 1.0, args.elbow_gravity_scale],
            abort_on_tracking_error=True, max_hold_error_rad=0.025,
            max_track_error_rad=0.035, max_transition_error_rad=0.035,
            approach_time=1.5, return_time=1.5,
            diagnostic_hold=True, diagnostic_segment=False,
            diagnostic_hold_time=2.0,
            diagnostic_q1_deg=float(args.q_deg[0]),
            diagnostic_q2_deg=float(args.q_deg[1]),
            diagnostic_q3_deg=float(args.q_deg[2]),
        )
        (directory / 'parameters.yaml').write_text(yaml.safe_dump({
            'smc_controller': {'ros__parameters': config}}, sort_keys=False))
        node = EffortControllerNode(SmcFactory(), parameter_overrides=[
            Parameter(name, value=value) for name, value in config.items()])
        if args.elbow_excursion_deg:
            end = target.copy()
            end[2] += math.radians(args.elbow_excursion_deg)
            if (np.any(target < node.dynamics.q_min + node.joint_margin)
                    or np.any(end > node.dynamics.q_max - node.joint_margin)):
                raise RuntimeError('Quỹ đạo elbow vượt khoảng cách an toàn tới giới hạn URDF')
            node.trajectory = ElbowOutAndBack(
                target, math.radians(args.elbow_excursion_deg), args.move_seconds)
            node.loops = 1
            (directory / 'trajectory.yaml').write_text(yaml.safe_dump({
                'kind': 'elbow_out_and_back',
                'start_deg': args.q_deg,
                'end_deg': [args.q_deg[0], args.q_deg[1],
                            args.q_deg[2] + args.elbow_excursion_deg],
                'seconds_per_leg': args.move_seconds,
                'elbow_gravity_scale': args.elbow_gravity_scale,
                'max_tracking_error_rad': 0.035,
                'max_held_joint_error_rad': 0.025,
            }, sort_keys=False))
        executor.add_node(node)
        deadline = time.monotonic() + 4.0
        while node.phase == 'WAIT':
            if time.monotonic() > deadline:
                raise RuntimeError('Không có /joint_states mới')
            executor.spin_once(timeout_sec=0.005)
        if node.phase != 'GRAVITY':
            raise RuntimeError(f'Pha trước switch không hợp lệ: {node.phase}')
        if np.max(np.abs(node.home_q - target)) > math.radians(1.0):
            raise RuntimeError('Tay lệch hơn 1° so với tư thế dự kiến')
        samples = deque(maxlen=100)
        deadline = time.monotonic() + 0.5
        last_seq = node.state_seq
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.005)
            if node.state_seq != last_seq:
                last_seq = node.state_seq
                samples.append((time.monotonic(), node.q.copy(), node.qd.copy()))
        if not stable_samples(samples):
            raise RuntimeError('Tay chưa đứng yên trước switch')
        if node.publisher.get_subscription_count() < 1:
            raise RuntimeError('Effort controller chưa nhận topic commands')
        require_single_hardware_launch()
        node._publish(node._hold_torque(node.dt_nom, integrate=False))
        # GRAVITY resets the algorithm on every tick while this flag is false.
        # Arm it before the switch service spins the executor so SMC's feedback
        # slew state (and super-twisting z) can build through the 1 s handoff.
        # The physical command still belongs to the position controller until
        # the switch completes.
        node.manual_hold_latched = True
        print(f'Switch torque tối đa {args.seconds:g}s tổng cộng, gồm ramp 1s.', flush=True)
        switch_attempted = True
        switch(True)
        started = time.monotonic()
        hold_samples = deque(maxlen=100)
        hold_seq = node.state_seq
        while time.monotonic() - started < args.seconds:
            executor.spin_once(timeout_sec=0.005)
            if node.phase == 'ABORT':
                raise RuntimeError('Node SMC đã ABORT')
            if node.q is None:
                continue
            if node.state_seq != hold_seq:
                hold_seq = node.state_seq
                hold_samples.append((time.monotonic(), node.q.copy(), node.qd.copy()))
            error = float(np.max(np.abs(node.q - node.home_q)))
            speed = float(np.max(np.abs(node.qd)))
            hold_limit = 0.015 if args.elbow_excursion_deg else 0.025
            if error > hold_limit or speed > 0.3:
                raise RuntimeError(
                    f'Trôi {math.degrees(error):.2f}°, tốc độ {speed:.2f} rad/s')
        if args.elbow_excursion_deg:
            if not stable_samples(hold_samples):
                raise RuntimeError('Chưa có mẫu đứng yên trước quỹ đạo')
            result_set = node.set_parameters([Parameter('autostart', value=True)])
            if not result_set[0].successful:
                raise RuntimeError('Không bật được autostart cho quỹ đạo')
            deadline = time.monotonic() + 2 * args.move_seconds + 8.0
            while node.phase not in ('HOLD', 'ABORT'):
                if time.monotonic() > deadline:
                    raise RuntimeError('Quỹ đạo quá thời gian dự kiến')
                executor.spin_once(timeout_sec=0.005)
                if node.q is None:
                    continue
                if node.phase == 'GRAVITY':
                    # autostart is consumed in the next controller tick.
                    # The APPROACH reference does not exist before that tick.
                    continue
                reference = node._reference_at_phase(node.phase_elapsed)[0]
                error = np.abs(node.q - reference)
                if np.max(error[:2]) > 0.025 or error[2] > 0.035:
                    raise RuntimeError(
                        f'Sai số quỹ đạo {np.degrees(error).round(2)}°, dừng sớm')
            if node.phase == 'ABORT':
                raise RuntimeError('Node SMC đã ABORT khi chạy quỹ đạo')
            result = f'elbow đi/về {args.elbow_excursion_deg:g}° thành công'
        else:
            result = 'giữ torque 2s thành công'
    except Exception as exc:
        result = f'ABORT: {exc}'
        print(result, flush=True)
    finally:
        if switch_attempted:
            try:
                switch(False)
                final = states()
                if (final.get('gim_arm_group_controller') != 'active'
                        or final.get('gim_arm_effort_controller') != 'inactive'):
                    raise RuntimeError(f'Trạng thái sau switch: {final}')
                print('Đã trả về position mode.', flush=True)
            except Exception as exc:
                result = f'{result}; KHÔNG XÁC MINH ĐƯỢC POSITION: {exc}'
                print(result, flush=True)
        if node is not None:
            node.dump()
            executor.remove_node(node)
            node.destroy_node()
        executor.remove_node(probe)
        probe.destroy_node()
        rclpy.shutdown()
    print(f'Kết quả: {result}; log: {path}', flush=True)
    return 0 if result in (
        'giữ torque 2s thành công',
        f'elbow đi/về {args.elbow_excursion_deg:g}° thành công') else 2


if __name__ == '__main__':
    raise SystemExit(main())
