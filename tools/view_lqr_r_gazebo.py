#!/usr/bin/env python3
"""Show an LQR letter-R trial in Gazebo, on a separate ROS domain."""

import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from gim_arm_controller_lqr.factory import LqrFactory
from gim_control.effort_controller_node import EffortControllerNode
from gim_control.plot_ee_error import load_log, plot, print_metrics


ROOT = Path(__file__).resolve().parents[1]


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '88':
        raise RuntimeError('Run through tools/view_lqr_r_gazebo.sh (ROS domain 88).')
    with socket.socket() as connection:
        if connection.connect_ex(('127.0.0.1', 11388)) == 0:
            raise RuntimeError('Gazebo port 11388 is already in use.')
    output = ROOT / 'results' / ('lqr_r_gazebo_' + time.strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True)
    csv_path = output / 'tracking.csv'
    print(f'Kết quả: {output}', flush=True)
    gazebo_log = (output / 'gazebo.log').open('w')
    gazebo = subprocess.Popen([
        'ros2', 'launch', 'gim_control', 'gazebo_effort_control.launch.py',
        'gui:=true', 'verbose:=false'], stdout=gazebo_log, stderr=subprocess.STDOUT,
        start_new_session=True)
    executor = probe = controller = None
    effort_active = False
    try:
        rclpy.init(args=[
            '--ros-args', '--params-file', str(ROOT / 'src' /
                'gim_arm_controller_lqr/config/lqr_gazebo_matlab.yaml'),
            '-p', 'trajectory_shape:=r', '-p', 'approach_time:=16.0',
            '-p', 'return_time:=16.0', '-p', 'loops:=1.0',
            '-p', 'max_transition_error_rad:=0.10', '-p', 'max_track_error_rad:=0.10',
            '-p', 'command_heartbeat_nm:=0.0', '-p', 'use_sim_time:=false',
            '-p', f'log_file:={csv_path}'])
        executor = SingleThreadedExecutor()
        probe = Node('gazebo_lqr_r_viewer')
        executor.add_node(probe)
        listing = probe.create_client(ListControllers, '/controller_manager/list_controllers')
        switching = probe.create_client(SwitchController, '/controller_manager/switch_controller')
        description = probe.create_client(GetParameters, '/robot_state_publisher/get_parameters')

        def call(client, request, timeout=60.0):
            deadline = time.monotonic() + timeout
            while not client.service_is_ready():
                if gazebo.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f'Gazebo/service chưa sẵn sàng: {client.srv_name}')
                executor.spin_once(timeout_sec=0.02)
            future = client.call_async(request)
            while not future.done():
                if time.monotonic() > deadline:
                    raise TimeoutError(client.srv_name)
                executor.spin_once(timeout_sec=0.005)
            return future.result()

        def switch(enable):
            request = SwitchController.Request()
            request.activate_controllers = ['gim_arm_effort_controller' if enable
                                            else 'forward_position_controller']
            request.deactivate_controllers = ['forward_position_controller' if enable
                                              else 'gim_arm_effort_controller']
            request.strictness = SwitchController.Request.STRICT
            request.activate_asap = True
            request.timeout.sec = 10
            if not call(switching, request).ok:
                raise RuntimeError('Không switch được controller mô phỏng.')

        deadline = time.monotonic() + 90.0
        while True:
            states = {entry.name: entry.state for entry in
                      call(listing, ListControllers.Request()).controller}
            if states.get('forward_position_controller') == 'active' and (
                    states.get('gim_arm_effort_controller') == 'inactive' and
                    states.get('gim_arm_group_controller') == 'inactive'):
                break
            if time.monotonic() > deadline:
                raise TimeoutError('Gazebo controllers')
            executor.spin_once(timeout_sec=0.02)
        request = GetParameters.Request(names=['robot_description'])
        urdf = call(description, request).values[0].string_value
        if 'gazebo_ros2_control/GazeboSystem' not in urdf:
            raise RuntimeError('Backend phải là GazeboSystem.')
        settle = time.monotonic() + 3.0
        while time.monotonic() < settle:
            executor.spin_once(timeout_sec=0.01)
        controller = EffortControllerNode(LqrFactory())
        executor.add_node(controller)
        deadline = time.monotonic() + 180.0
        while controller.phase == 'WAIT':
            if time.monotonic() > deadline:
                raise TimeoutError('Chốt HOME và chuẩn bị bảng gain LQR')
            executor.spin_once(timeout_sec=0.005)
        switch(True)
        effort_active = True
        controller.set_parameters([Parameter('autostart', value=True)])
        print('Bắt đầu: HOME → 16 s đưa lên → chữ R 27 s → 16 s về HOME.', flush=True)
        deadline = time.monotonic() + 180.0
        while controller.phase not in ('HOLD', 'ABORT'):
            if gazebo.poll() is not None:
                raise RuntimeError('Gazebo đã đóng.')
            if time.monotonic() > deadline:
                raise TimeoutError('Quỹ đạo chưa hoàn tất')
            executor.spin_once(timeout_sec=0.005)
        # Plotting blocks this loop; return to position before writing results.
        switch(False)
        effort_active = False
        executor.remove_node(controller)
        controller.dump()
        result = {
            'algorithm': 'lqr', 'trajectory_shape': 'r',
            'profile': 'lqr_gazebo_matlab.yaml', 'control_hz': 1.0 / controller.dt_nom,
            'phase': controller.phase, 'samples': len(controller.rows),
            'passed': controller.phase == 'HOLD',
        }
        (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result), flush=True)
        log = load_log(csv_path)
        print_metrics(log)
        plot(log, str(output / 'tracking.png'), False, 6000)
        print('Gazebo vẫn mở. Ctrl+C trong terminal này để đóng mô phỏng.', flush=True)
        # Keep the simulator visible after one complete trial, holding HOME.
        while gazebo.poll() is None:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if controller is not None:
            if effort_active:
                try:
                    switch(False)
                except Exception as error:
                    print(f'Controller cleanup: {error}', flush=True)
            controller.dump()
            executor.remove_node(controller)
            controller.destroy_node()
        if probe is not None:
            executor.remove_node(probe)
            probe.destroy_node()
        if executor is not None:
            executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        if gazebo.poll() is None:
            os.killpg(gazebo.pid, signal.SIGINT)
            try:
                gazebo.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(gazebo.pid, signal.SIGTERM)
        gazebo_log.close()


if __name__ == '__main__':
    main()
