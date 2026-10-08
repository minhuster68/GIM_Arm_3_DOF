"""Run circle/R/A once using the hardware position PID baseline."""

import math
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    def get(name):
        return LaunchConfiguration(name).perform(context)

    durations = {name: float(get(name)) for name in ('approach_time', 'return_time')}
    if any(not math.isfinite(value) or value <= 0 for value in durations.values()):
        raise ValueError('approach_time và return_time phải hữu hạn và > 0')
    params_file = os.path.abspath(os.path.expanduser(get('params_file')))
    if not os.path.isfile(params_file):
        raise ValueError(f'Không tìm thấy params_file: {params_file}')
    log_file = os.path.abspath(os.path.expanduser(get('log_file')))
    os.makedirs(os.path.dirname(log_file), exist_ok=True)
    shape = context.launch_configurations.get('trajectory_shape', 'circle')
    if shape not in ('circle', 'r', 'a'):
        raise ValueError('trajectory_shape phải là circle, r hoặc a')
    overrides = {
        'trajectory_shape': shape,
        'autostart': False,
        'use_sim_time': False,
        'diagnostic_hold': False,
        'diagnostic_segment': False,
        'cascade_hold': True,
        'start_velocity_limit_rad_s': 0.05,
        'loops': 1.0,
        'approach_time': durations['approach_time'],
        'return_time': durations['return_time'],
        'cache_file': '',
        'log_file': log_file,
    }
    return [Node(
        package='gim_arm_controller_pid', executable='cascade_pid_node',
        parameters=[params_file, overrides], output='screen')]


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('gim_arm_controller_pid'), 'config', 'pid_hardware_soft.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('trajectory_shape', default_value='circle', choices=['circle', 'r', 'a']),
        DeclareLaunchArgument('approach_time', default_value='16.0'),
        DeclareLaunchArgument('return_time', default_value='16.0'),
        DeclareLaunchArgument('log_file', default_value='results/pid_sweep_run01.csv'),
        OpaqueFunction(function=_launch_setup),
    ])
