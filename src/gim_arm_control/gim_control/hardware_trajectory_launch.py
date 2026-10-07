"""Hardware controller launches using the PID test's CAN workflow."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ALGORITHMS = {
    'lqr': ('gim_arm_controller_lqr', 'lqr_node', 'lqr_hardware_soft.yaml'),
    'mpc': ('gim_arm_controller_mpc', 'mpc_node', 'mpc_hardware_soft.yaml'),
    'smc': ('gim_arm_controller_smc', 'smc_node', 'smc_hardware_soft.yaml'),
}


def _launch_setup(context):
    def get(name):
        return LaunchConfiguration(name).perform(context)

    algorithm, shape = get('algorithm'), get('trajectory_shape')
    if algorithm not in ALGORITHMS or shape not in ('circle', 'r', 'a'):
        raise ValueError('algorithm phải là lqr/mpc/smc; trajectory_shape phải là circle/r/a')
    durations = {name: float(get(name)) for name in ('approach_time', 'return_time')}
    if any(not math.isfinite(value) or value <= 0 for value in durations.values()):
        raise ValueError('approach_time/return_time phải hữu hạn và > 0')
    package, executable, config = ALGORITHMS[algorithm]
    configured_file = get('params_file')
    params_file = (os.path.abspath(os.path.expanduser(configured_file)) if configured_file
                   else os.path.join(get_package_share_directory(package), 'config', config))
    if not os.path.isfile(params_file):
        raise ValueError(f'Không tìm thấy params_file: {params_file}')
    configured_log = get('log_file') or f'results/{algorithm}_{shape}_run01.csv'
    log_file = os.path.abspath(os.path.expanduser(configured_log))
    os.makedirs(os.path.dirname(log_file), exist_ok=True)
    overrides = {
        'autostart': False,
        'use_sim_time': False,
        'diagnostic_hold': False,
        'diagnostic_segment': False,
        'cascade_hold': False,
        'algorithm_hold': True,
        'start_velocity_limit_rad_s': 0.05,
        'trajectory_shape': shape,
        'loops': 1.0,
        'approach_time': durations['approach_time'],
        'return_time': durations['return_time'],
        'cache_file': '',
        'log_file': log_file,
    }
    return [Node(
        package=package, executable=executable, parameters=[params_file, overrides],
        output='screen', additional_env={
            'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'})]


def generate_hardware_launch(default_algorithm):
    return LaunchDescription([
        DeclareLaunchArgument('algorithm', default_value=default_algorithm,
                              choices=list(ALGORITHMS)),
        DeclareLaunchArgument('trajectory_shape', default_value='circle',
                              choices=['circle', 'r', 'a']),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('approach_time', default_value='16.0'),
        DeclareLaunchArgument('return_time', default_value='16.0'),
        DeclareLaunchArgument('log_file', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
