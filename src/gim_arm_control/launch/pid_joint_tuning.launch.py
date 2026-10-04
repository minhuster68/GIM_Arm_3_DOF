"""Test cascade PID on isolated joints or synchronized multi-joint references."""

import math
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


JOINTS = ('base', 'shoulder', 'elbow')
SELECTIONS = JOINTS + ('shoulder_elbow', 'all')


def _launch_setup(context):
    def get(name):
        return LaunchConfiguration(name).perform(context)

    joint = get('joint')
    if joint not in SELECTIONS:
        raise ValueError('joint phải là base, shoulder, elbow, shoulder_elbow hoặc all')
    target_deg = [0.0, 0.0, 0.0]
    if joint in ('shoulder_elbow', 'all'):
        target_deg[1] = float(get('shoulder_deg'))
        target_deg[2] = float(get('elbow_deg'))
        if joint == 'all':
            target_deg[0] = float(get('base_deg'))
    else:
        target_deg[JOINTS.index(joint)] = float(get('target_deg'))
    durations = {name: float(get(name)) for name in ('move_time', 'hold_time', 'return_time')}
    if not all(math.isfinite(angle) for angle in target_deg) or any(
            not math.isfinite(value) or value <= 0 for value in durations.values()):
        raise ValueError('Góc đích phải hữu hạn; các khoảng thời gian phải hữu hạn và > 0')
    params_file = os.path.abspath(os.path.expanduser(get('params_file')))
    if not os.path.isfile(params_file):
        raise ValueError(f'Không tìm thấy params_file: {params_file}')
    log_file = os.path.abspath(os.path.expanduser(get('log_file')))
    os.makedirs(os.path.dirname(log_file), exist_ok=True)
    overrides = {
        'autostart': False,
        'use_sim_time': False,
        'diagnostic_hold': True,
        'diagnostic_segment': False,
        'cascade_hold': True,
        'start_velocity_limit_rad_s': 0.05,
        'diagnostic_q1_deg': target_deg[0],
        'diagnostic_q2_deg': target_deg[1],
        'diagnostic_q3_deg': target_deg[2],
        'approach_time': durations['move_time'],
        'diagnostic_hold_time': durations['hold_time'],
        'return_time': durations['return_time'],
        'log_file': log_file,
    }
    return [Node(
        package='gim_arm_controller_pid', executable='cascade_pid_node',
        parameters=[params_file, overrides], output='screen')]


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('gim_arm_controller_pid'), 'config', 'pid_hardware_soft.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('joint', default_value='elbow', choices=list(SELECTIONS)),
        DeclareLaunchArgument('target_deg', default_value='30.0'),
        DeclareLaunchArgument(
            'base_deg', default_value='5.0',
            description='Absolute base target in degrees for joint:=all.'),
        DeclareLaunchArgument(
            'shoulder_deg', default_value='15.0',
            description='Absolute shoulder target for joint:=shoulder_elbow or all.'),
        DeclareLaunchArgument(
            'elbow_deg', default_value='30.0',
            description='Absolute elbow target for joint:=shoulder_elbow or all.'),
        DeclareLaunchArgument('move_time', default_value='6.0'),
        DeclareLaunchArgument('hold_time', default_value='3.0'),
        DeclareLaunchArgument('return_time', default_value='6.0'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('log_file', default_value='results/pid_joint_tuning.csv'),
        OpaqueFunction(function=_launch_setup),
    ])
