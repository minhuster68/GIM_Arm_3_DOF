"""
Start exactly one torque algorithm node.

The controller manager and MuJoCo/CAN simulator are intentionally launched
separately so switching the algorithm never changes the plant under test.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
ALGORITHMS = {'pid': ('gim_arm_controller_pid', 'cascade_pid_node', 'pid.yaml')}

def generate_launch_description():
    algorithm = LaunchConfiguration('algorithm')
    pid_profile = LaunchConfiguration('pid_profile')
    common_overrides = {
        'trajectory_shape': LaunchConfiguration('trajectory_shape'),
        'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
        'tau_scale': ParameterValue(LaunchConfiguration('tau_scale'), value_type=float),
        'control_hz': ParameterValue(LaunchConfiguration('control_hz'), value_type=float),
        'max_track_error_rad': ParameterValue(LaunchConfiguration('max_track_error_rad'), value_type=float),
        'max_transition_error_rad': ParameterValue(LaunchConfiguration('max_transition_error_rad'), value_type=float),
        'stale_recovery_threshold': ParameterValue(LaunchConfiguration('stale_recovery_threshold'), value_type=float),
        'stale_recovery_time': ParameterValue(LaunchConfiguration('stale_recovery_time'), value_type=float),
        'approach_time': ParameterValue(LaunchConfiguration('approach_time'), value_type=float),
        'return_time': ParameterValue(LaunchConfiguration('return_time'), value_type=float),
        'loops': ParameterValue(LaunchConfiguration('loops'), value_type=float),
        'diagnostic_hold': ParameterValue(LaunchConfiguration('diagnostic_hold'), value_type=bool),
        'diagnostic_segment': ParameterValue(LaunchConfiguration('diagnostic_segment'), value_type=bool),
        'diagnostic_q1_deg': ParameterValue(LaunchConfiguration('diagnostic_q1_deg'), value_type=float),
        'diagnostic_q2_deg': ParameterValue(LaunchConfiguration('diagnostic_q2_deg'), value_type=float),
        'diagnostic_q3_deg': ParameterValue(LaunchConfiguration('diagnostic_q3_deg'), value_type=float),
        'diagnostic_hold_time': ParameterValue(LaunchConfiguration('diagnostic_hold_time'), value_type=float),
        'diagnostic_qd1_rad_s': ParameterValue(LaunchConfiguration('diagnostic_qd1_rad_s'), value_type=float),
        'diagnostic_qd2_rad_s': ParameterValue(LaunchConfiguration('diagnostic_qd2_rad_s'), value_type=float),
        'diagnostic_qd3_rad_s': ParameterValue(LaunchConfiguration('diagnostic_qd3_rad_s'), value_type=float),
        'diagnostic_segment_time': ParameterValue(LaunchConfiguration('diagnostic_segment_time'), value_type=float),
        'command_heartbeat_nm': ParameterValue(LaunchConfiguration('command_heartbeat_nm'), value_type=float),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'log_file': LaunchConfiguration('log_file'),
    }
    actions = [
        DeclareLaunchArgument(
            'trajectory_shape',
            default_value='circle',
            choices=['circle', 'r', 'a'],
        ),
        DeclareLaunchArgument('algorithm', default_value='pid', choices=list(ALGORITHMS)),
        DeclareLaunchArgument(
            'pid_profile',
            default_value='gazebo_smooth',
            choices=['matlab_reference', 'gazebo_smooth'],
            description='PID only: single-loop position PID at 2 kHz for Gazebo, or the 100 Hz position PID baseline',
        ),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument(
            'tau_scale',
            default_value=PythonExpression(
                ["'0.35' if '", pid_profile, "' == 'matlab_reference' else ", "'0.50'"],
            ),
        ),
        DeclareLaunchArgument(
            'control_hz',
            default_value=PythonExpression(
                ["'100.0' if '", pid_profile, "' == 'matlab_reference' else ", "'2000.0'"],
            ),
        ),
        DeclareLaunchArgument('max_track_error_rad', default_value='0.05'),
        DeclareLaunchArgument('max_transition_error_rad', default_value='0.10'),
        DeclareLaunchArgument(
            'stale_recovery_threshold',
            default_value='0.03',
            description='State-feedback gap in seconds that activates damped-hold recovery',
        ),
        DeclareLaunchArgument(
            'stale_recovery_time',
            default_value='0.25',
            description='Damped-hold duration before an interrupted trajectory resumes',
        ),
        DeclareLaunchArgument('approach_time', default_value='5.0'),
        DeclareLaunchArgument('return_time', default_value='5.0'),
        DeclareLaunchArgument('loops', default_value='1.0'),
        DeclareLaunchArgument('diagnostic_hold', default_value='false'),
        DeclareLaunchArgument('diagnostic_segment', default_value='false'),
        DeclareLaunchArgument('diagnostic_q1_deg', default_value='0.0'),
        DeclareLaunchArgument('diagnostic_q2_deg', default_value='0.0'),
        DeclareLaunchArgument('diagnostic_q3_deg', default_value='0.0'),
        DeclareLaunchArgument('diagnostic_hold_time', default_value='15.0'),
        DeclareLaunchArgument('diagnostic_qd1_rad_s', default_value='0.0'),
        DeclareLaunchArgument('diagnostic_qd2_rad_s', default_value='0.0'),
        DeclareLaunchArgument('diagnostic_qd3_rad_s', default_value='0.0'),
        DeclareLaunchArgument('diagnostic_segment_time', default_value='2.0'),
        DeclareLaunchArgument('command_heartbeat_nm', default_value='1.0e-6'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_file', default_value=''),
    ]
    for (name, (package, executable, config)) in ALGORITHMS.items():
        config_file = config
        config_file = PythonExpression(
            [
                "'pid.yaml' if '",
                pid_profile,
                "' == 'matlab_reference' else ",
                "'pid_gazebo_smooth.yaml'",
            ],
        )
        parameter_overrides = [common_overrides]
        actions.append(
            Node(
                package=package,
                executable=executable,
                output='screen',
                parameters=[
                    PathJoinSubstitution([FindPackageShare(package), 'config', config_file]),
                    *parameter_overrides,
                ],
                condition=IfCondition(PythonExpression(["'", algorithm, "' == '", name, "'"])),
            ),
        )
    return LaunchDescription(actions)
