"""Gazebo-only rounded-square preview, with visible path and q/qd/qdd goals."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    control = FindPackageShare('gim_control')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('repeat', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            control, 'launch', 'gazebo_effort_control.launch.py'])), launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'world': PathJoinSubstitution([control, 'worlds', 'gim_arm_square_preview.world']),
                'show_tool_marker': 'true',
                'payload_mass_kg': '0.0',
            }.items()),
        Node(package='gim_control', executable='preview_gazebo_trajectory', output='screen',
             parameters=[{
                 'urdf_file': PathJoinSubstitution([
                     FindPackageShare('gim_arm_description'), 'urdf', 'gim_arm.urdf']),
                 'trajectory_shape': 'square',
                 'use_sim_time': True,
                 'repeat': ParameterValue(LaunchConfiguration('repeat'), value_type=bool),
             }]),
    ])
