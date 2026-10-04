"""Bring up the real arm through SocketCAN."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from gim_control.hardware_description import configure_hardware_description


def _launch_setup(context):
    description_share = get_package_share_directory('gim_arm_description')
    control_share = get_package_share_directory('gim_control')
    zero_option = LaunchConfiguration('set_zero_on_startup').perform(context).lower()
    if zero_option not in ('true', 'false'):
        raise ValueError('set_zero_on_startup phải là true hoặc false')
    urdf_xml = xacro.process_file(os.path.join(
        description_share, 'urdf', 'gim_arm.urdf')).toxml()
    robot_description = {'robot_description': ParameterValue(
        configure_hardware_description(
            urdf_xml,
            can_interface=LaunchConfiguration('can_interface').perform(context),
            set_zero_on_startup=zero_option == 'true',
            zero_method=LaunchConfiguration('zero_method').perform(context),
            torque_joint=LaunchConfiguration('torque_joint').perform(context)),
        value_type=str)}

    control_node = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[robot_description, os.path.join(
            control_share, 'config', 'controllers.yaml')], output='screen')
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[robot_description], output='screen')

    def spawner(name, inactive=False):
        arguments = [name, '--controller-manager', '/controller_manager']
        if inactive:
            arguments.append('--inactive')
        return Node(
            package='controller_manager', executable='spawner',
            arguments=arguments, output='screen')

    joint_states = spawner('joint_state_broadcaster')
    controllers = [
        spawner('gim_arm_group_controller'),
        spawner('forward_position_controller', inactive=True),
        spawner('gim_arm_effort_controller', inactive=True),
    ]

    def spawn_after_joint_states(event, context):
        if event.returncode != 0:
            return []
        return controllers

    return [
        RegisterEventHandler(OnProcessExit(
            target_action=control_node,
            on_exit=[EmitEvent(event=Shutdown(reason='ros2_control_node exited'))])),
        control_node, state_publisher, joint_states,
        RegisterEventHandler(OnProcessExit(
            target_action=joint_states, on_exit=spawn_after_joint_states)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument(
            'set_zero_on_startup', default_value='false', choices=['true', 'false'],
            description='Establish zero once at startup; support arm at q=0.'),
        DeclareLaunchArgument(
            'zero_method', default_value='can', choices=['can', 'software'],
            description='CAN resets counts; software captures encoder offsets for ROS.'),
        DeclareLaunchArgument(
            'torque_joint', default_value='all',
            choices=['all', 'base', 'shoulder', 'elbow', 'shoulder_elbow'],
            description='Selected joints use torque mode; others hold position. '
                        'all preserves URDF joint configuration.'),
        OpaqueFunction(function=_launch_setup),
    ])
