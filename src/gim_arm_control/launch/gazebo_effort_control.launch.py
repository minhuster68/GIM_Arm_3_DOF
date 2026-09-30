"""Start Gazebo Classic with GIM Arm and ros2_control effort interfaces."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from gim_control.gazebo_description import build_gazebo_description
from gim_control.sweep_trajectory import TOOL_OFFSET


def _launch_setup(context):
    description_share = get_package_share_directory("gim_arm_description")
    control_share = get_package_share_directory("gim_control")
    gazebo_share = get_package_share_directory("gazebo_ros")

    urdf_path = os.path.join(
        description_share, "urdf", "gim_arm.urdf")
    controllers_path = os.path.join(
        control_share, "config", "controllers_gazebo.yaml")
    world_path = os.path.join(
        control_share, "worlds", "gim_arm_zero_gravity.world")
    payload_mass_kg = float(
        LaunchConfiguration("payload_mass_kg").perform(context))
    robot_description = build_gazebo_description(
        urdf_path, controllers_path,
        payload_mass_kg=payload_mass_kg,
        payload_offset_xyz=TOOL_OFFSET)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "pause": "true",
            "verbose": LaunchConfiguration("verbose"),
            "world": world_path,
        }.items(),
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
        output="screen",
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "gim_arm",
        ],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )
    forward_position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )
    effort_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gim_arm_effort_controller",
            "--controller-manager", "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )
    trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gim_arm_group_controller",
            "--controller-manager", "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    hold_home = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once",
            "/forward_position_controller/commands",
            "std_msgs/msg/Float64MultiArray",
            "{data: [0.0, 0.0, 0.0]}",
        ],
        output="screen",
    )
    unpause = ExecuteProcess(
        cmd=[
            "ros2", "service", "call", "/unpause_physics",
            "std_srvs/srv/Empty", "{}",
        ],
        output="screen",
    )
    enable_gravity = ExecuteProcess(
        cmd=[
            "gz", "physics", "-g",
            ["0,0,", LaunchConfiguration("gravity_z")],
        ],
        output="screen",
    )

    start_physics = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=[unpause],
    ))
    start_jsb = RegisterEventHandler(OnProcessExit(
        target_action=unpause,
        on_exit=[joint_state_spawner],
    ))
    start_position = RegisterEventHandler(OnProcessExit(
        target_action=joint_state_spawner,
        on_exit=[forward_position_spawner],
    ))
    load_effort = RegisterEventHandler(OnProcessExit(
        target_action=forward_position_spawner,
        on_exit=[effort_spawner],
    ))
    load_trajectory = RegisterEventHandler(OnProcessExit(
        target_action=effort_spawner,
        on_exit=[trajectory_spawner],
    ))
    command_home = RegisterEventHandler(OnProcessExit(
        target_action=trajectory_spawner,
        on_exit=[hold_home],
    ))
    start_gravity = RegisterEventHandler(OnProcessExit(
        target_action=hold_home,
        on_exit=[enable_gravity],
    ))
    return [
        gazebo,
        state_publisher,
        spawn_robot,
        start_physics,
        start_jsb,
        start_position,
        load_effort,
        load_trajectory,
        command_home,
        start_gravity,
    ]


def generate_launch_description():
    gazebo_model_path = os.pathsep.join(filter(None, [
        "/usr/share/gazebo-11/models",
        os.environ.get("GAZEBO_MODEL_PATH", ""),
    ]))
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("verbose", default_value="false"),
        DeclareLaunchArgument("gravity_z", default_value="-9.81"),
        DeclareLaunchArgument(
            "payload_mass_kg",
            default_value="0.0",
            description=(
                "Unmodelled point payload at the EE tool offset; Gazebo only")),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
        OpaqueFunction(function=_launch_setup),
    ])
