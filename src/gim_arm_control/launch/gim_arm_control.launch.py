from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # robot_description: chạy qua xacro dù gim_arm.urdf hiện tại chưa dùng macro nào --
    # để sau này đổi sang .xacro (ví dụ tham số hoá kích thước) không phải sửa launch file.
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("gim_arm_description"), "urdf", "gim_arm.urdf"
        ]),
    ])
    # value_type=str là BẮT BUỘC (xem giải thích dài trong display.launch.py):
    # thiếu nó thì launch_ros thử yaml.safe_load() lên nội dung URDF, và một
    # dòng comment chứa ": " sẽ làm launch chết với "Unable to parse the value
    # of parameter robot_description as yaml".
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("gim_control"), "config", "controllers.yaml"
    ])

    # controller_manager: đọc <ros2_control> trong URDF để nạp plugin
    # gim_arm_hardware/GimArmSystemHardware, và chạy vòng lặp read()/write()
    # ở update_rate khai trong controllers.yaml.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    gim_arm_group_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gim_arm_group_controller", "--controller-manager", "/controller_manager"],
    )

    # forward_position_controller vẫn nạp sẵn để dùng khi cần test point-to-point
    # nhanh, nhưng KHÔNG active cùng lúc với gim_arm_group_controller -- cả 2
    # đều claim command_interfaces "position" của cùng 3 khớp, controller_manager
    # không cho 2 controller cùng giữ 1 command interface. Dùng
    # `ros2 control switch_controllers` để đổi qua lại khi cần.
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller", "--controller-manager", "/controller_manager",
            "--inactive",
        ],
    )

    # Đợi joint_state_broadcaster load xong rồi mới spawn 2 controller kia,
    # tránh race condition lúc controller_manager vừa mới lên.
    delay_controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gim_arm_group_controller_spawner, forward_position_controller_spawner],
        )
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_controllers_after_jsb,
    ])