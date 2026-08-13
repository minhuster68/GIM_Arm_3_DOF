from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("gim_arm_description"), "urdf", "gim_arm_origin.urdf"
        ]),
    ])
    # value_type=str là BẮT BUỘC (xem giải thích dài trong display.launch.py):
    # thiếu nó thì launch_ros thử yaml.safe_load() lên nội dung URDF, và một
    # dòng comment chứa ": " sẽ làm launch chết với "Unable to parse the value
    # of parameter robot_description as yaml".
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Thay cho hardware/CAN thật -- cho phép kéo thanh trượt để tự tay xoay
    # từng khớp, xem hình dạng/giới hạn góc mà không cần cắm bất kỳ động cơ nào.
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])