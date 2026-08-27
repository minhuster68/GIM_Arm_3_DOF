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
            FindPackageShare("gim_arm_description"), "urdf", "gim_arm.urdf"
        ]),
    ])
    # ParameterValue(..., value_type=str) là BẮT BUỘC, giống origin_gim_arm_control
    # .launch.py. Thiếu nó thì launch_ros đoán kiểu tham số bằng cách YAML-parse
    # chuỗi URDF, và URDF không phải YAML.
    #
    # ĐÃ KIỂM 27/08/2026, file này ĐANG VỠ THẬT: yaml.safe_load(gim_arm.urdf) ném
    # ScannerError "mapping values are not allowed here". Nguyên nhân là các
    # comment mới trong URDF có dòng kết thúc bằng dấu hai chấm rồi dòng sau trông
    # như "khoá: giá trị" -> YAML coi là mapping lồng nhau. Lỗi hiện ra ở launch
    # dưới dạng "Unable to parse the value of parameter robot_description as yaml"
    # và KHÔNG chỉ tới dòng comment gây ra, nên rất khó truy.
    # Ép kiểu str thì URDF muốn viết comment gì cũng được.
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