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
    # value_type=str là BẮT BUỘC, không phải tuỳ chọn cho gọn. Thiếu nó thì
    # launch_ros tự suy kiểu tham số bằng cách thử yaml.safe_load() lên toàn bộ
    # nội dung URDF. XML thường tình cờ không parse được thành YAML nên "chạy
    # được", nhưng chỉ cần MỘT dòng comment trong URDF kết thúc bằng dấu hai
    # chấm hoặc chứa ": " là YAML đọc thành mapping và launch chết ngay với
    # "Unable to parse the value of parameter robot_description as yaml".
    # Đã gặp thật: comment về urdf_path có dòng "...hardware BÊN TRONG
    # ros2_control:" làm vỡ display.launch.py. Bọc value_type=str thì URDF được
    # truyền nguyên văn dưới dạng chuỗi, miễn nhiễm với mọi nội dung comment.
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