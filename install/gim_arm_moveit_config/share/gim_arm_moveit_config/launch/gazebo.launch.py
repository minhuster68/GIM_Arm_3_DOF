import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Lấy đường dẫn tới các package
    moveit_config_dir = get_package_share_directory('gim_arm_moveit_config')
    description_dir = get_package_share_directory('gim_arm_description')
    
    # Đường dẫn file URDF
    urdf_file = os.path.join(description_dir, 'urdf', 'gim_arm.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. Chạy thế giới ảo Gazebo (Gazebo Classic)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # 2. Node công bố trạng thái Robot (Để RViz và Gazebo hiểu hình dáng)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. Spawn (Thả) cánh tay vào Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'gim_arm', '-topic', 'robot_description', '-z', '0.0'],
        output='screen'
    )

    # 4. Kích hoạt bộ đọc trạng thái khớp
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # 5. Kích hoạt bộ điều khiển tay máy
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gim_arm_group_controller'],
        output='screen'
    )

    # Sequence: Spawn xong -> Load Broadcaster -> Load Controller
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller],
            )
        ),
    ])