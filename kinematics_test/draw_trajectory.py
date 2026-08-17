#!/usr/bin/env python3
"""
draw_trajectory.py — cầu nối MỎNG duy nhất giữa gim_arm_kinematics.py (toán
thuần, không ROS) và ROS 2 thật. Chỉ làm đúng 1 việc: đóng gói chuỗi góc khớp
thành FollowJointTrajectory rồi gửi qua action client.

Hiện đang để CÙNG thư mục với gim_arm_kinematics.py (chưa đưa vào package
gim_control) -- import bên dưới dùng đúng kiểu "cùng thư mục" khớp với cách
đang làm. Khi nào chuyển hẳn vào gim_control/gim_control/, đổi dòng import
GimArmKinematics thành `from gim_control.gim_arm_kinematics import ...`.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory
import os

from gim_arm_kinematics import GimArmKinematics
import sweep_trajectory


class DrawTrajectoryNode(Node):
    def __init__(self):
        super().__init__("draw_trajectory_node")
        self._client = ActionClient(
            self, FollowJointTrajectory, "/gim_arm_group_controller/follow_joint_trajectory"
        )

    def send_trajectory(self, joint_names, q_list, dt: float,
                        lead_in: float = sweep_trajectory.TRANSITION_TIME):
        """lead_in: thời gian dành cho điểm ĐẦU TIÊN. Bản này không đọc
        /joint_states nên không biết tay đang ở đâu; cho điểm đầu hẳn vài giây
        để controller nội suy ÊM từ tư thế hiện tại về điểm đầu, thay vì ép đi
        hết quãng đường đó trong đúng 1 dt (giật mạnh, nguy hiểm với thiết bị
        đeo trên người). Bản trong package (origin_draw_trajectory.py) đọc
        /joint_states nên dựng được đoạn chuyển tiếp chuẩn hơn."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        t = lead_in
        for q in q_list:
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            goal.trajectory.points.append(pt)
            t += dt

        self.get_logger().info("Đang chờ action server gim_arm_group_controller...")
        self._client.wait_for_server()

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal bị từ chối -- kiểm tra lại giới hạn góc/vận tốc khớp.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f"Hoàn tất, error_code={result.error_code} (0 = thành công)")
        return result.error_code == 0


def main():
    rclpy.init()

    urdf_path = os.path.join(
        get_package_share_directory("gim_arm_description"), "urdf", "gim_arm.urdf"
    )
    kin = GimArmKinematics(urdf_path, tool_offset_xyz=sweep_trajectory.TOOL_OFFSET)

    positions, results = sweep_trajectory.solve(kin)
    ok, lines = sweep_trajectory.safety_report(kin, positions, results)
    for line in lines:
        print(line)
    if not ok:
        print("DỪNG: không gửi quỹ đạo chưa đạt ngưỡng an toàn xuống tay thật.")
        return

    node = DrawTrajectoryNode()
    q_list = [r.q for r in results]
    node.send_trajectory(kin.joint_names, q_list, dt=sweep_trajectory.DT)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()