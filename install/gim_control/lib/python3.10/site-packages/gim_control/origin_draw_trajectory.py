#!/usr/bin/env python3
"""
draw_trajectory.py — cầu nối MỎNG duy nhất giữa gim_arm_kinematics.py (toán
thuần, không ROS) và ROS 2 thật. Đóng gói chuỗi góc khớp thành
FollowJointTrajectory rồi gửi qua action client.

Tự đọc vị trí khớp hiện tại qua /joint_states, tự chèn 1 đoạn di chuyển êm
về điểm đầu quỹ đạo vẽ trước khi vẽ thật. Có đồ thị REAL-TIME (actual vs
desired, 3 khớp) cập nhật liên tục ngay trong lúc tay máy đang chạy.

Quỹ đạo lấy từ gim_control/sweep_trajectory.py -- ĐÚNG file mà
kinematics_test/test_sweep_mujoco.py dùng để mô phỏng, nên cái đã xem trong
MuJoCo chính là cái chạy trên tay thật.

Đặt trong gim_control/gim_control/ (cùng chỗ với gim_arm_kinematics.py và
shapes.py) -- import bên dưới dùng đúng đường dẫn package `gim_control.xxx`.
"""

import time

import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import os

from gim_control.gim_arm_kinematics import GimArmKinematics
from gim_control import sweep_trajectory


class DrawTrajectoryNode(Node):
    def __init__(self):
        super().__init__("draw_trajectory_node")
        self._client = ActionClient(
            self, FollowJointTrajectory, "/gim_arm_group_controller/follow_joint_trajectory"
        )

    def get_current_joint_positions(self, joint_names, timeout_sec: float = 5.0):
        """Đọc vị trí khớp hiện tại qua /joint_states, đúng thứ tự joint_names."""
        received = {}

        def callback(msg: JointState):
            for name, pos in zip(msg.name, msg.position):
                received[name] = pos

        sub = self.create_subscription(JointState, "/joint_states", callback, 10)
        start = time.time()
        while not all(n in received for n in joint_names) and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)

        if not all(n in received for n in joint_names):
            raise RuntimeError(
                f"Không nhận đủ /joint_states trong {timeout_sec}s -- "
                f"kiểm tra gim_arm_control.launch.py đã chạy chưa."
            )
        return [received[n] for n in joint_names]

    def send_trajectory(
        self, joint_names, q_list, dt: float, transition_time: float = 3.0,
        live_plot: bool = True,
    ):
        current_q = self.get_current_joint_positions(joint_names)
        self.get_logger().info(f"Vị trí hiện tại: {[round(x, 4) for x in current_q]}")
        self.get_logger().info(
            f"Sẽ di chuyển êm về điểm đầu quỹ đạo trong {transition_time}s trước khi vẽ."
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        pt0 = JointTrajectoryPoint()
        pt0.positions = [float(x) for x in current_q]
        pt0.time_from_start = Duration(sec=0, nanosec=0)
        goal.trajectory.points.append(pt0)

        desired_t = [0.0]
        desired_q = [list(current_q)]

        pt1 = JointTrajectoryPoint()
        pt1.positions = [float(x) for x in q_list[0]]
        sec = int(transition_time)
        nsec = int((transition_time - sec) * 1e9)
        pt1.time_from_start = Duration(sec=sec, nanosec=nsec)
        goal.trajectory.points.append(pt1)
        desired_t.append(transition_time)
        desired_q.append(list(q_list[0]))

        t = transition_time + dt
        for q in q_list[1:]:
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            goal.trajectory.points.append(pt)
            desired_t.append(t)
            desired_q.append(list(q))
            t += dt

        desired_q_arr = np.array(desired_q)

        # ---- Bắt đầu ghi actual + dựng đồ thị real-time ----
        actual_t = []
        actual_q = []
        t0_holder = [None]

        def state_callback(msg: JointState):
            now = self.get_clock().now().nanoseconds / 1e9
            if t0_holder[0] is None:
                t0_holder[0] = now
            pos_dict = dict(zip(msg.name, msg.position))
            if not all(n in pos_dict for n in joint_names):
                return
            actual_t.append(now - t0_holder[0])
            actual_q.append([pos_dict[n] for n in joint_names])

        state_sub = self.create_subscription(JointState, "/joint_states", state_callback, 50)

        fig = axes = actual_lines = None
        if live_plot:
            plt.ion()
            fig, axes = plt.subplots(
                len(joint_names), 1, figsize=(9, 3 * len(joint_names)), sharex=True
            )
            if len(joint_names) == 1:
                axes = [axes]
            actual_lines = []
            for i, name in enumerate(joint_names):
                axes[i].plot(
                    desired_t, desired_q_arr[:, i], "o--", color="tab:orange",
                    markersize=4, linewidth=1.5, label="Desired (mong muốn)",
                )
                (line,) = axes[i].plot([], [], "-", color="tab:blue", linewidth=1.5,
                                        label="Actual (thật)")
                actual_lines.append(line)
                axes[i].set_ylabel(f"{name}\n(rad)")
                axes[i].grid(True, alpha=0.3)
                axes[i].legend(loc="upper right", fontsize=8)
            axes[-1].set_xlabel("Thời gian (s)")
            fig.suptitle("Actual vs Desired -- cập nhật real-time")
            fig.tight_layout()
            plt.show(block=False)
            plt.pause(0.01)

        def refresh_plot():
            if not live_plot or len(actual_t) == 0:
                return
            aq = np.array(actual_q)
            for i, line in enumerate(actual_lines):
                line.set_data(actual_t, aq[:, i])
            for ax in axes:
                ax.relim()
                ax.autoscale_view()
            fig.canvas.draw_idle()
            plt.pause(0.001)

        self.get_logger().info("Đang chờ action server gim_arm_group_controller...")
        self._client.wait_for_server()

        send_future = self._client.send_goal_async(goal)
        while not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            refresh_plot()
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal bị từ chối -- kiểm tra lại giới hạn góc/vận tốc khớp.")
            self.destroy_subscription(state_sub)
            return False

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            refresh_plot()

        result = result_future.result().result
        self.get_logger().info(f"Hoàn tất, error_code={result.error_code} (0 = thành công)")
        self.destroy_subscription(state_sub)
        refresh_plot()

        if live_plot:
            print("Vẽ xong -- đóng cửa sổ đồ thị để kết thúc chương trình.")
            plt.ioff()
            plt.show()

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
    node.send_trajectory(
        kin.joint_names, q_list, dt=sweep_trajectory.DT,
        transition_time=sweep_trajectory.TRANSITION_TIME, live_plot=True,
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()