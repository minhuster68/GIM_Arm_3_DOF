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
from control_msgs.msg import JointTrajectoryControllerState
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
        # Ghi thời gian TUYỆT ĐỐI, chưa trừ gốc. Gốc đúng là lúc goal được chấp
        # nhận (t_start bên dưới) chứ KHÔNG phải lúc subscribe: giữa 2 mốc đó
        # còn wait_for_server() có thể chờ lâu tuỳ lúc. Lấy gốc sai thì đồ thị
        # actual bị trượt ngang so với desired, và sai số tính ra là sai số của
        # phép trượt đó chứ không phải của bộ điều khiển.
        actual_t = []
        actual_q = []
        t_start = [None]

        def state_callback(msg: JointState):
            pos_dict = dict(zip(msg.name, msg.position))
            if not all(n in pos_dict for n in joint_names):
                return
            actual_t.append(self.get_clock().now().nanoseconds / 1e9)
            actual_q.append([pos_dict[n] for n in joint_names])

        def rel_time():
            """Thời gian actual quy về cùng gốc với desired."""
            if not actual_t:
                return np.array([])
            base = t_start[0] if t_start[0] is not None else actual_t[0]
            return np.array(actual_t) - base

        state_sub = self.create_subscription(JointState, "/joint_states", state_callback, 50)

        # Sai số bám lấy TỪ CHÍNH JTC, không tự tính lại. JTC công bố
        # error = reference - feedback tại đúng cùng một thời điểm bên trong
        # vòng điều khiển, nên không dính bài toán căn trục thời gian giữa
        # /joint_states và mốc bắt đầu quỹ đạo. Tự nội suy desired theo đồng hồ
        # máy chủ thì chỉ lệch 0.3s là đã đẻ ra "sai số" 2 độ ở base_joint
        # (khớp này chạy 6.7 độ/s) -- lớn hơn sai số thật hàng chục lần.
        ctrl_err_t = []
        ctrl_err_q = []

        def ctrl_state_callback(msg: JointTrajectoryControllerState):
            if not msg.error.positions:
                return
            idx = {n: k for k, n in enumerate(msg.joint_names)}
            if not all(n in idx for n in joint_names):
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ctrl_err_t.append(stamp)
            ctrl_err_q.append([msg.error.positions[idx[n]] for n in joint_names])

        ctrl_sub = self.create_subscription(
            JointTrajectoryControllerState,
            "/gim_arm_group_controller/controller_state", ctrl_state_callback, 50)

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

        last_draw = [0.0]

        def refresh_plot(force=False):
            # Vẽ lại tối đa 5 lần/giây. Trước đây vẽ mỗi vòng lặp, mà mỗi lần vẽ
            # tốn hơn cả spin_once -> vòng lặp chậm, spin_once xử lý được ít
            # callback, và message bị rơi: đo được có ~6 mẫu/giây trong khi JTC
            # phát 50 Hz. Ít mẫu thì không thấy được dao động tần số cao.
            if not live_plot or len(actual_t) == 0:
                return
            if not force and (time.time() - last_draw[0]) < 0.2:
                return
            last_draw[0] = time.time()
            aq = np.array(actual_q)
            rt = rel_time()
            for i, line in enumerate(actual_lines):
                line.set_data(rt, aq[:, i])
            for ax in axes:
                ax.relim()
                ax.autoscale_view()
            fig.canvas.draw_idle()
            plt.pause(0.001)

        self.get_logger().info("Đang chờ action server gim_arm_group_controller...")
        self._client.wait_for_server()

        send_future = self._client.send_goal_async(goal)
        while not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
            refresh_plot()
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal bị từ chối -- kiểm tra lại giới hạn góc/vận tốc khớp.")
            self.destroy_subscription(state_sub)
            return False

        # Goal đã được nhận -> ĐÂY mới là t=0 của quỹ đạo.
        t_start[0] = self.get_clock().now().nanoseconds / 1e9

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
            refresh_plot()

        result = result_future.result().result
        self.get_logger().info(f"Hoàn tất, error_code={result.error_code} (0 = thành công)")
        self.destroy_subscription(state_sub)
        self.destroy_subscription(ctrl_sub)
        refresh_plot(force=True)

        self.print_tracking_error(
            joint_names, ctrl_err_t, ctrl_err_q, t_start[0], transition_time)

        if live_plot:
            print("Vẽ xong -- đóng cửa sổ đồ thị để kết thúc chương trình.")
            plt.ioff()
            plt.show()

        return result.error_code == 0

    def print_tracking_error(self, joint_names, err_t, err_q, t_start, transition_time):
        """In SỐ ĐO sai số bám, lấy thẳng từ trường `error` của JTC.

        Không có con số thì không tune được: mắt không phân biệt nổi 0.44mm với
        0.27mm trên đồ thị.

        Chỉ đo phần SAU đoạn chuyển tiếp -- đoạn đi từ tư thế hiện tại về điểm
        đầu quỹ đạo không phải là bám quỹ đạo. Mốc cắt lấy theo t_start nên có
        thể lệch vài trăm ms, nhưng điều đó chỉ đổi VÀI MẪU được tính, không
        làm sai giá trị sai số của từng mẫu (khác hẳn cách tự nội suy desired).

        Cách dùng để so sánh A/B: chạy 1 lần với feedforward tắt, 1 lần bật,
        rồi so 2 bảng. Bật/tắt bằng gravity_feedforward / velocity_feedforward
        trong gim_arm.urdf, không cần build lại C++."""
        if len(err_t) < 10:
            print("\nKhông nhận đủ /gim_arm_group_controller/controller_state "
                  "để tính sai số bám.")
            print("Kiểm: ros2 topic hz /gim_arm_group_controller/controller_state")
            return

        t = np.array(err_t)
        e = np.array(err_q)
        if t_start is not None:
            keep = (t - t_start) >= transition_time
            if keep.sum() >= 10:
                t, e = t[keep], e[keep]

        rms = np.degrees(np.sqrt((e ** 2).mean(axis=0)))
        mx = np.degrees(np.abs(e).max(axis=0))
        bias = np.degrees(e.mean(axis=0))

        print("\n" + "=" * 62)
        print(f"SAI SỐ BÁM (JTC tự tính, {len(t)} mẫu sau đoạn chuyển tiếp)")
        print("=" * 62)
        print(f"{'khớp':<18}{'RMS (độ)':>11}{'lớn nhất (độ)':>15}{'lệch TB (độ)':>15}")
        for i, name in enumerate(joint_names):
            print(f"{name:<18}{rms[i]:11.4f}{mx[i]:15.4f}{bias[i]:15.4f}")
        print("=" * 62)
        print("Lệch TB khác 0 nhiều = sai số CÓ HỆ THỐNG (võng trọng lực hoặc")
        print("trễ bám) -- đúng loại mà feedforward xử lý được. RMS lớn mà lệch")
        print("TB ~ 0 = dao động/nhiễu, phải xử lý bằng gain chứ không phải FF.")


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