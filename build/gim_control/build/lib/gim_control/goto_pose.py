#!/usr/bin/env python3
"""
goto_pose.py — đưa tay tới một tư thế MỘT CÁCH ÊM. Dùng đường PID sẵn có.

    ros2 run gim_control goto_pose 0.20 -0.89 -0.62
    ros2 run gim_control goto_pose 0.20 -0.89 -0.62 --time 8

Thay cho việc gõ vòng `for` với `ros2 topic pub` bằng tay. Ba thứ nó lo giúp:

1) NỘI SUY BẬC 5 từ tư thế hiện tại, 50 Hz. `input_mode = 1` của driver là
   passthrough -- gửi thẳng một điểm cách 0.9 rad là tay giật mạnh. Nội suy
   bậc 5 có q̇ = q̈ = 0 ở hai đầu nên vào/ra đều êm.

2) GIỮ NGUYÊN các khớp không đổi. Gõ tay rất dễ quên điền 2 khớp còn lại, và
   `forward_position_controller` nhận cả 3 số -- thiếu là khớp đó nhảy về 0.

3) CHẶN tư thế ngoài giới hạn URDF trước khi gửi bất cứ gì.

YÊU CẦU: `forward_position_controller` đang active.
    ros2 control switch_controllers \\
        --deactivate gim_arm_group_controller --activate forward_position_controller
"""

import argparse
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ["base_joint", "shoulder_joint", "elbow_joint"]


class GotoPose(Node):
    def __init__(self, target, duration, hz, topic):
        super().__init__("goto_pose")
        self.target = np.asarray(target, float)
        self.duration = float(duration)
        self.dt = 1.0 / hz
        self.q = None
        self.q0 = None
        self.t = 0.0
        self.done = False
        self.limits = self.load_limits()

        for i, n in enumerate(JOINTS):
            lo, hi = self.limits[i]
            if not (lo <= self.target[i] <= hi):
                self.get_logger().error(
                    f"{n} = {self.target[i]:.4f} nằm ngoài giới hạn URDF "
                    f"[{lo:.4f}, {hi:.4f}]. KHÔNG gửi gì cả.")
                raise SystemExit(1)

        self.pub = self.create_publisher(Float64MultiArray, topic, 10)
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(f"Đích: {np.round(self.target, 4)}, "
                               f"đi trong {self.duration:g}s. Chờ /joint_states...")

    def load_limits(self):
        import xml.etree.ElementTree as ET
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("gim_arm_description"),
                            "urdf", "gim_arm.urdf")
        root = ET.parse(path).getroot()
        out = []
        for n in JOINTS:
            j = next(x for x in root.findall("joint") if x.get("name") == n)
            lim = j.find("limit")
            out.append((float(lim.get("lower")), float(lim.get("upper"))))
        return out

    def on_state(self, msg):
        try:
            idx = [msg.name.index(j) for j in JOINTS]
        except ValueError:
            return
        self.q = np.array([msg.position[i] for i in idx], float)

    def tick(self):
        if self.q is None:
            return
        if self.q0 is None:
            self.q0 = self.q.copy()
            self.get_logger().info(
                f"Bắt đầu từ {np.round(self.q0, 4)}, "
                f"đi {np.round(np.abs(self.target - self.q0), 4)} rad")
        self.t += self.dt
        s = min(self.t / self.duration, 1.0)
        # đa thức bậc 5: q̇ = q̈ = 0 ở cả hai đầu -> không giật
        h = 10 * s**3 - 15 * s**4 + 6 * s**5
        cmd = self.q0 + h * (self.target - self.q0)
        m = Float64MultiArray()
        m.data = [float(x) for x in cmd]
        self.pub.publish(m)
        if s >= 1.0 and not self.done:
            self.done = True
            err = self.q - self.target
            self.get_logger().info(
                f"Xong. q = {np.round(self.q, 4)}, "
                f"sai lệch = {np.round(err, 4)} rad "
                f"({np.round(np.degrees(err), 2)} độ)")
            self.get_logger().info("Giữ nguyên lệnh. Ctrl-C khi đã đo xong.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("q", nargs=3, type=float, metavar=("BASE", "SHOULDER", "ELBOW"))
    ap.add_argument("--time", type=float, default=6.0, help="giây")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--topic", default="/forward_position_controller/commands")
    args, _ = ap.parse_known_args()

    rclpy.init()
    try:
        node = GotoPose(args.q, args.time, args.hz, args.topic)
    except SystemExit:
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())