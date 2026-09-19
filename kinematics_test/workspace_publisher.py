import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math

class GimArmWorkspace(Node):
    def __init__(self):
        super().__init__('gim_arm_workspace_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'ee_workspace_cloud', 10)
        
        self.get_logger().info('Đang tính toán Point Cloud với Offset của EE...')
        self.points = self.calculate_workspace()
        self.get_logger().info(f'Đã tính xong {len(self.points)} điểm! Đang publish...')

        self.timer = self.create_timer(2.0, self.timer_callback)

    def rpy_to_matrix(self, r, p, y):
        cx, sx = math.cos(r), math.sin(r)
        cy, sy = math.cos(p), math.sin(p)
        cz, sz = math.cos(y), math.sin(y)
        
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        
        return Rz @ Ry @ Rx

    def axis_angle_to_matrix(self, axis, q):
        axis = np.array(axis)
        axis = axis / np.linalg.norm(axis)
        kx, ky, kz = axis
        K = np.array([
            [0, -kz, ky],
            [kz, 0, -kx],
            [-ky, kx, 0]
        ])
        I = np.eye(3)
        R = I + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)
        return R

    def get_joint_transform(self, xyz, rpy, axis, q):
        T = np.eye(4)
        R_fixed = self.rpy_to_matrix(*rpy)
        R_joint = self.axis_angle_to_matrix(axis, q)
        
        T[:3, :3] = R_fixed @ R_joint
        T[:3, 3] = xyz
        return T

    def calculate_workspace(self):
        points = []
        
        # Joint 1: base_joint
        xyz_1  = [0.031381, -0.48621, 0.64846]
        rpy_1  = [1.4637, 1.364, 1.4753]
        axis_1 = [0.99932, -0.031099, -0.019999]
        q1_range = np.linspace(-0.7121, 1.0226, 30)
        
        # Joint 2: shoulder_joint
        xyz_2  = [-0.049966, 0.001555, 0.00099995]
        rpy_2  = [-1.376255, 0.017904, 1.358142]
        axis_2 = [0, -1, 0]
        q2_range = np.linspace(-0.2366, 1.3435, 30)

        # Joint 3: elbow_joint
        xyz_3  = [-0.014232, 0, -0.21358]
        rpy_3  = [-1.570821, 1.51637, -0.000021]
        axis_3 = [0, 0, -1]
        q3_range = np.linspace(-0.2336, 1.7762, 30)

        # === TẠO MA TRẬN OFFSET CHO END-EFFECTOR ===
        T_offset = np.eye(4)
        T_offset[0, 3] = 0.33226  # Offset X
        T_offset[1, 3] = 0.03092  # Offset Y
        T_offset[2, 3] = 0.13012  # Offset Z

        for q1 in q1_range:
            T1 = self.get_joint_transform(xyz_1, rpy_1, axis_1, q1)
            for q2 in q2_range:
                T2 = self.get_joint_transform(xyz_2, rpy_2, axis_2, q2)
                T12 = T1 @ T2 
                for q3 in q3_range:
                    T3 = self.get_joint_transform(xyz_3, rpy_3, axis_3, q3)
                    
                    # CỘNG DỒN OFFSET VÀO CUỐI CHUỖI NHÂN MA TRẬN
                    T_EE = T12 @ T3 @ T_offset
                    
                    x, y, z = T_EE[0, 3], T_EE[1, 3], T_EE[2, 3]
                    points.append([x, y, z])
                    
        return points

    def timer_callback(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        pc_msg = pc2.create_cloud_xyz32(header, self.points)
        self.publisher_.publish(pc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GimArmWorkspace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()