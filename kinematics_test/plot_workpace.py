import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def rpy_to_matrix(r, p, y):
    cx, sx = math.cos(r), math.sin(r)
    cy, sy = math.cos(p), math.sin(p)
    cz, sz = math.cos(y), math.sin(y)
    
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def axis_angle_to_matrix(axis, q):
    axis = np.array(axis)
    axis = axis / np.linalg.norm(axis)
    kx, ky, kz = axis
    K = np.array([
        [0, -kz, ky],
        [kz, 0, -kx],
        [-ky, kx, 0]
    ])
    I = np.eye(3)
    return I + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)

def get_joint_transform(xyz, rpy, axis, q):
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*rpy) @ axis_angle_to_matrix(axis, q)
    T[:3, 3] = xyz
    return T

def calculate_workspace():
    points = []
    
    # Joint 1: base_joint
    xyz_1, rpy_1, axis_1 = [0.031381, -0.48621, 0.64846], [1.4637, 1.364, 1.4753], [0.99932, -0.031099, -0.019999]
    q1_range = np.linspace(-0.7121, 1.0226, 25) # Giảm nhẹ số điểm để matplotlib render mượt hơn
    
    # Joint 2: shoulder_joint
    xyz_2, rpy_2, axis_2 = [-0.049966, 0.001555, 0.00099995], [-1.376255, 0.017904, 1.358142], [0, -1, 0]
    q2_range = np.linspace(-0.2366, 1.3435, 25)

    # Joint 3: elbow_joint
    xyz_3, rpy_3, axis_3 = [-0.014232, 0, -0.21358], [-1.570821, 1.51637, -0.000021], [0, 0, -1]
    q3_range = np.linspace(-0.2336, 1.7762, 25)

    # Offset End-Effector
    T_offset = np.eye(4)
    T_offset[0:3, 3] = [0.33226, 0.03092, 0.13012]

    print("Đang tính toán ma trận, vui lòng đợi...")
    for q1 in q1_range:
        T1 = get_joint_transform(xyz_1, rpy_1, axis_1, q1)
        for q2 in q2_range:
            T12 = T1 @ get_joint_transform(xyz_2, rpy_2, axis_2, q2)
            for q3 in q3_range:
                T3 = get_joint_transform(xyz_3, rpy_3, axis_3, q3)
                
                T_EE = T12 @ T3 @ T_offset
                points.append([T_EE[0, 3], T_EE[1, 3], T_EE[2, 3]])
                
    return np.array(points)

def plot_workspace():
    points = calculate_workspace()
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Vẽ đám mây điểm (màu xanh dương, độ trong suốt alpha=0.2 để nhìn thấu bên trong)
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='blue', s=2, alpha=0.2, label='Workspace')
    
    # Vẽ gốc tọa độ (0,0,0)
    # ax.scatter([0], [0], [0], c='red', s=50, label='Base Origin')
    
    # Vẽ cụm 3 trục tọa độ X, Y, Z tại gốc để dễ định hướng
    # ax.quiver(0, 0, 0, 0.5, 0, 0, color='r', arrow_length_ratio=0.1) # Trục X (Đỏ)
    # ax.quiver(0, 0, 0, 0, 0.5, 0, color='g', arrow_length_ratio=0.1) # Trục Y (Xanh lá)
    # ax.quiver(0, 0, 0, 0, 0, 0.5, color='b', arrow_length_ratio=0.1) # Trục Z (Xanh dương)

    # Thêm chữ X, Y, Z cho các trục
    # ax.text(0.55, 0, 0, 'X', color='red', fontsize=12, fontweight='bold')
    # ax.text(0, 0.55, 0, 'Y', color='green', fontsize=12, fontweight='bold')
    # ax.text(0, 0, 0.55, 'Z', color='blue', fontsize=12, fontweight='bold')

    # Cài đặt hiển thị
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('GIM_Arm_3_DOF Workspace')
    
    # Cân bằng tỉ lệ các trục (Tránh bị méo hình)
    ax.set_box_aspect([1,1,1])
    
    plt.legend()
    plt.show()

if __name__ == '__main__':
    plot_workspace()