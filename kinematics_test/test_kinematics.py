"""
test_kinematics.py — chạy thử để HIỂU từng hàm của GimArmKinematics làm gì.

Cách chạy: đặt file này CÙNG thư mục với gim_arm_kinematics.py và gim_arm.urdf,
rồi chạy: python3 test_kinematics.py
"""

import numpy as np
from gim_arm_kinematics import GimArmKinematics

# ---------------------------------------------------------------------------
# BƯỚC 1: Khởi tạo bộ "dịch" -- đọc URDF 1 lần, giữ trong biến `kin` để dùng lại.
# tool_offset_xyz: khoảng lệch từ điểm nối elbow tới đầu bút thật (đã tính ở
# lượt trước từ file STL: ~(0.4031, 0.049, -0.029)). THAY BẰNG SỐ THẬT của bạn.
# ---------------------------------------------------------------------------
kin = GimArmKinematics("gim_arm.urdf", tool_offset_xyz=(0.4031, 0.049, -0.029))
print("Tên 3 khớp, đúng thứ tự dùng trong mọi hàm dưới đây:", kin.joint_names)
print()


# ---------------------------------------------------------------------------
# BƯỚC 2: FK -- "biết góc khớp, hỏi đầu bút đang ở đâu"
# ---------------------------------------------------------------------------
# Giá trị cũ [0.5, 0.3, -0.4] KHÔNG dùng được nữa: sau khi đo lại giới hạn khớp
# bằng encoder (19/08/2026), elbow_joint chỉ còn xuống tới -0.2336 rad, nên -0.4
# nằm ngoài giới hạn -> IK/FK ra số vô nghĩa. Số dưới đây lấy từ ĐIỂM ĐẦU của
# quỹ đạo quét thật trong sweep_trajectory.py, nên là tư thế tay thật sự đi qua.
q_test = [0.86, 0.86, 0.95]  # radian, đúng thứ tự [base, shoulder, elbow]
vi_tri_dau_but = kin.fk_position(q_test)
print(f"Nếu 3 khớp ở góc {q_test} (rad) -> đầu bút ở vị trí (x,y,z) = {vi_tri_dau_but.round(4)} (mét)")
print()


# ---------------------------------------------------------------------------
# BƯỚC 3: check_singularity -- "ở đây quay được không, hay tay máy sẽ bị kẹt/yếu"
# Dùng để CHỌN vùng đặt bảng vẽ (task #4) -- tránh những chỗ trả về True.
# ---------------------------------------------------------------------------
la_singular = kin.check_singularity(q_test)
print(f"Ở cấu hình {q_test}, có phải vùng singularity không? -> {la_singular}")
print("(True = tránh, không nên đặt bảng vẽ ở đây; False = an toàn)")
print()


# ---------------------------------------------------------------------------
# BƯỚC 4: IK -- "biết muốn đầu bút ở đâu, hỏi ngược lại góc khớp phải là bao nhiêu"
# Đây chính là chiều NGƯỢC của bước 2.
# ---------------------------------------------------------------------------
# Cũng vậy: [0.25, -0.30, 0.55] giờ nằm NGOÀI vùng với được (IK lệch 190mm).
# Vùng với được là vỏ cầu quanh tâm vai (0.031, -0.538, 0.606), bán kính
# ~0.41..0.59m -- điểm dưới đây nằm gọn trong đó.
target = np.array([0.35, -0.12, 0.55])  # 1 điểm (x,y,z) muốn đầu bút tới, tự đổi số thử
ket_qua = kin.ik_position(target)
print(f"Muốn đầu bút tới {target} -> góc khớp cần thiết (rad) = {ket_qua.q.round(4)}")
print(f"  Có giải được không: {ket_qua.converged}, sai số còn lại: {ket_qua.position_error_m*1000:.4f} mm")
print()


# ---------------------------------------------------------------------------
# BƯỚC 5: solve_trajectory -- "biết cả 1 đường đi (nhiều điểm x,y,z), hỏi ngược
# lại 1 CHUỖI góc khớp tương ứng" -- đây chính là việc task #6 cần làm để vẽ
# chữ O/S: tham số hoá đường vẽ ra chuỗi điểm, rồi đưa vào đây.
# ---------------------------------------------------------------------------
tam_vong_tron = kin.fk_position(q_test)  # dùng lại tư thế hợp lệ ở BƯỚC 2 làm tâm
ban_kinh = 0.03  # 3cm
so_diem = 12
duong_ve = []
for i in range(so_diem + 1):
    goc = 2 * np.pi * i / so_diem
    diem = tam_vong_tron + np.array([ban_kinh * np.cos(goc), ban_kinh * np.sin(goc), 0.0])
    duong_ve.append(diem)

danh_sach_ket_qua = kin.solve_trajectory(duong_ve)
print(f"Đường vẽ có {len(duong_ve)} điểm -> giải ra {len(danh_sach_ket_qua)} bộ góc khớp:")
for i, r in enumerate(danh_sach_ket_qua):
    print(f"  điểm {i}: q(rad)={r.q.round(3)}  hội tụ={r.converged}  sai số={r.position_error_m*1000:.5f}mm")