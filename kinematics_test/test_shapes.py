"""
test_shapes.py — chạy thử để HIỂU shapes.py sinh ra cái gì, và cách nối nó
với gim_arm_kinematics.py để ra góc khớp thật.

Cách chạy: đặt CÙNG thư mục với shapes.py, gim_arm_kinematics.py, gim_arm.urdf,
rồi chạy: python3 test_shapes.py
"""

import numpy as np
from shapes import letter_o, discretize
from gim_arm_kinematics import GimArmKinematics


# ---------------------------------------------------------------------------
# BƯỚC 1: letter_o(...) không "vẽ" gì cả -- nó trả về 1 HÀM (path_o), giống
# như 1 công thức toán chưa tính ra số. Gọi hàm đó với 1 giá trị t cụ thể
# (từ 0 đến 1) mới ra 1 ĐIỂM (x,y,z) cụ thể.
# ---------------------------------------------------------------------------
path_o = letter_o(center=(-0.045, 0.7), radius=0.07, plane="x", plane_value=0.2)
print("path_o là 1 hàm:", path_o)
print("Gọi path_o(0.0)  ->", path_o(0.0), "(điểm bắt đầu vòng tròn)")
print("Gọi path_o(0.25) ->", path_o(0.25), "(đi được 1/4 vòng)")
print("Gọi path_o(0.5)  ->", path_o(0.5), "(đi được nửa vòng)")
print()


# ---------------------------------------------------------------------------
# BƯỚC 2: discretize() gọi path_o() lặp lại NHIỀU LẦN (ở đây 12 lần cho dễ
# nhìn, thực tế dùng 60 cho mượt) với các giá trị t rải đều 0->1, rồi GOM
# TẤT CẢ các điểm đó lại thành 1 danh sách -- đây chính là "vẽ" ra hình tròn
# bằng cách nối nhiều điểm rời rạc lại.
# ---------------------------------------------------------------------------
positions = discretize(path_o, n_points=12, close_loop=True)
print(f"discretize() trả về danh sách {len(positions)} điểm:")
for i, p in enumerate(positions):
    print(f"  điểm {i}: (x,y,z) = {p.round(4)}")
print()


# ---------------------------------------------------------------------------
# BƯỚC 3: mỗi điểm (x,y,z) ở trên là VỊ TRÍ MONG MUỐN của đầu bút. Muốn động
# cơ chạy tới đó, cần đổi (x,y,z) -> góc khớp bằng IK -- đúng việc
# solve_trajectory() đã làm ở lượt test_kinematics.py trước.
# ---------------------------------------------------------------------------
kin = GimArmKinematics("gim_arm.urdf", tool_offset_xyz=(0.4031, 0.049, -0.029))
ket_qua = kin.solve_trajectory(positions)

print(f"Sau khi giải IK, ra {len(ket_qua)} bộ góc khớp tương ứng:")
for i, r in enumerate(ket_qua):
    print(f"  điểm {i}: q(rad) = {r.q.round(3)}  hội tụ={r.converged}")
print()
print("=> Đây chính là chuỗi góc khớp sẽ đưa vào draw_trajectory.py để gửi")
print("   xuống động cơ thật (thay cho đoạn 'DEMO' vòng tròn tạm trong đó).")