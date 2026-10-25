# Các profile LQR

| YAML | Mục đích / nơi sử dụng |
|---|---|
| `lqr_joint_tuning.yaml` | Bộ phần cứng vừa tune từng khớp ngày 2026-10-24. Mặc định của terminal tune đơn khớp; hai khớp khác giữ position. |
| `lqr_circle_hardware.yaml` | Circle cả tay: independent/free_torque, R=[15.625,0.625,19.53125], Kalman vận tốc bậc 2 cho ba khớp. Mặc định launch phần cứng. |
| `lqr_gazebo_matlab.yaml` | Mô phỏng Gazebo/MATLAB, 2000 Hz; `tools/view_lqr_r_gazebo.py` sử dụng. |
| `lqr.yaml` | Profile đối chiếu MATLAB; `run_effort_algorithm.launch.py` sử dụng. |
| `lqr_safe_100hz.yaml` | Profile DARE cũ của `run_effort_algorithm.launch.py` khi chọn `lqr_profile:=safe_100hz`. |

Đã xóa `lqr_hardware_soft.yaml` vì profile phần cứng ban đầu không còn là bộ
được chọn. Không dùng các profile mô phỏng thay cho bộ đã tune trên tay thật.

Bộ `lqr_joint_tuning.yaml` đạt các bài cô lập nhưng không đạt mô hình cả tay
(rho khoảng 2,75). Bộ `lqr_circle_hardware.yaml` thiết kế gain SISO từ plant cả tay
tự do. Bộ circle hiện tại qua kiểm tra mô hình cả tay và nội suy có tính động học
Kalman, nhưng không qua kiểm tra thêm một tick trễ 10 ms. Gain do người dùng chọn
được giữ nguyên khi thêm lọc; hiệu quả bộ lọc chưa được xác nhận trên tay thật.
`require_discrete_stable: true` vẫn giữ ở cả hai bộ. Chỉ chuyển effort khi node báo READY.

Tham số lọc, quy trình circle và phạm vi kiểm chứng:
[README_LQR_KALMAN.md](../../../README_LQR_KALMAN.md).
