# Quỹ đạo vuông cho matlab-sim

File: `trajectory_new.mat` (MAT-file v5, MATLAB đọc trực tiếp bằng `load`).

Đã đối chiếu repo https://github.com/minhuster68/matlab-sim:

- `main`: commit `7660b885dc2d24046bb5d68d2108a23ef769710f`.
- `eso`: commit `96207e1a184398f12043e126ee62ed7662ce76e0`.
- `position-tracking`: commit `7323b36bf4997fc4b96b92c2640bfec968a165cd`.

Các file `urdf_PID/trajectory_new.mat`, `urdf_LQR/trajectory_new.mat`,
`urdf_MPC/trajectory_new.mat`, `urdf_SMC/trajectory_new.mat` có cùng nội dung
trên cả ba branch (Git blob `1f838a502a0f917a0c015129bfbd19661ae58012`).
Script `setup_pid.m`, `setup_lqr.m`, `setup_mpc.m`, `setup_smc.m` ở main/eso
đều đọc `t`, `q`, `qd`, `qdd`, rồi tạo `ts_q`, `ts_qd`, `ts_qdd`.

## Dữ liệu

| Biến | Kích thước | Ý nghĩa / đơn vị |
|---|---|---|
| `t` | 3701 × 1 double | Thời gian 0–37 s, bước 0.01 s |
| `q` | 3701 × 3 double | Góc mong muốn, rad |
| `qd` | 3701 × 3 double | Vận tốc mong muốn, rad/s |
| `qdd` | 3701 × 3 double | Gia tốc mong muốn, rad/s² |
| `segment` | 3701 × 1 uint8 | 0=tiếp cận, 1=vẽ vuông, 2=về HOME |
| `joint_names` | 1 × 3 cell | base_joint, shoulder_joint, elbow_joint |
| `initial_q` | 1 × 3 double | [0 0 0] |
| `cartesian_waypoints` | 369 × 3 double | Các điểm IK của đường vuông trong world URDF Gazebo, m |
| `sample_dt` | 1 × 1 double | 0.01 s |
| `transition_time` | 1 × 1 double | 5 s |
| `sweep_time` | 1 × 1 double | 27 s |
| `return_time` | 1 × 1 double | 5 s |
| `trajectory_shape` | char | square |

Các cột của q/qd/qdd lần lượt là base, shoulder, elbow. Đạo hàm được tính
giải tích từ cùng profile Gazebo; không lấy sai phân từ góc đã lấy mẫu.
Phần vẽ vuông là t=5…32 s, trùng q/qd/qdd trong `../square_reference.csv`.
HOME đầu/cuối đều [0 0 0], vận tốc và gia tốc bằng 0 ở các điểm chuyển pha.

## Đưa vào mô phỏng

1. Sao lưu `trajectory_new.mat` cũ trong thư mục thuật toán đang dùng.
2. Chép file `trajectory_new.mat` ở đây vào thư mục đó, giữ đúng tên.
   Cùng một file dùng cho PID, LQR, MPC, SMC và các model ESO.
3. Chạy lại script `setup_*.m` tương ứng để tạo lại các timeseries/gain;
   không chỉ `load` sau khi đã chạy setup vì workspace có thể còn quỹ đạo cũ.
4. Kiểm tra Stop Time = 37 s rồi chạy model bình thường.

Ví dụ MATLAB, Current Folder đang là `urdf_PID`:

```matlab
whos('-file', 'trajectory_new.mat')
setup_pid
set_param('urdf', 'StopTime', num2str(t_full(end)))
out = sim('urdf');
plot_trajectory
```

Với ESO, chọn đúng tên model của thuật toán, ví dụ `urdf_eso` cho PID,
`urdf_lqr_eso`, `urdf_mpc_eso`, `urdf_smc_eso`.

**Quy ước dấu:** file giữ dấu của URDF Gazebo, giống file trajectory_new.mat
cũ. Các setup MATLAB đã tự đảo dấu cột 2/3 của cả q/qd/qdd; không đảo thêm
trong file này. Không dùng trực tiếp q thô vào model rồi bỏ qua setup.

**Hình dạng EE:** đây là cùng quỹ đạo khớp đã xem trong Gazebo. URDF và
tool offset trong repo MATLAB khác bản Gazebo, nên đường EE của MATLAB
có thể hơi khác hình vuông 20 × 20 cm. Kiểm tra FK bằng URDF MATLAB hiện tại
và tool offset của setup cho dải x khoảng 20.1 cm, z khoảng 18.0 cm ở pha vẽ.

## Kiểm tra đã làm

- Load lại MAT bằng SciPy: đúng biến, kiểu dữ liệu và kích thước như file cũ
  (ngoại trừ số waypoint thay đổi theo hình và thêm tên hình).
- Thời gian tăng đều; mọi giá trị hữu hạn; đầu/cuối và điểm nối đúng q/qd/qdd.
- TRACK khớp CSV Gazebo tới sai số số học dưới 1e-12.
- Chưa chạy Simulink/MATLAB trên máy này.
