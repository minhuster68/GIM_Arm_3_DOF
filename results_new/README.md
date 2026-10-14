# Kết quả thực nghiệm theo branch

Mỗi branch test chỉ lưu CSV, ảnh và YAML của thuật toán tương ứng, trong `results_new/<thuật_toán>/`. Source điều khiển và gain gốc vẫn ở `src/`. Các tài liệu `RUN_CIRCLE.md`, `TUNED_PARAMETERS.md` và README này có trên cả bốn branch.

| Branch | Thư mục kết quả | CSV đã lưu | Lần chạy tiếp theo |
|---|---|---|---|
| [`pid-test`](https://github.com/minhuster68/GIM_Arm_3_DOF/tree/pid-test/results_new/pid) | `results_new/pid/` | `circle_run03.csv` | `circle_run04.csv` |
| [`lqr-test`](https://github.com/minhuster68/GIM_Arm_3_DOF/tree/lqr-test/results_new/lqr) | `results_new/lqr/` | `circle_run01.csv`, `circle_run01_states.csv` | `circle_run02.csv` |
| [`mpc-test`](https://github.com/minhuster68/GIM_Arm_3_DOF/tree/mpc-test/results_new/mpc) | `results_new/mpc/` | `circle_run01.csv` | `circle_run02.csv` |
| [`smc-test`](https://github.com/minhuster68/GIM_Arm_3_DOF/tree/smc-test/results_new/smc) | `results_new/smc/` | `circle_run01.csv` | `circle_run02.csv` |

PID, LQR và MPC đã lưu cả hình vị trí/torque và hình vận tốc riêng cho từng khớp. SMC hiện có CSV; có thể xuất thêm hình bằng công cụ plot. Mỗi thư mục giữ `params_tuned.yaml` của bài circle. SMC còn có bản sao trước khi tune lại và hướng dẫn `RETUNE.md`.

Bài PID `circle_run03.csv` dùng `pid/circle_run03_params.yaml`; `pid/params_tuned.yaml` là bộ số của các bài trước.

Ví dụ xem kết quả MPC đã chạy, sau khi dừng các node phần cứng/thuật toán của bài trước:

```bash
cd ~/GIM_Arm_3_DOF
git switch mpc-test
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run gim_control plot_joint_tracking results_new/mpc/circle_run01.csv --show
```

Sau khi switch, build lại package tương ứng trước khi chạy thuật toán; xem [RUN_CIRCLE.md](RUN_CIRCLE.md). Xem [TUNED_PARAMETERS.md](TUNED_PARAMETERS.md) để lấy bộ số cho báo cáo.

Với bài mới, chọn tên log chưa tồn tại và commit thư mục của thuật toán đang chạy; ví dụ ở `mpc-test`:

```bash
git branch --show-current
git add results_new/mpc
git commit -m "Save MPC circle run02 results"
git push origin mpc-test
```

Các thư mục dữ liệu đã chạy được chuyển giữa branch bằng nội dung Git nguyên vẹn. Không đổi tên CSV/PNG hoặc tính lại dữ liệu trong lần tổ chức này. Thư mục `results` cũ được bỏ khỏi cả bốn branch test.
