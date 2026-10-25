# ESO cho tay thật — các branch thử nghiệm riêng

Nguồn đối chiếu: [minhuster68/matlab-sim, branch eso](https://github.com/minhuster68/matlab-sim/tree/eso),
commit `96207e1a184398f12043e126ee62ed7662ce76e0` (đọc ngày 2026-10-25).
Đã đọc các MATLAB Function và dây nối từ XML trong bốn file Simulink:
`urdf_PID/urdf_eso.slx`, `urdf_LQR/urdf_lqr_eso.slx`,
`urdf_MPC/urdf_mpc_eso.slx`, `urdf_SMC/urdf_smc_eso.slx`.
Không có MATLAB/Simulink trong môi trường hiện tại: đây là port phương trình
và kiểm tra Python/ROS, không phải tuyên bố đã chạy lại mô phỏng MATLAB.

| Branch ESO | Tạo từ | YAML trong `src/gim_arm_controller_<algo>/config/` |
|---|---|---|
| `pid-eso-test` | `pid-test` | `pid_eso_hardware.yaml` |
| `lqr-eso-test` | `lqr-test` | `lqr_eso_hardware.yaml` |
| `mpc-eso-test` | `mpc-test` | `mpc_eso_hardware.yaml` |
| `smc-eso-test` | `smc-test` | `smc_eso_hardware.yaml` |

Mỗi branch xuất phát từ bộ gain và Kalman hiện có của đúng thuật toán.
Không tạo worktree mới. Các branch cơ sở được giữ nguyên. Commit ESO chỉ
nằm local cho tới khi có yêu cầu push. Trong mỗi branch chỉ có YAML ESO của
thuật toán tương ứng; bảng này mô tả cả bốn branch, không phải bốn YAML cùng branch.

## Phương trình và đường tín hiệu

ESO là LESO ba trạng thái cho mỗi khớp: `q_hat, v_hat, d_hat`.
Đo đầu vào **góc thật q**, không đưa quỹ đạo tham chiếu vào observer.
Nhiễu `d_hat` có đơn vị rad/s², không phải Nm.

```text
a_nom[k-1] = solve(M(q[k-1]), tau_applied[k-1] - C(q[k-1],v_hat)*v_hat - G(q[k-1]))
q_pred = q_hat_old + dt*v_hat_old + 0.5*dt²*(a_nom_previous + d_hat_old)
v_pred = v_hat_old + dt*(a_nom_previous + d_hat_old)
innovation = q_measured - q_pred
q_hat = q_pred + L1*innovation
v_hat = v_pred + L2*innovation
d_hat = d_hat_old + L3*innovation
```

Với từng khớp, `p=exp(-omega*dt)`:
`L1=1-p³`, `L2=1.5*(1-p)²*(1+p)/dt`, `L3=(1-p)³/dt²`.
Đây là công thức trong `setup_*.m` và block `ESO_update`. Trên tay thật dùng
dt của mẫu tiêu thụ (~0.01 s ở 100 Hz), không bê nguyên gain thiết kế 1 ms
của MATLAB PID/LQR/SMC. Mẫu trùng không cập nhật lại; mất mẫu trên 50 ms hoặc
timestamp quay ngược sẽ reset observer và khởi động lại ramp.

```text
Nhánh FF: q_ref, qd_ref, qdd_ref -> nghịch động lực học -> tau_ff
Nhánh FB: sai số góc + vận tốc Kalman hiện có -> thuật toán -> tau_fb
Nhánh ESO: góc đo + torque đã gửi sau giới hạn -> LESO -> d_hat
tau_dist_hat = M(q_measured) @ d_hat
tau_eso = giới hạn/ramp/slew(-gain * tau_dist_hat)
tau_cmd = giới hạn torque/slew cuối(tau_base + tau_eso)
```

`tau_base` là output hiện có của PID/LQR/MPC/SMC, đã qua giới hạn nội bộ của
thuật toán. Port ghép ESO bên ngoài như sơ đồ MATLAB, không thay thế bộ điều
khiển hoặc tối ưu lại Q/R. `v_hat` chỉ dùng trong mô hình nội bộ observer;
**vận tốc Kalman vẫn đi vào phản hồi**. Góc đo vẫn đi vào P/I, LQR/MPC và mặt trượt.
FF vẫn theo tham chiếu; việc observer dùng G(q đo) không sửa nhánh FF.

Observer dùng **ma trận M đầy đủ** như block `Disturbance_torque`, dù feedback
của controller độc lập. Đây là đổi nhiễu gia tốc sang torque nhiễu của plant,
không đưa lại sai số khớp khác vào luật phản hồi đã tune.

MPC giữ nguyên QP/prediction model. ESO là phần bù ngoài QP giống mô hình
MATLAB, nên ràng buộc dự đoán của QP không chứng minh khả thi cho hệ có ESO.
Giới hạn torque cuối vẫn bắt buộc; với thuật toán có cấu hình slew, adapter
còn giới hạn tốc độ đổi **lệnh cuối** theo rate đó. Khi giới hạn ngoài chặn
lệnh, adapter đóng băng tích phân của các khớp bị chặn và cập nhật lại lịch
sử slew/fallback theo lệnh thực gửi. Phải đánh giá lại tracking/solver trên
các nhánh mới; các kiểm tra ổn định LQR cũ không chứng minh hệ LQR+ESO ổn định.

## Đồng bộ với alpha feedforward trên tay thật

MATLAB dùng mô hình URDF nguyên bản (scale=1). Các bộ phần cứng hiện tại đã
calibrate base bằng `base_ff_alpha`. Nếu observer vẫn coi input/model có
scale=1, nó có thể xem chính phần hiệu chỉnh đã biết này là nhiễu rồi bù lặp.
Profile ESO vì vậy đặt `eso_nominal_input_scale=[base_ff_alpha,1,1]`.
Với S=diag(scale), mô hình ESO thực tế là:

```text
M_nom = S @ M(q)
N_nom = S @ inverse_dynamics(q,v_hat,0)
a_nom = solve(M(q), tau_applied/scale - inverse_dynamics(q,v_hat,0))
tau_dist_hat = scale * (M(q) @ d_hat)
```

Controller/FF không đổi; chỉ đồng bộ nominal model của observer với calibration
đã có. Đây là giả thiết hiệu chỉnh gain đầu vào, chưa phải nhận dạng động lực học
thực nghiệm đầy đủ. Nếu thay alpha sau này cần đồng bộ scale trong YAML ESO.
Đặt scale=[1.0,1.0,1.0] để đối chiếu nguyên bản MATLAB; không tự đổi alpha FF.

## Tham số ESO ban đầu

```yaml
eso_enabled: true
eso_bandwidth_rad_s: [15.0, 15.0, 15.0]  # rad/s, khoảng 2.39 Hz; không phải 15 Hz
eso_compensation_gain: [1.0, 1.0, 1.0]
eso_nominal_input_scale: [2.2, 1.0, 1.0] # LQR/MPC; PID=2.0, SMC=2.46 ở phần tử base
eso_compensation_limit_nm: [0.5, 2.0, 0.3]
eso_compensation_rate_nm_s: [0.5, 2.0, 0.3]
eso_warmup_s: 1.5
eso_ramp_s: 2.0
eso_reset_gap_s: 0.05
```

Đây là cấu hình thử ban đầu, chưa tune trên tay thật. `omega=15 rad/s` lấy
từ MATLAB. Các giới hạn/ramp là bổ sung cho phần cứng; không thay giới hạn
torque động cơ hiện có. Tăng omega giúp bám nhiễu nhanh hơn nhưng nhạy với
nhiễu vị trí hơn. Giới hạn bù nhỏ có thể không bù hết tải/ma sát; tăng nó
làm ảnh hưởng ESO lên hệ lớn hơn. Chưa thay gain controller cùng lúc.

ESO chỉ bật sau khi đọc `/controller_manager/list_controllers` thấy effort
active, position inactive, và `robot_description` xác nhận **torque_joint=all**.
Query là async, không chặn control tick. Khi chưa xác nhận, `tau_eso=0` và
controller gốc vẫn chạy. Khi controller/description không còn mới, ESO reset.
Không dùng profile ESO cho bài tune một khớp với hai khớp giữ position: torque
đã publish của các khớp đó không phải torque áp dụng. Cấu hình như vậy không
được gate cho bù ESO.

Sau switch torque, chờ ít nhất `max(eso_warmup_s, hardware_handoff+0.2)` rồi
mới ước lượng/bù, tiếp theo ramp 2 s. Nhờ vậy không học nhầm torque đang blend
với position hold. Lần seed đầu/gap seed q_hat bằng góc đo và v_hat bằng vận tốc
driver (MATLAB bắt đầu đứng yên nên seed v=0). Torque dùng trong a_nom là lệnh
cuối sau ESO, clip/slew và heartbeat. Không lấy `tau_ff` hoặc torque chưa kẹp
để giả là torque thật. Đây vẫn là lệnh gửi, không đo dòng thực của motor;
độ trễ/tracking của driver và sai mô hình vẫn cần kiểm chứng trên phần cứng.

## Build

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gim_control gim_arm_controller_mpc
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
```

## Chạy vòng tròn

Chỉ giữ một launch phần cứng với `torque_joint:=all` và một node thuật toán.
Khi position controller đang giữ HOME, chạy:

```bash
ros2 launch gim_control mpc_sweep_hardware.launch.py \
  trajectory_shape:=circle approach_time:=16 return_time:=16 \
  params_file:=src/gim_arm_controller_mpc/config/mpc_eso_hardware.yaml \
  log_file:=results_new/mpc/eso/circle_run01.csv
```

Sau READY, chuyển controller thành công rồi bắt đầu:

```bash
ros2 control switch_controllers --strict --activate-asap \
  --deactivate gim_arm_group_controller --activate gim_arm_effort_controller
ros2 param set /mpc_controller autostart true
```

Kết thúc: chuyển về position trước khi Ctrl+C node thuật toán để đóng log.

```bash
ros2 control switch_controllers --strict --activate-asap \
  --deactivate gim_arm_effort_controller --activate gim_arm_group_controller
```


## Log và đồ thị

```bash
ros2 run gim_control plot_joint_tracking results_new/mpc/eso/circle_run01.csv --show
ros2 run gim_control plot_eso results_new/mpc/eso/circle_run01.csv --show
```

Plot ESO tách riêng ba PNG, mỗi khớp có q_hat/góc đo, v_hat/vận tốc driver/Kalman,
và torque nhiễu/bù/lệnh cuối. Hình tracking thường cũng thêm các đường
`tau_pre_eso`, `tau_eso`, `tau_eso_actuator_adjustment` vào panel torque.
Cột `eso_d_hat_*` là rad/s²;
`tau_dist_hat_*` và `tau_eso_*` là Nm. `eso_status` cho biết gate/warmup/active.
`tau_pre_eso + tau_eso + tau_eso_actuator_adjustment = tau` (sai khác heartbeat
cỡ 1e-6 Nm). Các cột `tau_ff`, `tau_fb`, `tau_raw` cũ vẫn mô tả controller
gốc trước lớp ESO; không kết luận tau gửi sai công thức nếu bỏ qua các cột mới.

Tắt riêng bù bằng `eso_compensation_gain: [0.0,0.0,0.0]` để quan sát d_hat, hoặc
`eso_enabled: false` để bỏ toàn bộ lớp ESO. Restart node để nạp cấu hình mới.
Gain 0 vẫn giữ lớp giới hạn lệnh cuối; để so đúng baseline gốc dùng `eso_enabled: false`.
