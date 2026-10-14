# Tune lại SMC: elbow → shoulder → base

Branch hiện tại: `smc-test`. Không cần đổi branch hoặc tạo worktree.

- File tuner ghi khi `save`: `src/gim_arm_controller_smc/config/smc_joint_tuning.yaml`.
- Elbow đã chốt lại với slew 6 Nm/s; xem `ELBOW_RETUNE_20261014.md`.
- Bộ trước khi tune lại: `results_new/smc/params_before_retune_20261013.yaml`.
- Log và 6 PNG mỗi trial: `results_new/smc/joint_trials/smc_<joint>_<timestamp>/`.
- Bộ `results_new/smc/params_tuned.yaml` đã cập nhật elbow=6 và shoulder=25 Nm/s sau các lượt tune ngày 14/10; chưa chạy lại circle với bộ mới.

Tất cả terminal trong quy trình này dùng `ROS_DOMAIN_ID=0`, giống phiên hardware hiện tại. Chạy Python trực tiếp; script `tools/tune_smc_terminal.sh` của branch đang cố định domain 42.

## Trả về position và home trước khi đổi hardware

Node SMC circle đã dừng nhưng effort controller vẫn active tại lúc kiểm tra. Trong terminal khác:

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 control switch_controllers \
  --strict --activate-asap \
  --deactivate gim_arm_effort_controller \
  --activate gim_arm_group_controller
ros2 action send_goal --feedback \
  /gim_arm_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [base_joint, shoulder_joint, elbow_joint], points: [{positions: [0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0], time_from_start: {sec: 8}}]}}"
```

Chờ goal thành công. Ctrl+C launch hardware cũ, đặt/kê tay đúng zero trước khi khởi động lại và chụp mốc.

## Bắt đầu elbow

Terminal 1:

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 \
  set_zero_on_startup:=true zero_method:=software \
  torque_joint:=elbow
```

Chờ position active. Terminal 2:

```bash
cd ~/GIM_Arm_3_DOF
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
python3 tools/tune_smc_joint.py \
  --joint elbow \
  --params-file src/gim_arm_controller_smc/config/smc_joint_tuning.yaml \
  --log-dir results_new/smc/joint_trials
```

Tại prompt SMC:

```text
show
test 3 6 3
```

Đây là baseline, giữ nguyên bộ đang lưu. Chỉ khớp đang chọn nhận torque; hai khớp kia giữ position. Tuner tự chuyển mode, chạy đi/giữ/về/giữ và trả về position. DEG là góc dịch chuyển từ HOME, MOVE là số giây mỗi chiều, HOLD là số giây giữ mỗi đầu.

Xem trial đầu rồi điều chỉnh từng tham số. `set LAMBDA KS KR PHI` đổi bộ số trong bộ nhớ; `slew VALUE` đổi giới hạn biến thiên feedback torque. Gain chỉ áp dụng ở test tiếp theo. `save` mới ghi YAML. Khi thử nhỏ đã đạt, tăng biên độ, ví dụ `test 5 6 3`, rồi `test 10 6 3`. Trước **mỗi** lượt thử, đưa tay về 0 bằng action position ở trên để so sánh trong cùng tư thế HOME.

## Chuyển shoulder rồi base

Khi chốt elbow:

```text
save
next
```

`next` chọn shoulder trong tuner. Chờ position active, đưa về home bằng lệnh action ở trên nếu cần, rồi Ctrl+C hardware. Đặt/kê đúng zero, khởi động lại Terminal 1:

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 \
  set_zero_on_startup:=true zero_method:=software \
  torque_joint:=shoulder
```

Giữ tuner đang mở, chạy `show`, `test 3 6 3`; đánh giá và tune trước khi `save`, `next` sang base.

Với base, làm lại thao tác home/đổi hardware, dùng:

```bash
ros2 launch gim_control origin_gim_arm_control.launch.py \
  can_interface:=can0 \
  set_zero_on_startup:=true zero_method:=software \
  torque_joint:=base
```

Trong tuner chạy `show`, `test 3 6 3`. Chỉ ở base có thể dùng `alpha VALUE` để chỉnh riêng feedforward. Kết thúc bằng `save`, `quit`.

## Bộ hiện tại

| Khớp | lambda | ks | kr | phi | Feedback slew (Nm/s) |
|---|---:|---:|---:|---:|---:|
| elbow/ID2 | 18 | 30 | 1 | 0.5 | 6 |
| shoulder/ID1 | 20 | 44 | 4.2 | 0.15 | 25 |
| base/ID0 | 15.7 | 85 | 3 | 0.2 | 6.5 |

100 Hz, tau_scale=1, base_ff_alpha=2.46. `abort_on_tracking_error=false` giữ nguyên; các kiểm tra khác của tuner/hardware vẫn hoạt động.

Lưu ý: file tuning trong working tree hiện có thay đổi riêng elbow slew=7 chưa commit, trong khi bộ elbow đã chốt và `params_tuned.yaml` dùng slew=6. Không sao chép nguyên file tuning sang `params_tuned.yaml` trước khi quyết định giá trị elbow. Xem `SHOULDER_RETUNE_20261014.md` cho các lượt thử shoulder, kể cả lượt slew=12 bị mất ổn định.

## Sau khi chốt cả ba khớp

Đối chiếu từng khớp trong file tuning với `results_new/smc/params_tuned.yaml` trước khi cập nhật bộ chạy circle. Hiện shoulder đã đồng bộ ở 25 Nm/s, còn elbow trong file tuning là 7 Nm/s trong khi bộ đã chốt dùng 6 Nm/s.

Chạy circle tiếp theo với tên log mới `results_new/smc/circle_run02.csv`. Bộ dùng cho circle_run01 vẫn được lưu ở `params_before_retune_20261013.yaml`. Khi báo cáo bộ mới, dùng tham số đã lưu của từng trial và bộ YAML mới thay vì bảng số của đợt so sánh trước.
