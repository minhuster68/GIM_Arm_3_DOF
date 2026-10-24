# SMC

Bộ phần cứng đã chốt ngày 2026-10-24 nằm duy nhất tại
`config/smc_circle_from_terminal_tuning.yaml`. Dùng super-twisting 100 Hz,
FF nghịch động lực học đầy đủ theo tham chiếu và phản hồi riêng từng khớp
`tau_fb_i=M_ii(q_measured)*a_fb_i`.

Launch `smc_sweep_hardware.launch.py` và terminal tune mặc định cùng dùng
file chốt. Không cần ba YAML riêng cho base, shoulder và elbow. Xem
[quy trình tune](../../README_TUNE_SMC.md) và
[lệnh chạy vòng tròn](../../README_SMC_INDEPENDENT_FEEDBACK.md).

Ba YAML còn lại dành cho các chế độ mô phỏng được launch hỗ trợ:

- `smc_super_twisting_gazebo.yaml`: STA 1 kHz, `smc_profile:=super_twisting`.
- `smc_gazebo_safe.yaml`: saturation 100 Hz, `smc_profile:=gazebo_safe`.
- `smc.yaml`: tham chiếu MATLAB saturation, `smc_profile:=matlab_reference`.

Đó là profile mô phỏng, không phải bộ gain phần cứng đã chốt. Các snapshot và
profile tuning phần cứng cũ đã xóa. Kết quả CSV/PNG vẫn được giữ nguyên.
