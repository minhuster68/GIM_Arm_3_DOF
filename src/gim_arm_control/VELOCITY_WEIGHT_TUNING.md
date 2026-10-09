# Trọng số vận tốc và đánh giá bám vị trí

Profile phần cứng LQR/MPC dùng trạng thái `[integral(e), e, ed]`, với
`e = q - q_ref`, `ed = qd - qd_ref`. Hàm chi phí phạt cả ba thành phần.
Đánh giá tracking dùng RMS và max |e| theo độ; torque/saturation là thông
tin chẩn đoán. Không gộp sai số vận tốc vào điểm đánh giá vị trí.

## LQR — branch lqr-test

`lqr_hardware_soft.yaml` dùng `position_tracking_only: false`:

```text
QI = 1 / max_int_e²
QE = 1 / max_e²
QV = 1 / max_de²
R  = tau_penalty_scale / max_tau²
```

Baseline `max_de: [2, 2, 2]` rad/s cho QV = [0.25, 0.25, 0.25].
Giảm max_de làm tăng mức phạt sai số vận tốc. Đây là thang trọng số,
không phải giới hạn vận tốc cứng. Trong terminal quỹ đạo đầy đủ:

```text
set max_de 2 2 2
show
save
```

## MPC — branch mpc-test

`mpc_hardware_soft.yaml` dùng `q_velocity: [0.02, 0.02, 0.02]` và
`output_scale_velocity: [0.1, 0.1, 0.1]` rad/s. Với profile
`weight_convention: matlab_squared`, trọng số thực trong Q là:

```text
QV = (q_velocity / output_scale_velocity)² = [0.04, 0.04, 0.04]
```

Tăng q_velocity hoặc giảm output_scale_velocity tăng mức phạt sai số
vận tốc. Trong terminal `python3 tools/tune_mpc.py`:

```text
set q_velocity 0.02 0.02 0.02
set output_scale_velocity 0.1 0.1 0.1
show
save
```

Các số trên là baseline, chưa tune trên tay thật. Các profile mô phỏng
đối chiếu MATLAB riêng giữ nguyên để tái lập kết quả cũ. Giới hạn torque,
điều kiện đứng yên trước quỹ đạo và ngưỡng abort vị trí giữ nguyên.
