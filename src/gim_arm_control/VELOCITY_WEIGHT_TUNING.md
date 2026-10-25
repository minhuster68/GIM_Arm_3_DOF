# Trọng số vận tốc và đánh giá bám vị trí

Profile phần cứng LQR/MPC dùng trạng thái `[integral(e), e, ed]`, với
`e = q - q_ref`, `ed = qd - qd_ref`. Hàm chi phí phạt cả ba thành phần.
Đánh giá tracking dùng RMS và max |e| theo độ; torque/saturation là thông
tin chẩn đoán. Không gộp sai số vận tốc vào điểm đánh giá vị trí.

## LQR — branch lqr-test

`lqr_joint_tuning.yaml` dùng `position_tracking_only: false`:

```text
QI = 1 / max_int_e²
QE = 1 / max_e²
QV = 1 / max_de²
R  = tau_penalty_scale / max_tau²
```

Bộ LQR tune riêng từng khớp ngày 2026-10-24 dùng
`max_de: [1, 0.7071067811865475, 1]` rad/s cho QV = [1, 2, 1].
Bộ này chưa đạt kiểm tra mô hình khi cả ba khớp cùng dùng torque.
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

Các số MPC trên là baseline; LQR dùng bộ đã tune đơn khớp nêu ở trên. Các profile mô phỏng
đối chiếu MATLAB riêng giữ nguyên để tái lập kết quả cũ. Giới hạn torque,
điều kiện đứng yên trước quỹ đạo và ngưỡng abort vị trí giữ nguyên.
