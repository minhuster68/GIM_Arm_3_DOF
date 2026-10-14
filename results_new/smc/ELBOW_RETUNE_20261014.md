# Tune lại SMC elbow — 14/10/2026

Bộ chốt cho elbow/id2: `lambda=18`, `ks=30`, `kr=1`, `phi=0.5`,
`torque_slew_rate=6 Nm/s`, `tau_scale=1`. Hai khớp còn lại giữ position.
Bộ số đã được ghi vào `src/gim_arm_controller_smc/config/smc_joint_tuning.yaml`
và `results_new/smc/params_tuned.yaml`.

| Bài 10°, đi/về 5 s, giữ 2 s | RMS sai số vị trí | Sai số lớn nhất | Dao động nhanh torque khi đi/về |
|---|---:|---:|---:|
| Slew 7 (bộ cũ) | 0,273° | 0,743° | 0,0682 / 0,0640 Nm |
| Slew 6, lượt 1 | 0,276° | 0,634° | 0,0571 / 0,0556 Nm |
| Slew 6, lượt 2 | 0,299° | 0,705° | 0,0571 / 0,0537 Nm |

Dao động nhanh được đo bằng RMS của torque sau khi trừ trung bình trượt
11 mẫu ở tần số log 100 Hz. Nó giảm khoảng 13–16% ở pha đi/về;
sai số vị trí RMS tăng tối đa 0,026° trong hai lượt xác nhận.
Lượt 20° (đi/về 6 s, giữ 3 s) với slew 6 đạt RMS 0,297°,
sai số lớn nhất 0,932°, không chạm trần torque.

Các log chính:

- Bộ cũ: `joint_trials/smc_elbow_20261014_193220_271958/`
- Slew 6, 10° lượt 1: `joint_trials/smc_elbow_20261014_201208_132894/`
- Slew 6, 10° lượt 2: `joint_trials/smc_elbow_20261014_201306_593406/`
- Slew 6, 20°: `joint_trials/smc_elbow_20261014_201359_162936/`

Đã thử thay `ks`, tăng slew lên 10, giảm slew xuống 5, và lọc vận tốc
8/20 Hz. Các phương án đó không cải thiện đồng thời độ bám và độ rung
trong các lượt thử này. LPF thử nghiệm đã được loại khỏi mã và cấu hình.
Trước mỗi lượt so sánh, dùng position mode đưa cả ba khớp về 0° để tránh
HOME của lượt sau bị lệch theo sai số còn lại của lượt trước.
