# SMC base: tune ngày 14/10/2026

Base (ID0) chạy torque, shoulder và elbow giữ position. Mỗi lượt bắt đầu gần home 0 rad. Các tham số giữ cố định: `lambda_gain=15.7`, `kr=3`, `base_ff_alpha=2.46`, `tau_scale=1`, vòng điều khiển 100 Hz. `phi=0.2` trừ lượt ghi rõ 0.4.

| Lượt | Quỹ đạo | ks | Slew (Nm/s) | Phi | RMS sai số khi đi/về | Sai số đỉnh | Ripple góc¹ | Kết quả |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| `203825_731829` | 3°, 6 s | 85 | 6.5 | 0.2 | 0.176° | 0.442° | 0.018° | Hoàn thành |
| `203915_991675` | 10°, 6 s | 85 | 6.5 | 0.2 | 0.176° | 0.450° | 0.036° | Hoàn thành |
| `204017_351623` | 20°, 8 s | 85 | 6.5 | 0.2 | 0.343° | 2.374° | — | Tuner ngắt ở pha đi lên vì dao động tăng |
| `204144_202064` | 10°, 6 s | 85 | 10 | 0.2 | 0.187° | 0.800° | 0.061° | Hoàn thành; home ban đầu lệch khoảng -0.5° |
| `204252_340990` | 20°, 8 s | 85 | 10 | 0.2 | 0.195° | 0.671° | 0.069° | Hoàn thành, rung rõ hơn |
| `204358_361750` | 10°, 6 s | 85 | 10 | 0.4 | 0.199° | 0.628° | 0.070° | Tăng phi không giảm ripple |
| `204447_112157` | 10°, 6 s | 65 | 10 | 0.2 | 0.211° | 0.559° | 0.033° | Có cảnh báo thiếu state, dùng để tham khảo |
| `204551_191452` | 10°, 6 s | **65** | **10** | 0.2 | **0.218°** | **0.515°** | **0.035°** | Lặp lại, không có cảnh báo thiếu state |
| `204638_891473` | 20°, 8 s | **65** | **10** | 0.2 | **0.211°** | **0.513°** | **0.060°** | Hoàn thành |
| `204736_821404` | 20°, 8 s | 75 | 10 | 0.2 | 0.195° | 0.505° | 0.066° | Bám tốt hơn, rung hơn |
| `204840_161352` | 10°, 6 s | 75 | 10 | 0.2 | 0.227° | 0.819° | 0.079° | Có cảnh báo thiếu state; không dùng để chốt |

¹ RMS của góc sau khi trừ trung bình trượt 11 mẫu (khoảng 0.11 s), chỉ tính pha APPROACH và RETURN. Mỗi mã lượt tương ứng thư mục `joint_trials/smc_base_20261014_<mã>/` có CSV, YAML và sáu đồ thị.

**Bộ base được chọn:** `lambda_gain=15.7`, `ks=65`, `kr=3`, `phi=0.2`, `torque_slew_rate=10 Nm/s`, `base_ff_alpha=2.46`. So với lượt `ks=85, slew=10` ở 10° (có sai lệch home ban đầu), ripple góc giảm từ 0.061° xuống 0.035° (khoảng 43%), đổi lại RMS sai số khi di chuyển tăng từ 0.187° lên 0.218°. Bài 20° hoàn thành với đỉnh sai số 0.513°, không bão hòa torque. Ở `slew=6.5`, bài 20° không ổn định và tuner đã ngắt khi sai số vượt 2°; không dùng lại mức slew này cho bài base nhanh/biên độ lớn.

Lượt `ks=65` đầu tiên có khoảng 100 lần controller phải giữ torque cũ vì chưa có `/joint_states` mới. Sau đó `/joint_states` đo được xấp xỉ 100 Hz, CAN không báo packet lỗi; lượt lặp cùng bộ số không có cảnh báo. Lượt `ks=75` ở 10° cũng có cảnh báo thiếu state và không được dùng làm bằng chứng chính để chọn gain.

Bộ số base đã ghi vào `src/gim_arm_controller_smc/config/smc_joint_tuning.yaml` và `results_new/smc/params_tuned.yaml`. Profile circle trong `params_tuned.yaml` là `[ks: 65,44,30]`, `[slew: 10,25,6]` theo thứ tự base, shoulder, elbow; chưa chạy lại circle với cả ba khớp đã tune.

File tuning trong working tree có thay đổi shoulder slew=48 và elbow slew=7 từ trước lượt base, khác profile circle đã chốt shoulder=25 và elbow=6. Chỉ thay tham số base trong file tuning; không sao chép nguyên file này vào `params_tuned.yaml`.
