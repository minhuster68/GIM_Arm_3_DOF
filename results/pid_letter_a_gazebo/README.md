# PID Gazebo — chữ A

Hai trường hợp đều hoàn tất APPROACH 5 s, TRACK 27 s, RETURN 5 s và về HOLD.
Tham chiếu giữ đủ q/qd/qdd; chỉ đánh giá sai số vị trí. Gain PID giữ nguyên.
Điểm đầu/cuối TRACK giống vòng tròn. Vận tốc/gia tốc bằng 0 tại các điểm nối nét.

| Tải | RMS vị trí TRACK (độ), base/shoulder/elbow | Max sai số TRACK (độ) |
| --- | --- | --- |
| 0 kg | 0.050 / 0.222 / 0.066 | 0.223 / 0.607 / 0.204 |
| 0.5 kg | 0.172 / 0.210 / 0.286 | 0.495 / 0.926 / 1.529 |

Không tải: tau_scale=0.50; tải 0,5 kg: tau_scale=0.75.
Mỗi trường hợp có 74.003 mẫu; mỗi khớp có đồ thị vị trí, sai số vị trí và mô-men.

```bash
python3 tools/validate_pid_gazebo.py --trajectory-shape a
```

[Ảnh quỹ đạo tham chiếu](../trajectory_shapes/reference_paths.png) · [Log](validation.log)

Đã build 5 package và chạy 41 test đạt (quỹ đạo, PID, LQR, MPC).
Kiểm tra Gazebo trong báo cáo này dùng PID, không suy ra kết quả bám của các bộ điều khiển khác.
