# PID Gazebo — chữ R

Hai trường hợp đều hoàn tất APPROACH 5 s, TRACK 27 s, RETURN 5 s và về HOLD.
Tham chiếu giữ đủ q/qd/qdd; chỉ đánh giá sai số vị trí. Gain PID giữ nguyên.
Điểm đầu/cuối TRACK giống vòng tròn. Vận tốc/gia tốc bằng 0 tại các điểm nối nét.

| Tải | RMS vị trí TRACK (độ), base/shoulder/elbow | Max sai số TRACK (độ) |
| --- | --- | --- |
| 0 kg | 0.088 / 0.256 / 0.073 | 0.242 / 0.708 / 0.248 |
| 0.5 kg | 0.400 / 0.300 / 0.303 | 1.116 / 1.138 / 1.531 |

Không tải: tau_scale=0.50; tải 0,5 kg: tau_scale=0.75.
Mỗi trường hợp có 74.003 mẫu; mỗi khớp có đồ thị vị trí, sai số vị trí và mô-men.

```bash
python3 tools/validate_pid_gazebo.py --trajectory-shape r
```

[Ảnh quỹ đạo tham chiếu](../trajectory_shapes/reference_paths.png) · [Log](validation.log)

Đã build 5 package và chạy 41 test đạt (quỹ đạo, PID, LQR, MPC).
Kiểm tra Gazebo trong báo cáo này dùng PID, không suy ra kết quả bám của các bộ điều khiển khác.
