# SMC shoulder: tune ngày 14/10/2026

Khớp shoulder (ID1) chạy torque, base và elbow giữ position. Mỗi lượt bắt đầu gần home 0 rad, dùng `tau_scale=1`, `lambda_gain=20`, `ks=44`, `kr=4.2` và `phi=0.15` trừ lượt ghi rõ `phi=0.30`. Tốc độ vòng điều khiển là 100 Hz.

| Lượt | Quỹ đạo | Slew shoulder | Phi | RMS sai số khi đi/về | Sai số đỉnh khi đi/về | Ripple góc¹ | Ripple torque¹ | Kết quả |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| `202424_078909` | 3°, 6 s | 48 | 0.15 | 0.205° | 0.416° | 0.021° | 0.289 Nm | Chuẩn ban đầu; torque đổi gắt |
| `202510_048413` | 3°, 6 s | 20 | 0.15 | 0.216° | 0.460° | 0.014° | 0.110 Nm | Ít rung hơn ở biên độ nhỏ |
| `202602_098561` | 10°, 6 s | 20 | 0.15 | 0.236° | 0.930° | 0.030° | 0.194 Nm | Cú hụt khi về |
| `202648_828068` | 10°, 6 s | 20 | 0.30 | 0.260° | 0.546° | 0.028° | 0.202 Nm | Đỡ hụt nhưng RMS tăng |
| `202737_838689` | 10°, 6 s | 12 | 0.15 | 2.738° | 13.978° | — | — | Mất ổn định trong RETURN; ABORT |
| `203126_238072` | 10°, 6 s | 30 | 0.15 | 0.217° | 0.436° | 0.033° | 0.297 Nm | Bám tốt nhưng torque rung hơn |
| `203223_008135` | 10°, 6 s | **25** | 0.15 | **0.221°** | **0.489°** | **0.030°** | **0.259 Nm** | Cân bằng bám/rung |
| `203311_008539` | 15°, 8 s | **25** | 0.15 | **0.227°** | **0.587°** | **0.034°** | **0.248 Nm** | Hoàn thành, không ABORT |

¹ RMS của phần tín hiệu sau khi trừ trung bình trượt 11 mẫu (khoảng 0,11 s), chỉ tính pha APPROACH và RETURN. Chỉ số này phản ánh dao động nhanh theo cách đo đã dùng ở đợt tune elbow; nó không thay thế đánh giá bằng mắt và tai. Mỗi mã lượt tương ứng thư mục `joint_trials/smc_shoulder_20261014_<mã>/` có CSV, YAML và đồ thị.

**Bộ shoulder được chọn:** `lambda_gain=20`, `ks=44`, `kr=4.2`, `phi=0.15`, `torque_slew_rate=25 Nm/s`. Đã ghi vào `src/gim_arm_controller_smc/config/smc_joint_tuning.yaml` và `results_new/smc/params_tuned.yaml`. Bộ này đã qua bài 10° và 15°; chưa xác nhận bằng quỹ đạo circle sau khi tune lại.

**Sự cố ở slew 12:** Trong pha RETURN, khớp dao động lớn rồi controller báo gần giới hạn và ABORT. Sau khi trả về position mode, mẫu `/joint_states` đo được shoulder khoảng -0,347 rad, thấp hơn giới hạn URDF -0,2366 rad. Lệnh position 12 s đã đưa cả ba khớp về 0 thành công; người giám sát không thấy va chạm, tiếng lạ, nóng bất thường hay sai zero. Không dùng lại slew 12 cho shoulder. Torque shoulder cực đại trong log là 3,86 Nm, nên đây không phải bão hòa ở trần 40 Nm.

Tuner hiện có ngắt riêng khi khớp đang tune lệch quỹ đạo >2° hoặc khớp cố định lệch home >1°, nhằm trả về position trước khi đáp ứng phân kỳ tiến gần giới hạn. Ngắt này chỉ áp dụng cho bài tune từng khớp; cấu hình chạy circle vẫn giữ `abort_on_tracking_error=false` theo yêu cầu. Chín bài kiểm tra ROS giả lập cho tuner đã qua trên domain cô lập.

Thay đổi elbow slew=7 trong file tuning là thay đổi có sẵn trong working tree trước lượt shoulder này; bộ elbow đã chốt và `params_tuned.yaml` dùng slew=6. Không sao chép toàn bộ file tuning sang profile circle khi hai giá trị elbow còn khác nhau.
