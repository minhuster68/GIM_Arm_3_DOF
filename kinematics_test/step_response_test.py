"""
step_response_test.py — gửi 1 bước nhảy vị trí, ghi lại phản ứng thật, vẽ đồ
thị + tính overshoot/settling time bằng số -- dùng để tune PID bằng đồ thị
thay vì nhìn/nghe bằng cảm giác.

BẮT BUỘC: chạy khi tay máy đã lắp ráp đầy đủ (có tải thật), không chạy trên
motor trần rồi lắp lại sau -- gain tune trên tải nhẹ sẽ sai khi tải nặng hơn.

Cách dùng:
    pip install odrive matplotlib numpy
    python3 step_response_test.py

Sau mỗi lần đổi gain trong odrivetool, chạy lại file này để xem đồ thị mới,
so trực tiếp overshoot/settling time với lần trước.
"""

import time

import matplotlib.pyplot as plt
import numpy as np
import odrive
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL


def analyze_step_response(t, pos, target, start_pos, tolerance=0.02):
    """Tính overshoot (%) và settling time (s, trong dải ±tolerance quanh target)."""
    step_size = target - start_pos
    if abs(step_size) < 1e-9:
        return 0.0, 0.0

    if step_size > 0:
        peak = pos.max()
        overshoot_pct = max(0.0, (peak - target) / step_size * 100)
    else:
        peak = pos.min()
        overshoot_pct = max(0.0, (target - peak) / (-step_size) * 100)

    band = tolerance * abs(step_size)
    settled_mask = np.abs(pos - target) <= band
    settling_time = None
    for i in range(len(t)):
        if np.all(settled_mask[i:]):
            settling_time = t[i]
            break
    return overshoot_pct, settling_time


def run_step_test(axis, step_size_rev=0.05, duration=2.0, dt=0.005, start_pos=None):
    """Gửi 1 bước nhảy step_size_rev (vòng) từ start_pos (nếu cho trước, đưa
    axis về đó trước) hoặc từ vị trí hiện tại, ghi lại pos_estimate."""
    if start_pos is not None:
        axis.controller.input_pos = start_pos
        time.sleep(1.5)  # đợi ổn định về đúng điểm chuẩn trước khi test
    else:
        start_pos = axis.encoder.pos_estimate
    target = start_pos + step_size_rev

    times, positions = [], []
    axis.controller.input_pos = target

    t0 = time.time()
    while time.time() - t0 < duration:
        now = time.time() - t0
        times.append(now)
        positions.append(axis.encoder.pos_estimate)
        time.sleep(dt)

    return np.array(times), np.array(positions), start_pos, target


def plot_and_analyze(t, pos, start_pos, target, save_path="step_response.png"):
    overshoot_pct, settling_time = analyze_step_response(t, pos, target, start_pos)

    plt.figure(figsize=(9, 5))
    plt.plot(t, pos, label="pos_estimate (thật)", color="tab:blue")
    plt.axhline(target, color="tab:red", linestyle="--", label="Target")
    plt.axhline(start_pos, color="gray", linestyle=":", label="Điểm xuất phát")
    plt.xlabel("Thời gian (s)")
    plt.ylabel("Vị trí (vòng/rev)")
    plt.title(f"Step response -- Overshoot: {overshoot_pct:.1f}%, "
              f"Settling: {settling_time:.3f}s" if settling_time else
              f"Step response -- Overshoot: {overshoot_pct:.1f}%, chưa ổn định")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.savefig(save_path, dpi=120)
    plt.show()

    print(f"Overshoot: {overshoot_pct:.1f}%")
    if settling_time is not None:
        print(f"Settling time (trong ±2% dải bước nhảy): {settling_time:.3f}s")
    else:
        print("CHƯA ổn định trong thời gian đo -- tăng `duration` hoặc kiểm tra lại gain.")
    print(f"Đã lưu đồ thị: {save_path}")
    return overshoot_pct, settling_time


if __name__ == "__main__":
    print("Đang tìm ODrive qua USB...")
    odrv0 = odrive.find_any()
    print("Đã kết nối.")

    axis = odrv0.axis0  # đổi axis1 nếu cần

    if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print("CẢNH BÁO: axis chưa ở CLOSED_LOOP_CONTROL -- kiểm tra lại trước khi test.")

    print()
    print("=== Chế độ tương tác -- KHÔNG cần mở odrivetool song song ===")
    print("Lệnh: v <số>  = đổi vel_gain")
    print("      p <số>  = đổi pos_gain")
    print("      i <số>  = đổi vel_integrator_gain")
    print("      c       = vào CLOSED_LOOP_CONTROL (bắt buộc trước khi test)")
    print("      h       = LƯU vị trí hiện tại làm điểm chuẩn (làm 1 lần, trước khi tune)")
    print("      t       = chạy step test TỪ ĐIỂM CHUẨN (không phải từ vị trí trôi dạt)")
    print("      g       = in ra 3 gain hiện tại")
    print("      s       = save_configuration()")
    print("      q       = thoát")
    print()

    home_pos = [None]

    while True:
        try:
            cmd = input(">>> ").strip().split()
        except (EOFError, KeyboardInterrupt):
            break
        if not cmd:
            continue

        if cmd[0] == "q":
            break
        elif cmd[0] == "c":
            axis.controller.config.control_mode = 3  # CONTROL_MODE_POSITION_CONTROL
            axis.controller.config.input_mode = 3   
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(0.3)
            print(f"current_state = {axis.current_state} (8 = CLOSED_LOOP_CONTROL đúng)")
        elif cmd[0] == "v" and len(cmd) == 2:
            axis.controller.config.vel_gain = float(cmd[1])
            print(f"vel_gain = {axis.controller.config.vel_gain}")
        elif cmd[0] == "p" and len(cmd) == 2:
            axis.controller.config.pos_gain = float(cmd[1])
            print(f"pos_gain = {axis.controller.config.pos_gain}")
        elif cmd[0] == "i" and len(cmd) == 2:
            axis.controller.config.vel_integrator_gain = float(cmd[1])
            print(f"vel_integrator_gain = {axis.controller.config.vel_integrator_gain}")
        elif cmd[0] == "h":
            home_pos[0] = axis.encoder.pos_estimate
            print(f"Đã lưu điểm chuẩn: {home_pos[0]:.4f} -- mọi lần 't' từ giờ sẽ test từ đúng điểm này.")
        elif cmd[0] == "g":
            print(f"vel_gain={axis.controller.config.vel_gain}, "
                  f"pos_gain={axis.controller.config.pos_gain}, "
                  f"vel_integrator_gain={axis.controller.config.vel_integrator_gain}")
        elif cmd[0] == "t":
            if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                print("CẢNH BÁO: axis không ở CLOSED_LOOP_CONTROL, bỏ qua test.")
                continue
            if home_pos[0] is None:
                print("CẢNH BÁO: chưa có điểm chuẩn -- gõ 'h' trước để lưu 1 điểm cố định,"
                      " tránh mỗi lần test lại xuất phát từ 1 tư thế khác nhau.")
                continue
            t, pos, start_pos, target = run_step_test(
                axis, step_size_rev=0.5, duration=2.0, start_pos=home_pos[0]
            )
            plot_and_analyze(t, pos, start_pos, target)
        elif cmd[0] == "s":
            odrv0.save_configuration()
            print("Đã save_configuration() (driver có thể tự reboot ngắn).")
        else:
            print("Lệnh không hợp lệ -- xem lại danh sách lệnh ở trên.")