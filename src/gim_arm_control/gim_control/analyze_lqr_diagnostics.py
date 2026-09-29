#!/usr/bin/env python3
"""Summarize timing, gain changes and torque limits in a diagnostic LQR CSV."""

import argparse
import csv
import os

import numpy as np


JOINTS = ("base_joint", "shoulder_joint", "elbow_joint")
VELOCITY_LIMIT_RAD_S = np.asarray((15.708, 1.963, 15.708))
REQUIRED = (
    "t_wall", "phase", "event", "tick_wall_ns", "publish_wall_ns",
    "state_seq", "state_header_sim_ns", "state_rx_wall_ns",
    "clock_sim_ns", "clock_rx_wall_ns", "gain_schedule_index",
    "gain_delta_fro",
)


def load(path):
    with open(path, newline="") as stream:
        reader = csv.DictReader(stream)
        missing = set(REQUIRED).difference(reader.fieldnames or ())
        if missing:
            raise ValueError(
                "CSV chưa có cột chẩn đoán LQR: "
                + ", ".join(sorted(missing))
                + ". Cần build và chạy lại node với log_file khác rỗng.")
        rows = list(reader)
    if not rows:
        raise ValueError("CSV không có dữ liệu")
    return rows


def vector(rows, prefix):
    return np.asarray([
        [float(row[f"{prefix}_{joint}"]) for joint in JOINTS]
        for row in rows
    ])


def report(rows, lookback, state_rows=None):
    first_ns = int(rows[0]["tick_wall_ns"])
    time_s = np.asarray([
        (int(row["tick_wall_ns"]) - first_ns) * 1e-9
        for row in rows
    ])
    events = np.asarray([row["event"] for row in rows])
    control = events == "CONTROL"
    abort_indices = np.flatnonzero(~control)
    final_index = int(abort_indices[0]) if len(abort_indices) else len(rows) - 1
    focus_s = float(time_s[final_index])
    focus = control & (time_s >= focus_s - lookback) & (time_s <= focus_s)
    if not np.any(focus):
        raise ValueError("Không có mẫu CONTROL trong khoảng cần xem")

    tick_ns = np.asarray([int(row["tick_wall_ns"]) for row in rows])
    state_rx_ns = np.asarray([int(row["state_rx_wall_ns"]) for row in rows])
    state_sim_ns = np.asarray([
        int(row["state_header_sim_ns"]) for row in rows])
    clock_sim_ns = np.asarray([int(row["clock_sim_ns"]) for row in rows])
    state_seq = np.asarray([int(row["state_seq"]) for row in rows])
    gain_index = np.asarray([
        int(row["gain_schedule_index"]) for row in rows])
    gain_delta = np.asarray([
        float(row["gain_delta_fro"]) for row in rows])
    qd = vector(rows, "qd")
    qdref = vector(rows, "qdref")
    q = vector(rows, "q")
    qref = vector(rows, "qref")
    tau_raw = vector(rows, "tau_raw")
    tau_controller = vector(rows, "tau_controller")
    source_dt_s = np.diff(state_sim_ns).astype(float) * 1e-9
    sampled_qd = np.full_like(qd, np.nan)
    valid_difference = ((source_dt_s > 0.001)
                        & (source_dt_s < 0.020)
                        & (np.diff(state_seq) > 0))
    sampled_qd[1:][valid_difference] = (
        np.diff(q, axis=0)[valid_difference]
        / source_dt_s[valid_difference, None])
    velocity_consistency = np.max(np.abs(sampled_qd - qd), axis=1)

    print(f"Log: {len(rows)} mẫu; khoảng t=0..{time_s[-1]:.3f} s")
    print(f"Pha cuối: {rows[final_index]['phase']}; "
          f"sự kiện: {events[final_index]} tại t={focus_s:.3f} s")
    print(f"Cửa sổ đo: t={max(0.0, focus_s-lookback):.3f}.."
          f"{focus_s:.3f} s ({np.count_nonzero(focus)} mẫu CONTROL)")

    def peak(label, values, valid=focus, scale=1.0, unit=""):
        indices = np.flatnonzero(valid)
        values = np.asarray(values)
        if not len(indices) or not np.any(np.isfinite(values[indices])):
            print(f"  {label}: chưa có mẫu hợp lệ")
            return
        local = np.abs(values[indices])
        index = indices[int(np.nanargmax(local))]
        print(f"  {label}: {values[index]*scale:.3f}{unit} "
              f"(t={time_s[index]:.3f} s)")

    angle_error = np.max(np.abs(q - qref), axis=1)
    velocity_error = np.max(np.abs(qd - qdref), axis=1)
    raw_peak = np.max(np.abs(tau_raw), axis=1)
    clipping = np.max(np.abs(tau_raw - tau_controller), axis=1)
    receipt_age_ms = (tick_ns - state_rx_ns) * 1e-6
    valid_state = (state_rx_ns > 0) & focus
    sim_lag_ms = (clock_sim_ns - state_sim_ns) * 1e-6
    valid_sim = (clock_sim_ns >= 0) & (state_sim_ns >= 0) & focus

    peak("|q-qref| lớn nhất", angle_error, scale=180.0/np.pi,
         unit=" độ")
    peak("|qd-qdref| lớn nhất", velocity_error, unit=" rad/s")
    peak("|qd - Δq/Δt_Gazebo| lớn nhất", velocity_consistency,
         unit=" rad/s")
    peak("|torque trước kẹp| lớn nhất", raw_peak, unit=" Nm")
    peak("mức kẹp/slew torque lớn nhất", clipping, unit=" Nm")
    peak("độ đổi gain Frobenius lớn nhất", gain_delta)
    peak("tuổi mẫu kể từ lúc callback nhận", receipt_age_ms,
         valid=valid_state, unit=" ms")
    clock_updates = np.diff(np.unique(clock_sim_ns[valid_sim]))
    if len(clock_updates) and np.median(clock_updates) <= 20_000_000:
        peak("clock Gazebo - stamp trạng thái", sim_lag_ms,
             valid=valid_sim, unit=" ms")
    elif len(clock_updates):
        print(f"  /clock chỉ cập nhật mỗi "
              f"{np.median(clock_updates)*1e-6:.1f} ms; "
              "không suy ra được tuổi mẫu vật lý từ nó")

    selected = np.flatnonzero(focus)
    switches = np.count_nonzero(np.diff(gain_index[selected]))
    reused = np.count_nonzero(np.diff(state_seq[selected]) == 0)
    clipped = np.count_nonzero(clipping[selected] > 1e-6)
    print(f"  Đổi gain: {switches}; lặp lại cùng mẫu trạng thái: {reused}; "
          f"mẫu torque bị kẹp/slew: {clipped}")

    for label, condition in (
        ("|qd-qdref| > 1 rad/s", velocity_error > 1.0),
        ("|qd - Δq/Δt_Gazebo| > 0.5 rad/s",
         velocity_consistency > 0.5),
        ("tuổi mẫu nhận > 10 ms", receipt_age_ms > 10.0),
        ("torque bị kẹp/slew", clipping > 1e-6),
    ):
        indices = np.flatnonzero(focus & condition)
        if len(indices):
            print(f"  Lần đầu {label}: t={time_s[indices[0]]:.3f} s")

    if len(abort_indices):
        index = final_index
        print(f"Mẫu ABORT: |q-qref|={angle_error[index]*180.0/np.pi:.3f} độ; "
              "tau trong dòng này là lệnh trước ABORT, không phải torque "
              "mới tính cho mẫu lỗi.")
    window_label = "gần ABORT" if len(abort_indices) else "trong cửa sổ cuối"
    if state_rows:
        high_rate_consistency(
            state_rows, first_ns, focus_s, lookback, rows, window_label)
    else:
        print("Chưa có file *_states.csv để kiểm tra vận tốc ở tốc độ gốc.")
    has_effort_state = bool(
        state_rows
        and all(f"effort_{joint}" in state_rows[0] for joint in JOINTS))
    note = (
        "Effort state là lệnh đã qua giới hạn effort/velocity của Gazebo, "
        "không phải reaction torque tại khớp."
        if has_effort_state else
        "CSV cũ không đo được thời điểm Gazebo thực sự nhận torque.")
    print("Lưu ý: Δq/Δt_Gazebo tính từ các mẫu ở tần số controller, "
          f"có thể bỏ sót dao động nhanh. {note}")


def high_rate_consistency(
        rows, first_ns, focus_s, lookback, control_rows, window_label):
    stamp = np.asarray([
        int(row["state_header_sim_ns"]) for row in rows], dtype=np.int64)
    receipt = np.asarray([
        int(row["state_rx_wall_ns"]) for row in rows], dtype=np.int64)
    q = vector(rows, "q")
    qd = vector(rows, "qd")
    if len(rows) < 3:
        return
    # Gazebo/ODE uses a semi-implicit step: q[k] is advanced with qd[k].
    # Therefore (q[k]-q[k-1])/dt is the matching velocity sample.  A centred
    # difference averages qd[k] and qd[k+1]; during the fast oscillation under
    # investigation that average created a large but artificial mismatch.
    dt_s = (stamp[1:] - stamp[:-1]) * 1e-9
    valid = (dt_s > 0.0001) & (dt_s < 0.006)
    fd = np.full_like(qd[1:], np.nan)
    fd[valid] = (q[1:][valid] - q[:-1][valid]) / dt_s[valid, None]
    mismatch = np.abs(qd[1:] - fd)
    t = (receipt[1:] - first_ns) * 1e-9
    focus = (t >= focus_s - lookback) & (t <= focus_s) & valid
    baseline = (t >= 5.0) & (t < min(15.0, focus_s-lookback)) & valid
    print(f"/joint_states tốc độ gốc: {len(rows)} mẫu; "
          f"chu kỳ stamp trung vị "
          f"{np.median(np.diff(stamp))*1e-6:.3f} ms")
    if np.any(baseline):
        baseline_label = (
            "trước vùng lỗi" if window_label == "gần ABORT"
            else "trước cửa sổ cuối")
        print(f"  RMS |qd - Δq/Δt_Gazebo| {baseline_label} "
              f"[base, shoulder, elbow]: "
              f"{np.round(np.sqrt(np.nanmean(mismatch[baseline]**2, axis=0)), 3)} "
              "rad/s")
    if np.any(focus):
        print(f"  Đỉnh |qd - Δq/Δt_Gazebo| {window_label} "
              f"[base, shoulder, elbow]: "
              f"{np.round(np.nanmax(mismatch[focus], axis=0), 3)} rad/s")
        onset = np.flatnonzero(focus & np.any(mismatch > 0.5, axis=1))
        if len(onset):
            print(f"  Lần đầu độ lệch tốc độ gốc > 0.5 rad/s: "
                  f"t={t[onset[0]]:.3f} s")

        # Expose an inter-sample mode which can alias into a slower controller.
        focused_qd = qd[1:][focus]
        focused_stamp = stamp[1:][focus]
        sample_dt = np.median(np.diff(focused_stamp)) * 1e-9
        if len(focused_qd) >= 16 and sample_dt > 0.0:
            frequencies = np.fft.rfftfreq(len(focused_qd), sample_dt)
            spectrum = np.abs(np.fft.rfft(
                focused_qd - np.mean(focused_qd, axis=0), axis=0)) ** 2
            high_frequency = ((frequencies >= 20.0)
                              & (frequencies <= 0.45 / sample_dt))
            if np.any(high_frequency):
                candidates = np.flatnonzero(high_frequency)
                dominant = frequencies[candidates[
                    np.argmax(spectrum[high_frequency], axis=0)]]
                print(f"  Tần số qd trội >=20 Hz {window_label} "
                      "[base, shoulder, elbow]: "
                      f"{np.round(dominant, 1)} Hz")

    effort_columns = [f"effort_{joint}" for joint in JOINTS]
    if all(column in rows[0] for column in effort_columns):
        effort = vector(rows, "effort")[1:]
        if np.any(focus & np.all(np.isfinite(effort), axis=1)):
            print(f"  Đỉnh |effort joint Gazebo| {window_label} "
                  "[base, shoulder, elbow]: "
                  f"{np.round(np.nanmax(np.abs(effort[focus]), axis=0), 3)} Nm")
            report_effort_timing(
                stamp[1:], receipt[1:], qd[1:], effort,
                control_rows, first_ns, focus)
    else:
        print("  Log cũ chưa có effort state Gazebo; cần chạy lại diagnostic.")


def report_effort_timing(stamp, receipt, qd, effort, control_rows,
                         first_ns, focus):
    publish = np.asarray([
        int(row["publish_wall_ns"]) for row in control_rows], dtype=np.int64)
    state_used = np.asarray([
        int(row["state_header_sim_ns"]) for row in control_rows],
        dtype=np.int64)
    command = vector(control_rows, "tau")

    wall_delays_ms = []
    simulated_delays_ms = []
    for command_index in range(len(control_rows)):
        if control_rows[command_index]["event"] != "CONTROL":
            continue
        begin = int(np.searchsorted(receipt, publish[command_index]))
        end = int(np.searchsorted(
            receipt, publish[command_index] + 20_000_000))
        if end <= begin:
            continue
        difference = np.max(
            np.abs(effort[begin:end] - command[command_index]), axis=1)
        matches = np.flatnonzero(difference < 1.0e-8)
        if not len(matches):
            continue
        state_index = begin + int(matches[0])
        wall_delays_ms.append(
            (receipt[state_index] - publish[command_index]) * 1.0e-6)
        simulated_delays_ms.append(
            (stamp[state_index] - state_used[command_index]) * 1.0e-6)

    if wall_delays_ms:
        print("  Publish -> effort state: trung vị/p95 "
              f"{np.round(np.percentile(wall_delays_ms, [50, 95]), 3)} ms wall; "
              "state dùng -> state thấy effort: trung vị/p95 "
              f"{np.round(np.percentile(simulated_delays_ms, [50, 95]), 3)} ms sim")

    latest_command = np.searchsorted(publish, receipt, side="right") - 1
    valid_command = latest_command >= 0
    outward = np.sign(command[np.maximum(latest_command, 0)]) == np.sign(qd)
    velocity_exceeded = np.abs(qd) > VELOCITY_LIMIT_RAD_S
    force_was_zeroed = ((np.abs(effort) < 1.0e-12)
                        & (np.abs(command[np.maximum(latest_command, 0)]) > 1.0e-3)
                        & outward & velocity_exceeded
                        & valid_command[:, None])
    limited = focus[:, None] & force_was_zeroed
    if np.any(limited):
        indices = np.argwhere(limited)
        first_state, first_joint = indices[0]
        relative_time = (receipt[first_state] - first_ns) * 1.0e-9
        counts = np.count_nonzero(limited, axis=0)
        print("  Gazebo cắt effort do vượt velocity limit: "
              f"{counts} mẫu; lần đầu t={relative_time:.3f} s ở "
              f"{JOINTS[first_joint]} (qd={qd[first_state, first_joint]:.3f}, "
              f"limit={VELOCITY_LIMIT_RAD_S[first_joint]:.3f} rad/s)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", help="log_file của phiên LQR có chẩn đoán")
    parser.add_argument("--lookback", type=float, default=1.5,
                        help="số giây trước ABORT (mặc định 1.5)")
    args = parser.parse_args()
    if not np.isfinite(args.lookback) or args.lookback <= 0:
        parser.error("--lookback phải > 0 và hữu hạn")
    state_path = os.path.splitext(args.csv)[0] + "_states.csv"
    state_rows = None
    if os.path.exists(state_path):
        with open(state_path, newline="") as stream:
            state_rows = list(csv.DictReader(stream))
    report(load(args.csv), args.lookback, state_rows)


if __name__ == "__main__":
    main()
