#!/usr/bin/env python3
"""Plot joint tracking signals from a common effort-controller CSV log."""

import argparse
import csv
import os

import numpy as np


JOINT_NAMES = ("base_joint", "shoulder_joint", "elbow_joint")
JOINT_TITLES = ("q1 - Base joint", "q2 - Shoulder joint", "q3 - Elbow joint")
JOINT_SUFFIXES = ("q1_base", "q2_shoulder", "q3_elbow")
JOINT_COLORS = ("tab:blue", "tab:orange", "tab:green")
TRAJECTORY_PHASES = ("GRAVITY", "APPROACH", "TRACK", "RETURN", "HOLD")


def load_log(path):
    """Load joint reference, measured state and commanded torque columns."""
    with open(path, newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"t_wall", "phase"}
        for prefix in ("q", "qd", "qref", "qdref", "tau"):
            required.update(f"{prefix}_{name}" for name in JOINT_NAMES)
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(
                "CSV thiếu cột: " + ", ".join(sorted(missing)))

        columns = {
            "t_wall": [],
            "phase": [],
            "q": [],
            "qd": [],
            "q_ref": [],
            "qd_ref": [],
            "tau": [],
        }
        optional = [prefix for prefix in ('tau_ff', 'tau_fb', 'tau_p', 'tau_i',
                                         'tau_raw', 'saturated', 'qd_filtered',
                                         'tau_lambda', 'tau_k1', 'tau_k2', 'tau_slew')
                    if all(f'{prefix}_{name}' in reader.fieldnames for name in JOINT_NAMES)]
        columns.update({prefix: [] for prefix in optional})
        for row in reader:
            columns["t_wall"].append(float(row["t_wall"]))
            columns["phase"].append(row["phase"])
            columns["q"].append([
                float(row[f"q_{name}"]) for name in JOINT_NAMES])
            columns["qd"].append([
                float(row[f"qd_{name}"]) for name in JOINT_NAMES])
            columns["q_ref"].append([
                float(row[f"qref_{name}"]) for name in JOINT_NAMES])
            columns["qd_ref"].append([
                float(row[f"qdref_{name}"]) for name in JOINT_NAMES])
            columns["tau"].append([
                float(row[f"tau_{name}"]) for name in JOINT_NAMES])
            for prefix in optional:
                columns[prefix].append([
                    float({'True': '1', 'False': '0'}.get(
                        row[f'{prefix}_{name}'], row[f'{prefix}_{name}']))
                    for name in JOINT_NAMES])

    if not columns["t_wall"]:
        raise ValueError("CSV không có mẫu điều khiển")
    t_wall = np.asarray(columns["t_wall"], dtype=float)
    arrays = {
        key: np.asarray(value, dtype=float)
        for key, value in columns.items()
        if key not in ("t_wall", "phase")
    }
    arrays["t"] = t_wall - t_wall[0]
    arrays["phase"] = np.asarray(columns["phase"])
    if not all(np.all(np.isfinite(arrays[key])) for key in
               ('q', 'qd', 'q_ref', 'qd_ref', 'tau', 't')):
        raise ValueError("CSV chứa NaN/Inf")
    return arrays


def phase_groups(phases):
    """Yield the complete log and each trajectory phase that is present."""
    yield "ALL", np.ones(len(phases), dtype=bool)
    for phase in TRAJECTORY_PHASES:
        mask = phases == phase
        if np.any(mask):
            yield phase, mask


def format_vector(value):
    """Format a three-joint metric vector for the terminal table."""
    return " ".join(f"{item:7.3f}" for item in value)


def print_metrics(log):
    """Print position-error and torque metrics for controller comparison."""
    angle_error = np.degrees(log["q"] - log["q_ref"])
    print("\nMetrics từng khớp")
    print("phase       samples   RMS e_q [deg]       MAX |e_q| [deg]   "
          "MAX |tau| [Nm]")
    print("-" * 100)
    for label, mask in phase_groups(log["phase"]):
        rms_angle = np.sqrt(np.mean(angle_error[mask] ** 2, axis=0))
        max_angle = np.max(np.abs(angle_error[mask]), axis=0)
        max_tau = np.max(np.abs(log["tau"][mask]), axis=0)
        print(
            f"{label:<10} {np.count_nonzero(mask):>8d}   "
            f"[{format_vector(rms_angle)}]   "
            f"[{format_vector(max_angle)}]   "
            f"[{format_vector(max_tau)}]")
    if all(prefix in log for prefix in ('tau_ff', 'tau_fb', 'saturated')):
        print('\nTorque: peak |tau_ff|, |tau_fb| [Nm] and saturation [%]')
        for index, name in enumerate(JOINT_NAMES):
            valid = np.isfinite(log['tau_fb'][:, index])
            if not np.any(valid):
                continue
            ff = np.max(np.abs(log['tau_ff'][valid, index]))
            fb = np.max(np.abs(log['tau_fb'][valid, index]))
            saturation = 100.0 * np.mean(log['saturated'][valid, index])
            print(f'{name}: ff={ff:.4f}, fb={fb:.4f}, saturated={saturation:.1f}%')


def add_phase_boundaries(axes, t, phases):
    """Mark transitions such as APPROACH -> TRACK on every subplot."""
    transitions = np.flatnonzero(phases[1:] != phases[:-1]) + 1
    for index in transitions:
        transition_time = t[index]
        for axis in axes:
            axis.axvline(
                transition_time, color="0.35", linestyle=":",
                linewidth=0.9, alpha=0.8)
        axes[0].annotate(
            phases[index], xy=(transition_time, 1.0),
            xycoords=("data", "axes fraction"), xytext=(4, -4),
            textcoords="offset points", va="top", fontsize=8,
            color="0.25")


def output_paths(output, velocity=False):
    """Return separate position or velocity paths for the three joints."""
    if not output:
        raise ValueError("output không được rỗng")
    root, extension = os.path.splitext(output)
    if not extension:
        root, extension = output, ".png"
    signal_suffix = "_velocity" if velocity else ""
    return [f"{root}_{suffix}{signal_suffix}{extension}" for suffix in JOINT_SUFFIXES]


def plot(log, output, show, max_plot_points):
    """Create separate position/torque and velocity figures for each joint."""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    count = len(log["t"])
    stride = max(1, int(np.ceil(count / max_plot_points)))
    sample = slice(None, None, stride)
    t = log["t"][sample]
    phases = log["phase"][sample]
    q_deg = np.degrees(log["q"][sample])
    q_ref_deg = np.degrees(log["q_ref"][sample])
    angle_error = q_deg - q_ref_deg
    tau = log["tau"][sample]

    figures = []
    paths = output_paths(output)
    for index, (title, color, path) in enumerate(zip(
            JOINT_TITLES, JOINT_COLORS, paths)):
        figure, axes = plt.subplots(
            3, 1, figsize=(13, 10), sharex=True,
            constrained_layout=True)

        axes[0].plot(
            t, q_deg[:, index], color=color, linewidth=1.1,
            label="Thực tế")
        axes[0].plot(
            t, q_ref_deg[:, index], color="tab:red", linestyle="--",
            linewidth=1.1, label="Mong muốn")
        axes[0].set_ylabel("Góc (độ)")
        axes[0].set_title("Góc khớp")
        axes[0].legend(loc="best", ncol=2)

        full_error = np.degrees(log["q"][:, index] - log["q_ref"][:, index])
        angle_rms = float(np.sqrt(np.mean(full_error ** 2)))
        angle_max = float(np.max(np.abs(full_error)))
        axes[1].plot(
            t, angle_error[:, index], color=color, linewidth=1.0,
            label=f"RMS={angle_rms:.3f}°, MAX={angle_max:.3f}°")
        axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
        axes[1].set_ylabel("Sai số góc (độ)")
        axes[1].set_title("e_q = q thực tế - q mong muốn")
        axes[1].legend(loc="best")

        torque_peak = float(np.max(np.abs(tau[:, index])))
        axes[2].plot(
            t, tau[:, index], color="tab:purple", linewidth=1.0,
            label=f"MAX |tau|={torque_peak:.3f} Nm")
        sta_components = ('tau_lambda', 'tau_k1', 'tau_k2')
        has_sta_components = all(
            prefix in log and np.any(np.isfinite(log[prefix][:, index]))
            for prefix in sta_components)
        series = [('tau_ff', 'tab:green', 'tau_ff')]
        if has_sta_components:
            series.extend([
                ('tau_lambda', 'tab:red', 'tau_lambda (-lambda*de)'),
                ('tau_k1', 'tab:orange', 'tau_k1'),
                ('tau_k2', 'tab:blue', 'tau_k2 (từ z)'),
            ])
            if ('tau_slew' in log and np.any(
                    np.isfinite(log['tau_slew'][:, index])
                    & (np.abs(log['tau_slew'][:, index]) > 1e-9))):
                series.append(('tau_slew', 'tab:brown', 'tau_slew (hiệu chỉnh giới hạn)'))
        else:
            # Old logs and saturation SMC do not have a finite STA split.
            series.extend([('tau_fb', 'tab:orange', 'tau_fb'),
                           ('tau_p', 'tab:red', 'tau_p'), ('tau_i', 'tab:brown', 'tau_i')])
        for prefix, color, label in series:
            if prefix in log:
                axes[2].plot(t, log[prefix][sample, index], color=color,
                             linewidth=0.8, linestyle='--', label=label)
        if 'tau_raw' in log:
            # The node may scale gravity and clip torque after the controller
            # records tau_ff, tau_fb and tau_raw. Show that final correction so
            # the plotted terms sum to the actual commanded torque.
            tau_post = log['tau'][:, index] - log['tau_raw'][:, index]
            if np.any(np.isfinite(tau_post)) and np.nanmax(np.abs(tau_post)) > 1e-6:
                axes[2].plot(t, tau_post[sample], color='tab:gray',
                             linewidth=0.9, linestyle=':',
                             label='tau_post (gravity_scale/kẹp)')
        axes[2].axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
        axes[2].set_ylabel("Mô-men (Nm)")
        axes[2].set_title("Mô-men điều khiển đầu ra")
        axes[2].legend(loc="best")

        for axis in axes:
            axis.set_xlabel("Thời gian t (s)")
            axis.tick_params(labelbottom=True)
            axis.grid(alpha=0.3)
        add_phase_boundaries(axes, t, phases)
        figure.suptitle(f"{title} — output và sai số bám", fontsize=15)
        figure.savefig(path, dpi=150)
        figures.append(figure)
        print(f"Đã lưu: {os.path.abspath(path)}")

    qd_deg = np.degrees(log["qd"][sample])
    qd_ref_deg = np.degrees(log["qd_ref"][sample])
    velocity_error = qd_deg - qd_ref_deg
    for index, (title, color, path) in enumerate(zip(
            JOINT_TITLES, JOINT_COLORS, output_paths(output, velocity=True))):
        figure, axes = plt.subplots(
            2, 1, figsize=(13, 7), sharex=True,
            constrained_layout=True)
        axes[0].plot(
            t, qd_deg[:, index], color=color, linewidth=1.1,
            label="Thực tế")
        axes[0].plot(
            t, qd_ref_deg[:, index], color="tab:red", linestyle="--",
            linewidth=1.1, label="Mong muốn")
        axes[0].set_ylabel("Vận tốc (độ/s)")
        axes[0].set_title("Vận tốc khớp")
        axes[0].legend(loc="best", ncol=2)

        full_error = np.degrees(log["qd"][:, index] - log["qd_ref"][:, index])
        velocity_rms = float(np.sqrt(np.mean(full_error ** 2)))
        velocity_max = float(np.max(np.abs(full_error)))
        axes[1].plot(
            t, velocity_error[:, index], color=color, linewidth=1.0,
            label=f"RMS={velocity_rms:.3f}°/s, MAX={velocity_max:.3f}°/s")
        axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
        axes[1].set_ylabel("Sai số vận tốc (độ/s)")
        axes[1].set_title("e_qdot = vận tốc thực tế - vận tốc mong muốn")
        axes[1].legend(loc="best")

        if 'qd_filtered' in log:
            estimate = log['qd_filtered'][:, index]
            valid = np.isfinite(estimate)
            if np.any(valid) and np.any(np.abs(estimate[valid] - log['qd'][valid, index]) > 1e-9):
                filtered_deg = np.degrees(estimate)
                filtered_error = filtered_deg - np.degrees(log['qd_ref'][:, index])
                axes[0].plot(t, filtered_deg[sample], color='tab:cyan', linewidth=1.2,
                             label='Sau lọc (đưa vào điều khiển)')
                axes[1].plot(t, filtered_error[sample], color='tab:cyan', linewidth=1.0,
                             label='Sai số vận tốc sau lọc')
                axes[0].legend(loc='best', ncol=2)
                axes[1].legend(loc='best')

        for axis in axes:
            axis.set_xlabel("Thời gian t (s)")
            axis.tick_params(labelbottom=True)
            axis.grid(alpha=0.3)
        add_phase_boundaries(axes, t, phases)
        figure.suptitle(f"{title} — vận tốc và sai số vận tốc", fontsize=15)
        figure.savefig(path, dpi=150)
        figures.append(figure)
        print(f"Đã lưu: {os.path.abspath(path)}")

    if show:
        plt.show()
    for figure in figures:
        plt.close(figure)


def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description=(
            "Vẽ hình góc/sai số góc/mô-men và hình vận tốc/sai số vận tốc riêng cho "
            "từng khớp từ CSV effort controller."))
    parser.add_argument("csv", help="file CSV truyền bằng launch log_file:=...")
    parser.add_argument(
        "--output", default="",
        help=("prefix PNG đầu ra; mặc định <csv>_joints.png, sau đó thêm "
              "tên từng khớp; hình vận tốc thêm _velocity"))
    parser.add_argument("--show", action="store_true",
                        help="mở sáu cửa sổ đồ thị sau khi lưu (hai hình mỗi khớp)")
    parser.add_argument("--max-plot-points", type=int, default=6000,
                        help="giảm mẫu khi vẽ; metrics vẫn dùng toàn bộ mẫu")
    args = parser.parse_args()
    if args.max_plot_points <= 0:
        parser.error("--max-plot-points phải > 0")

    output = args.output
    if not output:
        output = os.path.splitext(args.csv)[0] + "_joints.png"
    log = load_log(args.csv)
    print_metrics(log)
    plot(log, output, args.show, args.max_plot_points)


if __name__ == "__main__":
    main()
