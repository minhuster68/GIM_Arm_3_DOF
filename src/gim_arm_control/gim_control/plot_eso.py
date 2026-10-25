"""Plot separate LESO diagnostics without mixing them into tracking metrics."""
import argparse
import csv
from pathlib import Path

import numpy as np


def plot_eso(path, show=False):
    import matplotlib.pyplot as plt
    with open(path, newline='') as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        rows = list(reader)
    joints = ('base_joint', 'shoulder_joint', 'elbow_joint')
    if not rows or not all('tau_eso_'+name in fields for name in joints):
        raise ValueError('CSV chưa có dữ liệu ESO; dùng profile *_eso_hardware.yaml')
    def vector(prefix, joint):
        return np.array([float(row[prefix+'_'+joint]) for row in rows])
    wall = np.array([float(row['t_wall']) for row in rows])
    t = wall-wall[0]
    figures, outputs = [], []
    for i, joint in enumerate(joints, 1):
        fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True, constrained_layout=True)
        axes[0].plot(t, np.rad2deg(vector('q', joint)), label='Góc đo')
        axes[0].plot(t, np.rad2deg(vector('eso_q_hat', joint)), '--', label='q_hat ESO')
        axes[0].set_ylabel('Góc (độ)')
        raw = 'qd_raw' if 'qd_raw_'+joint in fields else 'qd'
        axes[1].plot(t, np.rad2deg(vector(raw, joint)), color='gray', alpha=.5, label='Vận tốc driver')
        axes[1].plot(t, np.rad2deg(vector('eso_v_hat', joint)), label='v_hat ESO (nội bộ observer)')
        if 'qd_filtered_'+joint in fields:
            axes[1].plot(t, np.rad2deg(vector('qd_filtered', joint)), '--', label='Vận tốc Kalman cho FB')
        axes[1].set_ylabel('Vận tốc (độ/s)')
        for prefix, title in (('tau_dist_hat', 'M_nom(q)*d_hat'), ('tau_eso', 'Bù ESO (có dấu)'),
                              ('tau_pre_eso', 'Torque trước ESO'),
                              ('tau_eso_actuator_adjustment', 'Hiệu chỉnh giới hạn cuối'), ('tau', 'Torque gửi')):
            axes[2].plot(t, vector(prefix, joint), label=title)
        axes[2].set_ylabel('Torque (Nm)')
        axes[2].set_xlabel('Thời gian từ đầu log (s)')
        for ax in axes:
            ax.grid(alpha=.3)
            ax.legend(loc='best')
        fig.suptitle(f'ESO q{i}: {joint}')
        output = Path(path).with_suffix('').with_name(Path(path).stem+f'_eso_q{i}_{joint.removesuffix("_joint")}.png')
        fig.savefig(output, dpi=150)
        figures.append(fig)
        outputs.append(output)
    if show:
        plt.show()
    for fig in figures:
        plt.close(fig)
    return outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv')
    parser.add_argument('--show', action='store_true')
    args = parser.parse_args()
    for output in plot_eso(args.csv, args.show):
        print(output.resolve())
