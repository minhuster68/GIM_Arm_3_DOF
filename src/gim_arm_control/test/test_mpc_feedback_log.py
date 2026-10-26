"""CSV schema, skipped samples and plots preserve MPC feedback attribution."""
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure
import numpy as np
import rclpy
from rclpy.parameter import Parameter

from gim_arm_controller_mpc.factory import MpcFactory
from gim_control.controller_api import MPC_FEEDBACK_COMPONENTS
from gim_control.effort_controller_node import EffortControllerNode
from gim_control.plot_ee_error import load_log, plot


def test_feedback_csv_round_trip_and_plot_including_stale_sample(tmp_path, monkeypatch):
    root = Path(__file__).resolve().parents[3]
    path = tmp_path / 'mpc.csv'
    rclpy.init(args=[])
    node = None
    try:
        node = EffortControllerNode(MpcFactory(), parameter_overrides=[
            Parameter('urdf_file', value=str(root / 'src/gim_arm_description/urdf/gim_arm.urdf')),
            Parameter('log_file', value=str(path)),
            Parameter('algorithm_hold', value=True),
            Parameter('precompute_prediction', value=False),
            Parameter('feedback_bounds_mode', value='instantaneous'),
            Parameter('qp_solver', value='active_set'),
        ])
        node.phase = 'TRACK'
        node.q, node.qd = np.array([.01, -.01, .02]), np.zeros(3)
        zero = np.zeros(3)
        tau = node.controller.compute(node.q, node.qd, zero, zero, zero, .01)
        node._append_tracking_row(1_000_000_000, 0., .01, .001, zero, zero, tau)
        node._append_tracking_row(1_010_000_000, 0., .01, float('nan'), zero, zero, tau,
                                  event='STALE_STATE_HOLD')
        node.dump()
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 2
        assert all(None not in row and all(value is not None for value in row.values()) for row in rows)
        log = load_log(path)
        assert all(key in log for key in MPC_FEEDBACK_COMPONENTS)
        parts = sum(log[k][0] for k in MPC_FEEDBACK_COMPONENTS[:5])
        np.testing.assert_allclose(parts, log['tau_fb'][0])
        assert all(np.isnan(log[k][1]).all() for k in MPC_FEEDBACK_COMPONENTS)
        figures = []
        monkeypatch.setattr(Figure, 'savefig', lambda fig, *args, **kw: figures.append(fig))
        plot(log, str(tmp_path / 'tracking.png'), False, 100)
        assert len(figures) == 6
        labels = [line.get_label() for line in figures[0].axes[2].lines]
        assert 'Tích phân (QP tự do)' in labels
        assert 'Hiệu chỉnh ràng buộc/solver' in labels
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
