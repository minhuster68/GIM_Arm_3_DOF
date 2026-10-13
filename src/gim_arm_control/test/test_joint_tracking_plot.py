"""Verify separate joint figures and velocity signals from recorded state."""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import numpy as np
import pytest

from gim_control.plot_ee_error import JOINT_SUFFIXES, plot


@pytest.mark.parametrize('show', [False, True])
def test_position_and_velocity_figures_use_logged_signals(tmp_path, monkeypatch, show):
    # Constant position deliberately cannot reproduce these measured velocities.
    qd = np.arange(24, dtype=float).reshape(8, 3) / 10
    qd[1] = [10, 20, 30]  # Skipped when plotting; included in error metrics.
    qd_ref = np.full((8, 3), 0.5)
    log = {
        't': np.arange(8, dtype=float),
        'phase': np.array(['APPROACH'] * 2 + ['TRACK'] * 6),
        'q': np.zeros((8, 3)),
        'q_ref': np.zeros((8, 3)),
        'qd': qd,
        'qd_ref': qd_ref,
        'tau': np.ones((8, 3)),
    }
    figures = {}
    shown = []
    original_save = Figure.savefig

    def capture_save(figure, path, **kwargs):
        figures[str(path)] = figure
        original_save(figure, path, **kwargs)

    monkeypatch.setattr(Figure, 'savefig', capture_save)
    monkeypatch.setattr(plt, 'show', lambda: shown.append(len(plt.get_fignums())))
    plot(log, str(tmp_path / 'tracking.png'), show, max_plot_points=4)

    assert len(list(tmp_path.glob('*.png'))) == 6
    assert shown == ([6] if show else [])
    assert not plt.get_fignums()
    for index, suffix in enumerate(JOINT_SUFFIXES):
        position_path = tmp_path / f'tracking_{suffix}.png'
        velocity_path = tmp_path / f'tracking_{suffix}_velocity.png'
        assert position_path.stat().st_size > 0
        assert velocity_path.stat().st_size > 0
        assert len(figures[str(position_path)].axes) == 3
        axes = figures[str(velocity_path)].axes
        assert len(axes) == 2
        np.testing.assert_allclose(axes[0].lines[0].get_ydata(),
                                   np.degrees(qd[::2, index]))
        np.testing.assert_allclose(axes[0].lines[1].get_ydata(),
                                   np.degrees(qd_ref[::2, index]))
        np.testing.assert_allclose(axes[1].lines[0].get_ydata(),
                                   np.degrees((qd - qd_ref)[::2, index]))
        error = np.degrees((qd - qd_ref)[:, index])
        assert axes[1].lines[0].get_label() == (
            f'RMS={np.sqrt(np.mean(error ** 2)):.3f}°/s, '
            f'MAX={np.max(np.abs(error)):.3f}°/s')
        assert axes[0].texts[0].get_text() == 'TRACK'
        assert 'độ/s' in axes[0].get_ylabel()
        assert 'độ/s' in axes[1].get_ylabel()
