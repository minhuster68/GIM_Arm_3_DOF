"""Verify separate joint figures and velocity signals from recorded state."""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backend_bases import MouseEvent, PickEvent
import numpy as np
import pytest

from gim_control.plot_ee_error import JOINT_SUFFIXES, enable_torque_interaction, plot


@pytest.mark.parametrize('show', [False, True])
@pytest.mark.parametrize('filtered', [False, True])
def test_position_and_velocity_figures_use_logged_signals(tmp_path, monkeypatch, show, filtered):
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
    log['tau'][1] = [10, 20, 30]
    log['tau_ff'] = np.full((8, 3), 0.7)
    log['tau_fb'] = log['tau'] - log['tau_ff']
    if filtered:
        log.update(qd_raw=qd + .123, qd_filtered=qd)
    figures = {}
    shown = []
    original_save = Figure.savefig

    def capture_save(figure, path, **kwargs):
        figures[str(path)] = figure
        original_save(figure, path, **kwargs)

    monkeypatch.setattr(Figure, 'savefig', capture_save)
    monkeypatch.setattr(plt, 'show', lambda: shown.append(len(plt.get_fignums())))
    plot(log, str(tmp_path / 'tracking.png'), show, max_plot_points=4)

    assert len(list(tmp_path.glob('*.png'))) == 9
    assert shown == ([9] if show else [])
    assert not plt.get_fignums()
    for index, suffix in enumerate(JOINT_SUFFIXES):
        position_path = tmp_path / f'tracking_{suffix}.png'
        torque_path = tmp_path / f'tracking_{suffix}_torque.png'
        velocity_path = tmp_path / f'tracking_{suffix}_velocity.png'
        assert position_path.stat().st_size > 0
        assert torque_path.stat().st_size > 0
        assert velocity_path.stat().st_size > 0
        position_axes = figures[str(position_path)].axes
        assert len(position_axes) == 2
        assert position_axes[0].get_ylabel() == 'Góc (độ)'
        assert position_axes[1].get_ylabel() == 'Sai số góc (độ)'
        assert len(figures[str(torque_path)].axes) == 1
        torque_axis = figures[str(torque_path)].axes[0]
        assert torque_axis.get_ylabel() == 'Mô-men (Nm)'
        for line, key in zip(torque_axis.lines, ('tau', 'tau_ff', 'tau_fb')):
            np.testing.assert_allclose(line.get_ydata(), log[key][::2, index])
        assert f'{10 * (index + 1):.3f} Nm' in torque_axis.lines[0].get_label()
        assert torque_axis.texts[0].get_text() == 'TRACK'
        axes = figures[str(velocity_path)].axes
        assert len(axes) == 2
        np.testing.assert_allclose(axes[0].lines[0].get_ydata(),
                                   np.degrees(qd[::2, index]))
        np.testing.assert_allclose(axes[0].lines[1].get_ydata(),
                                   np.degrees(qd_ref[::2, index]))
        np.testing.assert_allclose(axes[1].lines[0].get_ydata(),
                                   np.degrees((qd - qd_ref)[::2, index]))
        if filtered:
            np.testing.assert_allclose(axes[0].lines[2].get_ydata(),
                                       np.degrees(log['qd_raw'][::2, index]))
            np.testing.assert_allclose(axes[1].lines[1].get_ydata(),
                                       np.degrees((log['qd_raw'] - qd_ref)[::2, index]))
        error = np.degrees((qd - qd_ref)[:, index])
        assert axes[1].lines[0].get_label() == (
            f'RMS={np.sqrt(np.mean(error ** 2)):.3f}°/s, '
            f'MAX={np.max(np.abs(error)):.3f}°/s')
        assert axes[0].texts[0].get_text() == 'TRACK'
        assert 'độ/s' in axes[0].get_ylabel()
        assert 'độ/s' in axes[1].get_ylabel()


def test_torque_wheel_zoom_and_legend_toggle():
    figure, axis = plt.subplots()
    try:
        curve, = axis.plot([0, 10], [0, 4], label='tau_fb')
        legend = axis.legend()
        enable_torque_interaction(figure, axis, legend)
        figure.canvas.draw()
        original_x, original_y = axis.get_xlim(), axis.get_ylim()
        x, y = axis.transData.transform((4, 2))
        event = MouseEvent('scroll_event', figure.canvas, x, y, button='up', step=1)
        figure.canvas.callbacks.process('scroll_event', event)
        np.testing.assert_allclose(np.diff(axis.get_xlim()), np.diff(original_x) / 1.25)
        np.testing.assert_allclose(np.diff(axis.get_ylim()), np.diff(original_y) / 1.25)
        event = MouseEvent('scroll_event', figure.canvas, x, y, button='down', step=-1)
        figure.canvas.callbacks.process('scroll_event', event)
        np.testing.assert_allclose(axis.get_xlim(), original_x)
        np.testing.assert_allclose(axis.get_ylim(), original_y)
        handle = legend.get_lines()[0]
        event = PickEvent('pick_event', figure.canvas, event, handle)
        figure.canvas.callbacks.process('pick_event', event)
        assert not curve.get_visible()
        assert handle.get_alpha() == 0.2
        figure.canvas.callbacks.process('pick_event', event)
        assert curve.get_visible()
        assert handle.get_alpha() == 1.0
    finally:
        plt.close(figure)
