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


def test_torque_plot_shows_post_controller_gravity_correction(tmp_path, monkeypatch):
    count = 4
    ff = np.full((count, 3), 1.4)
    fb = np.full((count, 3), 0.1)
    raw = ff + fb
    applied = raw - 0.31
    log = {
        't': np.arange(count, dtype=float),
        'phase': np.array(['TRACK'] * count),
        'q': np.zeros((count, 3)),
        'q_ref': np.zeros((count, 3)),
        'qd': np.zeros((count, 3)),
        'qd_ref': np.zeros((count, 3)),
        'tau': applied,
        'tau_ff': ff,
        'tau_fb': fb,
        'tau_raw': raw,
    }
    figures = {}
    original_save = Figure.savefig

    def capture_save(figure, path, **kwargs):
        figures[str(path)] = figure
        original_save(figure, path, **kwargs)

    monkeypatch.setattr(Figure, 'savefig', capture_save)
    plot(log, str(tmp_path / 'torque.png'), False, max_plot_points=count)
    axes = figures[str(tmp_path / 'torque_q3_elbow.png')].axes[2]
    lines = {line.get_label(): line.get_ydata() for line in axes.lines}
    np.testing.assert_allclose(lines['tau_post (gravity_scale/kẹp)'], -0.31)
    np.testing.assert_allclose(lines['MAX |tau|=1.190 Nm'],
                               lines['tau_ff'] + lines['tau_fb']
                               + lines['tau_post (gravity_scale/kẹp)'])


def test_velocity_plot_compares_filtered_signal_without_replacing_raw(tmp_path, monkeypatch):
    count = 5
    raw = np.tile([.1, .2, .3], (count, 1))
    filtered = raw.copy()
    filtered[:, 2] = .15
    log = dict(t=np.arange(count), phase=np.array(['TRACK'] * count),
               q=np.zeros((count, 3)), q_ref=np.zeros((count, 3)),
               qd=raw, qd_ref=np.zeros((count, 3)), qd_filtered=filtered,
               tau=np.zeros((count, 3)))
    figures = {}
    original_save = Figure.savefig

    def capture_save(figure, path, **kwargs):
        figures[str(path)] = figure
        original_save(figure, path, **kwargs)

    monkeypatch.setattr(Figure, 'savefig', capture_save)
    plot(log, str(tmp_path / 'kf.png'), False, max_plot_points=count)
    axes = figures[str(tmp_path / 'kf_q3_elbow_velocity.png')].axes
    lines = {line.get_label(): line.get_ydata() for line in axes[0].lines}
    np.testing.assert_allclose(lines['Thực tế'], np.degrees(raw[:, 2]))
    np.testing.assert_allclose(lines['Sau lọc (đưa vào điều khiển)'], np.degrees(filtered[:, 2]))
    errors = {line.get_label(): line.get_ydata() for line in axes[1].lines}
    np.testing.assert_allclose(errors['Sai số vận tốc sau lọc'], np.degrees(filtered[:, 2]))
    base_lines = figures[str(tmp_path / 'kf_q1_base_velocity.png')].axes[0].lines
    assert 'Sau lọc (đưa vào điều khiển)' not in [line.get_label() for line in base_lines]


@pytest.mark.parametrize('finite_components', [True, False])
def test_sta_torque_split_replaces_aggregate_and_sums_to_command(tmp_path, monkeypatch, finite_components):
    count = 4
    full = lambda value: np.full((count, 3), value, dtype=float)
    log = dict(t=np.arange(count), phase=np.array(['TRACK'] * count),
               q=full(0), q_ref=full(0), qd=full(0), qd_ref=full(0),
               tau_ff=full(1.4), tau_fb=full(-.2), tau_raw=full(1.2), tau=full(1.1),
               tau_lambda=full(.3), tau_k1=full(-.8), tau_k2=full(.2), tau_slew=full(.1))
    if not finite_components:
        for key in ('tau_lambda', 'tau_k1', 'tau_k2', 'tau_slew'):
            log[key][:] = np.nan  # SMC saturation uses the same CSV schema.
    figures = {}
    original_save = Figure.savefig

    def capture_save(figure, path, **kwargs):
        figures[str(path)] = figure
        original_save(figure, path, **kwargs)

    monkeypatch.setattr(Figure, 'savefig', capture_save)
    plot(log, str(tmp_path / 'sta.png'), False, max_plot_points=count)
    lines = {line.get_label(): line.get_ydata()
             for line in figures[str(tmp_path / 'sta_q3_elbow.png')].axes[2].lines}
    if finite_components:
        assert 'tau_fb' not in lines
        np.testing.assert_allclose(lines['tau_lambda (-lambda*de)'], .3)
        np.testing.assert_allclose(lines['tau_k1'], -.8)
        np.testing.assert_allclose(lines['tau_k2 (từ z)'], .2)
        np.testing.assert_allclose(lines['MAX |tau|=1.100 Nm'],
            lines['tau_ff'] + lines['tau_lambda (-lambda*de)'] + lines['tau_k1']
            + lines['tau_k2 (từ z)'] + lines['tau_slew (hiệu chỉnh giới hạn)']
            + lines['tau_post (gravity_scale/kẹp)'])
    else:
        assert 'tau_fb' in lines
        assert 'tau_k1' not in lines
