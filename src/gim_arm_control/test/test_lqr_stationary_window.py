"""Startup rejects position movement/biased velocity but tolerates encoder noise."""
from collections import deque
from types import SimpleNamespace

import numpy as np

from gim_control.lqr_joint_trial import append_hold_sample, stationary_hold_window
from gim_control.effort_controller_node import EffortControllerNode


def history(speed=0., noise=0., amplitude=0.):
    samples = deque()
    for i in range(41):
        t = .01*i
        append_hold_sample(samples, t, [0., 0., speed*t+amplitude*(-1)**i],
                           [0., 0., speed+noise*(-1)**i])
    return samples


def test_small_position_noise_with_zero_mean_velocity_can_start():
    assert stationary_hold_window(history(noise=.38, amplitude=.0016))


def test_drift_or_large_position_oscillation_cannot_start():
    assert not stationary_hold_window(history(speed=.06))
    assert not stationary_hold_window(history(noise=.38, amplitude=.004))
    samples = history()
    samples = deque((t, q, [0., 0., .15]) for t, q, _ in samples)
    assert not stationary_hold_window(samples)


def test_missing_stale_and_nonfinite_samples_cannot_start():
    assert not stationary_hold_window([])
    assert not stationary_hold_window(list(history())[:2])
    samples = list(history()); del samples[3:15]
    assert not stationary_hold_window(samples)
    samples = history(); samples[-1] = (samples[-1][0], [0., 0., np.nan], [0., 0., 0.])
    assert not stationary_hold_window(samples)


def test_regular_circle_runner_keeps_default_raw_start_velocity_gate():
    node = SimpleNamespace(qd=np.array([0., 0., .2]), start_velocity_limit=.05)
    assert not EffortControllerNode._start_velocity_is_safe(node)
    node.qd[:] = 0.
    assert EffortControllerNode._start_velocity_is_safe(node)
