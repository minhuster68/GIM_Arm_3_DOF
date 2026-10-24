"""A stationary driver with alternating velocity noise must not mask motion."""
from collections import deque
import math

from gim_control.mpc_joint_trial import append_stationary_sample, stationary_window


def samples(speed=0.0, noise=0.0):
    result = deque()
    for i in range(41):
        t = i * .01
        append_stationary_sample(result, t, [0., 0., speed*t],
                                 [0., 0., speed + noise*(-1)**i])
    return result


def test_held_joint_alternating_noise_passes_but_selected_raw_spike_does_not():
    window = samples(noise=.3)
    assert stationary_window(window, selected=1)
    assert not stationary_window(window, selected=2)


def test_drift_and_biased_velocity_still_reject():
    assert not stationary_window(samples(speed=.15), selected=1)
    window = samples()
    window[-1] = (window[-1][0], [0., 0., .03], [0., 0., 0.])
    assert not stationary_window(window, selected=1)
    window = samples()
    window = deque((t, q, [0., 0., .2]) for t, q, _ in window)
    assert not stationary_window(window, selected=1)


def test_stale_short_or_invalid_windows_reject():
    assert not stationary_window(list(samples())[:2], selected=1)
    window = list(samples())
    del window[5:15]
    assert not stationary_window(window, selected=1)
    window = samples()
    window[-1] = (window[-1][0], [0., math.nan, 0.], [0., 0., 0.])
    assert not stationary_window(window, selected=1)


def test_default_controller_start_gate_keeps_instantaneous_velocity_limit():
    from types import SimpleNamespace
    import numpy as np
    from gim_control.effort_controller_node import EffortControllerNode
    node = SimpleNamespace(start_velocity_limit=.05, qd=np.array([0., .15, 0.]))
    assert not EffortControllerNode._start_velocity_is_safe(node)
    node.qd[:] = 0.
    assert EffortControllerNode._start_velocity_is_safe(node)
