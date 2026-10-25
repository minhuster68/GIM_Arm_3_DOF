"""Velocity observer behavior, independent joints, and production state ingress."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from gim_control.velocity_filter import VelocityAccelerationKalman


def make_filter():
    return VelocityAccelerationKalman([.03, .01, .04], [180., 60., 240.], .01)


def test_seed_constant_ramp_and_independent_joints():
    f = make_filter()
    np.testing.assert_array_equal(f.update([1., 2., 3.], 0.), [1., 2., 3.])
    for k in range(1, 501):
        # Known constant acceleration; no mixing between joints.
        result = f.update([1. + .2*k*.01, 2., 3.], k*.01)
    np.testing.assert_allclose(result, [2., 2., 3.], atol=1e-8)
    np.testing.assert_allclose(f.state[:, 1], [.2, 0., 0.], atol=1e-8)
    assert np.min(np.linalg.eigvalsh(f.covariance)) > 0


def test_noise_reduction_and_nominal_model_match_actual_updates():
    f = make_filter()
    rng = np.random.default_rng(4)
    raw = rng.normal(size=(3000, 3)) * f.measurement_std
    af, bf, e = f.steady_state_model()
    filtered = [f.update(raw[0], 0.)]
    for k, velocity in enumerate(raw[1:], 1):
        predicted_state = af @ f.state.ravel() + bf @ velocity
        filtered.append(f.update(velocity, k*.01))
        np.testing.assert_allclose(f.state.ravel(), predicted_state, atol=1e-10)
        np.testing.assert_allclose(filtered[-1], e @ predicted_state, atol=1e-10)
    assert np.all(np.std(filtered[100:], axis=0) < .85*np.std(raw[100:], axis=0))


def test_irregular_sampling_duplicate_gap_reversal_and_invalid_data():
    f = make_filter()
    f.update([1., 2., 3.], 1.)
    f.update([1.1, 2., 3.], 1.017)
    saved, covariance = f.state.copy(), f.covariance.copy()
    np.testing.assert_array_equal(f.update([9., 9., 9.], 1.017), saved[:, 0])
    np.testing.assert_array_equal(f.covariance, covariance)
    for bad in ([np.nan, 0., 0.], [np.inf, 0., 0.], [1., 2.]):
        with pytest.raises(ValueError):
            f.update(bad, 1.02)
        np.testing.assert_array_equal(f.state, saved)
    for timestamp in (1.2, .5):
        np.testing.assert_array_equal(f.update([4., 5., 6.], timestamp), [4., 5., 6.])
        np.testing.assert_array_equal(f.state[:, 1], 0.)


@pytest.mark.parametrize('noise,jerk,dt,gap', [([0.], [1.], .01, .05),
    ([.1], [-1.], .01, .05), ([.1], [1.], 0., .05),
    ([.1], [1.], .01, .001), ([.1], [float('nan')], .01, .05)])
def test_invalid_configuration(noise, jerk, dt, gap):
    with pytest.raises(ValueError):
        VelocityAccelerationKalman(noise, jerk, dt, gap)


def test_raw_state_to_filtered_control_and_raw_motion_gate():
    from gim_control.effort_controller_node import EffortControllerNode
    node = SimpleNamespace(
        joint_names=['base_joint', 'shoulder_joint', 'elbow_joint'], n=3,
        velocity_filter=make_filter(), filter_rx_monotonic=None,
        state_header_ns=-1, state_seq=0, diagnostic_lqr=True,
        phase='TRACK', state_rows=[], start_velocity_limit=.05,
        get_clock=lambda: SimpleNamespace(now=lambda: object()), _abort=Mock())

    def message(nsec, velocity):
        return SimpleNamespace(name=node.joint_names, position=[0.]*3,
            velocity=velocity, effort=[0.]*3,
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=nsec)))

    EffortControllerNode._on_state(node, message(0, [0.]*3))
    EffortControllerNode._on_state(node, message(10000000, [.06, 0., 0.]))
    assert 0 < node.qd[0] < .05 < node.qd_raw[0]
    assert not EffortControllerNode._start_velocity_is_safe(node)
    np.testing.assert_array_equal(node.state_rows[-1][5:8], node.qd_raw)
    before = node.qd.copy()
    EffortControllerNode._on_state(node, message(10000000, [9.]*3))
    assert node.state_seq == 2
    np.testing.assert_array_equal(node.qd, before)
    EffortControllerNode._on_state(node, message(20000000, [float('nan')]*3))
    node._abort.assert_called_once()
    np.testing.assert_array_equal(node.qd, before)
