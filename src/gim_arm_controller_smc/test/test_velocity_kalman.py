"""Causal noisy-sensor estimation, independent axes and numerical validity."""

import numpy as np
import pytest

from gim_arm_controller_smc.velocity_kalman import VelocityKalman2


def test_noisy_ramp_with_irregular_samples_and_independent_axes():
    rng = np.random.default_rng(42)
    f = VelocityKalman2(3, [False, False, True], [1., 1., .02], [.0004] * 3)
    truth, measured, filtered = [], [], []
    elapsed = 0.
    for index in range(1200):
        dt = rng.uniform(.006, .018)
        elapsed += dt
        velocity = .15 + .3 * elapsed
        sample = np.array([index * .01, -index * .02, velocity + rng.normal(0., .02)])
        before = sample.copy()
        estimate = f.update(sample, dt)
        np.testing.assert_array_equal(sample, before)
        np.testing.assert_array_equal(estimate[:2], sample[:2])
        np.testing.assert_allclose(f.covariance[2], f.covariance[2].T, atol=1e-14)
        assert np.linalg.eigvalsh(f.covariance[2]).min() >= -1e-14
        if index > 200:
            truth.append(velocity); measured.append(sample[2]); filtered.append(estimate[2])
    raw_rms = np.sqrt(np.mean((np.asarray(measured) - truth)**2))
    filtered_rms = np.sqrt(np.mean((np.asarray(filtered) - truth)**2))
    assert filtered_rms < .6 * raw_rms
    assert abs(f.state[2, 1] - .3) < .15


def test_first_sample_and_reset_do_not_pull_velocity_to_zero():
    f = VelocityKalman2(3, [False, False, True])
    np.testing.assert_array_equal(f.update([1., 2., .8], .01), [1., 2., .8])
    for _ in range(20):
        f.update([1., 2., -.5], .01)
    f.reset()
    np.testing.assert_array_equal(f.update([0., 0., -.3], .01), [0., 0., -.3])
    assert f.state[2, 1] == 0.


@pytest.mark.parametrize('kwargs', [
    {'enabled': [0, 0, 1]}, {'enabled': [True]},
    {'jerk_psd': [1., 1., 0.]}, {'jerk_psd': [float('nan')] * 3},
    {'measurement_variance': [-1.] * 3}, {'measurement_variance': [float('inf')] * 3},
])
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        VelocityKalman2(3, **kwargs)


@pytest.mark.parametrize('sample,dt', [([0., 0., 0.], 0.), ([0., 0., 0.], float('nan')),
                                      ([0., 0., float('nan')], .01), ([0.], .01)])
def test_invalid_measurement_leaves_posterior_unchanged(sample, dt):
    f = VelocityKalman2(3, [False, False, True])
    f.update([0., 0., .1], .01)
    state, covariance = f.state.copy(), f.covariance.copy()
    with pytest.raises(ValueError):
        f.update(sample, dt)
    np.testing.assert_array_equal(f.state, state)
    np.testing.assert_array_equal(f.covariance, covariance)
