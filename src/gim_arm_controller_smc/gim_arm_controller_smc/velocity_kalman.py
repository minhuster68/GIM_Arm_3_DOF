"""Independent causal Kalman filters with state [velocity, acceleration].

The measurement is driver velocity, H=[1, 0]. Acceleration is modelled as a
random walk driven by continuous white jerk with spectral density jerk_psd:
F=[[1, dt], [0, 1]], Q=jerk_psd*[[dt**3/3, dt**2/2], [dt**2/2, dt]].
No reference trajectory or future measurement enters the estimate.
"""

import numpy as np


class VelocityKalman2:
    def __init__(self, nq, enabled=None, jerk_psd=None, measurement_variance=None):
        self.nq = nq
        self.enabled = np.zeros(nq, dtype=bool) if enabled is None else np.asarray(enabled)
        if self.enabled.shape != (nq,) or self.enabled.dtype.kind != 'b':
            raise ValueError('velocity_kalman_enabled cần một bool cho mỗi khớp')
        self.jerk_psd = self._positive_vector(
            jerk_psd, 1.0, 'velocity_kalman_jerk_psd')
        self.measurement_variance = self._positive_vector(
            measurement_variance, 0.0004, 'velocity_kalman_measurement_variance')
        self.reset()

    def _positive_vector(self, value, default, name):
        vector = np.full(self.nq, default) if value is None else np.asarray(value, dtype=float)
        if (vector.shape != (self.nq,) or not np.all(np.isfinite(vector))
                or np.any(vector <= 0)):
            raise ValueError(f'{name} cần một số hữu hạn > 0 cho mỗi khớp')
        return vector.copy()

    def reset(self):
        self.state = np.zeros((self.nq, 2))
        self.covariance = np.zeros((self.nq, 2, 2))
        self.initialized = np.zeros(self.nq, dtype=bool)

    def update(self, measured_velocity, dt):
        measurement = np.asarray(measured_velocity, dtype=float)
        dt = float(dt)
        if measurement.shape != (self.nq,) or not np.all(np.isfinite(measurement)):
            raise ValueError('Kalman cần vận tốc đo hữu hạn cho mỗi khớp')
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('Kalman dt phải hữu hạn và > 0')
        result = measurement.copy()
        transition = np.array([[1., dt], [0., 1.]])
        process = np.array([[dt**3 / 3., dt**2 / 2.], [dt**2 / 2., dt]])
        for index in np.flatnonzero(self.enabled):
            variance = self.measurement_variance[index]
            if not self.initialized[index]:
                # Seed from the first measurement, not zero or qd_ref. Initial
                # acceleration std is 1 rad/s^2; no fictitious start-up ramp.
                self.state[index] = [measurement[index], 0.]
                self.covariance[index] = np.diag([variance, 1.])
                self.initialized[index] = True
            else:
                predicted = transition @ self.state[index]
                prior = (transition @ self.covariance[index] @ transition.T
                         + self.jerk_psd[index] * process)
                gain = prior[:, 0] / (prior[0, 0] + variance)
                self.state[index] = predicted + gain * (measurement[index] - predicted[0])
                # Joseph update preserves positive covariance under roundoff.
                residual_map = np.eye(2) - np.outer(gain, [1., 0.])
                posterior = (residual_map @ prior @ residual_map.T
                             + variance * np.outer(gain, gain))
                self.covariance[index] = 0.5 * (posterior + posterior.T)
            result[index] = self.state[index, 0]
        return result
