"""Independent velocity/acceleration Kalman estimators, one per joint.

Only driver velocity is observed; position and the reference are not filtered.
Process noise is a constant random jerk over each measurement interval.
"""

import numpy as np
from scipy.linalg import solve_discrete_are


class VelocityAccelerationKalman:
    def __init__(self, measurement_std, jerk_std, nominal_dt, reset_gap_s=0.05):
        self.measurement_std = np.asarray(measurement_std, dtype=float)
        self.jerk_std = np.asarray(jerk_std, dtype=float)
        if (self.measurement_std.ndim != 1 or not self.measurement_std.size
                or self.jerk_std.shape != self.measurement_std.shape
                or not np.all(np.isfinite(self.measurement_std))
                or not np.all(np.isfinite(self.jerk_std))
                or np.any(self.measurement_std <= 0) or np.any(self.jerk_std <= 0)):
            raise ValueError('Kalman measurement_std/jerk_std cần vector hữu hạn > 0')
        self.n = len(self.measurement_std)
        self.nominal_dt = float(nominal_dt)
        self.reset_gap_s = float(reset_gap_s)
        if (not np.isfinite(self.nominal_dt) or self.nominal_dt <= 0
                or not np.isfinite(self.reset_gap_s)
                or self.reset_gap_s <= self.nominal_dt):
            raise ValueError('Kalman cần 0 < nominal_dt < reset_gap_s hữu hạn')
        self.R = self.measurement_std ** 2
        f, process = self._model(self.nominal_dt)
        h = np.array([[1., 0.]])
        self._initial_covariance = []
        self.gain = []
        for noise, r in zip(process, self.R):
            prior = solve_discrete_are(f.T, h.T, noise, np.array([[r]]))
            gain = prior[:, 0] / (prior[0, 0] + r)
            residual = np.eye(2) - np.outer(gain, h[0])
            self._initial_covariance.append(
                residual @ prior @ residual.T + r * np.outer(gain, gain))
            self.gain.append(gain)
        self._initial_covariance = np.asarray(self._initial_covariance)
        self.gain = np.asarray(self.gain)
        self.reset()

    def _model(self, dt):
        f = np.array([[1., dt], [0., 1.]])
        g = np.array([0.5 * dt * dt, dt])
        return f, self.jerk_std[:, None, None] ** 2 * np.outer(g, g)

    def reset(self):
        self.state = np.zeros((self.n, 2))
        self.covariance = self._initial_covariance.copy()
        self.timestamp = None

    def update(self, velocity, timestamp):
        velocity = np.asarray(velocity, dtype=float)
        timestamp = float(timestamp)
        if (velocity.shape != (self.n,) or not np.all(np.isfinite(velocity))
                or not np.isfinite(timestamp)):
            raise ValueError('Kalman nhận vận tốc/timestamp không hợp lệ')
        dt = None if self.timestamp is None else timestamp - self.timestamp
        if dt == 0:
            return self.state[:, 0].copy()
        if dt is None or dt < 0 or dt > self.reset_gap_s:
            # Seed from measurement; never start a moving joint at zero speed.
            self.state[:, 0] = velocity
            self.state[:, 1] = 0.
            self.covariance = self._initial_covariance.copy()
        else:
            f, process = self._model(dt)
            prior_state = self.state @ f.T
            prior = f @ self.covariance @ f.T + process
            gain = prior[:, :, 0] / (prior[:, 0, 0] + self.R)[:, None]
            self.state = prior_state + gain * (velocity - prior_state[:, 0])[:, None]
            residual = np.eye(2)[None, :, :] - gain[:, :, None] * np.array([1., 0.])
            self.covariance = (residual @ prior @ residual.transpose(0, 2, 1)
                               + self.R[:, None, None]
                               * gain[:, :, None] * gain[:, None, :])
        self.timestamp = timestamp
        return self.state[:, 0].copy()

    def steady_state_model(self):
        """z[k] = Af*z[k-1] + Bf*v_raw[k]; v_filtered[k] = E*z[k]."""
        f, _ = self._model(self.nominal_dt)
        af, bf, e = np.zeros((2*self.n, 2*self.n)), np.zeros((2*self.n, self.n)), np.zeros((self.n, 2*self.n))
        for i, gain in enumerate(self.gain):
            sl = slice(2*i, 2*i+2)
            af[sl, sl] = (np.eye(2) - np.outer(gain, [1., 0.])) @ f
            bf[sl, i] = gain
            e[i, 2*i] = 1.
        return af, bf, e
