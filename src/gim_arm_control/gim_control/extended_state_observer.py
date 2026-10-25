"""Position-based discrete LESO ported from minhuster68/matlab-sim, branch eso.

State per joint: [q_hat, v_hat, d_hat]; d_hat has units rad/s^2, not Nm.
The observer uses the previous interval's *applied* torque, not the reference
or the unconstrained controller output. See README_ESO.md for source and scope.
"""
import numpy as np


class DiscreteLeso:
    def __init__(self, bandwidth_rad_s):
        self.bandwidth = np.asarray(bandwidth_rad_s, dtype=float)
        if (self.bandwidth.ndim != 1 or not self.bandwidth.size
                or not np.all(np.isfinite(self.bandwidth)) or np.any(self.bandwidth <= 0)):
            raise ValueError('eso_bandwidth_rad_s cần vector hữu hạn > 0')
        self.n = len(self.bandwidth)
        self.reset()

    def reset(self):
        self.q_hat = self.v_hat = self.d_hat = None

    def gains(self, dt):
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('ESO dt cần hữu hạn > 0')
        p = np.exp(-self.bandwidth * dt)
        one_minus_p = -np.expm1(-self.bandwidth * dt)
        return (1 - p**3, 1.5 * one_minus_p**2 * (1 + p) / dt,
                one_minus_p**3 / dt**2)

    def seed(self, q, velocity=None):
        q = self._vector(q)
        velocity = np.zeros(self.n) if velocity is None else self._vector(velocity)
        self.q_hat, self.v_hat, self.d_hat = q.copy(), velocity.copy(), np.zeros(self.n)

    def _vector(self, value):
        value = np.asarray(value, dtype=float)
        if value.shape != (self.n,) or not np.all(np.isfinite(value)):
            raise ValueError('ESO cần vector đo/mô hình hữu hạn đúng số khớp')
        return value

    def update(self, q, nominal_acceleration_previous, dt):
        q, a = self._vector(q), self._vector(nominal_acceleration_previous)
        l1, l2, l3 = self.gains(dt)
        if self.q_hat is None:
            self.seed(q)
        else:
            acceleration = a + self.d_hat
            q_pred = self.q_hat + dt*self.v_hat + .5*dt*dt*acceleration
            v_pred = self.v_hat + dt*acceleration
            innovation = q - q_pred
            q_hat = q_pred + l1*innovation
            v_hat = v_pred + l2*innovation
            d_hat = self.d_hat + l3*innovation
            if not all(np.all(np.isfinite(v)) for v in (q_hat, v_hat, d_hat)):
                raise ValueError('ESO sinh NaN/Inf')
            self.q_hat, self.v_hat, self.d_hat = q_hat, v_hat, d_hat
        return self.q_hat.copy(), self.v_hat.copy(), self.d_hat.copy()


class EsoCompensator:
    """Bounded outer torque compensation; independent from controller.reset()."""
    def __init__(self, dynamics, tau_limit, bandwidth, compensation_gain,
                 compensation_limit, compensation_rate, warmup_s=1.5,
                 ramp_s=2., reset_gap_s=.05, nominal_input_scale=None):
        self.dyn, self.limit = dynamics, np.asarray(tau_limit, dtype=float)
        self.observer = DiscreteLeso(bandwidth)
        self.n = self.observer.n
        self.input_scale = (np.ones(self.n) if nominal_input_scale is None else
                            self.observer._vector(nominal_input_scale))
        if np.any(self.input_scale <= 0):
            raise ValueError('eso_nominal_input_scale cần hữu hạn > 0')
        self.gain = self.observer._vector(compensation_gain)
        self.comp_limit = self.observer._vector(compensation_limit)
        self.comp_rate = self.observer._vector(compensation_rate)
        if (np.any(self.gain < 0) or np.any(self.gain > 1)
                or np.any(self.comp_limit <= 0) or np.any(self.comp_limit > self.limit)
                or np.any(self.comp_rate <= 0)):
            raise ValueError('ESO gain cần [0,1], limit cần (0,tau_limit], rate cần > 0')
        self.warmup_s, self.ramp_s, self.reset_gap_s = map(float, (warmup_s, ramp_s, reset_gap_s))
        if (not np.all(np.isfinite([self.warmup_s, self.ramp_s, self.reset_gap_s]))
                or self.warmup_s < 1. or self.ramp_s <= 0 or self.reset_gap_s <= 0):
            raise ValueError('ESO cần warmup >= 1s, ramp/reset_gap > 0 hữu hạn')
        self.reset()

    def reset(self):
        self.observer.reset()
        self.timestamp_ns = None
        self.nominal_a_previous = None
        self.applied_sequence = None
        self.current_sequence = None
        self.warmup_elapsed = self.ramp_elapsed = 0.
        self.compensation = np.zeros(self.n)
        self.tau_dist_hat = np.zeros(self.n)
        self.status = 'WAIT_EFFORT_ALL'
        self.last = {}

    def prepare(self, q, raw_velocity, timestamp_ns, sequence, allowed):
        """Consume each fresh measurement once, after continuous torque handoff."""
        if not allowed:
            self.reset()
            return self.compensation.copy()
        if sequence == self.current_sequence:
            return self.compensation.copy()
        q, raw_velocity = self.observer._vector(q), self.observer._vector(raw_velocity)
        timestamp_ns = int(timestamp_ns)
        dt = (None if self.timestamp_ns is None else
              (timestamp_ns - self.timestamp_ns)*1e-9)
        if dt == 0:
            return self.compensation.copy()
        if dt is not None and (dt < 0 or dt > self.reset_gap_s):
            self.reset()
            dt = None
        previous_sequence = self.current_sequence
        self.current_sequence, self.timestamp_ns = sequence, timestamp_ns
        if dt is None:
            self.observer.seed(q, raw_velocity)
            self.status = 'WARMUP'
            return self.compensation.copy()
        self.warmup_elapsed += dt
        if self.warmup_elapsed < self.warmup_s:
            self.observer.seed(q, raw_velocity)
            self.status = 'WARMUP'
            return self.compensation.copy()
        if self.nominal_a_previous is None or self.applied_sequence != previous_sequence:
            # No applied command corresponding to the preceding sample: do not
            # fabricate acceleration from FF or from an ignored publication.
            self.observer.seed(q, raw_velocity)
            self.compensation.fill(0.)
            self.status = 'WAIT_APPLIED_COMMAND'
            return self.compensation.copy()
        self.observer.update(q, self.nominal_a_previous, dt)
        self.tau_dist_hat = self.input_scale * (self.dyn.mass_matrix(q) @ self.observer.d_hat)
        target = np.clip(-self.gain*self.tau_dist_hat, -self.comp_limit, self.comp_limit)
        self.ramp_elapsed += dt
        target *= min(1., self.ramp_elapsed/self.ramp_s)
        self.compensation = np.clip(target, self.compensation-self.comp_rate*dt,
                                    self.compensation+self.comp_rate*dt)
        self.status = 'ACTIVE'
        return self.compensation.copy()

    def record_applied(self, q, torque, sequence):
        if self.observer.q_hat is None or sequence != self.current_sequence:
            return
        # Same nominal model as MATLAB: a_nom=M(q)^-1*(tau_applied-C-G).
        # Friction/load/model error is left for the extended disturbance state.
        bias = self.dyn.inverse_dynamics(q, self.observer.v_hat, np.zeros(self.n))
        self.nominal_a_previous = np.linalg.solve(self.dyn.mass_matrix(q),
                                                  np.asarray(torque)/self.input_scale-bias)
        self.applied_sequence = sequence

    def apply(self, baseline, previous_command, actuator_rate, dt):
        baseline = self.observer._vector(baseline)
        requested = baseline + self.compensation
        applied = np.clip(requested, -self.limit, self.limit)
        rate = np.asarray(actuator_rate, dtype=float)
        if self.status == 'ACTIVE' and previous_command is not None:
            delta = np.where(rate > 0, rate*dt, np.inf)
            applied = np.clip(applied, previous_command-delta, previous_command+delta)
            applied = np.clip(applied, -self.limit, self.limit)
        self.last = dict(tau_pre_eso=baseline.copy(), tau_eso=self.compensation.copy(),
                         tau_with_eso_raw=requested,
                         tau_eso_actuator_adjustment=applied-requested,
                         tau_dist_hat=self.tau_dist_hat.copy())
        return applied
