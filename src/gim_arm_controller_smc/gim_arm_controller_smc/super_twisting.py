"""STA reaching law adapted from matlab-sim/urdf_SMC/sta_core.m (main c50c596).

Explicit sampled super-twisting in acceleration units, with reference-state
feedforward, torque limits and conditional integration.
"""

import numpy as np

from .controller import SmcController
from .velocity_kalman import VelocityKalman2


class SuperTwistingController(SmcController):
    """s = de + lambda*e; a_fb = -lambda*de - k1*sqrt(|s|)*sign(s) + z.

    z_dot = -k2*sign(s), integrated after computing this sample's torque.
    This is the standard sign-based law, without a saturation boundary layer.
    Explicit sampling produces a finite ripple around the sliding surface.
    """

    def __init__(self, dynamics, lambda_gain, k1, k2, z_limit,
                 velocity_lpf_hz=None, friction_ff_nm=None,
                 friction_transition_rad_s=0.005, velocity_kalman_enabled=None,
                 velocity_kalman_jerk_psd=None,
                 velocity_kalman_measurement_variance=None, **kwargs):
        for name, value in (('k1', k1), ('k2', k2), ('z_limit', z_limit)):
            vector = np.asarray(value, dtype=float)
            if (vector.shape != (dynamics.nq,) or not np.all(np.isfinite(vector))
                    or np.any(vector <= 0)):
                raise ValueError(f'{name} cần một số hữu hạn > 0 cho mỗi khớp')
            setattr(self, name, vector)
        super().__init__(dynamics, lambda_gain, np.zeros(dynamics.nq),
                         np.zeros(dynamics.nq), np.ones(dynamics.nq), **kwargs)
        self.velocity_lpf_hz = np.zeros(dynamics.nq) if velocity_lpf_hz is None else np.asarray(
            velocity_lpf_hz, dtype=float)
        if (self.velocity_lpf_hz.shape != (dynamics.nq,)
                or not np.all(np.isfinite(self.velocity_lpf_hz))
                or np.any(self.velocity_lpf_hz < 0)):
            raise ValueError('velocity_lpf_hz cần hữu hạn và >= 0 cho mỗi khớp')
        self.velocity_kalman = VelocityKalman2(
            dynamics.nq, velocity_kalman_enabled, velocity_kalman_jerk_psd,
            velocity_kalman_measurement_variance)
        if np.any(self.velocity_kalman.enabled & (self.velocity_lpf_hz > 0)):
            raise ValueError('Không bật LPF và Kalman cùng một khớp')
        self.friction_ff_nm = np.zeros(dynamics.nq) if friction_ff_nm is None else np.asarray(
            friction_ff_nm, dtype=float)
        if (self.friction_ff_nm.shape != (dynamics.nq,)
                or not np.all(np.isfinite(self.friction_ff_nm))
                or np.any(self.friction_ff_nm < 0)):
            raise ValueError('friction_ff_nm cần hữu hạn và >= 0 cho mỗi khớp')
        self.friction_transition_rad_s = float(friction_transition_rad_s)
        if (not np.isfinite(self.friction_transition_rad_s)
                or self.friction_transition_rad_s <= 0):
            raise ValueError('friction_transition_rad_s cần hữu hạn và > 0')
        self.filtered_qd = np.zeros(dynamics.nq)

    def reset(self):
        super().reset()
        self.z = np.zeros(self.dyn.nq)
        self.filtered_qd = np.zeros(self.dyn.nq)
        self._velocity_sample_time_ns = None
        self._previous_kalman_time_ns = None
        if hasattr(self, 'velocity_kalman'):
            self.velocity_kalman.reset()

    def set_velocity_sample_time(self, timestamp_ns):
        """Consumed ROS state timestamp; estimator time may differ from control dt."""
        self._velocity_sample_time_ns = int(timestamp_ns)

    def _filter_velocity(self, qd, dt):
        alpha = -np.expm1(-2.0 * np.pi * self.velocity_lpf_hz * dt)
        filtered = np.where(
            self.velocity_lpf_hz > 0,
            self.filtered_qd + alpha * (qd - self.filtered_qd), qd)
        if self.velocity_kalman.enabled.any():
            stamp, previous = self._velocity_sample_time_ns, self._previous_kalman_time_ns
            if stamp is not None and stamp == previous:
                # A repeated compute on the same ROS state is not another
                # independent measurement. Preserve the previous posterior.
                estimate = self.filtered_qd
            else:
                sample_dt = dt if stamp is None or previous is None else (stamp - previous) * 1e-9
                if sample_dt <= 0. or sample_dt > 0.25:
                    # Clock rewind or a long state gap invalidates acceleration.
                    self.velocity_kalman.reset()
                    sample_dt = dt
                estimate = self.velocity_kalman.update(qd, sample_dt)
                self._previous_kalman_time_ns = stamp
            filtered = np.where(self.velocity_kalman.enabled, estimate, filtered)
        self.filtered_qd = filtered
        return filtered

    def sampled_spectral_radius(self):
        # The saturation controller's linear boundary-layer test does not
        # apply to sqrt/sign dynamics with an additional integral state.
        return None

    def _reaching_acceleration(self, surface):
        switching = np.sign(surface)
        return -self.k1 * np.sqrt(np.abs(surface)) * switching + self.z, switching

    def compute(self, q, qd, q_ref, qd_ref, qdd_ref, dt):
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('dt phải hữu hạn và > 0')
        qd = np.asarray(qd, dtype=float)
        filtered_qd = self._filter_velocity(qd, dt)
        tau = super().compute(q, filtered_qd, q_ref, qd_ref, qdd_ref, dt)
        # Smooth Coulomb-friction feedforward follows the reference direction;
        # it is zero at rest and does not amplify measured velocity noise.
        friction = np.where(
            self.active,
            self.friction_ff_nm * np.tanh(
                np.asarray(qd_ref, dtype=float) / self.friction_transition_rad_s), 0.)
        tau_ff = self.last['tau_ff'] + friction
        tau_raw = self.last['tau_raw'] + friction
        tau = np.clip(tau_raw, -self.tau_limit, self.tau_limit)
        self.previous_feedback = tau - tau_ff
        self.last['tau_ff'] = tau_ff
        self.last['tau_raw'] = tau_raw
        self.last['tau'] = tau
        self.last['saturated'] = np.abs(tau_raw - tau) > 1e-12
        self.last['tau_friction'] = friction
        self.last['qd_filtered'] = self.filtered_qd.copy()
        self.last['integral'] = self.z.copy()  # z used for this torque, rad/s^2
        increment = np.where(self.active, -self.k2 * self.last['boundary'] * dt, 0.)
        # Each z_j drives torques through column j of the feedback mapping.
        # In diagonal mode only its own torque can block this integrator.
        # Freeze updates that push a constrained torque further into clipping/slew.
        residual = (self.last['tau_feedback_raw'] - self.last['tau_fb']
                    + self.last['tau_raw'] - tau)
        effective_mass = self._feedback_mass_matrix(q)
        # Attribute the three additive acceleration terms BEFORE the nonlinear
        # slew/torque constraints. k2 contributes via the current z (not z_dot
        # or the state updated for the next sample). Lambda also enters s, so
        # tau_lambda is the explicit -lambda*de term, not its total sensitivity.
        accelerations = np.column_stack([
            np.where(self.active, -self.lambda_gain * self.last['error_rate'], 0.),
            np.where(self.active, -self.k1 * np.sqrt(np.abs(self.last['surface']))
                     * self.last['boundary'], 0.),
            np.where(self.active, self.last['integral'], 0.),
        ])
        torque_components = effective_mass @ accelerations
        for column, name in enumerate(('tau_lambda', 'tau_k1', 'tau_k2')):
            self.last[name] = torque_components[:, column].copy()
        self.last['tau_slew'] = self.last['tau_fb'] - self.last['tau_feedback_raw']
        contributions = effective_mass * increment[np.newaxis, :]
        blocked = np.any(residual[:, np.newaxis] * contributions > 1e-12, axis=0)
        self.z = np.where(self.active, np.clip(
            self.z + np.where(blocked, 0., increment), -self.z_limit, self.z_limit), 0.)
        self.last['integral_blocked'] = blocked
        return tau

    def describe(self, q_nominal=None):
        feedback_mapping = ('diag(M(q_measured))' if not self.feedback_offdiag_scale.any()
                            else 'M_eff(q_measured)')
        return (
            'SMC super-twisting: s=de+lambda*e; '
            'a_fb=-lambda*de-k1*sqrt(abs(s))*sign(s)+z; z_dot=-k2*sign(s)\n'
            f'  lambda={self.lambda_gain}, k1={self.k1}, k2={self.k2}, '
            f'z_limit={self.z_limit} rad/s^2; explicit Euler, dt={self.dt:g}s\n'
            '  tau_ff=inverse_dynamics(q_ref, qdot_ref, qdd_ref); '
            f'tau_fb_raw={feedback_mapping} @ a_fb, then feedback slew and total torque limits\n'
            '  ZOH boundary-layer test is inapplicable; validate sampled tracking\n'
            f'  controlled_joint={self.controlled_joint}, base_ff_alpha={self.base_ff_alpha}, '
            f'velocity_lpf_hz={self.velocity_lpf_hz}, friction_ff_nm={self.friction_ff_nm}, '
            f'velocity_kalman_enabled={self.velocity_kalman.enabled}, '
            f'kalman_jerk_psd={self.velocity_kalman.jerk_psd}, '
            f'kalman_measurement_variance={self.velocity_kalman.measurement_variance}, '
            f'friction_transition_rad_s={self.friction_transition_rad_s}, '
            f'feedback_slew={self.slew}, '
            f'feedback_offdiag_scale={self.feedback_offdiag_scale}, '
            f'tau_limit={self.tau_limit} Nm')
