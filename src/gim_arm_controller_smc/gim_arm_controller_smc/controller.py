"""SMC with reference feedforward and measured-state error feedback."""

import numpy as np


class SmcController:
    def __init__(
            self, dynamics, lambda_gain, ks, kr, phi,
            control_hz=100.0, tau_limit=None, base_ff_alpha=1.0,
            controlled_joint='all', torque_slew_rate=None,
            feedback_offdiag_scale=None):
        self.dyn = dynamics
        self.dt = 1.0 / float(control_hz)
        self.lambda_gain = np.asarray(lambda_gain, dtype=float)
        self.ks = np.asarray(ks, dtype=float)
        self.kr = np.asarray(kr, dtype=float)
        self.phi = np.asarray(phi, dtype=float)
        self.tau_limit = np.asarray(
            dynamics.tau_max if tau_limit is None else tau_limit, dtype=float)
        self.base_ff_alpha = float(base_ff_alpha)
        if not np.isfinite(self.base_ff_alpha) or self.base_ff_alpha <= 0:
            raise ValueError('base_ff_alpha phải hữu hạn và > 0')
        names = list(dynamics.joint_names)
        self.base_index = names.index('base_joint') if 'base_joint' in names else None
        self.controlled_joint = str(controlled_joint)
        self.active = np.ones(dynamics.nq, dtype=bool)
        if self.controlled_joint != 'all':
            name = self.controlled_joint + '_joint'
            if name not in names:
                raise ValueError('controlled_joint phải là all, base, shoulder hoặc elbow')
            self.active[:] = False
            self.active[names.index(name)] = True
        self.slew = None if torque_slew_rate is None else np.asarray(torque_slew_rate, dtype=float)
        if self.slew is not None and (
                self.slew.shape != (dynamics.nq,) or not np.all(np.isfinite(self.slew))
                or np.any(self.slew <= 0)):
            raise ValueError('torque_slew_rate cần một số hữu hạn > 0 cho mỗi khớp')
        self.feedback_offdiag_scale = (
            np.ones(dynamics.nq) if feedback_offdiag_scale is None
            else np.asarray(feedback_offdiag_scale, dtype=float))
        if (self.feedback_offdiag_scale.shape != (dynamics.nq,)
                or not np.all(np.isfinite(self.feedback_offdiag_scale))
                or np.any(self.feedback_offdiag_scale < 0.)
                or np.any(self.feedback_offdiag_scale > 1.)):
            raise ValueError('feedback_offdiag_scale cần một số trong [0, 1] cho mỗi khớp')
        self.reset()

    def reset(self):
        self.last = {}
        self.previous_feedback = np.zeros(self.dyn.nq)

    def sampled_spectral_radius(self):
        """Estimate sampled stability for the ideal computed-torque plant."""
        # Inside the saturation boundary, the reaching law is linear:
        # e_ddot + (lambda + alpha)e_dot + lambda*alpha*e = 0,
        # alpha = ks + kr/phi.  Apply a ZOH to the acceleration command over
        # one sample and inspect the resulting double-integrator map.
        alpha = self.ks + self.kr / self.phi
        radii = []
        for lambda_value, alpha_value in zip(self.lambda_gain, alpha):
            kp = lambda_value * alpha_value
            kd = lambda_value + alpha_value
            matrix = np.array([
                [1.0 - 0.5 * kp * self.dt**2,
                 self.dt - 0.5 * kd * self.dt**2],
                [-kp * self.dt, 1.0 - kd * self.dt],
            ])
            radii.append(float(np.max(np.abs(np.linalg.eigvals(matrix)))))
        return np.asarray(radii)

    def _reaching_acceleration(self, surface):
        boundary = np.clip(surface / self.phi, -1.0, 1.0)
        return -self.ks * surface - self.kr * boundary, boundary

    def _feedback_mass_matrix(self, q):
        """Map correction accelerations to torques, independently of feedforward.

        With all row scales zero, only M_ii(q) * a_fb_i reaches torque i.
        M_ii may still depend on the whole pose; physical coupling remains.
        STA diagnostics and anti-windup must use this same mapping.
        """
        mass = self.dyn.mass_matrix(q)
        diagonal = np.diag(np.diag(mass))
        return diagonal + self.feedback_offdiag_scale[:, None] * (mass - diagonal)

    def compute(self, q, qd, q_ref, qd_ref, qdd_ref, dt):
        q = np.asarray(q, dtype=float)
        qd = np.asarray(qd, dtype=float)
        q_ref = np.asarray(q_ref, dtype=float)
        qd_ref = np.asarray(qd_ref, dtype=float)
        qdd_ref = np.asarray(qdd_ref, dtype=float)

        error = q - q_ref
        error_rate = qd - qd_ref
        surface = error_rate + self.lambda_gain * error
        reaching, boundary = self._reaching_acceleration(surface)
        feedback_acceleration = (
            - self.lambda_gain * error_rate
            + reaching
        )
        # Position-held joints have zero commanded acceleration in this trial.
        # Their measured errors must not create a coupled SMC correction.
        feedback_acceleration = np.where(self.active, feedback_acceleration, 0.0)
        reference_acceleration = np.where(self.active, qdd_ref, 0.0)
        virtual_acceleration = reference_acceleration + feedback_acceleration
        # Feedforward follows the desired trajectory. Measured/filtered state
        # is used only for error feedback and its mass-matrix conversion.
        tau_ff = np.asarray(self.dyn.inverse_dynamics(
            q_ref, qd_ref, reference_acceleration)).copy()
        if self.base_index is not None:
            tau_ff[self.base_index] *= self.base_ff_alpha
        mass = self._feedback_mass_matrix(q)
        feedback_self = np.diag(mass) * feedback_acceleration
        # Log the cross torque actually applied after row scaling, rather than
        # the hypothetical full-matrix cross term (zero in independent mode).
        feedback_cross = mass @ feedback_acceleration - feedback_self
        feedback_raw = feedback_self + feedback_cross
        feedback = feedback_raw.copy()
        if self.slew is not None:
            change = self.slew * self.dt
            feedback = np.clip(feedback, self.previous_feedback - change,
                               self.previous_feedback + change)
        tau_raw = tau_ff + feedback
        tau = np.clip(tau_raw, -self.tau_limit, self.tau_limit)
        self.previous_feedback = tau - tau_ff
        self.last = {
            "error": error,
            "error_rate": error_rate,
            "surface": surface,
            "boundary": boundary,
            "virtual_acceleration": virtual_acceleration,
            "tau_raw": tau_raw,
            "tau": tau,
            "tau_ff": tau_ff,
            "tau_fb": feedback,
            "tau_feedback": feedback,
            "tau_feedback_raw": feedback_raw,
            "tau_feedback_self": feedback_self,
            "tau_feedback_cross": feedback_cross,
            "slew_limited": np.abs(feedback_raw - feedback) > 1e-12,
            "integral": np.zeros(self.dyn.nq),
            "integral_blocked": np.zeros(self.dyn.nq, dtype=bool),
            "saturated": np.abs(tau_raw - tau) > 1e-12,
        }
        return tau

    def describe(self, q_nominal=None):
        alpha = self.ks + self.kr / self.phi
        return (
            "SMC: s=error_rate+lambda*error; reaching law dùng saturation "
            "để giảm chattering\n"
            f"  lambda={self.lambda_gain}, ks={self.ks}, kr={self.kr}, "
            f"phi={self.phi}\n"
            f"  controlled_joint={self.controlled_joint}, base_ff_alpha={self.base_ff_alpha}, "
            f"feedback_slew={self.slew}, "
            f"feedback_offdiag_scale={self.feedback_offdiag_scale}\n"
            f"  alpha_boundary=ks+kr/phi={alpha}, "
            f"rho_ZOH_ideal={np.round(self.sampled_spectral_radius(), 4)}\n"
            f"  tau_ff=inverse_dynamics(q_ref, qdot_ref, qdd_ref); "
            f"tau=tau_ff+slew(M(q_do)*a_feedback), "
            f"tau_limit={self.tau_limit} Nm")
