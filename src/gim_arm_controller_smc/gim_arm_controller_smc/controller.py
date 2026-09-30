"""SMC bám quỹ đạo, cùng luật với setup_smc.m."""

import numpy as np


class SmcController:
    def __init__(
            self, dynamics, lambda_gain, ks, kr, phi,
            control_hz=100.0, tau_limit=None):
        self.dyn = dynamics
        self.dt = 1.0 / float(control_hz)
        self.lambda_gain = np.asarray(lambda_gain, dtype=float)
        self.ks = np.asarray(ks, dtype=float)
        self.kr = np.asarray(kr, dtype=float)
        self.phi = np.asarray(phi, dtype=float)
        self.tau_limit = np.asarray(
            dynamics.tau_max if tau_limit is None else tau_limit, dtype=float)
        self.reset()

    def reset(self):
        self.last = {}

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

    def compute(self, q, qd, q_ref, qd_ref, qdd_ref, dt):
        q = np.asarray(q, dtype=float)
        qd = np.asarray(qd, dtype=float)
        q_ref = np.asarray(q_ref, dtype=float)
        qd_ref = np.asarray(qd_ref, dtype=float)
        qdd_ref = np.asarray(qdd_ref, dtype=float)

        error = q - q_ref
        error_rate = qd - qd_ref
        surface = error_rate + self.lambda_gain * error
        boundary = np.clip(surface / self.phi, -1.0, 1.0)
        virtual_acceleration = (
            qdd_ref
            - self.lambda_gain * error_rate
            - self.ks * surface
            - self.kr * boundary
        )
        tau_raw = self.dyn.inverse_dynamics(q, qd, virtual_acceleration)
        tau = np.clip(tau_raw, -self.tau_limit, self.tau_limit)
        self.last = {
            "error": error,
            "error_rate": error_rate,
            "surface": surface,
            "boundary": boundary,
            "virtual_acceleration": virtual_acceleration,
            "tau_raw": tau_raw,
            "tau": tau,
            "integral": np.zeros(self.dyn.nq),
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
            f"  alpha_boundary=ks+kr/phi={alpha}, "
            f"rho_ZOH_ideal={np.round(self.sampled_spectral_radius(), 4)}\n"
            f"  tau=inverse_dynamics(q_do, qdot_do, virtual_acceleration), "
            f"tau_limit={self.tau_limit} Nm")
