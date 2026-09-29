"""SMC bám quỹ đạo, cùng luật với setup_smc.m."""

import numpy as np


class SmcController:
    def __init__(self, dynamics, lambda_gain, ks, kr, phi, tau_limit=None):
        self.dyn = dynamics
        self.lambda_gain = np.asarray(lambda_gain, dtype=float)
        self.ks = np.asarray(ks, dtype=float)
        self.kr = np.asarray(kr, dtype=float)
        self.phi = np.asarray(phi, dtype=float)
        self.tau_limit = np.asarray(
            dynamics.tau_max if tau_limit is None else tau_limit, dtype=float)
        self.reset()

    def reset(self):
        self.last = {}

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
        return (
            "SMC: s=error_rate+lambda*error; reaching law dùng saturation "
            "để giảm chattering\n"
            f"  lambda={self.lambda_gain}, ks={self.ks}, kr={self.kr}, "
            f"phi={self.phi}\n"
            f"  tau=inverse_dynamics(q_do, qdot_do, virtual_acceleration), "
            f"tau_limit={self.tau_limit} Nm")
