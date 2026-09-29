"""Bản rời rạc của cascade PID đang dùng trong mô hình MATLAB."""

import numpy as np


class CascadePidController:
    """Position-P bên ngoài, velocity-PI bên trong, cộng inverse dynamics."""

    def __init__(
        self, dynamics, kpp, kvp, kvi, torque_constant=0.47,
        integral_limit=5.0, tau_limit=None,
    ):
        self.dyn = dynamics
        self.kpp = np.asarray(kpp, dtype=float)
        self.kvp = np.asarray(kvp, dtype=float)
        self.kvi = np.asarray(kvi, dtype=float)
        self.kt = float(torque_constant)
        requested_limit = float(integral_limit)
        self.integral_limit = (
            np.inf if requested_limit <= 0.0 else requested_limit)
        self.tau_limit = np.asarray(
            dynamics.tau_max if tau_limit is None else tau_limit, dtype=float)
        self.reset()

    def reset(self):
        self.integral = np.zeros(self.dyn.nq)
        self.last = {}

    def compute(self, q, qd, q_ref, qd_ref, qdd_ref, dt):
        q = np.asarray(q, dtype=float)
        qd = np.asarray(qd, dtype=float)
        q_ref = np.asarray(q_ref, dtype=float)
        qd_ref = np.asarray(qd_ref, dtype=float)

        position_error = q_ref - q
        velocity_command = qd_ref + self.kpp * position_error
        velocity_error = velocity_command - qd

        tau_ff = self.dyn.inverse_dynamics(q_ref, qd_ref, qdd_ref)
        current = self.kvp * velocity_error + self.kvi * self.integral
        tau_raw = tau_ff + self.kt * current
        tau = np.clip(tau_raw, -self.tau_limit, self.tau_limit)

        saturated = np.abs(tau_raw - tau) > 1e-12
        drives_back = np.sign(self.kt * self.kvi * velocity_error) != np.sign(tau_raw)
        integrate = ~saturated | drives_back
        self.integral = np.where(
            integrate, self.integral + velocity_error * float(dt), self.integral)
        self.integral = np.clip(
            self.integral, -self.integral_limit, self.integral_limit)

        self.last = {
            "position_error": position_error,
            "velocity_error": velocity_error,
            "velocity_command": velocity_command,
            "current_command": current,
            "tau_ff": tau_ff,
            "tau_raw": tau_raw,
            "tau": tau,
            "integral": self.integral.copy(),
            "saturated": saturated,
        }
        return tau

    def describe(self, q_nominal=None):
        return (
            "Cascade PID theo Simulink: qdot_cmd=qdot_ref+Kpp*(qref-q), "
            "Icmd=Kvp*e_qdot+Kvi*integral(e_qdot), "
            "tau=inverse_dynamics(ref)+Kt*Icmd\n"
            f"  Kpp={self.kpp}, Kvp={self.kvp}, Kvi={self.kvi}\n"
            f"  Kt={self.kt:g} Nm/A, |integral velocity error| <= "
            f"{self.integral_limit:g}, tau_limit={self.tau_limit} Nm")
