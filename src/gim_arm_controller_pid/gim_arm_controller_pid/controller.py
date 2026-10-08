"""Single-loop position PID with direct joint-torque output."""

import numpy as np


class PositionPidController:
    """One position-error PID loop with inverse-dynamics feedforward.

    Velocity is used only to obtain de/dt; there is no inner velocity loop.
    Kp, Ki and Kd output joint torque directly, without a motor Kt factor.
    """

    def __init__(self, dynamics, kp, ki, kd, integral_limit, tau_limit=None):
        self.dyn = dynamics
        self.kp = np.asarray(kp, dtype=float)
        self.ki = np.asarray(ki, dtype=float)
        self.kd = np.asarray(kd, dtype=float)
        self.integral_limit = np.asarray(integral_limit, dtype=float)
        self.tau_limit = np.asarray(
            dynamics.tau_max if tau_limit is None else tau_limit, dtype=float)
        self.reset()

    def reset(self):
        self.integral = np.zeros(self.dyn.nq)
        self.last = {}

    def compute(self, q, qd, q_ref, qd_ref, qdd_ref, dt):
        # d(q_ref-q)/dt = qd_ref-qd. Use the available velocity signals
        # instead of differencing position across irregular state samples.
        q = np.asarray(q, dtype=float)
        qd = np.asarray(qd, dtype=float)
        q_ref = np.asarray(q_ref, dtype=float)
        qd_ref = np.asarray(qd_ref, dtype=float)
        qdd_ref = np.asarray(qdd_ref, dtype=float)
        position_error = q_ref - q
        error_rate = qd_ref - qd
        tau_ff = self.dyn.inverse_dynamics(q_ref, qd_ref, qdd_ref)
        tau_position = self.kp * position_error
        tau_derivative = self.kd * error_rate
        tau_integral = self.ki * self.integral
        tau_fb = tau_position + tau_integral + tau_derivative
        tau_raw = tau_ff + tau_fb
        tau = np.clip(tau_raw, -self.tau_limit, self.tau_limit)

        saturated = np.abs(tau_raw - tau) > 1e-12
        drives_back = np.sign(self.ki * position_error) != np.sign(tau_raw)
        integrate = ~saturated | drives_back
        self.integral = np.where(
            integrate, self.integral + position_error * float(dt), self.integral)
        self.integral = np.clip(
            self.integral, -self.integral_limit, self.integral_limit)
        self.last = {
            "position_error": position_error,
            "error_rate": error_rate,
            "tau_fb": tau_fb,
            "tau_p": tau_position,
            "tau_i": tau_integral,
            "tau_d": tau_derivative,
            "tau_ff": tau_ff,
            "tau_position": tau_position,
            "tau_derivative": tau_derivative,
            "tau_integral": tau_integral,
            "tau_raw": tau_raw,
            "tau": tau,
            "integral": self.integral.copy(),
            "saturated": saturated,
        }
        return tau

    def describe(self, q_nominal=None):
        return (
            "Single-loop position PID: e=qref-q, "
            "tau=tau_ff+Kp*e+Ki*integral(e)+Kd*de/dt; "
            "de/dt=qdot_ref-qdot\n"
            "  tau_ff=M(qref)*qddot_ref+C(qref,qdot_ref)*qdot_ref+G(qref)\n"
            f"  Kp={self.kp} Nm/rad, Ki={self.ki} Nm/(rad*s), "
            f"Kd={self.kd} Nm/(rad/s)\n"
            f"  |integral position error| <= {self.integral_limit} rad*s, "
            f"tau_limit={self.tau_limit} Nm")
