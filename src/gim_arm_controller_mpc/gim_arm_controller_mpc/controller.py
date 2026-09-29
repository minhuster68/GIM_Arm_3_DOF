"""MPC rời rạc có giới hạn torque và slew-rate, ưu tiên kiểm trên MuJoCo."""

import time

import numpy as np
from scipy.linalg import expm
from scipy.optimize import Bounds, LinearConstraint, minimize


class MpcController:
    """
    Linearize tại reference hiện tại rồi giải QP hữu hạn bằng SLSQP.

    Đây là implementation độc lập để nối và benchmark pipeline ROS/MuJoCo.
    Nó chưa được coi là real-time cho đến khi log chứng minh thời gian giải
    luôn nằm trong chu kỳ 10 ms trên máy đích.
    """

    def __init__(
        self, dynamics, control_hz, prediction_horizon, control_horizon,
        q_integral, q_position, q_velocity, r_input, r_rate,
        torque_slew_rate, integral_limit=0.05, tau_limit=None,
        fd_step=1e-5, max_iterations=30,
    ):
        self.dyn = dynamics
        self.n = dynamics.nq
        self.dt = 1.0 / float(control_hz)
        self.np = int(prediction_horizon)
        self.nc = int(control_horizon)
        if self.np < 1 or self.nc < 1 or self.nc > self.np:
            raise ValueError("cần prediction_horizon >= control_horizon >= 1")
        self.Q = np.diag(np.concatenate([
            np.asarray(q_integral, dtype=float),
            np.asarray(q_position, dtype=float),
            np.asarray(q_velocity, dtype=float),
        ]))
        self.R = np.diag(np.asarray(r_input, dtype=float))
        self.Rd = np.diag(np.asarray(r_rate, dtype=float))
        self.slew = np.asarray(torque_slew_rate, dtype=float)
        requested_limit = float(integral_limit)
        self.integral_limit = (
            np.inf if requested_limit <= 0.0 else requested_limit)
        self.tau_limit = np.asarray(
            dynamics.tau_max if tau_limit is None else tau_limit, dtype=float)
        self.fd_step = float(fd_step)
        self.max_iterations = int(max_iterations)
        self._difference = self._difference_matrix()
        self._rate_weight = np.kron(np.eye(self.nc), self.Rd)
        self.reset()

    def reset(self):
        self.integral = np.zeros(self.n)
        self.previous_feedback = np.zeros(self.n)
        self.warm = np.zeros((self.nc, self.n))
        self.last = {}

    def _linearize(self, q_ref, qd_ref):
        n, step = self.n, self.fd_step
        inverse_mass = np.linalg.inv(self.dyn.mass_matrix(q_ref))
        stiffness = np.empty((n, n))
        damping = np.empty((n, n))
        for index in range(n):
            q_plus, q_minus = q_ref.copy(), q_ref.copy()
            q_plus[index] += step
            q_minus[index] -= step
            stiffness[:, index] = (
                self.dyn.gravity(q_plus) - self.dyn.gravity(q_minus)) / (2.0*step)

            v_plus, v_minus = qd_ref.copy(), qd_ref.copy()
            v_plus[index] += step
            v_minus[index] -= step
            damping[:, index] = (
                self.dyn.coriolis(q_ref, v_plus)
                - self.dyn.coriolis(q_ref, v_minus)) / (2.0*step)

        zero = np.zeros((n, n))
        eye = np.eye(n)
        continuous_a = np.block([
            [zero, eye, zero],
            [zero, zero, eye],
            [zero, -inverse_mass @ stiffness, -inverse_mass @ damping],
        ])
        continuous_b = np.vstack([zero, zero, inverse_mass])
        augmented = np.block([
            [continuous_a, continuous_b],
            [np.zeros((n, 4*n))],
        ])
        discrete = expm(augmented * self.dt)
        return discrete[:3*n, :3*n], discrete[:3*n, 3*n:]

    def _difference_matrix(self):
        size = self.nc * self.n
        matrix = np.eye(size)
        for step in range(1, self.nc):
            row = slice(step*self.n, (step+1)*self.n)
            previous = slice((step-1)*self.n, step*self.n)
            matrix[row, previous] = -np.eye(self.n)
        return matrix

    def _qp_terms(self, x0, ad, bd):
        """Condense prediction into 0.5*u.T*H*u + f.T*u."""
        decision_size = self.nc * self.n
        state_offset = x0.copy()
        state_map = np.zeros((x0.size, decision_size))
        hessian = np.zeros((decision_size, decision_size))
        gradient = np.zeros(decision_size)

        for step in range(self.np):
            move = min(step, self.nc - 1)
            columns = slice(move*self.n, (move+1)*self.n)
            state_offset = ad @ state_offset
            state_map = ad @ state_map
            state_map[:, columns] += bd
            hessian += 2.0 * state_map.T @ self.Q @ state_map
            gradient += 2.0 * state_map.T @ self.Q @ state_offset
            hessian[columns, columns] += 2.0 * self.R

        # du = D*u - [u_previous, 0, ..., 0].
        previous = np.zeros(decision_size)
        previous[:self.n] = self.previous_feedback
        weighted_difference = self._rate_weight @ self._difference
        hessian += 2.0 * self._difference.T @ weighted_difference
        gradient -= 2.0 * weighted_difference.T @ previous
        return 0.5 * (hessian + hessian.T), gradient

    @staticmethod
    def _quadratic_cost(flat_u, hessian, gradient):
        return float(0.5 * flat_u @ hessian @ flat_u + gradient @ flat_u)

    @staticmethod
    def _quadratic_gradient(flat_u, hessian, gradient):
        return hessian @ flat_u + gradient

    def compute(self, q, qd, q_ref, qd_ref, qdd_ref, dt):
        q = np.asarray(q, dtype=float)
        qd = np.asarray(qd, dtype=float)
        q_ref = np.asarray(q_ref, dtype=float)
        qd_ref = np.asarray(qd_ref, dtype=float)
        error = q - q_ref
        error_rate = qd - qd_ref
        state = np.concatenate([self.integral, error, error_rate])
        feedforward = self.dyn.inverse_dynamics(q_ref, qd_ref, qdd_ref)
        ad, bd = self._linearize(q_ref, qd_ref)

        lower = np.tile(-self.tau_limit - feedforward, self.nc)
        upper = np.tile(self.tau_limit - feedforward, self.nc)
        delta = np.tile(self.slew * self.dt, self.nc)
        rate_lower = -delta
        rate_upper = delta
        rate_lower[:self.n] += self.previous_feedback
        rate_upper[:self.n] += self.previous_feedback
        rate_constraint = LinearConstraint(
            self._difference, rate_lower, rate_upper)
        hessian, gradient = self._qp_terms(state, ad, bd)

        started = time.perf_counter()
        result = minimize(
            self._quadratic_cost,
            np.clip(self.warm.ravel(), lower, upper),
            args=(hessian, gradient),
            jac=self._quadratic_gradient,
            method="SLSQP",
            bounds=Bounds(lower, upper),
            constraints=[rate_constraint],
            options={"maxiter": self.max_iterations, "ftol": 1e-6, "disp": False},
        )
        solve_time = time.perf_counter() - started
        if result.success and np.all(np.isfinite(result.x)):
            sequence = result.x.reshape(self.nc, self.n)
            feedback = sequence[0]
            self.warm[:-1] = sequence[1:]
            self.warm[-1] = sequence[-1]
        else:
            feedback = np.clip(
                self.previous_feedback,
                -self.tau_limit - feedforward,
                self.tau_limit - feedforward)

        tau_raw = feedforward + feedback
        tau = np.clip(tau_raw, -self.tau_limit, self.tau_limit)
        saturated = np.abs(tau_raw - tau) > 1e-12
        pushing_further = saturated & (np.sign(tau_raw) == np.sign(-error))
        self.integral = np.where(
            pushing_further, self.integral, self.integral + error * float(dt))
        self.integral = np.clip(
            self.integral, -self.integral_limit, self.integral_limit)
        self.previous_feedback = feedback.copy()

        self.last = {
            "error": error,
            "error_rate": error_rate,
            "integral": self.integral.copy(),
            "tau_ff": feedforward,
            "tau_feedback": feedback,
            "tau_raw": tau_raw,
            "tau": tau,
            "saturated": saturated,
            "solver_success": bool(result.success),
            "solver_status": int(result.status),
            "solver_message": str(result.message),
            "solve_time": solve_time,
            "iterations": int(getattr(result, "nit", 0)),
        }
        return tau

    def describe(self, q_nominal=None):
        return (
            "MPC tuyến tính hoá cục bộ, có giới hạn total torque và slew-rate; "
            "QP hiện giải bằng scipy/SLSQP\n"
            f"  Np={self.np}, Nc={self.nc}, dt={self.dt:g}s, "
            f"slew={self.slew} Nm/s\n"
            f"  tau_limit={self.tau_limit} Nm. BẮT BUỘC benchmark deadline trên "
            "MuJoCo trước khi cân nhắc chạy phần cứng.")
