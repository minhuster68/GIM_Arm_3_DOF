"""MPC rời rạc có giới hạn torque và slew-rate cho runner dùng chung."""

import time

import numpy as np
from scipy.linalg import cho_factor, cho_solve, expm
from scipy.optimize import Bounds, LinearConstraint, minimize


class MpcController:
    """
    Linearize tại reference hiện tại rồi giải QP hữu hạn bằng SLSQP hoặc ADMM.

    Đây là implementation độc lập để nối và benchmark pipeline ROS/MuJoCo.
    Nó chưa được coi là real-time cho đến khi log chứng minh thời gian giải
    luôn nằm trong chu kỳ 10 ms trên máy đích.
    """

    def __init__(
        self, dynamics, control_hz, prediction_horizon, control_horizon,
        q_integral, q_position, q_velocity, r_input, r_rate,
        torque_slew_rate, integral_limit=0.05, tau_limit=None,
        fd_step=1e-5, max_iterations=30, qp_solver="slsqp",
        admm_rho=10.0, admm_iterations=100, admm_tolerance=1.0e-4,
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
        self.qp_solver = str(qp_solver).lower()
        if self.qp_solver not in ("slsqp", "admm"):
            raise ValueError("qp_solver phải là 'slsqp' hoặc 'admm'")
        self.admm_rho = float(admm_rho)
        self.admm_iterations = int(admm_iterations)
        self.admm_tolerance = float(admm_tolerance)
        if (self.admm_rho <= 0.0 or self.admm_iterations < 1
                or self.admm_tolerance <= 0.0):
            raise ValueError("tham số ADMM phải dương")
        self._difference = self._difference_matrix()
        self._rate_weight = np.kron(np.eye(self.nc), self.Rd)
        size = self.nc * self.n
        self._constraint_matrix = np.vstack([
            np.eye(size), self._difference])
        self._constraint_gram = (
            self._constraint_matrix.T @ self._constraint_matrix)
        self.reset()

    def reset(self):
        self.integral = np.zeros(self.n)
        self.previous_feedback = np.zeros(self.n)
        self.warm = np.zeros((self.nc, self.n))
        self.last = {}

    def _linearize(self, q_ref, qd_ref, qdd_ref):
        """Match ``urdf_MPC/+mpctune/error_model.m``.

        The mechanical six-state model is discretized with a ZOH, while the
        integral state uses the same forward-Euler update as the Simulink
        Discrete-Time Integrator.  Differentiating all of inverse dynamics is
        important on a moving reference: d(M(q) qdd_ref)/dq is not generally
        zero.
        """
        n, step = self.n, self.fd_step
        inverse_mass = np.linalg.inv(self.dyn.mass_matrix(q_ref))
        stiffness = np.empty((n, n))
        damping = np.empty((n, n))
        for index in range(n):
            q_plus, q_minus = q_ref.copy(), q_ref.copy()
            q_plus[index] += step
            q_minus[index] -= step
            stiffness[:, index] = (
                self.dyn.inverse_dynamics(q_plus, qd_ref, qdd_ref)
                - self.dyn.inverse_dynamics(q_minus, qd_ref, qdd_ref)
            ) / (2.0 * step)

            v_plus, v_minus = qd_ref.copy(), qd_ref.copy()
            v_plus[index] += step
            v_minus[index] -= step
            damping[:, index] = (
                self.dyn.inverse_dynamics(q_ref, v_plus, qdd_ref)
                - self.dyn.inverse_dynamics(q_ref, v_minus, qdd_ref)
            ) / (2.0 * step)

        zero = np.zeros((n, n))
        eye = np.eye(n)
        mechanical_a = np.block([
            [zero, eye],
            [-inverse_mass @ stiffness, -inverse_mass @ damping],
        ])
        mechanical_b = np.vstack([zero, inverse_mass])
        augmented = np.block([
            [mechanical_a, mechanical_b],
            [np.zeros((n, 3*n))],
        ])
        discrete = expm(augmented * self.dt)
        mechanical_ad = discrete[:2*n, :2*n]
        mechanical_bd = discrete[:2*n, 2*n:]
        ad = np.block([
            [eye, self.dt * eye, zero],
            [np.zeros((2*n, n)), mechanical_ad],
        ])
        bd = np.vstack([zero, mechanical_bd])
        return ad, bd

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

    def _project_feasible(self, proposed, lower, upper, delta):
        """Project a sequence onto box and slew constraints, in time order."""
        proposed = np.asarray(proposed, dtype=float).reshape(self.nc, self.n)
        sequence = np.empty_like(proposed)
        previous = self.previous_feedback
        for step in range(self.nc):
            lo = np.maximum(lower[step*self.n:(step+1)*self.n],
                            previous - delta[step*self.n:(step+1)*self.n])
            hi = np.minimum(upper[step*self.n:(step+1)*self.n],
                            previous + delta[step*self.n:(step+1)*self.n])
            # A very fast feedforward jump can make total-torque and slew
            # constraints incompatible.  Bounds win because they protect the
            # actuator; the feasibility test below will reject this seed.
            sequence[step] = np.minimum(
                np.maximum(proposed[step], lo), hi)
            sequence[step] = np.clip(
                sequence[step],
                lower[step*self.n:(step+1)*self.n],
                upper[step*self.n:(step+1)*self.n])
            previous = sequence[step]
        return sequence.ravel()

    def _solve_admm(self, hessian, gradient, warm_start,
                    constraint_lower, constraint_upper):
        """Solve the small convex QP with an OSQP-style splitting method.

        The constraint matrix is constant, so each tick only factors one
        30-by-30 positive-definite matrix.  A final causal projection makes
        the returned torque sequence strictly feasible even when ADMM stops
        at its iteration limit.
        """
        matrix = self._constraint_matrix
        rho = self.admm_rho
        factor = cho_factor(
            hessian + rho * self._constraint_gram,
            lower=True, check_finite=False)
        x = warm_start.copy()
        z = np.clip(matrix @ x, constraint_lower, constraint_upper)
        dual = np.zeros_like(z)
        primal_residual = dual_residual = np.inf
        iterations = self.admm_iterations
        for iteration in range(1, self.admm_iterations + 1):
            x = cho_solve(
                factor,
                -gradient + rho * matrix.T @ (z - dual),
                check_finite=False)
            product = matrix @ x
            previous_z = z
            z = np.clip(product + dual,
                        constraint_lower, constraint_upper)
            dual += product - z
            primal_residual = float(np.max(np.abs(product - z)))
            dual_residual = float(np.max(np.abs(
                rho * matrix.T @ (z - previous_z))))
            if max(primal_residual, dual_residual) <= self.admm_tolerance:
                iterations = iteration
                break
            # Balance primal and dual convergence as OSQP does.  The QP is
            # badly conditioned (joint-2 torque scale is 8x joint 1/3), so a
            # fixed rho converges unnecessarily slowly on active constraints.
            if iteration % 20 == 0:
                old_rho = rho
                if primal_residual > 10.0 * dual_residual:
                    rho = min(10.0 * rho, 1.0e4)
                elif dual_residual > 10.0 * primal_residual:
                    rho = max(0.1 * rho, 1.0e-6)
                if rho != old_rho:
                    dual *= old_rho / rho
                    factor = cho_factor(
                        hessian + rho * self._constraint_gram,
                        lower=True, check_finite=False)
        return x, iterations, primal_residual, dual_residual, rho

    def _constraint_violation(self, flat_u, lower, upper,
                              rate_lower, rate_upper):
        if flat_u is None or not np.all(np.isfinite(flat_u)):
            return np.inf
        rate = self._difference @ flat_u
        return float(max(
            np.max(np.maximum(lower - flat_u, 0.0)),
            np.max(np.maximum(flat_u - upper, 0.0)),
            np.max(np.maximum(rate_lower - rate, 0.0)),
            np.max(np.maximum(rate - rate_upper, 0.0)),
        ))

    def compute(self, q, qd, q_ref, qd_ref, qdd_ref, dt):
        q = np.asarray(q, dtype=float)
        qd = np.asarray(qd, dtype=float)
        q_ref = np.asarray(q_ref, dtype=float)
        qd_ref = np.asarray(qd_ref, dtype=float)
        error = q - q_ref
        error_rate = qd - qd_ref
        state = np.concatenate([self.integral, error, error_rate])
        feedforward = self.dyn.inverse_dynamics(q_ref, qd_ref, qdd_ref)
        ad, bd = self._linearize(q_ref, qd_ref, qdd_ref)

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
        warm_start = self._project_feasible(
            self.warm, lower, upper, delta)

        started = time.perf_counter()
        if self.qp_solver == "admm":
            constraint_lower = np.concatenate([lower, rate_lower])
            constraint_upper = np.concatenate([upper, rate_upper])
            (raw_candidate, iterations, primal_residual, dual_residual,
             final_rho) = (
                self._solve_admm(
                    hessian, gradient, warm_start,
                    constraint_lower, constraint_upper))
            candidate = self._project_feasible(
                raw_candidate, lower, upper, delta)
            solver_success = (
                max(primal_residual, dual_residual)
                <= self.admm_tolerance)
            solver_status = 0 if solver_success else 1
            solver_message = (
                "ADMM converged" if solver_success
                else "ADMM iteration limit reached; projected feasible")
        else:
            result = minimize(
                self._quadratic_cost,
                warm_start,
                args=(hessian, gradient),
                jac=self._quadratic_gradient,
                method="SLSQP",
                bounds=Bounds(lower, upper),
                constraints=[rate_constraint],
                options={
                    "maxiter": self.max_iterations,
                    "ftol": 1e-6,
                    "disp": False,
                },
            )
            candidate = getattr(result, "x", None)
            iterations = int(getattr(result, "nit", 0))
            solver_success = bool(result.success)
            solver_status = int(result.status)
            solver_message = str(result.message)
            primal_residual = dual_residual = float("nan")
            final_rho = float("nan")
        solve_time = time.perf_counter() - started
        violation = self._constraint_violation(
            candidate, lower, upper, rate_lower, rate_upper)
        candidate_cost = (
            self._quadratic_cost(candidate, hessian, gradient)
            if candidate is not None and np.all(np.isfinite(candidate))
            else np.inf)
        warm_cost = self._quadratic_cost(warm_start, hessian, gradient)
        cost_tolerance = 1.0e-10 * max(1.0, abs(warm_cost))
        accepted = (
            violation <= 1.0e-7
            and candidate_cost <= warm_cost + cost_tolerance)
        if accepted:
            sequence = candidate.reshape(self.nc, self.n)
            feedback = sequence[0]
            self.warm[:-1] = sequence[1:]
            self.warm[-1] = sequence[-1]
        else:
            sequence = warm_start.reshape(self.nc, self.n)
            feedback = sequence[0]
            self.warm[:-1] = sequence[1:]
            self.warm[-1] = sequence[-1]

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
            "solver_success": solver_success,
            "solver_accepted": bool(accepted),
            "solver_status": solver_status,
            "solver_message": solver_message,
            "constraint_violation": violation,
            "primal_residual": primal_residual,
            "dual_residual": dual_residual,
            "solver_rho": final_rho,
            "solve_time": solve_time,
            "iterations": iterations,
        }
        return tau

    def describe(self, q_nominal=None):
        return (
            "MPC tuyến tính hoá cục bộ, có giới hạn total torque và "
            "slew-rate; "
            f"QP giải bằng {self.qp_solver.upper()}\n"
            f"  solver={self.qp_solver}, Np={self.np}, Nc={self.nc}, "
            f"dt={self.dt:g}s, "
            f"slew={self.slew} Nm/s\n"
            f"  tau_limit={self.tau_limit} Nm. BẮT BUỘC benchmark deadline "
            "trên "
            "MuJoCo trước khi cân nhắc chạy phần cứng.")
