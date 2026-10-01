from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm, solve_continuous_are, solve_discrete_are
from scipy.spatial import cKDTree

_Z3 = np.zeros((3, 3))
_I3 = np.eye(3)

# Bước sai phân trung tâm để lấy Astiff/Adamp. Bằng `delta` của setup_lqr.m.
FD_STEP = 1e-5

# Bộ trọng số mặc định -- KHÔNG phải số của matlab, xem mục 2 của docstring.
DEFAULT_MAX_INT_E = (0.002, 0.002, 0.002)   # rad·s   -- sai số tích luỹ chịu được
DEFAULT_MAX_E = (0.1, 0.1, 0.1)             # rad     -- như matlab
DEFAULT_MAX_DE = (2.0, 2.0, 2.0)            # rad/s   -- như matlab
DEFAULT_TAU_PENALTY_SCALE = 512.0           # nhân vào R; matlab tương đương 1.0

# Bộ số ĐÚNG NHƯ matlab, để đối chiếu trong mô phỏng.
MATLAB_MAX_INT_E = (10.0, 20.0, 10.0)
MATLAB_MAX_E = (0.05, 0.02, 0.01)
MATLAB_MAX_DE = (10.0, 10.0, 10.0)
MATLAB_TAU_PENALTY_SCALE = 1.0


@dataclass
class LqrWeights:
    """Trọng số Bryson. Mỗi trường là mảng 3 phần tử (riêng từng khớp) hoặc số
    vô hướng (dùng chung). max_tau=None -> lấy <limit effort> của URDF, đúng như
    setup_lqr.m dùng [5, 40, 5]."""

    max_int_e: tuple = DEFAULT_MAX_INT_E
    max_e: tuple = DEFAULT_MAX_E
    max_de: tuple = DEFAULT_MAX_DE
    max_tau: tuple = None
    tau_penalty_scale: float = DEFAULT_TAU_PENALTY_SCALE

    def _v(self, x, n):
        a = np.asarray(x, dtype=float)
        return np.full(n, float(a)) if a.ndim == 0 else a.astype(float)

    def QR(self, tau_max, n: int = 3):
        """Q (3n x 3n) và R (n x n). Thứ tự trạng thái [∫e, e, ė] -- như matlab."""
        mie = self._v(self.max_int_e, n)
        me = self._v(self.max_e, n)
        mde = self._v(self.max_de, n)
        mt = self._v(tau_max if self.max_tau is None else self.max_tau, n)
        if np.any(mie <= 0) or np.any(me <= 0) or np.any(mde <= 0) or np.any(mt <= 0):
            raise ValueError("mọi max_* phải > 0 (luật Bryson lấy 1/max²)")
        Q = np.diag(np.concatenate([1.0 / mie**2, 1.0 / me**2, 1.0 / mde**2]))
        R = np.diag(float(self.tau_penalty_scale) / mt**2)
        return Q, R


class LqrController:
    """LQR biến thiên + tích phân. Mọi đại lượng ở PHÍA KHỚP (rad, Nm).

    Cùng giao diện ``compute(...) -> tau`` với PID, MPC và SMC."""

    def __init__(
        self,
        dynamics,
        weights: LqrWeights = None,
        i_limit: float = 0.004,
        tau_limit=None,
        control_hz: float = 100.0,
        gravity_at_measured: bool = True,
        recompute_every: int = 1,
        fd_step: float = FD_STEP,
        require_discrete_stable: bool = True,
        tau_rate_limit=None,
        use_discrete_lqr: bool = False,
        gain_schedule_hz: float = 0.0,
    ):
        self.dyn = dynamics
        self.n = int(dynamics.nq)
        self.weights = weights or LqrWeights()
        self.Q, self.R = self.weights.QR(dynamics.tau_max, self.n)
        self.i_limit = float(i_limit)
        self.tau_limit = (np.asarray(dynamics.tau_max, dtype=float)
                          if tau_limit is None else np.asarray(tau_limit, dtype=float))
        self.control_hz = float(control_hz)
        self.gravity_at_measured = bool(gravity_at_measured)
        self.recompute_every = max(1, int(recompute_every))
        self.fd_step = float(fd_step)
        self.use_discrete_lqr = bool(use_discrete_lqr)
        self.gain_schedule_hz = float(gain_schedule_hz)
        if not np.isfinite(self.gain_schedule_hz) or self.gain_schedule_hz < 0.0:
            raise ValueError("gain_schedule_hz phải là số hữu hạn không âm")
        self._gain_schedule_tree = None
        self._gain_schedule_scale = None
        self._gain_schedule_gains = None
        self._gain_schedule_index = None
        self._fixed_gain_index = None
        self.tau_rate_limit = np.zeros(self.n) if tau_rate_limit is None else (
            np.asarray(tau_rate_limit, dtype=float))
        if (self.tau_rate_limit.shape != (self.n,)
                or np.any(~np.isfinite(self.tau_rate_limit))
                or np.any(self.tau_rate_limit < 0.0)):
            raise ValueError(
                "tau_rate_limit phải là vector không âm, đơn vị Nm/s")
        self.K = self.gain(*self._nominal())
        self.reset()
        if require_discrete_stable:
            rep = self.stability_report()
            if not rep["stable_discrete"]:
                raise ValueError(
                    "Bộ trọng số này PHÂN KỲ khi rời rạc hoá ở "
                    f"{self.control_hz:g} Hz: bán kính phổ |z|max = "
                    f"{rep['spectral_radius']:.3f} (cần < 1), cực liên tục nhanh "
                    f"nhất {rep['fastest_pole']:.0f} rad/s so với Nyquist "
                    f"{np.pi*self.control_hz:.0f} rad/s.\n"
                    "Cách sửa: tăng tau_penalty_scale (mô-men đắt hơn -> hệ số "
                    "nhỏ hơn) hoặc tăng control_hz. Đặt "
                    "require_discrete_stable=False CHỈ khi chạy mô phỏng."
                )

    # ------------------------------------------------------------------
    def _nominal(self):
        return (self.dyn.q_min + self.dyn.q_max) / 2.0, np.zeros(self.n)

    def reset(self):
        self.integral = np.zeros(self.n)
        self.last = {}
        self._cycle = 0
        self.last_tau = None

    def precompute_gain_schedule(self, q_ref, qd_ref):
        """Solve Riccati before motion and build a fast reference lookup.

        The schedule is indexed by ``[q_ref, qd_ref]`` rather than wall time,
        so timer jitter cannot select a gain from the wrong trajectory phase.
        During ``compute()`` only a KD-tree query is performed; no CARE/DARE
        solve occurs in the real-time callback.
        """
        q_ref = np.asarray(q_ref, dtype=float)
        qd_ref = np.asarray(qd_ref, dtype=float)

        expected_columns = self.n
        if (q_ref.ndim != 2 or q_ref.shape[1] != expected_columns
                or qd_ref.shape != q_ref.shape or len(q_ref) == 0
                or not np.all(np.isfinite(q_ref))
                or not np.all(np.isfinite(qd_ref))):
            raise ValueError(
                "q_ref và qd_ref của gain schedule phải là các mảng "
                f"N x {self.n} hữu hạn, N > 0")

        gains = np.asarray([
            self.gain(q, qd) for q, qd in zip(q_ref, qd_ref)],
            dtype=float)
        features = np.hstack((q_ref, qd_ref))
        # Normalize position and velocity so neither unit dominates the
        # nearest-reference lookup. Floors keep constant dimensions valid.
        floors = np.concatenate((
            np.full(self.n, 0.05),
            np.full(self.n, 0.10),
        ))
        scale = np.maximum(np.ptp(features, axis=0), floors)
        self._gain_schedule_scale = scale
        self._gain_schedule_gains = gains
        self._gain_schedule_tree = cKDTree(features / scale)
        self._gain_schedule_index = None
        self._fixed_gain_index = None
        self.K = gains[0].copy()
        return len(gains)

    def set_fixed_gain_index(self, index):
        """Use one precomputed K until cleared; used only for diagnostics."""
        if index is None:
            self._fixed_gain_index = None
            return
        if self._gain_schedule_gains is None:
            raise ValueError("Chưa precompute bảng gain LQR")
        index = int(index)
        if index < 0 or index >= len(self._gain_schedule_gains):
            raise ValueError(f"Chỉ số gain cố định {index} ngoài bảng")
        self._fixed_gain_index = index

    # ------------------------------------------------------------------
    def linearize(self, q_ref, qd_ref):
        """(A, B) của hệ sai số 9 trạng thái tại điểm làm việc. Sai phân trung
        tâm hai phía -- KHÔNG phải một phía: một phía có sai số bậc h (1e-5) còn
        hai phía bậc h² (1e-10), mà Astiff đi thẳng vào A nên sai số đó nhân
        trực tiếp vào hệ số điều khiển."""
        q = np.asarray(q_ref, dtype=float)
        qd = np.asarray(qd_ref, dtype=float)
        n, h = self.n, self.fd_step
        Minv = np.linalg.inv(self.dyn.mass_matrix(q))

        Astiff = np.empty((n, n))
        Adamp = np.empty((n, n))
        for i in range(n):
            qp, qm = q.copy(), q.copy()
            qp[i] += h
            qm[i] -= h
            Astiff[:, i] = (self.dyn.gravity(qp) - self.dyn.gravity(qm)) / (2.0 * h)

            vp, vm = qd.copy(), qd.copy()
            vp[i] += h
            vm[i] -= h
            Adamp[:, i] = (self.dyn.coriolis(q, vp) - self.dyn.coriolis(q, vm)) / (2.0 * h)

        A = np.block([[_Z3, _I3, _Z3],
                      [_Z3, _Z3, _I3],
                      [_Z3, -Minv @ Astiff, -Minv @ Adamp]])
        B = np.vstack([_Z3, _Z3, Minv])
        return A, B

    def _zoh(self, A, B):
        """Discretize one local model at the configured control period."""
        n_state, n_input = B.shape
        augmented = np.block([
            [A, B],
            [np.zeros((n_input, n_state + n_input))],
        ])
        transition = expm(augmented / self.control_hz)
        return transition[:n_state, :n_state], transition[:n_state, n_state:]

    def _gain_from_model(self, A, B):
        if not self.use_discrete_lqr:
            P = solve_continuous_are(A, B, self.Q, self.R)
            return np.linalg.solve(self.R, B.T @ P)

        Ad, Bd = self._zoh(A, B)
        # Qd≈Q*dt và Rd≈R*dt là phép rời rạc hoá tích phân chi phí bậc nhất.
        # Cùng nhân dt ở Q/R không đổi trade-off, nhưng giữ đúng đơn vị cost.
        dt = 1.0 / self.control_hz
        P = solve_discrete_are(Ad, Bd, self.Q * dt, self.R * dt)
        return np.linalg.solve(
            self.R * dt + Bd.T @ P @ Bd,
            Bd.T @ P @ Ad)

    def gain(self, q_ref, qd_ref) -> np.ndarray:
        """K (n x 3n) tại điểm làm việc. Cột 0..n-1 nhân ∫e, n..2n-1 nhân e,
        2n..3n-1 nhân ė."""
        A, B = self.linearize(q_ref, qd_ref)
        return self._gain_from_model(A, B)

    # ------------------------------------------------------------------
    def compute(self, q, qd, q_ref, qd_ref, qdd_ref, dt: float) -> np.ndarray:
        """Một chu kỳ điều khiển. Trả về mô-men khớp (Nm) đã kẹp giới hạn."""
        q = np.asarray(q, dtype=float)
        qd = np.asarray(qd, dtype=float)
        q_ref = np.asarray(q_ref, dtype=float)
        qd_ref = np.asarray(qd_ref, dtype=float)

        if self._fixed_gain_index is not None:
            self._gain_schedule_index = self._fixed_gain_index
            self.K = self._gain_schedule_gains[self._fixed_gain_index]
        elif self._gain_schedule_tree is not None:
            feature = np.concatenate((q_ref, qd_ref))
            _, schedule_index = self._gain_schedule_tree.query(
                feature / self._gain_schedule_scale, k=1)
            self._gain_schedule_index = int(schedule_index)
            self.K = self._gain_schedule_gains[self._gain_schedule_index]
        elif self._cycle % self.recompute_every == 0:
            self.K = self.gain(q_ref, qd_ref)
        self._cycle += 1

        e = q - q_ref
        ed = qd - qd_ref
        integral_used = self.integral.copy()
        x = np.concatenate([integral_used, e, ed])

        # feedforward nghịch động lực học TRÊN QUỸ ĐẠO THAM CHIẾU
        tau_ff = self.dyn.inverse_dynamics(q_ref, qd_ref, qdd_ref)
        if self.gravity_at_measured:
            # đổi G(q_ref) -> G(q); xem mục 3 docstring
            tau_ff = tau_ff - self.dyn.gravity(q_ref) + self.dyn.gravity(q)

        tau = tau_ff - self.K @ x
        # Chỉ để chẩn đoán: giữ phép tính tau ở trên không đổi so với bộ gốc.
        tau_integral = -self.K[:, :self.n] @ integral_used
        tau_position = -self.K[:, self.n:2*self.n] @ e
        tau_velocity = -self.K[:, 2*self.n:] @ ed
        tau_sat = np.clip(tau, -self.tau_limit, self.tau_limit)

        # ROS topic và forward_command_controller giữ torque theo ZOH. Nếu
        # feedback vận tốc có một mẫu nhiễu lớn, lệnh không giới hạn có thể
        # đảo trực tiếp từ +tau_max sang -tau_max trong một chu kỳ và kích
        # limit-cycle mà mô hình liên tục không thể hiện. Giới hạn slew-rate
        # mô phỏng băng thông hữu hạn của current loop. Vector 0 giữ đúng luật
        # MATLAB gốc (không giới hạn).
        if self.last_tau is not None and np.any(self.tau_rate_limit > 0.0):
            max_delta = self.tau_rate_limit * max(float(dt), 0.0)
            unlimited = self.tau_rate_limit <= 0.0
            max_delta = np.where(unlimited, np.inf, max_delta)
            tau_sat = self.last_tau + np.clip(
                tau_sat - self.last_tau, -max_delta, max_delta)
            tau_sat = np.clip(tau_sat, -self.tau_limit, self.tau_limit)
        self.last_tau = tau_sat.copy()

        # chống windup: chạm trần VÀ vẫn đang đẩy cùng chiều -> ngừng tích luỹ
        saturated = np.abs(tau - tau_sat) > 1e-12
        pushing_further = saturated & (np.sign(tau) == np.sign(-e))
        self.integral = np.where(pushing_further, self.integral,
                                 self.integral + e * dt)
        self.integral = np.clip(self.integral, -self.i_limit, self.i_limit)

        self.last = dict(e=e, ed=ed, tau_ff=tau_ff, tau_raw=tau, tau=tau_sat,
                         integral=self.integral.copy(),
                         tau_integral=tau_integral,
                         tau_position=tau_position,
                         tau_velocity=tau_velocity,
                         saturated=saturated,
                         K=self.K.copy(),
                         gain_schedule_index=self._gain_schedule_index)
        return tau_sat

    # ------------------------------------------------------------------
    def integral_kick(self, i_limit: float = None) -> np.ndarray:
        """Mô-men (Nm) mà RIÊNG khâu tích phân có thể ra lệnh khi ∫e chạm kẹp.

        Thay cho công thức K[:,0]*i_limit*M_ii của bộ cũ, vốn sai đơn vị ở đây:
        K của bộ này đã là Nm/(rad·s), không phải (rad/s²)/(rad·s). Lấy chuẩn L1
        theo 3 cột tích phân, tức trường hợp xấu nhất khi cả 3 thành phần ∫e
        chạm kẹp cùng chiều có hại."""
        il = self.i_limit if i_limit is None else float(i_limit)
        n = self.n
        worst = np.zeros(n)
        for q in (self.dyn.q_min, self.dyn.q_max, (self.dyn.q_min + self.dyn.q_max) / 2):
            K = self.gain(q, np.zeros(n))
            worst = np.maximum(worst, np.abs(K[:, :n]).sum(axis=1) * il)
        return worst

    # ------------------------------------------------------------------
    def stability_report(self, samples: int = 24, seed: int = 0) -> dict:
        """Kiểm hệ SAI SỐ vòng kín trên nhiều điểm làm việc, cả liên tục lẫn
        RỜI RẠC HOÁ ZOH ở control_hz.

        Vì sao phải kiểm bản rời rạc: LQR liên tục luôn cho hệ ổn định về mặt
        toán học, nhưng cài đặt thật giữ mô-men không đổi suốt 1 chu kỳ (ZOH).
        Cực liên tục nhanh hơn ~1/dt thì bản rời rạc PHÂN KỲ dù bản liên tục
        hoàn hảo. Đây chính là chỗ bộ trọng số của setup_lqr.m vỡ ở 100 Hz."""
        rng = np.random.default_rng(seed)
        qs = np.vstack([(self.dyn.q_min + self.dyn.q_max) / 2.0,
                        self.dyn.q_min, self.dyn.q_max,
                        rng.uniform(self.dyn.q_min, self.dyn.q_max,
                                    size=(max(0, samples - 3), self.n))])
        rho = 0.0
        fast = 0.0
        slow = np.inf
        for q in qs:
            A, B = self.linearize(q, np.zeros(self.n))
            K = self._gain_from_model(A, B)
            ev = np.linalg.eigvals(A - B @ K)
            fast = max(fast, float(np.abs(ev).max()))
            slow = min(slow, float(np.abs(ev.real).min()))
            Ad, Bd = self._zoh(A, B)
            rho = max(rho, float(np.abs(np.linalg.eigvals(Ad - Bd @ K)).max()))
        return dict(spectral_radius=rho, stable_discrete=rho < 1.0,
                    fastest_pole=fast, slowest_pole=slow,
                    nyquist=np.pi * self.control_hz,
                    settling_time=(4.0 / slow if slow > 0 else np.inf))

    # ------------------------------------------------------------------
    def describe(self, q_nominal=None) -> str:
        w = self.weights
        rep = self.stability_report()
        max_tau = w.max_tau if w.max_tau is not None else self.dyn.tau_max
        gravity_text = (
            "+ G(q_đo) - G(q_ref)" if self.gravity_at_measured
            else "(G lấy ở q_ref, đúng kiểu matlab)")
        slew_text = "không giới hạn"
        if np.any(self.tau_rate_limit > 0.0):
            slew_text = f"{np.round(self.tau_rate_limit, 3)} Nm/s"
        if self.gain_schedule_hz > 0.0:
            gain_update_text = (
                f"K precompute dọc quỹ đạo @ {self.gain_schedule_hz:g} Hz; "
                "runtime chỉ tra bảng theo (q_ref, q̇_ref)")
        else:
            gain_update_text = (
                f"K tính online mỗi {self.recompute_every} chu kỳ, "
                "lập theo (q_ref, q̇_ref)")
        lines = [
            "Bộ điều khiển: TVLQR (LQR biến thiên 9 trạng thái + tích phân), "
            + ("DARE rời rạc theo ZOH" if self.use_discrete_lqr
               else "CARE liên tục theo setup_lqr.m"),
            f"  trọng số Bryson: max_int_e={np.round(w._v(w.max_int_e, self.n), 4)} rad·s  "
            f"max_e={np.round(w._v(w.max_e, self.n), 4)} rad  "
            f"max_de={np.round(w._v(w.max_de, self.n), 3)} rad/s",
            f"  max_tau={np.round(w._v(max_tau, self.n), 3)} Nm"
            f"  x tau_penalty_scale={w.tau_penalty_scale:g}",
            f"  feedforward: inverse_dynamics(q_ref) "
            f"{gravity_text}",
            f"  {gain_update_text}",
            f"  trần mô-men: {np.round(self.tau_limit, 3)} Nm   "
            f"|∫e| <= {self.i_limit:g} rad·s",
            f"  slew-rate mô-men: {slew_text}",
            "",
            f"  KIỂM TRA ZOH ({self.control_hz:g} Hz): |z|max = {rep['spectral_radius']:.4f}"
            f"  -> {'ỔN ĐỊNH' if rep['stable_discrete'] else 'PHÂN KỲ'}",
            f"  cực liên tục: nhanh nhất {rep['fastest_pole']:.1f} rad/s "
            f"(Nyquist vòng {rep['nyquist']:.0f} rad/s), "
            f"chậm nhất {rep['slowest_pole']:.3f} rad/s "
            f"-> t_xác_lập ≈ {rep['settling_time']:.2f} s",
            f"  mô-men riêng khâu tích phân khi ∫e chạm kẹp: "
            f"{np.round(self.integral_kick(), 3)} Nm",
        ]
        q0 = self._nominal()[0] if q_nominal is None else np.asarray(q_nominal, float)
        K = self.gain(q0, np.zeros(self.n))
        lines.append(f"  K tại q = {np.round(q0, 3)} (Nm trên mỗi đơn vị trạng thái):")
        for i, name in enumerate(self.dyn.joint_names):
            lines.append(
                f"    {name:<15} ∫e:{np.array2string(K[i, :self.n], precision=3)}"
                f"  e:{np.array2string(K[i, self.n:2*self.n], precision=2)}"
                f"  ė:{np.array2string(K[i, 2*self.n:], precision=2)}")
        lines.append("    (K đổi theo tư thế -- đây chỉ là một điểm làm việc)")
        return "\n".join(lines)


# ======================================================================
# Tự kiểm tra: mô phỏng vòng kín trên CHÍNH mô hình, so mấy bộ trọng số
# ======================================================================
def _simulate(dyn, ctrl, ref, duration, ctrl_hz=100.0, sim_sub=10):
    """Vòng kín: controller chạy ở ctrl_hz (ZOH), plant tích phân ở
    ctrl_hz*sim_sub bằng Runge-Kutta 4, có cả ma sát của mô hình.
    Trả về (t, q, q_ref, tau)."""
    dt_c = 1.0 / ctrl_hz
    dt_s = dt_c / sim_sub
    q, qd = ref(0.0)[0].copy(), ref(0.0)[1].copy()
    ts, qs, qrs, taus = [], [], [], []

    def acc(q_, qd_, tau_):
        rhs = tau_ - dyn.nonlinear(q_, qd_) - dyn.friction(qd_)
        return np.linalg.solve(dyn.mass_matrix(q_), rhs)

    n_steps = int(round(duration / dt_c))
    for k in range(n_steps):
        t = k * dt_c
        qr, qdr, qddr = ref(t)
        tau = ctrl.compute(q, qd, qr, qdr, qddr, dt_c)
        ts.append(t)
        qs.append(q.copy())
        qrs.append(qr.copy())
        taus.append(tau.copy())
        for _ in range(sim_sub):                       # ZOH: tau giữ nguyên
            k1v = acc(q, qd, tau)
            k2v = acc(q + 0.5*dt_s*qd, qd + 0.5*dt_s*k1v, tau)
            k3v = acc(q + 0.5*dt_s*(qd + 0.5*dt_s*k1v), qd + 0.5*dt_s*k2v, tau)
            k4v = acc(q + dt_s*(qd + 0.5*dt_s*k2v), qd + dt_s*k3v, tau)
            q = q + dt_s*(qd + dt_s*(k1v + k2v + k3v)/6.0)
            qd = qd + dt_s*(k1v + 2*k2v + 2*k3v + k4v)/6.0
        if not np.all(np.isfinite(q)) or np.abs(q).max() > 1e3:
            break
    return (np.array(ts), np.array(qs), np.array(qrs), np.array(taus))


if __name__ == "__main__":
    import sys

    from arm_dynamics import ArmDynamics

    urdf = sys.argv[1] if len(sys.argv) > 1 else "gim_arm.urdf"
    dyn = ArmDynamics(urdf)
    n = dyn.nq
    q0 = (dyn.q_min + dyn.q_max) / 2.0

    print("=" * 78)
    print("1) Astiff/Adamp có đúng là đạo hàm của G và C q̇ không")
    print("=" * 78)
    ctrl = LqrController(dyn)
    A, B = ctrl.linearize(q0, np.zeros(n))
    Minv = np.linalg.inv(dyn.mass_matrix(q0))
    # kiểm gián tiếp: -M⁻¹Astiff phải bằng d(q̈)/dq quanh điểm cân bằng tĩnh
    h = 1e-6
    num = np.empty((n, n))
    for i in range(n):
        qp, qm = q0.copy(), q0.copy()
        qp[i] += h
        qm[i] -= h
        num[:, i] = -(Minv @ (dyn.gravity(qp) - dyn.gravity(qm)) / (2 * h))
    print(f"  |(-M⁻¹∂G/∂q) khối A[6:9,3:6] - sai phân độc lập| = "
          f"{np.abs(A[2*n:, n:2*n] - num).max():.3e}")
    print(f"  B[6:9] - M⁻¹ = {np.abs(B[2*n:] - Minv).max():.3e}  (phải là 0)")
    print(f"  A[0:3,3:6] = I? {np.allclose(A[:n, n:2*n], np.eye(n))}   "
          f"A[3:6,6:9] = I? {np.allclose(A[n:2*n, 2*n:], np.eye(n))}")
    print()

    print("=" * 78)
    print("2) Bộ trọng số mặc định")
    print("=" * 78)
    print(ctrl.describe(q_nominal=q0))
    print()

    print("=" * 78)
    print("3) Bộ trọng số ĐÚNG NHƯ matlab -- ở 100 Hz thì sao")
    print("=" * 78)
    wm = LqrWeights(max_int_e=MATLAB_MAX_INT_E,
                    max_e=MATLAB_MAX_E,
                    max_de=MATLAB_MAX_DE,
                    tau_penalty_scale=MATLAB_TAU_PENALTY_SCALE)
    try:
        LqrController(dyn, weights=wm)
        print("  (khởi tạo được -- không chặn)")
    except ValueError as ex:
        print("  BỊ CHẶN, đúng như mong đợi:")
        for ln in str(ex).splitlines():
            print("   ", ln)
    print()

    print("=" * 78)
    print("4) Mô phỏng vòng kín: bám bước 0.10 rad ở cả 3 khớp, 100 Hz")
    print("=" * 78)

    class _Step:
        def __init__(self, q_start, amp, t_on=0.5):
            self.q0 = np.asarray(q_start, float)
            self.amp = np.asarray(amp, float)
            self.t_on = float(t_on)

        def at(self, t):
            z = np.zeros_like(self.q0)
            qr = self.q0 + (self.amp if t >= self.t_on else 0.0)
            return qr, z, z.copy()

    variants = [
        ("mặc định (int 0.002, R x512)", LqrWeights()),
        ("int 0.001 (bám hơn, quá kick)", LqrWeights(max_int_e=(0.001,)*3)),
        ("R x2048 (mềm hơn, ~băng thông bộ cũ)", LqrWeights(tau_penalty_scale=2048.0)),
        ("bộ trọng số matlab",
         LqrWeights(max_int_e=MATLAB_MAX_INT_E,
                    max_e=MATLAB_MAX_E, max_de=MATLAB_MAX_DE,
                    tau_penalty_scale=MATLAB_TAU_PENALTY_SCALE)),
    ]
    print(f"  {'bộ trọng số':<38} {'|z|max':>7} {'e_RMS(mrad)':>12} "
          f"{'e_cuối(mrad)':>13} {'|τ|đỉnh(Nm)':>26}")
    step = _Step(q0, np.full(n, 0.10))
    for label, w in variants:
        try:
            c = LqrController(dyn, weights=w, i_limit=0.05,
                              require_discrete_stable=False)
        except ValueError as ex:
            print(f"  {label:<38} bị chặn: {ex}")
            continue
        rep = c.stability_report(samples=6)
        t, q, qr, tau = _simulate(dyn, c, step.at, duration=6.0)
        if len(t) < 500:
            print(f"  {label:<38} {rep['spectral_radius']:7.3f}  PHÂN KỲ trong mô phỏng")
            continue
        err = q - qr
        m = t >= 1.0                       # bỏ 0.5 s trước bước + 0.5 s đầu
        print(f"  {label:<38} {rep['spectral_radius']:7.3f} "
              f"{1000*np.sqrt((err[m]**2).mean()):12.3f} "
              f"{1000*np.abs(err[-1]).max():13.3f} "
              f"{str(np.round(np.abs(tau).max(axis=0), 2)):>26}")
    print()
    print("  e_cuối = sai số xác lập lớn nhất trên 3 khớp ở cuối 6 s -- đây là chỗ")
    print("  khâu tích phân có tác dụng hay không hiện ra rõ nhất.")
