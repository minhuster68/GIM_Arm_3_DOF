#!/usr/bin/env python3
"""
lqi_node.py — chạy luật LQI trên tay thật, đẩy MÔ-MEN xuống phần cứng.

Đặt tại:  src/gim_arm_control/gim_control/lqi_node.py

CẦN CÓ arm_dynamics.py và lqi_controller.py TRONG CÙNG PACKAGE:
    cd src/gim_arm_control/gim_control
    ln -s ../../../kinematics_test/arm_dynamics.py  arm_dynamics.py
    ln -s ../../../kinematics_test/lqi_controller.py lqi_controller.py
(symlink giữ MỘT file thật duy nhất -> 5 script trong kinematics_test/ vẫn chạy.
 Dùng `cp` cũng được, nhưng khi đó bạn có 2 bản sẽ trôi lệch dần.)

===========================================================================
NÓ NẰM ĐÂU TRONG HỆ THỐNG
===========================================================================
    origin_draw_trajectory --FollowJointTrajectory--> JTC --(pos+vel)--┐
                                                                      ├-> hardware -> CAN
    lqi_node --Float64MultiArray--> lqi_effort_controller --(effort)---┘

Hai đường claim hai bộ command interface KHÁC NHAU nên đổi qua lại bằng:
    ros2 control switch_controllers \
        --deactivate gim_arm_group_controller --activate lqi_effort_controller

Plugin phần cứng phải có prepare/perform_command_mode_switch() để đổi driver
sang control_mode = 1 (mô-men). KHÔNG có bước đó thì driver vẫn ở chế độ vị trí
và mô-men LQI chỉ được cộng như feedforward vào vòng P -- tức đo ra một thứ
KHÔNG PHẢI LQI.

===========================================================================
BA CÁCH CHỌN HỆ SỐ -- tham số `tune_mode`
===========================================================================
Cả ba cho ra cùng dạng K = [k_i, k_e, k_v] và cùng luật
    τ = M(q)·(q̈_ref - k_i∫e - k_e·e - k_v·ė) + C(q,q̇)q̇ + G(q)
Chỉ khác CÁCH CHỌN 3 con số đó.

  "omega_lqr"  (MẶC ĐỊNH) -- một nút vặn `omega`, nhưng đi qua Riccati thật.
      Q = diag(ω⁶, 3ω⁴, 3ω²),  R = 1   ->  K = [ω³, 3ω², 3ω]
      Đã kiểm: trùng với gán cực tới 1e-12. Nghĩa là "gán cực" chỉ là TRƯỜNG
      HỢP RIÊNG của LQR, không phải phương pháp khác.
      Dùng cái này khi báo cáo cần trình bày đúng phương pháp LQR mà bạn vẫn
      muốn vặn một số duy nhất có đơn vị vật lý (rad/s).

  "weights"  -- khai trực tiếp q_int/q_pos/q_vel/r, giải Riccati.
      Rộng hơn "omega_lqr": cho được 3 cực KHÁC nhau. Ví dụ bộ mặc định của
      repo (6e6, 1.5e5, 1e3) ra cực [-28.77, -10.00, -8.52] -- không ω nào tái
      tạo được, vì "omega_lqr" luôn ép 3 cực trùng nhau.

  "place"  -- gán cực trực tiếp, K = [ω³, 3ω², 3ω], KHÔNG giải Riccati.
      Kết quả y hệt "omega_lqr". Giữ lại để đối chiếu, và để chạy được khi
      không có scipy.

Node in rõ mode đang chạy và CẢNH BÁO nếu bạn đặt tham số thuộc mode khác --
để không lặp lại chuyện tham số ngồi trong yaml mà im lặng không có tác dụng.

===========================================================================
MÁY TRẠNG THÁI
===========================================================================
    WAIT      chờ /joint_states có số thật
    GRAVITY   chỉ phát τ = G(q). Trạng thái an toàn mặc định, và là trạng thái
              node phát ra TRƯỚC KHI bạn activate controller -- để lúc
              controller vừa active nó đã có số hợp lệ, không phải NaN.
              Đây cũng là PHÉP THỬ DẤU MÔ-MEN an toàn nhất: dấu đúng thì tay
              nhẹ bẫng; dấu sai thì rơi nhanh gấp đôi lúc buông tay. Không có
              vòng kín nên không có gì leo thang.
    APPROACH  đa thức bậc 5 từ q hiện tại về q_ref(0)
    TRACK     bám quỹ đạo quét
    HOLD      giữ điểm cuối
    ABORT     bất thường -> tụt về τ = G(q) và ở lì đó

===========================================================================
NĂM CHỐT AN TOÀN -- đừng nới cái nào cho lần chạy đầu
===========================================================================
  tau_scale 0.30        KHÔNG thấp hơn: |G| đỉnh ở elbow là 1.518 Nm = 30.4%
                        trần URDF. Thấp hơn thì tay không tự giữ nổi DÙ dấu
                        mô-men đúng -> chẩn đoán sai thành lỗi dấu.
  max_track_error_rad   0.05 cho lần đầu. Nếu dấu mô-men sai thì vòng kín có
    0.05                cực dương +18.96 rad/s (ω=5): sai số gấp 10 sau 121 ms.
                        Ở 0.35 thì lúc abort bắt được, base đã có 6.0 rad/s.
  i_limit 0.004         Mặc định 0.5 của lqi_controller.py cho phép RIÊNG khâu
                        tích phân ra lệnh 200 Nm ở base (trần 5 Nm).
  joint_margin_rad      chạm gần giới hạn khớp -> ABORT
  state_timeout         /joint_states cũ quá -> ABORT

CHẾ ĐỘ MÔ-MEN KHÔNG CÓ LƯỚI AN TOÀN. PC chết là tay rơi. Bring-up trên bàn có
kê đỡ, tháo khỏi người.
"""

import csv
import os
import tempfile
import time

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.interpolate import CubicSpline
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from gim_control import sweep_trajectory
from gim_control.arm_dynamics import ArmDynamics
from gim_control.gim_arm_kinematics import GimArmKinematics
from gim_control.lqi_controller import LqiController, LqiWeights

WAIT, GRAVITY, APPROACH, TRACK, HOLD, ABORT = (
    "WAIT", "GRAVITY", "APPROACH", "TRACK", "HOLD", "ABORT")

OMEGA_LQR, WEIGHTS, PLACE = "omega_lqr", "weights", "place"


# ----------------------------------------------------------------------
# Quỹ đạo tham chiếu
# ----------------------------------------------------------------------
class PeriodicSpline:
    """Spline bậc 3 TUẦN HOÀN qua các điểm IK -> có q̈_ref giải tích.

    LQI cần q̈_ref cho phần feedforward M(q)q̈_ref. Sai phân hữu hạn hai lần
    trên lưới 0.3 s ra gia tốc đầy răng cưa -> mô-men giật theo, tay rung.
    bc_type='periodic' còn cho nối liền mạch cả q̇ và q̈ ở chỗ khép vòng.
    """

    def __init__(self, q_way, dt_way):
        q_way = np.asarray(q_way, float)
        n = len(q_way)
        self.period = n * dt_way
        ts = np.arange(n + 1) * dt_way
        self.sp = CubicSpline(ts, np.vstack([q_way, q_way[:1]]), axis=0,
                              bc_type="periodic")

    def at(self, t):
        tt = float(t) % self.period
        return self.sp(tt), self.sp(tt, 1), self.sp(tt, 2)


class Quintic:
    """Đi êm từ tư thế hiện tại về q_ref(0). Bậc 5 vì bậc 3 có q̈ ≠ 0 ở hai đầu
    -> mô-men feedforward nhảy bậc. Bước này BẮT BUỘC: bám thẳng vào q_ref(0)
    khi tay cách 0.5 rad thì k_e·e·M ra hàng chục Nm ngay chu kỳ đầu."""

    def __init__(self, q0, q1, T):
        self.q0 = np.asarray(q0, float)
        self.q1 = np.asarray(q1, float)
        self.T = float(T)

    def at(self, t):
        if self.T <= 0:
            z = np.zeros_like(self.q1)
            return self.q1.copy(), z, z.copy()
        s = np.clip(t / self.T, 0.0, 1.0)
        dq = self.q1 - self.q0
        h = 10 * s**3 - 15 * s**4 + 6 * s**5
        hd = (30 * s**2 - 60 * s**3 + 30 * s**4) / self.T
        hdd = (60 * s - 180 * s**2 + 120 * s**3) / self.T**2
        return self.q0 + h * dq, hd * dq, hdd * dq


class Hold:
    def __init__(self, q):
        self.q = np.asarray(q, float)

    def at(self, t):
        z = np.zeros_like(self.q)
        return self.q.copy(), z, z.copy()


def solve_waypoints(urdf, cache, log):
    """Giải IK cả vòng, dùng ĐÚNG định nghĩa quỹ đạo của đường PID.

    Import từ sweep_trajectory chứ không chép tham số sang đây: phép so sánh
    LQI/PID chỉ có nghĩa nếu hai bên bám đúng một quỹ đạo.
    """
    mt = os.path.getmtime(urdf)
    if cache and os.path.exists(cache):
        try:
            c = np.load(cache)
            if float(c["mt"]) == mt and int(c["n"]) == sweep_trajectory.N_POINTS:
                log.info(f"Dùng waypoint đã cache: {cache}")
                return c["q"], float(c["dt"])
        except Exception:
            pass
    kin = GimArmKinematics(urdf, end_effector_frame="lower_arm_link",
                           tool_offset_xyz=sweep_trajectory.TOOL_OFFSET)
    positions, results = sweep_trajectory.solve(kin)
    ok, lines = sweep_trajectory.safety_report(kin, positions, results)
    for ln in lines:
        log.info(ln)
    if not ok:
        raise RuntimeError(
            "safety_report() BÁO HỎNG -> không sinh quỹ đạo. Đây là cùng cổng "
            "kiểm mà đường PID phải qua; bỏ qua nó cho LQI là tự bỏ lưới an toàn."
        )
    q = np.array([r.q for r in results], float)
    if cache:
        np.savez(cache, q=q, dt=float(sweep_trajectory.DT), mt=mt,
                 n=sweep_trajectory.N_POINTS)
    return q, float(sweep_trajectory.DT)


# ----------------------------------------------------------------------
class LqiNode(Node):

    def __init__(self):
        super().__init__("lqi_node")
        p = self.declare_parameter
        p("urdf_file", "")
        p("cache_file", os.path.join(tempfile.gettempdir(), "gim_lqi_way.npz"))
        p("command_topic", "/lqi_effort_controller/commands")
        p("control_hz", 100.0)

        # --- chọn hệ số: xem docstring đầu file ---
        p("tune_mode", WEIGHTS)       # weights | omega_lqr | place
        p("omega", 5.0)               # dùng cho omega_lqr và place
        p("q_int", 6.0e6)             # chỉ dùng khi tune_mode = weights
        p("q_pos", 1.5e5)
        p("q_vel", 1.0e3)
        p("r", 1.0)

        # gravity_scale: nhân vào G(q) trước khi gửi xuống, RIÊNG TỪNG KHỚP.
        #
        # Đây KHÔNG phải nút chỉnh bộ điều khiển -- nó là hệ số HIỆU CHỈNH MÔ
        # HÌNH, và cũng là một phép đo. Gọi c là hệ số quy đổi thật (kể cả dấu),
        # mô-men tác động lên khớp là c·α·G còn trọng lực là G, nên cân bằng khi
        #     α = 1/c
        # Suy ra: NẾU tồn tại α > 0 làm khớp lơ lửng thì c > 0, tức DẤU ĐÚNG.
        # Nếu quét α từ 0 tới 2 mà khớp rơi ở mọi giá trị thì c < 0, dấu sai, và
        # không α nào cứu được. Đó là cách phân biệt "sai dấu" với "sai hệ số".
        #
        # α = 0 cho mốc chuẩn "rơi tự do trông như thế nào" để đối chiếu.
        # Đọc lại MỖI CHU KỲ nên đổi được bằng `ros2 param set` mà không restart.
        p("gravity_scale", [1.0, 1.0, 1.0])

        p("i_limit", 0.004)
        p("friction_ff", False)

        # --- an toàn ---
        p("tau_scale", 0.30)
        p("max_track_error_rad", 0.05)
        p("joint_margin_rad", 0.05)
        p("state_timeout", 0.25)

        # --- kịch bản ---
        p("approach_time", 5.0)
        p("loops", 2.0)
        p("autostart", False)
        p("log_file", "")

        g = self.get_parameter
        self.dt_nom = 1.0 / float(g("control_hz").value)
        self.tau_scale = float(g("tau_scale").value)
        self.max_err = float(g("max_track_error_rad").value)
        self.margin = float(g("joint_margin_rad").value)
        self.timeout = float(g("state_timeout").value)
        self.T_app = float(g("approach_time").value)
        self.loops = float(g("loops").value)

        self.phase, self.q, self.qd = WAIT, None, None
        self.stamp = self.phase_t0 = self._prev = None
        self.rows = []

        urdf = str(g("urdf_file").value) or self.find_urdf()
        self.get_logger().info(f"URDF: {urdf}")
        self.dyn = ArmDynamics(urdf)
        self.names = list(self.dyn.joint_names)
        self.n = len(self.names)
        self.tau_lim = np.asarray(self.dyn.tau_max, float) * self.tau_scale

        self.ctrl = self.build_controller()

        self.get_logger().info("\n" + self.ctrl.describe(
            q_nominal=(self.dyn.q_min + self.dyn.q_max) / 2.0))
        self.get_logger().info(
            f"Trần mô-men đang dùng (Nm): {np.round(self.tau_lim, 3)}  "
            f"(trần URDF × tau_scale {self.tau_scale})")
        a0 = self.gravity_scale()
        self.get_logger().info(f"gravity_scale = {np.round(a0, 3)}")
        if not np.allclose(a0, 1.0):
            self.get_logger().warn(
                "gravity_scale KHÁC 1.0 -> đang chạy với mô hình trọng lực đã "
                "hiệu chỉnh. Ghi lại giá trị này cùng mọi số đo, nếu không kết "
                "quả sẽ không tái lập được.")
        self.check_i_limit(float(g("i_limit").value))

        qw, dtw = solve_waypoints(urdf, str(g("cache_file").value), self.get_logger())
        self.traj = PeriodicSpline(qw, dtw)
        self.get_logger().info(
            f"Quỹ đạo {len(qw)} điểm, chu kỳ {self.traj.period:.1f}s, "
            f"chạy {self.loops:g} vòng.")

        self.pub = self.create_publisher(
            Float64MultiArray, str(g("command_topic").value), 10)
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.create_timer(self.dt_nom, self.tick)
        self.get_logger().info(
            "Sẵn sàng. Node đang ở GRAVITY (chỉ bù trọng lực). "
            "`ros2 param set /lqi_node autostart true` để bắt đầu bám.")

    # ------------------------------------------------------------------
    def build_controller(self):
        """Dựng LqiController theo tune_mode, và NÓI RÕ tham số nào bị bỏ qua."""
        g = self.get_parameter
        mode = str(g("tune_mode").value).strip().lower()
        w = float(g("omega").value)
        kw = dict(friction_ff=bool(g("friction_ff").value),
                  i_limit=float(g("i_limit").value), tau_limit=self.tau_lim)
        log = self.get_logger()

        if mode == OMEGA_LQR:
            if w <= 0.0:
                raise ValueError(f"tune_mode=omega_lqr cần omega > 0, đang là {w}")
            # Q = diag(ω⁶, 3ω⁴, 3ω²), R = 1 -> K = [ω³, 3ω², 3ω].
            # Kiểm bằng số: trùng với gán cực tới 1e-12. Đi qua CARE thật, nên
            # describe() sẽ in "Nguồn hệ số: LQR, Q = diag(...)".
            weights = LqiWeights(q_int=w**6, q_pos=3.0 * w**4,
                                 q_vel=3.0 * w**2, r=1.0)
            log.info(
                f"tune_mode = omega_lqr, omega = {w:g} rad/s\n"
                f"  -> Q = diag({w**6:.4g}, {3.0*w**4:.4g}, {3.0*w**2:.4g}), R = 1\n"
                f"  -> giải Riccati (solve_continuous_are) để ra K\n"
                f"  BỎ QUA: q_int, q_pos, q_vel, r trong yaml")
            return LqiController(self.dyn, weights=weights, **kw)

        if mode == WEIGHTS:
            weights = LqiWeights(
                q_int=float(g("q_int").value), q_pos=float(g("q_pos").value),
                q_vel=float(g("q_vel").value), r=float(g("r").value))
            log.info(
                f"tune_mode = weights\n"
                f"  -> Q = diag({weights.q_int:.4g}, {weights.q_pos:.4g}, "
                f"{weights.q_vel:.4g}), R = {weights.r:g}\n"
                f"  -> giải Riccati (solve_continuous_are) để ra K\n"
                f"  BỎ QUA: omega trong yaml")
            return LqiController(self.dyn, weights=weights, **kw)

        if mode == PLACE:
            if w <= 0.0:
                raise ValueError(f"tune_mode=place cần omega > 0, đang là {w}")
            log.info(
                f"tune_mode = place, omega = {w:g} rad/s\n"
                f"  -> K = [ω³, 3ω², 3ω] trực tiếp, KHÔNG giải Riccati\n"
                f"  -> kết quả y hệt omega_lqr\n"
                f"  BỎ QUA: q_int, q_pos, q_vel, r trong yaml")
            return LqiController(self.dyn, bandwidth=w, **kw)

        raise ValueError(
            f"tune_mode = '{mode}' không hợp lệ. Chọn một trong: "
            f"'{OMEGA_LQR}', '{WEIGHTS}', '{PLACE}'.")

    def gravity_scale(self):
        """Đọc lại mỗi chu kỳ để đổi được lúc đang chạy."""
        a = np.asarray(self.get_parameter("gravity_scale").value, dtype=float)
        if a.size == 1:
            a = np.full(self.n, float(a))
        if a.size != self.n:
            self.get_logger().warn(
                f"gravity_scale phải có {self.n} phần tử, đang có {a.size}. Dùng 1.0.")
            a = np.ones(self.n)
        return a

    def find_urdf(self):
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("gim_arm_description"),
                            "urdf", "gim_arm.urdf")

    def check_i_limit(self, il):
        # np.maximum.reduce, KHÔNG phải max(): max() so cả MẢNG -> ValueError.
        M = np.maximum.reduce([np.diag(self.dyn.mass_matrix(q)) for q in
                               (self.dyn.q_min, self.dyn.q_max,
                                (self.dyn.q_min + self.dyn.q_max) / 2)])
        kick = self.ctrl.K[:, 0] * il * M
        if np.any(kick > 0.5 * self.tau_lim):
            self.get_logger().warn(
                f"i_limit={il} cho phép RIÊNG khâu tích phân ra lệnh "
                f"{np.round(kick, 2)} Nm so với trần {np.round(self.tau_lim, 2)} Nm. "
                "Giảm i_limit trước khi chạy trên tay thật.")

    # ------------------------------------------------------------------
    def on_state(self, msg):
        try:
            idx = [msg.name.index(j) for j in self.names]
        except ValueError:
            return
        self.q = np.array([msg.position[i] for i in idx], float)
        self.qd = (np.array([msg.velocity[i] for i in idx], float)
                   if len(msg.velocity) >= len(msg.name) else np.zeros(self.n))
        self.stamp = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        dt = self.dt_nom if self._prev is None else max(
            1e-4, (now - self._prev).nanoseconds * 1e-9)
        self._prev = now
        if self.q is None:
            return

        age = (now - self.stamp).nanoseconds * 1e-9
        if age > self.timeout:
            self.abort(f"/joint_states cũ {age*1000:.0f} ms")

        if self.phase == WAIT:
            self.phase = GRAVITY
            self.get_logger().info("WAIT -> GRAVITY (chỉ bù trọng lực)")

        if self.phase != ABORT and (
                np.any(self.q < self.dyn.q_min + self.margin)
                or np.any(self.q > self.dyn.q_max - self.margin)):
            self.abort(f"chạm giới hạn khớp: q = {np.round(self.q, 3)}")

        if self.phase == GRAVITY and bool(self.get_parameter("autostart").value):
            self.ref = Quintic(self.q, self.traj.at(0.0)[0], self.T_app)
            self.ctrl.reset()
            self.phase, self.phase_t0 = APPROACH, now
            self.get_logger().info(f"GRAVITY -> APPROACH ({self.T_app:g}s)")

        t = 0.0 if self.phase_t0 is None else (now - self.phase_t0).nanoseconds * 1e-9

        if self.phase == APPROACH and t >= self.T_app:
            self.phase, self.phase_t0, t = TRACK, now, 0.0
            self.get_logger().info("APPROACH -> TRACK")
        elif self.phase == TRACK and t >= self.loops * self.traj.period:
            self.ref = Hold(self.traj.at(0.0)[0])
            self.phase, self.phase_t0, t = HOLD, now, 0.0
            self.get_logger().info("TRACK -> HOLD (xong)")
            self.dump()

        if self.phase in (GRAVITY, ABORT):
            self.publish(self.grav())
            return
        if self.phase == TRACK:
            qr, qdr, qddr = self.traj.at(t)
        else:
            qr, qdr, qddr = self.ref.at(t)

        e = float(np.max(np.abs(self.q - qr)))
        if e > self.max_err:
            self.abort(f"sai số bám {e:.3f} rad > {self.max_err:.3f} "
                       "-- kiểm tra DẤU mô-men trước tiên")
            self.publish(self.grav())
            return

        tau = self.ctrl.compute(self.q, self.qd, qr, qdr, qddr, dt)
        # compute() đã cộng G(q) đầy đủ ở phần feedforward. Trừ bớt phần thừa
        # để hệ số hiệu chỉnh áp NHẤT QUÁN cho cả pha GRAVITY lẫn pha bám --
        # nếu chỉ áp ở grav() thì lúc bật autostart mô-men sẽ nhảy bậc.
        alpha = self.gravity_scale()
        if not np.allclose(alpha, 1.0):
            tau = tau - (1.0 - alpha) * self.dyn.gravity(self.q)
        self.publish(tau)
        if self.phase in (APPROACH, TRACK):
            self.rows.append([time.time(), self.phase, dt, *self.q, *self.qd,
                              *qr, *qdr, *tau, *self.ctrl.last["integral"]])

    # ------------------------------------------------------------------
    def grav(self):
        """τ = α·G(q): tay 'không trọng lượng', không bị kéo về đâu. Không phải
        lưới an toàn thật (bị đẩy là trôi), nhưng hơn hẳn phát 0 Nm (rơi)."""
        return np.clip(self.gravity_scale() * self.dyn.gravity(self.q),
                       -self.tau_lim, self.tau_lim)

    def publish(self, tau):
        m = Float64MultiArray()
        m.data = [float(x) for x in np.asarray(tau).ravel()]
        self.pub.publish(m)

    def abort(self, why):
        if self.phase == ABORT:
            return
        self.phase = ABORT
        self.get_logger().error(
            f"ABORT: {why}. Tụt về bù trọng lực. Deactivate "
            "lqi_effort_controller rồi kiểm tra trước khi chạy lại.")
        self.dump()

    def dump(self):
        path = str(self.get_parameter("log_file").value)
        if not path or not self.rows:
            return
        hdr = ["t_wall", "phase", "dt"]
        for pre in ("q", "qd", "qref", "qdref", "tau", "int"):
            hdr += [f"{pre}_{j}" for j in self.names]
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(hdr)
            w.writerows(self.rows)
        self.get_logger().info(f"Ghi {len(self.rows)} dòng -> {path}")


def main():
    rclpy.init()
    node = LqiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.dump()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()