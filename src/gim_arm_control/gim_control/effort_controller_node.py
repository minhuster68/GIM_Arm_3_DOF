"""
Runner ROS 2 dùng chung cho mọi thuật toán điều khiển mô-men.

Package thuật toán chỉ khai tham số và cung cấp một controller có giao diện
``compute(q, qd, q_ref, qd_ref, qdd_ref, dt) -> tau``. File này sở hữu mọi
thứ phải giống nhau khi so sánh: trajectory, state machine, safety, logging và
topic effort.
"""

import csv
import os
import tempfile
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

from gim_control.arm_dynamics import ArmDynamics
from gim_control.reference_trajectory import (
    Hold,
    Quintic,
    TimedHold,
    load_trajectory,
)


WAIT, GRAVITY, APPROACH, TRACK, RETURN, HOLD, ABORT = (
    "WAIT", "GRAVITY", "APPROACH", "TRACK", "RETURN", "HOLD", "ABORT")


class EffortControllerNode(Node):
    """State machine và lớp an toàn chung quanh một torque controller."""

    def __init__(self, factory):
        self.factory = factory
        super().__init__(f"{factory.algorithm_name}_controller")

        declare = self.declare_parameter
        declare("urdf_file", "")
        declare(
            "cache_file",
            os.path.join(tempfile.gettempdir(), "gim_effort_way.npz"),
        )
        declare("command_topic", "/gim_arm_effort_controller/commands")
        declare("hand_guiding_service", "/gim_arm/set_hand_guiding")
        declare("control_hz", 100.0)
        declare("use_armature", True)
        declare("gravity_scale", [1.0, 1.0, 1.0])
        declare("drag_kd", [0.3, 1.0, 0.3])
        declare("hold_kp", [5.0, 12.0, 4.0])
        declare("hold_kd", [1.0, 3.0, 0.8])
        declare("hold_ki", [0.8, 2.0, 0.8])
        declare("hold_i_torque_limit_nm", [0.5, 1.0, 0.5])
        declare("tau_scale", 0.35)
        declare("max_track_error_rad", 0.05)
        declare("start_velocity_limit_rad_s", 0.0)
        declare("max_transition_error_rad", 0.10)
        declare("joint_margin_rad", 0.05)
        declare("state_timeout", 0.25)
        declare("stale_recovery_threshold", 0.03)
        declare("stale_recovery_time", 0.25)
        declare("approach_time", 5.0)
        declare("return_time", 5.0)
        declare("loops", 1.0)
        declare("trajectory_shape", "circle")
        declare("diagnostic_hold", False)
        declare("cascade_hold", False)
        declare("diagnostic_segment", False)
        declare("diagnostic_q1_deg", 0.0)
        declare("diagnostic_q2_deg", 0.0)
        declare("diagnostic_q3_deg", 0.0)
        declare("diagnostic_hold_time", 15.0)
        declare("diagnostic_qd1_rad_s", 0.0)
        declare("diagnostic_qd2_rad_s", 0.0)
        declare("diagnostic_qd3_rad_s", 0.0)
        declare("diagnostic_segment_time", 2.0)
        declare("fixed_track_gain_index", -1)
        declare("autostart", False)
        declare("log_file", "")
        declare("deadline_warn_fraction", 0.8)
        declare("command_heartbeat_nm", 1.0e-6)
        factory.declare_parameters(self)

        get = self.get_parameter
        control_hz = float(get("control_hz").value)
        self.tau_scale = float(get("tau_scale").value)
        if control_hz <= 0.0:
            raise ValueError("control_hz phải > 0")
        if not 0.0 < self.tau_scale <= 1.0:
            raise ValueError("tau_scale phải nằm trong (0, 1]")

        self.dt_nom = 1.0 / control_hz
        self.max_track_error = float(get("max_track_error_rad").value)
        self.start_velocity_limit = float(get("start_velocity_limit_rad_s").value)
        if not np.isfinite(self.start_velocity_limit) or self.start_velocity_limit < 0:
            raise ValueError("start_velocity_limit_rad_s phải hữu hạn và >= 0")
        self.max_transition_error = float(
            get("max_transition_error_rad").value)
        self.joint_margin = float(get("joint_margin_rad").value)
        self.state_timeout = float(get("state_timeout").value)
        self.stale_recovery_threshold = float(
            get("stale_recovery_threshold").value)
        self.stale_recovery_time = float(get("stale_recovery_time").value)
        self.approach_time = float(get("approach_time").value)
        self.return_time = float(get("return_time").value)
        loops = float(get("loops").value)
        if loops <= 0.0 or not np.isclose(loops, round(loops)):
            raise ValueError("loops phải là số nguyên dương")
        self.loops = int(round(loops))
        if self.approach_time <= 0.0 or self.return_time <= 0.0:
            raise ValueError("approach_time và return_time phải > 0")
        if self.max_track_error <= 0.0 or self.max_transition_error <= 0.0:
            raise ValueError("các ngưỡng sai số phải > 0")
        if (self.stale_recovery_threshold < 0.0
                or self.stale_recovery_time < 0.0):
            raise ValueError(
                "stale_recovery_threshold/time phải >= 0")
        self.command_heartbeat = float(get("command_heartbeat_nm").value)
        if not 0.0 <= self.command_heartbeat <= 1.0e-3:
            raise ValueError("command_heartbeat_nm phải nằm trong [0, 1e-3]")
        self.heartbeat_sign = 1.0
        self.deadline_warn = self.dt_nom * float(
            get("deadline_warn_fraction").value)

        self.phase = WAIT
        self.q = self.qd = self.measured_effort = self.state_stamp = None
        self.previous_tick = None
        self.phase_elapsed = 0.0
        self.home_q = self.reference = None
        self.hand_guiding = False
        self.manual_hold_latched = False
        self.autostart_block_warned = False
        self.rows = []
        self.state_rows = []
        self.last_dump_count = 0
        self.deadline_misses = 0
        self.diagnostic_lqr = (factory.algorithm_name == "lqr"
                               and bool(get("log_file").value))
        self.diagnostic_mpc = (factory.algorithm_name == "mpc"
                               and bool(get("log_file").value))
        self.diagnostic_pid = (factory.algorithm_name == "cascade_pid"
                               and bool(get("log_file").value))
        self.diagnostic_control = (
            self.diagnostic_lqr or self.diagnostic_mpc or self.diagnostic_pid)
        self.cascade_hold = bool(get("cascade_hold").value)
        if self.cascade_hold and factory.algorithm_name != "cascade_pid":
            raise ValueError("cascade_hold chỉ dùng với cascade PID")
        self.mpc_solver_rejections = 0
        self.state_seq = 0
        self.last_control_state_seq = -1
        self.stale_state_skips = 0
        self.consecutive_stale_time = 0.0
        self.recovery_steps_remaining = 0
        self.recovery_target = None
        self.state_header_ns = -1
        self.state_rx_wall_ns = -1
        self.clock_sim_ns = -1
        self.clock_rx_wall_ns = -1
        self.previous_logged_k = None
        self.last_command = None
        self.last_publish_wall_ns = -1

        urdf = str(get("urdf_file").value) or self._find_urdf()
        self.dynamics = ArmDynamics(
            urdf, use_armature=bool(get("use_armature").value))
        self.get_logger().info(
            "Quán tính rotor cộng vào mô hình M(q): "
            f"{np.round(self.dynamics.armature, 8)} kg·m² "
            f"(use_armature={bool(get('use_armature').value)})")
        self.joint_names = list(self.dynamics.joint_names)
        self.n = len(self.joint_names)
        self.tau_limit = np.asarray(self.dynamics.tau_max) * self.tau_scale
        self.drag_kd = self._gain_vector("drag_kd")
        self.hold_kp = self._gain_vector("hold_kp")
        self.hold_kd = self._gain_vector("hold_kd")
        self.hold_ki = self._gain_vector("hold_ki")
        self.hold_i_torque_limit = self._gain_vector(
            "hold_i_torque_limit_nm")
        self.hold_integral = np.zeros(self.n)
        self.controller = factory.build(
            self, self.dynamics, self.tau_limit, control_hz)
        self.diagnostic_segment = bool(get("diagnostic_segment").value)
        self.fixed_track_gain_index = int(
            get("fixed_track_gain_index").value)
        if self.fixed_track_gain_index < -1:
            raise ValueError("fixed_track_gain_index phải >= -1")
        if self.fixed_track_gain_index >= 0 and (
                factory.algorithm_name != "lqr"
                or not self.diagnostic_segment):
            raise ValueError(
                "fixed_track_gain_index chỉ dùng với LQR diagnostic_segment")
        self.controller_schedule_ready = False

        description = self.controller.describe(
            q_nominal=(self.dynamics.q_min + self.dynamics.q_max) / 2.0)
        self.get_logger().info(
            f"Thuật toán: {factory.algorithm_name}\n{description}")
        self.get_logger().info(
            f"Trần controller: {np.round(self.tau_limit, 3)} Nm "
            f"(effort URDF {np.round(self.dynamics.tau_max, 3)} "
            f"× {self.tau_scale:g})")
        self.get_logger().info(
            "HAND_GUIDING dùng gravity + damping: "
            f"Kd_drag={np.round(self.drag_kd, 3)} Nm/(rad/s)")
        if self.cascade_hold:
            self.get_logger().info(
                "PREP/HOLD_HOME dùng cùng cascade PID như quỹ đạo; "
                "tau_cmd = inverse_dynamics(ref) + tau_fb")
        else:
            self.get_logger().info(
                "HOLD_HOME dùng gravity-hold PID: "
                f"Kp={np.round(self.hold_kp, 3)} Nm/rad, "
                f"Kd={np.round(self.hold_kd, 3)} Nm/(rad/s), "
                f"Ki={np.round(self.hold_ki, 3)} Nm/(rad*s), "
                f"|tau_i|<={np.round(self.hold_i_torque_limit, 3)} Nm")

        self.diagnostic_hold = bool(get("diagnostic_hold").value)
        if self.diagnostic_hold and self.diagnostic_segment:
            raise ValueError(
                "Chỉ chọn một trong diagnostic_hold và diagnostic_segment")
        if self.diagnostic_hold or self.diagnostic_segment:
            q_target_deg = np.asarray([
                get("diagnostic_q1_deg").value,
                get("diagnostic_q2_deg").value,
                get("diagnostic_q3_deg").value,
            ], dtype=float)
            q_target = np.radians(q_target_deg)
            lower = self.dynamics.q_min + self.joint_margin
            upper = self.dynamics.q_max - self.joint_margin
            if (not np.all(np.isfinite(q_target))
                    or np.any(q_target < lower)
                    or np.any(q_target > upper)):
                raise ValueError(
                    "diagnostic_q*_deg phải hữu hạn và cách giới hạn khớp "
                    f"ít nhất {self.joint_margin:g} rad; nhận "
                    f"{np.round(q_target_deg, 3)} deg")
            self.loops = 1
            if self.diagnostic_hold:
                hold_time = float(get("diagnostic_hold_time").value)
                self.trajectory = TimedHold(q_target, hold_time)
                self.get_logger().warn(
                    "CHẾ ĐỘ CHẨN ĐOÁN GIỮ TƯ THẾ: "
                    f"q_ref={np.round(q_target_deg, 3)} deg trong "
                    f"{hold_time:g}s")
                track_description = f"TRACK_HOLD {hold_time:g}s"
            else:
                qd_mid = np.asarray([
                    get("diagnostic_qd1_rad_s").value,
                    get("diagnostic_qd2_rad_s").value,
                    get("diagnostic_qd3_rad_s").value,
                ], dtype=float)
                duration = float(get("diagnostic_segment_time").value)
                if (not np.all(np.isfinite(qd_mid))
                        or not np.isfinite(duration) or duration <= 0.0):
                    raise ValueError(
                        "diagnostic_qd*_rad_s phải hữu hạn và "
                        "diagnostic_segment_time phải > 0")
                # Quintic có h'(0.5)=1.875; tâm đoạn khớp đúng (q, qdot)
                # đã đo ở vùng lỗi, còn hai đầu đều có qdot=qddot=0.
                delta = qd_mid * duration / 1.875
                q_start = q_target - 0.5 * delta
                q_end = q_target + 0.5 * delta
                if (np.any(q_start < lower) or np.any(q_start > upper)
                        or np.any(q_end < lower) or np.any(q_end > upper)):
                    raise ValueError(
                        "Hai đầu diagnostic_segment vượt giới hạn khớp")
                self.trajectory = Quintic(q_start, q_end, duration)
                self.get_logger().warn(
                    "CHẾ ĐỘ CHẨN ĐOÁN ĐOẠN NGẮN: "
                    f"q_mid={np.round(q_target_deg, 3)} deg, "
                    f"qd_mid={np.round(qd_mid, 4)} rad/s, "
                    f"q_start={np.round(np.degrees(q_start), 3)} deg, "
                    f"q_end={np.round(np.degrees(q_end), 3)} deg")
                track_description = f"TRACK_SEGMENT {duration:g}s"
        else:
            self.trajectory = load_trajectory(
                urdf, str(get("cache_file").value), self.get_logger(),
                str(get("trajectory_shape").value))
            track_description = (
                f"TRACK {self.loops} vòng "
                f"({self.trajectory.duration:g}s/vòng)")
        self.get_logger().info(
            f"Kịch bản: HOME -> APPROACH {self.approach_time:g}s -> "
            f"{track_description} -> RETURN {self.return_time:g}s "
            "-> HOLD_HOME")

        self.publisher = self.create_publisher(
            Float64MultiArray, str(get("command_topic").value), 10)
        self.create_subscription(
            JointState, "/joint_states", self._on_state, 10)
        if self.diagnostic_lqr:
            self.create_subscription(
                Clock, "/clock", self._on_clock,
                QoSProfile(depth=1,
                           reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_service(
            SetBool, str(get("hand_guiding_service").value),
            self._set_hand_guiding)
        self.create_timer(self.dt_nom, self._tick)
        if self.cascade_hold:
            self.get_logger().info(
                "Chờ /joint_states để chốt HOME bằng cascade PID. "
                "Chỉ bật autostart sau khi effort controller đã được activate.")
        else:
            self.get_logger().info(
                "Sẵn sàng ở GRAVITY_HOLD. Service /gim_arm/set_hand_guiding: "
                "true=kéo tay, false=chốt và giữ vị trí hiện tại. Chỉ bật "
                "autostart sau khi effort controller đã được activate.")

    @staticmethod
    def _find_urdf():
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory("gim_arm_description"),
            "urdf", "gim_arm.urdf")

    def _gravity_scale(self):
        scale = np.asarray(
            self.get_parameter("gravity_scale").value, dtype=float)
        if scale.size == 1:
            scale = np.full(self.n, float(scale.ravel()[0]))
        if scale.size != self.n or not np.all(np.isfinite(scale)):
            self.get_logger().error("gravity_scale không hợp lệ; dùng [1,1,1]")
            return np.ones(self.n)
        return scale

    def _gain_vector(self, name):
        value = np.asarray(self.get_parameter(name).value, dtype=float)
        if value.size == 1:
            value = np.full(self.n, float(value.ravel()[0]))
        if (value.size != self.n or not np.all(np.isfinite(value))
                or np.any(value < 0.0)):
            raise ValueError(
                f"{name} phải là số không âm hoặc vector {self.n} phần tử")
        return value

    def _prepare_controller_schedule(self, home_q):
        """Precompute an optional gain schedule before effort mode starts.

        Return ``True`` when the callback was deliberately blocked for
        precomputation, so the caller can discard the now-stale joint sample.
        """
        prepare = getattr(self.controller, "precompute_gain_schedule", None)
        schedule_hz = float(getattr(
            self.controller, "gain_schedule_hz", 0.0))
        if not callable(prepare) or schedule_hz <= 0.0:
            self.controller_schedule_ready = True
            return False

        def sample(reference, duration):
            count = max(2, int(np.ceil(duration * schedule_hz)) + 1)
            times = np.linspace(0.0, duration, count)
            values = [reference.at(sample_time) for sample_time in times]
            return (
                np.asarray([value[0] for value in values], dtype=float),
                np.asarray([value[1] for value in values], dtype=float),
            )

        target_q = self.trajectory.at(0.0)[0]
        sweep_end_q = self.trajectory.at(self.trajectory.duration)[0]
        references = (
            (Quintic(home_q, target_q, self.approach_time),
             self.approach_time),
            (self.trajectory, self.trajectory.duration),
            (Quintic(sweep_end_q, home_q, self.return_time),
             self.return_time),
        )
        q_parts, qd_parts = [], []
        for reference, duration in references:
            q_ref, qd_ref = sample(reference, duration)
            q_parts.append(q_ref)
            qd_parts.append(qd_ref)

        q_schedule = np.vstack(q_parts)
        qd_schedule = np.vstack(qd_parts)
        started = time.perf_counter()
        self.get_logger().info(
            f"Bắt đầu precompute {len(q_schedule)} gain LQR "
            f"@ {schedule_hz:g} Hz. Giữ position controller active...")
        count = prepare(q_schedule, qd_schedule)
        if self.fixed_track_gain_index >= count:
            raise ValueError(
                f"fixed_track_gain_index={self.fixed_track_gain_index} "
                f"ngoài bảng {count} gain")
        elapsed = time.perf_counter() - started
        self.controller_schedule_ready = True
        self.get_logger().info(
            f"Precompute xong {count} gain trong {elapsed:.2f}s; "
            "runtime sẽ chỉ tra bảng, không giải Riccati. Đang chờ "
            "mẫu /joint_states mới...")
        return True

    def _on_state(self, msg):
        try:
            indices = [msg.name.index(name) for name in self.joint_names]
        except ValueError:
            return
        self.q = np.asarray([msg.position[i] for i in indices], dtype=float)
        self.qd = (
            np.asarray([msg.velocity[i] for i in indices], dtype=float)
            if len(msg.velocity) >= len(msg.name) else np.zeros(self.n))
        self.measured_effort = (
            np.asarray([msg.effort[i] for i in indices], dtype=float)
            if len(msg.effort) >= len(msg.name)
            else np.full(self.n, np.nan))
        self.state_seq += 1
        self.state_header_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec))
        self.state_rx_wall_ns = time.time_ns()
        if self.diagnostic_lqr:
            if self.phase in (APPROACH, TRACK, RETURN):
                self.state_rows.append([
                    self.state_header_ns, self.state_rx_wall_ns,
                    *self.q, *self.qd, *self.measured_effort,
                ])
        self.state_stamp = self.get_clock().now()

    def _on_clock(self, msg):
        self.clock_sim_ns = (
            int(msg.clock.sec) * 1_000_000_000
            + int(msg.clock.nanosec))
        self.clock_rx_wall_ns = time.time_ns()

    def _append_tracking_row(self, tick_wall_ns, phase_elapsed, dt,
                             compute_time, q_ref, qd_ref, tau,
                             event="CONTROL"):
        row = [
            tick_wall_ns * 1e-9, self.phase, dt, compute_time,
            *self.q, *self.qd, *q_ref, *qd_ref, *tau,
        ]
        if self.diagnostic_control:
            row.extend([
                event, phase_elapsed, tick_wall_ns,
                self.last_publish_wall_ns, self.state_seq,
                self.state_header_ns, self.state_rx_wall_ns,
            ])
        if self.diagnostic_lqr:
            last = self.controller.last if event == "CONTROL" else {}
            gain = last.get("K")
            gain_delta = (
                float(np.linalg.norm(gain - self.previous_logged_k))
                if gain is not None and self.previous_logged_k is not None
                else float("nan"))
            if gain is not None:
                self.previous_logged_k = gain.copy()
            missing = np.full(self.n, np.nan)
            row.extend([
                self.clock_sim_ns, self.clock_rx_wall_ns,
                last.get("gain_schedule_index", -1), gain_delta,
                *(last.get("tau_ff", missing)),
                *(last.get("tau_raw", missing)),
                *(last.get("tau", missing)),
                *(last.get("integral", missing)),
                *(last.get("tau_integral", missing)),
                *(last.get("tau_position", missing)),
                *(last.get("tau_velocity", missing)),
            ])
        elif self.diagnostic_mpc:
            last = self.controller.last if event == "CONTROL" else {}
            missing = np.full(self.n, np.nan)
            row.extend([
                int(bool(last.get("solver_success", False))),
                int(bool(last.get("solver_accepted", False))),
                last.get("solver_status", -1),
                last.get("solve_time", float("nan")),
                last.get("iterations", -1),
                last.get("constraint_violation", float("nan")),
                last.get("primal_residual", float("nan")),
                last.get("dual_residual", float("nan")),
                last.get("solver_rho", float("nan")),
                *(last.get("tau_ff", missing)),
                *(last.get("tau_feedback", missing)),
                *(last.get("tau_raw", missing)),
                *(last.get("saturated", missing)),
            ])
        elif self.diagnostic_pid:
            last = self.controller.last if event == "CONTROL" else {}
            missing = np.full(self.n, np.nan)
            for name in (
                    "tau_ff", "tau_fb", "tau_p", "tau_i", "tau_raw",
                    "tau", "position_error", "velocity_command",
                    "velocity_error", "integral", "saturated"):
                row.extend(float(value) for value in last.get(name, missing))
            for name in ("kpp", "kvp", "kvi"):
                row.extend(getattr(self.controller, name))
        self.rows.append(row)

    def _set_hand_guiding(self, request, response):
        if self.q is None:
            response.success = False
            response.message = "Chưa có /joint_states"
            return response
        if self.phase not in (GRAVITY, HOLD):
            response.success = False
            response.message = (
                f"Không đổi clutch khi đang ở pha {self.phase}; "
                "chỉ cho phép ở GRAVITY_HOLD hoặc HOLD_HOME")
            return response

        if request.data:
            self.hand_guiding = True
            self.manual_hold_latched = False
            self.hold_integral.fill(0.0)
            self.home_q = self.q.copy()
            self.autostart_block_warned = False
            response.message = (
                "CLUTCH ON: HAND_GUIDING, có thể kéo tay robot")
            self.get_logger().warn(
                f"CLUTCH ON -> HAND_GUIDING tại q="
                f"{np.round(self.home_q, 4)}")
        else:
            self.hand_guiding = False
            self.manual_hold_latched = True
            self.home_q = self.q.copy()
            self.hold_integral.fill(0.0)
            if self.phase == HOLD:
                self.reference = Hold(self.home_q)
            self.autostart_block_warned = False
            response.message = (
                "CLUTCH OFF: đã chốt vị trí hiện tại và bật gravity-hold PID")
            self.get_logger().info(
                f"CLUTCH OFF -> giữ q_hold="
                f"{np.round(self.home_q, 4)}")
        response.success = True
        return response

    def _reference_at_phase(self, elapsed):
        """Return the active reference without advancing its logical clock."""
        if self.phase == TRACK:
            loop_time = min(
                elapsed % self.trajectory.duration,
                self.trajectory.duration)
            return self.trajectory.at(loop_time)
        return self.reference.at(elapsed)

    def _hold_command_for_stale_state(self, tick_wall_ns, timer_dt):
        """Keep the previous torque when the timer outruns joint states."""
        self.stale_state_skips += 1
        self.consecutive_stale_time += timer_dt
        if self.last_command is not None:
            self._publish(self.last_command)
        if (self.diagnostic_control
                and self.phase in (APPROACH, TRACK, RETURN)):
            q_ref, qd_ref, _ = self._reference_at_phase(
                self.phase_elapsed)
            held_tau = (
                self.last_command.copy()
                if self.last_command is not None
                else np.full(self.n, np.nan))
            self._append_tracking_row(
                tick_wall_ns, self.phase_elapsed, timer_dt,
                float("nan"), q_ref, qd_ref, held_tau,
                event="STALE_STATE_HOLD")
        if self.stale_state_skips in (1, 10, 100, 1000):
            self.get_logger().warn(
                "Bỏ qua control tick vì chưa có /joint_states mới; "
                f"giữ torque trước ({self.stale_state_skips} lần, "
                f"timer_dt={timer_dt*1000:.2f} ms)")

    def _begin_stale_recovery(self, stale_time):
        """Start a damped hold before handing control back to the algorithm."""
        steps = int(np.ceil(self.stale_recovery_time / self.dt_nom))
        if steps <= 0:
            return
        self.recovery_steps_remaining = steps
        self.recovery_target = self.q.copy()
        self.hold_integral.fill(0.0)
        self.controller.reset()
        self.get_logger().warn(
            "/joint_states đã ngừng "
            f"{stale_time*1000:.1f} ms; giữ tư thế và dập vận tốc "
            f"{self.stale_recovery_time:g}s trước khi chạy tiếp "
            f"pha {self.phase}")

    def _stale_recovery_torque(self):
        """Gravity/PD command used while state feedback settles after a gap."""
        gravity = self._gravity_scale() * self.dynamics.gravity(self.q)
        position = self.hold_kp * (self.recovery_target - self.q)
        # Some algorithm profiles deliberately use a very soft HOLD damping.
        # Recovery must at least retain the hand-guiding damping.
        damping_gain = np.maximum(self.hold_kd, self.drag_kd)
        damping = -damping_gain * self.qd
        return np.clip(
            gravity + position + damping,
            -self.tau_limit, self.tau_limit)

    def _run_stale_recovery(self, tick_wall_ns, timer_dt):
        """Freeze trajectory time and issue one valid damped-hold step."""
        q_ref, qd_ref, _ = self._reference_at_phase(self.phase_elapsed)
        tau = self._stale_recovery_torque()
        self._publish(tau)
        if (self.diagnostic_control
                and self.phase in (APPROACH, TRACK, RETURN)):
            self._append_tracking_row(
                tick_wall_ns, self.phase_elapsed, timer_dt,
                0.0, q_ref, qd_ref, tau,
                event="STALE_RECOVERY_HOLD")
        self.recovery_steps_remaining -= 1
        if self.recovery_steps_remaining <= 0:
            self.recovery_steps_remaining = 0
            self.recovery_target = None
            # Drop integral, warm-start and previous-feedback state accumulated
            # before the data gap. The final recovery command is close to
            # gravity after the velocity has settled, so this reset is smooth.
            self.controller.reset()
            self.get_logger().info(
                f"Kết thúc phục hồi /joint_states; tiếp tục {self.phase} "
                f"tại t={self.phase_elapsed:.3f}s")

    def _tick(self):
        tick_wall_ns = time.time_ns()
        now = self.get_clock().now()
        timer_dt = self.dt_nom if self.previous_tick is None else max(
            1e-4, (now - self.previous_tick).nanoseconds * 1e-9)
        self.previous_tick = now
        if self.q is None:
            return

        age = (now - self.state_stamp).nanoseconds * 1e-9
        # Khi use_sim_time vừa bật, callback joint_states có thể đến trước
        # mẫu /clock đầu tiên và mang mốc now() = 0. Bỏ qua đúng mẫu chuyển
        # clock này; callback kế tiếp sẽ cập nhật state_stamp theo giờ Gazebo.
        if self.state_stamp.nanoseconds > 0 and age > self.state_timeout:
            self._abort(f"/joint_states cũ {age*1000:.0f} ms")

        # Timer và subscription chạy trong cùng single-threaded executor. Sau
        # một scheduler stall, timer có thể được phục vụ trước các JointState
        # đang xếp hàng. Không được đưa mẫu q/qd cũ vào controller trong khi
        # reference đã tiến: đó là nguồn cú giật đã đo ở PID/LQR/MPC. Giữ lệnh
        # trước và đóng băng thời gian quỹ đạo cho tới khi state_seq thay đổi.
        if self.state_seq == self.last_control_state_seq:
            self._hold_command_for_stale_state(tick_wall_ns, timer_dt)
            return
        self.last_control_state_seq = self.state_seq
        stale_time = self.consecutive_stale_time
        self.consecutive_stale_time = 0.0
        # Mọi controller được thiết kế ở control_hz cố định. Một tick hợp lệ
        # luôn tương ứng đúng một bước mô hình; khoảng wall-time bị hụt không
        # được tích phân bù vào I-state hay slew constraint.
        dt = self.dt_nom

        if self.phase == WAIT:
            # Chụp HOME khi node vừa nhận state, lúc position controller vẫn
            # đang giữ tay. Nếu chờ đến sau khi switch sang effort, khoảng trễ
            # giữa hai lệnh CLI sẽ làm tư thế đã trôi bị ghi nhầm thành HOME.
            self.home_q = self.q.copy()
            if not self.controller_schedule_ready:
                try:
                    state_became_stale = self._prepare_controller_schedule(
                        self.home_q)
                except Exception as error:
                    self._abort(f"precompute gain LQR thất bại: {error}")
                    self._publish(self._gravity_torque())
                    return
                if state_became_stale:
                    # The single-threaded executor could not service the
                    # JointState subscription while solving the gain table.
                    # Force a fresh callback instead of tripping state_timeout
                    # on the sample that existed before precomputation.
                    self.q = None
                    self.qd = None
                    self.state_stamp = None
                    self.previous_tick = None
                    return
            self.phase = GRAVITY
            ready_phase = "READY_PID" if self.cascade_hold else "GRAVITY"
            self.get_logger().info(
                f"WAIT -> {ready_phase}, đã chụp HOME={np.round(self.home_q, 4)}")

        if self.phase != ABORT and (
                np.any(self.q < self.dynamics.q_min + self.joint_margin)
                or np.any(self.q > self.dynamics.q_max - self.joint_margin)):
            self._abort(f"gần giới hạn khớp: q={np.round(self.q, 3)}")

        if (
            self.phase == GRAVITY
            and bool(self.get_parameter("autostart").value)
        ):
            if self.hand_guiding:
                if not self.autostart_block_warned:
                    self.get_logger().warn(
                        "Chưa chạy quỹ đạo vì clutch vẫn ON; gọi "
                        "/gim_arm/set_hand_guiding false trước")
                    self.autostart_block_warned = True
                self._publish(self._drag_torque())
                return
            if (self.start_velocity_limit > 0
                    and np.max(np.abs(self.qd)) > self.start_velocity_limit):
                self._abort(
                    "tay chưa đứng yên trước quỹ đạo: "
                    f"qd={np.round(self.qd, 3)} rad/s, "
                    f"giới hạn {self.start_velocity_limit:g} rad/s")
                return
            # Bắt đầu APPROACH từ state thực sau switch để không tạo bước nhảy
            # reference, nhưng RETURN vẫn về HOME đã chụp trước switch.
            start_q = self.q.copy()
            target = self.trajectory.at(0.0)[0]
            self.reference = Quintic(start_q, target, self.approach_time)
            if not self.cascade_hold:
                self.controller.reset()
            self.hold_integral.fill(0.0)
            self.manual_hold_latched = False
            self.phase = APPROACH
            self.phase_elapsed = 0.0
            self.get_logger().info(
                f"GRAVITY -> APPROACH ({self.approach_time:g}s), "
                f"START={np.round(start_q, 4)}, "
                f"HOME={np.round(self.home_q, 4)}")

        # A state gap freezes not only the reference clock but also phase
        # transitions. Recover at the exact point where valid feedback ended.
        if (self.phase in (APPROACH, TRACK, RETURN)
                and stale_time > 0.0
                and stale_time >= self.stale_recovery_threshold
                and self.recovery_steps_remaining == 0):
            self._begin_stale_recovery(stale_time)
        if (self.phase in (APPROACH, TRACK, RETURN)
                and self.recovery_steps_remaining > 0):
            self._run_stale_recovery(tick_wall_ns, timer_dt)
            return

        elapsed = self.phase_elapsed
        if self.phase == APPROACH and elapsed >= self.approach_time:
            if self.fixed_track_gain_index >= 0:
                self.controller.set_fixed_gain_index(
                    self.fixed_track_gain_index)
                self.get_logger().warn(
                    f"TRACK dùng K_{self.fixed_track_gain_index} cố định")
            self.phase, self.phase_elapsed, elapsed = TRACK, 0.0, 0.0
            self.get_logger().info("APPROACH -> TRACK")
        elif (
            self.phase == TRACK
            and elapsed >= self.loops * self.trajectory.duration
        ):
            if self.fixed_track_gain_index >= 0:
                self.controller.set_fixed_gain_index(None)
            sweep_end = self.trajectory.at(self.trajectory.duration)[0]
            self.reference = Quintic(sweep_end, self.home_q, self.return_time)
            self.phase, self.phase_elapsed, elapsed = RETURN, 0.0, 0.0
            self.get_logger().info(f"TRACK -> RETURN ({self.return_time:g}s)")
        elif self.phase == RETURN and elapsed >= self.return_time:
            self.reference = Hold(self.home_q)
            if not self.cascade_hold:
                self.controller.reset()
            self.hold_integral.fill(0.0)
            self.phase, self.phase_elapsed, elapsed = HOLD, 0.0, 0.0
            # Không ghi CSV ngay trong callback điều khiển. Với 2 kHz, một
            # vòng tạo hàng chục nghìn dòng; ghi đồng bộ sẽ ngừng phát torque
            # gần một giây và tự kích hoạt state-timeout. File được ghi sạch
            # khi người dùng Ctrl-C node sau khi đã quan sát HOLD_HOME.
            hold_controller = "cascade PID" if self.cascade_hold else "gravity-hold PID"
            self.get_logger().info(
                f"RETURN -> HOLD_HOME {hold_controller} (Ctrl-C để ghi file log)")

        if self.phase == ABORT:
            self._publish(self._gravity_torque())
            return

        if self.phase in (GRAVITY, HOLD) and self.hand_guiding:
            # Cho q_hold chạy theo tay trong lúc clutch ON. Khi clutch OFF,
            # service sẽ chụp lại đúng mẫu q mới nhất rồi bật gravity-hold PID.
            self.home_q = self.q.copy()
            self.hold_integral.fill(0.0)
            self._publish(self._drag_torque())
            return

        if self.phase == GRAVITY:
            # Trước khi chạy quỹ đạo, giữ mềm ngay sau switch để không còn
            # khoảng trôi. Chỉ cho I tích lũy sau khi người dùng đã thả clutch
            # và chốt một q_hold mới; như vậy không windup lúc effort inactive.
            self._publish(self._hold_torque(
                dt, integrate=self.manual_hold_latched))
            if self.diagnostic_pid and self.cascade_hold:
                self._append_tracking_row(
                    tick_wall_ns, 0.0, timer_dt, float("nan"),
                    self.home_q, np.zeros(self.n), self.last_command)
            return

        q_ref, qd_ref, qdd_ref = self._reference_at_phase(elapsed)

        error = float(np.max(np.abs(self.q - q_ref)))
        error_limit = (
            self.max_transition_error
            if self.phase in (APPROACH, RETURN)
            else self.max_track_error)
        if error > error_limit:
            if (self.diagnostic_control
                    and self.phase in (APPROACH, TRACK, RETURN)):
                previous_tau = (
                    self.last_command
                    if self.last_command is not None
                    else np.zeros(self.n))
                self._append_tracking_row(
                    tick_wall_ns, elapsed, timer_dt, float("nan"),
                    q_ref, qd_ref, previous_tau,
                    event="ABORT_TRACK_ERROR_PREVIOUS_TAU")
            self._abort(
                f"sai số bám pha {self.phase} {error:.3f} rad "
                f"> {error_limit:.3f} rad")
            self._publish(self._gravity_torque())
            return

        if self.phase == HOLD and not self.cascade_hold:
            self._publish(self._hold_torque(dt, integrate=True))
            return

        started = time.perf_counter()
        tau = np.asarray(self.controller.compute(
            self.q, self.qd, q_ref, qd_ref, qdd_ref, dt), dtype=float)
        compute_time = time.perf_counter() - started
        if (self.factory.algorithm_name == "mpc"
                and not self.controller.last.get("solver_accepted", False)):
            self.mpc_solver_rejections += 1
            if self.mpc_solver_rejections in (1, 10, 100):
                last = self.controller.last
                self.get_logger().warn(
                    "MPC từ chối nghiệm solver, dùng warm-start khả thi: "
                    f"status={last.get('solver_status')}, "
                    f"violation={last.get('constraint_violation'):.3g} "
                    f"({self.mpc_solver_rejections} lần)")
        if compute_time > self.deadline_warn:
            self.deadline_misses += 1
            if self.deadline_misses in (1, 10, 100):
                self.get_logger().warn(
                    f"{self.factory.algorithm_name}: compute "
                    f"{compute_time*1000:.2f} ms "
                    f"> ngân sách cảnh báo {self.deadline_warn*1000:.2f} ms "
                    f"({self.deadline_misses} lần)")

        if tau.shape != (self.n,) or not np.all(np.isfinite(tau)):
            self._abort(f"controller trả torque không hợp lệ: {tau}")
            self._publish(self._gravity_torque())
            return

        scale = self._gravity_scale()
        if not np.allclose(scale, 1.0):
            tau -= (1.0 - scale) * self.dynamics.gravity(self.q)
        tau = np.clip(tau, -self.tau_limit, self.tau_limit)
        self._publish(tau)

        if self.phase in (APPROACH, TRACK, RETURN) or (
                self.phase == HOLD and self.cascade_hold):
            self._append_tracking_row(
                tick_wall_ns, elapsed, timer_dt, compute_time,
                q_ref, qd_ref, tau)
            self.phase_elapsed += self.dt_nom

    def _gravity_torque(self):
        return np.clip(
            self._gravity_scale() * self.dynamics.gravity(self.q),
            -self.tau_limit, self.tau_limit)

    def _drag_torque(self):
        gravity = self._gravity_scale() * self.dynamics.gravity(self.q)
        damping = -self.drag_kd * self.qd
        return np.clip(
            gravity + damping, -self.tau_limit, self.tau_limit)

    def _hold_torque(self, dt, integrate):
        if self.cascade_hold:
            if not integrate:
                self.controller.reset()
            zero = np.zeros(self.n)
            tau = self.controller.compute(
                self.q, self.qd, self.home_q, zero, zero, dt)
            scale = self._gravity_scale()
            tau = tau - (1.0 - scale) * self.dynamics.gravity(self.q)
            if not integrate:
                self.controller.integral.fill(0.0)
                self.controller.last['integral'] = self.controller.integral.copy()
            return np.clip(tau, -self.tau_limit, self.tau_limit)
        gravity = self._gravity_scale() * self.dynamics.gravity(self.q)
        error = self.home_q - self.q
        position = self.hold_kp * error
        damping = -self.hold_kd * self.qd
        base_tau = gravity + position + damping

        # Kẹp trực tiếp phần mô-men I để tham số có đơn vị dễ hiểu. Chỉ nhận
        # mẫu tích phân mới khi tổng torque chưa bão hòa, hoặc khi sai số đang
        # kéo torque ra khỏi bão hòa (conditional-integration anti-windup).
        dt_i = min(max(float(dt), 0.0), 5.0 * self.dt_nom)
        candidate = self.hold_integral + error * dt_i
        candidate_i_tau = np.clip(
            self.hold_ki * candidate,
            -self.hold_i_torque_limit,
            self.hold_i_torque_limit)
        candidate_tau = base_tau + candidate_i_tau
        saturated_high = candidate_tau > self.tau_limit
        saturated_low = candidate_tau < -self.tau_limit
        drives_back = ((saturated_high & (error < 0.0))
                       | (saturated_low & (error > 0.0)))
        accept_integral = ((self.hold_ki > 0.0)
                           & (~(saturated_high | saturated_low) | drives_back)
                           & bool(integrate))
        self.hold_integral = np.where(
            accept_integral, candidate, self.hold_integral)

        integral_tau = np.clip(
            self.hold_ki * self.hold_integral,
            -self.hold_i_torque_limit,
            self.hold_i_torque_limit)
        nonzero_ki = self.hold_ki > 0.0
        self.hold_integral = np.divide(
            integral_tau, self.hold_ki,
            out=np.zeros_like(self.hold_integral), where=nonzero_ki)
        return np.clip(
            base_tau + integral_tau, -self.tau_limit, self.tau_limit)

    def _publish(self, tau):
        command = np.asarray(tau, dtype=float).ravel().copy()
        if self.command_heartbeat > 0.0 and command.size:
            # forward_command_controller giữ nguyên giá trị cuối nếu publisher
            # chết. Hardware watchdog vì thế chỉ có thể nhận ra nguồn còn sống
            # khi vector command thay đổi. Đảo một lượng 1e-6 Nm ở base mỗi
            # chu kỳ: đủ khác ở command_interface double nhưng hoàn toàn không
            # đáng kể về cơ học. Nếu node chết, giá trị ngừng đảo và watchdog
            # vẫn fallback về G(q) sau effort_stale_cycles như thiết kế.
            self.heartbeat_sign *= -1.0
            command[0] += self.heartbeat_sign * self.command_heartbeat
            command = np.clip(command, -self.tau_limit, self.tau_limit)
        message = Float64MultiArray()
        message.data = [float(value) for value in command]
        self.publisher.publish(message)
        self.last_command = command
        self.last_publish_wall_ns = time.time_ns()

    def _abort(self, reason):
        if self.phase == ABORT:
            return
        self.phase = ABORT
        self.get_logger().error(
            f"ABORT: {reason}. Chuyển về gravity compensation; deactivate "
            "gim_arm_effort_controller trước khi kiểm tra.")
        # Ghi CSV tốc độ cao có thể mất vài trăm ms; phát lệnh an toàn trước.
        if self.q is not None:
            self._publish(self._gravity_torque())
        self.dump()

    def dump(self):
        path = str(self.get_parameter("log_file").value)
        if not path or not self.rows or len(self.rows) == self.last_dump_count:
            return
        header = ["t_wall", "phase", "dt", "compute_time"]
        for prefix in ("q", "qd", "qref", "qdref", "tau"):
            header.extend(f"{prefix}_{name}" for name in self.joint_names)
        if self.diagnostic_control:
            header.extend([
                "event", "phase_elapsed_s", "tick_wall_ns",
                "publish_wall_ns", "state_seq", "state_header_sim_ns",
                "state_rx_wall_ns",
            ])
        if self.diagnostic_lqr:
            header.extend([
                "clock_sim_ns", "clock_rx_wall_ns",
                "gain_schedule_index", "gain_delta_fro",
            ])
            for prefix in (
                    "tau_ff", "tau_raw", "tau_controller", "integral",
                    "tau_integral", "tau_position", "tau_velocity"):
                header.extend(f"{prefix}_{name}" for name in self.joint_names)
        elif self.diagnostic_mpc:
            header.extend([
                "solver_success", "solver_accepted", "solver_status",
                "solve_time", "solver_iterations", "constraint_violation",
                "primal_residual", "dual_residual", "solver_rho",
            ])
            for prefix in ("tau_ff", "tau_feedback", "tau_raw", "saturated"):
                header.extend(f"{prefix}_{name}" for name in self.joint_names)
        elif self.diagnostic_pid:
            for prefix in (
                    "tau_ff", "tau_fb", "tau_p", "tau_i", "tau_raw",
                    "tau_controller", "position_error", "velocity_command",
                    "velocity_error", "integral", "saturated", "kpp", "kvp", "kvi"):
                header.extend(f"{prefix}_{name}" for name in self.joint_names)
        self._write_csv_atomic(path, header, self.rows)
        if self.diagnostic_lqr and self.state_rows:
            state_path = os.path.splitext(path)[0] + "_states.csv"
            state_header = ["state_header_sim_ns", "state_rx_wall_ns"]
            for prefix in ("q", "qd", "effort"):
                state_header.extend(
                    f"{prefix}_{name}" for name in self.joint_names)
            self._write_csv_atomic(
                state_path, state_header, self.state_rows)
            self.get_logger().info(
                f"Ghi {len(self.state_rows)} mẫu /joint_states -> "
                f"{state_path}")
        self.last_dump_count = len(self.rows)
        if rclpy.ok():
            self.get_logger().info(f"Ghi {len(self.rows)} dòng -> {path}")

    @staticmethod
    def _write_csv_atomic(path, header, rows):
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                    mode="w", newline="", delete=False,
                    dir=os.path.dirname(os.path.abspath(path)),
                    prefix=".gim-log-") as stream:
                temporary_path = stream.name
                writer = csv.writer(stream)
                writer.writerow(header)
                writer.writerows(rows)
            os.replace(temporary_path, path)
        finally:
            if temporary_path is not None and os.path.exists(temporary_path):
                os.unlink(temporary_path)


def run_controller(factory):
    rclpy.init()
    node = EffortControllerNode(factory)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # Humble can race a subscription take with SIGINT shutdown and raise
        # instead of delivering KeyboardInterrupt.  Preserve real runtime
        # failures while keeping an intentional Ctrl-C shutdown clean.
        if rclpy.ok():
            raise
    finally:
        node.dump()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
