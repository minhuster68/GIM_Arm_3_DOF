import numpy as np

from gim_control.controller_api import ControllerFactory, vector_parameter

from .controller import LqrController, LqrWeights


class LqrFactory(ControllerFactory):
    algorithm_name = "lqr"

    def declare_parameters(self, node):
        node.declare_parameter("max_int_e", [10.0, 20.0, 10.0])
        node.declare_parameter("max_e", [0.05, 0.02, 0.01])
        position_tracking_only = node.declare_parameter(
            "position_tracking_only", False).value
        if not position_tracking_only:
            node.declare_parameter("max_de", [10.0, 10.0, 10.0])
        node.declare_parameter("max_tau", [5.0, 40.0, 5.0])
        node.declare_parameter("tau_penalty_scale", 1.0)
        node.declare_parameter("linearization_step", 1.0e-5)
        node.declare_parameter("integral_limit", 0.5)
        node.declare_parameter("gravity_at_measured", False)
        node.declare_parameter("recompute_every", 1)
        node.declare_parameter("require_discrete_stable", False)
        node.declare_parameter("max_tau_rate_nm_s", [0.0, 0.0, 0.0])
        node.declare_parameter("use_discrete_lqr", False)
        node.declare_parameter("gain_schedule_hz", 20.0)
        node.declare_parameter("gain_schedule_mode", "reference_state")
        node.declare_parameter("base_ff_alpha", 1.0)
        node.declare_parameter("feedback_model_mode", "coupled")
        node.declare_parameter("isolated_joint_index", -1)
        node.declare_parameter("independent_design_model", "position_held")
        node.declare_parameter("velocity_filter_type", "none")
        node.declare_parameter("velocity_kalman_measurement_std", [0.03, 0.01, 0.04])
        node.declare_parameter("velocity_kalman_jerk_std", [180.0, 60.0, 240.0])
        node.declare_parameter("velocity_kalman_reset_gap_s", 0.05)

    def build(self, node, dynamics, tau_limit, control_hz):
        n = dynamics.nq
        isolated = int(node.get_parameter('isolated_joint_index').value)
        if isolated >= 0 and not bool(node.get_parameter('diagnostic_hold').value):
            raise ValueError('isolated_joint_index chỉ dành cho bài tune giữ đơn khớp; '
                             'quỹ đạo đầy đủ cần kiểm tra robot ghép')
        filter_type = str(node.get_parameter('velocity_filter_type').value)
        velocity_filter = None
        if filter_type == 'kalman2':
            from gim_control.velocity_filter import VelocityAccelerationKalman
            velocity_filter = VelocityAccelerationKalman(
                vector_parameter(node, 'velocity_kalman_measurement_std', n, positive=True),
                vector_parameter(node, 'velocity_kalman_jerk_std', n, positive=True),
                1. / control_hz,
                node.get_parameter('velocity_kalman_reset_gap_s').value)
        elif filter_type != 'none':
            raise ValueError('velocity_filter_type cần none hoặc kalman2')
        configured_tau = vector_parameter(node, "max_tau", n)
        position_tracking_only = bool(
            node.get_parameter("position_tracking_only").value)
        weights = LqrWeights(
            max_int_e=vector_parameter(node, "max_int_e", n, positive=True),
            max_e=vector_parameter(node, "max_e", n, positive=True),
            max_de=(None if position_tracking_only else
                    vector_parameter(node, "max_de", n, positive=True)),
            position_tracking_only=position_tracking_only,
            max_tau=None if np.all(configured_tau <= 0.0) else configured_tau,
            tau_penalty_scale=float(
                node.get_parameter("tau_penalty_scale").value),
        )
        return LqrController(
            dynamics,
            weights=weights,
            i_limit=float(node.get_parameter("integral_limit").value),
            tau_limit=tau_limit,
            control_hz=control_hz,
            gravity_at_measured=bool(
                node.get_parameter("gravity_at_measured").value),
            recompute_every=int(node.get_parameter("recompute_every").value),
            fd_step=float(
                node.get_parameter("linearization_step").value),
            require_discrete_stable=bool(
                node.get_parameter("require_discrete_stable").value),
            tau_rate_limit=vector_parameter(
                node, "max_tau_rate_nm_s", n),
            use_discrete_lqr=bool(
                node.get_parameter("use_discrete_lqr").value),
            gain_schedule_hz=float(
                node.get_parameter("gain_schedule_hz").value),
            gain_schedule_mode=str(
                node.get_parameter("gain_schedule_mode").value),
            base_ff_alpha=node.get_parameter("base_ff_alpha").value,
            feedback_model_mode=node.get_parameter("feedback_model_mode").value,
            isolated_joint_index=isolated,
            independent_design_model=str(node.get_parameter('independent_design_model').value),
            velocity_filter=velocity_filter,
        )
