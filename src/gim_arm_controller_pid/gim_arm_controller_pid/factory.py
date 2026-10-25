from gim_control.controller_api import ControllerFactory, vector_parameter

from .controller import PositionPidController


class PositionPidFactory(ControllerFactory):
    # Keep the ROS node name so existing launch files and commands still work.
    algorithm_name = "cascade_pid"

    def declare_parameters(self, node):
        node.declare_parameter("position_kp", [16.92, 33.84, 16.45])
        node.declare_parameter("position_ki", [12.69, 16.92, 9.87])
        node.declare_parameter("position_kd", [0.94, 1.41, 1.175])
        node.declare_parameter("position_integral_limit", [0.3, 0.3, 0.3])
        node.declare_parameter("base_ff_alpha", 1.0)
        node.declare_parameter("velocity_filter_type", "none")
        node.declare_parameter("velocity_kalman_measurement_std", [0.03, 0.01, 0.04])
        node.declare_parameter("velocity_kalman_jerk_std", [180.0, 60.0, 240.0])
        node.declare_parameter("velocity_kalman_reset_gap_s", 0.05)

    def build(self, node, dynamics, tau_limit, control_hz):
        n = dynamics.nq
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
        return PositionPidController(
            dynamics,
            kp=vector_parameter(node, "position_kp", n, nonnegative=True),
            ki=vector_parameter(node, "position_ki", n, nonnegative=True),
            kd=vector_parameter(node, "position_kd", n, nonnegative=True),
            integral_limit=vector_parameter(
                node, "position_integral_limit", n, positive=True),
            tau_limit=tau_limit,
            base_ff_alpha=node.get_parameter("base_ff_alpha").value,
            velocity_filter=velocity_filter,
        )


# Compatibility for existing callers; this factory builds only position PID.
CascadePidFactory = PositionPidFactory
