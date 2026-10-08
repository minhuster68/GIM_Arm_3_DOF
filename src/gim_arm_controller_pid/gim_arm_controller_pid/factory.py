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

    def build(self, node, dynamics, tau_limit, control_hz):
        n = dynamics.nq
        return PositionPidController(
            dynamics,
            kp=vector_parameter(node, "position_kp", n, nonnegative=True),
            ki=vector_parameter(node, "position_ki", n, nonnegative=True),
            kd=vector_parameter(node, "position_kd", n, nonnegative=True),
            integral_limit=vector_parameter(
                node, "position_integral_limit", n, positive=True),
            tau_limit=tau_limit,
        )


# Compatibility for existing callers; this factory builds only position PID.
CascadePidFactory = PositionPidFactory
