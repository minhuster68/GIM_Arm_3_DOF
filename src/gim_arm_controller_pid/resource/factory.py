from gim_control.controller_api import ControllerFactory, vector_parameter

from ..gim_arm_controller_pid.controller import CascadePidController


class CascadePidFactory(ControllerFactory):
    algorithm_name = "cascade_pid"

    def declare_parameters(self, node):
        node.declare_parameter("kpp", [46.0, 60.0, 36.0])
        node.declare_parameter("kvp", [3.76, 4.465, 4.7])  # Nm/(rad/s)
        node.declare_parameter("kvi", [3.572, 3.572, 3.572])  # Nm/rad
        node.declare_parameter("velocity_integral_limit", 0.0)

    def build(self, node, dynamics, tau_limit, control_hz):
        n = dynamics.nq
        integral_limit = float(
            node.get_parameter("velocity_integral_limit").value)
        if integral_limit < 0.0:
            raise ValueError("velocity_integral_limit phải >= 0 (0 = tắt kẹp)")
        return CascadePidController(
            dynamics,
            kpp=vector_parameter(node, "kpp", n, positive=True),
            kvp=vector_parameter(node, "kvp", n, positive=True),
            kvi=vector_parameter(node, "kvi", n, positive=True),
            integral_limit=integral_limit,
            tau_limit=tau_limit,
        )
