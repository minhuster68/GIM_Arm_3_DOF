from gim_control.controller_api import ControllerFactory, vector_parameter

from .controller import CascadePidController, PositionPidController


class CascadePidFactory(ControllerFactory):
    algorithm_name = "cascade_pid"

    def declare_parameters(self, node):
        node.declare_parameter("pid_mode", "cascade")
        node.declare_parameter("position_kp", [16.92, 33.84, 16.45])
        node.declare_parameter("position_ki", [12.69, 16.92, 9.87])
        node.declare_parameter("position_kd", [0.94, 1.41, 1.175])
        node.declare_parameter("position_integral_limit", [0.3, 0.3, 0.3])
        node.declare_parameter("kpp", [46.0, 60.0, 36.0])
        node.declare_parameter("kvp", [8.0, 9.5, 10.0])
        node.declare_parameter("kvi", [7.6, 7.6, 7.6])
        node.declare_parameter("torque_constant", 0.47)
        node.declare_parameter("velocity_integral_limit", 0.0)

    def build(self, node, dynamics, tau_limit, control_hz):
        n = dynamics.nq
        mode = str(node.get_parameter("pid_mode").value)
        if mode == "position":
            return PositionPidController(
                dynamics,
                kp=vector_parameter(node, "position_kp", n, positive=True),
                ki=vector_parameter(node, "position_ki", n, nonnegative=True),
                kd=vector_parameter(node, "position_kd", n, nonnegative=True),
                integral_limit=vector_parameter(
                    node, "position_integral_limit", n, positive=True),
                tau_limit=tau_limit,
            )
        if mode != "cascade":
            raise ValueError("pid_mode phải là position hoặc cascade")
        torque_constant = float(
            node.get_parameter("torque_constant").value)
        integral_limit = float(
            node.get_parameter("velocity_integral_limit").value)
        if torque_constant <= 0.0:
            raise ValueError("torque_constant phải > 0")
        if integral_limit < 0.0:
            raise ValueError(
                "velocity_integral_limit phải >= 0 (0 = tắt kẹp)")
        return CascadePidController(
            dynamics,
            kpp=vector_parameter(node, "kpp", n, positive=True),
            kvp=vector_parameter(node, "kvp", n, positive=True),
            kvi=vector_parameter(node, "kvi", n, positive=True),
            torque_constant=torque_constant,
            integral_limit=integral_limit,
            tau_limit=tau_limit,
        )
