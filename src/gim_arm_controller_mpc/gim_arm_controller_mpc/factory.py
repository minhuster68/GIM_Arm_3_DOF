from gim_control.controller_api import ControllerFactory, vector_parameter

from .controller import MpcController


class MpcFactory(ControllerFactory):
    algorithm_name = "mpc"

    def declare_parameters(self, node):
        node.declare_parameter("prediction_horizon", 40)
        node.declare_parameter("control_horizon", 10)
        node.declare_parameter("q_integral", [2.0, 2.0, 2.0])
        node.declare_parameter("q_position", [0.6271, 1.5677, 0.6271])
        node.declare_parameter("q_velocity", [0.02, 0.02, 0.02])
        node.declare_parameter("r_input", [0.0097, 0.0097, 0.0097])
        node.declare_parameter("r_rate", [0.03, 0.03, 0.03])
        node.declare_parameter("output_scale_integral", [0.25, 0.25, 0.25])
        node.declare_parameter("output_scale_position", [0.05, 0.05, 0.05])
        node.declare_parameter("output_scale_velocity", [0.10, 0.10, 0.10])
        node.declare_parameter("input_scale", [5.0, 40.0, 5.0])
        node.declare_parameter("torque_slew_rate", [50.0, 200.0, 50.0])
        node.declare_parameter("integral_limit", 0.0)
        node.declare_parameter("linearization_step", 1.0e-5)
        node.declare_parameter("max_solver_iterations", 60)
        node.declare_parameter("qp_solver", "slsqp")
        node.declare_parameter("admm_rho", 10.0)
        node.declare_parameter("admm_iterations", 100)
        node.declare_parameter("admm_tolerance", 0.0001)

    def build(self, node, dynamics, tau_limit, control_hz):
        n = dynamics.nq
        # Chuẩn hoá tương tự ScaleFactor của MPC Toolbox để các trọng số giữ
        # cùng ý nghĩa khi chuyển sang bài toán QP viết tay.
        integral_scale = vector_parameter(
            node, "output_scale_integral", n, positive=True)
        position_scale = vector_parameter(
            node, "output_scale_position", n, positive=True)
        velocity_scale = vector_parameter(
            node, "output_scale_velocity", n, positive=True)
        qi = vector_parameter(
            node, "q_integral", n, positive=True) / integral_scale**2
        qp = vector_parameter(
            node, "q_position", n, positive=True) / position_scale**2
        qv = vector_parameter(
            node, "q_velocity", n, positive=True) / velocity_scale**2
        # MATLAB dùng MV.ScaleFactor = tauMax = [5, 40, 5], không dùng trần
        # bring-up sau khi nhân tau_scale. tau_limit vẫn là constraint an toàn.
        input_scale = vector_parameter(
            node, "input_scale", n, positive=True)
        ri = vector_parameter(
            node, "r_input", n, positive=True) / input_scale**2
        rr = vector_parameter(
            node, "r_rate", n, positive=True) / input_scale**2
        return MpcController(
            dynamics,
            control_hz=control_hz,
            prediction_horizon=int(
                node.get_parameter("prediction_horizon").value),
            control_horizon=int(node.get_parameter("control_horizon").value),
            q_integral=qi,
            q_position=qp,
            q_velocity=qv,
            r_input=ri,
            r_rate=rr,
            torque_slew_rate=vector_parameter(
                node, "torque_slew_rate", n, positive=True),
            integral_limit=float(node.get_parameter("integral_limit").value),
            tau_limit=tau_limit,
            fd_step=float(node.get_parameter("linearization_step").value),
            max_iterations=int(
                node.get_parameter("max_solver_iterations").value),
            qp_solver=str(node.get_parameter("qp_solver").value),
            admm_rho=float(node.get_parameter("admm_rho").value),
            admm_iterations=int(
                node.get_parameter("admm_iterations").value),
            admm_tolerance=float(
                node.get_parameter("admm_tolerance").value),
        )
