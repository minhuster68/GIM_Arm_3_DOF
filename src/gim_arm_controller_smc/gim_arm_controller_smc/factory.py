from gim_control.controller_api import ControllerFactory, vector_parameter

from .controller import SmcController


class SmcFactory(ControllerFactory):
    algorithm_name = "smc"

    def declare_parameters(self, node):
        node.declare_parameter("lambda_gain", [15.0, 85.0, 155.0])
        node.declare_parameter("ks", [5.0, 137.0, 180.0])
        node.declare_parameter("kr", [30.0, 130.0, 90.0])
        node.declare_parameter("phi", [0.03, 0.05, 0.15])

    def build(self, node, dynamics, tau_limit, control_hz):
        n = dynamics.nq
        controller = SmcController(
            dynamics,
            lambda_gain=vector_parameter(
                node, "lambda_gain", n, positive=True),
            ks=vector_parameter(node, "ks", n, positive=True),
            kr=vector_parameter(node, "kr", n, nonnegative=True),
            phi=vector_parameter(node, "phi", n, positive=True),
            control_hz=control_hz,
            tau_limit=tau_limit,
        )
        radii = controller.sampled_spectral_radius()
        if (radii >= 1.0).any():
            node.get_logger().warn(
                "SMC không ổn định trong xấp xỉ ZOH computed-torque "
                f"ở {control_hz:g} Hz: rho={radii.round(3)}. "
                "Chỉ dùng profile này để đối chiếu, không activate "
                "effort controller trên robot.")
        return controller
