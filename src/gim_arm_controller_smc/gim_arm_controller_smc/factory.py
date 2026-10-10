from gim_control.controller_api import ControllerFactory, vector_parameter

from .controller import SmcController


class SmcFactory(ControllerFactory):
    algorithm_name = "smc"

    def declare_parameters(self, node):
        node.declare_parameter("lambda_gain", [15.0, 85.0, 155.0])
        node.declare_parameter("ks", [5.0, 137.0, 180.0])
        node.declare_parameter("kr", [30.0, 130.0, 90.0])
        node.declare_parameter("phi", [0.03, 0.05, 0.15])
        node.declare_parameter("base_ff_alpha", 1.0)
        node.declare_parameter("controlled_joint", "all")
        # Zero disables slew in existing simulation/full-arm profiles.
        node.declare_parameter("torque_slew_rate", [0.0, 0.0, 0.0])

    def build(self, node, dynamics, tau_limit, control_hz):
        n = dynamics.nq
        slew = vector_parameter(node, "torque_slew_rate", n, nonnegative=True)
        if (slew == 0).any() and not (slew == 0).all():
            raise ValueError('torque_slew_rate phải toàn 0 (tắt) hoặc toàn > 0')
        controller = SmcController(
            dynamics,
            lambda_gain=vector_parameter(
                node, "lambda_gain", n, positive=True),
            ks=vector_parameter(node, "ks", n, positive=True),
            kr=vector_parameter(node, "kr", n, nonnegative=True),
            phi=vector_parameter(node, "phi", n, positive=True),
            control_hz=control_hz,
            tau_limit=tau_limit,
            base_ff_alpha=float(node.get_parameter("base_ff_alpha").value),
            controlled_joint=str(node.get_parameter("controlled_joint").value),
            torque_slew_rate=None if (slew == 0).all() else slew,
        )
        radii = controller.sampled_spectral_radius()
        if (radii[controller.active] >= 1.0).any():
            raise ValueError(
                "SMC không ổn định trong xấp xỉ ZOH computed-torque "
                f"ở {control_hz:g} Hz: rho={radii.round(3)}. "
                "Giảm lambda/ks/kr hoặc tăng phi trước khi chạy.")
        return controller
