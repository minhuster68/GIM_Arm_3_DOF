from gim_control.controller_api import ControllerFactory, vector_parameter

from .controller import SmcController
from .super_twisting import SuperTwistingController


class SmcFactory(ControllerFactory):
    algorithm_name = "smc"

    def declare_parameters(self, node):
        node.declare_parameter("smc_variant", "saturation")
        node.declare_parameter("k1", [15.0, 30.0, 25.0])
        node.declare_parameter("k2", [30.0, 100.0, 80.0])
        node.declare_parameter("z_limit", [100.0, 500.0, 500.0])
        node.declare_parameter("lambda_gain", [15.0, 85.0, 155.0])
        node.declare_parameter("ks", [5.0, 137.0, 180.0])
        node.declare_parameter("kr", [30.0, 130.0, 90.0])
        node.declare_parameter("phi", [0.03, 0.05, 0.15])
        node.declare_parameter("base_ff_alpha", 1.0)
        node.declare_parameter("controlled_joint", "all")
        # Zero disables slew in existing simulation/full-arm profiles.
        node.declare_parameter("torque_slew_rate", [0.0, 0.0, 0.0])
        # Scale the off-diagonal mass-matrix contribution per output joint.
        node.declare_parameter("feedback_offdiag_scale", [1.0, 1.0, 1.0])
        # Zero bypasses the measured-velocity filter on that joint.
        node.declare_parameter("velocity_lpf_hz", [0.0, 0.0, 0.0])
        node.declare_parameter("velocity_kalman_enabled", [False, False, False])
        node.declare_parameter("velocity_kalman_jerk_psd", [1.0, 1.0, 1.0])
        node.declare_parameter("velocity_kalman_measurement_variance", [0.0004, 0.0004, 0.0004])
        node.declare_parameter("friction_ff_nm", [0.0, 0.0, 0.0])
        node.declare_parameter("friction_transition_rad_s", 0.005)

    def build(self, node, dynamics, tau_limit, control_hz):
        n = dynamics.nq
        slew = vector_parameter(node, "torque_slew_rate", n, nonnegative=True)
        if (slew == 0).any() and not (slew == 0).all():
            raise ValueError('torque_slew_rate phải toàn 0 (tắt) hoặc toàn > 0')
        common = dict(
            lambda_gain=vector_parameter(
                node, "lambda_gain", n, positive=True),
            control_hz=control_hz,
            tau_limit=tau_limit,
            base_ff_alpha=float(node.get_parameter("base_ff_alpha").value),
            controlled_joint=str(node.get_parameter("controlled_joint").value),
            torque_slew_rate=None if (slew == 0).all() else slew,
            feedback_offdiag_scale=vector_parameter(
                node, "feedback_offdiag_scale", n, nonnegative=True),
        )
        variant = str(node.get_parameter("smc_variant").value)
        if variant == 'super_twisting':
            return SuperTwistingController(
                dynamics, k1=vector_parameter(node, 'k1', n, positive=True),
                k2=vector_parameter(node, 'k2', n, positive=True),
                z_limit=vector_parameter(node, 'z_limit', n, positive=True),
                velocity_lpf_hz=vector_parameter(
                    node, 'velocity_lpf_hz', n, nonnegative=True),
                velocity_kalman_enabled=node.get_parameter('velocity_kalman_enabled').value,
                velocity_kalman_jerk_psd=vector_parameter(
                    node, 'velocity_kalman_jerk_psd', n, positive=True),
                velocity_kalman_measurement_variance=vector_parameter(
                    node, 'velocity_kalman_measurement_variance', n, positive=True),
                friction_ff_nm=vector_parameter(
                    node, 'friction_ff_nm', n, nonnegative=True),
                friction_transition_rad_s=float(
                    node.get_parameter('friction_transition_rad_s').value), **common)
        if variant != 'saturation':
            raise ValueError('smc_variant phải là saturation hoặc super_twisting')
        controller = SmcController(
            dynamics, ks=vector_parameter(node, 'ks', n, positive=True),
            kr=vector_parameter(node, 'kr', n, nonnegative=True),
            phi=vector_parameter(node, 'phi', n, positive=True), **common)
        radii = controller.sampled_spectral_radius()
        if (radii[controller.active] >= 1.0).any():
            raise ValueError(
                "SMC không ổn định trong xấp xỉ ZOH computed-torque "
                f"ở {control_hz:g} Hz: rho={radii.round(3)}. "
                "Giảm lambda/ks/kr hoặc tăng phi trước khi chạy.")
        return controller
