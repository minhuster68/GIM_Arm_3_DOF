"""ROS runner adapter for LESO. No hardware switch or motor commands here."""
import time
import xml.etree.ElementTree as ET

import numpy as np

from .controller_api import vector_parameter
from .extended_state_observer import EsoCompensator


ESO_VECTORS = ('eso_q_hat', 'eso_v_hat', 'eso_d_hat', 'tau_dist_hat',
               'tau_eso', 'tau_pre_eso', 'tau_with_eso_raw', 'tau_eso_actuator_adjustment')


def declare_eso_parameters(node):
    for key, value in dict(
            eso_enabled=False, eso_bandwidth_rad_s=[15., 15., 15.],
            eso_compensation_gain=[1., 1., 1.],
            eso_compensation_limit_nm=[.5, 2., .3],
            eso_compensation_rate_nm_s=[.5, 2., .3],
            eso_warmup_s=1.5, eso_ramp_s=2., eso_reset_gap_s=.05).items():
        node.declare_parameter(key, value)


def hardware_all_torque(description, names):
    root = ET.fromstring(description)
    joints = {j.get('name'): j for j in root.findall('.//ros2_control/joint')}
    valid = all(name in joints and (joints[name].findtext(
        "param[@name='torque_mode_enable']") or '').strip().lower() == 'true'
        for name in names)
    text = root.findtext(".//ros2_control/hardware/param[@name='torque_handoff_duration_s']")
    handoff = 1. if text is None else float(text)
    if not np.isfinite(handoff) or not 0 < handoff <= 5:
        raise ValueError('torque_handoff_duration_s không hợp lệ')
    return valid, handoff


class EsoModeGate:
    """Asynchronously confirm effort active, position inactive, all drivers torque.

    Published torque is not applied while JTC owns the arm. Never identify
    disturbance from commands ignored in position mode or during torque handoff.
    """
    def __init__(self, node):
        from controller_manager_msgs.srv import ListControllers
        from rcl_interfaces.srv import GetParameters
        self.node = node
        self.list_type, self.params_type = ListControllers, GetParameters
        self.list_client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.params_client = node.create_client(GetParameters, '/controller_manager/get_parameters')
        self.mode_future = self.params_future = None
        self.mode_stamp = self.description_stamp = 0.
        self.effort_active = self.all_torque = False
        self.handoff_s = 1.
        self.timer = node.create_timer(.1, self.poll)

    @property
    def allowed(self):
        now = time.monotonic()
        return (self.effort_active and self.all_torque
                and now-self.mode_stamp < .5 and now-self.description_stamp < 2.)

    def poll(self):
        if not self.list_client.service_is_ready():
            self.effort_active = False
        elif self.mode_future is None or self.mode_future.done():
            self.mode_future = self.list_client.call_async(self.list_type.Request())
            self.mode_future.add_done_callback(self._mode_done)
        if not self.params_client.service_is_ready():
            self.all_torque = False
        elif ((self.params_future is None or self.params_future.done())
              and time.monotonic()-self.description_stamp > .5):
            req = self.params_type.Request(names=['robot_description'])
            self.params_future = self.params_client.call_async(req)
            self.params_future.add_done_callback(self._description_done)

    def _mode_done(self, future):
        try:
            states = {s.name: s.state for s in future.result().controller}
            self.effort_active = (states.get('gim_arm_effort_controller') == 'active'
                                 and states.get('gim_arm_group_controller') == 'inactive')
            self.mode_stamp = time.monotonic()
        except Exception:
            self.effort_active = False

    def _description_done(self, future):
        try:
            result = future.result().values
            self.all_torque, self.handoff_s = hardware_all_torque(
                result[0].string_value, self.node.joint_names)
            self.description_stamp = time.monotonic()
        except Exception:
            self.all_torque = False


class EsoRuntime:
    def __init__(self, node):
        self.node = node
        n = node.n
        v = lambda name: vector_parameter(node, name, n, positive=True)
        self.compensator = EsoCompensator(
            node.dynamics, node.tau_limit, v('eso_bandwidth_rad_s'),
            vector_parameter(node, 'eso_compensation_gain', n, nonnegative=True),
            v('eso_compensation_limit_nm'), v('eso_compensation_rate_nm_s'),
            node.get_parameter('eso_warmup_s').value,
            node.get_parameter('eso_ramp_s').value,
            node.get_parameter('eso_reset_gap_s').value)
        self.configured_warmup = self.compensator.warmup_s
        self.gate = EsoModeGate(node)
        self.blocked = np.zeros(n, dtype=bool)
        node.get_logger().info(
            f'ESO MATLAB: bandwidth={self.compensator.observer.bandwidth} rad/s; '
            f'tau_cmd=limits(tau_base+tau_eso), tau_eso=-M(q)*d_hat; '
            f'|tau_eso|<={self.compensator.comp_limit} Nm. '
            'Chờ effort active, torque_joint=all và hết handoff; Kalman/FF giữ nguyên.')

    def compute(self, q_ref, qd_ref, qdd_ref, dt):
        node, eso = self.node, self.compensator
        allowed = (self.gate.allowed and not node.hand_guiding
                   and node.phase in ('GRAVITY', 'APPROACH', 'TRACK', 'RETURN', 'HOLD'))
        eso.warmup_s = max(self.configured_warmup, self.gate.handoff_s + .2)
        stamp = node.state_header_ns if node.state_header_ns > 0 else node.state_rx_wall_ns
        eso.prepare(node.q, getattr(node, 'qd_raw', node.qd), stamp, node.state_seq, allowed)
        saved = {key: getattr(node.controller, key).copy() for key in ('integral', 'z')
                 if isinstance(getattr(node.controller, key, None), np.ndarray)}
        baseline = node.controller.compute(node.q, node.qd, q_ref, qd_ref, qdd_ref, dt)
        rate = getattr(node.controller, 'tau_rate_limit', None)
        if rate is None:
            rate = getattr(node.controller, 'slew', None)
        rate = np.zeros(node.n) if rate is None else np.asarray(rate)
        applied = eso.apply(baseline, node.last_command, rate, dt)
        self.blocked = np.abs(applied-(baseline+eso.compensation)) > 1e-10
        for key, value in saved.items():
            current = getattr(node.controller, key)
            setattr(node.controller, key, np.where(self.blocked, value, current))
        # Align internal slew/fallback history with the command after the outer
        # compensation, excluding ESO when interpreting controller feedback.
        if hasattr(node.controller, 'last_tau'):
            node.controller.last_tau = applied.copy()
        if hasattr(node.controller, 'previous_feedback'):
            ff = node.controller.last.get('tau_ff', np.zeros(node.n))
            node.controller.previous_feedback = applied-eso.compensation-ff
        return applied

    def note_applied(self, command):
        node = self.node
        if (not self.gate.allowed or node.hand_guiding or node.phase in ('WAIT', 'ABORT')):
            self.compensator.reset()
            return
        if node.q is not None:
            self.compensator.record_applied(node.q, command, node.state_seq)

    def log_values(self):
        eso = self.compensator
        missing = np.full(self.node.n, np.nan)
        values = dict(eso_q_hat=eso.observer.q_hat, eso_v_hat=eso.observer.v_hat,
                      eso_d_hat=eso.observer.d_hat, **eso.last)
        vectors = [missing if values.get(k) is None else values[k] for k in ESO_VECTORS]
        return [eso.status, *(v for vector in vectors for v in vector)]
