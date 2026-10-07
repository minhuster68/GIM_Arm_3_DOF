"""Exercise every hardware controller/shape with ideal state feedback, no CAN."""

from pathlib import Path
import time
from unittest.mock import patch

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from gim_arm_controller_lqr.factory import LqrFactory
from gim_arm_controller_mpc.factory import MpcFactory
from gim_arm_controller_smc.factory import SmcFactory
from gim_control.effort_controller_node import EffortControllerNode
from gim_control.plot_ee_error import load_log, print_metrics


@pytest.mark.parametrize('algorithm,factory', [
    ('lqr', LqrFactory), ('mpc', MpcFactory), ('smc', SmcFactory)])
@pytest.mark.parametrize('shape', ['circle', 'r', 'a'])
def test_ready_track_return_hold_and_position_metrics(tmp_path, capsys, algorithm, factory, shape):
    source = Path(__file__).resolve().parents[2]
    params = source / f'gim_arm_controller_{algorithm}/config/{algorithm}_hardware_soft.yaml'
    log_path = tmp_path / 'hardware.csv'
    rclpy.init(args=[
        '--ros-args', '--params-file', str(params),
        '-p', f'trajectory_shape:={shape}', '-p', 'approach_time:=16.0',
        '-p', 'return_time:=16.0', '-p', "cache_file:=''", '-p', f'log_file:={log_path}'])
    node = None
    try:
        node = EffortControllerNode(factory())

        def state(q, qd):
            message = JointState()
            message.name = node.joint_names
            message.position = np.asarray(q, dtype=float).tolist()
            message.velocity = np.asarray(qd, dtype=float).tolist()
            node._on_state(message)

        def capture(torque):
            node.last_command = np.asarray(torque).copy()
            node.last_publish_wall_ns = time.time_ns()

        with patch.object(node, '_publish', side_effect=capture):
            for _ in range(3):
                state(np.zeros(3), np.zeros(3))
                node._tick()
                if node.phase == 'GRAVITY':
                    break
            assert node.phase == 'GRAVITY'
            assert node.controller_hold and not node.cascade_hold
            assert node.get_parameter('autostart').value is False
            node.set_parameters([Parameter('autostart', value=True)])
            phases = set()
            for _ in range(6100):
                if node.phase == 'GRAVITY':
                    q, qd = np.zeros(3), np.zeros(3)
                else:
                    q, qd, _ = node._reference_at_phase(node.phase_elapsed)
                state(q, qd)
                node._tick()
                phases.add(node.phase)
                assert node.phase != 'ABORT'
                assert np.isfinite(node.last_command).all()
                assert np.all(np.abs(node.last_command) <= node.tau_limit + 1e-8)
                if node.phase == 'HOLD':
                    break
            assert phases == {'APPROACH', 'TRACK', 'RETURN', 'HOLD'}
            if algorithm in ('lqr', 'mpc'):
                assert node.controller.reference_time == 59.0
            if algorithm == 'mpc':
                assert node.controller.last['solver_accepted']
            node.dump()
        log = load_log(log_path)
        assert log['phase'][-1] == 'HOLD'
        print_metrics(log)
        output = capsys.readouterr().out
        assert 'RMS e_q [deg]' in output
        assert 'e_qdot' not in output
        assert 'MAX |e_q|' in output
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
