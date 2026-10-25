"""Shared ESO runner integration for the selected branch, no CAN/hardware."""
from importlib import import_module
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import csv
import time

import matplotlib
matplotlib.use('Agg')
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
import yaml

from gim_control.effort_controller_node import EffortControllerNode
from gim_control.eso_runtime import ESO_VECTORS


ROOT = Path(__file__).resolve().parents[3]


def test_eso_profile_preserves_its_baseline_and_logs_actual_command_decomposition(tmp_path):
    profiles = list((ROOT/'src').glob('gim_arm_controller_*/config/*_eso_hardware.yaml'))
    assert len(profiles) == 1
    profile = profiles[0]
    algorithm = profile.name.split('_')[0]
    defaults = dict(pid='pid_hardware_soft.yaml', lqr='lqr_circle_hardware.yaml',
                    mpc='mpc_circle_independent_feedback.yaml', smc='smc_circle_from_terminal_tuning.yaml')
    params = next(iter(yaml.safe_load(profile.read_text()).values()))['ros__parameters']
    original = next(iter(yaml.safe_load(profile.with_name(defaults[algorithm]).read_text()).values()))['ros__parameters']
    assert all(params[k] == v for k, v in original.items())
    factory_name = dict(pid='CascadePidFactory', lqr='LqrFactory', mpc='MpcFactory', smc='SmcFactory')[algorithm]
    factory = getattr(import_module(f'gim_arm_controller_{algorithm}.factory'), factory_name)()
    params = dict(params, urdf_file=str(ROOT/'src/gim_arm_description/urdf/gim_arm.urdf'),
                  cache_file='', log_file=str(tmp_path/'eso.csv'))
    rclpy.init(args=[])
    node = None
    gate = SimpleNamespace(allowed=True, handoff_s=1.)
    try:
        with patch('gim_control.eso_runtime.EsoModeGate', return_value=gate):
            node = EffortControllerNode(factory, parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        node.phase = 'HOLD'
        node.home_q = np.zeros(3)
        zero = np.zeros(3)
        controller = node.controller
        if hasattr(controller, 'prepare_reference'):
            controller.prepare_reference(np.zeros((2, 3)), np.zeros((2, 3)),
                                         np.zeros((2, 3)), [0., .01])
        if hasattr(controller, 'precompute_gain_schedule'):
            controller.precompute_gain_schedule(np.zeros((2, 3)), np.zeros((2, 3)),
                                                reference_times=[0., .01])
        with patch.object(node.publisher, 'publish'):
            for k in range(260):
                msg = JointState()
                msg.name = node.joint_names
                # Small externally imposed motion: observer should detect a
                # residual even when calibrated FF is an exact static hold.
                msg.position = [.0005*np.sin(k*.02), .0002*np.cos(k*.02), .0003*np.sin(k*.02)]
                msg.velocity = [.001*np.cos(k*.02), -.0004*np.sin(k*.02), .0006*np.cos(k*.02)]
                msg.header.stamp.sec = 1+k//100
                msg.header.stamp.nanosec = (k % 100)*10000000
                node._on_state(msg)
                tau = node._compute_control(zero, zero, zero, .01)
                node._publish(tau)
                node._append_tracking_row(time.time_ns(), k*.01, .01, 0., zero, zero, tau)
                assert np.all(np.isfinite(tau)) and np.all(np.abs(tau) <= node.tau_limit+1e-10)
        assert node.eso.compensator.status == 'ACTIVE'
        assert np.any(np.abs(node.eso.compensator.compensation) > 1e-5)
        assert np.all(np.abs(node.eso.compensator.compensation) <= node.eso.compensator.comp_limit)
        node.dump()
        with open(params['log_file']) as stream:
            rows = list(csv.DictReader(stream))
        for row in rows:
            assert None not in row and all(v is not None for v in row.values())
            for joint in node.joint_names:
                actual = float(row['tau_'+joint])
                expected = sum(float(row[p+'_'+joint]) for p in
                               ('tau_pre_eso', 'tau_eso', 'tau_eso_actuator_adjustment'))
                np.testing.assert_allclose(actual, expected, atol=2e-6)
                assert all(p+'_'+joint in row for p in ESO_VECTORS)
        from gim_control.plot_eso import plot_eso
        from gim_control.plot_ee_error import load_log
        parsed = load_log(params['log_file'])
        assert 'tau_eso' in parsed and len(parsed['tau_eso']) == len(rows)
        outputs = plot_eso(params['log_file'])
        assert len(outputs) == 3 and all(p.stat().st_size > 0 for p in outputs)
        gate.allowed = False
        node._compute_control(zero, zero, zero, .01)
        assert node.eso.compensator.status == 'WAIT_EFFORT_ALL'
        np.testing.assert_array_equal(node.eso.compensator.compensation, zero)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
