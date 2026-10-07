#!/usr/bin/env python3
"""Offline hardware-profile checks; no CAN socket or controller-manager call."""

import argparse
import json
import os
from pathlib import Path

import numpy as np
import rclpy

from gim_arm_controller_lqr.factory import LqrFactory
from gim_arm_controller_mpc.factory import MpcFactory
from gim_arm_controller_smc.factory import SmcFactory
from gim_control.effort_controller_node import EffortControllerNode
from gim_control.reference_trajectory import Quintic


ROOT = Path(__file__).resolve().parents[1]
FACTORIES = {'lqr': LqrFactory, 'mpc': MpcFactory, 'smc': SmcFactory}


def check_case(algorithm, shape):
    params = ROOT / f'src/gim_arm_controller_{algorithm}/config/{algorithm}_hardware_soft.yaml'
    rclpy.init(args=[
        '--ros-args', '--params-file', str(params),
        '-p', f'urdf_file:={ROOT / "src/gim_arm_description/urdf/gim_arm.urdf"}',
        '-p', f'trajectory_shape:={shape}', '-p', 'approach_time:=16.0',
        '-p', 'return_time:=16.0', '-p', "cache_file:=''", '-p', "log_file:=''"])
    node = None
    try:
        node = EffortControllerNode(FACTORIES[algorithm]())
        node.home_q = np.zeros(3)
        node.q, node.qd = node.home_q.copy(), np.zeros(3)
        node._prepare_controller_schedule(node.home_q)
        trajectory = node.trajectory
        endpoint = trajectory.at(0.0)[0]
        np.testing.assert_allclose(trajectory.at(trajectory.duration)[0], endpoint, atol=1e-12)
        references = (
            ('APPROACH', Quintic(node.home_q, endpoint, node.approach_time)),
            ('TRACK', trajectory),
            ('RETURN', Quintic(endpoint, node.home_q, node.return_time)),
        )
        node.phase = 'GRAVITY'
        prep = node._hold_torque(node.dt_nom, integrate=False)
        assert np.isfinite(prep).all()
        node.controller.reset()
        peak, ff_peak = np.zeros(3), np.zeros(3)
        solver_accepted = samples = 0
        for phase, reference in references:
            node.phase = phase
            for elapsed in np.linspace(0.0, reference.duration, 101):
                q, qd, qdd = reference.at(elapsed)
                set_time = getattr(node.controller, 'set_reference_time', None)
                if callable(set_time):
                    set_time(node._gain_schedule_time(elapsed))
                # Exercise actual model/feedback and solver with a small position disturbance.
                measured = q + np.array([0.001, -0.001, 0.001])
                torque = node.controller.compute(measured, qd, q, qd, qdd, node.dt_nom)
                if not np.isfinite(torque).all() or np.any(np.abs(torque) > node.tau_limit + 1e-8):
                    raise RuntimeError(f'{algorithm}/{shape}: invalid torque')
                if algorithm == 'mpc':
                    if not node.controller.last['solver_accepted']:
                        raise RuntimeError(f'MPC/{shape}: infeasible solver output')
                    solver_accepted += 1
                peak = np.maximum(peak, np.abs(torque))
                ff_peak = np.maximum(ff_peak, np.abs(node.dynamics.inverse_dynamics(q, qd, qdd)))
                samples += 1
        node.q, node.qd = node.home_q.copy(), np.zeros(3)
        node.phase = 'HOLD'
        hold = node._hold_torque(node.dt_nom, integrate=True)
        assert np.isfinite(hold).all()
        result = {
            'algorithm': algorithm, 'trajectory_shape': shape, 'passed': True,
            'check': 'offline model/schedule/solver; hardware not connected',
            'samples': samples, 'control_hz': 1.0 / node.dt_nom,
            'tau_limit_nm': node.tau_limit.tolist(), 'peak_tau_nm': peak.tolist(),
            'peak_feedforward_nm': ff_peak.tolist(),
            'track_duration_s': trajectory.duration,
            'mpc_accepted_samples': solver_accepted if algorithm == 'mpc' else None,
        }
        print(json.dumps(result), flush=True)
        return result
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--algorithm', choices=tuple(FACTORIES), default='lqr')
    parser.add_argument('--trajectory-shape', choices=('circle', 'r', 'a'), default='circle')
    parser.add_argument('--all', action='store_true')
    parser.add_argument('--ros-domain-id', type=int, default=82)
    parser.add_argument('--output', type=Path, default=ROOT / 'results/hardware_preflight.json')
    args = parser.parse_args()
    if not 0 <= args.ros_domain_id <= 232:
        parser.error('ROS domain must be between 0 and 232')
    os.environ.update(ROS_DOMAIN_ID=str(args.ros_domain_id), ROS_LOCALHOST_ONLY='1',
                      OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    cases = ([(algorithm, shape) for algorithm in FACTORIES for shape in ('circle', 'r', 'a')]
             if args.all else [(args.algorithm, args.trajectory_shape)])
    results = [check_case(algorithm, shape) for algorithm, shape in cases]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(results, indent=2) + '\n')
    print(f'PASS: {len(results)} offline checks. CAN/hardware validation remains to be run.')


if __name__ == '__main__':
    main()
