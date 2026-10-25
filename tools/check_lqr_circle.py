#!/usr/bin/env python3
"""Offline circle checks using the production controller; never opens ROS/CAN."""

import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace

os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'

import numpy as np
import yaml

from gim_arm_controller_lqr.factory import LqrFactory
from gim_control.arm_dynamics import ArmDynamics
from gim_control.reference_trajectory import load_trajectory, Quintic


ROOT = Path(__file__).resolve().parents[1]


def build_controller(parameters, dynamics):
    values = deepcopy(parameters)
    if not values.get('require_discrete_stable'):
        raise ValueError('Validation requires require_discrete_stable=true')
    if values.get('isolated_joint_index', -1) != -1:
        raise ValueError('Circle must validate the full plant')
    values['diagnostic_hold'] = False
    node = SimpleNamespace(
        declare_parameter=lambda n, v: SimpleNamespace(value=values.setdefault(n, v)),
        get_parameter=lambda n: SimpleNamespace(value=values[n]))
    factory = LqrFactory()
    factory.declare_parameters(node)
    return factory.build(node, dynamics, dynamics.tau_max * values['tau_scale'],
                         values['control_hz'])


def references(urdf, approach, returning):
    trajectory = load_trajectory(str(urdf), '', SimpleNamespace(info=lambda *_: None),
                                 shape='circle')
    zero = np.zeros(3)
    return [(Quintic(zero, trajectory.at(0.)[0], approach), approach),
            (trajectory, trajectory.duration),
            (Quintic(trajectory.at(trajectory.duration)[0], zero, returning), returning)]


def reference_at(segments, elapsed):
    for reference, duration in segments:
        if elapsed <= duration:
            return reference.at(elapsed)
        elapsed -= duration
    return np.zeros(3), np.zeros(3), np.zeros(3)


def prepare(controller, segments):
    positions, velocities, times = [], [], []
    offset = 0.
    for reference, duration in segments:
        grid = np.linspace(0., duration, max(2, int(np.ceil(
            duration * controller.gain_schedule_hz)) + 1))
        for elapsed in grid[1 if times else 0:]:
            q, qd, _ = reference.at(elapsed)
            positions.append(q)
            velocities.append(qd)
            times.append(offset + elapsed)
        offset += duration
    return controller.precompute_gain_schedule(positions, velocities, times)


def delayed_radius(controller, q, qd, K):
    delayed = controller._closed_loop_transition(q, qd, K, command_delay=True)
    return float(np.max(np.abs(np.linalg.eigvals(delayed))))


def nonlinear_rollout(controller, dynamics, segments, substeps, delayed):
    """Full nominal URDF plant with estimated friction, 100-Hz held torque.

    The empirical base_ff_alpha is applied by the actual controller. The URDF
    plant has unscaled gravity: any resulting bias is reported, not cancelled.
    Model the configured one-second measured-hold torque blend and one second
    of settling before motion. This is not an identified hardware plant.
    """
    controller.reset()
    velocity_filter = controller.velocity_filter
    if velocity_filter is not None:
        velocity_filter.reset()
    q, qd = np.zeros(3), np.zeros(3)
    previous_torque = dynamics.gravity(q)
    initial_hold_torque = previous_torque.copy()
    handoff_s, settle_s = 1., 1.
    errors, torques, positions = [], [], []
    saturated = 0
    failure = None
    dt = 1. / controller.control_hz
    duration = handoff_s + settle_s + sum(d for _, d in segments) + 3.
    for step in range(int(round(duration / dt)) + 1):
        wall_elapsed = step * dt
        elapsed = max(0., wall_elapsed - handoff_s - settle_s)
        qr, vr, ar = reference_at(segments, elapsed)
        controller.set_reference_time(elapsed)
        measured_velocity = (qd if velocity_filter is None else
                             velocity_filter.update(qd, wall_elapsed))
        command = controller.compute(q, measured_velocity, qr, vr, ar, dt)
        applied = previous_torque if delayed else command
        previous_torque = command.copy()
        blend = min(1., wall_elapsed / handoff_s)
        applied = (1. - blend) * initial_hold_torque + blend * applied
        errors.append(q - qr)
        torques.append(command)
        positions.append(q.copy())
        saturated += int(np.any(controller.last['saturated']))
        for _ in range(substeps):
            h = dt / substeps
            acceleration = dynamics.forward_dynamics(q, qd, applied)
            qd += h * acceleration
            q += h * qd
        if (not np.all(np.isfinite(q)) or not np.all(np.isfinite(qd))
                or np.any(q < dynamics.q_min) or np.any(q > dynamics.q_max)):
            failure = dict(time_s=elapsed, q_rad=q.tolist())
            break
    errors = np.rad2deg(np.asarray(errors))
    positions = np.asarray(positions)
    return dict(
        completed=failure is None, samples=len(errors), failure=failure,
        modeled_handoff_s=handoff_s, modeled_settling_s=settle_s,
        integration_step_s=dt / substeps, command_delay_ticks=int(delayed),
        rms_error_deg=np.sqrt(np.mean(errors**2, axis=0)).tolist(),
        max_error_deg=np.max(np.abs(errors), axis=0).tolist(),
        peak_torque_nm=np.max(np.abs(torques), axis=0).tolist(),
        min_joint_margin_rad=float(min((positions - dynamics.q_min).min(),
                                       (dynamics.q_max - positions).min())),
        rate_or_torque_limited_fraction=saturated / len(errors))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params-file', type=Path, default=ROOT /
                        'src/gim_arm_controller_lqr/config/lqr_circle_hardware.yaml')
    parser.add_argument('--urdf', type=Path, default=ROOT /
                        'src/gim_arm_description/urdf/gim_arm.urdf')
    parser.add_argument('--output', type=Path, default=ROOT /
                        'results_new/lqr/circle_readiness/verification.json')
    parser.add_argument('--approach-time', type=float, default=16.)
    parser.add_argument('--return-time', type=float, default=16.)
    args = parser.parse_args()
    params = yaml.safe_load(args.params_file.read_text())['lqr_controller']['ros__parameters']
    dynamics = ArmDynamics(str(args.urdf), use_armature=params['use_armature'])
    controller = build_controller(params, dynamics)
    segments = references(args.urdf, args.approach_time, args.return_time)
    count = prepare(controller, segments)
    print(f'Production constructor and {count}-sample gain schedule passed.', flush=True)
    report = controller.stability_report()
    rng = np.random.default_rng(0)
    poses = np.vstack([(dynamics.q_min + dynamics.q_max) / 2,
                       dynamics.q_min, dynamics.q_max,
                       rng.uniform(dynamics.q_min, dynamics.q_max, size=(21, 3))])
    delayed_static = max(delayed_radius(controller, q, np.zeros(3),
                                        controller.gain(q, np.zeros(3))) for q in poses)
    nominal_worst = delayed_worst = 0.
    duration = sum(d for _, d in segments)
    grid = np.linspace(0., duration, int(round(duration * controller.control_hz)) + 1)
    for elapsed in grid:
        q, qd, _ = reference_at(segments, elapsed)
        controller.set_reference_time(elapsed)
        K = controller._interpolated_gain()
        nominal_worst = max(nominal_worst, controller._plant_spectral_radius(q, qd, K))
        delayed_worst = max(delayed_worst, delayed_radius(controller, q, qd, K))
    print(f'{len(grid)} interpolated gains: rho={nominal_worst:.9f}; '
          f'one-tick delay rho={delayed_worst:.9f}', flush=True)
    rollouts = [nonlinear_rollout(controller, dynamics, segments, steps, delayed)
                for steps, delayed in ((10, False), (20, False), (20, True))]
    checks = dict(
        static_full_plant=report['stable_discrete'],
        interpolated_full_plant=nominal_worst < 1.,
        static_one_tick_delay=delayed_static < 1.,
        interpolated_one_tick_delay=delayed_worst < 1.,
        nonlinear_nominal_urdf_within_limits=all(r['completed'] for r in rollouts))
    result = dict(
        scope='Offline model checks only; no physical-arm validation, no ROS/CAN access.',
        limitations=[
            'Frozen local spectral radii do not prove nonlinear or time-varying stability.',
            'URDF friction is an uncalibrated estimate; only the one-second torque blend '
            'is modeled, not internal driver dynamics or sensor noise.',
            'Empirical base feedforward alpha is not a calibrated plant gravity model; '
            'the nominal URDF rollout retains and reports this feedforward mismatch.',
            'Passing model checks does not establish physical tracking performance.'],
        params_file=str(args.params_file.resolve()),
        params_sha256=hashlib.sha256(args.params_file.read_bytes()).hexdigest(),
        urdf_file=str(args.urdf.resolve()),
        urdf_sha256=hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
        costs=dict(QI=np.diag(controller.Q)[:3].tolist(), QE=np.diag(controller.Q)[3:6].tolist(),
                   QV=np.diag(controller.Q)[6:].tolist(), R=np.diag(controller.R).tolist()),
        actual_torque_limits_nm=controller.tau_limit.tolist(),
        static_model=report, gain_schedule_samples=count,
        interpolated_samples=len(grid), interpolated_rho=nominal_worst,
        static_delay_1_tick_rho=delayed_static, interpolated_delay_1_tick_rho=delayed_worst,
        nonlinear_scenarios=rollouts, checks=checks, passed=all(checks.values()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(dict(checks=checks, passed=result['passed'], output=str(args.output)), indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
