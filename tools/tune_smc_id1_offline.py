"""Shoulder STA design study. Numerical simulation only; no ROS/CAN access.

Base and elbow are held at zero. The controller kernel includes all three
torques because their slew limits can block the selected joint's z update.
Friction, measurement noise and actuator delay below are assumed, not fitted.
"""

import argparse
from itertools import product
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/gim_arm_control'))
sys.path.insert(0, str(ROOT / 'src/gim_arm_controller_smc'))
from gim_control.arm_dynamics import ArmDynamics
from gim_arm_controller_smc.super_twisting import SuperTwistingController

DT = .01
KEYS = ('motion_rms_deg', 'hold_rms_deg', 'velocity_rms_deg_s', 'torque_step_rms_nm')
# Assumed transmission/friction and telemetry stress cases, not measurements.
TRAIN = [
    dict(name='nominal', inertia=1., g=1., fc=.256, fs=0., noise=.005, delay=0, bias=0.),
    dict(name='stiction', inertia=1., g=1., fc=.30, fs=.42, noise=.008, delay=1, bias=.04),
    dict(name='model_error', inertia=1.2, g=1.10, fc=.34, fs=.50, noise=.012, delay=1, bias=-.08),
]
STRESS = [
    dict(name='light', inertia=.85, g=.92, fc=.20, fs=.35, noise=.01, delay=1, bias=.06),
    dict(name='delayed', inertia=1.15, g=1.08, fc=.32, fs=.55, noise=.015, delay=2, bias=-.08),
    dict(name='high_stiction', inertia=1.25, g=1.15, fc=.40, fs=.65, noise=.01, delay=1, bias=.12),
]


def basis(q):
    q = np.asarray(q)
    return np.stack([np.ones_like(q), np.cos(q), np.sin(q),
                     np.cos(2*q), np.sin(2*q)], axis=-1)


class ShoulderStudy:
    def __init__(self, params, urdf):
        self.params = params
        self.model = ArmDynamics(str(urdf), use_armature=params['use_armature'])
        self.limits = self.model.tau_max * float(params['tau_scale'])
        self.z_limit = float(params['z_limit'][1])
        self.slew = np.asarray(params['torque_slew_rate'], dtype=float)
        self.alpha = float(params['base_ff_alpha'])
        self.mass = float(self.model.mass_matrix(np.zeros(3))[1, 1])
        # Exact Fourier representation for a single revolute axis; independently
        # checked against full URDF dynamics at random positions/velocities.
        angles = np.linspace(-np.pi, np.pi, 64, endpoint=False)
        samples = []
        for angle in angles:
            q = np.array([0., angle, 0.])
            samples.append(np.stack([self.model.gravity(q),
                                     self.model.mass_matrix(q)[:, 1],
                                     self.model.coriolis(q, [0., 1., 0.])]))
        self.coefficients = np.linalg.lstsq(
            basis(angles), np.array(samples).reshape(64, 9), rcond=None)[0]
        assert np.all(self.limits > 0) and self.z_limit > 0

    def terms(self, q):
        return (basis(q) @ self.coefficients).reshape((*np.shape(q), 3, 3))

    def gravity(self, q):
        # Selected-axis gravity coefficients; evaluate cheaply in the plant.
        return basis(q) @ self.coefficients[:, 1]

    def step(self, q, v, qr, vr, ar, z, previous, gains):
        lam, k1, k2, slew = [gains[:, i, None] for i in range(4)]
        surface = v-vr + lam*(q-qr)
        acc = -lam*(v-vr)-k1*np.sqrt(np.abs(surface))*np.sign(surface)+z
        terms = self.terms(q)
        g, col, coriolis = terms[..., 0, :], terms[..., 1, :], terms[..., 2, :]
        ff = g + col*ar + coriolis*v[..., None]**2
        ff[..., 0] *= self.alpha
        raw_fb = col*acc[..., None]
        rates = np.broadcast_to(self.slew, (*q.shape, 3)).copy()
        rates[..., 1] = slew
        fb = np.clip(raw_fb, previous-rates*DT, previous+rates*DT)
        raw = ff+fb
        tau = np.clip(raw, -self.limits, self.limits)
        increment = -k2*np.sign(surface)*DT
        blocked = np.any((raw_fb-fb+raw-tau)*col*increment[..., None] > 1e-12, axis=-1)
        zn = np.clip(z+np.where(blocked, 0., increment), -self.z_limit, self.z_limit)
        return tau, zn, tau-ff, raw

    def verify(self, gains):
        params = self.params
        lam, k1, k2 = [np.array(params[name], dtype=float) for name in ('lambda_gain', 'k1', 'k2')]
        lam[1], k1[1], k2[1] = gains[:3]
        rates = self.slew.copy(); rates[1] = gains[3]
        controller = SuperTwistingController(
            self.model, lam, k1, k2, params['z_limit'], controlled_joint='shoulder',
            control_hz=100., tau_limit=self.limits, torque_slew_rate=rates,
            base_ff_alpha=self.alpha)
        rng = np.random.default_rng(7)
        z = np.zeros((1, 1)); previous = np.zeros((1, 1, 3))
        for index in range(300):
            # Last 100 steps exercise torque clipping as well as the normal
            # small-angle/slew-limited trajectory regime.
            q, v, qr, vr, ar = rng.normal(0., .2, 5)
            if index >= 200:
                v, vr, ar = rng.normal(0., [2., 2., 400.])
            expected = controller.compute([0,q,0], [0,v,0], [0,qr,0],
                                          [0,vr,0], [0,ar,0], DT)
            tau, zn, fb, _ = self.step(np.array([[q]]), np.array([[v]]), qr, vr, ar,
                                      z, previous, gains[None, :])
            np.testing.assert_allclose(tau[0, 0], expected, atol=1e-10, rtol=0)
            np.testing.assert_allclose(zn[0, 0], controller.z[1], atol=1e-10, rtol=0)
            z, previous = zn, fb
            full = self.model.inverse_dynamics([0,q,0], [0,v,0], [0,ar,0])
            t = self.terms(q)
            np.testing.assert_allclose(t[0]+t[1]*ar+t[2]*v*v, full, atol=1e-10, rtol=0)
            np.testing.assert_allclose(t[1, 1], self.mass, atol=1e-10, rtol=0)

    def simulate(self, gains, cases, degrees, move, hold, seed=4, substeps=5, trace=False):
        t, qr, vr, ar, moving = reference(degrees, move, hold)
        nc, ns = len(gains), len(cases)
        shape = (nc, ns)
        q = np.zeros(shape); v = q.copy(); z = q.copy()
        previous = np.zeros((*shape, 3))
        mass, gs, fc, fs, noise, bias, delay = [
            np.array([c[key] for c in cases])[None, :]
            for key in ('inertia', 'g', 'fc', 'fs', 'noise', 'bias', 'delay')]
        mass = mass*self.mass
        past1 = np.broadcast_to(self.gravity(q)*gs-bias, shape).copy(); past2 = past1.copy()
        rng = np.random.default_rng(seed)
        esq = q.copy(); hsq = q.copy(); vsq = q.copy(); stepsq = q.copy()
        peak = q.copy(); clipped = q.copy(); last = past1.copy(); history = []
        for k in range(len(t)):
            qm = q+rng.normal(0., 2e-5, (1, ns))
            vm = v+noise*rng.normal(0., 1., (1, ns))
            all_tau, z, previous, raw = self.step(qm, vm, qr[k], vr[k], ar[k], z, previous, gains)
            tau = all_tau[..., 1]
            applied = np.where(delay == 0, tau, np.where(delay == 1, past1, past2))
            past2, past1 = past1, tau
            error = np.rad2deg(q-qr[k])
            if moving[k]: esq += error*error
            else: hsq += error*error
            vsq += np.rad2deg(v-vr[k])**2
            if k: stepsq += (tau-last)**2
            last = tau
            peak = np.maximum(peak, np.abs(error))
            clipped += np.any(np.abs(raw) > self.limits+1e-12, axis=-1)
            if trace: history.append(np.stack([q.copy(), v.copy(), tau.copy(), z.copy()]))
            h = DT/substeps
            for _ in range(substeps):
                drive = applied-gs*self.gravity(q)+bias-self.model.viscous[1]*v
                friction = fc*np.tanh(v/.002)
                stuck = (np.abs(v)<.0003)&(np.abs(drive)<=fs)&(fs>0)
                acceleration = np.where(stuck, 0., (drive-friction)/mass)
                vn = np.where(stuck, 0., v+h*acceleration)
                vn = np.where((vn*v<0)&(np.abs(drive)<fs)&(fs>0), 0., vn)
                q += .5*h*(v+vn); v = vn
        result = dict(zip(KEYS, [np.sqrt(esq/sum(moving)), np.sqrt(hsq/sum(~moving)),
                                np.sqrt(vsq/len(t)), np.sqrt(stepsq/(len(t)-1))]))
        result.update(max_error_deg=peak, saturated_samples=clipped)
        if trace: result.update(trace=np.stack(history), t=t, qr=qr)
        return result


def reference(degrees, move, hold):
    t = np.arange(round((2*move+2*hold)/DT))*DT
    qr = np.zeros(len(t)); vr = qr.copy(); ar = qr.copy(); moving = np.zeros(len(t), dtype=bool)
    for begin, start, end in [(0, 0, degrees), (move+hold, degrees, 0)]:
        mask = (t>=begin)&(t<begin+move)
        u = (t[mask]-begin)/move; delta = np.deg2rad(end-start)
        qr[mask] = np.deg2rad(start)+delta*(10*u**3-15*u**4+6*u**5)
        vr[mask] = delta*(30*u**2-60*u**3+30*u**4)/move
        ar[mask] = delta*(60*u-180*u**2+120*u**3)/move**2
        moving[mask] = True
    qr[(t>=move)&(t<move+hold)] = np.deg2rad(degrees)
    return t, qr, vr, ar, moving


def aggregate(results):
    metrics = {key: np.sqrt(np.mean(np.concatenate([r[key] for r in results], axis=1)**2, axis=1))
               for key in KEYS}
    metrics['max_error_deg'] = np.max(np.concatenate([r['max_error_deg'] for r in results], axis=1), axis=1)
    metrics['saturated_samples'] = np.sum(np.concatenate([r['saturated_samples'] for r in results], axis=1), axis=1)
    return metrics


def rows(metrics, indices):
    return [dict(label=label, **{key: float(value[i]) for key, value in metrics.items()})
            for label, i in indices]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params-file', type=Path, default=ROOT/'src/gim_arm_controller_smc/config/smc_circle_from_terminal_tuning.yaml')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'results_new/smc_super_twisting/id1_offline_search')
    args = parser.parse_args()
    params = yaml.safe_load(args.params_file.read_text())['smc_controller']['ros__parameters']
    if params['smc_variant'] != 'super_twisting' or float(params['control_hz']) != 100.:
        raise ValueError('This study requires STA at 100 Hz')
    study = ShoulderStudy(params, ROOT/'src/gim_arm_description/urdf/gim_arm.urdf')
    baseline = np.array([[params[key][1] for key in ('lambda_gain','k1','k2','torque_slew_rate')]], dtype=float)
    study.verify(baseline[0])
    grid = np.array(list(product([3.,4.,6.,8.,10.], [1.5,3.,4.,6.,9.], [.5,2.,4.,8.,12.], [2.,4.,8.,16.,48.])))
    allg = np.vstack([baseline, grid])
    print(f'ID1: evaluating {len(grid)} candidates; baseline={baseline[0].tolist()}, tau_limits={study.limits.tolist()}', flush=True)
    train = aggregate([study.simulate(allg, TRAIN, 3, 6, 3), study.simulate(allg, TRAIN, 10, 5, 2)])
    score = sum(weight*train[key]/train[key][0] for key, weight in zip(KEYS, [.45,.20,0.,.35]))
    feasible = ((train['max_error_deg']<2.) & (train['saturated_samples']==0)
                & (train[KEYS[0]]<train[KEYS[0]][0]) & (train[KEYS[3]]<train[KEYS[3]][0])
                & (train[KEYS[1]]<=train[KEYS[1]][0]) & (train[KEYS[2]]<=train[KEYS[2]][0]))
    ids = np.flatnonzero(feasible)
    order = ids[np.argsort(score[ids])]
    if not len(order): raise RuntimeError('No candidate met the training criteria')
    probes = order
    probe = np.vstack([baseline, allg[probes]])
    print(f'{len(ids)} passed training screen; testing leading {len(probes)} on extra assumed scenarios', flush=True)
    stress_results = [study.simulate(probe, STRESS, d,m,h,seed=s,substeps=10)
                      for d,m,h,s in [(3,6,3,91),(10,5,2,92),(7,4,3,137)]]
    stress = aggregate(stress_results)
    feasible = ((stress['max_error_deg']<2.) & (stress['saturated_samples']==0)
                & (stress[KEYS[0]]<stress[KEYS[0]][0]) & (stress[KEYS[3]]<stress[KEYS[3]][0])
                & (stress[KEYS[1]]<=stress[KEYS[1]][0]) & (stress[KEYS[2]]<=stress[KEYS[2]][0]))
    feasible[0] = False
    # A good aggregate must not conceal an unstable reference: apply the
    # improvement criteria separately to each screened reference as well.
    for result in stress_results:
        ref_metrics = aggregate([result])
        for key in KEYS:
            feasible &= ref_metrics[key] <= ref_metrics[key][0]
    if not feasible.any(): raise RuntimeError('No screened candidate met the stress criteria')
    slot = int(np.flatnonzero(feasible)[0])
    selected = probe[slot]; comparison = np.vstack([baseline, selected])
    study.verify(selected)
    checks = []
    for name, specs in [('plant_integrator_1ms', [(3,6,3,91),(10,5,2,92)]),
                        ('screened_7deg_reference_1ms', [(7,4,3,137)]),
                        ('new_5deg_reference_noise', [(5,5,4,2026)])]:
        metrics = aggregate([study.simulate(comparison, STRESS, d,m,h,seed=s,substeps=10)
                             for d,m,h,s in specs])
        checks.append(dict(name=name, metrics=rows(metrics, [('baseline',0),('candidate',1)])))
    report = dict(status='SIMULATION_ONLY', joint='shoulder/id1', hardware_modified=False, robot_trials_run=0,
        candidate_count=len(grid), stress_screened_count=len(probes),
        parameter_order=['lambda','k1','k2','slew_nm_s'], baseline=baseline[0].tolist(), candidate=selected.tolist(),
        source_params=params, source_params_file=str(args.params_file),
        selection='Training rank: 45% motion RMS + 20% hold RMS + 35% torque-step RMS, normalized to baseline; require lower motion and torque-step RMS and no degradation in hold/velocity. Screen ALL feasible training candidates on 3/10/7deg at plant dt=1ms: no degradation in any of four aggregate metrics for each reference; worst error <2deg and no torque clipping.',
        checks='300 random controller steps per parameter set, including 100 clipping stress steps: all three torques and shoulder z match full repository controller to absolute 1e-10; Fourier dynamics checked against URDF inverse dynamics.',
        dt_s=DT, z_limit=study.z_limit, torque_limits_nm=study.limits.tolist(),
        inertia_kgm2=study.mass, held_joints='base=0, elbow=0, ideal position holds',
        viscous_nm_s_rad=float(study.model.viscous[1]), model_training=TRAIN, model_stress=STRESS,
        stress_metrics=rows(stress,[('baseline',0),('candidate',slot)]), extra_checks=checks,
        warning='Friction/delay/noise are assumed. No ID1 physical log identifies this model; this is not a validated hardware tuning.',
        top_training_candidates=[dict(parameters=allg[i].tolist(), score=float(score[i])) for i in order[:10]])
    out = args.output_dir; out.mkdir(parents=True, exist_ok=True)
    (out/'simulation_candidate.json').write_text(json.dumps(report, indent=2)+'\n')
    # Snapshot is the INPUT baseline, not a deployable candidate profile.
    (out/'input_parameters.yaml').write_text(yaml.safe_dump(
        {'smc_controller': {'ros__parameters': params}}, sort_keys=False))
    trace = study.simulate(comparison,[STRESS[1]],10,5,2,seed=92,trace=True)
    fig, axes = plt.subplots(3,1,figsize=(12,9),sharex=True,layout='constrained')
    t, history = trace['t'], trace['trace']
    axes[0].plot(t, np.rad2deg(trace['qr']), 'k--', label='Reference')
    for i, (label, color) in enumerate([('Baseline','#c06030'),('Simulation candidate','#1766a2')]):
        axes[0].plot(t,np.rad2deg(history[:,0,i,0]),color=color,lw=1,label=label)
        axes[1].plot(t,np.rad2deg(history[:,0,i,0]-trace['qr']),color=color,lw=1,label=label)
        axes[2].plot(t,history[:,2,i,0],color=color,lw=.8,label=label)
    for ax, label in zip(axes,['Position (deg)','Position error (deg)','Commanded torque (Nm)']):
        ax.set_ylabel(label); ax.grid(alpha=.2); ax.legend()
    axes[-1].set_xlabel('Simulated time (s)')
    fig.suptitle('ID1 STA: assumed friction/delay/noise, simulation only; no robot trials')
    fig.savefig(out/'simulation_comparison.png',dpi=160); plt.close(fig)
    print(json.dumps(dict(candidate=selected.tolist(),stress_metrics=report['stress_metrics'],extra_checks=checks),indent=2),flush=True)


if __name__ == '__main__':
    main()
