"""Numerical STA study only: no ROS, CAN, motor commands or hardware writes."""
from pathlib import Path
from itertools import product
import json
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/gim_arm_control'))
sys.path.insert(0, str(ROOT/'src/gim_arm_controller_smc'))
from gim_control.arm_dynamics import ArmDynamics
from gim_arm_controller_smc.super_twisting import SuperTwistingController
OUT=ROOT/'results_new/smc_super_twisting/id2_offline_search'
OUT.mkdir(parents=True,exist_ok=True)
DT=.01
model=ArmDynamics(str(ROOT/'src/gim_arm_description/urdf/gim_arm.urdf'),use_armature=True)
M=float(model.mass_matrix(np.zeros(3))[2,2])
g0,g90,g180=[model.gravity([0.,0.,q])[2] for q in (0.,np.pi/2,np.pi)]
GC=(g0+g180)/2; GA=g0-GC; GB=g90-GC

def gravity(q):return GA*np.cos(q)+GB*np.sin(q)+GC

def sta_step(q,v,qr,vr,ar,z,previous,gains):
    lam,k1,k2,slew=[gains[:,i,None] for i in range(4)]
    error=q-qr; de=v-vr; surface=de+lam*error
    ff=gravity(q)+M*ar
    raw_fb=M*(-lam*de-k1*np.sqrt(np.abs(surface))*np.sign(surface)+z)
    fb=np.clip(raw_fb,previous-slew*DT,previous+slew*DT)
    raw=ff+fb; tau=np.clip(raw,-1.75,1.75)
    inc=-k2*np.sign(surface)*DT
    blocked=(raw_fb-fb+raw-tau)*(M*inc)>1e-12
    zn=np.clip(z+np.where(blocked,0.,inc),-30.,30.)
    return tau,zn,tau-ff,raw

def verify_kernel():
    class Dyn:
        nq=3
        joint_names=['base_joint','shoulder_joint','elbow_joint']
        tau_max=np.array([5.,40.,5.])
        def mass_matrix(self,q):return np.diag([.05,.16,M])
        def inverse_dynamics(self,q,v,a):return self.mass_matrix(q)@a+np.array([0.,0.,gravity(q[2])])
    gains=np.array([[6.,3.,2.,1.]])
    c=SuperTwistingController(Dyn(),[3.,4.,6.],[3.,4.,3.],[1.,2.,2.],
        [20.,40.,30.],controlled_joint='elbow',control_hz=100.,tau_limit=[1.75,14.,1.75],torque_slew_rate=[10.,48.,1.])
    rng=np.random.default_rng(1);z=np.zeros((1,1));prev=z.copy()
    for _ in range(200):
        q,v,qr,vr,ar=rng.normal(0.,.08,5)
        a,zn,p,_=sta_step(np.array([[q]]),np.array([[v]]),qr,vr,ar,z,prev,gains)
        expected=c.compute([0,0,q],[0,0,v],[0,0,qr],[0,0,vr],[0,0,ar],DT)
        np.testing.assert_allclose(a[0,0],expected[2],atol=1e-12)
        np.testing.assert_allclose(zn[0,0],c.z[2],atol=1e-12)
        z,prev=zn,p


def verify_full_model(gains):
    lam,k1,k2,slew=gains
    controller=SuperTwistingController(model,[3.,4.,lam],[3.,4.,k1],
        [1.,2.,k2],[20.,40.,30.],controlled_joint='elbow',control_hz=100.,
        tau_limit=[1.75,14.,1.75],torque_slew_rate=[10.,48.,slew],base_ff_alpha=2.46)
    rng=np.random.default_rng(7);z=np.zeros((1,1));previous=z.copy()
    for _ in range(200):
        q,v,qr,vr,ar=rng.normal(0.,.05,5)
        actual=controller.compute([0,0,q],[0,0,v],[0,0,qr],[0,0,vr],[0,0,ar],DT)
        tau,zn,feedback,_=sta_step(np.array([[q]]),np.array([[v]]),
            qr,vr,ar,z,previous,gains[None,:])
        np.testing.assert_allclose(actual[2],tau[0,0],atol=1e-10)
        np.testing.assert_allclose(controller.z[2],zn[0,0],atol=1e-10)
        z,previous=zn,feedback


def reference(degrees,move,hold):
    t=np.arange(round((2*move+2*hold)/DT))*DT
    qr=np.zeros(len(t));vr=qr.copy();ar=qr.copy();active=np.zeros(len(t),dtype=bool)
    for begin,duration,start,end in [(0,move,0,degrees),(move+hold,move,degrees,0)]:
        mask=(t>=begin)&(t<begin+duration)
        u=(t[mask]-begin)/duration;delta=np.deg2rad(end-start)
        qr[mask]=np.deg2rad(start)+delta*(10*u**3-15*u**4+6*u**5)
        vr[mask]=delta*(30*u**2-60*u**3+30*u**4)/duration
        ar[mask]=delta*(60*u-180*u**2+120*u**3)/duration**2
        active[mask]=True
    qr[(t>=move)&(t<move+hold)]=np.deg2rad(degrees)
    return t,qr,vr,ar,active

# These are assumed stress cases, not identified physical motor parameters.
TRAIN=[
 dict(name='nominal_smooth',inertia=1.,g=1.,fc=.032,fs=0.,noise=.005,delay=0,bias=0.),
 dict(name='static_friction',inertia=1.,g=1.,fc=.04,fs=.065,noise=.008,delay=1,bias=.005),
 dict(name='model_error',inertia=1.2,g=1.10,fc=.045,fs=.07,noise=.012,delay=1,bias=-.01),
]
VALID=[
 dict(name='heldout_light',inertia=.85,g=.92,fc=.025,fs=.05,noise=.01,delay=1,bias=.008),
 dict(name='heldout_delay',inertia=1.15,g=1.08,fc=.04,fs=.08,noise=.015,delay=2,bias=-.01),
 dict(name='heldout_stiction',inertia=1.25,g=1.15,fc=.055,fs=.09,noise=.01,delay=1,bias=.015),
]

def simulate(gains,cases,degrees,move,hold,seed=4,trace=False,substeps=5):
    t,qr,vr,ar,moving=reference(degrees,move,hold)
    nc,ns=len(gains),len(cases); shape=(nc,ns)
    q=np.zeros(shape);v=q.copy();z=q.copy();prev=q.copy()
    mass=M*np.array([s['inertia'] for s in cases])[None,:]
    gs=np.array([s['g'] for s in cases])[None,:]
    fc=np.array([s['fc'] for s in cases])[None,:]
    fs=np.array([s['fs'] for s in cases])[None,:]
    noise=np.array([s['noise'] for s in cases])[None,:]
    bias=np.array([s['bias'] for s in cases])[None,:]
    delay=np.array([s['delay'] for s in cases])[None,:]
    past1=np.broadcast_to(gravity(q)*gs-bias,shape).copy();past2=past1.copy()
    rng=np.random.default_rng(seed)
    esq=np.zeros(shape);hsq=esq.copy();vsq=esq.copy();stepsq=esq.copy();maxe=esq.copy();maxv=esq.copy();sats=esq.copy()
    lasttau=past1.copy();history=[]
    for k in range(len(t)):
        qm=q+rng.normal(0.,2e-5,(1,ns));vm=v+noise*rng.normal(0.,1.,(1,ns))
        tau,z,prev,raw=sta_step(qm,vm,qr[k],vr[k],ar[k],z,prev,gains)
        applied=np.where(delay==0,tau,np.where(delay==1,past1,past2))
        past2,past1=past1,tau
        e=np.rad2deg(q-qr[k])
        if moving[k]:esq+=e*e
        else:hsq+=e*e
        vsq+=(np.rad2deg(v-vr[k]))**2
        if k>0:stepsq+=(tau-lasttau)**2
        lasttau=tau
        maxe=np.maximum(maxe,np.abs(e));maxv=np.maximum(maxv,np.abs(np.rad2deg(v)))
        sats+=(np.abs(raw)>1.75)
        if trace:history.append(np.stack([q.copy(),v.copy(),tau.copy(),z.copy()]))
        hstep=DT/substeps
        for _ in range(substeps):
            drive=applied-gs*gravity(q)+bias-.00128*v
            friction=fc*np.tanh(v/.002)
            stuck=(np.abs(v)<.0003)&(np.abs(drive)<=fs)&(fs>0)
            acceleration=np.where(stuck,0.,(drive-friction)/mass)
            vn=np.where(stuck,0.,v+hstep*acceleration)
            crossing=(vn*v<0)&(np.abs(drive)<fs)&(fs>0)
            vn=np.where(crossing,0.,vn)
            q+=.5*hstep*(v+vn);v=vn
    metrics=dict(rms=np.sqrt(esq/sum(moving)),hold_rms=np.sqrt(hsq/sum(~moving)),velocity_rms=np.sqrt(vsq/len(t)),ripple=np.sqrt(stepsq/(len(t)-1)),
                 max_error=maxe,peak_velocity=maxv,saturated=sats)
    if trace:metrics['trace']=np.stack(history);metrics['t']=t;metrics['qr']=qr;metrics['vr']=vr
    return metrics


def main():
    verify_kernel()
    grid=np.array(list(product([3.,4.,6.,8.,10.],[1.5,3.,4.5,6.,9.],[.5,1.,2.,4.,8.],[.5,1.,2.,4.,7.])))
    baseline=np.array([[4.,3.,1.,7.]])
    allg=np.vstack([baseline,grid])
    train=[simulate(allg,TRAIN,3,6,3),simulate(allg,TRAIN,10,5,2)]
    rms=np.concatenate([r['rms'] for r in train],axis=1)
    ripple=np.concatenate([r['ripple'] for r in train],axis=1)
    peaks=np.concatenate([r['max_error'] for r in train],axis=1)
    clipping=np.concatenate([r['saturated'] for r in train],axis=1)
    hold=np.concatenate([r['hold_rms'] for r in train],axis=1)
    velocity=np.concatenate([r['velocity_rms'] for r in train],axis=1)
    hold_score=np.sqrt(np.mean(hold*hold,axis=1))
    velocity_score=np.sqrt(np.mean(velocity*velocity,axis=1))
    error_score=np.sqrt(np.mean(rms*rms,axis=1));ripple_score=np.sqrt(np.mean(ripple*ripple,axis=1))
    score=.45*error_score/error_score[0]+.35*ripple_score/ripple_score[0]+.20*hold_score/hold_score[0]
    feasible=(peaks.max(axis=1)<2.)&(clipping.max(axis=1)==0)&(error_score<error_score[0])&(ripple_score<ripple_score[0])&(hold_score<=hold_score[0])&(velocity_score<=velocity_score[0])
    ids=np.flatnonzero(feasible);order=ids[np.argsort(score[ids])]
    if not len(order):raise RuntimeError('No candidate improves both model tracking and torque-step RMS')
    # Screen the leading candidates against the extra assumed stress cases.
    # This is a numerical design screen, not a validation on real hardware.
    probe_ids=order[:30]
    probe=np.vstack([baseline,allg[probe_ids]])
    validation=[simulate(probe,VALID,3,6,3,seed=91),simulate(probe,VALID,10,5,2,seed=92)]
    aggregate={key:np.sqrt(np.mean(np.concatenate([v[key] for v in validation],axis=1)**2,axis=1))
               for key in ('rms','hold_rms','velocity_rms','ripple')}
    acceptable=np.ones(len(probe),dtype=bool)
    for key in aggregate:
        acceptable &= aggregate[key] < aggregate[key][0]
    acceptable[0]=False
    if not acceptable.any():
        raise RuntimeError('No screened candidate improves all four simulation metrics')
    slot=int(np.flatnonzero(acceptable)[0])
    selected=probe[slot]
    comparison=np.vstack([baseline,selected])
    verify_full_model(selected)
    summary=[]
    for label,index in [('baseline',0),('candidate',slot)]:
        summary.append(dict(label=label,parameters=dict(zip(['lambda','k1','k2','slew_nm_s'],probe[index].tolist())),
            validation_rms_motion_deg=float(aggregate['rms'][index]),
            validation_rms_hold_deg=float(aggregate['hold_rms'][index]),
            validation_rms_velocity_error_deg_s=float(aggregate['velocity_rms'][index]),
            validation_rms_torque_step_nm=float(aggregate['ripple'][index]),
            validation_worst_error_deg=float(max(np.max(v['max_error'][index]) for v in validation)),
            validation_saturated_samples=int(sum(np.sum(v['saturated'][index]) for v in validation))))
    extra_checks=[]
    for name,results in [
            ('plant_integrator_1ms',[
                simulate(comparison,VALID,3,6,3,seed=91,substeps=10),
                simulate(comparison,VALID,10,5,2,seed=92,substeps=10)]),
            ('new_reference_7deg_new_noise',[
                simulate(comparison,VALID,7,4,3,seed=137,substeps=10)])]:
        extra_checks.append(dict(name=name,metrics=[dict(
            label=label,
            motion_rms_deg=float(np.sqrt(np.mean(np.concatenate([v['rms'][index] for v in results])**2))),
            hold_rms_deg=float(np.sqrt(np.mean(np.concatenate([v['hold_rms'][index] for v in results])**2))),
            velocity_rms_deg_s=float(np.sqrt(np.mean(np.concatenate([v['velocity_rms'][index] for v in results])**2))),
            torque_step_rms_nm=float(np.sqrt(np.mean(np.concatenate([v['ripple'][index] for v in results])**2))))
            for label,index in [('baseline',0),('candidate',1)]]))
    report=dict(status='SIMULATION_ONLY',hardware_modified=False,robot_trials_run=0,
        algorithm='SMC super-twisting, explicit Euler, 100 Hz, z_limit=30 rad/s^2, tau_limit=1.75 Nm',
        candidate_count=len(grid),stress_screened_count=len(probe_ids),
        selection='45% normalized motion RMS + 35% torque-step RMS + 20% hold RMS; require improvement in motion, hold, velocity and torque-step RMS on the extra assumed stress scenarios',
        kernel_verification='200-step equivalence check against repository SuperTwistingController passed',
        coupled_urdf_verification='200 additional steps match full coupled URDF controller output and z at 1e-10 tolerance.',
        extra_checks=extra_checks,
        model=dict(inertia_kgm2=M,gravity_coefficients=[float(GA),float(GB),float(GC)],held_joints='base=0, shoulder=0',
            integration_step_s=.002,training=TRAIN,validation=VALID),
        summary=summary,
        warning='Assumed stress cases have not been identified or validated on the physical id2 after repair. This result is not a deployment profile.',
        top_training_candidates=[dict(parameters=allg[i].tolist(),score=float(score[i]),motion_rms=float(error_score[i]),torque_step_rms=float(ripple_score[i])) for i in order[:10]])
    (OUT/'simulation_candidate.json').write_text(json.dumps(report,indent=2)+'\n')
    trace=simulate(comparison,[VALID[1]],10,5,2,seed=92,trace=True)
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True,layout='constrained')
    t=trace['t'];h=trace['trace']
    axes[0].plot(t,np.rad2deg(trace['qr']),'k--',lw=1.3,label='Reference')
    for i,(label,color) in enumerate([('Baseline','#c06030'),('Simulation candidate','#1766a2')]):
        axes[0].plot(t,np.rad2deg(h[:,0,i,0]),color=color,lw=1.,label=label)
        axes[1].plot(t,np.rad2deg(h[:,0,i,0]-trace['qr']),color=color,lw=1.,label=label)
        axes[2].plot(t,h[:,2,i,0],color=color,lw=.8,label=label)
    for ax,label in zip(axes,['Position (deg)','Position error (deg)','Commanded torque (Nm)']):
        ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend()
    axes[-1].set_xlabel('Simulated time (s)')
    fig.suptitle('ID2 STA — assumed delayed/friction model, simulation only; no robot trials')
    fig.savefig(OUT/'simulation_comparison.png',dpi=160);plt.close(fig)
    print(json.dumps(summary,indent=2),flush=True)
    print('Training candidates improving both metrics:',len(ids),flush=True)


if __name__ == "__main__":
    main()
