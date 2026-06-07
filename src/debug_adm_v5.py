"""debug — check force + orientation response"""
import numpy as np, mujoco, os, csv
from collections import deque
from scipy.spatial.transform import Rotation as R
from src.dual_arm_ik_v5 import DualArmIKV5
from src.alfa_interface_v5 import AlfaRobotInterfaceV5

DT=0.002;F_TARGET=-5.0;T_TARGET=np.zeros(3)

class Adm6D:
    def __init__(self):
        self.M=np.array([0.08,0.08,0.08,0.02,0.02,0.02]);self.D=np.array([25.,25.,25.,12.,12.,12.])
        self.vmax=np.array([0.005,0.005,0.005,0.2,0.2,0.2]);self.cmax=np.array([0.02,0.02,0.01,0.2,0.2,0.2])
        self.vel=np.zeros(6);self.corr=np.zeros(6)
    def reset(self):self.vel[:]=0;self.corr[:]=0
    def step(self,e,dt):
        self.vel+=(e-self.D*self.vel)/self.M*dt;self.vel=np.clip(self.vel,-self.vmax,self.vmax)
        self.corr+=self.vel*dt;self.corr=np.clip(self.corr,-self.cmax,self.cmax)
        return self.corr[:3].copy(),self.corr[3:].copy()

def T_s(m,d,n):
    ii=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,n)
    T=np.eye(4);T[:3,3]=d.site_xpos[ii];T[:3,:3]=d.site_xmat[ii].reshape(3,3);return T
def T_b(m,d,n):
    ii=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n)
    T=np.eye(4);T[:3,3]=d.xpos[ii];T[:3,:3]=d.xmat[ii].reshape(3,3);return T
def Tee(m,d,s):return np.linalg.inv(T_b(m,d,'updown_link'))@T_s(m,d,f'{s}_ee')
def gq(m,d,s):return np.array([d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,f'{s}joint{i+1}')]]for i in range(6)])
def sq(m,d,r,q,s):
    for i in range(6):
        n=f'{s}joint{i+1}';jid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)
        d.qpos[m.jnt_qposadr[jid]]=q[i];d.qvel[m.jnt_dofadr[jid]]=0.0
        aid=r.actuator_ids.get(n,-1)
        if aid>=0:d.ctrl[aid]=q[i]
def sctrl(m,d,r,q,s):
    for i in range(6):
        aid=r.actuator_ids.get(f'{s}joint{i+1}',-1)
        if aid>=0:d.ctrl[aid]=q[i]
def gft(m,d,s):
    gid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,f'{s}_suction_col')
    Tee_=T_s(m,d,f'{s}_ee');ep=Tee_[:3,3];eR=Tee_[:3,:3];Fw=np.zeros(3);Tw=np.zeros(3);bn=None
    for i in range(d.ncon):
        c=d.contact[i]
        if c.geom2==gid:o2=True
        elif c.geom1==gid:o2=False
        else:continue
        oi=c.geom1 if o2 else c.geom2
        on=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,m.geom_bodyid[oi])or''
        if not on.startswith('box_'):continue
        if bn is None:bn=on
        cf=np.zeros(6);mujoco.mj_contactForce(m,d,i,cf);fr=c.frame.reshape(3,3)
        Fi=fr@cf[:3]if o2 else-fr@cf[:3];Fw+=Fi;Tw+=np.cross(c.pos-ep,Fi)
    return Fw,eR.T@Fw,eR.T@Tw,bn
def sstep(t):return 3*t*t-2*t*t*t
def sim_jac6(m,d,r,s):
    eps=0.001;J=np.zeros((6,6))
    eid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,f'{s}_ee')
    q0=gq(m,d,s);sq(m,d,r,q0,s);mujoco.mj_forward(m,d)
    p0=d.site_xpos[eid].copy();R0=d.site_xmat[eid].reshape(3,3).copy()
    for i in range(6):
        qp=q0.copy();qp[i]+=eps;sq(m,d,r,qp,s);mujoco.mj_forward(m,d)
        p1=d.site_xpos[eid].copy();R1=d.site_xmat[eid].reshape(3,3)
        J[:3,i]=(p1-p0)/eps
        dR=R1@R0.T;J[3:,i]=np.array([dR[2,1]-dR[1,2],dR[0,2]-dR[2,0],dR[1,0]-dR[0,1]])/(2*eps)
    sq(m,d,r,q0,s);mujoco.mj_forward(m,d)
    return J

m=mujoco.MjModel.from_xml_path('scene_v5.xml');d=mujoco.MjData(m);mujoco.mj_resetData(m,d)
r=AlfaRobotInterfaceV5(m,d);ik=DualArmIKV5()
q_home=np.array([0,np.pi/2,0,-np.pi/2,0,0]);orient=np.radians([0,90,0])
adm={s:Adm6D()for s in['left','right']}

# Init
r.apply_hybrid_target(dict(base_x=0,base_y=0,base_yaw=0,pitch=0,turn=0,updown=0.3,
    leftjoint1=0,leftjoint2=0,leftjoint3=0,leftjoint4=-np.pi/2,leftjoint5=0,leftjoint6=0,
    rightjoint1=0,rightjoint2=0,rightjoint3=0,rightjoint4=-np.pi/2,rightjoint5=0,rightjoint6=0,
    right_suction=0,left_suction=0))
mujoco.mj_forward(m,d)
for _ in range(200):mujoco.mj_step(m,d)

# IK
q_above={'left':ik.solve_left_pose(([0.90,0.29,0.50],orient),q_init=q_home),
         'right':ik.solve_right_pose(([0.90,-0.29,0.50],orient),q_init=q_home)}
q_pre={'left':ik.solve_left_pose(([0.94,0.29,0.30],orient),q_init=q_above['left']),
       'right':ik.solve_right_pose(([0.94,-0.29,0.30],orient),q_init=q_above['right'])}

# Path
q0={s:gq(m,d,s)for s in['left','right']}
for i in range(200):
    a=sstep(i/199)
    for s in['left','right']:sq(m,d,r,q0[s]+a*(q_above[s]-q0[s]),s)
    mujoco.mj_step(m,d)
for _ in range(50):mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
q0={s:gq(m,d,s)for s in['left','right']}
for i in range(200):
    a=sstep(i/199)
    for s in['left','right']:sq(m,d,r,q0[s]+a*(q_pre[s]-q0[s]),s)
    mujoco.mj_step(m,d)
for _ in range(50):mujoco.mj_step(m,d);mujoco.mj_forward(m,d)

# Orientation perturbation
np.random.seed(42)
for s in['left','right']:
    q_now=gq(m,d,s)
    perturb=np.zeros(6)
    perturb[4]=np.random.uniform(-0.05,0.05)
    # no j5
    sq(m,d,r,q_now+perturb,s)
mujoco.mj_forward(m,d)
for _ in range(30):mujoco.mj_step(m,d)
mujoco.mj_forward(m,d)

# Print initial state
for s in['left','right']:
    Tu=Tee(m,d,s);rpy=R.from_matrix(Tu[:3,:3]).as_euler('xyz')
    print(f'{s} pre-admit: pos={np.round(Tu[:3,3],3)} rpy_deg={np.round(np.degrees(rpy),1)}',flush=True)

# Admittance loop
phase={s:'appr'for s in['left','right']};done={s:False for s in['left','right']}
cs={s:0 for s in['left','right']};Tref={};Fb={s:deque(maxlen=10)for s in['left','right']}
J={};lost={s:0 for s in['left','right']}
rec={s:{'t':[],'F':[],'T':[],'pos':[],'rpy':[],'corr':[]}for s in['left','right']}
contact_step={}

for step in range(6000):
    for jn in['base_x','base_y','base_yaw']:
        qadr=r.jnt_qpos_adrs.get(jn)
        if qadr is not None:d.qpos[qadr]=0.0;d.qvel[r.jnt_qvel_adrs[jn]]=0.0
    mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
    for s in['left','right']:
        if done[s]:continue
        Fw,Fe,Te,bn=gft(m,d,s);Tu=Tee(m,d,s);rpy=R.from_matrix(Tu[:3,:3]).as_euler('xyz')
        Fb[s].append(Fe.copy());Ff=np.mean(np.array(Fb[s]),axis=0);Fmag=np.linalg.norm(Ff)
        rec[s]['t'].append(step*DT);rec[s]['F'].append(Fe.copy());rec[s]['T'].append(Te.copy())
        rec[s]['pos'].append(Tu[:3,3].copy());rec[s]['rpy'].append(rpy);rec[s]['corr'].append(adm[s].corr.copy())
        if phase[s]=='appr':
            if step%10==0:J[s]=sim_jac6(m,d,r,s)
            Jj=J.get(s,sim_jac6(m,d,r,s));J3=Jj[:3];JJt=J3@J3.T+0.05*np.eye(3)
            dq=J3.T@np.linalg.inv(JJt)@np.array([0.0005,0,0]);dq=np.clip(dq,-0.015,0.015)
            qt=gq(m,d,s)+dq
            if Fmag>=0.3 and len(Fb[s])>=10:
                phase[s]='admit';adm[s].reset();Tref[s]=Tu.copy();cs[s]=0;lost[s]=0;contact_step[s]=step
                print(f'[{step:4d}] {s}: CONTACT->ADMIT Fz={Fe[2]:.1f}N rpy={np.round(np.degrees(rpy),1)}',flush=True)
                continue
            sq(m,d,r,qt,s)
        else:
            cs[s]+=1
            if Fmag<0.3:lost[s]+=1
            else:lost[s]=0
            Fe6=np.array([Fe[0],Fe[1],Fe[2]-F_TARGET]);te6=Te-T_TARGET
            dp,dr=adm[s].step(np.concatenate([Fe6,te6]),DT)
            if step%15==0:J[s]=sim_jac6(m,d,r,s)
            dp_w=Tref[s][:3,:3]@dp
            dr_w=Tref[s][:3,:3]@dr
            Jj=J.get(s,sim_jac6(m,d,r,s));JJt=Jj@Jj.T+0.1*np.eye(6)
            dq=Jj.T@np.linalg.inv(JJt)@np.concatenate([dp_w,dr_w]);dq=np.clip(dq,-0.03,0.03)
            qt=gq(m,d,s)+dq;sctrl(m,d,r,qt,s)
            if lost[s]>2000 or cs[s]>5000:done[s]=True
    if step%200==0:
        st=[]
        for x in['left','right']:
            c=adm[x].corr;fb=Fb[x];fz=fb[-1][2]if fb else 0
            st.append(f'{x[0]}={phase[x][:5]} Fz={fz:.0f} corrP=[{c[0]:.3f},{c[1]:.3f},{c[2]:.3f}] corrR=[{c[3]:.2f},{c[4]:.2f},{c[5]:.2f}]')
        print(f'  [{step:4d}] {" | ".join(st)}',flush=True)
    if all(done.values()):break

# Save CSV
os.makedirs('output',exist_ok=True)
for s in['left','right']:
    t=np.array(rec[s]['t']);Fa=np.array(rec[s]['F']);Ta=np.array(rec[s]['T'])
    pa=np.array(rec[s]['pos']);ra=np.degrees(np.array(rec[s]['rpy']));ca=np.array(rec[s]['corr'])
    n=len(t)
    with open(f'output/adm_{s}.csv','w',newline='')as f:
        w=csv.writer(f)
        w.writerow(['t','Fx','Fy','Fz','Tx','Ty','Tz','X','Y','Z','roll_deg','pitch_deg','yaw_deg','corrPx','corrPy','corrPz','corrRx','corrRy','corrRz'])
        for i in range(n):w.writerow([t[i],*Fa[i],*Ta[i],*pa[i],*ra[i],*ca[i]])

# Summary
for s in['left','right']:
    Fa=np.array(rec[s]['F']);Ta=np.array(rec[s]['T']);ra=np.degrees(np.array(rec[s]['rpy']));ca=np.array(rec[s]['corr'])
    ic=np.where(np.abs(Fa[:,2])>0.1)[0]
    print(f'\n{s}: {len(ic)} contact samples (out of {len(Fa)})')
    if len(ic)>0:
        print(f'  Fz: mean={Fa[ic,2].mean():.1f}N std={Fa[ic,2].std():.1f}N max={np.abs(Fa[ic,2]).max():.1f}N')
        print(f'  |T|: mean={np.linalg.norm(Ta[ic],axis=1).mean():.2f}Nm max={np.linalg.norm(Ta[ic],axis=1).max():.2f}Nm')
        print(f'  Roll: {ra[ic,0].mean():.1f}->{ra[ic,-1]:.1f}deg  Pitch: {ra[ic,1].mean():.1f}->{ra[ic,-1]:.1f}deg')
        print(f'  corrR_end: [{ca[-1,3]:.2f},{ca[-1,4]:.2f},{ca[-1,5]:.2f}]')
        print(f'  corrP_end: [{ca[-1,0]:.3f},{ca[-1,1]:.3f},{ca[-1,2]:.3f}]')

print('\nDone. output/adm_{left,right}.csv')
