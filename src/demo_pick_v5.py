"""
v5 双臂六维导纳 — 无焊点自由箱子 + EMA力滤波
"""
import numpy as np, mujoco, mujoco.viewer, os, sys, csv
from collections import deque
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from dual_arm_ik_v5 import DualArmIKV5
from alfa_interface_v5 import AlfaRobotInterfaceV5

DT=0.002;F_TARGET=-5.0;T_TARGET=np.zeros(3);ADMIT_STEPS=8000

class Adm6D:
    def __init__(self):
        self.M=np.array([0.08,0.08,0.08,0.02,0.02,0.02]);self.D=np.array([25.,25.,25.,6.,6.,6.])
        self.vmax=np.array([0.005,0.005,0.005,0.5,0.5,0.5]);self.cmax=np.array([0.02,0.02,0.01,0.4,0.4,0.4])
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
    Tee_=T_s(m,d,f'{s}_ee');ep=Tee_[:3,3];eR=Tee_[:3,:3];Fw=np.zeros(3);Tw=np.zeros(3)
    for i in range(d.ncon):
        c=d.contact[i]
        if c.geom2==gid:o2=True
        elif c.geom1==gid:o2=False
        else:continue
        oi=c.geom1 if o2 else c.geom2
        on=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,m.geom_bodyid[oi])or''
        if not on.startswith('box_'):continue
        cf=np.zeros(6);mujoco.mj_contactForce(m,d,i,cf);fr=c.frame.reshape(3,3)
        Fi=fr@cf[:3]if o2 else-fr@cf[:3];Fw+=Fi;Tw+=np.cross(c.pos-ep,Fi)
    return Fw,eR.T@Fw,eR.T@Tw
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

def main():
    print("Loading...")
    m=mujoco.MjModel.from_xml_path('scene_v5.xml');d=mujoco.MjData(m);mujoco.mj_resetData(m,d)
    r=AlfaRobotInterfaceV5(m,d);ik=DualArmIKV5()
    q_home=np.array([0,np.pi/2,0,-np.pi/2,0,0]);orient=np.radians([0,90,0])
    t_above_L=([0.85,0.29,0.50],orient);t_above_R=([0.85,-0.29,0.50],orient)
    t_pre_L=([0.88,0.29,0.30],orient);t_pre_R=([0.88,-0.29,0.30],orient)
    adm={s:Adm6D()for s in['left','right']}

    viewer=mujoco.viewer.launch_passive(m,d)
    viewer.cam.distance=4.5;viewer.cam.elevation=-20;viewer.cam.lookat[:]=[1.5,0,0.8]

    print("[1] Init")
    r.apply_hybrid_target(dict(base_x=0,base_y=0,base_yaw=0,pitch=0,turn=0,updown=0.3,
        leftjoint1=0,leftjoint2=0,leftjoint3=0,leftjoint4=-np.pi/2,leftjoint5=0,leftjoint6=0,
        rightjoint1=0,rightjoint2=0,rightjoint3=0,rightjoint4=-np.pi/2,rightjoint5=0,rightjoint6=0,
        right_suction=0,left_suction=0))
    mujoco.mj_forward(m,d)
    for _ in range(200):
        if not viewer.is_running():sys.exit(0)
        mujoco.mj_step(m,d);viewer.sync()

    print("[2] IK")
    q_above={};q_pre={}
    for s,ta,tp in[('left',t_above_L,t_pre_L),('right',t_above_R,t_pre_R)]:
        q_above[s]=ik.solve_left_pose(ta,q_init=q_home)if s=='left'else ik.solve_right_pose(ta,q_init=q_home)
        q_pre[s]=ik.solve_left_pose(tp,q_init=q_above[s])if s=='left'else ik.solve_right_pose(tp,q_init=q_above[s])

    print("[3] Path")
    q0={s:gq(m,d,s)for s in['left','right']}
    for i in range(200):
        a=sstep(i/199)
        for s in['left','right']:sq(m,d,r,q0[s]+a*(q_above[s]-q0[s]),s)
        if not viewer.is_running():sys.exit(0)
        mujoco.mj_step(m,d)
        if i%3==0:viewer.sync()
    for _ in range(50):mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
    q0={s:gq(m,d,s)for s in['left','right']}
    for i in range(200):
        a=sstep(i/199)
        for s in['left','right']:sq(m,d,r,q0[s]+a*(q_pre[s]-q0[s]),s)
        if not viewer.is_running():sys.exit(0)
        mujoco.mj_step(m,d)
        if i%3==0:viewer.sync()
    for _ in range(50):mujoco.mj_step(m,d);mujoco.mj_forward(m,d)

    # 姿态加噪
    np.random.seed(42)
    for s in['left','right']:
        perturb=np.zeros(6)
        perturb[4]=np.random.uniform(-0.12,0.12)
        sq(m,d,r,gq(m,d,s)+perturb,s)
    mujoco.mj_forward(m,d)
    for _ in range(30):mujoco.mj_step(m,d)
    mujoco.mj_forward(m,d)

    print(f"[4] 6-DOF Admittance (F_target={F_TARGET}N, no welds)")

    rec={};phase={s:'appr'for s in['left','right']};done={s:False for s in['left','right']}
    cs={s:0 for s in['left','right']};Tref={};Fb={s:deque(maxlen=10)for s in['left','right']}
    J={};lost={s:0 for s in['left','right']}
    for s in['left','right']:rec[s]={'t':[],'F':[],'T':[],'pos':[],'rpy':[],'dp':[],'dr':[],'F_f':[],'T_f':[]}
    # EMA滤波
    Fe_ema={s:None for s in['left','right']};Te_ema={s:None for s in['left','right']}

    for step in range(ADMIT_STEPS):
        if not viewer.is_running():break
        for jn in['base_x','base_y','base_yaw']:
            qadr=r.jnt_qpos_adrs.get(jn)
            if qadr is not None:d.qpos[qadr]=0.0;d.qvel[r.jnt_qvel_adrs[jn]]=0.0
        mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
        for s in['left','right']:
            if done[s]:continue
            Fw,Fe,Te=gft(m,d,s);Tu=Tee(m,d,s);rpy=R.from_matrix(Tu[:3,:3]).as_euler('xyz')
            Fb[s].append(Fe.copy());Ff=np.mean(np.array(Fb[s]),axis=0);Fmag=np.linalg.norm(Ff)
            # EMA滤波力/力矩
            a=0.1
            Fe_ema[s]=Fe.copy()if Fe_ema[s]is None else a*Fe+(1-a)*Fe_ema[s]
            Te_ema[s]=Te.copy()if Te_ema[s]is None else a*Te+(1-a)*Te_ema[s]
            rec[s]['t'].append(step*DT);rec[s]['F'].append(Fe.copy());rec[s]['T'].append(Te.copy())
            rec[s]['pos'].append(Tu[:3,3].copy());rec[s]['rpy'].append(rpy)
            rec[s]['F_f'].append(Fe_ema[s].copy());rec[s]['T_f'].append(Te_ema[s].copy())
            if phase[s]=='appr':
                if step%10==0:J[s]=sim_jac6(m,d,r,s)
                Jj=J.get(s,sim_jac6(m,d,r,s));J3=Jj[:3];JJt=J3@J3.T+0.05*np.eye(3)
                dq=J3.T@np.linalg.inv(JJt)@np.array([0.0005,0,0]);dq=np.clip(dq,-0.015,0.015)
                qt=gq(m,d,s)+dq
                if Fmag>=0.3 and len(Fb[s])>=10:
                    phase[s]='admit';adm[s].reset();Tref[s]=Tu.copy();cs[s]=0;lost[s]=0
                    print(f"\n  [{step:4d}] {s}: CONTACT -> ADMIT  Fz={Fe_ema[s][2]:.1f}N",flush=True)
                    continue
                sq(m,d,r,qt,s)
            else:
                cs[s]+=1
                if Fmag<0.3:lost[s]+=1
                else:lost[s]=0
                Fe6=np.array([Fe_ema[s][0],Fe_ema[s][1],Fe_ema[s][2]-F_TARGET])
                te6=Te_ema[s]-T_TARGET
                dp,dr=adm[s].step(np.concatenate([Fe6,te6]),DT)
                rec[s].setdefault('dp',[]).append(dp.copy());rec[s].setdefault('dr',[]).append(dr.copy())
                if step%15==0:J[s]=sim_jac6(m,d,r,s)
                dp_w=Tref[s][:3,:3]@dp;dr_w=Tref[s][:3,:3]@dr
                Jj=J.get(s,sim_jac6(m,d,r,s));JJt=Jj@Jj.T+0.1*np.eye(6)
                dq=Jj.T@np.linalg.inv(JJt)@np.concatenate([dp_w,0.6*dr_w]);dq=np.clip(dq,-0.02,0.02)
                qt=gq(m,d,s)+dq;sctrl(m,d,r,qt,s)
                if lost[s]>2000 or cs[s]>6000:done[s]=True
        if step%200==0:
            st=[f"{x[0]}={phase[x][:5]} Fz={Fe_ema[x][2]if Fe_ema[x]is not None else 0:.0f}" for x in['left','right']]
            print(f"  [{step:4d}] {' | '.join(st)}",end="\r")
        if step%3==0:viewer.sync()
        if all(done.values()):break
    viewer.close()

    # Save
    print("\nSaving...")
    os.makedirs('output',exist_ok=True)
    for s in['left','right']:
        t=np.array(rec[s]['t'])
        if len(t)<2:continue
        Fa=np.array(rec[s]['F']);Ta=np.array(rec[s]['T']);pa=np.array(rec[s]['pos']);ra=np.degrees(np.array(rec[s]['rpy']))
        Ff=np.array(rec[s]['F_f']);Tf=np.array(rec[s]['T_f'])
        dp=np.array(rec[s].get('dp',[[0]*3]*len(t)));dr=np.array(rec[s].get('dr',[[0]*3]*len(t)))
        n=min(len(t),len(dp),len(dr))
        with open(f'output/admittance_{s}.csv','w',newline='')as f:
            w=csv.writer(f)
            w.writerow(['t','Fx','Fy','Fz','Tx','Ty','Tz','X','Y','Z','roll','pitch','yaw',
                        'Fx_f','Fy_f','Fz_f','Tx_f','Ty_f','Tz_f','dpX','dpY','dpZ','drX','drY','drZ'])
            for i in range(n):w.writerow([t[i],*Fa[i],*Ta[i],*pa[i],*ra[i],*Ff[i],*Tf[i],*dp[i],*dr[i]])

    fig,axes=plt.subplots(4,2,figsize=(18,20))
    for col,(s,lb)in enumerate([('left','Left'),('right','Right')]):
        t=np.array(rec[s]['t'])
        if len(t)<2:continue
        Fa=np.array(rec[s]['F']);Ta=np.array(rec[s]['T']);pa=np.array(rec[s]['pos']);ra=np.degrees(np.array(rec[s]['rpy']))
        Ff=np.array(rec[s]['F_f']);Tf=np.array(rec[s]['T_f'])
        dp=np.array(rec[s].get('dp',[[0]*3]*len(t)));dr=np.array(rec[s].get('dr',[[0]*3]*len(t)))
        ax=axes[0,col]
        ax.plot(t,Fa[:,0],'r',lw=0.5,alpha=0.3);ax.plot(t,Fa[:,1],'g',lw=0.5,alpha=0.3);ax.plot(t,Fa[:,2],'b',lw=0.5,alpha=0.3)
        ax.plot(t,Ff[:,0],'r',lw=1.5,label='Fx');ax.plot(t,Ff[:,1],'g',lw=1.5,label='Fy');ax.plot(t,Ff[:,2],'b',lw=1.5,label='Fz')
        ax.axhline(F_TARGET,color='b',ls='--',lw=0.8,alpha=0.5);ax.legend(fontsize=7);ax.set_ylabel('Force EE (N)');ax.set_title(f'{lb}: Force');ax.grid(alpha=0.3)
        ax=axes[1,col]
        ax.plot(t,Ta[:,0],'r',lw=0.5,alpha=0.3);ax.plot(t,Ta[:,1],'g',lw=0.5,alpha=0.3);ax.plot(t,Ta[:,2],'b',lw=0.5,alpha=0.3)
        ax.plot(t,Tf[:,0],'r',lw=1.5,label='Tx');ax.plot(t,Tf[:,1],'g',lw=1.5,label='Ty');ax.plot(t,Tf[:,2],'b',lw=1.5,label='Tz')
        ax.axhline(0,color='gray',lw=0.5);ax.legend(fontsize=7);ax.set_ylabel('Torque EE (Nm)');ax.set_title(f'{lb}: Torque');ax.grid(alpha=0.3)
        ax=axes[2,col]
        ax.plot(t,ra[:,0],'r',lw=1,label='roll');ax.plot(t,ra[:,1],'g',lw=1,label='pitch');ax.plot(t,ra[:,2],'b',lw=1,label='yaw')
        if len(dr)>0:n_=min(len(t),len(dr));ax.plot(t[:n_],np.degrees(dr[:n_,0]),'r:',lw=0.8);ax.plot(t[:n_],np.degrees(dr[:n_,1]),'g:',lw=0.8);ax.plot(t[:n_],np.degrees(dr[:n_,2]),'b:',lw=0.8)
        ax.legend(fontsize=6);ax.set_ylabel('RPY (deg)');ax.set_title(f'{lb}: Orientation');ax.grid(alpha=0.3)
        ax=axes[3,col]
        ax.plot(t,pa[:,0],'r',lw=1,label='X');ax.plot(t,pa[:,1],'g',lw=1,label='Y');ax.plot(t,pa[:,2],'b',lw=1,label='Z')
        if len(dp)>0:n_=min(len(t),len(dp));ax.plot(t[:n_],dp[:n_,2],'m',lw=1,ls=':',label='corr_Z')
        ax.legend(fontsize=7);ax.set_xlabel('Time (s)');ax.set_ylabel('Pos (m)');ax.set_title(f'{lb}: Position');ax.grid(alpha=0.3)
    fig.suptitle(f'v5 6-DOF Admittance (F_target={F_TARGET}N, free boxes)',fontsize=13);plt.tight_layout()
    plt.savefig('output/admittance_data.png',dpi=150);plt.savefig('output/admittance_data.pdf')
    print("Done. output/admittance_{left,right}.csv + admittance_data.{png,pdf}")

if __name__=="__main__":main()
