"""
直推记录: viewer仿真 + 六维力/位姿曲线
"""
import numpy as np, mujoco, mujoco.viewer, os, sys, csv, time
from scipy.spatial.transform import Rotation as R
from src.dual_arm_ik_v5 import DualArmIKV5
from src.alfa_interface_v5 import AlfaRobotInterfaceV5
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

DT = 0.002
PUSH_SPEED = 0.0005
MAX_STEPS = 1500

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
def sim_jac(m,d,r,s):
    eps=0.001;J=np.zeros((3,6))
    eid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,f'{s}_ee')
    q0=gq(m,d,s);sq(m,d,r,q0,s);mujoco.mj_forward(m,d);p0=d.site_xpos[eid].copy()
    for i in range(6):
        qp=q0.copy();qp[i]+=eps;sq(m,d,r,qp,s);mujoco.mj_forward(m,d)
        J[:,i]=(d.site_xpos[eid].copy()-p0)/eps
    sq(m,d,r,q0,s);mujoco.mj_forward(m,d)
    return J

print("Loading...")
m=mujoco.MjModel.from_xml_path('scene_v5.xml');d=mujoco.MjData(m);mujoco.mj_resetData(m,d)
r=AlfaRobotInterfaceV5(m,d);ik=DualArmIKV5()
q_home=np.array([0,np.pi/2,0,-np.pi/2,0,0]);orient=np.radians([0,90,0])
fwd=R.from_euler('xyz',orient).as_matrix()[:,2].copy()

viewer=mujoco.viewer.launch_passive(m,d)
viewer.cam.distance=4.5;viewer.cam.elevation=-20;viewer.cam.lookat[:]=[1.5,0,0.8]

r.apply_hybrid_target(dict(base_x=0,base_y=0,base_yaw=0,pitch=0,turn=0,updown=0.3,
    leftjoint1=0,leftjoint2=0,leftjoint3=0,leftjoint4=-np.pi/2,leftjoint5=0,leftjoint6=0,
    rightjoint1=0,rightjoint2=0,rightjoint3=0,rightjoint4=-np.pi/2,rightjoint5=0,rightjoint6=0,
    right_suction=0,left_suction=0))
mujoco.mj_forward(m,d)
for _ in range(200):
    if not viewer.is_running():sys.exit(0)
    mujoco.mj_step(m,d);viewer.sync()

q_above={'left':ik.solve_left_pose(([0.90,0.29,0.50],orient),q_init=q_home),
         'right':ik.solve_right_pose(([0.90,-0.29,0.50],orient),q_init=q_home)}
q_pre={'left':ik.solve_left_pose(([0.94,0.29,0.30],orient),q_init=q_above['left']),
       'right':ik.solve_right_pose(([0.94,-0.29,0.30],orient),q_init=q_above['right'])}

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

print(f"Pushing {PUSH_SPEED*1000:.1f}mm/step x {MAX_STEPS} steps...")

rec={s:{'t':[],'F':[],'T':[],'pos':[],'rpy':[]}for s in['left','right']}
J={}

for step in range(MAX_STEPS):
    if not viewer.is_running():break
    for jn in['base_x','base_y','base_yaw']:
        qadr=r.jnt_qpos_adrs.get(jn)
        if qadr is not None:d.qpos[qadr]=0.0;d.qvel[r.jnt_qvel_adrs[jn]]=0.0
    mujoco.mj_step(m,d);mujoco.mj_forward(m,d)

    for s in['left','right']:
        Fw,Fe,Te=gft(m,d,s);Tu=Tee(m,d,s);rpy=R.from_matrix(Tu[:3,:3]).as_euler('xyz')
        rec[s]['t'].append(step*DT);rec[s]['F'].append(Fe.copy());rec[s]['T'].append(Te.copy())
        rec[s]['pos'].append(Tu[:3,3].copy());rec[s]['rpy'].append(rpy)

        if step%10==0:J[s]=sim_jac(m,d,r,s)
        Jj=J.get(s,sim_jac(m,d,r,s));J3=Jj[:3];JJt=J3@J3.T+0.05*np.eye(3)
        dq=J3.T@np.linalg.inv(JJt)@np.array([PUSH_SPEED,0,0]);dq=np.clip(dq,-0.015,0.015)
        qt=gq(m,d,s)+dq;sq(m,d,r,qt,s)

    if step%3==0:viewer.sync()
    if step%500==0:
        fb=rec['left']['F'][-1];tb=rec['left']['T'][-1]
        print(f"  [{step:4d}] L: Fz={fb[2]:.0f}N |T|={np.linalg.norm(tb):.0f}Nm",flush=True)

viewer.close()

# ---- PLOTS (六维力 + 六维位姿) ----
os.makedirs('output',exist_ok=True)
fig,axes=plt.subplots(4,2,figsize=(18,20))
for col,(s,lb)in enumerate([('left','Left'),('right','Right')]):
    t=np.array(rec[s]['t']);Fa=np.array(rec[s]['F']);Ta=np.array(rec[s]['T'])
    pa=np.array(rec[s]['pos']);ra=np.degrees(np.array(rec[s]['rpy']))

    # EMA滤波 (α=0.1)
    def ema(x,alpha=0.1):
        y=np.zeros_like(x)
        y[0]=x[0]
        for i in range(1,len(x)):y[i]=alpha*x[i]+(1-alpha)*y[i-1]
        return y
    Fa_f=np.array([ema(Fa[:,0]),ema(Fa[:,1]),ema(Fa[:,2])]).T
    Ta_f=np.array([ema(Ta[:,0]),ema(Ta[:,1]),ema(Ta[:,2])]).T

    with open(f'output/push_{s}.csv','w',newline='')as f:
        w=csv.writer(f)
        w.writerow(['t','Fx','Fy','Fz','Tx','Ty','Tz','X','Y','Z','roll_deg','pitch_deg','yaw_deg',
                    'Fx_f','Fy_f','Fz_f','Tx_f','Ty_f','Tz_f'])
        for i in range(len(t)):w.writerow([t[i],*Fa[i],*Ta[i],*pa[i],*ra[i],*Fa_f[i],*Ta_f[i]])

    ax=axes[0,col]
    ax.plot(t,Fa[:,0],'r',lw=0.5,alpha=0.3);ax.plot(t,Fa[:,1],'g',lw=0.5,alpha=0.3);ax.plot(t,Fa[:,2],'b',lw=0.5,alpha=0.3)
    ax.plot(t,Fa_f[:,0],'r',lw=1.5,label='Fx');ax.plot(t,Fa_f[:,1],'g',lw=1.5,label='Fy');ax.plot(t,Fa_f[:,2],'b',lw=1.5,label='Fz')
    ax.set_ylabel('Force EE (N)');ax.set_title(f'{lb}: Force (thin=raw, thick=EMA)');ax.grid(alpha=0.3);ax.legend(fontsize=7)

    ax=axes[1,col]
    ax.plot(t,Ta[:,0],'r',lw=0.5,alpha=0.3);ax.plot(t,Ta[:,1],'g',lw=0.5,alpha=0.3);ax.plot(t,Ta[:,2],'b',lw=0.5,alpha=0.3)
    ax.plot(t,Ta_f[:,0],'r',lw=1.5,label='Tx');ax.plot(t,Ta_f[:,1],'g',lw=1.5,label='Ty');ax.plot(t,Ta_f[:,2],'b',lw=1.5,label='Tz')
    ax.set_ylabel('Torque EE (Nm)');ax.set_title(f'{lb}: Torque');ax.grid(alpha=0.3);ax.legend(fontsize=7)

    ax=axes[2,col];ax.plot(t,pa[:,0],'r',lw=1,label='X');ax.plot(t,pa[:,1],'g',lw=1,label='Y');ax.plot(t,pa[:,2],'b',lw=1,label='Z')
    ax.set_ylabel('Pos (m)');ax.set_title(f'{lb}: Position');ax.grid(alpha=0.3);ax.legend(fontsize=7)

    ax=axes[3,col];ax.plot(t,ra[:,0],'r',lw=1,label='roll');ax.plot(t,ra[:,1],'g',lw=1,label='pitch');ax.plot(t,ra[:,2],'b',lw=1,label='yaw')
    ax.set_xlabel('Time (s)');ax.set_ylabel('RPY (deg)');ax.set_title(f'{lb}: Orientation');ax.grid(alpha=0.3);ax.legend(fontsize=7)

fig.suptitle('Push Test: 6-DOF Force/Torque + 6-DOF Pose',fontsize=13);plt.tight_layout()
plt.savefig('output/push_test.png',dpi=150);plt.savefig('output/push_test.pdf')

for s in['left','right']:
    Fa=np.array(rec[s]['F']);Ta=np.array(rec[s]['T']);ra=np.degrees(np.array(rec[s]['rpy']))
    ic=np.where(np.abs(Fa[:,2])>0.5)[0]
    if len(ic)>0:
        print(f"{s}: {len(ic)} pts. Fz=[{Fa[ic,2].min():.0f},{Fa[ic,2].max():.0f}]N "
              f"|T|=[{np.linalg.norm(Ta[ic],axis=1).min():.0f},{np.linalg.norm(Ta[ic],axis=1).max():.0f}]Nm "
              f"rpy: roll[{ra[ic,0].min():.1f},{ra[ic,0].max():.1f}] "
              f"pitch[{ra[ic,1].min():.1f},{ra[ic,1].max():.1f}] "
              f"yaw[{ra[ic,2].min():.1f},{ra[ic,2].max():.1f}]")
print("Saved: output/push_{left,right}.csv + push_test.{png,pdf}")
