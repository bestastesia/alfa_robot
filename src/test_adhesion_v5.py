"""Test robot_v2 adhesion-only grasping (no weld)"""
import numpy as np, mujoco
from scipy.spatial.transform import Rotation as R
from src.alfa_interface_v5 import AlfaRobotInterfaceV5
from src.dual_arm_ik_v5 import DualArmIKV5
from collections import deque

DT = 0.002; F_TARGET = -3.0; T_TARGET = np.zeros(3)

class Adm6D:
    def __init__(self):
        self.M = np.array([0.1, 0.1, 0.1, 0.06, 0.06, 0.06])
        self.D = np.array([20.0, 20.0, 20.0, 25.0, 25.0, 25.0])
        self.vmax = np.array([0.006, 0.006, 0.008, 0.10, 0.10, 0.10])
        self.cmax = np.array([0.02, 0.02, 0.015, 0.10, 0.10, 0.10])
        self.reset()
    def reset(self): self.vel = np.zeros(6); self.corr = np.zeros(6)
    def step(self, e, dt):
        self.vel += (e - self.D * self.vel) / self.M * dt
        self.vel = np.clip(self.vel, -self.vmax, self.vmax)
        self.corr += self.vel * dt
        self.corr = np.clip(self.corr, -self.cmax, self.cmax)
        return self.corr[:3].copy(), self.corr[3:].copy()

def T_s(m, d, n):
    ii = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, n)
    T = np.eye(4); T[:3, 3] = d.site_xpos[ii]; T[:3, :3] = d.site_xmat[ii].reshape(3, 3)
    return T

def T_b(m, d, n):
    ii = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    T = np.eye(4); T[:3, 3] = d.xpos[ii]; T[:3, :3] = d.xmat[ii].reshape(3, 3)
    return T

def Tee(m, d, s):
    return np.linalg.inv(T_b(m, d, "updown_link")) @ T_s(m, d, f"{s}_ee")

def gq(m, d, s):
    return np.array([d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{s}joint{i+1}")]] for i in range(6)])

def sq(m, d, r, q, s):
    for i in range(6):
        n = f"{s}joint{i+1}"; jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        d.qpos[m.jnt_qposadr[jid]] = q[i]; d.qvel[m.jnt_dofadr[jid]] = 0.0
        aid = r.actuator_ids.get(n, -1)
        if aid >= 0: d.ctrl[aid] = q[i]

def sctrl(m, d, r, q, s):
    for i in range(6):
        aid = r.actuator_ids.get(f"{s}joint{i+1}", -1)
        if aid >= 0: d.ctrl[aid] = q[i]

def _set_qpos_only(m, d, s, q):
    for i in range(6):
        n = f"{s}joint{i+1}"; jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        d.qpos[m.jnt_qposadr[jid]] = q[i]; d.qvel[m.jnt_dofadr[jid]] = 0.0

def sim_jacobian_3dof(m, d, r, s):
    eps = 0.001; J = np.zeros((3, 6))
    eid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{s}_ee")
    q0 = gq(m, d, s)
    _set_qpos_only(m, d, s, q0); mujoco.mj_forward(m, d)
    p0 = d.site_xpos[eid].copy()
    for i in range(6):
        qp = q0.copy(); qp[i] += eps
        _set_qpos_only(m, d, s, qp); mujoco.mj_forward(m, d)
        J[:, i] = (d.site_xpos[eid].copy() - p0) / eps
    _set_qpos_only(m, d, s, q0); mujoco.mj_forward(m, d)
    return J

def sim_jacobian_6dof(m, d, r, s):
    eps = 0.001; J = np.zeros((6, 6))
    eid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{s}_ee")
    q0 = gq(m, d, s)
    _set_qpos_only(m, d, s, q0); mujoco.mj_forward(m, d)
    p0 = d.site_xpos[eid].copy(); R0 = d.site_xmat[eid].reshape(3, 3).copy()
    for i in range(6):
        qp = q0.copy(); qp[i] += eps
        _set_qpos_only(m, d, s, qp); mujoco.mj_forward(m, d)
        p1 = d.site_xpos[eid].copy(); R1 = d.site_xmat[eid].reshape(3, 3).copy()
        J[:3, i] = (p1 - p0) / eps
        Rdiff = R1 @ R0.T; J[3:6, i] = R.from_matrix(Rdiff).as_rotvec() / eps
    _set_qpos_only(m, d, s, q0); mujoco.mj_forward(m, d)
    return J

def gft(m, d, s):
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{s}_suction_col")
    Tee_ = T_s(m, d, f"{s}_ee"); ep = Tee_[:3, 3]; eR = Tee_[:3, :3]
    Fw = np.zeros(3); Tw = np.zeros(3); bn = None
    for i in range(d.ncon):
        c = d.contact[i]
        if c.geom2 == gid: o2 = True
        elif c.geom1 == gid: o2 = False
        else: continue
        oi = c.geom1 if o2 else c.geom2
        on = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[oi]) or ""
        if not on.startswith("box_"): continue
        if bn is None: bn = on
        cf = np.zeros(6); mujoco.mj_contactForce(m, d, i, cf)
        fr = c.frame.reshape(3, 3)
        Fi = fr @ cf[:3] if o2 else -fr @ cf[:3]
        Fw += Fi; Tw += np.cross(c.pos - ep, Fi)
    return Fw, eR.T @ Fw, eR.T @ Tw, bn

def sstep(t): return 3*t*t - 2*t*t*t

print("Loading...")
m = mujoco.MjModel.from_xml_path("scene_v5.xml")
d = mujoco.MjData(m); mujoco.mj_resetData(m, d)
r = AlfaRobotInterfaceV5(m, d); ik = DualArmIKV5()
orient = np.radians([0, 90, 0])
q_home = np.array([0, np.pi/2, 0, -np.pi/2, 0, 0])

# Init
r.apply_hybrid_target(dict(
    base_x=0, base_y=0, base_yaw=0, pitch=0, turn=0, updown=0.3,
    leftjoint1=0, leftjoint2=0, leftjoint3=0, leftjoint4=-np.pi/2, leftjoint5=0, leftjoint6=0,
    rightjoint1=0, rightjoint2=0, rightjoint3=0, rightjoint4=-np.pi/2, rightjoint5=0, rightjoint6=0,
    right_suction=0, left_suction=0))
mujoco.mj_forward(m, d)
for _ in range(200): mujoco.mj_step(m, d)

# IK + Path
t_pre_L = ([0.88, 0.29, 0.30], orient); t_pre_R = ([0.88, -0.29, 0.30], orient)
q_pre = {}
for s, tp in [("left", t_pre_L), ("right", t_pre_R)]:
    q_pre[s] = ik.solve_left_pose(tp, q_init=q_home) if s == "left" else ik.solve_right_pose(tp, q_init=q_home)

q0 = {s: gq(m, d, s) for s in ["left", "right"]}
for i in range(200):
    a = sstep(i/199)
    for s in ["left", "right"]: sq(m, d, r, q0[s] + a*(q_pre[s] - q0[s]), s)
    mujoco.mj_step(m, d)
for _ in range(100): mujoco.mj_step(m, d)
mujoco.mj_forward(m, d)

for s in ["left", "right"]:
    sc_gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{s}_suction_col")
    print(f"{s}: SC={np.round(d.geom_xpos[sc_gid], 3)}")

# Main loop: approach + admittance + adhesion (robot_v2 logic, no grasp weld)
adm = {s: Adm6D() for s in ["left", "right"]}
phase = {s: "appr" for s in ["left", "right"]}
done = {s: False for s in ["left", "right"]}
suctioned = {s: False for s in ["left", "right"]}
cs = {s: 0 for s in ["left", "right"]}
lost_count = {s: 0 for s in ["left", "right"]}
Fb = {s: deque(maxlen=5) for s in ["left", "right"]}
bn = {}
J3_cache = {}; J6_cache = {}

print("\n=== Approach + 6D Admittance + Adhesion ===")
for step in range(8000):
    for jn in ["base_x", "base_y", "base_yaw"]:
        qadr = r.jnt_qpos_adrs.get(jn)
        if qadr is not None: d.qpos[qadr] = 0.0; d.qvel[r.jnt_qvel_adrs[jn]] = 0.0
    mujoco.mj_step(m, d); mujoco.mj_forward(m, d)

    for s in ["left", "right"]:
        if done[s]: continue
        Fw, Fe, Te, boxn = gft(m, d, s)
        Fb[s].append(Fe.copy()); Ff = np.mean(np.array(Fb[s]), axis=0)
        Fmag = np.linalg.norm(Ff); Tmag = np.linalg.norm(Te)
        inst_mag = np.linalg.norm(Fe)

        if phase[s] == "admit" and Fmag < 0.15: lost_count[s] += 1
        elif Fmag >= 0.2: lost_count[s] = 0

        if lost_count[s] > 120:
            print(f"  [{step:4d}] {s}: LOST, backing up", flush=True)
            ee_pose = T_s(m, d, f"{s}_ee")
            back_w = -ee_pose[:3, :3][:, 2]
            dx_w = back_w * 0.002
            Jj_back = sim_jacobian_3dof(m, d, r, s)
            JJt = Jj_back @ Jj_back.T + 0.02 * np.eye(3)
            dq_back = Jj_back.T @ np.linalg.inv(JJt) @ dx_w
            qt = gq(m, d, s) + np.clip(dq_back, -0.03, 0.03)
            sq(m, d, r, qt, s)
            for _ in range(10): mujoco.mj_step(m, d)
            mujoco.mj_forward(m, d)
            phase[s] = "appr"; adm[s].reset(); lost_count[s] = 0; cs[s] = 0

        if phase[s] == "appr":
            ee_pose = T_s(m, d, f"{s}_ee")
            appr_w = ee_pose[:3, :3][:, 2]
            if inst_mag > 0.5:       step_mm = 0.00003
            elif inst_mag > 0.1:     step_mm = 0.00006
            elif inst_mag > 0.02:    step_mm = 0.00015
            else:                    step_mm = 0.0003
            dx_w = appr_w * step_mm

            if step < 5 or step % 15 == 0:
                J3_cache[s] = sim_jacobian_3dof(m, d, r, s)

            Jj = J3_cache.get(s, sim_jacobian_3dof(m, d, r, s))
            JJt = Jj @ Jj.T + 0.03 * np.eye(3)
            dq = Jj.T @ np.linalg.inv(JJt) @ dx_w
            dq = np.clip(dq, -0.03, 0.03)
            qt = gq(m, d, s) + dq
            sq(m, d, r, qt, s)

            if ((inst_mag > 0.4 or (Fmag >= 0.12 and len(Fb[s]) >= 3)) and len(Fb[s]) >= 2):
                phase[s] = "admit"; adm[s].reset(); cs[s] = 0; lost_count[s] = 0
                if boxn: bn[s] = boxn
                print(f"  [{step:4d}] {s}: CONTACT Fz={Ff[2]:.2f}N inst={inst_mag:.1f}N Tmag={Tmag:.3f} box={boxn}", flush=True)
        else:
            cs[s] += 1
            Fe6 = np.array([Fe[0], Fe[1], Fe[2] - F_TARGET])
            Te6 = Te - T_TARGET
            dp_ee, dr_ee = adm[s].step(np.concatenate([Fe6, Te6]), DT)

            if step < 5 or step % 25 == 0:
                J6_cache[s] = sim_jacobian_6dof(m, d, r, s)

            J_full = J6_cache.get(s, sim_jacobian_6dof(m, d, r, s))
            ee_R = T_s(m, d, f"{s}_ee")[:3, :3]
            dp_w = ee_R @ dp_ee; dr_w = ee_R @ dr_ee

            lam = np.array([0.1, 0.1, 0.1, 0.5, 0.5, 0.5])
            JJt = J_full @ J_full.T + np.diag(lam)
            v_ee = np.concatenate([dp_w, dr_w])
            try: dq = J_full.T @ np.linalg.inv(JJt) @ v_ee
            except np.linalg.LinAlgError: dq = J_full.T @ np.linalg.lstsq(JJt, v_ee, rcond=None)[0]
            dq = np.clip(dq, -0.015, 0.015)
            qt = gq(m, d, s) + dq
            sctrl(m, d, r, qt, s)

            if cs[s] % 100 == 0:
                print(f"  [{step:4d}] {s}[{cs[s]:4d}]: Fz={Fe[2]:.1f}N(err={Fe[2]-F_TARGET:.1f}) |T|={Tmag:.3f}Nm dp_z={dp_ee[2]*1000:.2f}mm dr={np.linalg.norm(dr_ee)*57.3:.1f}deg", flush=True)

            # robot_v2 logic: adhesion ON, fix weld OFF, NO grasp weld
            aligned = Tmag < 0.5
            force_ok = abs(Fe[2] - F_TARGET) < 4.0
            well_regulated = abs(Fe[2] - F_TARGET) < 1.5 and Tmag < 0.25
            if not suctioned[s] and ((cs[s] > 80 and Fmag >= 0.15 and force_ok and aligned) or
                (cs[s] > 40 and Fmag >= 0.2 and well_regulated)):
                suctioned[s] = True
                r.apply_hybrid_target({f"{s}_suction": 1.0})  # adhesion ON
                sctrl(m, d, r, gq(m, d, s), s)  # release forward pressure
                if bn.get(s):
                    wid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, f"fix_{bn[s]}")
                    if wid >= 0: d.eq_active[wid] = 0  # fix weld OFF
                print(f"  [{step:4d}] {s}: SUCTION (adhesion ON, fix OFF) Fz={Fe[2]:.1f}N |T|={Tmag:.3f}Nm box={bn.get(s)}", flush=True)
                done[s] = True

        if cs[s] > 3000 and not suctioned[s]:
            done[s] = True
            print(f"  [{step:4d}] {s}: TIMEOUT", flush=True)

    if all(done.values()): break

# Test lift (adhesion only, no weld)
print("\n=== Lifting (adhesion only) ===")
for s in ["left", "right"]:
    if bn.get(s) is None: continue
    box_bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn[s])
    box_init = d.xpos[box_bid].copy()
    print(f"{s} {bn[s]} initial: {np.round(box_init, 3)}")

    Tu = Tee(m, d, s)
    t_lift = (Tu[:3, 3] + np.array([0., 0., 0.25]), orient)
    q_lift = ik.solve_left_pose(t_lift, q_init=gq(m, d, s), max_iter=200) if s == "left" else ik.solve_right_pose(t_lift, q_init=gq(m, d, s), max_iter=200)
    q0_a = gq(m, d, s)
    for i in range(800):
        a = min(1.0, i/600); q_tgt = q0_a + a*(q_lift - q0_a)
        if i % 5 == 0: sctrl(m, d, r, q_tgt, s)
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)

    box_final = d.xpos[box_bid].copy()
    dz = box_final[2] - box_init[2]
    print(f"{s} {bn[s]} after lift: {np.round(box_final, 3)} (dZ={dz*1000:.0f}mm)")
    if dz > 0.05: print(f"  >>> SUCCESS: Box lifted by adhesion!")

# Release
print("\n=== Release ===")
for s in ["left", "right"]:
    if bn.get(s) is not None:
        r.apply_hybrid_target({f"{s}_suction": 0.0})
for _ in range(300): mujoco.mj_step(m, d)
mujoco.mj_forward(m, d)
for s in ["left", "right"]:
    if bn.get(s) is not None:
        box_bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bn[s])
        print(f"{s} {bn[s]} after release: {np.round(d.xpos[box_bid], 3)}")

print("\nDone!")
