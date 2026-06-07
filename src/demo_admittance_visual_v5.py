"""
6-DOF Cartesian admittance (v5: 左臂 6-DOF 全铰链)
使用 IK 将笛卡尔空间的力/力矩误差映射到关节空间
F_x -> EE X位移, tau_y -> EE绕Y旋转, tau_z -> EE绕Z旋转
Data saved to ./output/ at exit.
"""
import math, os, numpy as np, mujoco, mujoco.viewer
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from left_arm_ik_v5 import LeftArmIKV5
from alfa_interface_v5 import AlfaRobotInterfaceV5


class Adm1D:
    def __init__(self, M=0.5, D=60.0, vmax=0.005, cmax=0.10):
        self.M = M; self.D = D; self.vmax = vmax; self.cmax = cmax; self.reset()
    def reset(self): self.vel = 0.0; self.pos_corr = 0.0
    def step(self, e, dt):
        a = (e - self.D * self.vel) / self.M; self.vel += a * dt
        self.vel = np.clip(self.vel, -self.vmax, self.vmax)
        dp = self.vel * dt; p = np.clip(self.pos_corr + dp, -self.cmax, self.cmax)
        dp = p - self.pos_corr; self.pos_corr = p; return dp


def get_site_pose(m, d, n):
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, n)
    T = np.eye(4); T[:3, 3] = d.site_xpos[sid]; T[:3, :3] = d.site_xmat[sid].reshape(3, 3)
    return T


def set_joint(m, d, robot, jn, v):
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
    d.qpos[m.jnt_qposadr[jid]] = v; d.qvel[m.jnt_dofadr[jid]] = 0.0
    aid = robot.actuator_ids.get(jn, -1)
    if aid >= 0: d.ctrl[aid] = v


def get_ik_q(model, data):
    names = ["leftjoint1", "leftjoint2", "leftjoint3",
             "leftjoint4", "leftjoint5", "leftjoint6"]
    return np.array([data.qpos[model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]] for n in names])


def apply_ik_q(model, data, robot, q):
    for i, n in enumerate(["leftjoint1", "leftjoint2", "leftjoint3",
                           "leftjoint4", "leftjoint5", "leftjoint6"]):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
        data.qpos[model.jnt_qposadr[jid]] = q[i]
        data.qvel[model.jnt_dofadr[jid]] = 0.0
        aid = robot.actuator_ids.get(n, -1)
        if aid >= 0: data.ctrl[aid] = q[i]


def get_ft(m, d, ee_w, ee_R):
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "left_suction_col")
    Fw = np.zeros(3); Tw = np.zeros(3)
    for i in range(d.ncon):
        c = d.contact[i]; our_is_geom2 = None
        if c.geom2 == gid: our_is_geom2 = True
        elif c.geom1 == gid: our_is_geom2 = False
        else: continue
        obn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom1 if our_is_geom2 else c.geom2]) or ""
        if not obn.startswith("box_"): continue
        cf = np.zeros(6); mujoco.mj_contactForce(m, d, i, cf)
        frame = c.frame.reshape(3, 3); F2 = frame @ cf[:3]
        Fi = F2 if our_is_geom2 else -F2
        Fw += Fi; Tw += np.cross(c.pos - ee_w, Fi)
    return Fw, ee_R.T @ Fw, ee_R.T @ Tw


def main():
    Fd = -5.0; DT = 0.002

    model = mujoco.MjModel.from_xml_path("scene_v5.xml")
    data = mujoco.MjData(model); mujoco.mj_resetData(model, data)
    robot = AlfaRobotInterfaceV5(model, data); ik = LeftArmIKV5()

    # 3 个导纳控制器: F_x, tau_y, tau_z
    adm_f  = Adm1D(M=0.05, D=15.0, vmax=0.003, cmax=0.08)
    adm_ty = Adm1D(M=0.01, D=5.0,  vmax=0.08, cmax=0.40)
    adm_tz = Adm1D(M=0.01, D=5.0,  vmax=0.08, cmax=0.40)
    F_avg = 0.0

    # ---- Data buffers ----
    t_hist = []; Fx_hist = []; Fy_hist = []; Fz_hist = []
    tx_hist = []; ty_hist = []; tz_hist = []
    px_hist = []; py_hist = []; pz_hist = []
    rx_hist = []; ry_hist = []; rz_hist = []
    q1_hist = []; q5_hist = []; q3_hist = []

    print("[1] init v5 (Cartesian admittance via IK)")
    robot.apply_hybrid_target({
        "base_x": 0.0, "base_y": 0.0, "base_yaw": 0.0,
        "pitch": 0.13, "turn": 0.0, "updown": 0.3,
        "rightjoint1": 0.0, "rightjoint2": math.pi/2, "rightjoint3": 0.0,
        "rightjoint4": -math.pi/2, "rightjoint5": 0.0, "rightjoint6": 0.0,
        "leftjoint1": 0.0, "leftjoint2": math.pi/2, "leftjoint3": 0.0,
        "leftjoint4": -math.pi/2, "leftjoint5": 0.0, "leftjoint6": 0.0,
        "right_suction": 0.0, "left_suction": 0.0,
    })
    mujoco.mj_forward(model, data)
    for _ in range(500): mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    T_ee = get_site_pose(model, data, "left_ee")
    ee_w = T_ee[:3, 3]; fwd_w = T_ee[:3, :3][:, 0]
    misalign = np.degrees(np.arccos(np.clip(np.dot(fwd_w, [1., 0., 0.]), -1, 1)))
    print(f"    EE: {np.round(ee_w, 3)}  fwd_align: {misalign:.1f}deg")

    # 找最近的箱子
    box_names = [f"box_r{r}_c{c}_l{l}" for r in range(3) for c in range(2) for l in range(3)]
    best = None; bd = np.inf; bc = None
    for bn in box_names:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bn)
        if bid < 0: continue
        c = data.xpos[bid]; v = c - ee_w; a = np.dot(v, fwd_w)
        if a > 0.05 and np.linalg.norm(v - a * fwd_w) < 0.5:
            if a < bd: bd = a; best = bn; bc = c
    if best is None:
        for bn in box_names:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bn)
            if bid < 0: continue
            d_ = np.linalg.norm(data.xpos[bid] - ee_w)
            if d_ < bd: bd = d_; best = bn; bc = data.xpos[bid]
    box_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, best)
    print(f"    目标箱子: {best}, 距离: {bd:.2f}m")

    # 驱动机器人靠近箱子
    box_gid = -1
    for i in range(model.ngeom):
        if model.geom_bodyid[i] == box_bid and model.geom_type[i] == 6:
            box_gid = i; break
    half = abs(np.dot(model.geom_size[box_gid], fwd_w)) if box_gid >= 0 else 0.48
    base_need = max(0, (bd - half) - 0.25)
    if base_need > 0.01:
        print(f"[2] base_x +{base_need:.2f}m (face={(bd - half) * 1000:.0f}mm)")
        for bx in np.linspace(0.0, base_need, 10):
            set_joint(model, data, robot, "base_x", bx)
            set_joint(model, data, robot, "base_y", 0)
            set_joint(model, data, robot, "base_yaw", 0)
            mujoco.mj_forward(model, data)
            for _ in range(5): mujoco.mj_step(model, data)
        for _ in range(200): mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)

    # 获取 updown_link 世界位姿用于坐标变换
    def get_updown_pose():
        uid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "updown_link")
        T = np.eye(4)
        T[:3, 3] = data.xpos[uid]; T[:3, :3] = data.xmat[uid].reshape(3, 3)
        return T

    # 用于记录的关节 ID
    j2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "leftjoint2")
    j3_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "leftjoint3")
    j5_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "leftjoint5")

    phase = "approach"; contact = False; cstep = 0
    T_target = None  # IK目标位姿 (updown frame)
    print(f"\n[3] Cartesian admittance F_d={Fd}N  [v5]")
    print(f"    {'Step':>5s} {'phase':>7s} {'F_x':>7s} {'|F|':>6s} {'ty':>7s} {'tz':>7s} "
          f"{'q2':>7s} {'q5':>7s} {'q3':>7s}")
    print(f"    {'-' * 80}")

    RENDER_EVERY = 3
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 3.5; viewer.cam.elevation = -20; viewer.cam.lookat[:] = [1.0, 0.3, 1.0]
        for sim_step in range(100000):
            if not viewer.is_running(): break
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)

            T_ee = get_site_pose(model, data, "left_ee")
            ee_w = T_ee[:3, 3]; R_ee_w = T_ee[:3, :3]
            _, F_ee, T_ee_ft = get_ft(model, data, ee_w, R_ee_w)
            Fmag = np.linalg.norm(F_ee)

            if phase == "approach" and Fmag > 0.5:
                phase = "regulate"; contact = True
                adm_f.reset(); adm_ty.reset(); adm_tz.reset()
                # 记录当前 EE 位姿作为目标基准
                T_w_u = get_updown_pose()
                T_ee_u = np.linalg.inv(T_w_u) @ T_ee
                T_target = T_ee_u.copy()
                print(f"\n    >>> contact! F={Fmag:.1f}N")

            if phase == "approach":
                # 笛卡尔逼近: 沿吸盘朝前方向 (EE frame X-axis) 推进
                fwd_ee = R_ee_w[:, 0]
                pos_target = ee_w + fwd_ee * 0.0008  # 0.4mm per step
                q_cur = get_ik_q(model, data)
                # 转 updown 系
                T_w_u = get_updown_pose()
                pos_u = (np.linalg.inv(T_w_u) @ np.append(pos_target, 1.0))[:3]
                q_tgt = ik.solve_position(pos_u, q_init=q_cur, tol_pos=5e-3, max_iter=40)
                if not np.any(np.isnan(q_tgt)):
                    apply_ik_q(model, data, robot, q_tgt)
            else:
                # 笛卡尔导纳: F_x -> EE沿X位移, tau_y -> EE绕Y旋转, tau_z -> EE绕Z旋转
                F_avg = 0.95 * F_avg + 0.05 * F_ee[0]
                if Fmag > 0.3:
                    dx  = adm_f.step(F_avg - Fd, DT)
                    dry = adm_ty.step(T_ee_ft[1], DT)
                    drz = adm_tz.step(-T_ee_ft[2], DT)
                else:
                    # 脱离接触: 沿接近方向推进重新接触
                    fwd_ee = R_ee_w[:, 0]
                    T_w_u = get_updown_pose()
                    T_target[:3, 3] += (np.linalg.inv(T_w_u)[:3, :3] @ fwd_ee) * 0.0002
                    dx = 0; dry = 0; drz = 0
                    adm_ty.vel *= 0.98; adm_tz.vel *= 0.98

                # 更新目标位姿
                T_w_u = get_updown_pose()
                if T_target is None:
                    T_ee_u = np.linalg.inv(T_w_u) @ T_ee
                    T_target = T_ee_u.copy()

                # 位置调整 (沿EE的X轴)
                ee_x_u = (np.linalg.inv(T_w_u)[:3, :3] @ R_ee_w[:, 0])
                T_target[:3, 3] += ee_x_u * dx

                # 姿态调整 (绕EE的Y和Z轴)
                Ry = R.from_euler('y', dry).as_matrix()
                Rz = R.from_euler('z', drz).as_matrix()
                T_target[:3, :3] = T_target[:3, :3] @ Ry @ Rz

                # IK 求解
                q_cur = get_ik_q(model, data)
                q_tgt = ik.solve(T_target, q_init=q_cur, tol_pos=5e-3, tol_rot=5e-2, max_iter=40)
                if not np.any(np.isnan(q_tgt)):
                    apply_ik_q(model, data, robot, q_tgt)

            # Record every step
            t_hist.append(sim_step * model.opt.timestep)
            Fx_hist.append(F_ee[0]); Fy_hist.append(F_ee[1]); Fz_hist.append(F_ee[2])
            tx_hist.append(T_ee_ft[0]); ty_hist.append(T_ee_ft[1]); tz_hist.append(T_ee_ft[2])
            px_hist.append(ee_w[0]); py_hist.append(ee_w[1]); pz_hist.append(ee_w[2])
            euler = R.from_matrix(R_ee_w).as_euler('xyz')
            rx_hist.append(np.degrees(euler[0])); ry_hist.append(np.degrees(euler[1])); rz_hist.append(np.degrees(euler[2]))
            q1_hist.append(data.qpos[model.jnt_qposadr[j2_id]])
            q5_hist.append(data.qpos[model.jnt_qposadr[j5_id]])
            q3_hist.append(data.qpos[model.jnt_qposadr[j3_id]])

            if cstep % 500 == 0:
                print(f"    {cstep:>5d} {phase:>7s} {F_ee[0]:>7.1f} {Fmag:>6.1f} "
                      f"{T_ee_ft[1]:>7.2f} {T_ee_ft[2]:>7.2f} "
                      f"{data.qpos[model.jnt_qposadr[j2_id]]:>7.4f} "
                      f"{data.qpos[model.jnt_qposadr[j5_id]]:>7.3f} "
                      f"{data.qpos[model.jnt_qposadr[j3_id]]:>7.3f}")

            cstep += 1
            if sim_step % RENDER_EVERY == 0: viewer.sync()

    # ---- Downsample and save plots ----
    os.makedirs("output", exist_ok=True)
    step = max(1, len(t_hist) // 200)
    t = np.array(t_hist[::step])
    Fx = np.array(Fx_hist)[::step]; Fy = np.array(Fy_hist)[::step]; Fz = np.array(Fz_hist)[::step]
    tx = np.array(tx_hist)[::step]; ty = np.array(ty_hist)[::step]; tz = np.array(tz_hist)[::step]
    px = np.array(px_hist)[::step]; py = np.array(py_hist)[::step]; pz = np.array(pz_hist)[::step]
    rx = np.array(rx_hist)[::step]; ry = np.array(ry_hist)[::step]; rz = np.array(rz_hist)[::step]

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    (ax1, ax2, ax3, ax4) = axes
    lw = 1.2

    ax1.plot(t, Fx, 'r-', lw=lw, label='F_x'); ax1.plot(t, Fy, 'g-', lw=lw, label='F_y'); ax1.plot(t, Fz, 'b-', lw=lw, label='F_z')
    ax1.axhline(Fd, color='red', ls='--', lw=1, alpha=0.4, label=f'F_d={Fd}N')
    ax1.set_ylabel('Force (N)'); ax1.legend(loc='upper right', ncol=4, fontsize=8)
    ax1.grid(True, alpha=0.3); ax1.set_title('End-Effector Force (EE frame) [v5]')

    ax2.plot(t, tx, 'r-', lw=lw, label='tau_x'); ax2.plot(t, ty, 'g-', lw=lw, label='tau_y'); ax2.plot(t, tz, 'b-', lw=lw, label='tau_z')
    ax2.axhline(0, color='gray', lw=0.5)
    ax2.set_ylabel('Torque (Nm)'); ax2.legend(loc='upper right', ncol=3, fontsize=8)
    ax2.grid(True, alpha=0.3); ax2.set_title('End-Effector Torque (EE frame, target: 0)')

    ax3.plot(t, px, 'r-', lw=lw, label='X'); ax3.plot(t, py, 'g-', lw=lw, label='Y'); ax3.plot(t, pz, 'b-', lw=lw, label='Z')
    ax3.set_ylabel('Position (m)'); ax3.legend(loc='upper right', ncol=3, fontsize=8)
    ax3.grid(True, alpha=0.3); ax3.set_title('End-Effector Position (world)')

    ax4.plot(t, rx, 'r-', lw=lw, label='Roll'); ax4.plot(t, ry, 'g-', lw=lw, label='Pitch'); ax4.plot(t, rz, 'b-', lw=lw, label='Yaw')
    ax4.set_ylabel('Angle (deg)'); ax4.set_xlabel('Time (s)')
    ax4.legend(loc='upper right', ncol=3, fontsize=8)
    ax4.grid(True, alpha=0.3); ax4.set_title('End-Effector Orientation (world, euler xyz)')

    fig.suptitle(f'Cartesian Admittance Control (v5)  |  F_d={Fd}N  |  tau_d=0', fontsize=13)
    plt.tight_layout()
    plt.savefig("output/admittance_6dof_v5.png", dpi=150)
    plt.savefig("output/admittance_6dof_v5.pdf")
    print(f"\nPlots saved to output/admittance_6dof_v5.{{png,pdf}}")

    T_ee = get_site_pose(model, data, "left_ee")
    _, F_ee, T_ee_ft = get_ft(model, data, T_ee[:3, 3], T_ee[:3, :3])
    print(f"Final: F={np.round(F_ee, 1)}N tau={np.round(T_ee_ft, 2)}Nm "
          f"|F|={np.linalg.norm(F_ee):.1f}N {'CONTACT' if contact else ''}")


if __name__ == "__main__":
    main()
