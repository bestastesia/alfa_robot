"""
左臂导纳控制器 (v5: 6-DOF 全铰链左臂)
策略: IK 位置控制 (直接设 qpos) -> 慢速沿 EE->箱子方向推进 -> 导纳力控
v5 注意: leftjoint2=+pi/2 方向前伸 (不同于 v2 的 -pi/2)
"""
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R
from left_arm_ik_v5 import LeftArmIKV5
from alfa_env_v5 import AlfaEnvV5


class AdmittanceController:
    """1-DOF 导纳"""
    def __init__(self, M=0.3, D=80.0, vel_limit=0.003, max_corr=0.02):
        self.M = M; self.D = D; self.vel_limit = vel_limit; self.max_corr = max_corr
        self.reset()
    def reset(self): self.vel = 0.0; self.pos_corr = 0.0
    def step(self, Fm, Fd, dt):
        err = Fm - Fd; acc = (err - self.D * self.vel) / self.M; self.vel += acc * dt
        self.vel = np.clip(self.vel, -self.vel_limit, self.vel_limit)
        d = self.vel * dt; p = np.clip(self.pos_corr + d, -self.max_corr, self.max_corr)
        d = p - self.pos_corr; self.pos_corr = p; return d


# ---- 工具 ----
def get_body_pose(model, data, name):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    T = np.eye(4); T[:3, 3] = data.xpos[bid]; T[:3, :3] = data.xmat[bid].reshape(3, 3)
    return T


def get_ik_q(model, data):
    """获取左臂6个关节位置"""
    names = ["leftjoint1", "leftjoint2", "leftjoint3",
             "leftjoint4", "leftjoint5", "leftjoint6"]
    return np.array([data.qpos[model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]] for n in names])


def apply_ik_q(model, data, robot, q):
    """直接设 qpos + ctrl (运动学控制, 无PD跟踪误差)"""
    for i, n in enumerate(["leftjoint1", "leftjoint2", "leftjoint3",
                           "leftjoint4", "leftjoint5", "leftjoint6"]):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
        data.qpos[model.jnt_qposadr[jid]] = q[i]
        data.qvel[model.jnt_dofadr[jid]] = 0.0
        aid = robot.actuator_ids.get(n, -1)
        if aid >= 0: data.ctrl[aid] = q[i]


def get_box_contact_force(model, data, direction_world):
    """吸盘与箱子间接触力"""
    gids = []
    for gn in ["left_suction_col"]:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, gn)
        if gid >= 0: gids.append(gid)
    Fw = np.zeros(3)
    for i in range(data.ncon):
        c = data.contact[i]; our_is_geom2 = None
        for gid in gids:
            if c.geom1 == gid: our_is_geom2 = False; break
            if c.geom2 == gid: our_is_geom2 = True; break
        if our_is_geom2 is None: continue
        other = c.geom1 if our_is_geom2 else c.geom2
        obid = model.geom_bodyid[other]
        obn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, obid) or ""
        if not obn.startswith("box_"): continue
        cf = np.zeros(6); mujoco.mj_contactForce(model, data, i, cf)
        frame = c.frame.reshape(3, 3)
        F_on_geom2 = frame @ cf[:3]
        if our_is_geom2: Fw += F_on_geom2
        else: Fw -= F_on_geom2
    s = float(np.dot(Fw, direction_world)) if direction_world is not None else 0.0
    return Fw, s


# ======================================================================
def main():
    F_DESIRED = -5.0; DT = 0.02; APPROACH_STEP = 0.003

    env = AlfaEnvV5(model_path="scene_v5.xml"); env.reset()
    ik = LeftArmIKV5(); adm = AdmittanceController()
    m, d = env.model, env.data; robot = env.robot

    # ---- (1) 重置箱子位置, 移动底座 ----------------------------------
    print("[1] 初始化 (v5)...")
    box_name = "box_r0_c1_l0"
    box_bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, box_name)
    jadr = m.body_jntadr[box_bid]
    d.qpos[jadr:jadr + 7] = m.qpos0[jadr:jadr + 7]
    d.qvel[m.body_dofadr[box_bid]:m.body_dofadr[box_bid] + 6] = 0.0

    # 逐步移动底座到集装箱门口
    for bx in np.linspace(0.0, 1.5, 10):
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "base_x")]] = bx
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "base_y")]] = 0.18
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "base_yaw")]] = 0.0
        mujoco.mj_forward(m, d)
        for _ in range(5): mujoco.mj_step(m, d)

    # 初始姿态 (v5: 臂前伸, leftjoint2=+pi/2)
    apply_ik_q(m, d, robot, [0.0, np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])
    robot.apply_hybrid_target({
        "pitch": 0.13, "turn": 0, "updown": 0.3,
        "rightjoint1": 0, "rightjoint2": np.pi/2, "rightjoint3": 0,
        "rightjoint4": -np.pi/2, "rightjoint5": 0, "rightjoint6": 0,
        "right_suction": 0, "left_suction": 0,
    })
    mujoco.mj_forward(m, d)
    for _ in range(300): mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)

    box_actual = d.xpos[box_bid]
    ee_sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "left_ee")
    ee_w = d.site_xpos[ee_sid]
    print(f"    箱子: {np.round(box_actual, 3)}")
    print(f"    EE:   {np.round(ee_w, 3)}")
    print(f"    距离: {np.linalg.norm(box_actual - ee_w) * 1000:.0f}mm")

    # ---- (2) IK 定位到箱子附近 --------------------------------------
    print("[2] IK 定位到箱子前方...")
    T_w_u = get_body_pose(m, d, "updown_link"); T_u_w = np.linalg.inv(T_w_u)
    box_u = (T_u_w @ np.append(box_actual, 1.0))[:3]
    q_now = get_ik_q(m, d); ee_u = ik.fk_solve(q_now)[:3, 3]
    to_box_u = box_u - ee_u; dbox_u = to_box_u / (np.linalg.norm(to_box_u) + 1e-9)

    pre_u = box_u - dbox_u * 0.15
    q_pre = ik.solve_position(pre_u, q_init=q_now, tol_pos=3e-3, max_iter=200)
    T_fk = ik.fk_solve(q_pre)
    print(f"    IK误差: {np.linalg.norm(T_fk[:3, 3] - pre_u) * 1000:.1f}mm")

    apply_ik_q(m, d, robot, q_pre)
    for _ in range(500): mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)

    ee_w = d.site_xpos[ee_sid]; box_actual = d.xpos[box_bid]
    to_box = box_actual - ee_w; dist = np.linalg.norm(to_box)
    to_box_dir_w = to_box / (dist + 1e-9)
    print(f"    EE->箱子: {dist * 1000:.0f}mm, 方向: {np.round(to_box_dir_w, 3)}")

    q_cur = get_ik_q(m, d); pos_nom = ik.fk_solve(q_cur)[:3, 3].copy()

    # ---- (3) 逼近 + 力控 -------------------------------------------
    print(f"\n[3] 逼近+力控 (期望={F_DESIRED}N)")
    print(f"    {'Step':>5s} {'阶段':>8s} {'F_box':>8s} {'dX_mm':>7s} "
          f"{'Corr_mm':>7s} {'Dist':>7s} {'Touch':>6s}")
    print(f"    {'-' * 60}")

    phase = "approach"; contact = False; cstep = 0

    for sim_step in range(5000):
        mujoco.mj_step(m, d)
        if sim_step % env.frame_skip != 0: continue
        mujoco.mj_forward(m, d)

        q_cur = get_ik_q(m, d)
        T_w_u = get_body_pose(m, d, "updown_link"); T_u_w = np.linalg.inv(T_w_u)
        ee_u = ik.fk_solve(q_cur)[:3, 3]
        box_actual = d.xpos[box_bid]; box_u = (T_u_w @ np.append(box_actual, 1.0))[:3]
        to_box_u = box_u - ee_u; dist_u = np.linalg.norm(to_box_u)
        if dist_u < 1e-6: continue
        appr_dir_u = to_box_u / dist_u
        appr_dir_w = T_w_u[:3, :3] @ appr_dir_u

        _, F_box = get_box_contact_force(m, d, appr_dir_w)
        touch = robot.get_left_suction_touch()

        if phase == "approach":
            if abs(F_box) > 0.2 or touch > 0:
                phase = "regulate"; adm.reset()
                pos_nom = ee_u.copy(); contact = True
                print(f"\n    {cstep:>5d}  >>> 接触! F={F_box:.2f}N")

            pos_nom = ee_u + appr_dir_u * APPROACH_STEP
        else:
            dX = adm.step(F_box, F_DESIRED, DT)
            pos_nom += appr_dir_u * dX

        q_tgt = ik.solve_position(pos_nom, q_init=q_cur, tol_pos=3e-3, max_iter=50)
        if not np.any(np.isnan(q_tgt)):
            apply_ik_q(m, d, robot, q_tgt)

        if cstep % 30 == 0:
            dX = (adm.vel * DT * 1000) if phase == "regulate" else APPROACH_STEP * 1000
            print(f"    {cstep:>5d} {phase:>8s} {F_box:>8.2f} {dX:>7.2f} "
                  f"{abs(adm.pos_corr) * 1000:>6.1f} {dist_u * 1000:>6.0f} {touch:>6.3f}")

        cstep += 1

    # ---- (4) 结果 --------------------------------------------------
    mujoco.mj_forward(m, d)
    ee_w_f = d.site_xpos[ee_sid]; box_f = d.xpos[box_bid]
    to_box_f = box_f - ee_w_f; dist_f = np.linalg.norm(to_box_f)
    dir_f = to_box_f / (dist_f + 1e-9) if dist_f > 1e-6 else np.zeros(3)
    _, F_fin = get_box_contact_force(m, d, dir_f)
    print(f"\n[4] 结果")
    print(f"    EE:    {np.round(ee_w_f, 3)}")
    print(f"    箱子:  {np.round(box_f, 3)}")
    print(f"    距离:  {dist_f * 1000:.0f}mm")
    print(f"    力:    {F_fin:.2f}N (期望{F_DESIRED}N)")
    print(f"    误差:  {abs(F_fin - F_DESIRED):.2f}N")
    if contact and abs(F_fin - F_DESIRED) < 3:
        print(f"    >>> 导纳控制成功!")
    elif contact:
        print(f"    >>> 已接触, 可调增益改善力跟踪")
    else:
        print(f"    >>> 未接触")

    env.close()


if __name__ == "__main__":
    main()
