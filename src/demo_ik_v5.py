"""
v5 IK demo — 设定目标, 求解, 驱动仿真, 对比目标与实际
"""
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R
from alfa_env_v5 import AlfaEnvV5
from dual_arm_ik_v5 import DualArmIKV5


def main():
    # ================================================================
    # 1. 设定目标 — 位置 + 姿态 (RPY 欧拉角, 度)
    # ================================================================
    target_L = ([1.0,  0.5, 0.65],  np.radians([0, 90, 0]))   # (pos_xyz, rpy)
    target_R = ([1.0, -0.5, 0.65],  np.radians([0, 90, 0]))

    # ================================================================
    # 2. IK 求解 (完整位姿)
    # ================================================================
    ik = DualArmIKV5()
    q_home = [0.0, np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]

    qL = ik.solve_left_pose(target_L, q_init=q_home)
    qR = ik.solve_right_pose(target_R, q_init=q_home)

    T_target_L = ik.fk.solve_left(qL)
    T_target_R = ik.fk.solve_right(qR)

    # ================================================================
    # 3. 启动仿真, 驱动到位
    # ================================================================
    env = AlfaEnvV5(model_path="E:\\qyx\\alfa_robot_v2_arm_v5\\scene_v5.xml")
    env.reset()
    m, d, r = env.model, env.data, env.robot

    # --- 初始归位 ---
    r.apply_hybrid_target({
        "base_x":0, "base_y":0, "base_yaw":0,
        "pitch":0, "turn":0, "updown":0.3,
        "leftjoint1":0, "leftjoint2":0, "leftjoint3":0,
        "leftjoint4":-np.pi/2, "leftjoint5":0, "leftjoint6":0,
        "rightjoint1":0, "rightjoint2":0, "rightjoint3":0,
        "rightjoint4":-np.pi/2, "rightjoint5":0, "rightjoint6":0,
        "right_suction":0, "left_suction":0,
    })
    mujoco.mj_forward(m, d)
    for _ in range(300): mujoco.mj_step(m, d)

    # --- 插值驱动到目标 ---
    def get_q(side):
        names = [f"{side}joint{i}" for i in range(1,7)]
        return np.array([d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]] for n in names])

    def set_q(q, side):
        for i in range(6):
            n = f"{side}joint{i+1}"
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
            d.qpos[m.jnt_qposadr[jid]] = q[i]
            d.qvel[m.jnt_dofadr[jid]] = 0.0
            aid = r.actuator_ids.get(n, -1)
            if aid >= 0: d.ctrl[aid] = q[i]

    qL0, qR0 = get_q("left"), get_q("right")
    for i in range(800):
        t = i / 799
        alpha = 3*t*t - 2*t*t*t          # smoothstep
        set_q(qL0 + alpha*(qL - qL0), "left")
        set_q(qR0 + alpha*(qR - qR0), "right")
        mujoco.mj_step(m, d)
        env.render()

    mujoco.mj_forward(m, d)

    # ================================================================
    # 4. 对比目标 vs 仿真实际
    # ================================================================
    def get_actual_T(side):
        uid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "updown_link")
        Twu = np.eye(4); Twu[:3,3]=d.xpos[uid]; Twu[:3,:3]=d.xmat[uid].reshape(3,3)
        eid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{side}_ee")
        Twe = np.eye(4); Twe[:3,3]=d.site_xpos[eid]; Twe[:3,:3]=d.site_xmat[eid].reshape(3,3)
        return np.linalg.inv(Twu) @ Twe

    T_actual_L = get_actual_T("left")
    T_actual_R = get_actual_T("right")

    for side, T_tgt, T_act, q_sol in [
        ("左臂 left_ee", T_target_L, T_actual_L, qL),
        ("右臂 right_ee", T_target_R, T_actual_R, qR),
    ]:
        pe = np.linalg.norm(T_tgt[:3,3] - T_act[:3,3]) * 1000
        re = np.linalg.norm(R.from_matrix(T_tgt[:3,:3] @ T_act[:3,:3].T).as_rotvec())
        re = np.degrees(re)

        print(f"\n{'─'*55}")
        print(f"  {side}")
        print(f"{'─'*55}")
        print(f"  关节解: {np.array2string(q_sol, precision=3, suppress_small=True)}")
        print(f"\n  目标位姿 T_target (FK计算):")
        print(f"  {np.array2string(T_tgt, prefix='  ', precision=4, suppress_small=True)}")
        print(f"\n  实际位姿 T_actual (仿真测量):")
        print(f"  {np.array2string(T_act, prefix='  ', precision=4, suppress_small=True)}")
        print(f"\n  位置误差 = {pe:.2f} mm    姿态误差 = {re:.4f} deg")

    # 挂起
    while env.render():
        pass
    env.close()


if __name__ == "__main__":
    main()
