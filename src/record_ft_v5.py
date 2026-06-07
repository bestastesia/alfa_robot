"""
记录六维力传感器数据 — 双臂靠近箱子, 看看接触力是否正常
"""
import numpy as np, mujoco, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from src.alfa_interface_v5 import AlfaRobotInterfaceV5


def get_ft_raw(model, data, side):
    """直接从 contact 读取世界系力/力矩, 不做变换"""
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_suction_col")
    eid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_ee")
    ep = data.site_xpos[eid]; eR = data.site_xmat[eid].reshape(3,3)

    Fw = np.zeros(3); Tw = np.zeros(3)
    for i in range(data.ncon):
        c = data.contact[i]
        if c.geom2 == gid:    our2 = True
        elif c.geom1 == gid:  our2 = False
        else: continue
        other = c.geom1 if our2 else c.geom2
        obn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[other]) or ""
        cf = np.zeros(6); mujoco.mj_contactForce(model, data, i, cf)
        fr = c.frame.reshape(3,3)
        Fi = fr @ cf[:3] if our2 else -fr @ cf[:3]
        Fw += Fi
        Tw += np.cross(c.pos - ep, Fi)

    F_ee = eR.T @ Fw   # world -> EE frame
    T_ee = eR.T @ Tw
    return Fw, Tw, F_ee, T_ee, data.ncon


def main():
    model = mujoco.MjModel.from_xml_path("scene_v5.xml")
    data = mujoco.MjData(model); mujoco.mj_resetData(model, data)
    robot = AlfaRobotInterfaceV5(model, data)

    # 从远到近, 逐步逼近箱子, 记录力数据
    DURATION = 5.0       # 秒
    DT = model.opt.timestep
    STEPS = int(DURATION / DT)

    # 初始: 收臂姿态
    robot.apply_hybrid_target(dict(
        base_x=0,base_y=0,base_yaw=0,pitch=0,turn=0,updown=0.3,
        leftjoint1=0,leftjoint2=np.pi/2,leftjoint3=0,leftjoint4=-np.pi/2,leftjoint5=0,leftjoint6=0,
        rightjoint1=0,rightjoint2=np.pi/2,rightjoint3=0,rightjoint4=-np.pi/2,rightjoint5=0,rightjoint6=0,
        right_suction=0,left_suction=0))
    mujoco.mj_forward(model, data)
    for _ in range(300): mujoco.mj_step(model, data)

    # buffers
    t_hist = np.zeros(STEPS)
    FwL = np.zeros((STEPS,3)); TwL = np.zeros((STEPS,3))
    FeL = np.zeros((STEPS,3)); TeL = np.zeros((STEPS,3))
    FwR = np.zeros((STEPS,3)); TwR = np.zeros((STEPS,3))
    FeR = np.zeros((STEPS,3)); TeR = np.zeros((STEPS,3))
    ncL = np.zeros(STEPS); ncR = np.zeros(STEPS)
    eeL = np.zeros((STEPS,3)); eeR = np.zeros((STEPS,3))

    # 目标: 缓慢前移 base_x, 让吸盘逐渐接触箱子
    jid_bx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_x")

    for step in range(STEPS):
        # 从 0 到 2.5m 线性推进 base_x
        bx = 2.5 * step / STEPS
        data.qpos[model.jnt_qposadr[jid_bx]] = bx

        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)

        t_hist[step] = step * DT

        for side in ["left", "right"]:
            Fw, Tw, Fe, Te, nc = get_ft_raw(model, data, side)
            eid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_ee")
            if side == "left":
                FwL[step]=Fw; TwL[step]=Tw; FeL[step]=Fe; TeL[step]=Te; ncL[step]=nc
                eeL[step]=data.site_xpos[eid]
            else:
                FwR[step]=Fw; TwR[step]=Tw; FeR[step]=Fe; TeR[step]=Te; ncR[step]=nc
                eeR[step]=data.site_xpos[eid]

    # ---- 绘图 ----
    fig, axes = plt.subplots(3, 2, figsize=(16, 12))

    for col, (side, Fe, Te, Fw, nc) in enumerate([
        ("Left", FeL, TeL, FwL, ncL),
        ("Right", FeR, TeR, FwR, ncR),
    ]):
        ax1 = axes[0, col]; ax2 = axes[1, col]; ax3 = axes[2, col]

        # EE 系力
        ax1.plot(t_hist, Fe[:,0], 'r-', lw=1, label='F_x')
        ax1.plot(t_hist, Fe[:,1], 'g-', lw=1, label='F_y')
        ax1.plot(t_hist, Fe[:,2], 'b-', lw=1, label='F_z (fwd)')
        ax1.axhline(0, color='gray', lw=0.5)
        ax1.set_ylabel('Force EE (N)'); ax1.legend(fontsize=7); ax1.grid(True, alpha=0.3)
        ax1.set_title(f'{side} — Force in EE frame')

        # EE 系力矩
        ax2.plot(t_hist, Te[:,0], 'r-', lw=1, label='tau_x')
        ax2.plot(t_hist, Te[:,1], 'g-', lw=1, label='tau_y')
        ax2.plot(t_hist, Te[:,2], 'b-', lw=1, label='tau_z')
        ax2.axhline(0, color='gray', lw=0.5)
        ax2.set_ylabel('Torque EE (Nm)'); ax2.legend(fontsize=7); ax2.grid(True, alpha=0.3)
        ax2.set_title(f'{side} — Torque in EE frame')

        # 接触数
        ax3.plot(t_hist, nc, 'k-', lw=1)
        ax3.set_ylabel('ncon'); ax3.set_xlabel('Time (s)'); ax3.grid(True, alpha=0.3)
        ax3.set_title(f'{side} — Contact count')
        ax3.set_ylim(-0.5, 10)

    fig.suptitle('v5 Force/Torque Sensor Record — base_x 0→2.5m', fontsize=13)
    plt.tight_layout()
    plt.savefig("output/ft_record_v5.png", dpi=150)
    plt.savefig("output/ft_record_v5.pdf")
    print("Saved to output/ft_record_v5.{png,pdf}")

    # 打印统计
    for side, Fe, Te in [("Left", FeL, TeL), ("Right", FeR, TeR)]:
        mask = np.abs(Fe[:,2]) > 0.1   # 有接触力的区域
        if mask.any():
            print(f"\n{side} (contact region):")
            print(f"  F_ee mean: {np.round(Fe[mask].mean(axis=0),2)} N")
            print(f"  F_ee std:  {np.round(Fe[mask].std(axis=0),2)} N")
            print(f"  F_ee max:  {np.round(np.abs(Fe[mask]).max(axis=0),2)} N")
            print(f"  T_ee mean: {np.round(Te[mask].mean(axis=0),3)} Nm")
            print(f"  T_ee std:  {np.round(Te[mask].std(axis=0),3)} Nm")


if __name__ == "__main__":
    main()
