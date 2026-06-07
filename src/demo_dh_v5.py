import time
import math
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R
from src.alfa_env_v5 import AlfaEnvV5


# ======================================================================
# 1. 运动学解算器 (v5 6-DOF 全铰链)
# ======================================================================
class LeftArmFK:
    def __init__(self):
        self.kinematics_chain = [
            {"name": "leftjoint1", "pos": [0.105, 0.26, 0.2],
             "rot_type": None, "rot_val": None, "joint_type": "hinge", "axis": "z"},
            {"name": "leftjoint2", "pos": [0, 0.0905, 0.058],
             "rot_type": "euler", "rot_val": [-1.5708, 0, 0], "joint_type": "hinge", "axis": "z"},
            {"name": "leftjoint3", "pos": [0, -0.4, 0.072],
             "rot_type": None, "rot_val": None, "joint_type": "hinge", "axis": "z"},
            {"name": "leftjoint4", "pos": [0, -0.339, -0.15],
             "rot_type": "euler", "rot_val": [1.5708, 0, 0], "joint_type": "hinge", "axis": "z"},
            {"name": "leftjoint5", "pos": [0, 0.075, 0.058],
             "rot_type": "euler", "rot_val": [-1.5708, 0, 0], "joint_type": "hinge", "axis": "z"},
            {"name": "leftjoint6", "pos": [0, -0.083, 0.059],
             "rot_type": "euler", "rot_val": [1.5708, 0, 0], "joint_type": "hinge", "axis": "z"},
            {"name": "left_ee", "pos": [0, 0, 0.128],
             "rot_type": None, "rot_val": None, "joint_type": "fixed", "axis": None}
        ]

    def _build_offset_matrix(self, pos, rot_type, rot_val):
        T = np.eye(4)
        T[:3, 3] = pos
        if rot_type == "euler":
            T[:3, :3] = R.from_euler('xyz', rot_val).as_matrix()
        elif rot_type == "quat":
            w, x, y, z = rot_val
            T[:3, :3] = R.from_quat([x, y, z, w]).as_matrix()
        return T

    def _build_joint_matrix(self, joint_type, axis, q):
        T = np.eye(4)
        if joint_type == "fixed": return T
        if joint_type == "slide":
            if axis == "x": T[0, 3] = q
            elif axis == "y": T[1, 3] = q
            elif axis == "z": T[2, 3] = q
        elif joint_type == "hinge":
            c, s = np.cos(q), np.sin(q)
            if axis == "z": T[:2, :2] = [[c, -s], [s, c]]
            elif axis == "x": T[1:3, 1:3] = [[c, -s], [s, c]]
            elif axis == "y":
                T[0, 0], T[0, 2] = c, s
                T[2, 0], T[2, 2] = -s, c
        return T

    def solve(self, q_list):
        q_list = list(q_list) + [0.0]
        T_current = np.eye(4)
        for i, link in enumerate(self.kinematics_chain):
            T_offset = self._build_offset_matrix(link["pos"], link["rot_type"], link["rot_val"])
            T_joint = self._build_joint_matrix(link["joint_type"], link["axis"], q_list[i])
            T_current = T_current @ T_offset @ T_joint
        return T_current


# ======================================================================
# 2. 从 MuJoCo 仿真底层读取真实的物理位姿
# ======================================================================
def get_sim_relative_pose(env):
    updown_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "updown_link")
    pos_u = env.data.xpos[updown_id]
    mat_u = env.data.xmat[updown_id].reshape(3, 3)

    T_w_u = np.eye(4)
    T_w_u[:3, :3] = mat_u
    T_w_u[:3, 3] = pos_u

    ee_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, "left_ee")
    pos_e = env.data.site_xpos[ee_id]
    mat_e = env.data.site_xmat[ee_id].reshape(3, 3)

    T_w_e = np.eye(4)
    T_w_e[:3, :3] = mat_e
    T_w_e[:3, 3] = pos_e

    T_u_e = np.linalg.inv(T_w_u) @ T_w_e
    return T_u_e


# ======================================================================
# 3. 主控制与对比流程
# ======================================================================
def main():
    # v5 左臂: 6 个旋转关节
    test_q = [
        math.pi / 6,    # leftjoint1 (30度)
        -math.pi / 4,   # leftjoint2 (-45度)
        math.pi / 6,    # leftjoint3 (30度)
        math.pi / 2,    # leftjoint4 (90度)
        -math.pi / 6,   # leftjoint5 (-30度)
        math.pi / 4,    # leftjoint6 (45度)
    ]

    print(f"\n[1] 正在使用纯数学 FK 求解器计算 (v5 6-DOF)...")
    fk_solver = LeftArmFK()
    T_fk = fk_solver.solve(test_q)
    print("理论计算的末端位姿矩阵 T_updown^{left_ee}:")
    print(np.round(T_fk, 4))

    print("\n[2] 启动 MuJoCo 仿真环境，驱动机械臂前往目标位置...")
    env = AlfaEnvV5(model_path="scene_v5.xml", sim_dt=0.002, frame_skip=10)
    env.reset()

    ctrl_cmds = {
        "base_x": 0.0, "base_y": 0.0, "base_yaw": 0.0,
        "pitch": 0.13, "turn": 0.0, "updown": 0.3,

        "rightjoint1": 0.0, "rightjoint2": 0.0, "rightjoint3": 0.0,
        "rightjoint4": 0.0, "rightjoint5": 0.0, "rightjoint6": 0.0,

        "leftjoint1": test_q[0], "leftjoint2": test_q[1], "leftjoint3": test_q[2],
        "leftjoint4": test_q[3], "leftjoint5": test_q[4], "leftjoint6": test_q[5],

        "right_suction": 0.0, "left_suction": 0.0,
    }

    env.step(ctrl_cmds)

    steps_to_simulate = 1500
    for _ in range(steps_to_simulate):
        mujoco.mj_step(env.model, env.data)
        env.render()

    print("\n[3] 机械臂已到达物理稳定状态，提取仿真底层传感器位姿...")
    T_sim = get_sim_relative_pose(env)
    print("仿真底层的实际位姿矩阵 T_updown^{left_ee}:")
    print(np.round(T_sim, 4))

    # === 误差对比 ===
    print("\n================== 误差分析 ==================")
    pos_err = np.linalg.norm(T_fk[:3, 3] - T_sim[:3, 3])
    print(f"末端位置误差 (L2 Norm): {pos_err * 1000:.6f} 毫米 (mm)")

    euler_fk = R.from_matrix(T_fk[:3, :3]).as_euler('xyz', degrees=True)
    euler_sim = R.from_matrix(T_sim[:3, :3]).as_euler('xyz', degrees=True)
    print(f"理论姿态 (Roll, Pitch, Yaw): [{euler_fk[0]:.2f}, {euler_fk[1]:.2f}, {euler_fk[2]:.2f}] deg")
    print(f"真实姿态 (Roll, Pitch, Yaw): [{euler_sim[0]:.2f}, {euler_sim[1]:.2f}, {euler_sim[2]:.2f}] deg")

    env.close()


if __name__ == "__main__":
    main()
