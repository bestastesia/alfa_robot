import time
import math
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R
from alfa_env import AlfaEnv

# ======================================================================
# 1. 运动学解算器 (复用之前提取的 DH 偏置树)
# ======================================================================
class LeftArmFK:
    def __init__(self):
        self.kinematics_chain = [
            {"name": "leftarmbase", "pos": [-0.16145, 0.3282, 0.146], "rot_type": None, "rot_val": None, "joint_type": "slide", "axis": "y"},
            {"name": "leftjoint1",  "pos": [0.15658, -0.096099, 0.086], "rot_type": None, "rot_val": None, "joint_type": "slide", "axis": "x"},
            {"name": "leftjoint2",  "pos": [0.67335, 0.10135, 0.04],  "rot_type": "euler", "rot_val": [0, 1.5708, 0], "joint_type": "hinge", "axis": "z"},
            {"name": "leftjoint3",  "pos": [0.01, 0.0058, 0.098],     "rot_type": "quat", "rot_val": [-0.5, 0.5, 0.5, 0.5], "joint_type": "hinge", "axis": "z"},
            {"name": "leftjoint4",  "pos": [0.3418, -0.01, -0.082],   "rot_type": "quat", "rot_val": [0.5, 0.5, 0.5, 0.5], "joint_type": "hinge", "axis": "z"},
            {"name": "leftjoint5",  "pos": [0.01, -0.0022, 0.098],    "rot_type": "quat", "rot_val": [-0.5, 0.5, 0.5, 0.5], "joint_type": "hinge", "axis": "z"},
            {"name": "leftjoint6",  "pos": [0.13, 0, 0.1435],         "rot_type": "euler", "rot_val": [1.5708, 0, 0], "joint_type": "slide", "axis": "x"},
            {"name": "left_ee",     "pos": [0.132, 0, 0],             "rot_type": None, "rot_val": None, "joint_type": "fixed", "axis": None}
        ]

    def _build_offset_matrix(self, pos, rot_type, rot_val):
        T = np.eye(4)
        T[:3, 3] = pos
        if rot_type == "euler":
            T[:3, :3] = R.from_euler('xyz', rot_val).as_matrix()
        elif rot_type == "quat": # MuJoCo quat order is [w, x, y, z]
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
    """
    读取仿真中基座(updown_link)和末端(left_ee)的世界坐标系位姿，
    并计算它们之间的相对变换矩阵。
    """
    # 获取 updown_link (基座) 的世界坐标系位姿 T_world^{updown}
    updown_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "updown_link")
    pos_u = env.data.xpos[updown_id]
    mat_u = env.data.xmat[updown_id].reshape(3, 3)
    
    T_w_u = np.eye(4)
    T_w_u[:3, :3] = mat_u
    T_w_u[:3, 3] = pos_u

    # 获取 left_ee (末端吸盘Site) 的世界坐标系位姿 T_world^{ee}
    ee_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, "left_ee")
    pos_e = env.data.site_xpos[ee_id]
    mat_e = env.data.site_xmat[ee_id].reshape(3, 3)

    T_w_e = np.eye(4)
    T_w_e[:3, :3] = mat_e
    T_w_e[:3, 3] = pos_e

    # 计算相对齐次变换矩阵: T_updown^{ee} = (T_world^{updown})^-1 * T_world^{ee}
    T_u_e = np.linalg.inv(T_w_u) @ T_w_e
    return T_u_e

# ======================================================================
# 3. 主控制与对比流程
# ======================================================================
def main():
    # 设定测试的左臂目标关节状态 (共7个DOF)
    test_q = [
        0.05,           # leftarmbase (横移 5cm)
        0.2,            # leftjoint1 (伸长 20cm)
        -math.pi / 4,   # leftjoint2 (旋转 -45度)
        math.pi / 6,    # leftjoint3 (旋转 30度)
        math.pi / 2,    # leftjoint4 (旋转 90度)
        -math.pi / 6,   # leftjoint5 (旋转 -30度)
        0.08            # leftjoint6 (末端伸长 8cm)
    ]

    print(f"\n[1] 正在使用纯数学 FK 求解器计算...")
    fk_solver = LeftArmFK()
    T_fk = fk_solver.solve(test_q)
    print("理论计算的末端位姿矩阵 T_updown^{left_ee}:")
    print(np.round(T_fk, 4))

    print("\n[2] 启动 MuJoCo 仿真环境，驱动机械臂前往目标位置...")
    env = AlfaEnv(model_path="scene.xml", sim_dt=0.002, frame_skip=10)
    env.reset()

    # 构造控制指令
    ctrl_cmds = {
        # 底盘和躯干保持原位 (包含上一轮新增的base关节)
        "base_x": 0.0, "base_y": 0.0, "base_yaw": 0.0,
        "turn": 0.0, "updown": 0.3, "plate": 0.0,
        
        # 右臂保持原位
        "rightarmbase": 0.0, "rightjoint1": 0.0, "rightjoint2": 0.0,
        "rightjoint3": 0.0, "rightjoint4": 0.0,

        # 写入左臂的测试控制量
        "leftarmbase": test_q[0], "leftjoint1": test_q[1], "leftjoint2": test_q[2],
        "leftjoint3":  test_q[3], "leftjoint4": test_q[4], "leftjoint5": test_q[5],
        "leftjoint6":  test_q[6],
        
        "right_suction": 0.0, "left_suction": 0.0,
    }

    # 执行控制指令 (由于有关节阻尼 damping=3000，需要一定的步数让机械臂稳定到位)
    env.step(ctrl_cmds)
    
    steps_to_simulate = 1500  # 1500步 * 0.002s = 3秒的仿真时间，足够 PD 控制器稳定
    for _ in range(steps_to_simulate):
        mujoco.mj_step(env.model, env.data)
        env.render() # 如果不需要渲染动画看过程，可以注释掉这行加快计算

    print("\n[3] 机械臂已到达物理稳定状态，提取仿真底层传感器位姿...")
    T_sim = get_sim_relative_pose(env)
    print("仿真底层的实际位姿矩阵 T_updown^{left_ee}:")
    print(np.round(T_sim, 4))

    # === 误差对比 ===
    print("\n================== 误差分析 ==================")
    pos_err = np.linalg.norm(T_fk[:3, 3] - T_sim[:3, 3])
    print(f"末端位置误差 (L2 Norm): {pos_err * 1000:.6f} 毫米 (mm)")

    # 提取理论和真实的欧拉角做对比
    euler_fk = R.from_matrix(T_fk[:3, :3]).as_euler('xyz', degrees=True)
    euler_sim = R.from_matrix(T_sim[:3, :3]).as_euler('xyz', degrees=True)
    print(f"理论姿态 (Roll, Pitch, Yaw): [{euler_fk[0]:.2f}, {euler_fk[1]:.2f}, {euler_fk[2]:.2f}] 度")
    print(f"真实姿态 (Roll, Pitch, Yaw): [{euler_sim[0]:.2f}, {euler_sim[1]:.2f}, {euler_sim[2]:.2f}] 度")

    env.close()

if __name__ == "__main__":
    main()