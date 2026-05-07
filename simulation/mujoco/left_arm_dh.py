import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class LeftArmFK:
    """
    左臂正向运动学求解器 (基于 alfa_robot.xml 局部坐标树)
    包含 7 个活动自由度 (DOF) 和 1 个末端固连补偿
    """
    def __init__(self):
        # 预定义运动学链的偏置参数 (从父连杆到子连杆的固定位姿)
        # pos: [x, y, z]
        # rot_type: 'euler' (xyz) 或 'quat' (w, x, y, z)
        self.kinematics_chain = [
            {
                "name": "leftarmbase",
                "pos": [-0.16145, 0.3282, 0.146],
                "rot_type": None, "rot_val": None,
                "joint_type": "slide", "axis": "y"
            },
            {
                "name": "leftjoint1",
                "pos": [0.15658, -0.096099, 0.086],
                "rot_type": None, "rot_val": None,
                "joint_type": "slide", "axis": "x"
            },
            {
                "name": "leftjoint2",
                "pos": [0.67335, 0.10135, 0.04],
                "rot_type": "euler", "rot_val": [0, 1.5708, 0],
                "joint_type": "hinge", "axis": "z"
            },
            {
                "name": "leftjoint3",
                "pos": [0.01, 0.0058, 0.098],
                "rot_type": "quat", "rot_val": [-0.5, 0.5, 0.5, 0.5], # MuJoCo quat order: w, x, y, z
                "joint_type": "hinge", "axis": "z"
            },
            {
                "name": "leftjoint4",
                "pos": [0.3418, -0.01, -0.082],
                "rot_type": "quat", "rot_val": [0.5, 0.5, 0.5, 0.5],
                "joint_type": "hinge", "axis": "z"
            },
            {
                "name": "leftjoint5",
                "pos": [0.01, -0.0022, 0.098],
                "rot_type": "quat", "rot_val": [-0.5, 0.5, 0.5, 0.5],
                "joint_type": "hinge", "axis": "z"
            },
            {
                "name": "leftjoint6",
                "pos": [0.13, 0, 0.1435],
                "rot_type": "euler", "rot_val": [1.5708, 0, 0],
                "joint_type": "slide", "axis": "x"
            },
            {
                "name": "left_ee", # 末端吸盘偏移
                "pos": [0.132, 0, 0], 
                "rot_type": None, "rot_val": None,
                "joint_type": "fixed", "axis": None
            }
        ]

    def _build_offset_matrix(self, pos, rot_type, rot_val):
        """生成连杆之间的固定齐次变换矩阵"""
        T = np.eye(4)
        T[:3, 3] = pos
        if rot_type == "euler":
            # MuJoCo 的默认 Euler 顺序是 'xyz' (内部旋转)
            T[:3, :3] = R.from_euler('xyz', rot_val).as_matrix()
        elif rot_type == "quat":
            # MuJoCo 的四元数是 [w, x, y, z]，而 scipy 使用 [x, y, z, w]
            w, x, y, z = rot_val
            T[:3, :3] = R.from_quat([x, y, z, w]).as_matrix()
        return T

    def _build_joint_matrix(self, joint_type, axis, q):
        """生成关节运动带来的齐次变换矩阵"""
        T = np.eye(4)
        if joint_type == "fixed":
            return T

        if joint_type == "slide":
            if axis == "x": T[0, 3] = q
            elif axis == "y": T[1, 3] = q
            elif axis == "z": T[2, 3] = q
        elif joint_type == "hinge":
            c, s = np.cos(q), np.sin(q)
            if axis == "z":
                T[:2, :2] = [[c, -s], [s, c]]
            elif axis == "x":
                T[1:3, 1:3] = [[c, -s], [s, c]]
            elif axis == "y":
                T[0, 0], T[0, 2] = c, s
                T[2, 0], T[2, 2] = -s, c
        return T

    def solve(self, q_list):
        """
        输入: q_list 长度为 7 的关节状态列表 [q_base, q1, q2, q3, q4, q5, q6]
        输出: 4x4 末端齐次变换矩阵 (相对于底盘 updown_link 坐标系)
        """
        assert len(q_list) == 7, "左臂需要 7 个关节输入"
        
        # 补 1 个 0.0 用于末端 fixed 关节的占位，方便遍历
        q_list = list(q_list) + [0.0] 
        
        # 初始化为单位矩阵
        T_current = np.eye(4)
        
        for i, link in enumerate(self.kinematics_chain):
            # 1. 父子坐标系的固定偏移矩阵
            T_offset = self._build_offset_matrix(link["pos"], link["rot_type"], link["rot_val"])
            
            # 2. 关节运动带来的动态变换矩阵
            T_joint = self._build_joint_matrix(link["joint_type"], link["axis"], q_list[i])
            
            # 连乘累计：T_new = T_current * T_offset * T_joint
            T_current = T_current @ T_offset @ T_joint
            
        return T_current

if __name__ == "__main__":
    # 初始化运动学求解器
    fk_solver = LeftArmFK()

    # 提取自 demo.py 中的左臂初始测试位姿
    test_q = [
        0.0,            # leftarmbase (横移)
        0.0,            # leftjoint1 (伸缩)
        -math.pi / 2,   # leftjoint2 (旋转)
        0.0,            # leftjoint3 (旋转)
        math.pi / 2,    # leftjoint4 (旋转)
        0.0,            # leftjoint5 (旋转)
        0.0             # leftjoint6 (末端伸缩)
    ]

    # 求解末端位姿
    T_ee = fk_solver.solve(test_q)

    # 提取位置(XYZ)与姿态(Euler角)
    pos_xyz = T_ee[:3, 3]
    rot_euler = R.from_matrix(T_ee[:3, :3]).as_euler('xyz', degrees=True)

    print("===== 左臂正向运动学求解结果 =====")
    print("输入的关节角度/位移:", [round(q, 4) for q in test_q])
    print("\n末端吸盘相对于机器人基座(updown_link)的 4x4 齐次变换矩阵 T_op:")
    print(np.round(T_ee, 4))
    print(f"\n末端坐标 (X, Y, Z):  [{pos_xyz[0]:.4f}, {pos_xyz[1]:.4f}, {pos_xyz[2]:.4f}] m")
    print(f"末端姿态 (R, P, Y):  [{rot_euler[0]:.2f}, {rot_euler[1]:.2f}, {rot_euler[2]:.2f}] 度")