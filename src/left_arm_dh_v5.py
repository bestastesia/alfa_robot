import numpy as np
from scipy.spatial.transform import Rotation as R
import math


class LeftArmFKV5:
    """
    左臂正向运动学求解器 (v5: 6-DOF 全铰链, 基于 URDF 局部坐标树)
    参考坐标系: updown_link
    """
    def __init__(self):
        # 预定义运动学链的偏置参数 (从父连杆到子连杆的固定位姿)
        # pos: [x, y, z] 来自 URDF joint origin xyz
        # rot_type/rot_val: 来自 URDF joint origin rpy
        # joint_type: all "hinge" for v5
        # joint_axis: all "z" for v5 (in local body frame)
        self.kinematics_chain = [
            {
                "name": "leftjoint1",
                "pos": [0.105, 0.26, 0.2],
                "rot_type": None, "rot_val": None,
                "joint_type": "hinge", "axis": "z"
            },
            {
                "name": "leftjoint2",
                "pos": [0, 0.0905, 0.058],
                "rot_type": "euler", "rot_val": [-1.5708, 0, 0],
                "joint_type": "hinge", "axis": "z"
            },
            {
                "name": "leftjoint3",
                "pos": [0, -0.4, 0.072],
                "rot_type": None, "rot_val": None,
                "joint_type": "hinge", "axis": "z"
            },
            {
                "name": "leftjoint4",
                "pos": [0, -0.339, -0.15],
                "rot_type": "euler", "rot_val": [1.5708, 0, 0],
                "joint_type": "hinge", "axis": "z"
            },
            {
                "name": "leftjoint5",
                "pos": [0, 0.075, 0.058],
                "rot_type": "euler", "rot_val": [-1.5708, 0, 0],
                "joint_type": "hinge", "axis": "z"
            },
            {
                "name": "leftjoint6",
                "pos": [0, -0.083, 0.059],
                "rot_type": "euler", "rot_val": [1.5708, 0, 0],
                "joint_type": "hinge", "axis": "z"
            },
            {
                "name": "left_ee",
                "pos": [0.128, 0, 0],
                "rot_type": None, "rot_val": None,
                "joint_type": "fixed", "axis": None
            }
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
        输入: q_list 长度为 6 的关节状态列表 [q1, q2, q3, q4, q5, q6] (弧度)
        输出: 4x4 末端齐次变换矩阵 (相对于 updown_link 坐标系)
        """
        assert len(q_list) == 6, "v5 左臂需要 6 个关节输入"

        q_list = list(q_list) + [0.0]

        T_current = np.eye(4)

        for i, link in enumerate(self.kinematics_chain):
            T_offset = self._build_offset_matrix(link["pos"], link["rot_type"], link["rot_val"])
            T_joint = self._build_joint_matrix(link["joint_type"], link["axis"], q_list[i])
            T_current = T_current @ T_offset @ T_joint

        return T_current


if __name__ == "__main__":
    fk_solver = LeftArmFKV5()

    test_q = [
        0.0,            # leftjoint1
        -math.pi / 2,   # leftjoint2
        0.0,            # leftjoint3
        math.pi / 2,    # leftjoint4
        0.0,            # leftjoint5
        0.0             # leftjoint6
    ]

    T_ee = fk_solver.solve(test_q)

    pos_xyz = T_ee[:3, 3]
    rot_euler = R.from_matrix(T_ee[:3, :3]).as_euler('xyz', degrees=True)

    print("===== v5 左臂正向运动学求解结果 =====")
    print("输入的关节角度:", [round(q, 4) for q in test_q])
    print("\n末端吸盘相对于 updown_link 的 4x4 齐次变换矩阵 T:")
    print(np.round(T_ee, 4))
    print(f"\n末端坐标 (X, Y, Z):  [{pos_xyz[0]:.4f}, {pos_xyz[1]:.4f}, {pos_xyz[2]:.4f}] m")
    print(f"末端姿态 (R, P, Y):  [{rot_euler[0]:.2f}, {rot_euler[1]:.2f}, {rot_euler[2]:.2f}] deg")
