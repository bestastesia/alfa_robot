import numpy as np
from scipy.spatial.transform import Rotation as R


class DualArmFKV5:
    """
    v5 双臂正向运动学求解器
    参考坐标系: updown_link
    左臂/右臂各 6 个全铰链关节
    """

    # ---- 运动学链定义 ----
    _left_chain = [
        {"name": "leftjoint1",  "pos": [0.105,  0.26,   0.2],   "rpy": [0,       0, 0]},
        {"name": "leftjoint2",  "pos": [0,      0.0905, 0.058], "rpy": [-1.5708, 0, 0]},
        {"name": "leftjoint3",  "pos": [0,     -0.4,    0.072], "rpy": [0,       0, 0]},
        {"name": "leftjoint4",  "pos": [0,     -0.339, -0.15],  "rpy": [1.5708,  0, 0]},
        {"name": "leftjoint5",  "pos": [0,      0.075,  0.058], "rpy": [-1.5708, 0, 0]},
        {"name": "leftjoint6",  "pos": [0,     -0.083,  0.059], "rpy": [1.5708,  0, 0]},
        {"name": "left_ee",     "pos": [0,      0,      0.156], "rpy": [0,       0, 0]},
    ]

    _right_chain = [
        {"name": "rightjoint1", "pos": [0.105, -0.26,    0.2],   "rpy": [0,       0, 0]},
        {"name": "rightjoint2", "pos": [0,     -0.0905,  0.058], "rpy": [-1.5708, 0, 0]},
        {"name": "rightjoint3", "pos": [0,     -0.4,    -0.072], "rpy": [0,       0, 0]},
        {"name": "rightjoint4", "pos": [0,     -0.339,   0.15],  "rpy": [1.5708,  0, 0]},
        {"name": "rightjoint5", "pos": [0,     -0.076,   0.058], "rpy": [-1.5708, 0, 0]},
        {"name": "rightjoint6", "pos": [0,     -0.083,  -0.058], "rpy": [1.5708,  0, 0]},
        {"name": "right_ee",    "pos": [0,      0,       0.156], "rpy": [0,       0, 0]},
    ]

    def __init__(self):
        self.n_dof = 6

    # ------------------------------------------------------------------
    def solve_left(self, q):
        """
        左臂正运动学
        q: [q0..q5] 6 个关节角 (rad), leftjoint1..leftjoint6
        返回: 4x4 齐次矩阵 T_updown^{left_ee}
        """
        return self._solve_chain(self._left_chain, q)

    def solve_right(self, q):
        """
        右臂正运动学
        q: [q0..q5] 6 个关节角 (rad), rightjoint1..rightjoint6
        返回: 4x4 齐次矩阵 T_updown^{right_ee}
        """
        return self._solve_chain(self._right_chain, q)

    def solve(self, q_left, q_right=None):
        """
        双臂正运动学
        q_left:  左臂 6 关节角
        q_right: 右臂 6 关节角 (None 则只算左臂, 返回单矩阵)
        返回: (T_left, T_right) 或 T_left
        """
        T_left = self.solve_left(q_left)
        if q_right is None:
            return T_left
        T_right = self.solve_right(q_right)
        return T_left, T_right

    # ------------------------------------------------------------------
    def _solve_chain(self, chain, q):
        assert len(q) == self.n_dof, f"需要 {self.n_dof} 个关节输入"
        q = list(q) + [0.0]  # 末端 fixed 占位
        T = np.eye(4)
        for i, link in enumerate(chain):
            T_off = self._make_offset(link["pos"], link["rpy"])
            T_jnt = self._rot_z(q[i]) if i < self.n_dof else np.eye(4)
            T = T @ T_off @ T_jnt
        return T

    @staticmethod
    def _make_offset(pos, rpy):
        T = np.eye(4)
        T[:3, 3] = pos
        if any(r != 0 for r in rpy):
            T[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
        return T

    @staticmethod
    def _rot_z(q):
        c, s = np.cos(q), np.sin(q)
        return np.array([[c, -s, 0, 0],
                         [s,  c, 0, 0],
                         [0,  0, 1, 0],
                         [0,  0, 0, 1]])


# ======================================================================
def main():
    fk = DualArmFKV5()
    np.set_printoptions(precision=3, suppress=True)

    # 前伸姿态
    q_home = [0.0, np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]

    print("=" * 60)
    print("v5 双臂正向运动学")
    print("=" * 60)

    T_left, T_right = fk.solve(q_home, q_home)

    print(f"\n左臂 (q={np.round(q_home, 2)}):")
    print(f"  位置: {T_left[:3, 3]}")
    euler_l = R.from_matrix(T_left[:3, :3]).as_euler('xyz', degrees=True)
    print(f"  姿态 (RPY deg): {np.round(euler_l, 1)}")

    print(f"\n右臂 (q={np.round(q_home, 2)}):")
    print(f"  位置: {T_right[:3, 3]}")
    euler_r = R.from_matrix(T_right[:3, :3]).as_euler('xyz', degrees=True)
    print(f"  姿态 (RPY deg): {np.round(euler_r, 1)}")

    print(f"\n双臂间距: {np.linalg.norm(T_left[:3, 3] - T_right[:3, 3]):.3f}m")

    # 打印完整矩阵
    print("\n" + "-" * 40)
    print("左臂完整 4x4 矩阵:")
    print(np.round(T_left, 4))
    print("\n右臂完整 4x4 矩阵:")
    print(np.round(T_right, 4))


if __name__ == "__main__":
    main()
