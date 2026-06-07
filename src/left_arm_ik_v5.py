import numpy as np
from scipy.spatial.transform import Rotation as R
from left_arm_dh_v5 import LeftArmFKV5


class LeftArmIKV5:
    """
    左臂逆运动学求解器 (6-DOF, 全铰链)
    使用阻尼最小二乘法 (DLS) + 数值雅可比 + 零空间优化
    参考坐标系: updown_link
    """
    def __init__(self):
        self.fk = LeftArmFKV5()
        # 关节限位 [q0..q5] (全铰链, 弧度)
        self.joint_limits = [
            (-np.pi, np.pi),      # leftjoint1
            (-np.pi, np.pi),      # leftjoint2
            (-np.pi, np.pi),      # leftjoint3
            (-np.pi, np.pi),      # leftjoint4
            (-np.pi, np.pi),      # leftjoint5
            (-np.pi, np.pi),      # leftjoint6
        ]
        self._default_q = np.array([0.0, np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------
    def solve(self, target, q_init=None, *, pos_only=False,
              max_iter=200, tol_pos=1e-3, tol_rot=1e-2,
              damping=0.05, step_scale=0.5, nullspace_gain=0.01):
        T_target = self._parse_target(target)
        if q_init is None:
            q_init = self._default_q.copy()
        q = np.array(q_init, dtype=float)

        for _ in range(max_iter):
            T_curr = self.fk.solve(q)
            err = self._pose_error(T_curr, T_target, pos_only)
            if np.linalg.norm(err) < (tol_pos if pos_only else tol_pos + tol_rot):
                break

            J = self._numerical_jacobian(q, pos_only)
            dq = self._dls_step(J, err, damping)
            dq = self._nullspace_project(J, dq, q, nullspace_gain, damping)
            q = self._clamp_to_limits(q + step_scale * dq)

        return q

    def solve_position(self, pos_xyz, q_init=None, **kwargs):
        kwargs.setdefault('tol_pos', 1e-3)
        return self.solve(pos_xyz, q_init, pos_only=True, **kwargs)

    def solve_pose(self, target, q_init=None, **kwargs):
        return self.solve(target, q_init, pos_only=False, **kwargs)

    def fk_solve(self, q):
        return self.fk.solve(q)

    # ------------------------------------------------------------------
    # 内部方法
    # ------------------------------------------------------------------
    def _parse_target(self, target):
        if isinstance(target, np.ndarray) and target.shape == (4, 4):
            return target.copy()
        if isinstance(target, np.ndarray) and target.ndim == 1 and len(target) == 3:
            T = np.eye(4)
            T[:3, 3] = target
            return T
        if isinstance(target, (list, tuple)):
            if len(target) == 2:
                pos, orient = target
                orient = np.asarray(orient)
                T = np.eye(4)
                T[:3, 3] = pos
                if orient.shape == (3,):
                    T[:3, :3] = R.from_euler('xyz', orient).as_matrix()
                elif orient.shape == (4,):
                    T[:3, :3] = R.from_quat(orient).as_matrix()
                else:
                    raise ValueError(f"Unknown orientation shape: {orient.shape}")
                return T
            elif len(target) == 3:
                T = np.eye(4)
                T[:3, 3] = target
                return T
        raise ValueError(f"Cannot parse target: {target}")

    def _pose_error(self, T_curr, T_target, pos_only):
        p_err = T_target[:3, 3] - T_curr[:3, 3]
        if pos_only:
            return p_err
        R_err = T_target[:3, :3] @ T_curr[:3, :3].T
        rot_vec = R.from_matrix(R_err).as_rotvec()
        return np.concatenate([p_err, rot_vec])

    def _numerical_jacobian(self, q, pos_only, eps=1e-6):
        T_curr = self.fk.solve(q)
        m = 3 if pos_only else 6
        J = np.zeros((m, 6))
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = eps
            T_pert = self.fk.solve(q + dq)
            J[:3, i] = (T_pert[:3, 3] - T_curr[:3, 3]) / eps
            if not pos_only:
                R_diff = T_pert[:3, :3] @ T_curr[:3, :3].T
                rot_vec = R.from_matrix(R_diff).as_rotvec()
                J[3:6, i] = rot_vec / eps
        return J

    def _dls_step(self, J, err, damping):
        JJT = J @ J.T
        lam_sq = damping * damping
        A = JJT + lam_sq * np.eye(JJT.shape[0])
        try:
            dq = J.T @ np.linalg.solve(A, err)
        except np.linalg.LinAlgError:
            dq = J.T @ np.linalg.lstsq(A, err, rcond=None)[0]
        return dq

    def _nullspace_project(self, J, dq, q, gain, damping=0.05):
        q_mid = np.array([(lo + hi) / 2.0 for lo, hi in self.joint_limits])
        grad = 2.0 * (q - q_mid) / np.array([max(hi - lo, 1e-6) for lo, hi in self.joint_limits])
        m = J.shape[0]
        JJT_damped = J @ J.T + (damping * damping) * np.eye(m)
        J_dls_inv = J.T @ np.linalg.inv(JJT_damped)
        nullspace = np.eye(6) - J_dls_inv @ J
        return dq - gain * (nullspace @ grad)

    def _clamp_to_limits(self, q):
        qc = q.copy()
        for i, (lo, hi) in enumerate(self.joint_limits):
            qc[i] = np.clip(qc[i], lo, hi)
        return qc


# ======================================================================
# 测试 & 演示
# ======================================================================
if __name__ == "__main__":
    ik = LeftArmIKV5()

    print("=" * 60)
    print("测试 1: 完整位姿 IK (v5 6-DOF)")
    q_test = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    T_target = ik.fk_solve(q_test)
    q_solved = ik.solve_pose(T_target)
    T_solved = ik.fk_solve(q_solved)
    pos_err = np.linalg.norm(T_target[:3, 3] - T_solved[:3, 3])
    rot_err = np.linalg.norm(R.from_matrix(T_target[:3, :3] @ T_solved[:3, :3].T).as_rotvec())
    print(f"  位置误差: {pos_err*1000:.3f} mm")
    print(f"  姿态误差: {np.degrees(rot_err):.4f} deg")
    print(f"  关节解: {np.round(q_solved, 4)}")

    print("\n" + "=" * 60)
    print("测试 2: 仅位置 IK")
    pos_target = [0.8, 0.3, 0.5]
    q_solved = ik.solve_position(pos_target)
    T_solved = ik.fk_solve(q_solved)
    pos_err = np.linalg.norm(pos_target - T_solved[:3, 3])
    print(f"  目标位置: {pos_target}")
    print(f"  实际位置: {np.round(T_solved[:3, 3], 4)}")
    print(f"  位置误差: {pos_err*1000:.3f} mm")
    print(f"  关节解: {np.round(q_solved, 4)}")

    print("\n" + "=" * 60)
    print("测试 3: 关节限位检查")
    for i, (lo, hi) in enumerate(ik.joint_limits):
        in_range = lo <= q_solved[i] <= hi
        print(f"  q{i}: {q_solved[i]:.4f} in [{lo}, {hi}]  {'OK' if in_range else 'VIOLATED'}")
