"""
v5 双臂逆运动学求解器 (DLS + 数值雅可比)
左臂/右臂各 6-DOF 全铰链, 参考坐标系 updown_link
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from dual_arm_fk_v5 import DualArmFKV5


class DualArmIKV5:
    def __init__(self):
        self.fk = DualArmFKV5()
        self.n_dof = 6
        self.joint_limits = [(-np.pi, np.pi)] * 6
        self._default_qL = np.array([0.0,  np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])
        self._default_qR = np.array([0.0,  np.pi/2, 0.0, -np.pi/2, 0.0, 0.0])

    # ==================================================================
    # 左臂
    # ==================================================================
    def solve_left(self, target, q_init=None, *, pos_only=False,
                   max_iter=200, tol_pos=1e-3, tol_rot=1e-2,
                   damping=0.05, step_scale=0.5, nullspace_gain=0.01):
        return self._solve(target, (q_init if q_init is not None else self._default_qL.copy()),
                           self.fk.solve_left, pos_only,
                           max_iter, tol_pos, tol_rot,
                           damping, step_scale, nullspace_gain)

    def solve_left_position(self, pos_xyz, q_init=None, **kw):
        kw.setdefault('tol_pos', 1e-3)
        return self.solve_left(pos_xyz, q_init, pos_only=True, **kw)

    def solve_left_pose(self, target, q_init=None, **kw):
        return self.solve_left(target, q_init, pos_only=False, **kw)

    # ==================================================================
    # 右臂
    # ==================================================================
    def solve_right(self, target, q_init=None, *, pos_only=False,
                    max_iter=200, tol_pos=1e-3, tol_rot=1e-2,
                    damping=0.05, step_scale=0.5, nullspace_gain=0.01):
        return self._solve(target, (q_init if q_init is not None else self._default_qR.copy()),
                           self.fk.solve_right, pos_only,
                           max_iter, tol_pos, tol_rot,
                           damping, step_scale, nullspace_gain)

    def solve_right_position(self, pos_xyz, q_init=None, **kw):
        kw.setdefault('tol_pos', 1e-3)
        return self.solve_right(pos_xyz, q_init, pos_only=True, **kw)

    def solve_right_pose(self, target, q_init=None, **kw):
        return self.solve_right(target, q_init, pos_only=False, **kw)

    # ==================================================================
    # 内部
    # ==================================================================
    def _solve(self, target, q, fk_fn, pos_only,
               max_iter, tol_pos, tol_rot, damping, step_scale, ns_gain):
        T_target = self._parse_target(target)
        for _ in range(max_iter):
            T_curr = fk_fn(q)
            err = self._pose_error(T_curr, T_target, pos_only)
            if np.linalg.norm(err) < (tol_pos if pos_only else tol_pos + tol_rot):
                break
            J = self._numerical_jacobian(q, fk_fn, pos_only)
            dq = self._dls_step(J, err, damping)
            dq = self._nullspace_project(J, dq, q, ns_gain, damping)
            q = self._clamp(q + step_scale * dq)
        return q

    def _numerical_jacobian(self, q, fk_fn, pos_only, eps=1e-6):
        T_curr = fk_fn(q)
        m = 3 if pos_only else 6
        J = np.zeros((m, self.n_dof))
        for i in range(self.n_dof):
            dq = np.zeros(self.n_dof); dq[i] = eps
            T_pert = fk_fn(q + dq)
            J[:3, i] = (T_pert[:3, 3] - T_curr[:3, 3]) / eps
            if not pos_only:
                R_diff = T_pert[:3, :3] @ T_curr[:3, :3].T
                J[3:6, i] = R.from_matrix(R_diff).as_rotvec() / eps
        return J

    def _dls_step(self, J, err, damping):
        A = J @ J.T + (damping ** 2) * np.eye(J.shape[0])
        try:
            return J.T @ np.linalg.solve(A, err)
        except np.linalg.LinAlgError:
            return J.T @ np.linalg.lstsq(A, err, rcond=None)[0]

    def _nullspace_project(self, J, dq, q, gain, damping):
        lo = np.array([l for l, _ in self.joint_limits])
        hi = np.array([h for _, h in self.joint_limits])
        mid = (lo + hi) / 2
        grad = 2.0 * (q - mid) / np.maximum(hi - lo, 1e-6)
        m = J.shape[0]
        JJT_d = J @ J.T + (damping ** 2) * np.eye(m)
        N = np.eye(self.n_dof) - J.T @ np.linalg.inv(JJT_d) @ J
        return dq - gain * (N @ grad)

    def _clamp(self, q):
        return np.array([np.clip(q[i], lo, hi)
                         for i, (lo, hi) in enumerate(self.joint_limits)])

    # ---- 目标解析 ----
    @staticmethod
    def _parse_target(target):
        if isinstance(target, np.ndarray) and target.shape == (4, 4):
            return target.copy()
        if isinstance(target, np.ndarray) and target.ndim == 1 and len(target) == 3:
            T = np.eye(4); T[:3, 3] = target; return T
        if isinstance(target, (list, tuple)):
            if len(target) == 2:
                pos, orient = target; orient = np.asarray(orient)
                T = np.eye(4); T[:3, 3] = pos
                if orient.shape == (3,):
                    T[:3, :3] = R.from_euler('xyz', orient).as_matrix()
                elif orient.shape == (4,):
                    T[:3, :3] = R.from_quat(orient).as_matrix()
                return T
            if len(target) == 3:
                T = np.eye(4); T[:3, 3] = target; return T
        raise ValueError(f"Cannot parse target: {target}")

    @staticmethod
    def _pose_error(T_curr, T_target, pos_only):
        p_err = T_target[:3, 3] - T_curr[:3, 3]
        if pos_only: return p_err
        R_err = T_target[:3, :3] @ T_curr[:3, :3].T
        return np.concatenate([p_err, R.from_matrix(R_err).as_rotvec()])


# ======================================================================
def main():
    ik = DualArmIKV5()
    np.set_printoptions(precision=4, suppress=True)

    # 左臂自洽测试
    print("=" * 55)
    print("左臂 IK 自洽测试")
    q_test = [0.0, np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
    T_target = ik.fk.solve_left(q_test)
    q_solved = ik.solve_left_pose(T_target)
    T_solved = ik.fk.solve_left(q_solved)
    pos_err = np.linalg.norm(T_target[:3, 3] - T_solved[:3, 3])
    rot_err = np.linalg.norm(R.from_matrix(T_target[:3, :3] @ T_solved[:3, :3].T).as_rotvec())
    print(f"  pos err: {pos_err*1000:.3f} mm")
    print(f"  rot err: {np.degrees(rot_err):.4f} deg")
    print(f"  q: {np.round(q_solved, 3)}")

    # 左臂位置 IK
    print("\n" + "=" * 55)
    print("左臂 位置 IK: target [0.8, 0.3, 0.5]")
    q = ik.solve_left_position([0.8, 0.3, 0.5])
    T = ik.fk.solve_left(q)
    print(f"  actual: {np.round(T[:3, 3], 4)}")
    print(f"  err: {np.linalg.norm([0.8,0.3,0.5] - T[:3,3])*1000:.2f} mm")
    print(f"  q: {np.round(q, 3)}")

    # 右臂自洽测试
    print("\n" + "=" * 55)
    print("右臂 IK 自洽测试")
    q_test = [0.0, np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
    T_target = ik.fk.solve_right(q_test)
    q_solved = ik.solve_right_pose(T_target)
    T_solved = ik.fk.solve_right(q_solved)
    pos_err = np.linalg.norm(T_target[:3, 3] - T_solved[:3, 3])
    rot_err = np.linalg.norm(R.from_matrix(T_target[:3, :3] @ T_solved[:3, :3].T).as_rotvec())
    print(f"  pos err: {pos_err*1000:.3f} mm")
    print(f"  rot err: {np.degrees(rot_err):.4f} deg")
    print(f"  q: {np.round(q_solved, 3)}")

    # 右臂位置 IK
    print("\n" + "=" * 55)
    print("右臂 位置 IK: target [0.8, -0.3, 0.5]")
    q = ik.solve_right_position([0.8, -0.3, 0.5])
    T = ik.fk.solve_right(q)
    print(f"  actual: {np.round(T[:3, 3], 4)}")
    print(f"  err: {np.linalg.norm([0.8,-0.3,0.5] - T[:3,3])*1000:.2f} mm")
    print(f"  q: {np.round(q, 3)}")


if __name__ == "__main__":
    main()
