from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy

from v3_rezero_analytic import RezeroAnalytic


ROOT = Path(__file__).resolve().parents[1]
PROBE_OFFSET_M = .070


def nearby_targets(target, tilt_deg=25.):
    if not 0 <= tilt_deg <= 30:
        raise ValueError('朝向偏转必须在0～30度')
    output = [('原朝向', target.copy(), 0.)]
    for axis, vector in [('X', np.array([1., 0., 0.])), ('Y', np.array([0., 1., 0.])), ('Z', np.array([0., 0., 1.]))]:
        for sign in (-1, 1):
            pose = target.copy()
            pose[:3, :3] = target[:3, :3] @ Rotation.from_rotvec(vector * np.deg2rad(tilt_deg * sign)).as_matrix()
            output.append((f'局部{axis}{tilt_deg * sign:+g}°', pose, tilt_deg))
    return output


class ProbeTouchIK:
    def __init__(self, updown_m):
        self.updown_m = float(updown_m)
        self.model = yourdfpy.URDF.load(
            ROOT / 'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf',
            load_meshes=False)
        joint = self.model.joint_map['updown']
        if not joint.limit.lower <= self.updown_m <= joint.limit.upper:
            raise ValueError('固定Updown超出模型限位')
        self.analytic = RezeroAnalytic()

    def state(self, left):
        values = {name: 0. for name in self.model.actuated_joint_names}
        values['updown'] = self.updown_m
        values.update({f'left_joint{index}': angle for index, angle in enumerate(left, 1)})
        self.model.update_cfg(values)

    def forward(self, left):
        self.state(left)
        pose = self.model.get_transform('left_link7', self.model.base_link).copy()
        pose[:3, 3] += pose[:3, :3] @ np.array([0., 0., PROBE_OFFSET_M])
        return pose

    def solve(self, target, seed, step_deg=3., swivel_samples=None):
        target = np.asarray(target, dtype=float)
        seed = np.asarray(seed, dtype=float)
        if (target.shape != (4, 4) or seed.shape != (7,)
                or not np.isfinite(target).all() or not np.isfinite(seed).all()
                or not np.allclose(target[3], [0, 0, 0, 1])
                or not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-7)
                or not np.isclose(np.linalg.det(target[:3, :3]), 1.)
                or not 0.1 <= step_deg <= 30.):
            raise ValueError('目标必须为合法SE(3)，种子为七轴有限弧度，步长0.1～30度')
        self.state(seed)
        carriage = self.model.get_transform('arm_carriage', self.model.base_link)
        link_to_old_tool = self.model.get_transform('left_tool0', 'left_link7')
        probe_to_link = np.eye(4)
        probe_to_link[2, 3] = -PROBE_OFFSET_M
        old_target = np.linalg.inv(carriage) @ target @ probe_to_link @ link_to_old_tool
        swivels = np.r_[self.analytic.swivel(0, seed), np.deg2rad(np.arange(-180., 180., step_deg))]
        if swivel_samples is not None:
            swivels = np.asarray(swivel_samples, dtype=float)
            if swivels.ndim != 1 or not np.isfinite(swivels).all():
                raise ValueError('冗余角采样必须为有限一维数组')
        solutions = []
        for swivel in swivels:
            for angles in self.analytic.solve(0, old_target, float(swivel), seed):
                if any(np.max(np.abs(angles - row['joints'])) < 1e-6 for row in solutions):
                    continue
                actual = self.forward(angles)
                position_error = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
                rotation_error = float(Rotation.from_matrix(actual[:3, :3].T @ target[:3, :3]).magnitude())
                if position_error > 2e-6 or rotation_error > 2e-6:
                    continue
                solutions.append(dict(joints=angles.copy(), swivel_rad=float(swivel),
                    joint_distance_rad=float(np.linalg.norm(angles - seed)),
                    position_error_m=position_error, rotation_error_rad=rotation_error))
        solutions.sort(key=lambda row: row['joint_distance_rad'])
        self.state(seed)
        return solutions

    def solve_nearest(self, target, seed):
        seed = np.asarray(seed, dtype=float)
        current = self.analytic.swivel(0, seed)
        samples = np.r_[np.deg2rad(np.arange(-180., 180., 15.)),
            current + np.deg2rad([0., -1., 1., -3., 3., -7., 7.])]
        rows = self.solve(target, seed, swivel_samples=samples)
        if not rows:
            rows = self.solve(target, seed, step_deg=3.)
        if not rows:
            return None
        best = rows[0]
        for radius, step in [(3., .5), (.5, .1)]:
            refined = self.solve(target, seed, swivel_samples=best['swivel_rad'] + np.deg2rad(np.arange(-radius, radius + step / 2, step)))
            if refined and refined[0]['joint_distance_rad'] < best['joint_distance_rad']:
                best = refined[0]
        return best

    def solve_local(self, target, seed, locked=None, max_iterations=60):
        import time
        started = time.perf_counter()
        target = np.asarray(target, dtype=float)
        seed = np.asarray(seed, dtype=float)
        if target.shape != (4, 4) or seed.shape != (7,) or not np.isfinite([*target.ravel(), *seed]).all():
            raise ValueError('非法目标或种子')
        if not np.allclose(target[3], [0,0,0,1]) or not np.allclose(target[:3,:3].T @ target[:3,:3], np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(target[:3,:3]), 1.):
            raise ValueError('目标必须是SE(3)')
        angles = np.clip(seed.copy(), self.analytic.lower[0], self.analytic.upper[0])
        active = list(range(7))
        if locked is not None:
            index = locked['joint_index']
            if index not in active or not self.analytic.lower[0,index] <= locked['angle_rad'] <= self.analytic.upper[0,index]:
                raise ValueError('非法锁定轴或角度')
            angles[index] = locked['angle_rad']
            active.remove(index)
        for iteration in range(max_iterations):
            actual = self.forward(angles)
            position = target[:3,3] - actual[:3,3]
            rotation = Rotation.from_matrix(target[:3,:3] @ actual[:3,:3].T).as_rotvec()
            if np.linalg.norm(position) <= 2e-6 and np.linalg.norm(rotation) <= 2e-6:
                return dict(joints=angles.copy(), joint_distance_rad=float(np.linalg.norm(angles-seed)),
                    position_error_m=float(np.linalg.norm(position)), rotation_error_rad=float(np.linalg.norm(rotation)),
                    solver='local_damped_jacobian', iterations=iteration, solve_ms=(time.perf_counter()-started)*1000)
            if time.perf_counter() - started > .35:
                break
            columns = []
            for index in active:
                joint = self.model.joint_map[f'left_joint{index+1}']
                frame = self.model.get_transform(joint.child, self.model.base_link)
                axis = frame[:3,:3] @ joint.axis
                columns.append(np.r_[np.cross(axis, actual[:3,3]-frame[:3,3]), axis*.25])
            jacobian = np.column_stack(columns)
            error = np.r_[position, rotation*.25]
            delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + np.eye(6)*1e-6, error)
            delta *= min(1., .15 / max(float(np.max(np.abs(delta))), 1e-12))
            accepted = False
            for factor in (1., .5, .25, .1):
                candidate = angles.copy()
                candidate[active] = np.clip(angles[active] + delta*factor, self.analytic.lower[0,active], self.analytic.upper[0,active])
                pose = self.forward(candidate)
                candidate_error = np.r_[target[:3,3]-pose[:3,3], Rotation.from_matrix(target[:3,:3] @ pose[:3,:3].T).as_rotvec()*.25]
                if np.linalg.norm(candidate_error) < np.linalg.norm(error):
                    angles = candidate
                    accepted = True
                    break
            if not accepted:
                break
        self.state(seed)
        return None

    def solve_fixed(self, target, seed, joint_index, angle_rad, step_deg=1.):
        from scipy.optimize import brentq, minimize_scalar, linear_sum_assignment
        target = np.asarray(target, dtype=float)
        seed = np.asarray(seed, dtype=float)
        if not isinstance(joint_index, int) or not 0 <= joint_index < 7:
            raise ValueError('锁定关节必须为J1～J7')
        if not np.isfinite(angle_rad) or not self.analytic.lower[0, joint_index] <= angle_rad <= self.analytic.upper[0, joint_index]:
            raise ValueError('锁定角超出模型限位')
        if not .1 <= step_deg <= 3.:
            raise ValueError('固定轴搜索网格必须0.1～3度')
        if target.shape != (4, 4) or seed.shape != (7,) or not np.isfinite([*target.ravel(), *seed]).all():
            raise ValueError('非法目标或种子')
        if not np.allclose(target[3], [0, 0, 0, 1]) or not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(target[:3, :3]), 1.):
            raise ValueError('目标必须是SE(3)')
        self.state(seed)
        carriage = self.model.get_transform('arm_carriage', self.model.base_link)
        probe_to_link = np.eye(4)
        probe_to_link[2, 3] = -PROBE_OFFSET_M
        old_target = np.linalg.inv(carriage) @ target @ probe_to_link @ self.model.get_transform('left_tool0', 'left_link7')

        def evaluate(swivel):
            return self.analytic.solve(0, old_target, float(swivel), seed)

        solutions = []

        def retain(swivel, angles):
            error = abs(float(angles[joint_index] - angle_rad))
            if error > 1e-6 or any(np.max(np.abs(angles - row['joints'])) < 1e-5 for row in solutions):
                return
            actual = self.forward(angles)
            position_error = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
            rotation_error = float(Rotation.from_matrix(actual[:3, :3].T @ target[:3, :3]).magnitude())
            if position_error <= 2e-6 and rotation_error <= 2e-6:
                solutions.append(dict(joints=angles.copy(), swivel_rad=float(swivel),
                    joint_distance_rad=float(np.linalg.norm(angles - seed)), position_error_m=position_error,
                    rotation_error_rad=rotation_error, fixed_joint_index=joint_index,
                    fixed_angle_rad=float(angle_rad), fixed_error_rad=error))

        original = self.analytic.swivel(0, seed)
        for angles in evaluate(original):
            retain(original, angles)
        grid = np.deg2rad(np.linspace(-180., 180., round(360 / step_deg) + 1))
        sampled = [evaluate(value) for value in grid]
        for swivel, candidates in zip(grid, sampled):
            for angles in candidates:
                retain(swivel, angles)
        for index in range(len(grid) - 1):
            left, right = sampled[index:index + 2]
            if not len(left) or not len(right):
                continue
            distances = np.linalg.norm(left[:, None] - right[None, :], axis=2)
            first_indices, second_indices = linear_sum_assignment(distances)
            for first, second in zip(first_indices, second_indices):
                if np.max(np.abs(left[first] - right[second])) > np.deg2rad(25.):
                    continue
                low, high = grid[index:index + 2]
                anchor = (left[first] + right[second]) / 2

                def residual(swivel):
                    candidates = evaluate(swivel)
                    if not len(candidates):
                        raise ValueError('该分支在区间内不可用')
                    angles = candidates[np.argmin(np.linalg.norm(candidates - anchor, axis=1))]
                    if np.max(np.abs(angles - anchor)) > np.deg2rad(30.):
                        raise ValueError('解析分支发生跳变')
                    return float(angles[joint_index] - angle_rad), angles

                left_error = float(left[first, joint_index] - angle_rad)
                right_error = float(right[second, joint_index] - angle_rad)
                if abs(left_error) <= 1e-6 and abs(right_error) <= 1e-6:
                    continue
                try:
                    if left_error * right_error < 0:
                        root = brentq(lambda value: residual(value)[0], low, high, xtol=1e-12)
                        retain(root, residual(root)[1])
                    elif min(abs(left_error), abs(right_error)) < np.deg2rad(.2):
                        minimum = minimize_scalar(lambda value: residual(value)[0] ** 2,
                            bounds=(low, high), method='bounded', options={'xatol': 1e-12})
                        retain(minimum.x, residual(minimum.x)[1])
                except ValueError:
                    continue
        solutions.sort(key=lambda row: row['joint_distance_rad'])
        self.state(seed)
        return solutions
