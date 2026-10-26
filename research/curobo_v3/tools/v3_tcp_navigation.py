import heapq
import itertools
import time

import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation, RotationSpline, Slerp


def box_arrays(objects, radius):
    centers = np.asarray([item['position'] for item in objects], dtype=float).reshape(-1, 3)
    rotations = Rotation.from_quat(np.asarray([item['quaternion_wxyz'][1:] + item['quaternion_wxyz'][:1] for item in objects])).as_matrix() if objects else np.empty((0, 3, 3))
    extents = np.asarray([item['dimensions'] for item in objects], dtype=float).reshape(-1, 3) / 2 + radius
    return centers, rotations, extents


def segment_clear(start, end, boxes):
    centers, rotations, extents = boxes
    local_start = np.einsum('ni,nij->nj', np.asarray(start) - centers, rotations)
    local_delta = np.einsum('i,nij->nj', np.asarray(end) - start, rotations)
    parallel = np.abs(local_delta) < 1e-12
    divisor = np.where(parallel, 1., local_delta)
    first = (-extents - local_start) / divisor
    second = (extents - local_start) / divisor
    lower = np.where(parallel, -np.inf, np.minimum(first, second))
    upper = np.where(parallel, np.inf, np.maximum(first, second))
    separated = (parallel & (np.abs(local_start) > extents)).any(axis=1)
    enter = np.maximum(lower.max(axis=1), 0.)
    leave = np.minimum(upper.min(axis=1), 1.)
    return not bool((~separated & (enter <= leave)).any())


def route(start, goal, objects, radius, resolution=.05, budget_s=3.):
    boxes = box_arrays(objects, radius)
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    for label, point in [('起点', start), ('目标', goal)]:
        if not segment_clear(point, point, boxes):
            raise ValueError(f'{label}碰撞：TCP球体及安全余量与场景相交，请调整当前帧或球半径/目标间距')
    if segment_clear(start, goal, boxes):
        return np.stack((start, goal)), 0
    lower = np.minimum(start, goal) - [.8, .8, .6]
    upper = np.maximum(start, goal) + [.8, .8, .6]
    lower[2] = max(lower[2], radius + .001)
    origin = start.copy()
    directions = [np.asarray(delta, dtype=int) for delta in itertools.product((-1, 0, 1), repeat=3) if any(delta)]
    initial = (0, 0, 0)
    parents, costs = {initial: None}, {initial: 0.}
    queue = [(float(np.linalg.norm(goal - start)), 0., initial)]
    began = time.perf_counter()
    expanded = 0
    while queue:
        if time.perf_counter() - began > budget_s:
            raise RuntimeError('三维导航搜索超时，未输出路径')
        _, cost, key = heapq.heappop(queue)
        if cost > costs[key] + 1e-9:
            continue
        point = origin + np.asarray(key) * resolution
        expanded += 1
        if np.linalg.norm(point - goal) < resolution * 2 and segment_clear(point, goal, boxes):
            chain = [goal, point]
            while parents[key] is not None:
                key = parents[key]
                chain.append(origin + np.asarray(key) * resolution)
            path = np.asarray(chain[::-1])
            shortened = [path[0]]
            index = 0
            while index < len(path) - 1:
                next_index = next(candidate for candidate in range(len(path) - 1, index, -1) if segment_clear(path[index], path[candidate], boxes))
                shortened.append(path[next_index])
                index = next_index
            return np.asarray(shortened), expanded
        for direction in directions:
            next_key = tuple(np.asarray(key) + direction)
            target = origin + np.asarray(next_key) * resolution
            if np.any(target < lower) or np.any(target > upper):
                continue
            next_cost = cost + float(np.linalg.norm(direction)) * resolution
            if next_cost >= costs.get(next_key, float('inf')):
                continue
            if not segment_clear(point, target, boxes):
                continue
            parents[next_key], costs[next_key] = key, next_cost
            heapq.heappush(queue, (next_cost + float(np.linalg.norm(target - goal)), next_cost, next_key))
    raise RuntimeError('当前导航空间内未找到通路')


def round_route(points, boxes):
    output = [points[0]]
    for index in range(1, len(points) - 1):
        previous, corner, following = points[index - 1:index + 2]
        radius = min(.12, np.linalg.norm(previous - corner) * .25, np.linalg.norm(following - corner) * .25)
        accepted = None
        for scale in (1., .5, .25, .125):
            entry = corner + (previous - corner) * radius * scale / np.linalg.norm(previous - corner)
            exit_point = corner + (following - corner) * radius * scale / np.linalg.norm(following - corner)
            fractions = np.linspace(0, 1, 31)[:, None]
            curve = (1 - fractions) ** 2 * entry + 2 * fractions * (1 - fractions) * corner + fractions ** 2 * exit_point
            candidate = np.vstack((output[-1], curve))
            if all(segment_clear(first, second, boxes) for first, second in zip(candidate[:-1], candidate[1:])):
                accepted = curve
                break
        if accepted is None:
            raise RuntimeError('转角无法在保持碰撞约束下圆滑，停止输出；可调整球半径或导航分辨率')
        output.extend(accepted)
    output.append(points[-1])
    values = np.asarray(output)
    return values[np.r_[True, np.linalg.norm(np.diff(values, axis=0), axis=1) > 1e-10]]


def forward_route(points, initial_rotation, final_rotation, boxes, pose_validator=None):
    distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    if distances[-1] < 1e-6:
        raise ValueError('沿Z前进需要非零平移距离；原地旋转请使用原模式')
    base_tangents = np.gradient(points, distances, axis=0)
    base_tangents /= np.maximum(np.linalg.norm(base_tangents, axis=1, keepdims=True), 1e-12)
    count = max(121, int(np.ceil(distances[-1] / .005)) + 1)
    parameter = np.linspace(0., distances[-1], count)
    candidates = []
    for strength in (.15, .3, .6, 1., 1.5):
        tangents = base_tangents.copy()
        tangents[0] = initial_rotation[:, 2] * strength
        tangents[-1] = final_rotation[:, 2] * strength
        curve = CubicHermiteSpline(distances, points, tangents)
        positions = curve(parameter)
        if not all(segment_clear(first, second, boxes) for first, second in zip(positions[:-1], positions[1:])):
            continue
        tangent = curve(parameter, 1)
        norms = np.linalg.norm(tangent, axis=1)
        if np.any(norms < 1e-8):
            continue
        tangent /= norms[:, None]
        transported = [initial_rotation.copy()]
        for direction in tangent[1:]:
            previous = transported[-1]
            axis = np.cross(previous[:, 2], direction)
            sine = np.linalg.norm(axis)
            cosine = np.clip(np.dot(previous[:, 2], direction), -1., 1.)
            if sine < 1e-10:
                correction = Rotation.identity() if cosine > 0 else Rotation.from_rotvec(previous[:, 0] * np.pi)
            else:
                correction = Rotation.from_rotvec(axis / sine * np.arctan2(sine, cosine))
            transported.append(correction.as_matrix() @ previous)
        transported = np.asarray(transported)
        residual = transported[-1].T @ final_rotation
        twist = np.arctan2(residual[1, 0], residual[0, 0])
        progress = parameter / parameter[-1]
        blend = 10*progress**3 - 15*progress**4 + 6*progress**5
        rotations = transported @ Rotation.from_rotvec(np.column_stack((np.zeros(len(progress)), np.zeros(len(progress)), twist*blend))).as_matrix()
        rotations[0], rotations[-1] = initial_rotation, final_rotation
        if pose_validator is not None and not pose_validator(positions, rotations):
            continue
        length = float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())
        turn = float((Rotation.from_matrix(rotations[:-1]).inv()*Rotation.from_matrix(rotations[1:])).magnitude().sum())
        steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        curvature = np.linalg.norm(np.diff(tangent, axis=0), axis=1) / np.maximum(steps, 1e-8)
        score = length + .08*turn + .08*float(curvature.max())
        candidates.append((score, positions, rotations, strength))
    if not candidates:
        raise RuntimeError('沿Z前进的曲线候选均未通过碰撞检查；可切回原路线，未自动退回原模式')
    _, positions, rotations, strength = min(candidates, key=lambda candidate: candidate[0])
    return positions, rotations, strength


def alignment_metrics(positions, quaternions):
    positions = np.asarray(positions)
    steps = np.diff(positions, axis=0)
    lengths = np.linalg.norm(steps, axis=1)
    moving = lengths > 1e-7
    if not moving.any():
        return dict(mean_deg=0., max_deg=0., moving_segments=0)
    orientations = Rotation.from_quat(np.roll(quaternions, -1, axis=1))
    midpoint = orientations[:-1] * Rotation.from_rotvec((orientations[:-1].inv()*orientations[1:]).as_rotvec()*.5)
    direction = midpoint.as_matrix()[:, :, 2]
    angles = np.rad2deg(np.arccos(np.clip(np.sum(direction[moving]*steps[moving], axis=1)/lengths[moving], -1., 1.)))
    return dict(mean_deg=float(np.average(angles, weights=lengths[moving])), max_deg=float(angles.max()), moving_segments=int(moving.sum()))


def peak_normal_acceleration(positions, times):
    positions = np.asarray(positions, dtype=float)
    intervals = np.diff(np.asarray(times, dtype=float))
    if np.any(intervals <= 0):
        raise ValueError('Trajectory timestamps must increase')
    if len(intervals) < 2:
        return 0.
    velocities = np.diff(positions, axis=0) / intervals[:, None]
    accelerations = np.diff(velocities, axis=0) / ((intervals[:-1]+intervals[1:])*.5)[:, None]
    middle_velocity = (velocities[:-1]+velocities[1:])*.5
    directions = middle_velocity / np.maximum(np.linalg.norm(middle_velocity, axis=1, keepdims=True), 1e-12)
    normal = accelerations - np.sum(accelerations*directions, axis=1, keepdims=True)*directions
    moving = np.linalg.norm(middle_velocity, axis=1) > 1e-10
    return float(np.linalg.norm(normal[moving], axis=1).max()) if moving.any() else 0.


def timed_route(points, initial_rotation, final_rotation, speed=.25, acceleration=.5, angular_speed=45., angular_acceleration=90., dt=1/60, orientation_knots=None, normal_acceleration=None):
    if normal_acceleration is not None and normal_acceleration <= 0:
        raise ValueError('Normal acceleration limit must be positive')
    distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    total = distances[-1]
    rotation = Rotation.from_matrix(np.stack((initial_rotation, final_rotation)))
    angle = float((rotation[0].inv() * rotation[1]).magnitude())
    interpolator = RotationSpline(distances / total, Rotation.from_matrix(orientation_knots)) if orientation_knots is not None else Slerp([0., 1.], rotation)
    duration = max(.5, 1.875 * total / speed, 1.875 * angle / np.deg2rad(angular_speed))
    for attempt in range(8):
        times = np.linspace(0., duration, int(np.ceil(duration / dt)) + 1)
        fraction = times / duration
        progress = 10*fraction**3 - 15*fraction**4 + 6*fraction**5
        positions = np.column_stack([np.interp(progress * total, distances, points[:, axis]) for axis in range(3)])
        orientations = interpolator(progress)
        quaternion = np.roll(orientations.as_quat(), 1, axis=1)
        timestep = times[1] - times[0]
        velocities = np.diff(positions, axis=0) / timestep
        peak_speed = float(np.linalg.norm(velocities, axis=1).max())
        peak_acceleration = float(np.linalg.norm(np.diff(velocities, axis=0) / timestep, axis=1).max()) if len(velocities) > 1 else 0.
        angular_velocity = orientations[:-1].apply((orientations[:-1].inv()*orientations[1:]).as_rotvec()) / timestep
        peak_angular_speed = float(np.linalg.norm(angular_velocity, axis=1).max())
        peak_angular_acceleration = float(np.linalg.norm(np.diff(angular_velocity, axis=0)/timestep, axis=1).max()) if len(angular_velocity) > 1 else 0.
        peak_normal = peak_normal_acceleration(positions, times)
        factor = max(peak_speed/speed, np.sqrt(peak_acceleration/acceleration), peak_angular_speed/np.deg2rad(angular_speed), np.sqrt(peak_angular_acceleration/np.deg2rad(angular_acceleration)))
        if factor <= 1.0001:
            if normal_acceleration is not None and peak_normal > normal_acceleration:
                normal_scale = np.sqrt(peak_normal/normal_acceleration)*1.01
                times *= normal_scale
                duration *= normal_scale
                peak_speed /= normal_scale
                peak_acceleration /= normal_scale**2
                peak_normal /= normal_scale**2
                peak_angular_speed /= normal_scale
                peak_angular_acceleration /= normal_scale**2
            return dict(positions=positions.tolist(), quaternions_wxyz=quaternion.tolist(), times_s=times.tolist(), duration_s=duration, peak_speed_m_s=peak_speed, peak_acceleration_m_s2=peak_acceleration, peak_normal_acceleration_m_s2=peak_normal, peak_angular_speed_deg_s=float(np.rad2deg(peak_angular_speed)), peak_angular_acceleration_deg_s2=float(np.rad2deg(peak_angular_acceleration)))
        duration *= factor * 1.03
    raise RuntimeError('时间参数化未达到设定运动限制')
