import hashlib
import json
import math

import numpy as np

from ipc_left_prefix.smooth_timing import exact_peaks, rt_curves


def selected_endpoint(row, locked=None):
    if row is None:
        return np.zeros(7), dict(kind='all_seven_zero', saved_lock=None if locked is None else dict(locked))
    goal = np.asarray(row['joints'], dtype=float)
    if goal.shape != (7,) or not np.isfinite(goal).all():
        raise ValueError('所选解必须为七轴有限数值')
    if locked is not None and abs(goal[locked['joint_index']] - locked['angle_rad']) > 1e-6:
        raise ValueError('所选解不满足保存的锁定角，请重新求解')
    return goal.copy(), dict(kind='selected_ik', endpoint_lock=None if locked is None else dict(locked))


def interpolate_motion(start, goal, speed_deg_s=10.):
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    if start.shape != (7,) or goal.shape != (7,) or not np.isfinite([start, goal]).all():
        raise ValueError('需要七轴有限数值')
    if not 0.1 <= speed_deg_s <= 15:
        raise ValueError('速度必须0.1～15度/秒')
    delta = goal - start
    distance = float(np.max(np.abs(delta)))
    duration = max(.5, 1.875 * distance / math.radians(speed_deg_s),
        math.sqrt(5.773503 * distance / math.radians(30)),
        (60 * distance / math.radians(150)) ** (1 / 3))
    duration = math.ceil(duration * 1.01 * 10) / 10
    if duration > 60:
        raise ValueError('插值轨迹超过60秒，拒绝发送')
    points = []
    for index in range(round(duration * 10) + 1):
        stamp = index * .1
        fraction = stamp / duration
        position = start + delta * (10 * fraction**3 - 15 * fraction**4 + 6 * fraction**5)
        velocity = delta * (30 * fraction**2 - 60 * fraction**3 + 30 * fraction**4) / duration
        acceleration = delta * (60 * fraction - 180 * fraction**2 + 120 * fraction**3) / duration**2
        points.append(dict(time_s=stamp, positions=position.tolist(), velocities=velocity.tolist(), accelerations=acceleration.tolist()))
    for point, pose in [(points[0], start), (points[-1], goal)]:
        point.update(positions=pose.tolist(), velocities=[0.] * 7, accelerations=[0.] * 7)
    peaks = exact_peaks(points)
    if any(peak > limit + 1e-5 for peak, limit in zip(peaks, [speed_deg_s, 30, 150])):
        raise ValueError('实际插值速度/加速度/jerk超限')
    return points, dict(duration_s=duration, points=len(points), speed_limit_deg_s=speed_deg_s,
        peak_velocity_deg_s=peaks[0], peak_acceleration_deg_s2=peaks[1], peak_jerk_deg_s3=peaks[2])


class ProbeCollision:
    def __init__(self):
        import torch
        import yaml
        from curobo.scene import Scene, Cuboid
        from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
        from probe_touch_ik import ROOT
        from v3_batched_loaded_search import GpuValidity
        from v3_base_cuboid_layer import BaseCuboidLayer
        from v3_base_mesh_layer import replace_arm_base_pairs
        self.torch = torch
        self.robot = yaml.safe_load((ROOT / 'generated/zero_model_candidate_20261018/fixed.yaml').read_text())
        replace_arm_base_pairs(self.robot)
        self.checker = RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(
            robot_config=self.robot, scene_model=Scene(cuboid=[Cuboid(name='remote_unused', dims=[.01]*3, pose=[100,100,100,1,0,0,0])]),
            n_cuboids=1, n_meshes=0, collision_activation_distance=0.))
        self.checker.base_mesh_layer = BaseCuboidLayer(self.checker, self.robot,
            ROOT / 'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json')
        self.names = list(self.robot['kinematics']['cspace']['joint_names'])
        self.home = json.loads((ROOT / 'config/rezero_home.json').read_text())
        self.validity = GpuValidity(self.checker, check_ground=True, joint_names=self.names)

    def mask(self, rows):
        values = np.zeros((len(rows), len(self.names)))
        for index, name in enumerate(self.names):
            if name == 'updown':
                values[:, index] = -.3
            elif name.startswith('left_joint'):
                values[:, index] = np.asarray(rows)[:, int(name[-1]) - 1]
            elif name.startswith('right_joint'):
                values[:, index] = np.deg2rad(self.home['right_deg'][int(name[-1]) - 1])
        return self.validity.mask(self.torch.tensor(values, device='cuda', dtype=self.torch.float32)).cpu().numpy()

    def check_motion(self, points):
        result = self.inspect_motion(points)
        if not result['collision_passed']:
            raise ValueError(f"关节插值路径在{result['first_collision_s']:.3f}秒碰撞；未改用其他路径，禁止发送")
        return result['collision_samples']

    def inspect_motion(self, points):
        curves = rt_curves(points)
        duration = points[-1]['time_s']
        stamps = np.linspace(0, duration, math.ceil(duration * 200) + 1)
        rows = np.column_stack([curve(stamps) for curve in curves])
        valid = self.mask(rows)
        failed = np.flatnonzero(~valid)
        return dict(collision_samples=len(stamps), collision_passed=bool(valid.all()),
            first_collision_s=None if not len(failed) else float(stamps[failed[0]]),
            collision_frames=len(failed))


def motion_payload(points, report, samples, target, collision_result=None, motion_context=None):
    from ipc_left_prefix.coordinate_mapping import EXECUTION_REVISION, model_to_motor, POLICY
    report = dict(report, collision_checked=True, collision_samples=samples, left_task=True,
        collision_scope='模型自碰撞/底盘cuboid/地面；右臂为Home参考；不含实测外部障碍或测试杆标定', within_duration=True)
    if collision_result is not None:
        report.update(collision_result, collision_advisory=True)
    motor_points = [dict(point, positions=model_to_motor(point['positions']).tolist()) for point in points]
    payload = dict(task='probe_touch', points=motor_points, report=report, target_tcp=target.tolist(),
        fixed_updown_m=-.3, tcp_in_left_link7=[0.,0.,.070], coordinate_revision=EXECUTION_REVISION,
        point_coordinate_frame='motor', motor_minus_model_deg=POLICY['motor_minus_model_deg'])
    if motion_context is not None:
        payload['motion_context'] = motion_context
    payload['preview_id'] = hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()
    return payload
