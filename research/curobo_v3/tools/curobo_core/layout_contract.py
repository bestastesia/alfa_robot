"""User's 2026-11-23 box-center plane and joint-defined carry contract (CPU only)."""
from functools import lru_cache
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CARRY_DEGREES = np.array([40., -46., 0., -100., -30., -75., 90.])
DEFAULT_FRONT_DISTANCE_M = .66  # selected by distance_calibration_v2 (48 requests +360-layout necessary screen)
DEFAULT_SIDE_MIN_HEIGHT_M = .79  # selected by fixed66_global79: original19 +3 switching boundaries,21/22 passed; remaining face model unresolved


def carry_configuration(updown):
    if not np.isfinite(updown):
        raise ValueError('carry updown must be finite')
    return dict(updown=float(updown), **{
        f'{side}_joint{i}': float(np.deg2rad(angle)*sign)
        for side, sign in (('left', 1), ('right', -1))
        for i, angle in enumerate(CARRY_DEGREES, 1)})


@lru_cache(maxsize=1)
def carry_reference():
    import yourdfpy
    robot = yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
    robot.update_cfg(carry_configuration(0.))
    poses = {s: robot.get_transform(s+'_tool0', robot.base_link).copy() for s in ('left', 'right')}
    robot.update_cfg({'updown': -.1})
    axis = (robot.get_transform('left_tool0', robot.base_link)[:3, 3]-poses['left'][:3, 3])/-.1
    limit = robot.joint_map['updown'].limit
    return poses, axis, (float(limit.lower), float(limit.upper))


def carry_tool(side, updown):
    poses, axis, bounds = carry_reference()
    if not np.isfinite(updown) or not bounds[0] <= updown <= bounds[1]:
        raise ValueError('carry updown is outside the actual URDF limits')
    result = poses[side].copy()
    result[:3, 3] += axis*updown
    return result


@lru_cache(maxsize=1)
def base_front_x():
    import trimesh
    import yourdfpy
    robot = yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
    base = robot.get_transform('base_link', robot.base_link)
    fronts = []
    for collision in robot.link_map['base_link'].collisions:
        mesh = trimesh.load(collision.geometry.mesh.filename, force='mesh')
        mesh.vertices *= collision.geometry.mesh.scale
        mesh.apply_transform(base@collision.origin)
        fronts.append(float(mesh.vertices[:, 0].max()))
    return max(fronts)


def validate_pair(picks):
    if set(picks) != {'left', 'right'}:
        raise ValueError('exactly one box per arm is required')
    if any(p.side != side for side, p in picks.items()):
        raise ValueError('box assignment key must match its arm side')
    if not all(p.carry_joint_home for p in picks.values()):
        raise ValueError('this layout task requires the user-specified carry joint angles')
    if any(np.any(np.asarray(p.carry_offset_m) != 0.) for p in picks.values()):
        raise ValueError('Cartesian carry offsets cannot override the specified joint angles')
    left, right = picks['left'], picks['right']
    if abs(left.side_min_height_m-right.side_min_height_m) > 1e-12:
        raise ValueError('both boxes must use the same global side-to-top height threshold')
    if left.carry_updown_m != right.carry_updown_m:
        raise ValueError('both carry endpoints must use the same shared updown')
    if abs(left.center_m[0]-right.center_m[0]) > 1e-9:
        raise ValueError('两箱中心必须位于同一前向竖直平面（相同X）')
    if abs(left.center_m[2]-right.center_m[2]) > .6+1e-9:
        raise ValueError('两箱中心高度差不得超过0.60m')
    distance = left.center_m[0]-base_front_x()
    if not .60-1e-9 <= distance <= .90+1e-9 or abs(distance/.02-round(distance/.02)) > 1e-7:
        raise ValueError('箱心平面距底盘最前沿必须为0.60–0.90m之间的0.02m倍数')
    return dict(base_front_x_m=base_front_x(), front_distance_m=float(distance),
                center_plane_x_m=float(left.center_m[0]), max_height_difference_m=.6,
                carry_joint_degrees={s: (CARRY_DEGREES*sign).tolist() for s, sign in (('left', 1), ('right', -1))},
                carry_updown_policy='choose within URDF limits for each request; no fixed return value')
