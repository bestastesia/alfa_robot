import hashlib
import json
from pathlib import Path

import numpy as np
import torch
from curobo.types import JointState
from curobo_core.scene import digest
from v3_wall_ik_benchmark import ACTIVE_JOINTS


ROOT = Path(__file__).resolve().parents[1]
CUBOID_PATH = ROOT / 'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json'


def atlas_path(planner):
    return ROOT / ('generated/placement_analytic_atlas/top.npz' if planner.top_grasp else
                   'generated/side_upward_placement/atlas/side.npz')


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def planner_identity(planner, boxes):
    return digest({'scene': planner.snapshot.geometry_key, 'model': planner.model_id,
                   'robot': planner.robot, 'mobile_robot': planner.mobile_robot,
                   'tasks': boxes, 'targets': planner.target_poses,
                   'atlas': file_hash(atlas_path(planner)), 'cuboids': file_hash(CUBOID_PATH),
                   'policy': file_hash(ROOT / 'config/placement_task_policy.json'),
                   'payload': planner.box_fit, 'contact_grid': {'updown': .05, 'psi_deg': 10}})


def validate_atlas(planner):
    path = atlas_path(planner)
    identity = digest((file_hash(path), planner.model_id, planner.target_poses))
    if getattr(planner, '_validated_atlas', None) == identity:
        return
    checker = planner.checker(planner.selected_task)
    rows = np.load(path, allow_pickle=False)['joints']
    if rows.ndim != 2 or rows.shape[1] != 15 or len(rows) == 0 or not np.isfinite(rows).all():
        raise ValueError('invalid placement atlas')
    maximum_position = 0.0
    maximum_angle = 0.0
    for offset in range(0, len(rows), 1024):
        values = torch.tensor(rows[offset:offset + 1024], device='cuda', dtype=torch.float32)
        poses = checker.kinematics.compute_kinematics(JointState.from_position(
            values[None], joint_names=ACTIVE_JOINTS)).tool_poses.to_dict()
        for frame, target in planner.target_poses.items():
            position = poses[frame].position.reshape(-1, 3)
            quaternion = poses[frame].quaternion.reshape(-1, 4)
            target_position = torch.tensor(target['position'], device='cuda')
            target_quaternion = torch.tensor(target['quaternion'], device='cuda')
            maximum_position = max(maximum_position, float(torch.linalg.vector_norm(position - target_position, dim=-1).max()))
            angle = 2 * torch.acos(torch.clamp((quaternion * target_quaternion).sum(-1).abs(), max=1))
            maximum_angle = max(maximum_angle, float(angle.max()))
    if maximum_position > .002 or maximum_angle > np.deg2rad(1):
        raise ValueError(f'placement policy/atlas mismatch: {maximum_position * 1000:.4f}mm, {np.rad2deg(maximum_angle):.4f}deg')
    planner._validated_atlas = identity
    planner.atlas_validation = {'path': str(path), 'sha256': file_hash(path), 'rows': len(rows),
                                'position_error_mm': maximum_position * 1000,
                                'orientation_error_deg': float(np.rad2deg(maximum_angle))}
