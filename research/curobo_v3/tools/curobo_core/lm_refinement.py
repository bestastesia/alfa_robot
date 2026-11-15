from dataclasses import dataclass
from functools import lru_cache
import time

import numpy as np
from scipy.interpolate import BSpline
from scipy.spatial.transform import Rotation
import torch

from curobo.types import JointState
from .contracts import ACTIVE_JOINTS


@dataclass(frozen=True)
class CartesianReference:
    start: np.ndarray
    end: np.ndarray
    attachment: np.ndarray

    def __post_init__(self):
        for name in ('start', 'end', 'attachment'):
            value = np.asarray(getattr(self, name), dtype=float).copy()
            if value.shape != (4, 4) or not np.isfinite(value).all():
                raise ValueError(f'{name} must be a finite 4x4 transform')
            if not np.allclose(value[3], [0, 0, 0, 1], atol=1e-7):
                raise ValueError(f'{name} has invalid homogeneous row')
            rotation = value[:3, :3]
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5):
                raise ValueError(f'{name} must have a proper rotation')
            value.setflags(write=False)
            object.__setattr__(self, name, value)
        if np.linalg.norm(self.end[:3, 3] - self.start[:3, 3]) < 1e-7:
            raise ValueError('Cartesian reference displacement must be nonzero')


def validate_inputs(checker, rows, references, validity, count, iterations):
    rows = np.asarray(rows, dtype=np.float64)
    if rows.ndim != 2 or rows.shape[1] != 15 or len(rows) < 2 or not np.isfinite(rows).all():
        raise ValueError('LM refinement requires finite [N,15] m/rad rows')
    if tuple(checker.kinematics.joint_names) != tuple(ACTIVE_JOINTS):
        raise ValueError('LM refinement joint order mismatch')
    if validity.checker is not checker or not getattr(checker, 'guidance_payload', False):
        raise ValueError('LM refinement requires the current loaded phase checker')
    if set(references) != {'left', 'right'} or any(not isinstance(item, CartesianReference) for item in references.values()):
        raise ValueError('LM refinement currently requires two independent arm references')
    if not isinstance(count, int) or count < 4 or not isinstance(iterations, int) or iterations < 1:
        raise ValueError('Invalid control count or iteration budget')
    return rows


@lru_cache(maxsize=8)
def basis_matrices(count, samples, device):
    grid = np.linspace(0, 1, samples)
    spline = BSpline(np.arange(-3, count + 6) / (count + 2), np.eye(count + 5), 3)
    return tuple(torch.tensor(spline(grid, nu=order), device=device, dtype=torch.float32) for order in (0, 2, 3))


def quaternion_product(first, second):
    return torch.cat((first[..., :1] * second[..., :1] - (first[..., 1:] * second[..., 1:]).sum(-1, keepdim=True),
        first[..., :1] * second[..., 1:] + second[..., :1] * first[..., 1:]
        + torch.linalg.cross(first[..., 1:], second[..., 1:])), -1)


def reference_residuals(point, quaternion, reference):
    origin, direction, offset, attachment_rotation, first, last = reference
    cross = 2 * torch.linalg.cross(quaternion[:, 1:], offset.expand_as(quaternion[:, 1:]))
    center = point + offset + quaternion[:, :1] * cross + torch.linalg.cross(quaternion[:, 1:], cross)
    fraction = ((center - origin) * direction).sum(-1) / (direction @ direction)
    line_error = center - origin - fraction[:, None] * direction
    angle = torch.acos((first @ last).clamp(-.999999, .999999))
    progress = fraction.clamp(0, 1)[:, None]
    desired = (torch.sin((1 - progress) * angle) * first + torch.sin(progress * angle) * last) / torch.sin(angle)
    actual = quaternion_product(quaternion, attachment_rotation.expand_as(quaternion))
    orientation_error = torch.minimum((actual - desired).square().sum(-1), (actual + desired).square().sum(-1))
    return fraction, line_error, orientation_error, actual, desired


def refine_lm_path(checker, rows, references, validity, fit_path, count=64, iterations=32):
    rows = validate_inputs(checker, rows, references, validity, count, iterations)
    def clock():
        torch.cuda.synchronize()
        return time.perf_counter()
    began = clock()
    controls, start, goal, fit, curve = fit_path(dict(positions=rows), count)
    device = checker.kinematics.get_joint_limits().position.device
    matrices = basis_matrices(count, 257, str(device))
    initial = torch.tensor(curve(np.linspace(0, 1, 257)), device=device, dtype=torch.float32)
    boundary = torch.tensor(np.vstack((np.tile(start, (3, 1)), np.tile(goal, (3, 1)))), device=device, dtype=torch.float32)
    inner = torch.tensor(controls[:-1], device=device, dtype=torch.float32, requires_grad=True)
    tensors = {}
    for side, item in references.items():
        first = np.roll(Rotation.from_matrix(item.start[:3, :3]).as_quat(), 1)
        last = np.roll(Rotation.from_matrix(item.end[:3, :3]).as_quat(), 1)
        if first @ last < 0:last = -last
        values = (item.start[:3, 3], item.end[:3, 3] - item.start[:3, 3], item.attachment[:3, 3],
                  np.roll(Rotation.from_matrix(item.attachment[:3, :3]).as_quat(), 1), first, last)
        tensors[side] = tuple(torch.tensor(value.copy(), device=device, dtype=torch.float32) for value in values)
    checker.setup_batch_tensors(1, 257)
    ground_indices = [index for index in range(checker.kinematics.total_spheres) if index not in validity.ground_exempt_sphere_indices]
    optimizer = torch.optim.LBFGS([inner], lr=.5, max_iter=iterations, max_eval=iterations * 2,
        line_search_fn='strong_wolfe', tolerance_grad=1e-8, tolerance_change=1e-10)
    evaluations = 0
    def closure():
        nonlocal evaluations
        optimizer.zero_grad()
        evaluations += 1
        coefficients = torch.cat((boundary[:3], inner, boundary[3:]))
        positions = matrices[0] @ coefficients
        state = checker.kinematics.compute_kinematics(JointState.from_position(positions[None], joint_names=list(ACTIVE_JOINTS)))
        poses = state.tool_poses.to_dict()
        progress = []
        loss = positions.sum() * 0
        for side in references:
            pose = poses[side + '_tool0']
            fraction, error, orientation, _, _ = reference_residuals(pose.position.reshape(-1, 3), pose.quaternion.reshape(-1, 4), tensors[side])
            loss = loss + 1e6 * error.square().mean() + 1e3 * orientation.mean()
            progress.append(fraction)
        loss = loss + 1e4 * (progress[0] - progress[1]).square().mean()
        for fraction in progress:
            loss = loss + 1e3 * torch.relu(-torch.diff(fraction)).square().mean()
            loss = loss + 1e3 * (torch.relu(-fraction).square() + torch.relu(fraction - 1).square()).mean()
        loss = loss + (positions - initial).square().mean()
        loss = loss + 1e-5 * (matrices[1] @ coefficients).square().mean() + 1e-9 * (matrices[2] @ coefficients).square().mean()
        spheres = state.robot_spheres.reshape(1, 257, -1, 4)
        checker.collision_constraint.update_num_spheres(spheres.shape[2], batch_size=1, horizon=257)
        for cost in (checker.get_self_collision(spheres), checker.collision_constraint.forward(state), checker.get_bound(positions[None])):
            loss = loss + 1e6 * cost.square().mean()
        base_layer = getattr(checker, 'base_mesh_layer', None)
        if base_layer is not None:loss = loss + 1e6 * base_layer.cost(state, hard=False).square().mean()
        ground = spheres[:, :, ground_indices]
        penetration = torch.where(ground[..., 3] > 0, torch.relu(validity.ground_z + ground[..., 3] - ground[..., 2]), 0.)
        loss = loss + 1e6 * penetration.square().mean()
        loss.backward()
        return loss
    preparation_ms = (clock() - began) * 1000
    began = clock()
    with torch.enable_grad():optimizer.step(closure)
    solve_ms = (clock() - began) * 1000
    if not bool(torch.isfinite(inner).all()):raise ValueError('LM refinement returned nonfinite controls')
    with torch.no_grad():
        positions = matrices[0] @ torch.cat((boundary[:3], inner, boundary[3:]))
        state = checker.kinematics.compute_kinematics(JointState.from_position(positions[None], joint_names=list(ACTIVE_JOINTS)))
        quality = {}
        for side, reference in tensors.items():
            pose = state.tool_poses.to_dict()[side + '_tool0']
            fraction, error, _, actual, desired = reference_residuals(pose.position.reshape(-1, 3), pose.quaternion.reshape(-1, 4), reference)
            cosine = (torch.nn.functional.normalize(actual, dim=-1) * torch.nn.functional.normalize(desired, dim=-1)).sum(-1).abs().clamp(max=1)
            quality[side] = dict(max_line_error_mm=float(error.norm(dim=-1).max() * 1000),
                max_orientation_error_deg=float(torch.rad2deg(2 * torch.acos(cosine)).max()),
                backtracking=float(torch.relu(-torch.diff(fraction)).sum()))
    diagnostics_ms = (clock() - began) * 1000 - solve_ms
    output = torch.cat((inner.detach(), torch.tensor(goal[None], device=device, dtype=torch.float32)))[None].contiguous()
    endpoints = [torch.tensor(value[None], device=device, dtype=torch.float32) for value in (start, goal)]
    diagnostics = dict(method='LM native cubic joint refinement',controls=count,iterations=iterations,
        actual_iterations=int(optimizer.state[inner].get('n_iter', 0)),evaluations=evaluations,prepare_ms=preparation_ms,
        solve_ms=solve_ms,diagnostics_ms=diagnostics_ms,seed_fit=fit,quality=quality,
        fixed_joint_endpoints=True,reference_frame='current phase robot base',cost_constraints='soft penalties, not feasibility certification',
        scene_cost_penetration_tolerance_m=0.,acceptance_scene_tolerance_m=validity.scene_penetration_tolerance_m)
    return output, *endpoints, diagnostics
