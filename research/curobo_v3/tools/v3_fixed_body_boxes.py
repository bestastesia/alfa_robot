import copy

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from v3_predicted_selected_lm import PredictedSelectedLmPlanner


def disable_fixed_body_spheres(robot):
    result = copy.deepcopy(robot)
    for sphere in result['kinematics']['collision_spheres']['base_link']:
        sphere['radius'] = -1.
    return result


def obb_penetration(centers, rotations, halves, obstacle_centers, obstacle_rotations,
                    obstacle_halves, tolerance=0.):
    axes_a = rotations.transpose(-1, -2).unsqueeze(-3)
    axes_b = obstacle_rotations.transpose(-1, -2).expand(*centers.shape[:-1], -1, -1, -1)
    axes_a = axes_a.expand_as(axes_b)
    cross = torch.linalg.cross(axes_a.unsqueeze(-2), axes_b.unsqueeze(-3), dim=-1)
    axes = torch.cat((axes_a, axes_b, cross.flatten(-3, -2)), dim=-2)
    norms = torch.linalg.vector_norm(axes, dim=-1)
    axes = axes / norms.clamp(min=1e-8).unsqueeze(-1)
    delta = centers.unsqueeze(-2) - obstacle_centers
    distance = (axes * delta.unsqueeze(-2)).sum(-1).abs()
    projected_a = torch.matmul(axes, rotations.unsqueeze(-3)).abs()
    projected_b = torch.matmul(axes, obstacle_rotations).abs()
    radii = (projected_a * halves.unsqueeze(-2).unsqueeze(-2)).sum(-1)
    radii = radii + (projected_b * obstacle_halves.unsqueeze(-2)).sum(-1)
    overlap = torch.where(norms > 1e-7, radii - distance - tolerance, float('inf'))
    return overlap.amin(-1).clamp(min=0)


class FixedBodyWorld:
    def __init__(self, base_scene, scene, names, device='cuda'):
        self.names = list(names)
        self.device = device
        self.centers, self.rotations, self.halves = self._tensors(base_scene.cuboid)
        self.update(scene)

    def _tensors(self, boxes):
        centers = np.asarray([box.pose[:3] for box in boxes], dtype=np.float32).reshape(-1, 3)
        rotations = np.asarray([Rotation.from_quat(np.roll(box.pose[3:], -1)).as_matrix()
                                for box in boxes], dtype=np.float32).reshape(-1, 3, 3)
        halves = np.asarray([box.dims for box in boxes], dtype=np.float32).reshape(-1, 3) / 2
        return tuple(torch.tensor(value, device=self.device) for value in (centers, rotations, halves))

    def update(self, scene):
        if scene.mesh:
            raise ValueError('Fixed body boxes require cuboid world obstacles')
        identity = tuple((box.name, tuple(box.pose), tuple(box.dims)) for box in scene.cuboid or [])
        if getattr(self, '_scene_identity', None) == identity:
            return
        self._scene_identity = identity
        self.obstacle_centers, self.obstacle_rotations, self.obstacle_halves = self._tensors(scene.cuboid or [])
        self._fixed_costs = {}

    def cost(self, values, scale=1., tolerance=0.):
        if not len(self.obstacle_centers):
            return values[..., 0] * 0
        if 'base_yaw' not in self.names:
            key = scale, tolerance
            if key not in self._fixed_costs:
                self._fixed_costs[key] = obb_penetration(self.centers, self.rotations,
                    self.halves * scale, self.obstacle_centers, self.obstacle_rotations,
                    self.obstacle_halves, tolerance).square().sum()
            return self._fixed_costs[key].expand(values.shape[:-1])
        shape = values.shape[:-1]
        centers = self.centers.expand(*shape, -1, -1)
        rotations = self.rotations.expand(*shape, -1, -1, -1)
        if 'base_yaw' in self.names:
            yaw = values[..., self.names.index('base_yaw')]
            cosine, sine = torch.cos(yaw), torch.sin(yaw)
            zero, one = torch.zeros_like(yaw), torch.ones_like(yaw)
            matrix = torch.stack((cosine, -sine, zero, sine, cosine, zero, zero, zero, one), -1).reshape(*shape, 3, 3)
            translation = torch.stack((values[..., self.names.index('base_x')],
                                       values[..., self.names.index('base_y')], zero), -1)
            centers = torch.matmul(matrix.unsqueeze(-3), centers.unsqueeze(-1)).squeeze(-1) + translation.unsqueeze(-2)
            rotations = torch.matmul(matrix.unsqueeze(-3), rotations)
        return obb_penetration(centers, rotations, self.halves * scale,
            self.obstacle_centers, self.obstacle_rotations, self.obstacle_halves, tolerance).square().sum((-1, -2))


class WorldMarginChecker:
    def __init__(self, checker, scale, tolerance):
        self._checker, self._scale, self._tolerance = checker, scale, tolerance

    def __getattr__(self, name):
        return getattr(self._checker, name)

    def get_collision_constraint(self, spheres, *args, **kwargs):
        query = spheres.clone()
        radius = spheres[..., 3]
        query[..., 3] = torch.where(radius > 0,
            (radius + self._tolerance) * self._scale - self._tolerance, radius)
        return self._checker.get_collision_constraint(query, *args, **kwargs)

    def get_bound(self, values):
        bounds = self._checker.fixed_body_original_bound(values)
        extra = self._checker.fixed_body_world.cost(values, self._scale, self._tolerance)
        return bounds + extra.unsqueeze(-1).expand_as(bounds) / bounds.shape[-1]


class InflatedPlanningPlanner(PredictedSelectedLmPlanner):
    def __init__(self, radius_scale=1.1, assets=None):
        if not np.isfinite(radius_scale) or radius_scale < 1:
            raise ValueError('Planning radius scale must be finite and >= 1')
        super().__init__(assets)
        self.radius_scale = radius_scale
        self.robot = disable_fixed_body_spheres(self.robot)
        self.mobile_robot = disable_fixed_body_spheres(self.mobile_robot)
        self.persistent_checkers.clear()

    def collision_checker(self, boxes, payload=False, mobile=False):
        checker = super().collision_checker(boxes, payload, mobile)
        scene = self.scene(boxes, mobile=mobile)
        if not hasattr(checker, 'fixed_body_world'):
            layer = checker.base_cuboid_layer
            local = getattr(layer, 'local_layer', layer)
            checker.fixed_body_world = FixedBodyWorld(local.scene, scene, checker.kinematics.joint_names)
            checker.fixed_body_original_bound = checker.get_bound
            def bounds(values):
                result = checker.fixed_body_original_bound(values)
                extra = checker.fixed_body_world.cost(values, tolerance=.001)
                return result + extra.unsqueeze(-1).expand_as(result) / result.shape[-1]
            checker.get_bound = bounds
        else:
            checker.fixed_body_world.update(scene)
        return checker

    def _inflated_search(self, search, *args, **kwargs):
        validity = copy.copy(args[4])
        validity.scene_penetration_tolerance_m = .001
        validity.contact_penetration_tolerance_m = .001
        validity.checker = WorldMarginChecker(validity.checker, self.radius_scale, .001)
        arguments = list(args)
        arguments[4] = validity
        return search(*arguments, **kwargs)

    def verified_rrt(self, *args, **kwargs):
        return self._inflated_search(super().verified_rrt, *args, **kwargs)

    def search_path(self, *args, **kwargs):
        return self._inflated_search(super().search_path, *args, **kwargs)
