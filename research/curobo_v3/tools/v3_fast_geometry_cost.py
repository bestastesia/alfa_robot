from dataclasses import dataclass

import numpy as np
import torch
from curobo._src.rollout.cost_manager.cost_manager_robot import RobotCostManager
from curobo._src.rollout.cost_manager.cost_manager_robot_cfg import RobotCostManagerCfg


WEIGHTS = np.array([np.deg2rad(15) / .1] + [1.] * 14)


class ProjectCostManager(RobotCostManager):
    checker = None
    ground_indices = None

    def compute_costs(self, state, cost_collection=None, goal=None, **kwargs):
        result = super().compute_costs(state, cost_collection, goal, **kwargs)
        robot_state = state.cuda_robot_model_state
        spheres = robot_state.robot_spheres
        if not hasattr(self, '_project_base_layer'):
            self._project_base_layer = self.checker.base_cuboid_layer
            self._ground_indices_tensor = torch.as_tensor(self.ground_indices, device=spheres.device, dtype=torch.long)
        base = self._project_base_layer.cost(robot_state).sum(-1)
        selected = spheres.index_select(-2, self._ground_indices_tensor)
        ground = torch.where(selected[..., 3] > 0,
            torch.relu(selected[..., 3] - selected[..., 2] + .002), 0.).square().sum(-1)
        extra = 10000. * (base + ground + self._project_base_layer.fixed_world_cost)
        reference = result.values[0]
        result.add(extra.reshape(reference.shape[0], reference.shape[1], 1).expand_as(reference)
                   / reference.shape[-1], 'project_base_and_ground')
        return result


class GeometricCostManager(ProjectCostManager):
    geometry_scale = 100.

    def compute_costs(self, state, cost_collection=None, goal=None, **kwargs):
        result = super().compute_costs(state, cost_collection, goal, **kwargs)
        values = state.joint_state.position
        if not hasattr(self, '_geometry_weights_tensor'):
            self._geometry_weights_tensor = torch.as_tensor(WEIGHTS, device=values.device, dtype=values.dtype)
            self._geometry_scale_tensor = torch.tensor(self.geometry_scale, device=values.device, dtype=values.dtype)
        weights = self._geometry_weights_tensor
        delta = (values[:, 1:] - values[:, :-1]) * weights
        lengths = torch.sqrt(delta.square().sum(-1) + 1e-10)
        tangents = delta / lengths.unsqueeze(-1)
        bend = (tangents[:, 1:] - tangents[:, :-1]).square().sum(-1).mean(-1)
        reverse = (delta.abs().sum(1) - ((values[:, -1] - values[:, 0]) * weights).abs()).clamp(min=0).sum(-1)
        tcp_bend = torch.zeros_like(reverse)
        spin = torch.zeros_like(reverse)
        for side in ('left', 'right'):
            pose = state.tool_poses.to_dict()[side + '_tool0']
            positions = pose.position.reshape(values.shape[0], values.shape[1], 3)
            steps = positions[:, 1:] - positions[:, :-1]
            directions = steps / torch.sqrt(steps.square().sum(-1, keepdim=True) + 1e-10)
            tcp_bend += (directions[:, 1:] - directions[:, :-1]).square().sum(-1).mean(-1)
            quaternions = pose.quaternion.reshape(values.shape[0], values.shape[1], 4)
            following = torch.where((quaternions[:, 1:] * quaternions[:, :-1]).sum(-1, keepdim=True) < 0,
                                    -quaternions[:, 1:], quaternions[:, 1:])
            changes = following - quaternions[:, :-1]
            tcp_bend += 100 * (changes[:, 1:] - changes[:, :-1]).square().sum(-1).mean(-1)
            spin += torch.sqrt(4 * changes.square().sum(-1) + 1e-10).sum(-1)
        score = (.5 * lengths.sum(-1) + .15 * reverse + 30 * bend + 40 * tcp_bend + .3 * spin)
        score = score * self._geometry_scale_tensor
        result.add((score[:, None, None] / values.shape[1]).expand(result.values[0].shape), 'geometric_path_shape')
        return result


@dataclass
class GeometricCostConfig(RobotCostManagerCfg):
    @staticmethod
    def create(*args, **kwargs):
        base = RobotCostManagerCfg.create(*args, **kwargs)
        data = args[0] if args else kwargs['data_dict']
        config = GeometricCostConfig(**vars(base))
        config.class_type = GeometricCostManager if data.get('_project_geometric_path_shape', False) else ProjectCostManager
        return config
