import heapq
import math
import time
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import torch
import trimesh
from scipy.spatial.transform import Rotation
from curobo.types import JointState
from curobo_core.adapter import matrix_pose, pose_matrix
from curobo_core.conveyor import DEFAULT_CONVEYOR_MESH
from curobo_core.distance_metric import joint_distance_weights
from curobo_core.scene import SceneObject
from v3_batched_loaded_search import GpuValidity
from v3_conveyor_cycle_planner import ConveyorCyclePlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS


class CuboidConveyorPlanner(ConveyorCyclePlanner):
    def __init__(self, args):
        super().__init__(args)
        mesh = trimesh.load(DEFAULT_CONVEYOR_MESH, force='mesh')
        bounds = mesh.bounds
        corners = np.array([[x, y, z] for x in bounds[:, 0] for y in bounds[:, 1] for z in bounds[:, 2]])
        if len(mesh.vertices) != 8 or len(mesh.faces) != 12 or not np.isclose(mesh.volume, np.prod(mesh.extents)):
            raise ValueError('Conveyor mesh is not the expected exact cuboid')
        if not all(np.min(np.linalg.norm(corners - vertex, axis=1)) < 1e-7 for vertex in mesh.vertices):
            raise ValueError('Conveyor vertices differ from cuboid corners')
        center = bounds.mean(axis=0)
        objects = []
        for part in self.original_snapshot.meshes:
            transform = pose_matrix(part.pose)
            transform[:3, 3] += transform[:3, :3] @ center
            objects.append(SceneObject(part.object_id, tuple(mesh.extents), matrix_pose(transform)))
        self.conveyor_cuboids = tuple(objects)
        self.original_snapshot = replace(self.original_snapshot, meshes=(), objects=self.original_snapshot.objects + self.conveyor_cuboids)
        self.snapshot = self.original_snapshot

    def configure_task(self, boxes):
        result = super().configure_task(boxes)
        objects = tuple(replace(part, pose=replace(part.pose, position=(part.pose.position[0] - self.base_advance, *part.pose.position[1:]))) for part in self.conveyor_cuboids)
        self.snapshot = replace(self.snapshot, objects=self.snapshot.objects + objects)
        return result

    def full_cycle(self, boxes, progress=lambda text: None):
        result = super().full_cycle(boxes, progress)
        result['conveyor_collision_validation'] = 'enabled: two exact cuboids converted from frozen STL; no navigation path validation'
        result['conveyor_geometry'] = 'cuboid'
        return result


class PriorityContactPlanner(CuboidConveyorPlanner):
    contact_batch_size = 256

    def build_contacts(self, boxes):
        began = time.perf_counter()
        checker = self.checker(boxes)
        validity = GpuValidity(checker, check_ground=True)
        lower, upper = [value.cpu().numpy() for value in checker.kinematics.get_joint_limits().position]
        targets = self.contact_poses_for_task(boxes)
        transforms = {}
        for side in ['left', 'right']:
            pose = targets[side + '_tool0']
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_quat(np.roll(pose['quaternion'], -1)).as_matrix()
            matrix[:3, 3] = pose['position']
            transforms[side] = matrix
        weights = np.array(joint_distance_weights(ACTIVE_JOINTS))
        grids = []
        raw_count = 0
        enumeration_started = time.perf_counter()
        for updown in np.round(np.arange(-1., .0001, .05), 2):
            if updown < lower[0] - 1e-7 or updown > upper[0] + 1e-7:
                continue
            reference = self.home_values.copy()
            reference[0] = updown
            self.fk(reference)
            carriage = np.linalg.inv(self.fk_robot.get_transform('arm_carriage', self.fk_robot.base_link))
            arms = []
            for arm, side in enumerate(['left', 'right']):
                offset = 1 + arm * 7
                values = []
                for swivel in range(-180, 180, 10):
                    branches = self.analytic.solve(arm, carriage @ transforms[side], math.radians(swivel), reference[offset:offset + 7])
                    if len(branches) >= 16:
                        raise RuntimeError('Analytic bridge capacity reached')
                    for branch in branches:
                        if np.all(branch >= lower[offset:offset + 7] - 1e-7) and np.all(branch <= upper[offset:offset + 7] + 1e-7):
                            values.append(branch)
                values = np.array(values).reshape(-1, 7)
                distances = np.sum(((values - self.home_values[offset:offset + 7]) * weights[offset:offset + 7]) ** 2, axis=1)
                order = np.argsort(distances, kind='stable')
                arms.append((values[order], distances[order]))
            count = len(arms[0][0]) * len(arms[1][0])
            raw_count += count
            if count:
                grids.append((float(updown), arms, float(((updown - self.home_values[0]) * weights[0]) ** 2)))
        enumeration_ms = (time.perf_counter() - enumeration_started) * 1000
        heap = []
        for grid_index, (_, arms, lift_cost) in enumerate(grids):
            for left_index, cost in enumerate(arms[0][1]):
                heapq.heappush(heap, (float(cost + arms[1][1][0] + lift_cost), grid_index, left_index, 0))
        accepted = {}
        checked = 0
        filter_ms = 0.
        cutoff = None
        while heap:
            candidates = []
            while heap and len(candidates) < self.contact_batch_size:
                cost, grid_index, left_index, right_index = heapq.heappop(heap)
                updown, arms, lift_cost = grids[grid_index]
                if cutoff is not None and cost > cutoff + 1e-6:
                    heapq.heappush(heap, (cost, grid_index, left_index, right_index))
                    break
                candidates.append([updown, *arms[0][0][left_index], *arms[1][0][right_index]])
                next_index = right_index + 1
                if next_index < len(arms[1][0]):
                    heapq.heappush(heap, (float(arms[0][1][left_index] + arms[1][1][next_index] + lift_cost), grid_index, left_index, next_index))
            if not candidates:
                break
            values = np.array(candidates, dtype=np.float32)
            checked += len(values)
            filter_started = time.perf_counter()
            tensor = torch.tensor(values, device='cuda')
            mask = validity.mask(tensor)
            fk = checker.kinematics.compute_kinematics(JointState.from_position(tensor[None], joint_names=ACTIVE_JOINTS))
            for side in ['left', 'right']:
                pose = fk.tool_poses.to_dict()[side + '_tool0']
                target = targets[side + '_tool0']
                mask &= torch.linalg.vector_norm(pose.position.reshape(-1, 3) - torch.tensor(target['position'], device='cuda'), dim=-1) <= .002
                mask &= (pose.quaternion.reshape(-1, 4) * torch.tensor(target['quaternion'], device='cuda')).sum(-1).abs() >= math.cos(math.radians(1) / 2)
            for value in values[mask.cpu().numpy()]:
                key = tuple(np.round(value, 6))
                accepted.setdefault(key, value)
            torch.cuda.synchronize()
            filter_ms += (time.perf_counter() - filter_started) * 1000
            if len(accepted) >= 64:
                valid_values = np.array(list(accepted.values()))
                costs = np.sum(((valid_values - self.home_values) * weights) ** 2, axis=1)
                cutoff = float(np.partition(costs, 63)[63])
                if not heap or heap[0][0] > cutoff + 1e-6:
                    break
        if not accepted:
            raise RuntimeError('No legal analytic contacts')
        values = np.array(list(accepted.values()))
        keys = np.round(values, 6)
        canonical = np.lexsort(tuple(keys[:, index] for index in range(14, -1, -1)))
        values = values[canonical]
        costs = np.linalg.norm((values - self.home_values) * weights, axis=1)
        order = np.argsort(costs, kind='stable')[:64]
        selected = values[order]
        self.priority_selection_report = dict(raw_pair_count=raw_count,checked_pairs=checked,legal_unique_checked=len(values),selected_count=len(selected),batch_size=self.contact_batch_size,total_ms=(time.perf_counter()-began)*1000)
        self.analytic_contact_report = dict(updown_step_m=.05,swivel_step_deg=10,raw_pair_count=raw_count,valid_unique_count=len(values),selected_count=len(selected),enumeration_ms=enumeration_ms,collision_and_fk_filter_ms=filter_ms,total_ms=(time.perf_counter()-began)*1000,cost_min=float(costs[order].min()),cost_max=float(costs[order].max()),priority_checked_pairs=checked,cost='original Home weighted distance, globally ordered combinations with tie completion')
        return SimpleNamespace(js_solution=JointState.from_position(torch.tensor(selected[None], device='cuda'), joint_names=ACTIVE_JOINTS),success=torch.ones((1,len(selected)),device='cuda',dtype=torch.bool))
