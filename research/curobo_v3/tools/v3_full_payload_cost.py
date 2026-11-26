"""Full cuboid guidance for loaded TrajOpt; independent CPU audit remains the gate."""
import numpy as np
from scipy.spatial.transform import Rotation
import torch
from curobo._src.geom.transform import torch_quaternion_to_matrix

from v3_fixed_body_boxes import obb_penetration
from curobo_core.payload_geometry import box_overlap_depth_projected_tensor


class FullPayloadCost:
    def __init__(self, checker, original_robot, dimensions, scene, *, dtype=torch.float32):
        self.dtype = dtype
        self.device = checker.kinematics.get_joint_limits().position.device
        self.attachments = {side: torch.as_tensor(matrix, device=self.device, dtype=self.dtype)
                            for side, matrix in checker.task_tool_to_box.items()}
        self.halves = {side: torch.as_tensor(dimensions[side], device=self.device, dtype=self.dtype)/2
                       for side in self.attachments}
        kin = checker.kinematics.config.kinematics_config
        own, payload = {}, set()
        for side in self.attachments:
            indices = kin.get_sphere_index_from_link_name(side+'_link7').cpu().tolist()
            count = len(original_robot['kinematics']['collision_spheres'][side+'_link7'])
            if len(indices) <= count:raise ValueError('full payload guidance requires attached spheres')
            own[side] = set(indices[:count]); payload.update(indices[count:])
        self.robot_ids = {side: torch.as_tensor([i for i in range(checker.kinematics.total_spheres)
            if i not in payload and i not in own[side]], device=self.device, dtype=torch.long) for side in self.attachments}
        self.internal = list(checker.base_cuboid_layer.scene.cuboid)
        self.update_world(scene)
        self.source_slots = {}

    def update_world(self, scene):
        boxes = list(scene.cuboid or [])+self.internal
        values = (np.asarray([box.pose[:3] for box in boxes]),
                  Rotation.from_quat(np.roll([box.pose[3:7] for box in boxes], -1, axis=1)).as_matrix(),
                  np.asarray([box.dims for box in boxes])/2)
        for name, data in zip(('world_centers', 'world_rotations', 'world_halves'), values):
            tensor = torch.as_tensor(data, device=self.device, dtype=self.dtype)
            if hasattr(self, name):
                old = getattr(self, name)
                if old.shape != tensor.shape:raise ValueError('payload guidance scene topology changed; rebuild solver')
                old.copy_(tensor)  # Retain graph-captured addresses when only world poses change.
            else:setattr(self, name, tensor)

    def update_source_slots(self, slots):
        """Keep process-guidance buffers stable across CUDA graph replays."""
        if self.source_slots and set(slots) != set(self.source_slots):
            raise ValueError('source-slot guidance topology changed; rebuild solver')
        for side, (center, half, limit) in slots.items():
            if side not in self.attachments:
                raise ValueError('source-slot guidance has no matching attached carton')
            values = dict(center=center, rotation=np.eye(3), half=half, limit=limit)
            if side not in self.source_slots:
                self.source_slots[side] = {name:torch.as_tensor(value, device=self.device, dtype=self.dtype)
                    for name,value in values.items()}
            else:
                for name,value in values.items():
                    self.source_slots[side][name].copy_(torch.as_tensor(value, device=self.device, dtype=self.dtype))

    def _overlaps(self, state, margin=0., *, signed=False):
        spheres = state.cuda_robot_model_state.robot_spheres.to(self.dtype)
        shape = spheres.shape[:2]
        poses = state.tool_poses.to_dict()
        depths, boxes = {}, {}
        uncertain = torch.zeros(shape, device=spheres.device, dtype=torch.bool)
        for side, attachment in self.attachments.items():
            tool = poses[side+'_tool0']
            rotation = torch_quaternion_to_matrix(tool.quaternion.reshape(*shape, 4).to(self.dtype))
            center = tool.position.reshape(*shape, 3).to(self.dtype)+(rotation@attachment[:3, 3, None])[..., 0]
            rotation = rotation@attachment[:3, :3]
            half = self.halves[side]
            boxes[side] = (center, rotation, half)
            if not signed and side in self.source_slots:
                slot = self.source_slots[side]
                depths[side+'_source_slot'] = obb_penetration(center, rotation, half,
                    slot['center'][None], slot['rotation'][None], slot['half'][None], tolerance=slot['limit']-margin)
            if signed:
                depth, near_axis, face_depth = box_overlap_depth_projected_tensor(center, rotation, half,
                    self.world_centers, self.world_rotations, self.world_halves)
                depths[side+'_world'] = depth
                # A well-separated face already certifies the whole SAT decision;
                # tiny cross axes matter only without that separating witness.
                uncertain |= (near_axis & (face_depth >= .0010001-1e-7)).any(-1)
            else:
                depths[side+'_world'] = obb_penetration(center, rotation, half,
                    self.world_centers, self.world_rotations, self.world_halves, tolerance=-margin)
            selected = spheres.index_select(-2, self.robot_ids[side])
            local = ((selected[..., :3]-center[..., None, :]).unsqueeze(-2)@rotation.unsqueeze(-3))[..., 0, :]
            delta = local.abs()-half
            distance = torch.linalg.vector_norm(delta.clamp(min=0), dim=-1)+delta.amax(-1).clamp(max=0)
            depth = selected[..., 3]+margin-distance
            depths[side+'_robot'] = torch.where(selected[..., 3] > 0,
                depth if signed else depth.clamp(min=0), -float('inf') if signed else 0.)
            extent_z = (rotation[..., 2, :].abs()*half).sum(-1)
            depth = extent_z+margin-center[..., 2]
            depths[side+'_ground'] = (depth if signed else depth.clamp(min=0)).unsqueeze(-1)
        if len(boxes) == 2:
            first, second = boxes.values()
            if signed:
                depth, near_axis, face_depth = box_overlap_depth_projected_tensor(*first, second[0].unsqueeze(-2),
                    second[1].unsqueeze(-3), second[2].unsqueeze(-2))
                depths['box_box'] = depth
                uncertain |= (near_axis & (face_depth >= 1e-6-1e-7)).any(-1)
            else:
                depths['box_box'] = obb_penetration(*first, second[0].unsqueeze(-2),
                    second[1].unsqueeze(-3), second[2].unsqueeze(-2), tolerance=-margin)
        return depths, uncertain

    def overlaps(self, state, margin=0.):
        return self._overlaps(state, margin)[0]

    def classify(self, state):
        return self.classify_with_robot_depth(state)[:2]

    def classify_with_robot_depth(self, state):
        if self.dtype != torch.float64:raise ValueError('payload search classification requires float64')
        spheres = state.cuda_robot_model_state.robot_spheres
        if (spheres.is_cuda and spheres.shape[0] == 1
                and 0 < spheres.shape[1] <= 32):
            return self._classify_small_batch(state)
        return self._classify_eager(state)

    def _classify_small_batch(self, state):
        from types import SimpleNamespace
        from curobo._src.util.cuda_graph_util import GraphExecutor
        spheres = state.cuda_robot_model_state.robot_spheres
        count = spheres.shape[1]
        tools = state.tool_poses.to_dict()
        inputs = [spheres]
        for side in self.attachments:
            pose = tools[side+'_tool0']
            inputs.extend((pose.position.reshape(1, count, 3), pose.quaternion.reshape(1, count, 4)))
        if count < 32:
            inputs = [torch.cat((value, value[:, -1:].expand(1, 32-count, *value.shape[2:])), dim=1)
                      for value in inputs]
        if not hasattr(self, '_search_graph'):
            def capture(spheres, *poses):
                tools = {side+'_tool0':SimpleNamespace(position=poses[2*i], quaternion=poses[2*i+1])
                         for i, side in enumerate(self.attachments)}
                return self._classify_eager(SimpleNamespace(cuda_robot_model_state=SimpleNamespace(robot_spheres=spheres),
                    tool_poses=SimpleNamespace(to_dict=lambda:tools)))
            # ponytail: one32-row LM graph; larger search queries stay eager to bound capture cost and memory.
            self._search_graph = GraphExecutor(capture, spheres.device, use_cuda_graph=True,
                                               clone_inputs=True, clone_outputs=True)
        return tuple(value[:, :count] for value in self._search_graph(*inputs))

    def _classify_eager(self, state):
        if self.dtype != torch.float64:raise ValueError('payload search classification requires float64')
        depths, uncertain = self._overlaps(state, signed=True)
        valid = torch.ones_like(uncertain)
        robot_depth = torch.full_like(uncertain, -float('inf'), dtype=self.dtype)
        for name, depth in depths.items():
            if name.endswith('_robot'):
                robot_depth = torch.maximum(robot_depth, depth.amax(-1))
            threshold = .0010001 if name.endswith('_world') else (.001 if name.endswith('_ground') else 1e-6)
            valid &= ~(depth > threshold).any(-1)
            uncertain |= ((depth-threshold).abs() <= 1e-7).any(-1)
        return valid, uncertain, robot_depth

    def cost(self, state):
        # Actual overlap keeps the strong penalty. A valid sub-15mm gap must not
        # be penalized like penetration and push an arm into real self collision.
        # Process slots need an early clearance bias, rather than an unconstrained
        # optimizer undoing a valid extraction. This is guidance, not extra clearance in the final gate.
        return sum((value.square() if name.endswith('_source_slot') else
                    (value-.015).clamp(min=0).square()+.001*value.square()).sum(-1)
                   for name,value in self.overlaps(state, margin=.015).items())
