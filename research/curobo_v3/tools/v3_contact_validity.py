import numpy as np
import torch
from curobo.types import JointState
from v3_batched_loaded_search import GpuValidity
from v3_wall_ik_benchmark import ACTIVE_JOINTS


class TargetContactValidity(GpuValidity):
    def __init__(self, checker, targets, contact_spheres, allow_contact=False):
        super().__init__(checker, check_ground=True, joint_names=checker.kinematics.joint_names)
        self.targets = targets
        self.contact_spheres = contact_spheres
        self.allow_contact = allow_contact

    def evaluate(self, values, base_margin_m=0.0):
        valid, stability = super().evaluate(values, base_margin_m=base_margin_m)
        for offset in range(0, len(values), 4096):
            chunk = values[offset:offset + 4096]
            fk = self.checker.kinematics.compute_kinematics(JointState.from_position(chunk.unsqueeze(0), joint_names=self.joint_names))
            spheres = fk.robot_spheres.reshape(len(chunk), -1, 4)
            for side, target in self.targets.items():
                center = torch.tensor(target['center'], device=values.device, dtype=values.dtype)
                half = torch.tensor(target['dimensions'], device=values.device, dtype=values.dtype) / 2
                delta = torch.abs(spheres[..., :3] - center) - half
                distance = torch.linalg.vector_norm(torch.clamp(delta, min=0), dim=-1) + torch.clamp(delta.max(dim=-1).values, max=0)
                penetration = spheres[..., 3] - distance
                blocked = (penetration > 0) & (spheres[..., 3] > 0)
                if self.allow_contact:
                    pose = fk.tool_poses.to_dict()[side + '_tool0']
                    position = pose.position.reshape(len(chunk), 3)
                    gap = torch.linalg.vector_norm(position - torch.tensor(target['contact'],device=values.device,dtype=values.dtype),dim=-1)
                    for index in self.contact_spheres[side]:
                        blocked[:, index] &= ~((gap <= 0.005) & (penetration[:, index] <= 0.005))
                valid[offset:offset + len(chunk)] &= ~blocked.any(dim=-1)
        return valid, stability
