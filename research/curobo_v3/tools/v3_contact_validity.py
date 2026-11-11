import numpy as np
import torch
from curobo.types import JointState
from v3_batched_loaded_search import GpuValidity
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def target_in_front_of_contact_plane(position,normal,target,tolerance_m,allow_contact):
    center=torch.as_tensor(target['center'],device=position.device,dtype=position.dtype)
    contact=torch.as_tensor(target['contact'],device=position.device,dtype=position.dtype)
    rotation=torch.as_tensor(target.get('rotation',np.eye(3)),device=position.device,dtype=position.dtype)
    half=torch.as_tensor(target['dimensions'],device=position.device,dtype=position.dtype)/2
    inward=center-contact
    inward=inward/torch.linalg.vector_norm(inward)
    delta=contact-position
    axial=(delta*inward).sum(-1)
    lateral=torch.linalg.vector_norm(delta-axial[:,None]*inward,dim=-1)
    extent=(torch.abs(normal@rotation)*half).sum(-1)
    minimum_signed=((center-position)*normal).sum(-1)-extent
    minimum_gap=-tolerance_m if allow_contact else .005
    return ((normal*inward).sum(-1)>=np.cos(np.deg2rad(1.))) & (lateral<=.005) & (axial>=minimum_gap) & (axial<=target['tool_cap_reach_m']+.005) & (minimum_signed>=-tolerance_m)


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
                rotation = torch.tensor(target.get('rotation', np.eye(3)), device=values.device, dtype=values.dtype)
                delta = torch.abs((spheres[..., :3] - center) @ rotation) - half
                distance = torch.linalg.vector_norm(torch.clamp(delta, min=0), dim=-1) + torch.clamp(delta.max(dim=-1).values, max=0)
                penetration = spheres[..., 3] - distance
                blocked = (penetration > self.scene_penetration_tolerance_m) & (spheres[..., 3] > 0)
                if target.get('tool_cap_sphere_ids'):
                    pose=fk.tool_poses.to_dict()[side+'_tool0']
                    position=pose.position.reshape(len(chunk),3)
                    quaternion=pose.quaternion.reshape(len(chunk),4)
                    qw,qx,qy,qz=quaternion.unbind(-1)
                    normal=torch.stack((2*(qx*qz+qw*qy),2*(qy*qz-qw*qx),1-2*(qx*qx+qy*qy)),dim=-1)
                    permitted=target_in_front_of_contact_plane(position,normal,target,self.scene_penetration_tolerance_m,self.allow_contact)
                    indices=target['tool_cap_sphere_ids']
                    blocked[:,indices] &= ~permitted[:,None]
                if self.allow_contact:
                    pose = fk.tool_poses.to_dict()[side + '_tool0']
                    position = pose.position.reshape(len(chunk), 3)
                    gap = torch.linalg.vector_norm(position - torch.tensor(target['contact'],device=values.device,dtype=values.dtype),dim=-1)
                    for index in self.contact_spheres[side]:
                        blocked[:, index] &= ~((gap <= 0.005) & (penetration[:, index] <= getattr(self,'contact_penetration_tolerance_m',0.001)))
                valid[offset:offset + len(chunk)] &= ~blocked.any(dim=-1)
        return valid, stability
