import numpy as np
import torch
from scipy.spatial.transform import Rotation
from curobo.types import JointState
from v3_task_scene import grasp_transform


def quaternion_multiply(first,second):
    scalar=first[...,:1]*second[...,:1]-(first[...,1:]*second[...,1:]).sum(-1,keepdim=True)
    vector=first[...,:1]*second[...,1:]+second[...,:1]*first[...,1:]+torch.linalg.cross(first[...,1:],second[...,1:])
    return torch.cat([scalar,vector],-1)


def rotation_vector(quaternion):
    quaternion=quaternion/torch.clamp(torch.linalg.vector_norm(quaternion,dim=-1,keepdim=True),min=1e-8)
    quaternion=torch.where(quaternion[...,:1]<0,-quaternion,quaternion)
    length=torch.linalg.vector_norm(quaternion[...,1:],dim=-1,keepdim=True)
    angle=2*torch.atan2(length,torch.clamp(quaternion[...,:1],min=0))
    return quaternion[...,1:]*torch.where(length>1e-6,angle/torch.clamp(length,min=1e-8),torch.full_like(length,2.))


class BoxRotationCost:
    def __init__(self,checker,joint_names,start,goal,top):
        self.checker=checker;self.joint_names=joint_names
        self.attachments={side:torch.tensor(np.roll(Rotation.from_matrix(grasp_transform(side,top)[:3,:3]).as_quat(),1),device='cuda',dtype=torch.float32) for side in ['left','right']}
        initial=self.orientations(start[None]);final=self.orientations(goal[None])
        self.initial={side:initial[side].clone() for side in initial}
        self.axes={};self.angles={}
        for side in initial:
            relative=quaternion_multiply(self.conjugate(initial[side]),final[side]);vector=rotation_vector(relative)[0]
            angle=torch.linalg.vector_norm(vector)
            self.axes[side]=vector/torch.clamp(angle,min=1e-6);self.angles[side]=angle
        self.weights=dict(deviation=0.4,backtrack=1.0,rotation=0.2)

    @staticmethod
    def conjugate(quaternion):return torch.cat([quaternion[...,:1],-quaternion[...,1:]],-1)

    def orientations(self,values):
        state=self.checker.kinematics.compute_kinematics(JointState.from_position(values.unsqueeze(0),joint_names=self.joint_names))
        poses=state.tool_poses.to_dict()
        return {side:quaternion_multiply(poses[side+'_tool0'].quaternion.reshape(-1,4).clone(),self.attachments[side].expand(len(values),-1)) for side in self.attachments}

    def terms(self,quaternions):
        vector=rotation_vector(quaternion_multiply(self.conjugate(self.initial[self.current_side]).expand_as(quaternions),quaternions))
        angle=self.angles[self.current_side];axis=self.axes[self.current_side]
        progress=(vector*axis).sum(-1) if float(angle)>1e-4 else torch.zeros(len(vector),device=vector.device)
        ideal=torch.clamp(progress,min=0,max=float(angle)).unsqueeze(-1)*axis
        deviation=torch.linalg.vector_norm(vector-ideal,dim=-1)
        return progress,deviation

    def edge_cost(self,starts,ends):
        fractions=torch.linspace(0,1,9,device=starts.device)
        values=starts[:,None]+fractions[None,:,None]*(ends-starts)[:,None]
        orientations=self.orientations(values.reshape(-1,starts.shape[-1]))
        total=torch.zeros(len(starts),device=starts.device)
        for side,quats in orientations.items():
            self.current_side=side
            progress,deviation=self.terms(quats);progress=progress.reshape(len(starts),9);deviation=deviation.reshape(len(starts),9)
            quats=quats.reshape(len(starts),9,4)
            motion=torch.linalg.vector_norm(rotation_vector(quaternion_multiply(self.conjugate(quats[:,:-1]),quats[:,1:])),dim=-1)
            excess=torch.relu(motion.sum(-1)-torch.clamp(progress[:,-1]-progress[:,0],min=0))
            total+=self.weights['deviation']*torch.trapezoid(deviation,dx=1/8,dim=-1)*motion.sum(-1)
            total+=self.weights['backtrack']*torch.relu(progress[:,:-1]-progress[:,1:]).sum(-1)
            total+=self.weights['rotation']*excess
        return total

    def metrics(self,values):
        orientations=self.orientations(values)
        report={}
        for side,quats in orientations.items():
            self.current_side=side;progress,deviation=self.terms(quats)
            motion=torch.linalg.vector_norm(rotation_vector(quaternion_multiply(self.conjugate(quats[:-1]),quats[1:])),dim=-1)
            report[side]=dict(required_rotation_deg=float(self.angles[side]*180/np.pi),total_rotation_deg=float(motion.sum()*180/np.pi),
                backtracking_deg=float(torch.relu(progress[:-1]-progress[1:]).sum()*180/np.pi),max_deviation_deg=float(deviation.max()*180/np.pi))
        return report
