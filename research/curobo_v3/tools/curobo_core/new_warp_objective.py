import copy
import numpy as np
import torch
from scipy.interpolate import BSpline
from scipy.spatial.transform import Rotation
from curobo._src.cost.cost_scene_collision import SceneCollisionCost
from curobo._src.cost.cost_self_collision import SelfCollisionCost
from curobo._src.robot.kinematics.kinematics import KinematicsState
from curobo.types import JointState
from v3_batched_loaded_search import GpuValidity
from v3_wall_ik_benchmark import ACTIVE_JOINTS


class SelfCollisionGradient(torch.autograd.Function):
    @staticmethod
    def forward(ctx,spheres,operation):
        output=operation.forward(spheres)
        ctx.save_for_backward(operation._out_grad.clone())
        return output.clone()

    @staticmethod
    def backward(ctx,gradient):
        return ctx.saved_tensors[0]*gradient.unsqueeze(-1),None


def spline_basis(controls=32,samples=129):
    knots=np.r_[np.zeros(4),np.linspace(0,1,controls-2)[1:-1],np.ones(4)]
    return BSpline(knots,np.eye(controls),3)(np.linspace(0,1,samples))


class CorridorObjective:
    def __init__(self,checker,document,controls=32,samples=129,batch=3):
        self.checker=checker
        self.controls=controls
        self.samples=samples
        self.batch=batch
        self.basis=torch.tensor(spline_basis(controls,samples),device='cuda',dtype=torch.float32)
        self.start=torch.tensor(document['start'],device='cuda',dtype=torch.float32)
        self.goal=torch.tensor(document['goal'],device='cuda',dtype=torch.float32)
        self.weights=torch.tensor([np.deg2rad(15)/.1]+[1.]*14,device='cuda',dtype=torch.float32)
        self.lower,self.upper=checker.kinematics.get_joint_limits().position
        source=document['methods']['lm']['endpoint_soft']['node_diagnostics']['targets']
        self.targets={}
        for side,matrix in source.items():
            matrix=np.asarray(matrix)
            progress=np.linspace(0,1,128)
            fraction=np.linspace(0,1,samples)
            positions=np.column_stack([np.interp(fraction,progress,matrix[:,axis,3]) for axis in range(3)])
            from scipy.spatial.transform import Slerp
            rotations=Slerp(progress,Rotation.from_matrix(matrix[:,:3,:3]))(fraction)
            self.targets[side]=(torch.tensor(positions,device='cuda',dtype=torch.float32),
                torch.tensor(np.roll(rotations.as_quat(),1,axis=1),device='cuda',dtype=torch.float32))
        config=copy.copy(checker.collision_cost.config)
        config.use_grad_input=True
        config.activation_distance=torch.full_like(config.activation_distance,.01)
        self.world=SceneCollisionCost(config)
        self.world.update_num_spheres(checker.kinematics.total_spheres,batch_size=batch,horizon=samples)
        config=copy.copy(checker.self_collision_cost.config)
        config.use_grad_input=False
        self.self_collision=SelfCollisionCost(config)
        self.self_collision.setup_batch_tensors(batch,samples)
        layer=checker.base_cuboid_layer
        config=copy.copy(layer.mesh_checker.collision_cost.config)
        config.use_grad_input=True
        config.activation_distance=torch.full_like(config.activation_distance,.01)
        self.base=SceneCollisionCost(config)
        self.base.update_num_spheres(checker.kinematics.total_spheres,batch_size=batch,horizon=samples)
        self.base_selected=layer.selected
        self.ground_selected=torch.ones(checker.kinematics.total_spheres,device='cuda',dtype=torch.bool)
        self.ground_selected[list(GpuValidity(checker,check_ground=True).ground_exempt_sphere_indices)]=False

    def decode(self,interior):
        controls=torch.cat([self.start[None,None].expand(len(interior),1,-1),interior,self.goal[None,None].expand(len(interior),1,-1)],1)
        return torch.einsum('tn,bnd->btd',self.basis,controls).contiguous()

    def terms(self,interior):
        values=self.decode(interior)
        state=self.checker.kinematics.compute_kinematics(JointState.from_position(values,joint_names=ACTIVE_JOINTS))
        spheres=state.robot_spheres
        world=self.world.forward(state).reshape(self.batch,self.samples,-1).sum(-1)
        own=SelfCollisionGradient.apply(spheres,self.self_collision).reshape(self.batch,self.samples,-1).sum(-1)
        masked=torch.cat([spheres[...,:3],torch.where(self.base_selected,spheres[...,3],torch.full_like(spheres[...,3],-1)).unsqueeze(-1)],-1)
        base=self.base.forward(KinematicsState(robot_spheres=masked)).reshape(self.batch,self.samples,-1).sum(-1)
        ground=torch.relu(spheres[...,3]-spheres[...,2]+.005)
        ground=torch.where(self.ground_selected&(spheres[...,3]>0),ground,0).square().sum(-1)
        collision=3000*(world.mean(-1)+own.mean(-1)+base.mean(-1))+500*(world.max(-1).values+own.max(-1).values+base.max(-1).values)+3000*ground.mean(-1)
        differences=(values[:,1:]-values[:,:-1])*self.weights
        lengths=torch.sqrt(differences.square().sum(-1)+1e-8)
        length=.5*lengths.sum(-1)
        bending=2*((differences[:,1:]-differences[:,:-1])*(self.samples-1)).square().sum(-1).mean(-1)
        maximum=10*torch.relu(differences.abs().amax(-1)-np.deg2rad(5)).square().sum(-1)
        reference=torch.zeros(self.batch,device='cuda')
        rotation_length=torch.zeros(self.batch,device='cuda')
        for side,(position,quaternion) in self.targets.items():
            pose=state.tool_poses.to_dict()[side+'_tool0']
            error=torch.linalg.vector_norm(pose.position.reshape(self.batch,self.samples,3)-position,dim=-1)
            dot=(pose.quaternion.reshape(self.batch,self.samples,4)*quaternion).sum(-1).abs().clamp(0,1)
            orientation=1-dot.square()
            reference+=100*torch.relu(error-.03).square().mean(-1)+100*torch.relu(error-.15).square().max(-1).values
            reference+=4*torch.relu(orientation-np.sin(np.deg2rad(5)/2)**2).mean(-1)
            quaternions=pose.quaternion.reshape(self.batch,self.samples,4)
            aligned=torch.where((quaternions[:,1:]*quaternions[:,:-1]).sum(-1,keepdim=True)<0,-quaternions[:,1:],quaternions[:,1:])
            rotation_length+=.3*torch.sqrt(4*(aligned-quaternions[:,:-1]).square().sum(-1)+1e-9).sum(-1)
        safe=torch.minimum(-self.lower[[6,13]],self.upper[[6,13]])-np.deg2rad(20)
        wrist=.3*torch.relu(values[:,:,[6,13]].abs()-safe).square().sum(-1).mean(-1)
        return dict(collision=collision,length=length,bending=bending,maximum=maximum,reference=reference,wrist=wrist,rotation_length=rotation_length)


def path_metrics(checker,rows,samples):
    tracks={side:dict(positions=[],quaternions_wxyz=[]) for side in ('left','right')}
    rows=np.asarray(rows)
    for offset in range(0,len(rows),512):
        state=checker.kinematics.compute_kinematics(JointState.from_position(torch.tensor(rows[offset:offset+512][None],device='cuda',dtype=torch.float32),joint_names=ACTIVE_JOINTS))
        for side in tracks:
            pose=state.tool_poses.to_dict()[side+'_tool0']
            tracks[side]['positions'].extend(pose.position.reshape(-1,3).detach().cpu().tolist())
            tracks[side]['quaternions_wxyz'].extend(pose.quaternion.reshape(-1,4).detach().cpu().tolist())
    for side,track in tracks.items():
        rotations=Rotation.from_quat(np.roll(track['quaternions_wxyz'],-1,axis=1))
        track['total_rotation_deg']=float(np.rad2deg((rotations[:-1].inv()*rotations[1:]).magnitude()).sum())
        position=np.asarray(track['positions'])
        target=samples[side][:,:3,3]
        track['mean_nearest_reference_m']=float(np.linalg.norm(position[:,None]-target[None],axis=-1).min(-1).mean())
    weights=np.array([np.deg2rad(15)/.1]+[1.]*14)
    return dict(tracks=tracks,weighted_joint_length=float(np.linalg.norm(np.diff(rows,axis=0)*weights,axis=1).sum()),
        frames=len(rows),joint6_min_margin_deg={side:float(110-np.abs(np.rad2deg(rows[:,index])).max()) for side,index in [('left',6),('right',13)]})


class FixedLiftObjective(CorridorObjective):
    def __init__(self, checker, document, lift):
        super().__init__(checker, document, controls=32, samples=128, batch=1)
        self.lift = torch.tensor(lift, device='cuda', dtype=torch.float32)

    def decode(self, interior):
        rows = super().decode(interior)
        return torch.cat((self.lift[None, :, None].expand(len(rows), -1, -1), rows[..., 1:]), dim=-1).contiguous()
