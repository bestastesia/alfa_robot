"""Experiment-local full OBB guidance with both physically verified terminal caps."""
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from curobo._src.geom.transform import torch_quaternion_to_matrix
from v3_fixed_body_boxes import obb_penetration

class TransferCost:
    def __init__(self,checker,geometry,side,rows,cap_ids,owner):
        self.side=side
        def tensor(x):return torch.as_tensor(x,device='cuda',dtype=torch.float32)
        self.cap_ids={s:torch.tensor(ids,device='cuda',dtype=torch.long) for s,ids in cap_ids.items()}
        if 'attachments' in geometry:
            attachments=geometry['attachments'];nominal=geometry['nominal_attachments']
        else:  # Existing standalone experiment callers.
            attachments={'left':geometry['left_attachment'],'right':geometry['right_measured_attachment']}
            nominal={'left':geometry['left_attachment'],'right':geometry['right_nominal_attachment']}
        self.attachment=tensor(attachments[side])
        self.inverse_nominal={s:tensor(np.linalg.inv(nominal[s])) for s in cap_ids}
        self.half=tensor(geometry['box_dimensions_m']/2)
        obs=list(checker.guidance_scene.cuboid)+list(checker.base_cuboid_layer.scene.cuboid)
        self.centers=tensor(np.asarray([o.pose[:3] for o in obs]))
        self.rotations=tensor(Rotation.from_quat(np.roll([o.pose[3:7] for o in obs],-1,axis=1)).as_matrix())
        self.halves=tensor(np.asarray([o.dims for o in obs])/2)
        self.slot_center=tensor(owner.center_m);self.slot_rotation=tensor(np.eye(3));self.slot_half=tensor(np.asarray(owner.dimensions_m)/2)
        self.fixed=tensor(np.flatnonzero(np.max(np.abs(rows-rows[0]),axis=0)==0)).long();self.reference=tensor(rows[0])
        from curobo_core.suction_grasp import contact_cap_spheres
        self.reach={s:float(np.max(contact_cap_spheres(s)[:,2]+contact_cap_spheres(s)[:,3])) for s in cap_ids}

    def cost(self,state):
        spheres=state.cuda_robot_model_state.robot_spheres
        shape=spheres.shape[:2]
        poses={}
        for side,pose in state.tool_poses.to_dict().items():
            poses[side.removesuffix('_tool0')]=(pose.position.reshape(*shape,3),torch_quaternion_to_matrix(pose.quaternion.reshape(*shape,4)))
        p,R=poses[self.side];C=p+(R@self.attachment[:3,3,None])[...,0];B=R@self.attachment[:3,:3]
        local=((spheres[...,:3]-C[...,None,:]).unsqueeze(-2)@B.unsqueeze(-3))[...,0,:]
        delta=local.abs()-self.half
        depth=spheres[...,3]-(torch.linalg.vector_norm(delta.clamp(min=0),dim=-1)+delta.amax(-1).clamp(max=0))
        penalties=torch.relu(depth-1e-6).square()*(spheres[...,3]>0)
        for side,ids in self.cap_ids.items():
            p,R=poses[side];inverse=self.inverse_nominal[side];inward=(B@inverse[:3,:3])[...,2]
            contact=C+(B@inverse[:3,3,None])[...,0];normal=R[...,2]
            difference=contact-p;axial=(difference*inward).sum(-1);lateral=torch.linalg.vector_norm(difference-axial[...,None]*inward,dim=-1)
            extent=((normal.unsqueeze(-2)@B)[...,0,:].abs()*self.half).sum(-1)
            signed=((C-p)*normal).sum(-1)-extent
            axis=(B.transpose(-1,-2)@inward[...,None])[...,0].abs().argmax(-1)
            excess=local.index_select(-2,ids).abs()+spheres.index_select(-2,ids)[...,3,None]-self.half
            tangent=torch.arange(3,device='cuda')!=axis[...,None]
            footprint=torch.where(tangent[...,None,:],excess,-torch.inf).amax((-1,-2))
            permitted=((normal*inward).sum(-1)>=np.cos(np.deg2rad(1)))&(lateral<=.005)&(axial>=-.001)&(axial<=self.reach[side]+.005)&(signed>=-.001)&(footprint<=.001)
            penalties[...,ids]=torch.where(permitted[...,None],0.,penalties.index_select(-1,ids))
        world=obb_penetration(C,B,self.half,self.centers,self.rotations,self.halves,tolerance=.001).square().sum(-1)
        source=obb_penetration(C,B,self.half,self.slot_center[None],self.slot_rotation[None],self.slot_half[None],tolerance=1e-6).square().sum(-1)
        bottom=C[...,2]-(B[...,2,:].abs()*self.half).sum(-1)
        fixed=(state.joint_state.position.index_select(-1,self.fixed)-self.reference.index_select(-1,self.fixed)).square().sum(-1)
        return penalties.sum(-1)+world+source+torch.relu(-bottom-.001).square()+fixed


def install():
    from v3_fast_geometry_cost import GeometricCostManager
    original=GeometricCostManager.compute_costs
    def compute(self,state,cost_collection=None,goal=None,**kwargs):
        result=original(self,state,cost_collection,goal,**kwargs)
        if not hasattr(self,'_transfer_layer'):self._transfer_layer=GeometricCostManager.transfer_layer
        reference=result.values[0]
        extra=1e7*self._transfer_layer.cost(state)
        result.add(extra.unsqueeze(-1).expand_as(reference)/reference.shape[-1],'transfer_single_obb_and_cap')
        return result
    GeometricCostManager.compute_costs=compute
