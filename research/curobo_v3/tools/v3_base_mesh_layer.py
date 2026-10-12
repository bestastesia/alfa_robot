import copy
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import torch
import trimesh
import yourdfpy
from curobo.scene import Scene, Mesh
from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg


class CachedMeshQuery(torch.autograd.Function):
    @staticmethod
    def forward(ctx,spheres,template,operation):
        masked=copy.copy(template)
        masked.robot_spheres=spheres
        result=operation.forward(masked)
        ctx.scale_gradient=operation.config.use_grad_input
        if ctx.needs_input_grad[0]:
            ctx.save_for_backward(operation._collision_buffer.gradient.clone())
        return result.clone()

    @staticmethod
    def backward(ctx,gradient_output):
        gradient=None
        if ctx.needs_input_grad[0]:
            gradient=ctx.saved_tensors[0]
            if ctx.scale_gradient:
                gradient=gradient*gradient_output.unsqueeze(-1)
        return gradient,None,None


def replace_arm_base_pairs(robot):
    ignore=robot['kinematics'].setdefault('self_collision_ignore',{})
    base=ignore.setdefault('base_link',[])
    for side in ['left','right']:
        for index in range(1,8):
            name=side+'_link'+str(index)
            if name not in base:base.append(name)


class BaseMeshLayer:
    def __init__(self,checker,robot):
        self.checker=checker
        kin=robot['kinematics']
        tree=ET.parse(kin['urdf_path']).getroot()
        model=yourdfpy.URDF.load(kin['urdf_path'],load_meshes=False,build_scene_graph=True)
        model.update_cfg(np.zeros(len(model.actuated_joint_names)))
        base=model.get_transform('base_link',model.base_link)
        meshes=[]
        bounds=[]
        self.mesh_manifest=[]
        for index,item in enumerate(tree.find("link[@name='base_link']").findall('collision')):
            element=item.find('geometry/mesh')
            solid=trimesh.load(element.attrib['filename'],force='mesh')
            solid.vertices*=np.fromstring(element.attrib.get('scale','1 1 1'),sep=' ')
            transform=np.eye(4);origin=item.find('origin')
            if origin is not None:
                transform[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.attrib.get('rpy','0 0 0'),sep=' ')).as_matrix()
                transform[:3,3]=np.fromstring(origin.attrib.get('xyz','0 0 0'),sep=' ')
            solid.apply_transform(base@transform)
            bounds.append(solid.bounds)
            meshes.append(Mesh(name='base_stl_'+str(index),pose=[0,0,0,1,0,0,0],vertices=solid.vertices.tolist(),faces=solid.faces.tolist()))
            self.mesh_manifest.append(dict(path=element.attrib['filename'],vertices=len(solid.vertices),faces=len(solid.faces),watertight=bool(solid.is_watertight)))
        self.scene=Scene(mesh=meshes)
        self.mesh_checker=RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(robot_config=copy.deepcopy(robot),scene_model=self.scene,n_meshes=len(meshes),n_cuboids=0,collision_activation_distance=0.005))
        config=checker.kinematics.config.kinematics_config
        indices=[]
        self.excluded_links=['base_link','arm_carriage']
        for link in kin['collision_link_names']:
            if link not in self.excluded_links:
                indices+=config.get_sphere_index_from_link_name(link).cpu().tolist()
        self.indices=torch.tensor(indices,device='cuda',dtype=torch.long)
        self.count=checker.kinematics.total_spheres
        self.selected=torch.zeros(self.count,device='cuda',dtype=torch.bool)
        self.selected[self.indices]=True
        self.bounds_lower=torch.tensor(np.min(np.asarray(bounds)[:,0],axis=0),device='cuda',dtype=torch.float32)
        self.bounds_upper=torch.tensor(np.max(np.asarray(bounds)[:,1],axis=0),device='cuda',dtype=torch.float32)
        self.broadphase_enabled=all(item['watertight'] for item in self.mesh_manifest)
        self.query_cache={}

    def prepare_query(self,batch,horizon,hard=False):
        key=(hard,batch,horizon)
        if key not in self.query_cache:
            source=self.mesh_checker.collision_constraint if hard else self.mesh_checker.collision_cost
            operation=copy.copy(source)
            operation.config=copy.copy(source.config)
            operation.update_num_spheres(self.count,batch_size=batch,horizon=horizon)
            self.query_cache[key]=operation
        return self.query_cache[key]

    def prewarm(self,shapes):
        for batch,horizon in shapes:
            self.prepare_query(batch,horizon)
            self.prepare_query(batch,horizon,hard=True)

    def cost(self,state,hard=False):
        spheres=state.robot_spheres
        selected=self.selected
        operation=self.prepare_query(spheres.shape[0],spheres.shape[1],hard)
        if self.broadphase_enabled:
            outside=torch.relu(self.bounds_lower-spheres[...,:3])+torch.relu(spheres[...,:3]-self.bounds_upper)
            activation=operation.config.activation_distance.max()
            selected=selected & (outside.square().sum(-1)<=(torch.clamp(spheres[...,3],min=0)+activation+1e-5).square())
        masked=torch.cat([spheres[...,:3],torch.where(selected,spheres[...,3],torch.full_like(spheres[...,3],-1)).unsqueeze(-1)],dim=-1)
        return CachedMeshQuery.apply(masked,state,operation)
