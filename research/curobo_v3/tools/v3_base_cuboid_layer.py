import copy
from pathlib import Path
import json
import numpy as np
import torch
import yourdfpy
from scipy.spatial.transform import Rotation
from curobo.scene import Scene,Cuboid
from curobo.collision_checking import RobotCollisionChecker,RobotCollisionCheckerCfg
from v3_base_mesh_layer import BaseMeshLayer


class BaseCuboidLayer(BaseMeshLayer):
    def __init__(self,checker,robot,definition):
        self.checker=checker
        data=json.loads(Path(definition).read_text())
        assert data['frame']=='base_link' and data['cuboids']
        kin=robot['kinematics']
        urdf=yourdfpy.URDF.load(kin['urdf_path'],load_meshes=False,build_scene_graph=True)
        urdf.update_cfg(np.zeros(len(urdf.actuated_joint_names)))
        base=urdf.get_transform('base_link',urdf.base_link)
        boxes=[]
        for item in data['cuboids']:
            local=np.eye(4)
            local[:3,:3]=Rotation.from_euler('xyz',item['rotation_deg'],degrees=True).as_matrix()
            local[:3,3]=item['center']
            transform=base@local
            quaternion=Rotation.from_matrix(transform[:3,:3]).as_quat()
            boxes.append(Cuboid(name=item['name'],dims=item['dimensions'],pose=[*transform[:3,3],quaternion[3],*quaternion[:3]]))
        self.scene=Scene(cuboid=boxes)
        self.mesh_checker=RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(robot_config=copy.deepcopy(robot),scene_model=self.scene,n_meshes=0,n_cuboids=len(boxes),collision_activation_distance=.005))
        self.indices=torch.cat([checker.kinematics.config.kinematics_config.get_sphere_index_from_link_name(link) for link in kin['collision_link_names'] if link not in ['base_link','arm_carriage']]).long()
        self.count=checker.kinematics.total_spheres
        self.selected=torch.zeros(self.count,device='cuda',dtype=torch.bool)
        self.selected[self.indices]=True
        self.broadphase_enabled=False
        self.query_cache={}
