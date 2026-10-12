import copy
import math
import time

import numpy as np
import torch
from curobo.inverse_kinematics import InverseKinematics,InverseKinematicsCfg
from curobo.types import JointState
from v3_task_cycle_planner import TaskCyclePlanner
from curobo_core.backend import goal
from curobo_core.planner import CycleBlocked
from v3_endpoint_optimizer_compare import EndpointObjective,EnvelopeCostManager,EnvelopeCostConfig,unique_legal
from v3_base_geometry_compare import NativeBaseSpheres
from v3_batched_loaded_search import GpuValidity,loaded_robot,densify
from v3_box_rotation_cost import BoxRotationCost
from v3_wall_ik_benchmark import ACTIVE_JOINTS
from pathlib import Path
from v3_base_cuboid_layer import BaseCuboidLayer
from v3_base_mesh_layer import replace_arm_base_pairs


class MobileCuboidLayer:
    def __init__(self,mobile_checker,local_layer,names):
        self.local_layer=local_layer
        self.names=names
        self.values=None
        original=mobile_checker.get_bound
        def bound(values):
            self.values=values
            return original(values)
        mobile_checker.get_bound=bound

    def cost(self,state,hard=False):
        spheres=state.robot_spheres
        values=self.values
        center=torch.stack([values[...,self.names.index('base_x')],values[...,self.names.index('base_y')]],-1)
        yaw=values[...,self.names.index('base_yaw')]
        xy=spheres[...,:2]-center.unsqueeze(-2)
        cosine=torch.cos(yaw).unsqueeze(-1);sine=torch.sin(yaw).unsqueeze(-1)
        local_xy=torch.stack([cosine*xy[...,0]+sine*xy[...,1],-sine*xy[...,0]+cosine*xy[...,1]],-1)
        local=copy.copy(state)
        local.robot_spheres=torch.cat([local_xy,spheres[...,2:]],-1)
        return self.local_layer.cost(local,hard)


class BallFullCyclePlanner(TaskCyclePlanner):
    def __init__(self,args):
        super().__init__(args)
        replace_arm_base_pairs(self.robot)
        replace_arm_base_pairs(self.mobile_robot)
        self.endpoint_resources={}

    def collision_checker(self,boxes,payload=False,mobile=False):
        checker=super().collision_checker(boxes,payload,mobile)
        if not mobile and not hasattr(checker,'base_cuboid_layer'):
            checker.base_cuboid_layer=BaseCuboidLayer(checker,self.robot,Path(__file__).resolve().parents[1]/'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json')
            checker.base_mesh_layer=checker.base_cuboid_layer
        if mobile and not hasattr(checker,'base_cuboid_layer'):
            local=super().collision_checker(boxes,payload,False)
            layer=BaseCuboidLayer(local,self.robot,Path(__file__).resolve().parents[1]/'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json')
            checker.base_cuboid_layer=MobileCuboidLayer(checker,layer,list(self.mobile_robot['kinematics']['cspace']['joint_names']))
            checker.base_mesh_layer=checker.base_cuboid_layer
        return checker

    def plan(self,boxes,start_values):
        began=time.perf_counter()
        timing={}
        checker=self.checker(boxes,payload=True)
        checker.base_mesh_layer=checker.base_cuboid_layer
        start=torch.tensor(start_values,device='cuda',dtype=torch.float32)
        targets=copy.deepcopy(self.target_poses)
        for pose in targets.values():
            pose['position']=list(pose['position'])
            pose['position'][0]=.55
        objective=EndpointObjective(checker,targets,start)
        resource_key=(repr(self.snapshot),self.top_grasp)
        if resource_key not in self.endpoint_resources:
            self.endpoint_resources.clear()
            robot=loaded_robot(self.robot,self.box_fit,attachments=self.task_attachments(boxes))
            config=InverseKinematicsCfg.create(robot=copy.deepcopy(robot),scene_model=self.scene(boxes),collision_cache={'cuboid':max(40,len(self.snapshot.objects)),'mesh':len(self.snapshot.meshes)},num_seeds=512,self_collision_check=True,use_cuda_graph=False,position_tolerance=.002,orientation_tolerance=math.radians(1),override_iters_for_multi_link_ik=160,optimizer_collision_activation_distance=.005,cost_manager_config_instance_type=EnvelopeCostConfig)
            config.exit_early=False
            self.endpoint_resources[resource_key]=InverseKinematics(config)
        solver=self.endpoint_resources[resource_key]
        torch.cuda.synchronize()
        timing['endpoint_setup']=(time.perf_counter()-began)*1000
        segment=time.perf_counter()
        EnvelopeCostManager.objective=objective
        torch.manual_seed(621648851)
        seeds=objective.lower+(objective.upper-objective.lower)*torch.rand(1024,15,device='cuda')
        seeds[0]=start
        solved=solver.solve_pose(goal(targets),current_state=JointState.from_position(start[None],joint_names=ACTIVE_JOINTS),seed_config=seeds[None],return_seeds=512)
        torch.cuda.synchronize()
        timing['endpoint_ik']=(time.perf_counter()-segment)*1000
        segment=time.perf_counter()
        names=solved.js_solution.joint_names
        values=solved.js_solution.position.reshape(-1,len(names))[:,[names.index(name) for name in ACTIVE_JOINTS]]
        legal,diagnostics=unique_legal(values,objective)
        timing['endpoint_filter']=(time.perf_counter()-segment)*1000
        if len(legal)<64:
            raise CycleBlocked('放置收束IK',f'去重合法终点{len(legal)}/64；{diagnostics}',self.last_partial)
        endpoint=legal[0]
        validity=GpuValidity(checker,check_ground=True,weights=[math.radians(15)/.1]+[1.]*14)
        validity.collision_weights=torch.tensor([5.]+[1.]*14,device='cuda')
        if not validity.mask(start[None]).all():
            raise CycleBlocked('抽离终点','新球模型起点碰撞',self.last_partial)
        metric=BoxRotationCost(checker,ACTIVE_JOINTS,start,endpoint,self.top_grasp)
        validity.box_rotation_cost=metric
        segment=time.perf_counter()
        path,stats=self.search_path(start,endpoint[None],objective.lower+1e-5,objective.upper-1e-5,validity,2.,621648851)
        torch.cuda.synchronize()
        timing['transport_rrt']=(time.perf_counter()-segment)*1000
        if path is None:
            raise CycleBlocked('搬运路径','2秒上限内无合法首解',self.last_partial)
        transport=np.asarray(densify(path))
        if not validity.mask(torch.tensor(transport,device='cuda',dtype=torch.float32)).all():
            raise CycleBlocked('搬运细验','球碰撞验收未通过',self.last_partial)
        mobile_names=list(self.mobile_robot['kinematics']['cspace']['joint_names'])
        frames=[]
        for row in transport:
            mapping=dict(zip(ACTIVE_JOINTS,row))
            frames.append([float(mapping.get(name,0)) for name in mobile_names])
        yaw=mobile_names.index('base_yaw')
        end=frames[-1].copy()
        for step in range(1,361):
            row=end.copy();row[yaw]=math.pi*step/360
            frames.append(row)
        segment=time.perf_counter()
        mobile_checker=self.checker(boxes,payload=True,mobile=True)
        mobile_validity=GpuValidity(mobile_checker,check_ground=True,joint_names=mobile_names,weights=[2.,2.,1.,5.]+[1.]*14)
        mask=mobile_validity.mask(torch.tensor(frames,device='cuda',dtype=torch.float32))
        torch.cuda.synchronize()
        timing['loaded_turn_validation']=(time.perf_counter()-segment)*1000
        bad=(~mask).nonzero().reshape(-1)
        failure=None
        if len(bad):
            index=int(bad[0])
            failure={'stage':'Yaw180' if index>=len(transport) else '搬运','frame_index':index,'yaw_deg':math.degrees(frames[index][yaw])}
        self.last_partial['transport_rotation_metrics']=metric.metrics(torch.tensor(transport,device='cuda',dtype=torch.float32))
        self.last_partial['endpoint_legal_count']=len(legal)
        self.last_partial['base_internal_geometry']='four native cuboids, no STL'
        return {'success':failure is None,'failure':failure,'frames':frames,'transport_frames':len(transport),'joint_names':mobile_names,'yaw_frames':360,'rrt_stats':stats,'timing_ms':timing,'total_ms':(time.perf_counter()-began)*1000,'target_ik_ms':timing['endpoint_ik'],'search_ms':timing['transport_rrt'],'rotation_metrics':metric.metrics(torch.tensor(transport,device='cuda',dtype=torch.float32)),'endpoint_legal_count':len(legal),'endpoint_diagnostics':diagnostics,'stl_checks':False}
