import argparse
import copy
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.types import JointState
from curobo._src.rollout.cost_manager.cost_manager_robot import RobotCostManager
from curobo._src.rollout.cost_manager.cost_manager_robot_cfg import RobotCostManagerCfg
from curobo_core.backend import goal
from v3_task_cycle_planner import TaskCyclePlanner
from v3_base_mesh_layer import BaseMeshLayer, replace_arm_base_pairs
from v3_batched_loaded_search import GpuValidity, loaded_robot
from v3_wall_ik_benchmark import ACTIVE_JOINTS
from v3_interactive_60_tasks import DEFAULT_ROBOT_CONFIG, DEFAULT_URDF, DEFAULT_NAMED_POSES, DEFAULT_BOX_FIT, DEFAULT_MOBILE_ROBOT_CONFIG


def synchronize():
    torch.cuda.synchronize()


class EndpointObjective:
    def __init__(self, checker, targets, start):
        self.checker = checker
        self.targets = targets
        self.start = start
        self.weights = torch.tensor([math.radians(15)/.1]+[1.]*14, device='cuda')
        self.validity = GpuValidity(checker, check_ground=True)
        self.lower, self.upper = checker.kinematics.get_joint_limits().position
        self.ground_indices = [index for index in range(checker.kinematics.total_spheres) if index not in self.validity.ground_exempt_sphere_indices]
        self.ground_index_tensor=torch.tensor(self.ground_indices,device='cuda',dtype=torch.long)
        self.calls = 0

    def fk(self, values):
        return self.checker.kinematics.compute_kinematics(JointState.from_position(values.unsqueeze(0), joint_names=ACTIVE_JOINTS))

    def errors(self, state):
        parts = []
        for side in ['left', 'right']:
            pose = state.tool_poses.to_dict()[side+'_tool0']
            position = pose.position.reshape(-1,3)
            quaternion = pose.quaternion.reshape(-1,4)
            target = self.targets[side+'_tool0']
            target_quaternion = torch.tensor(target['quaternion'], device='cuda')
            quaternion = quaternion*torch.where((quaternion*target_quaternion).sum(-1,keepdim=True)>=0,1.,-1.)
            rotation = -quaternion[:,:1]*target_quaternion[1:]+target_quaternion[0]*quaternion[:,1:]-torch.linalg.cross(quaternion[:,1:],target_quaternion[1:].expand_as(quaternion[:,1:]))
            parts.extend([position-torch.tensor(target['position'],device='cuda'),2*rotation])
        return torch.cat(parts,-1)

    def extra(self, state, values):
        spheres = state.robot_spheres.reshape(len(values),-1,4)
        radial = torch.linalg.vector_norm(spheres[...,:2],dim=-1)+spheres[...,3]
        envelope = torch.relu(radial-.995).square().sum(-1)*10000
        distance = ((values-self.start)*self.weights).square().sum(-1)*.01
        mesh = self.checker.base_mesh_layer.cost(state).reshape(len(values),-1).sum(-1)*10000
        selected = spheres.index_select(1,self.ground_index_tensor)
        ground = torch.where(selected[...,3]>0,torch.relu(selected[...,3]-selected[...,2]),0.).square().sum(-1)*10000
        self.calls += 1
        return envelope+distance+mesh+ground

    def custom_loss(self, values):
        state = self.fk(values)
        error = self.errors(state)
        pose = error.square().sum(-1)*100000
        spheres = state.robot_spheres
        self.checker.setup_batch_tensors(1,len(values))
        self.checker.collision_cost.update_num_spheres(spheres.shape[-2],batch_size=1,horizon=len(values))
        world = self.checker.collision_cost.forward(state).reshape(len(values),-1).sum(-1)
        config = self.checker.kinematics.get_self_collision_config()
        pairs = config.collision_pairs.long()
        first = spheres.reshape(len(values),-1,4)[:,pairs[:,0]]
        second = spheres.reshape(len(values),-1,4)[:,pairs[:,1]]
        clearance = torch.linalg.vector_norm(first[...,:3]-second[...,:3],dim=-1)-first[...,3]-second[...,3]-config.sphere_padding[pairs[:,0]]-config.sphere_padding[pairs[:,1]]
        collision = torch.relu(.005-clearance).square().sum(-1)
        return pose+self.extra(state,values)+(world+collision)*10000

    def audit(self, values):
        with torch.no_grad():
            state = self.fk(values)
            error = self.errors(state)
            spheres = state.robot_spheres.reshape(len(values),-1,4)
            radius = (torch.linalg.vector_norm(spheres[...,:2],dim=-1)+spheres[...,3]).max(-1).values
            position = torch.maximum(torch.linalg.vector_norm(error[:,:3],dim=-1),torch.linalg.vector_norm(error[:,6:9],dim=-1))
            orientation = torch.maximum(torch.linalg.vector_norm(error[:,3:6],dim=-1),torch.linalg.vector_norm(error[:,9:12],dim=-1))
            collision = self.validity.mask(values)
            legal = collision & (radius<=1) & (position<=.002) & (orientation<=2*math.sin(math.radians(1)/2)) & ((values>=self.lower)&(values<=self.upper)).all(-1)
            return legal, {'pose_pass':int(((position<=.002)&(orientation<=2*math.sin(math.radians(1)/2))).sum()),'radius_pass':int((radius<=1).sum()),'collision_pass':int(collision.sum()),'legal_raw':int(legal.sum())}


class EnvelopeCostManager(RobotCostManager):
    objective = None

    def compute_costs(self, state, cost_collection=None, goal=None, **kwargs):
        result = super().compute_costs(state,cost_collection,goal,**kwargs)
        if self.objective is not None:
            values = state.joint_state.position.reshape(-1,15)
            extra = self.objective.extra(state.cuda_robot_model_state,values)
            reference = result.values[0]
            result.add(extra.reshape(reference.shape[0],reference.shape[1],1).expand_as(reference)/reference.shape[-1],'endpoint_envelope_and_start')
        return result


class EnvelopeCostConfig(RobotCostManagerCfg):
    @staticmethod
    def create(*args, **kwargs):
        base = RobotCostManagerCfg.create(*args,**kwargs)
        config = EnvelopeCostConfig(**vars(base))
        config.class_type = EnvelopeCostManager
        return config


def unique_legal(values, objective):
    if not len(values):
        return values, {}
    mask, diagnostics = objective.audit(values)
    accepted = values[mask].detach()
    distance = ((accepted-objective.start)*objective.weights).square().sum(-1)
    accepted = accepted[torch.argsort(distance)]
    distinct = []
    for candidate in accepted:
        if not distinct or bool((torch.linalg.vector_norm((torch.stack(distinct)-candidate)*objective.weights,dim=-1)>math.radians(1)).all()):
            distinct.append(candidate)
    return torch.stack(distinct) if distinct else accepted[:0], diagnostics


def run_trial(method, solver, objective, targets, seed, budget, count):
    torch.manual_seed(seed)
    initial = objective.start.clone()
    current = JointState.from_position(initial[None],joint_names=ACTIVE_JOINTS)
    initial_seeds = objective.lower+(objective.upper-objective.lower)*torch.rand(512,15,device='cuda')
    initial_seeds[0] = initial
    synchronize()
    started = time.perf_counter()
    phases = {}
    candidates = initial[:0].reshape(0,15)
    iterations = 0
    diagnostics = {}
    if method == 'curobo_optimizer':
        result = solver.solve_pose(goal(targets),current_state=current,seed_config=initial_seeds[None],return_seeds=512)
        synchronize()
        phases['ik_and_joint_optimization_ms'] = (time.perf_counter()-started)*1000
        names = result.js_solution.joint_names
        values = result.js_solution.position.reshape(-1,len(names))[:,[names.index(name) for name in ACTIVE_JOINTS]]
        candidates, diagnostics = unique_legal(values,objective)
    else:
        result = solver.seed_ik_solver.solve_single(goal(targets),current_state=current,seed_config=initial_seeds[None],return_seeds=512)
        synchronize()
        phases['pose_bootstrap_ms'] = (time.perf_counter()-started)*1000
        names = result.js_solution.joint_names
        values = result.js_solution.position.reshape(-1,len(names))[:,[names.index(name) for name in ACTIVE_JOINTS]].detach().clone().requires_grad_(True)
        _, bootstrap_diagnostics = objective.audit(values)
        phases['bootstrap_diagnostics'] = bootstrap_diagnostics
        candidates, _ = unique_legal(values,objective)
        optimizer = torch.optim.Adam([values],lr=.015)
        for iteration in range(200):
            if time.perf_counter()-started>=budget:
                break
            optimizer.zero_grad(set_to_none=True)
            loss = objective.custom_loss(values)
            loss.sum().backward()
            optimizer.step()
            with torch.no_grad():
                values.clamp_(objective.lower+1e-5,objective.upper-1e-5)
            iterations = iteration+1
            if iteration%10==9:
                new,diagnostics = unique_legal(values,objective)
                candidates,_ = unique_legal(torch.cat([candidates,new]),objective)
                if len(candidates)>=count:
                    break
        new, diagnostics = unique_legal(values,objective)
        candidates, _ = unique_legal(torch.cat([candidates,new]),objective)
    synchronize()
    elapsed = (time.perf_counter()-started)*1000
    distance = torch.linalg.vector_norm((candidates-objective.start)*objective.weights,dim=-1) if len(candidates) else torch.empty(0,device='cuda')
    return {'method':method,'seed':seed,'elapsed_ms':elapsed,'success':len(candidates)>=count and elapsed<=budget*1000,'unique_legal_count':len(candidates),'duplicate_threshold_weighted_rad':math.radians(1),'phases':phases,'iterations':iterations if method=='custom_adam' else None,'diagnostics':diagnostics,'distance_min':float(distance.min()) if len(distance) else None,'distance_mean':float(distance.mean()) if len(distance) else None,'candidates':candidates[:count].cpu().tolist()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source',type=Path,default=Path('generated/extract_to_place/live/1791452385002705270_side.json'))
    parser.add_argument('--output',type=Path,default=Path('generated/endpoint_optimizer_compare/pilot.json'))
    parser.add_argument('--budget',type=float,default=5.)
    parser.add_argument('--methods',nargs='+',choices=['curobo_optimizer','custom_adam'],default=['curobo_optimizer','custom_adam'])
    args = parser.parse_args()
    source = json.loads(args.source.read_text())
    setup_started = time.perf_counter()
    planner = TaskCyclePlanner(argparse.Namespace(robot_config=DEFAULT_ROBOT_CONFIG,urdf=DEFAULT_URDF,named_poses=DEFAULT_NAMED_POSES,box_fit=DEFAULT_BOX_FIT,mobile_robot_config=DEFAULT_MOBILE_ROBOT_CONFIG))
    planner.configure_task(source['task'])
    replace_arm_base_pairs(planner.robot)
    checker = planner.checker(source['task'],payload=True)
    checker.base_mesh_layer = BaseMeshLayer(checker,planner.robot)
    initial = torch.tensor(source['start'],device='cuda',dtype=torch.float32)
    objective = EndpointObjective(checker,source['target_tool_poses'],initial)
    EnvelopeCostManager.objective = objective
    robot = loaded_robot(planner.robot,planner.box_fit,attachments=planner.task_attachments(source['task']))
    config = InverseKinematicsCfg.create(robot=copy.deepcopy(robot),scene_model=planner.scene(source['task']),collision_cache={'cuboid':max(40,len(planner.snapshot.objects)),'mesh':len(planner.snapshot.meshes)},num_seeds=512,self_collision_check=True,use_cuda_graph=False,position_tolerance=.002,orientation_tolerance=math.radians(1),override_iters_for_multi_link_ik=160,optimizer_collision_activation_distance=.005,cost_manager_config_instance_type=EnvelopeCostConfig)
    config.exit_early = False
    solver = InverseKinematics(config)
    synchronize()
    setup_ms = (time.perf_counter()-setup_started)*1000
    report = {'source':str(args.source),'setup_ms':setup_ms,'budget_s':args.budget,'requested_unique_legal':64,'weights':{'pose_custom':100000,'envelope':10000,'start_distance':.01,'mesh':10000,'ground':10000},'notes':['Official cuRobo optimizer with custom cost extension; vendor source unchanged','Custom Adam uses cuRobo LM pose bootstrap included in timing','1 degree weighted joint separation for distinct outputs; does not prove globally distinct IK branches','Fixed-base demo; no path search in endpoint timings','160 configured cuRobo multi-link iterations, custom at most200 iterations, both512 seeds'],'warmup':[],'pilot':[]}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    for method in args.methods:
        result = run_trial(method,solver,objective,source['target_tool_poses'],123,args.budget,64)
        report['warmup'].append(result)
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print('warmup',method,{key:value for key,value in result.items() if key!='candidates'},flush=True)
    for method in args.methods:
        result = run_trial(method,solver,objective,source['target_tool_poses'],621648851,args.budget,64)
        report['pilot'].append(result)
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print('pilot',method,{key:value for key,value in result.items() if key!='candidates'},flush=True)
    report['twenty_trial_gate_passed'] = len(report['pilot'])==2 and all(result['success'] for result in report['pilot'])
    report['status'] = 'eligible_for_twenty_trials' if report['twenty_trial_gate_passed'] else 'stopped_at_user_five_second_gate'
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
