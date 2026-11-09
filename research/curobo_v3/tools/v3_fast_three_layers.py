import argparse
import copy
import inspect
import json
import time
from pathlib import Path
from types import SimpleNamespace

import curobo
import numpy as np
import torch
from curobo.kinematics import KinematicsCfg
from curobo.trajectory_optimizer import TrajectoryOptimizer,TrajectoryOptimizerCfg
from curobo.trajectory_timeopt import TrajectoryTimeOpt,TrajectoryTimeOptCfg
from curobo.types import JointState

from curobo_core.torque import TorqueModel
from curobo_core.scene import digest
from v3_base_cuboid_layer import BaseCuboidLayer
from v3_fast_path import ROOT, read, save, stage_setup, ACTIVE_JOINTS, fit_path
from v3_fixed_body_boxes import InflatedPlanningPlanner, FixedBodyWorld
from v3_batched_loaded_search import loaded_robot,GpuValidity
from v3_fast_geometry_cost import GeometricCostManager, GeometricCostConfig, ProjectCostManager
from v3_fast_assets import prepare_fast_assets

POLICY=read(ROOT/'config/fast_three_layers.json')
RRT_PHASES=tuple(POLICY['trajectory_optimization_phases'])
PHASES=tuple(POLICY['time_optimization_phases'])
OUTPUT=ROOT/'generated/fast_three_layers_20261107'


def stamp():
    torch.cuda.synchronize()
    return time.perf_counter()


class FastThreeLayerPlanner:
    def __init__(self):
        OUTPUT.mkdir(parents=True,exist_ok=True)
        if 'enable_timeopt' not in inspect.signature(TrajectoryOptimizerCfg.create).parameters:
            raise RuntimeError('Run with isolated cuRoboV2-Fast on PYTHONPATH')
        assets, dynamics_path = prepare_fast_assets()
        self.planner=InflatedPlanningPlanner(POLICY['planning_world_radius_scale'], assets)
        self.dynamics=TorqueModel(dynamics_path,payload_mass_kg=POLICY['payload_kg_per_box'])
        self.robot=KinematicsCfg.from_robot_yaml_file(str(dynamics_path))
        if list(self.robot.kinematics_config.joint_names)!=ACTIVE_JOINTS:raise ValueError('joint order mismatch')
        self.time_solvers={}
        self.geometry_solvers={}

    def optimize_path(self,source,phase,active,validity,scene):
        return self.trajopt_path(source,phase,active,validity,scene)

    def trajopt_path(self,source,phase,active,validity,scene,geometry_scale=None):
        if phase not in RRT_PHASES:raise ValueError('TrajOpt forbidden for non-RRT stages')
        geometry_scale=float(POLICY.get('trajopt_geometry_scale',1.) if geometry_scale is None else geometry_scale)
        if not np.isfinite(geometry_scale) or geometry_scale<=0:raise ValueError('geometry scale must be positive and finite')
        if len(scene.cuboid)>48:raise ValueError('Graph TrajOpt scene exceeds fixed48-cuboid capacity')
        began=stamp()
        robot=loaded_robot(self.planner.robot,self.planner.box_fit,attachments=self.planner.task_attachments(source['task'])) if phase=='transport' else copy.deepcopy(self.planner.robot)
        managers=(GeometricCostManager,ProjectCostManager)
        ground_indices=[index for index in range(validity.checker.kinematics.total_spheres) if index not in validity.ground_exempt_sphere_indices]
        key=(digest(robot),tuple(ground_indices),geometry_scale)
        resource=self.geometry_solvers.get(phase)
        created=resource is None or resource['key']!=key
        if created:
            if resource is not None:resource['solver'].destroy()
            base_layer=BaseCuboidLayer(validity.checker,self.planner.robot,ROOT/'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json')
            base_layer.fixed_world_cost = torch.zeros((), device='cuda')
        else:base_layer=resource['base_layer']
        fixed_world = FixedBodyWorld(base_layer.scene, scene, ACTIVE_JOINTS)
        base_layer.fixed_world_cost.copy_(fixed_world.cost(torch.zeros((1, 15), device='cuda'), tolerance=.001).sum())
        for manager in managers:
            manager.checker=SimpleNamespace(base_cuboid_layer=base_layer)
            manager.ground_indices=ground_indices
        if created:
            cfg=read(ROOT/'config/fast_trajopt.json')
            config=TrajectoryOptimizerCfg.create(robot=robot,scene_model=scene,collision_cache={'cuboid':48,'mesh':0},
                transition_model=cfg['transition'],optimizer_configs=[cfg['optimizer']],metrics_rollout=cfg['metrics'],num_seeds=1,
                self_collision_check=True,use_cuda_graph=True,optimizer_collision_activation_distance=.003,
                cost_manager_config_instance_type=GeometricCostConfig,minimum_trajectory_dt=1.,maximum_trajectory_dt=1.,
                interpolation_buffer_size=8000,interpolation_dt=.025,enable_timeopt=False,override_optimizer_num_iters={'lbfgs':50})
            solver=TrajectoryOptimizer(config)
            for rollout in solver.core.get_all_rollout_instances(include_optimizer_rollouts=True):
                for manager in rollout._cost_manager_list:
                    if isinstance(manager,GeometricCostManager):
                        manager.geometry_scale=geometry_scale
                        if hasattr(manager,'_geometry_scale_tensor'):manager._geometry_scale_tensor.fill_(geometry_scale)
            self.geometry_solvers[phase]=dict(key=key,solver=solver,base_layer=base_layer)
        else:
            solver=resource['solver']
            solver.scene_collision_checker.load_collision_model(scene)
        solver.reset_seed()
        setup_ms=(stamp()-began)*1000
        began=stamp();controls,start,goal,fit,_=fit_path(dict(positions=active),32)
        tensors=[torch.tensor(value[None],device='cuda',dtype=torch.float32).contiguous() for value in (controls,start,goal)]
        initial=JointState.from_position(tensors[1],joint_names=ACTIVE_JOINTS)
        target=JointState.from_position(tensors[2],joint_names=ACTIVE_JOINTS)
        fit_ms=(stamp()-began)*1000;began=stamp()
        result=solver.solve_cspace(goal_state=target,current_state=initial,seed_traj=tensors[0][:,None],num_seeds=1,return_seeds=1,finetune_attempts=1)
        solve_ms=(stamp()-began)*1000
        native=result.js_solution
        knot=native.knot.detach().reshape(1,32,15).clone();knot_dt=native.knot_dt.detach().reshape(1).clone()
        if not torch.isfinite(knot).all():raise ValueError('TrajOpt returned nonfinite geometry')
        report=dict(method='Fast geometric TrajOpt',geometry_scale=geometry_scale,setup_ms=setup_ms,fit_ms=fit_ms,solve_ms=solve_ms,seed_fit=fit,solver_success=bool(result.success.any()),timeopt_inside=False,cuda_graph=True,new_solver=created,timing_input='static path; geometric dt is not a physical time seed; remove only constant initial span')
        if result.metrics is not None:
            constraints = result.metrics.costs_and_constraints.constraints
            report['native_constraints'] = {name: float(value.max()) for name, value in zip(constraints.names, constraints.values)}
        else:
            report['native_constraints_unavailable'] = True
        for manager in managers:manager.checker=None
        return knot,tensors[1],tensors[2],None,report

    def time_path(self,controls,start,goal,knot_dt,sides,attachments,benchmark_recompute=True):
        count=controls.shape[1]
        key=(count,tuple(sides),json.dumps({side:np.asarray(attachments[side]).tolist() for side in sides},sort_keys=True))
        began=stamp();created=key not in self.time_solvers
        if created:
            robot=copy.deepcopy(self.robot)
            robot.kinematics_config=self.dynamics._create(sides,attachments).kinematics_config
            effort=torch.tensor([1e6]+POLICY['arm_continuous_torque_nm']*2,device='cuda',dtype=torch.float32)
            config=TrajectoryTimeOptCfg(robot,n_knots=count,limit_scale=POLICY['planning_limit_scale'],jerk_limit_scale=POLICY['planning_limit_scale'],
                velocity_limits=torch.tensor([1e4]+[np.deg2rad(POLICY['arm_velocity_deg_s'])]*14,device='cuda',dtype=torch.float32),acceleration_limits=torch.full((15,),1e6,device='cuda'),
                jerk_limits=torch.tensor([1e6]+[POLICY['arm_jerk_rad_s3']]*14,device='cuda',dtype=torch.float32),torque_lower=-effort,torque_upper=effort,use_cuda_graph=True)
            self.time_solvers[key]=TrajectoryTimeOpt(config)
        solver=self.time_solvers[key];setup_ms=(stamp()-began)*1000
        began=stamp();result=solver.timeopt(controls,start,goal,knot_dt=knot_dt);first_ms=(stamp()-began)*1000
        success=bool(result.success[0]);report=dict(method='Fast TrajectoryTimeOpt',setup_ms=setup_ms,solve_ms=first_ms,new_solver=created,success=success,status=int(result.status[0]),cuda_graph=True)
        if not success:return None,report
        duration=float(result.time[0,-1]);began=stamp()
        if benchmark_recompute:
            solver.timeopt(controls,start,goal,knot_dt=knot_dt)
            report['warm_recompute_ms']=(stamp()-began)*1000
        else:report['warm_recompute_ms']=0.
        began=stamp()
        dense=solver.sample(torch.linspace(0,duration,4001,device='cuda')[None].contiguous())
        torque=solver._compute_torque(dense)
        report.update(duration_s=duration,peak_velocity_deg_s=float(torch.rad2deg(dense.velocity[...,1:].abs()).max()),peak_jerk_rad_s3=float(dense.jerk[...,1:].abs().max()),peak_torque_ratio=float((torque[...,1:].abs()/torch.tensor([294,294,178,178,87,52,10]*2,device='cuda')).max()),endpoint_velocity=float(dense.velocity[:,[0,-1]].abs().max()),endpoint_acceleration=float(dense.acceleration[:,[0,-1]].abs().max()))
        times=torch.linspace(0,duration,4001,device='cuda')[None].contiguous()
        state=dense
        arrays={name:getattr(state,name)[0].detach().cpu().numpy().copy() for name in ('position','velocity','acceleration','jerk')}
        arrays.update(times=times[0].cpu().numpy(),torque=torque[0,:,1:].detach().cpu().numpy().copy())
        report['sample_and_diagnostics_ms']=(stamp()-began)*1000
        report['dynamic_valid']=bool(report['peak_velocity_deg_s']<=60.01 and report['peak_jerk_rad_s3']<=500.1 and report['peak_torque_ratio']<=1.001)
        return arrays,report

    def full_cycle(self,index,progress=lambda message:None,output_directory=None,benchmark_recompute=False):
        output=Path(output_directory) if output_directory is not None else OUTPUT
        output.mkdir(parents=True,exist_ok=True)
        progress('轨迹规划：新生成吸附候选、LM抽离、多起终点运输连接和LM放置')
        began=stamp();source=self.planner.full_cycle(self.planner.task_options[index][1],progress);planning_ms=(stamp()-began)*1000
        save(output/f'{index:03d}_source.json.gz',source)
        record=dict(index=index,task=source['task'],planning_ms=planning_ms,planning_success=bool(source['success']),stages=[],curobo_path=str(curobo.__file__))
        if not source['success']:
            record.update(success=False,complete_candidate=False,blocked_stage=source.get('blocked_stage'),blocked_details=source.get('blocked_details'))
            save(output/f'{index:03d}_report.json',record)
            return record
        for phase in PHASES:
            progress(f'{phase}：'+('Fast TrajOpt → Fast TimeOpt' if phase in RRT_PHASES else '保持LM/贴近路径 → 表示适配 → Fast TimeOpt（无TrajOpt）'))
            stage_started=stamp();full,columns,validity,scene=stage_setup(self.planner,source,phase);active=np.ascontiguousarray(full[:,columns],dtype=np.float32)
            stage=dict(phase=phase,trajopt_applied=phase in RRT_PHASES,rounding_applied=False,lm_source_unchanged=phase in ('extract','conveyor_place'),input_sha256=__import__('hashlib').sha256(active.tobytes()).hexdigest())
            try:
                if phase in RRT_PHASES:
                    controls,start,goal,knot_dt,details=self.optimize_path(source,phase,active,validity,scene);stage['trajectory_optimization']=details
                else:
                    fit_began=stamp();values,first,last,fit,_=fit_path(dict(positions=active),128)
                    controls,start,goal=[torch.tensor(value[None],device='cuda',dtype=torch.float32).contiguous() for value in (values,first,last)]
                    knot_dt=None;stage['representation_adaptation']=dict(fit_ms=(stamp()-fit_began)*1000,**fit)
                sides=tuple(source['task']) if phase in ('extract','transport','conveyor_place') else ()
                arrays,timing=self.time_path(controls,start,goal,knot_dt,sides,source['visual_tool_to_box'],benchmark_recompute=benchmark_recompute);stage['time_optimization']=timing
                if arrays is not None:
                    np.savez_compressed(output/f'{index:03d}_{phase}.npz',**arrays)
                    stage['endpoint_error']=float(abs(arrays['position'][[0,-1]]-active[[0,-1]]).max())
                    if phase=='contact_approach':validity=self.planner.target_validity(self.planner.checker(source['task']),allow_contact=True)
                    validity.scene_penetration_tolerance_m=.001;validity.contact_penetration_tolerance_m=.001
                    stage.update(posthoc_checked_frames=0,posthoc_ms=0.,posthoc_policy='disabled per requested planning policy')
                stage['total_ms']=(stamp()-stage_started)*1000
            except Exception as error:
                import traceback
                traceback.print_exc();stage['error']=repr(error)
            record['stages'].append(stage);save(output/f'{index:03d}_report.json',record)
            print('THREE_LAYER',index,phase,json.dumps(stage),flush=True)
        record['motion_s']=sum(s['time_optimization']['duration_s'] for s in record['stages']) if all(s.get('time_optimization',{}).get('success') for s in record['stages']) else None
        record['geometry_optimization_ms']=sum(s.get('trajectory_optimization',{}).get('solve_ms',0) for s in record['stages'])
        record['time_optimization_ms']=sum(s.get('time_optimization',{}).get('solve_ms',0) for s in record['stages'])
        record['time_warm_recompute_ms']=sum(s.get('time_optimization',{}).get('warm_recompute_ms',0) for s in record['stages'])
        record['complete_candidate']=record['motion_s'] is not None
        record['all_trajopt_success']=all(s.get('trajectory_optimization',{}).get('solver_success',True) for s in record['stages'])
        record['dynamic_valid']=all(s.get('time_optimization',{}).get('dynamic_valid',False) for s in record['stages'])
        record['success']=record['complete_candidate'] and record['all_trajopt_success'] and record['dynamic_valid']
        record['total_wall_ms']=(stamp()-began)*1000
        save(output/f'{index:03d}_report.json',record)
        return record


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--indices',type=int,nargs='+',default=[9,44,90]);args=parser.parse_args()
    OUTPUT.mkdir(parents=True,exist_ok=True);began=stamp();planner=FastThreeLayerPlanner();initialization_ms=(stamp()-began)*1000;records=[]
    for index in args.indices:
        np.random.seed(20261107+index);torch.manual_seed(20261107+index)
        records.append(planner.full_cycle(index));save(OUTPUT/'summary.json',dict(initialization_ms=initialization_ms,records=records,policy='Inflated-world multi-candidate planning + LM extraction/placement; Fast TrajOpt only RRT phases; Fast TimeOpt all motion phases; no posthoc collision recheck; not hardware execution'))


if __name__=='__main__':main()
