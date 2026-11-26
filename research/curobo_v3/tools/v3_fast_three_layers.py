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
from v3_lm_refinement import optimize_lm_stage

POLICY=read(ROOT/'config/fast_three_layers.json')
RRT_PHASES=tuple(POLICY['trajectory_optimization_phases'])
PHASES=tuple(POLICY['time_optimization_phases'])
OUTPUT=ROOT/'generated/fast_three_layers_20261107'


def stamp():
    torch.cuda.synchronize()
    return time.perf_counter()


def native_sample_torque(solver, dense):
    """Audit every native sample without resizing the captured preparation workspace."""
    capacity = solver._dynamics._tau_buffer.shape[0]
    if dense.position.shape[0] != 1 or capacity < 1:
        raise ValueError('native sampled torque expects one path and an allocated workspace')
    chunks = []
    for offset in range(0, dense.position.shape[1], capacity):
        state = JointState(**{name:getattr(dense, name)[:, offset:offset+capacity].contiguous()
                              for name in ('position', 'velocity', 'acceleration')})
        # RNEA returns a borrowed workspace; clone before the next chunk overwrites it.
        chunks.append(solver._compute_torque(state).clone())
    return torch.cat(chunks, dim=1)


class FastThreeLayerPlanner:
    def __init__(self, planner=None):
        OUTPUT.mkdir(parents=True,exist_ok=True)
        if 'enable_timeopt' not in inspect.signature(TrajectoryOptimizerCfg.create).parameters:
            raise RuntimeError('Run with isolated cuRoboV2-Fast on PYTHONPATH')
        assets, dynamics_path = prepare_fast_assets()
        self.planner=planner if planner is not None else InflatedPlanningPlanner(POLICY['planning_world_radius_scale'], assets)
        self.dynamics=TorqueModel(dynamics_path,payload_mass_kg=POLICY['payload_kg_per_box'],
            payload_dimensions=(self.planner.box_fit.get('dimensions_m') or
                {side: fit['dimensions_m'] for side, fit in self.planner.box_fit.items()}))
        self.robot=KinematicsCfg.from_robot_yaml_file(str(dynamics_path))
        if list(self.robot.kinematics_config.joint_names)!=ACTIVE_JOINTS:raise ValueError('joint order mismatch')
        self.time_solvers={}
        self.geometry_solvers={}

    def optimize_path(self,source,phase,active,validity,scene,*,seed_only=False):
        return self.trajopt_path(source,phase,active,validity,scene,seed_only=seed_only)

    def trajopt_path(self,source,phase,active,validity,scene,geometry_scale=None,*,seed_only=False):
        if phase not in RRT_PHASES:raise ValueError('TrajOpt forbidden for non-RRT stages')
        geometry_scale=float(POLICY.get('trajopt_geometry_scale',1.) if geometry_scale is None else geometry_scale)
        if not np.isfinite(geometry_scale) or geometry_scale<=0:raise ValueError('geometry scale must be positive and finite')
        if len(scene.cuboid)>48:raise ValueError('Graph TrajOpt scene exceeds fixed48-cuboid capacity')
        began=stamp()
        explicit_payload = source.get('active_payload_sides')
        attached = self.planner.task_attachments(source['task']) if explicit_payload is not None or phase == 'transport' else ()
        if explicit_payload is not None:
            attached = tuple(a for a in attached if a.parent_link.removesuffix('_tool0') in explicit_payload)
            if {a.parent_link.removesuffix('_tool0') for a in attached} != set(explicit_payload):
                raise ValueError('native geometry payload sides do not match physical attachments')
        robot=loaded_robot(self.planner.robot,self.planner.box_fit,attachments=attached) if attached else copy.deepcopy(self.planner.robot)
        managers=(GeometricCostManager,ProjectCostManager)
        ground_indices=[index for index in range(validity.checker.kinematics.total_spheres) if index not in validity.ground_exempt_sphere_indices]
        guide_payload = (bool(attached) if explicit_payload is not None else phase == 'transport') and getattr(self.planner, 'full_payload_trajopt', False)
        if seed_only and not guide_payload:raise ValueError('native seed retry is restricted to audited layout transport')
        key=(digest(robot),tuple(ground_indices),geometry_scale,guide_payload,len(scene.cuboid))
        resource=self.geometry_solvers.get(phase)
        created=resource is None or resource['key']!=key
        if created:
            if resource is not None:resource['solver'].destroy()
            base_layer=BaseCuboidLayer(validity.checker,self.planner.robot,ROOT/'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json')
            base_layer.fixed_world_cost = torch.zeros((), device='cuda')
        else:base_layer=resource['base_layer']
        payload_layer = None
        if guide_payload:
            if created:
                from v3_full_payload_cost import FullPayloadCost
                payload_layer = FullPayloadCost(validity.checker, self.planner.robot,
                    {side: (self.planner.box_fit[side] if side in self.planner.box_fit else self.planner.box_fit)['dimensions_m']
                     for side in validity.checker.task_tool_to_box}, scene)
            else:
                payload_layer = resource['payload_layer']
                payload_layer.update_world(scene)
        if payload_layer is not None and hasattr(self.planner, 'transport_source_slots'):
            payload_layer.update_source_slots(self.planner.transport_source_slots(validity.checker, active[0]))
        GeometricCostManager.payload_layer = payload_layer
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
            self.geometry_solvers[phase]=dict(key=key,solver=solver,base_layer=base_layer,payload_layer=payload_layer)
        else:
            solver=resource['solver']
            solver.scene_collision_checker.load_collision_model(scene)
        solver.reset_seed()
        setup_ms=(stamp()-began)*1000
        bounds = [v.cpu().numpy() for v in validity.checker.kinematics.get_joint_limits().position] if guide_payload else None
        began=stamp();controls,start,goal,fit,_=fit_path(dict(positions=active),32,control_bounds=bounds)
        tensors=[torch.tensor(value[None],device='cuda',dtype=torch.float32).contiguous() for value in (controls,start,goal)]
        initial=JointState.from_position(tensors[1],joint_names=ACTIVE_JOINTS)
        target=JointState.from_position(tensors[2],joint_names=ACTIVE_JOINTS)
        fit_ms=(stamp()-began)*1000;began=stamp()
        seed_snapshot=tensors[0].detach().clone()
        initial_snapshot=initial.clone();target_snapshot=target.clone()
        if seed_only:
            result=solver.evaluate_cspace_seed(goal_state=target_snapshot,current_state=initial_snapshot,
                seed_traj=seed_snapshot[:,None],return_seeds=1,trajectory_source='native_seed_after_full_gate_rejection')
        else:
            result=solver.solve_cspace(goal_state=target,current_state=initial,seed_traj=tensors[0][:,None],num_seeds=1,return_seeds=1,finetune_attempts=1)
        solve_ms=(stamp()-began)*1000
        allow_seed_retention=getattr(self.planner,'full_payload_trajopt',False) and phase in RRT_PHASES
        optimizer_success=None if seed_only else bool(result.success.any())
        seed_retention=dict(success=bool(result.success.any()),after_full_gate_rejection=True) if seed_only else None
        if allow_seed_retention and not seed_only and not optimizer_success:
            # Result processors reuse mutable buffers. Keep the rejected result
            # and original seed intact before asking the SAME native validator.
            optimized=result.clone()
            validation={name:bool(value.all()) for name,value in result.debug_info.get('validation',{}).items()}
            fallback_began=stamp()
            candidate=solver.evaluate_cspace_seed(goal_state=target_snapshot,current_state=initial_snapshot,
                seed_traj=seed_snapshot[:,None],return_seeds=1,trajectory_source='native_seed_after_rejected_trajopt')
            seed_retention=dict(success=bool(candidate.success.any()),optimizer_validation=validation,
                validation={name:bool(value.all()) for name,value in candidate.debug_info.get('validation',{}).items()},
                wall_ms=(stamp()-fallback_began)*1000)
            result=candidate if seed_retention['success'] else optimized
        native=result.js_solution
        knot=native.knot.detach().reshape(1,32,15).clone();knot_dt=native.knot_dt.detach().reshape(1).clone()
        if not torch.isfinite(knot).all():raise ValueError('TrajOpt returned nonfinite geometry')
        report=dict(method='Fast geometric TrajOpt',geometry_scale=geometry_scale,setup_ms=setup_ms,fit_ms=fit_ms,solve_ms=solve_ms,seed_fit=fit,solver_success=bool(result.success.any()),timeopt_inside=False,cuda_graph=True,new_solver=created,timing_input='static path; geometric dt is not a physical time seed; remove only constant initial span')
        report['optimizer_success']=optimizer_success
        report['seed_retention']=seed_retention
        report['selected_trajectory_source']=getattr(result,'trajectory_source',None) or 'trajopt'
        if seed_retention is not None and seed_retention['success']:
            report['method']='Original input selected after genuine native seed validation; not a successful optimized candidate'
        report['full_payload_guidance'] = guide_payload
        report['source_slot_guidance'] = bool(payload_layer is not None and payload_layer.source_slots)
        report['native_validation'] = {name: bool(value.all()) for name,value in result.debug_info.get('validation',{}).items()}
        metrics=result.interpolated_metrics if result.interpolated_metrics is not None else result.metrics
        if metrics is None:
            # Preserve original success while inspecting the same optimized geometry at native dt.
            diagnostic=JointState.from_position(native.position[..., [native.joint_names.index(name) for name in ACTIVE_JOINTS]].reshape(1,-1,15).clone(),joint_names=ACTIVE_JOINTS)
            diagnostic.knot=knot.clone();diagnostic.knot_dt=knot_dt.clone()
            diagnostic.control_space=native.control_space
            metrics=solver._interpolate_and_compute_metrics(diagnostic)
        report['native_constraint_details']={}
        for category in ('constraints','hybrid_costs_constraints'):
            collection=getattr(metrics.costs_and_constraints,category)
            for name,value in zip(collection.names,collection.values):
                detail=dict(maximum=float(value.max()),positive_values=int((value>0).sum()),first=float(value[:,0].max()),last=float(value[:,-1].max()))
                if detail['positive_values']:
                    per_time=value.reshape(value.shape[0],value.shape[1],-1).amax(-1)
                    index=int(per_time[0].argmax())
                    detail.update(peak_timestep=index,configuration=metrics.state.joint_state.position[0,index].detach().cpu().tolist(),
                        joint_names=metrics.state.joint_state.joint_names or solver.joint_names)
                report['native_constraint_details'][category+'/'+name]=detail
        native_config=solver.kinematics.config.kinematics_config
        reference_config=validity.checker.kinematics.config.kinematics_config
        report['sphere_layout_differences']={}
        for link in self.planner.robot['kinematics']['collision_link_names']:
            actual=native_config.get_sphere_index_from_link_name(link).cpu().tolist()
            reference=reference_config.get_sphere_index_from_link_name(link).cpu().tolist()
            if actual!=reference:report['sphere_layout_differences'][link]=dict(native=actual,checker=reference)
        for manager in managers:manager.checker=None
        GeometricCostManager.payload_layer = None
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
        if not success:
            # Diagnose the failed fixed path; this does not prove the requested task impossible.
            positions=solver._evaluation.path[0,0].detach().cpu().numpy().copy()
            report['failure_diagnostics']=dict(status_meaning={1:'no initialized feasible timing seed',2:'sampled feasibility rejected',3:'invalid input'}.get(report['status'],'unknown'))
            if report['status'] in (1,2) and np.isfinite(positions).all():
                gravity,gravity_ms=self.dynamics.torque(positions,np.zeros_like(positions),np.zeros_like(positions),sides,attachments)
                ratios=np.abs(gravity[:,1:])/np.asarray(POLICY['arm_continuous_torque_nm']*2)
                sample,joint=np.unravel_index(np.argmax(ratios),ratios.shape)
                report['failure_diagnostics'].update(peak_static_torque_ratio=float(ratios[sample,joint]),
                    peak_static_ratio_to_scaled_limit=float(ratios[sample,joint]/POLICY['planning_limit_scale']),
                    peak_joint=ACTIVE_JOINTS[joint+1],sample_index=int(sample),configuration=positions[sample].tolist(),
                    gravity_compute_ms=gravity_ms,status_meaning={1:'no initialized feasible timing seed',2:'sampled feasibility rejected',3:'invalid input'}.get(report['status'],'unknown'))
            return None,report
        duration=float(result.time[0,-1]);began=stamp()
        if benchmark_recompute:
            solver.timeopt(controls,start,goal,knot_dt=knot_dt)
            report['warm_recompute_ms']=(stamp()-began)*1000
        else:report['warm_recompute_ms']=0.
        began=stamp()
        dense=solver.sample(torch.linspace(0,duration,4001,device='cuda')[None].contiguous())
        torque=native_sample_torque(solver,dense)
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
        record=dict(index=index,task=source['task'],planning_ms=planning_ms,planning_success=bool(source['success']),stages=[],curobo_path=str(curobo.__file__),pipeline_revision=POLICY['revision'],policy_snapshot=copy.deepcopy(POLICY),policy_digest=digest(POLICY),robot_config_digest=digest(self.planner.robot))
        if not source['success']:
            record.update(success=False,complete_candidate=False,blocked_stage=source.get('blocked_stage'),blocked_details=source.get('blocked_details'))
            save(output/f'{index:03d}_report.json',record)
            return record
        for phase in PHASES:
            refinement=POLICY.get('lm_refinement',{})
            refine_lm=refinement.get('enabled',False) and phase in refinement.get('phases',[]) and len(source['task'])==2
            progress(f'{phase}：'+('Fast TrajOpt → Fast TimeOpt' if phase in RRT_PHASES else ('LM输出 → 64控制点32轮关节路径调整 → Fast TimeOpt' if refine_lm else '保持LM/贴近路径 → 表示适配 → Fast TimeOpt（无TrajOpt）')))
            stage_started=stamp();full,columns,validity,scene=stage_setup(self.planner,source,phase);active=np.ascontiguousarray(full[:,columns],dtype=np.float32)
            stage=dict(phase=phase,trajopt_applied=phase in RRT_PHASES,rounding_applied=False,lm_source_unchanged=phase in ('extract','conveyor_place'),lm_refinement_applied=refine_lm,input_sha256=__import__('hashlib').sha256(active.tobytes()).hexdigest())
            try:
                if phase in RRT_PHASES:
                    controls,start,goal,knot_dt,details=self.optimize_path(source,phase,active,validity,scene);stage['trajectory_optimization']=details
                elif refine_lm:
                    controls,start,goal,knot_dt,details=optimize_lm_stage(self.planner,source,phase,active,validity,refinement)
                    stage['lm_joint_refinement']=details
                else:
                    fit_began=stamp();values,first,last,fit,_=fit_path(dict(positions=active),128)
                    controls,start,goal=[torch.tensor(value[None],device='cuda',dtype=torch.float32).contiguous() for value in (values,first,last)]
                    knot_dt=None;stage['representation_adaptation']=dict(fit_ms=(stamp()-fit_began)*1000,**fit)
                    if phase in ('extract','conveyor_place'):
                        stage['lm_refinement_skipped']='disabled by config' if not refinement.get('enabled',False) else 'single-arm adaptation not validated'
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
        record['lm_refinement_ms']=sum(sum(s.get('lm_joint_refinement',{}).get(key,0) for key in ('prepare_ms','solve_ms','diagnostics_ms')) for s in record['stages'])
        record['time_warm_recompute_ms']=sum(s.get('time_optimization',{}).get('warm_recompute_ms',0) for s in record['stages'])
        record['complete_candidate']=record['motion_s'] is not None
        record['all_trajopt_success']=all(s.get('trajectory_optimization',{}).get('solver_success',True) for s in record['stages'])
        record['dynamic_valid']=all(s.get('time_optimization',{}).get('dynamic_valid',False) for s in record['stages'])
        record['success']=record['complete_candidate'] and record['all_trajopt_success'] and record['dynamic_valid']
        record['total_wall_ms']=(stamp()-began)*1000
        save(output/f'{index:03d}_report.json',record)
        return record


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--indices',type=int,nargs='+',default=[0,38]);parser.add_argument('--output',type=Path,default=OUTPUT);args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True);began=stamp();planner=FastThreeLayerPlanner();initialization_ms=(stamp()-began)*1000;records=[]
    for index in args.indices:
        np.random.seed(20261107+index);torch.manual_seed(20261107+index)
        records.append(planner.full_cycle(index,output_directory=args.output));save(args.output/'summary.json',dict(initialization_ms=initialization_ms,records=records,policy='Inflated-world multi-candidate planning + LM extraction/placement; Fast TrajOpt only RRT phases;64-control32-iteration LM refinement; Fast TimeOpt all motion phases; no posthoc collision recheck; not hardware execution'))


if __name__=='__main__':main()
