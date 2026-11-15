import argparse
import copy
from dataclasses import replace
import gc
import hashlib
import heapq
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import torch

from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.types import GoalToolPose, JointState, Pose
from curobo_core.contracts import PlannerAssets
from curobo_core.adapter import to_curobo_scene
from curobo_core.scene import Pose as ScenePose, SceneObject
from v3_adaptive_raised_cycle_planner import AdaptiveRaisedCyclePlanner
from v3_batched_loaded_search import GpuValidity
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def clock():
    torch.cuda.synchronize()
    return time.perf_counter()


def sample_route(document, count):
    positions = {side: np.asarray(track['positions']) for side, track in document['tracks'].items()}
    increments = np.maximum.reduce([np.linalg.norm(np.diff(values, axis=0), axis=1) for values in positions.values()])
    for track in document['tracks'].values():
        orientations = Rotation.from_quat(np.roll(track['quaternions_wxyz'], -1, axis=1))
        increments = np.maximum(increments, .1*(orientations[:-1].inv()*orientations[1:]).magnitude())
    progress = np.r_[0., np.cumsum(increments)]
    retained = np.r_[True, np.diff(progress) > 1e-10]
    progress = progress[retained]
    target = np.linspace(0., progress[-1], count)
    samples = {}
    for side, values in positions.items():
        matrices = np.repeat(np.eye(4)[None], count, axis=0)
        matrices[:, :3, 3] = np.column_stack([np.interp(target, progress, values[retained, axis]) for axis in range(3)])
        matrices[:, :3, :3] = Slerp(progress, Rotation.from_quat(np.roll(document['tracks'][side]['quaternions_wxyz'], -1, axis=1))[retained])(target).as_matrix()
        samples[side] = matrices
    return samples, target / target[-1]


def summarize_filter(planner, checker, samples, rows, point_ids, initial_success=None):
    began = clock()
    rows = np.asarray(rows, dtype=np.float32).reshape(-1, 15)
    point_ids = np.asarray(point_ids, dtype=np.int32)
    pose_masks, collision_masks = [], []
    fk_ms, collision_ms = 0., 0.
    validity = GpuValidity(checker, check_ground=True)
    for offset in range(0, len(rows), 512):
        tensor = torch.as_tensor(rows[offset:offset+512], device='cuda')
        started = clock()
        state = checker.kinematics.compute_kinematics(JointState.from_position(tensor[None], joint_names=ACTIVE_JOINTS))
        valid = torch.ones(len(tensor), device='cuda', dtype=torch.bool)
        limits = checker.kinematics.get_joint_limits().position
        valid &= ((tensor >= limits[0]-1e-6) & (tensor <= limits[1]+1e-6)).all(-1)
        for side, matrices in samples.items():
            targets = matrices[point_ids[offset:offset+512]]
            target_position = torch.tensor(targets[:, :3, 3], device='cuda', dtype=torch.float32)
            target_quaternion = torch.tensor(np.roll(Rotation.from_matrix(targets[:, :3, :3]).as_quat(), 1, axis=1), device='cuda', dtype=torch.float32)
            pose = state.tool_poses.to_dict()[side+'_tool0']
            valid &= torch.linalg.vector_norm(pose.position.reshape(-1,3)-target_position, dim=-1) <= .002
            valid &= (pose.quaternion.reshape(-1,4)*target_quaternion).sum(-1).abs() >= math.cos(math.radians(1)/2)
        pose_masks.extend(valid.cpu().tolist())
        fk_ms += (clock()-started)*1000
        started = clock()
        collision_masks.extend(validity.mask(tensor).cpu().tolist())
        collision_ms += (clock()-started)*1000
    pose_masks = np.asarray(pose_masks, dtype=bool)
    collision_masks = np.asarray(collision_masks, dtype=bool)
    accepted = pose_masks & collision_masks
    counts, unique_counts, diverse_counts = [], [], []
    for index in range(len(next(iter(samples.values())))):
        values = rows[(point_ids==index) & accepted]
        unique = np.unique(np.round(values, 4), axis=0)
        diverse = []
        for value in unique:
            if not diverse or np.linalg.norm((np.asarray(diverse)-value)*np.array([math.radians(15)/.1]+[1.]*14), axis=1).min() >= .15:
                diverse.append(value)
        counts.append(len(values)); unique_counts.append(len(unique)); diverse_counts.append(len(diverse))
    report = dict(total_candidates=len(rows), pose_and_limits_passed=int(pose_masks.sum()), collision_passed=int(collision_masks.sum()), legal=int(accepted.sum()), legal_unique=int(sum(unique_counts)), diverse_015=int(sum(diverse_counts)), covered_points=int(np.count_nonzero(counts)), point_count=len(counts), legal_per_point=counts, unique_per_point=unique_counts, diverse_per_point=diverse_counts, empty_point_indices=np.flatnonzero(np.asarray(counts)==0).tolist(), fk_pose_filter_ms=fk_ms, collision_filter_ms=collision_ms, filter_and_summary_ms=(clock()-began)*1000)
    if initial_success is not None:
        report['solver_success']=int(np.asarray(initial_success).sum())
        report['solver_success_and_legal']=int((accepted & initial_success).sum())
    return report, dict(joints=rows, point_indices=point_ids, pose_valid=pose_masks, collision_valid=collision_masks, legal=accepted)


def enumerate_analytic(planner, checker, samples, cap, references=None, wrist_weight=0.):
    lower, upper = [value.cpu().numpy() for value in checker.kinematics.get_joint_limits().position]
    transforms = []
    for height in np.round(np.arange(-1., .0001, .05),2):
        if not lower[0] <= height <= upper[0]:
            continue
        row=planner.home_values.copy(); row[0]=height; planner.fk(row)
        transforms.append((height,np.linalg.inv(planner.fk_robot.get_transform('arm_carriage',planner.fk_robot.base_link))))
    calls, raw_pairs, rows, identifiers = 0, 0, [], []
    arm_counts, compatible_heights = [], []
    for point in range(len(next(iter(samples.values())))):
        reference=planner.home_values if references is None else references[point]
        grids=[]
        totals=[0,0]
        for height,inverse in transforms:
            arms=[]
            for arm,side in enumerate(('left','right')):
                offset=1+7*arm; seed=reference[offset:offset+7]
                solutions=[]
                target=inverse@samples[side][point]
                for swivel in range(-180,180,10):
                    branches=planner.analytic.solve(arm,target,math.radians(swivel),seed);calls+=1
                    for branch in branches:
                        if np.all(branch>=lower[offset:offset+7]) and np.all(branch<=upper[offset:offset+7]):
                            solutions.append(branch)
                values=np.unique(np.round(np.asarray(solutions).reshape(-1,7),8),axis=0)
                cost=np.sum((values-seed)**2,axis=1)
                if wrist_weight and len(values):
                    safe=min(abs(lower[offset+5]),abs(upper[offset+5]))-math.radians(20)
                    cost+=wrist_weight*np.maximum(np.abs(values[:,5])-safe,0)**2
                order=np.argsort(cost,kind='stable');arms.append((values[order],cost[order]))
                totals[arm]+=len(values)
            raw_pairs+=len(arms[0][0])*len(arms[1][0])
            if len(arms[0][0]) and len(arms[1][0]):grids.append((height,arms))
        arm_counts.append(totals)
        compatible_heights.append([float(height) for height,arms in grids])
        heap=[]
        for index,(height,arms) in enumerate(grids):
            lift=((height-reference[0])*math.radians(15)/.1)**2
            for left in range(len(arms[0][0])):
                heapq.heappush(heap,(float(lift+arms[0][1][left]+arms[1][1][0]),index,left,0))
        selected=0
        while heap and selected<cap:
            cost,grid,left,right=heapq.heappop(heap);height,arms=grids[grid]
            rows.append([height,*arms[0][0][left],*arms[1][0][right]]);identifiers.append(point);selected+=1
            if right+1<len(arms[1][0]):
                next_cost=cost-arms[1][1][right]+arms[1][1][right+1]
                heapq.heappush(heap,(float(next_cost),grid,left,right+1))
    return rows,identifiers,dict(single_arm_analytic_calls=calls,raw_limit_valid_dual_combinations=raw_pairs,per_point_candidate_cap=cap,updown_step_m=.05,swivel_step_deg=10,selection='Home weighted distance before collision; no downstream planning',single_arm_limit_valid_per_point=arm_counts,compatible_updown_per_point=compatible_heights)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--points',type=int,default=100)
    parser.add_argument('--method',choices=['analytic','numeric'],required=True)
    parser.add_argument('--seeds',type=int,default=32)
    parser.add_argument('--cap',type=int,default=256)
    parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--solver-mode',choices=['full','lm'],default='full')
    parser.add_argument('--ik-batch-points',type=int,default=0)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    batch_points=args.ik_batch_points or args.points
    if batch_points<1 or args.points%batch_points:
        raise ValueError('Point count must be divisible by IK batch size for an unpadded comparison')
    document=json.loads(args.input.read_text());samples,progress=sample_route(document,args.points)
    samples_started=time.perf_counter()
    planner=AdaptiveRaisedCyclePlanner(PlannerAssets());planner.configure_task(document['snapshot']['task'])
    objects=tuple(SceneObject(item['name'],tuple(item['dimensions']),ScenePose(tuple(item['position']),tuple(item['quaternion_wxyz']))) for item in document['snapshot']['objects'] if item['name']!='navigation_ground')
    planner.snapshot=replace(planner.snapshot,objects=objects,meshes=())
    for side in ('left','right'):
        planner.fk(planner.home_values)
        initial=planner.fk_robot.get_transform(side+'_tool0',planner.fk_robot.base_link)
        if np.max(np.abs(initial-np.asarray(document['snapshot']['tcp_world'][side])))>.002:
            raise RuntimeError('Frozen route initial Pose differs from old Home; must supply matching base/state instead of silently assuming world==base')
    checker=planner.checker(planner.selected_task)
    full_scene=to_curobo_scene(planner.snapshot,())
    checker.update_world(full_scene)
    validity=GpuValidity(checker,check_ground=True)
    for count in (1,512):validity.mask(torch.tensor(np.repeat(planner.home_values[None],count,axis=0),device='cuda'))
    setup_ms=(clock()-samples_started)*1000
    report=dict(method=args.method,points=args.points,seeds=args.seeds if args.method=='numeric' else None,cap=args.cap if args.method=='analytic' else None,setup_ms=setup_ms,input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),sampling_progress=progress.tolist(),task=document['snapshot']['task'],motion_mode=document.get('motion_mode'),trials=[],collision_contract='same native robot/self/world/ground and four internal base cuboids; original target boxes present; no attachments; no endpoint cylinder restriction')
    report['scene_object_names']=[item.name for item in full_scene.cuboid]
    maximum_translation=max(np.max(np.linalg.norm(np.diff(values[:,:3,3],axis=0),axis=1)) for values in samples.values())
    maximum_rotation=max(np.max((Rotation.from_matrix(values[:-1,:3,:3]).inv()*Rotation.from_matrix(values[1:,:3,:3])).magnitude()) for values in samples.values())
    report['max_sample_translation_m']=float(maximum_translation);report['max_sample_rotation_deg']=float(np.rad2deg(maximum_rotation))
    solver=None
    if args.method=='numeric':
        began=clock()
        config=InverseKinematicsCfg.create(robot=copy.deepcopy(planner.robot),scene_model=None,self_collision_check=False,num_seeds=args.seeds,seed_solver_num_seeds=args.seeds,max_batch_size=batch_points,position_tolerance=.002,orientation_tolerance=math.radians(1),use_cuda_graph=True,override_iters_for_multi_link_ik=160,random_seed=20261021)
        config.exit_early=args.solver_mode=='lm'
        config.exit_early_batch_success_threshold=0.
        solver=InverseKinematics(config)
        goals=[]
        for offset in range(0,args.points,batch_points):
            goals.append(GoalToolPose.from_poses({side+'_tool0':Pose(position=torch.tensor(values[offset:offset+batch_points,:3,3],device='cuda',dtype=torch.float32),quaternion=torch.tensor(np.roll(Rotation.from_matrix(values[offset:offset+batch_points,:3,:3]).as_quat(),1,axis=1),device='cuda',dtype=torch.float32)) for side,values in samples.items()},ordered_tool_frames=['left_tool0','right_tool0']))
        state=JointState.from_position(torch.tensor(np.repeat(planner.home_values[None],batch_points,axis=0),device='cuda'),joint_names=ACTIVE_JOINTS)
        active_goal=copy.deepcopy(goals[0])
        report['solver_setup_ms']=(clock()-began)*1000
        began=clock()
        for poses in goals:
            active_goal.position.copy_(poses.position)
            active_goal.quaternion.copy_(poses.quaternion)
            with torch.enable_grad():solver.solve_pose(active_goal,current_state=state,return_seeds=args.seeds)
        report['first_full_batch_warmup_ms']=(clock()-began)*1000
        print('NUMERICAL_WARMUP',report['first_full_batch_warmup_ms'],flush=True)
    for trial in range(args.repeats):
        began=clock();torch.cuda.reset_peak_memory_stats()
        success=None
        if solver is None:
            rows,identifiers,details=enumerate_analytic(planner,checker,samples,args.cap)
        else:
            solutions=[];successes=[];batch_times=[];reported_times=[]
            for poses in goals:
                batch_started=clock()
                active_goal.position.copy_(poses.position)
                active_goal.quaternion.copy_(poses.quaternion)
                with torch.enable_grad():result=solver.solve_pose(active_goal,current_state=state,return_seeds=args.seeds)
                batch_times.append((clock()-batch_started)*1000)
                solutions.append(result.js_solution.position.detach().clone().reshape(batch_points,args.seeds,-1))
                successes.append(result.success.detach().clone())
                reported_times.append(float(result.solve_time))
            returned=torch.cat(solutions,dim=0)
            indices=[result.js_solution.joint_names.index(name) for name in ACTIVE_JOINTS]
            rows=returned[:,:,indices].cpu().numpy().reshape(-1,15)
            identifiers=np.repeat(np.arange(args.points),args.seeds)
            success=torch.cat(successes,dim=0).cpu().numpy().reshape(-1)
            details=dict(batch_points=batch_points,batch_count=len(goals),batch_wall_ms=batch_times,pure_ik_ms=sum(batch_times),parallel_seeds_per_point=args.seeds,parallel_joint_problems=batch_points*args.seeds,total_joint_problems=args.points*args.seeds,solver_reported_s=sum(reported_times),graph=True,collision_in_ik=False,multi_link_iterations=160,solver_mode=args.solver_mode)
        solve_ms=(clock()-began)*1000
        filtered,arrays=summarize_filter(planner,checker,samples,rows,identifiers,success)
        entry=dict(trial=trial,solve_and_pack_ms=solve_ms,total_online_ms=solve_ms+filtered['filter_and_summary_ms'],peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20,details=details,**filtered)
        report['trials'].append(entry)
        np.savez_compressed(args.output/f'trial_{trial}.npz',**arrays,sampling_progress=progress,**{side+'_targets':values for side,values in samples.items()})
        (args.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print('TRIAL',json.dumps(entry,ensure_ascii=False),flush=True)
    if solver is not None:solver.destroy()
    print('DONE',args.output,flush=True)


if __name__=='__main__':main()
