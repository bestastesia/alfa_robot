import copy
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.scene import Scene, Cuboid
from curobo.types import GoalToolPose, JointState, Pose
from curobo_core.contracts import PlannerAssets
from v3_batched_loaded_search import densify
from v3_guidance_continuity_report import analyze
from v3_guided_cycle_planner import SumCosts
from v3_multiroot_search import batched_rrt_connect_multi_goal
from v3_route_guided_compare import RouteGuidance, TcpRouteCost
from v3_route_ik_benchmark import enumerate_analytic
from v3_wall_ik_benchmark import ACTIVE_JOINTS
from v3_wrist_guided_cycle_planner import WristGuidedCyclePlanner


ROOT=Path('/mnt/mydisk/ALFA/alfa_robot_v3_curobo/research/curobo_v3/generated/wrist_all_stage_20261022')


def endpoint_references(start, goal, count=128):
    progress=np.linspace(0,1,count)
    return (1-progress[:,None])*np.asarray(start)+progress[:,None]*np.asarray(goal)


def biased_seeds(reference,lower,upper,seed=20261023):
    generator=np.random.default_rng(seed)
    noise=generator.normal(size=(16,15))*np.deg2rad(15)
    noise[:,0]*=.025/np.deg2rad(15)
    noise[0]=0
    values=np.asarray(reference)[:,None,:]+noise[None]
    values[:,12:]=generator.uniform(lower,upper,size=(len(reference),4,15))
    return np.clip(values,lower,upper).astype(np.float32)


def pose_filter(checker, validity, rows,ids,samples):
    legal=[];collisions=[]
    lower,upper=checker.kinematics.get_joint_limits().position
    for offset in range(0,len(rows),512):
        batch=rows[offset:offset+512]
        collision=validity.mask(batch)
        valid=collision&((batch>=lower-1e-6)&(batch<=upper+1e-6)).all(-1)
        state=checker.kinematics.compute_kinematics(JointState.from_position(batch[None],joint_names=ACTIVE_JOINTS))
        for side in ('left','right'):
            target=samples[side][ids[offset:offset+512]]
            pose=state.tool_poses.to_dict()[side+'_tool0']
            position=torch.tensor(target[:,:3,3],device='cuda',dtype=torch.float32)
            quaternion=torch.tensor(np.roll(Rotation.from_matrix(target[:,:3,:3]).as_quat(),1,axis=1),device='cuda',dtype=torch.float32)
            valid&=torch.linalg.vector_norm(pose.position.reshape(-1,3)-position,dim=-1)<=.002
            valid&=(pose.quaternion.reshape(-1,4)*quaternion).sum(-1).abs()>=math.cos(np.deg2rad(1)/2)
        legal.extend(valid.cpu().tolist());collisions.extend(collision.cpu().tolist())
    return np.array(legal),np.array(collisions)


def polish(checker,rows,ids,samples,references,iterations=32):
    output=[]
    lower,upper=checker.kinematics.get_joint_limits().position
    weights=torch.tensor([np.deg2rad(15)/.1]+[1.]*14,device='cuda',dtype=torch.float32)
    for offset in range(0,len(rows),512):
        initial=rows[offset:offset+512].detach().clone()
        indices=ids[offset:offset+512]
        reference=torch.tensor(references[indices],device='cuda',dtype=torch.float32)
        target_positions={side:torch.tensor(matrix[indices,:3,3],device='cuda',dtype=torch.float32) for side,matrix in samples.items()}
        target_quaternions={side:torch.tensor(np.roll(Rotation.from_matrix(matrix[indices,:3,:3]).as_quat(),1,axis=1),device='cuda',dtype=torch.float32) for side,matrix in samples.items()}
        with torch.enable_grad():
            variable=initial.clone().requires_grad_(True)
            optimizer=torch.optim.Adam([variable],lr=.004)
            best=initial.clone()
            best_cost=torch.full((len(initial),),float('inf'),device='cuda')
            for iteration in range(iterations+1):
                state=checker.kinematics.compute_kinematics(JointState.from_position(variable[None],joint_names=ACTIVE_JOINTS))
                pose_cost=torch.zeros(len(initial),device='cuda');feasible=torch.ones(len(initial),device='cuda',dtype=torch.bool)
                for side in ('left','right'):
                    pose=state.tool_poses.to_dict()[side+'_tool0']
                    difference=pose.position.reshape(-1,3)-target_positions[side]
                    cosine=(pose.quaternion.reshape(-1,4)*target_quaternions[side]).sum(-1)
                    pose_cost+=10000*difference.square().sum(-1)+500*(1-cosine.square())
                    feasible&=torch.linalg.vector_norm(difference,dim=-1)<=.002
                    feasible&=cosine.abs()>=math.cos(np.deg2rad(1)/2)
                soft=.01*((variable-reference)*weights).square().mean(-1)
                safe=torch.minimum(-lower[[6,13]],upper[[6,13]])-np.deg2rad(20)
                soft+=.1*torch.relu(variable[:,[6,13]].abs()-safe).square().sum(-1)
                cost=pose_cost+soft
                feasible&=((variable>=lower)&(variable<=upper)).all(-1)
                take=feasible&(cost.detach()<best_cost)
                best[take]=variable.detach()[take];best_cost[take]=cost.detach()[take]
                if iteration==iterations:
                    break
                optimizer.zero_grad()
                cost.sum().backward()
                optimizer.step()
                with torch.no_grad():
                    variable.clamp_(lower,upper)
        output.append(best.detach())
    return torch.cat(output)


def solve_numeric(planner,method,samples,start,references,checker,modified,cache):
    key=(method,modified)
    began=time.perf_counter()
    setup=0.
    if key not in cache:
        config=InverseKinematicsCfg.create(robot=copy.deepcopy(planner.robot),scene_model=None,self_collision_check=False,
            num_seeds=16,seed_solver_num_seeds=16,max_batch_size=64,position_tolerance=.002,orientation_tolerance=np.deg2rad(1),
            use_cuda_graph=True,override_iters_for_multi_link_ik=160,random_seed=20261021)
        config.exit_early=method=='lm';config.exit_early_batch_success_threshold=0.
        cache[key]=[InverseKinematics(config),None]
        setup=(time.perf_counter()-began)*1000
    solver,buffer=cache[key]
    lower,upper=[value.cpu().numpy() for value in checker.kinematics.get_joint_limits().position]
    seeds=biased_seeds(references,lower,upper) if modified else None
    rows=[]
    torch.cuda.synchronize();began=time.perf_counter()
    for offset in (0,64):
        goals=GoalToolPose.from_poses({side+'_tool0':Pose(position=torch.tensor(matrix[offset:offset+64,:3,3],device='cuda',dtype=torch.float32),
            quaternion=torch.tensor(np.roll(Rotation.from_matrix(matrix[offset:offset+64,:3,:3]).as_quat(),1,axis=1),device='cuda',dtype=torch.float32)) for side,matrix in samples.items()},ordered_tool_frames=['left_tool0','right_tool0'])
        if buffer is None:buffer=copy.deepcopy(goals)
        else:
            buffer.position.copy_(goals.position);buffer.quaternion.copy_(goals.quaternion)
        state_rows=references[offset:offset+64] if modified else np.repeat(start[None],64,axis=0)
        state=JointState.from_position(torch.tensor(state_rows,device='cuda',dtype=torch.float32),joint_names=ACTIVE_JOINTS)
        arguments={}
        if modified:arguments['seed_config']=torch.tensor(seeds[offset:offset+64],device='cuda')
        with torch.enable_grad():result=solver.solve_pose(buffer,current_state=state,return_seeds=16,**arguments)
        indices=[result.js_solution.joint_names.index(name) for name in ACTIVE_JOINTS]
        rows.append(result.js_solution.position.reshape(-1,len(result.js_solution.joint_names))[:,indices].detach().clone())
    cache[key][1]=buffer
    rows=torch.cat(rows)
    torch.cuda.synchronize()
    solve_ms=(time.perf_counter()-began)*1000
    ids=np.repeat(np.arange(128),16)
    began=time.perf_counter()
    if modified:rows=polish(checker,rows,ids,samples,references)
    torch.cuda.synchronize()
    return rows,ids,dict(solve_ms=solve_ms,soft_refine_ms=(time.perf_counter()-began)*1000,solver_setup_ms=setup)


def main():
    output=ROOT/'endpoint_guidance_20261023'
    output.mkdir(exist_ok=True)
    source=ROOT/'worst_three_ik'
    selection=[[],[],['approach'],['approach','return_home'],['approach','return_home'],['transport'],['conveyor_place'],['approach','return_home']]
    planner=WristGuidedCyclePlanner(PlannerAssets(),ROOT/'all119/frozen_wrist_settings.json')
    cache={};manifest=[]
    for task_index,item in enumerate(json.loads((source/'manifest.json').read_text())):
        original=json.loads((source/item['path']).read_text())
        chosen=[stage for stage in original['comparisons'] if stage['phase'] in selection[task_index]] or [None]
        for stage in chosen:
            label=item['label']+' · '+(stage['phase'] if stage else '前置抽离失败')
            saved_filename=f'{len(manifest):02d}.json'
            if (output/saved_filename).exists():
                manifest.append(dict(label=label,task=item['task'],path=saved_filename))
                continue
            print('STAGE',label,flush=True)
            document=dict(label=label,task=item['task'],methods={},blocked_stage=original.get('blocked_stage'),blocked_details=original.get('blocked_details'))
            if stage is not None:
                planner.configure_task(item['task'])
                payload=stage['phase'] in ('transport','conveyor_place')
                checker=planner.checker(item['task'],payload=payload)
                objects=stage['reference_metadata']['reference_scene_objects']
                checker.update_world(Scene(cuboid=[Cuboid(name=obj['name'],dims=obj['dimensions'],pose=[*obj['position'],*obj['quaternion_wxyz']]) for obj in objects]))
                samples={side:np.asarray(matrix) for side,matrix in stage['methods']['lm']['node_diagnostics']['targets'].items()}
                roots=np.asarray(stage['start']);goals=np.asarray(stage['goals'])
                weights=np.array([np.deg2rad(15)/.1]+[1.]*14)
                distance=np.linalg.norm((roots[:,None]-goals[None])*weights,axis=-1)
                pair=np.unravel_index(distance.argmin(),distance.shape)
                start,goal=roots[pair[0]],goals[pair[1]]
                reference=endpoint_references(start,goal)
                document.update(phase=stage['phase'],reference_tracks=stage['reference_tracks'],scene_objects=objects,
                    start=start.tolist(),goal=goal.tolist(),joint_reference=reference.tolist(),seed=stage['seed'],budget_s=2.)
                for method in ('lm','full','analytic'):
                    document['methods'][method]={}
                    for modified in (False,True):
                        variant='endpoint_soft' if modified else 'original'
                        validity=planner.single_validity(checker,start,goal,payload) if len(item['task'])==1 else None
                        if validity is None:
                            from v3_batched_loaded_search import GpuValidity
                            from v3_box_rotation_cost import BoxRotationCost
                            validity=GpuValidity(checker,check_ground=True)
                            if payload:validity.box_rotation_cost=BoxRotationCost(checker,ACTIVE_JOINTS,torch.tensor(start,device='cuda',dtype=torch.float32),torch.tensor(goal,device='cuda',dtype=torch.float32),planner.top_grasp)
                        validity.weights=torch.tensor(weights,device='cuda',dtype=torch.float32)
                        began=time.perf_counter()
                        if method=='analytic':
                            rows,ids,_=enumerate_analytic(planner,checker,samples,64,references=reference if modified else None,wrist_weight=4. if modified else 0.)
                            rows=torch.tensor(np.asarray(rows),device='cuda',dtype=torch.float32);ids=np.asarray(ids)
                            solve_stats=dict(solve_ms=(time.perf_counter()-began)*1000,soft_refine_ms=0.,solver_setup_ms=0.)
                        else:rows,ids,solve_stats=solve_numeric(planner,method,samples,start,reference,checker,modified,cache)
                        began=time.perf_counter()
                        legal,collision=pose_filter(checker,validity,rows,ids,samples)
                        entry=dict(**solve_stats,filter_ms=(time.perf_counter()-began)*1000,success=False,
                            node_diagnostics=dict(joints=rows.cpu().tolist(),point_indices=ids.tolist(),legal=legal.tolist(),collision_valid=collision.tolist(),targets={s:t.tolist() for s,t in samples.items()}))
                        entry['continuity_analysis']=analyze(entry)
                        entry['legal_nodes']=int(legal.sum());entry['covered_points']=len(np.unique(ids[legal]))
                        entry['joint6_margin_deg']={side:float(np.rad2deg((np.deg2rad(110)-rows[torch.tensor(legal,device='cuda'),index].abs()).min().item())) if legal.any() else None for side,index in [('left',6),('right',13)]}
                        cost=TcpRouteCost(checker,samples,validity.weights)
                        if legal.any():
                            start_tensor=torch.tensor(start,device='cuda',dtype=torch.float32);goal_tensor=torch.tensor(goal[None],device='cuda',dtype=torch.float32)
                            validity.post_sampling_transform=RouteGuidance(rows.cpu().numpy()[legal],ids[legal],start_tensor,goal_tensor,validity.weights)
                            validity.box_rotation_cost=SumCosts(getattr(validity,'box_rotation_cost',None),cost)
                            lower,upper=checker.kinematics.get_joint_limits().position
                            began=time.perf_counter()
                            path,search=batched_rrt_connect_multi_goal(start_tensor,goal_tensor,lower,upper,validity,2.,stage['seed'],informed_sampling=True,stop_at_first_valid=True)
                            entry['search_ms']=(time.perf_counter()-began)*1000;entry['search']=search
                            if path is not None:
                                dense=np.asarray(densify(path));tensor=torch.tensor(dense,device='cuda',dtype=torch.float32)
                                assert validity.mask(tensor).all()
                                metrics=[cost.describe(chunk) for chunk in tensor.split(512)]
                                positions=torch.cat([value[3] for value in metrics]).cpu().numpy();quaternions=torch.cat([value[4] for value in metrics]).cpu().numpy()
                                entry.update(success=True,trajectory=dense.tolist(),tracks={})
                                for arm,side in enumerate(('left','right')):
                                    rotation=Rotation.from_quat(np.roll(quaternions[:,arm],-1,axis=1))
                                    target=samples[side][-1]
                                    assert np.linalg.norm(positions[-1,arm]-target[:3,3])<=.002
                                    assert (rotation[-1].inv()*Rotation.from_matrix(target[:3,:3])).magnitude()<=np.deg2rad(1)
                                    entry['tracks'][side]=dict(positions=positions[:,arm].tolist(),quaternions_wxyz=quaternions[:,arm].tolist(),total_rotation_deg=float(np.rad2deg((rotation[:-1].inv()*rotation[1:]).magnitude()).sum()))
                        document['methods'][method][variant]=entry
                        print(method,variant,entry['success'],entry['covered_points'],entry['continuity_analysis']['full_128_chain_minimax_deg'],flush=True)
            filename=f'{len(manifest):02d}.json'
            (output/filename).write_text(json.dumps(document,ensure_ascii=False))
            manifest.append(dict(label=label,task=item['task'],path=filename))
            (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
