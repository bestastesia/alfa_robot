import copy
import gzip
import json

import numpy as np
from scipy.spatial.transform import Rotation
import torch
from curobo.types import JointState,GoalToolPose,Pose
from v3_batched_loaded_search import GpuValidity,scene_query_spheres
from v3_frozen_cycle import FrozenCyclePlanner
from v3_frozen_cycle_validate import OUTPUT,save
from v3_lm_conveyor import prepare_case,dense_segment,box_poses,interpolate_boxes,reference_errors,WEIGHTS,SequentialPlacementLm
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def collision_details(planner,checker,row):
    tensor=torch.tensor(np.asarray(row,dtype=np.float32)[None],device='cuda')
    checker.setup_batch_tensors(1,1)
    state=checker.kinematics.compute_kinematics(JointState.from_position(tensor[None],joint_names=ACTIVE_JOINTS))
    spheres=state.robot_spheres.reshape(1,1,-1,4)
    checker.collision_constraint.update_num_spheres(spheres.shape[2],batch_size=1,horizon=1)
    components=dict(self_collision=float(checker.get_self_collision(spheres).sum())>0,
        world=float(checker.get_collision_constraint(scene_query_spheres(spheres,.001)).sum())>0,
        base_cuboids=float(checker.base_cuboid_layer.cost(state,hard=True).sum())>0,
        limits=float(checker.get_bound(tensor[None]).sum())>0)
    values=spheres.reshape(-1,4).cpu().numpy().astype(float)
    labels={}
    for link in planner.robot['kinematics']['collision_link_names']:
        indices=checker.kinematics.config.kinematics_config.get_sphere_index_from_link_name(link).cpu().tolist()
        original=len(planner.robot['kinematics']['collision_spheres'][link])
        for offset,index in enumerate(indices):
            label=link
            if link.endswith('_link7') and offset>=original:label=link.split('_')[0]+'_box'
            labels[index]=label
    pairs=[]
    if components['self_collision']:
        config=checker.kinematics.config.self_collision_config
        indices=config.collision_pairs.cpu().numpy().astype(int)
        padding=config.sphere_padding.cpu().numpy()
        first=values[indices[:,0]]
        second=values[indices[:,1]]
        distance=np.linalg.norm(first[:,:3]-second[:,:3],axis=-1)
        depth=first[:,3]+second[:,3]+padding[indices[:,0]]+padding[indices[:,1]]-distance
        valid=np.flatnonzero((depth>1e-7)&(first[:,3]>0)&(second[:,3]>0))
        for selected in valid[np.argsort(depth[valid])[::-1]][:8]:
            first_index,second_index=indices[selected]
            weight=first[selected,3]/(first[selected,3]+second[selected,3])
            point=first[selected,:3]+weight*(second[selected,:3]-first[selected,:3])
            pairs.append(dict(kind='self_collision',first=labels[first_index],second=labels[second_index],
                first_sphere_local=values[first_index].tolist(),second_sphere_local=values[second_index].tolist(),
                first_padding=float(padding[first_index]),second_padding=float(padding[second_index]),
                depth_mm=float(depth[selected]*1000),point_local=point.tolist()))
    for kind,objects,tolerance,selected in (
        ('world',checker.guidance_scene.cuboid or [],.001,np.ones(len(values),bool)),
        ('base_cuboids',checker.base_cuboid_layer.scene.cuboid,0.,checker.base_cuboid_layer.selected.cpu().numpy())):
        if not components[kind]:continue
        for obstacle in objects:
            rotation=Rotation.from_quat(np.roll(obstacle.pose[3:7],-1)).as_matrix()
            local=(values[:,:3]-np.asarray(obstacle.pose[:3]))@rotation
            delta=np.abs(local)-np.asarray(obstacle.dims)/2
            distance=np.linalg.norm(np.maximum(delta,0),axis=-1)+np.minimum(delta.max(-1),0)
            depth=values[:,3]-distance
            valid=np.flatnonzero(selected&(values[:,3]>0)&(depth>tolerance+1e-7))
            for index in valid[np.argsort(depth[valid])[::-1]][:4]:
                point=np.clip(local[index],-np.asarray(obstacle.dims)/2,np.asarray(obstacle.dims)/2)@rotation.T+obstacle.pose[:3]
                pairs.append(dict(kind=kind,first=labels[index],second=obstacle.name,depth_mm=float(depth[index]*1000),point_local=point.tolist()))
    base_indices=checker.kinematics.config.kinematics_config.get_sphere_index_from_link_name('base_link').cpu().tolist()
    below=[index for index,value in enumerate(values) if index not in base_indices and value[3]>0 and value[2]-value[3]<0]
    components['ground']=bool(below)
    for index in below[:4]:pairs.append(dict(kind='ground',first=labels[index],second='ground',depth_mm=float((values[index,3]-values[index,2])*1000),point_local=[*values[index,:2],0.]))
    return dict(components=components,pairs=pairs)


def replay_last_candidates(planner,source,case):
    lm=source['conveyor_place_lm']
    engine=SequentialPlacementLm(planner,active_sides=tuple(sorted(source['task'])))
    columns=[source['joint_names'].index(name) for name in ACTIVE_JOINTS]
    rows=np.asarray(source['frames'],dtype=np.float32)[np.asarray(source['phases'])=='conveyor_place'][:,columns]
    progress=np.asarray(lm['progress'])
    if len(rows)==len(progress)-1:rows=np.concatenate((case['start'][None],rows))
    if len(rows)!=len(progress):raise RuntimeError('cannot align saved LM reference progress')
    point=lm['point_reports'][-1]['point']
    previous=rows[-1]
    previous_previous=rows[np.argmin(np.abs(progress-(point-2)/(engine.samples-1)))]
    generator=np.random.default_rng(20261031)
    for unused in range(point):
        noise=generator.normal(size=(engine.seeds,15))*np.deg2rad(3.)
        noise[:,0]=generator.normal(size=engine.seeds)*.005
        noise[:,engine.idle_indices]=0.
    seeds=previous+noise
    seeds[0]=previous
    seeds[1]=previous+(previous-previous_previous)
    lower,upper=[value.cpu().numpy() for value in case['checker'].kinematics.get_joint_limits().position]
    seeds=np.clip(seeds,lower+1e-6,upper-1e-6).astype(np.float32)
    boxes=interpolate_boxes(case['starts'],case['ends'],[lm['point_reports'][-1]['progress']])
    tools={side:values@np.linalg.inv(case['attachments'][side]) for side,values in boxes.items()}
    target=GoalToolPose.from_poses({side+'_tool0':Pose(position=torch.tensor(matrix[:,:3,3],device='cuda',dtype=torch.float32),
        quaternion=torch.tensor(np.roll(Rotation.from_matrix(matrix[:,:3,:3]).as_quat(),1,axis=1),device='cuda',dtype=torch.float32))
        for side,matrix in tools.items()},ordered_tool_frames=[side+'_tool0' for side in engine.active_sides])
    current=JointState.from_position(torch.tensor(previous[engine.to_solver][None],device='cuda'),joint_names=engine.solver.joint_names)
    with torch.enable_grad():
        solved=engine.solver.solve_single(target,current_state=current,seed_config=torch.tensor(seeds[:,engine.to_solver][None],device='cuda').contiguous(),return_seeds=engine.seeds)
    values=solved.js_solution.position.reshape(-1,15)[:,engine.to_active].detach().cpu().numpy()
    actual=box_poses(case['checker'],values,case['attachments'])
    position,angle=reference_errors(actual,{side:np.repeat(matrix,len(values),axis=0) for side,matrix in boxes.items()})
    mask=solved.success.reshape(-1).cpu().numpy().astype(bool)&(position<=.002)&(angle<=1.)
    engine.owner.destroy()
    return values[mask]


def main():
    summary=json.loads((OUTPUT/'summary.json').read_text())
    planner=FrozenCyclePlanner()
    records=[]
    for entry in summary['records']:
        if entry['blocked_stage']!='龙头车放置LM':continue
        path=OUTPUT/entry['result_file']
        source=json.loads(gzip.decompress(path.read_bytes()))
        lm=source['conveyor_place_lm']
        replay=copy.deepcopy(source)
        placement_start=source['phases'].index('conveyor_place')
        navigation=source['phases'].index('base_reposition')
        replay['frames'][placement_start]=source['frames'][navigation]
        case=prepare_case(planner,replay)
        columns=[source['joint_names'].index(name) for name in ACTIVE_JOINTS]
        previous=np.asarray(source['frames'][-1],dtype=np.float32)[columns]
        validity=GpuValidity(case['checker'],check_ground=True,scene_penetration_tolerance_m=.001)
        candidates=np.asarray(lm['failed_candidates'],dtype=np.float32)
        candidate_valid=validity.mask(torch.tensor(candidates,device='cuda')).cpu().numpy()
        candidate_source='saved_candidates'
        if lm['point_reports'][-1]['collision_valid'] and not candidate_valid.any():
            candidates=replay_last_candidates(planner,source,case)
            candidate_valid=validity.mask(torch.tensor(candidates,device='cuda')).cpu().numpy()
            if int(candidate_valid.sum())!=lm['point_reports'][-1]['collision_valid']:
                raise RuntimeError('one-step LM replay does not reproduce saved valid candidate count')
            candidate_source='one_step_replay_same_saved_seed_and_reference'
        evaluations=[]
        for index,candidate in enumerate(candidates):
            rows,progress=dense_segment(previous,candidate,lm['completed_progress'],lm['point_reports'][-1]['progress'])
            valid=validity.mask(torch.tensor(rows,device='cuda')).cpu().numpy()
            actual=box_poses(case['checker'],rows,case['attachments'])
            expected=interpolate_boxes(case['starts'],case['ends'],progress)
            position,angle=reference_errors(actual,expected)
            bad=np.flatnonzero(~valid|(position>.002)|(angle>1.))
            collision_bad=np.flatnonzero(~valid)
            inspection=int(collision_bad[0]) if len(collision_bad) else (int(bad[0]) if len(bad) else len(rows)-1)
            collision=collision_details(planner,case['checker'],rows[inspection])
            evaluations.append(dict(candidate=index,node_valid=bool(candidate_valid[index]),invalid_edge_samples=int((~valid).sum()),
                max_position_error_mm=float(position.max()*1000),max_orientation_error_deg=float(angle.max()),
                distance=float(np.linalg.norm((candidate-previous)*WEIGHTS)),inspection_joints=rows[inspection].tolist(),
                inspection_progress=float(progress[inspection]),collision=collision))
        eligible=[item for item in evaluations if item['node_valid']] if lm['point_reports'][-1]['collision_valid'] else evaluations
        if not eligible:raise RuntimeError('saved candidates do not include a collision-valid node; replay all LM candidates before diagnosing')
        representative=min(eligible,key=lambda item:item['distance'])
        collision=representative['collision']
        if collision['components']['self_collision']:
            rows,progress=dense_segment(previous,candidates[representative['candidate']],lm['completed_progress'],lm['point_reports'][-1]['progress'])
            state=case['checker'].kinematics.compute_kinematics(JointState.from_position(torch.tensor(rows[None],device='cuda'),joint_names=ACTIVE_JOINTS))
            spheres=state.robot_spheres.reshape(len(rows),-1,4).cpu().numpy().astype(float)
            config=case['checker'].kinematics.config.self_collision_config
            pairs=config.collision_pairs.cpu().numpy().astype(int)
            padding=config.sphere_padding.cpu().numpy()
            first=spheres[:,pairs[:,0]]
            second=spheres[:,pairs[:,1]]
            depth=first[:,:,3]+second[:,:,3]+padding[pairs[:,0]]+padding[pairs[:,1]]-np.linalg.norm(first[:,:,:3]-second[:,:,:3],axis=-1)
            depth[(first[:,:,3]<=0)|(second[:,:,3]<=0)]=-np.inf
            peak=int(np.unravel_index(np.argmax(depth),depth.shape)[0])
            representative['max_self_overlap_mm']=float(depth.max()*1000)
            representative['inspection_joints']=rows[peak].tolist()
            representative['inspection_progress']=float(progress[peak])
            representative['collision']=collision_details(planner,case['checker'],rows[peak])
            collision=representative['collision']
        if representative['invalid_edge_samples']:
            reason='collision'
        elif representative['max_position_error_mm']>2.:
            reason='reference_position_error'
        elif representative['max_orientation_error_deg']>1.:
            reason='reference_orientation_error'
        else:raise RuntimeError('saved rejected candidate does not reproduce rejection')
        record=dict(index=entry['index'],label=entry['label'],task=entry['task'],result_file=entry['result_file'],
            completed_progress=lm['completed_progress'],rejected_progress=lm['point_reports'][-1]['progress'],
            original_failure=lm['failure'],original_counts=lm['point_reports'][-1],reason=reason,
            inspected_candidate_count=len(candidates),candidate_source=candidate_source,evaluations=evaluations,representative=representative,
            last_valid_joints=previous.tolist(),last_valid_frame=len(source['frames'])-1,placement_start_frame=placement_start,
            world_base=case['world_base'].tolist(),collision_pairs=collision['pairs'],read_only=True)
        if not bool(validity.mask(torch.tensor(previous[None],device='cuda')).all()):raise RuntimeError('last accepted frame is not collision-valid')
        records.append(record)
        print('DIAG',record['label'],reason,'position_mm',round(representative['max_position_error_mm'],3),'self_overlap_mm',round(representative.get('max_self_overlap_mm',0.),4),flush=True)
    save(OUTPUT/'failure_review.json',dict(records=records,count=len(records),scope='read-only review of archived candidates; truncated archives use same-seed one-step LM reconstruction; frozen40 native collision pairs; no full-cycle replanning or threshold change'))


if __name__=='__main__':main()
