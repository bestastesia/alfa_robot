#!/usr/bin/env python3
"""Plan all five vertical door pairs in the original outer/inner/center column order."""
import argparse
from dataclasses import asdict,replace
import fcntl
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np
import torch
from curobo_core.adapter import matrix_pose,pose_matrix
from curobo_core.contracts import ACTIVE_JOINTS,PlannerAssets
from curobo_core.door_recenter import (DOOR_COLUMN_ORDER,DoorPlanner,DoorTask,ROOT,door_pair,door_request)
from curobo_core.resource_stats import resource_delta,resource_snapshot
from curobo_core.scene import AttachedObject,Pose



def _side(value):return value.replace('left','__swap__').replace('right','left').replace('__swap__','right')
def _box(value):row,column=divmod(int(value),5);return row*5+4-column
def _mirror(row):
    row=np.asarray(row,dtype=float);out=row.copy();out[1]*=-1;out[2]*=-1;q=row[3:];out[3]=q[0];out[4:11]=-q[8:15];out[11:18]=-q[1:8];return out

def mirror_round(source,source_path,planner,request):
    began=time.perf_counter();frames=np.asarray([_mirror(row) for row in source['frames']]);phases=[_side(value) for value in source['phases']]
    ids={f'wall_box_{value:02d}':f'wall_box_{_box(value):02d}' for value in range(25)};wall={x.object_id:x.pose for x in request.snapshot.objects if x.object_id.startswith('wall_box_')};active={};attachments=[];previous=set()
    for frame,items in zip(frames,source['attachments_by_frame']):
        current={ids[item['object_id']] for item in items}
        for object_id in current-previous:
            original=next(item for item in items if ids[item['object_id']]==object_id);side=_side(original['parent_link'].removesuffix('_tool0'));planner.fk(frame[3:]);tool=planner.fk_robot.get_transform(side+'_tool0',planner.fk_robot.base_link);yaw=frame[2];base=np.eye(4);base[:3,:3]=((math.cos(yaw),-math.sin(yaw),0),(math.sin(yaw),math.cos(yaw),0),(0,0,1));base[:2,3]=frame[:2];local=np.linalg.inv(base)@pose_matrix(wall[object_id]);active[object_id]=AttachedObject(object_id,tuple(original['dimensions_m']),side+'_tool0',matrix_pose(np.linalg.inv(tool)@local),(side+'_link7',))
        for object_id in previous-current:active.pop(object_id,None)
        attachments.append([asdict(active[x]) for x in sorted(current)]);previous=current
    result=dict(source);first=frames[0];yaw=first[2];base=Pose((first[0],first[1],0.),(math.cos(yaw/2),0.,0,math.sin(yaw/2)));initial=replace(request.snapshot,state=request.snapshot.state.with_positions(dict(zip(ACTIVE_JOINTS,first[3:])),base));result.update(success=True,column=0,mode='door_recenter_mirrored_symmetry',frames=frames.tolist(),phases=phases,task={_side(k):_box(v) for k,v in source['task'].items()},attachments_by_frame=attachments,payload=[bool(x) for x in attachments],initial_snapshot=initial.to_dict(),pick_base_pose=asdict(base),symmetry_source=str(source_path),symmetry_source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),navigation_path_validated=False,plan_cache=dict(enabled=False,hit=False,symmetry_derived=True))
    for key in ('velocities','accelerations','jerks'):result[key]=[_mirror(row).tolist() for row in source[key]]
    result['timed_segments']=[dict(segment,**{key:[_mirror(row).tolist() for row in segment[key]] for key in ('positions','velocities','accelerations','jerks') if key in segment}) for segment in source.get('timed_segments',())]
    release=phases.index('release');result['predicted_scene_events']=[dict(phase='release',object_id=f'wall_box_{value:02d}',frame_index=release) for value in (15,20)];final=frames[-1];yaw=final[2];base=Pose((final[0],final[1],0.),(math.cos(yaw/2),0.,0,math.sin(yaw/2)));objects=tuple(x for x in request.snapshot.objects if x.object_id not in {'wall_box_15','wall_box_20'});result['final_snapshot']=replace(request.snapshot,objects=objects,attachments=(),state=request.snapshot.state.with_positions(dict(zip(ACTIVE_JOINTS,final[3:])),base)).to_dict();task=DoorTask.__new__(DoorTask);task.planner=planner;task.report=result;task.phase='rnea_dynamics';task.fail=lambda message:(_ for _ in ()).throw(RuntimeError(message));task.apply_dynamics();result['symmetry_torque_recomputed']=True;result['source_planning_total_ms']=source.get('total_ms');result['source_stage_timing_ms']=source.get('stage_timing_ms',{});result['symmetry_generation_ms']=(time.perf_counter()-began)*1000;result['total_ms']=result['symmetry_generation_ms'];result['stage_timing_ms']={'symmetry_transform_and_rnea':result['total_ms']};result['stage_breakdown_ms']={'symmetry_transform_and_rnea':{'symmetry_and_dynamics':result['total_ms']}};result['stage_timing_spans']=[{'stage':'symmetry_transform_and_rnea','start_ms':0.,'wall_ms':result['total_ms']}];return result

ASSET_NAMES=('robot_config','mobile_robot_config','urdf','named_poses','box_fit')

def acquire_gpu(lock,wait_s):
    """Hold the shared GPU lock and require no other compute process before timing."""
    deadline=time.monotonic()+wait_s
    while True:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:occupied='shared lock held'
        else:
            occupied=subprocess.check_output(['nvidia-smi','--query-compute-apps=pid','--format=csv,noheader'],text=True).strip()
            if not occupied:return
            fcntl.flock(lock,fcntl.LOCK_UN)
        if time.monotonic()>=deadline:raise RuntimeError('GPU unavailable: '+occupied)
        time.sleep(1)

def load_assets(assets_root):
    assets=PlannerAssets()
    return replace(assets,**{name:assets_root/getattr(assets,name).relative_to(ROOT) for name in ASSET_NAMES})

def asset_identity(assets):
    return {name:dict(path=str(getattr(assets,name)),sha256=hashlib.sha256(getattr(assets,name).read_bytes()).hexdigest()) for name in ASSET_NAMES}

def source_identity():
    return {str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted((ROOT/'tools').rglob('*.py'))}

def plan_round(planner,number,column,removed,seed,policy,context,rounds,progress=print):
    """One door pair, timed end to end from the request to the RNEA-validated trajectory."""
    torch.cuda.synchronize();torch.cuda.reset_peak_memory_stats();resources=resource_snapshot(planner)
    round_started=time.perf_counter();error=None
    request=door_request(planner,seed,column,removed);planner.set_snapshot(request.snapshot)
    if column==0:
        source_path=Path(rounds[0]['path']);report=mirror_round(json.loads(source_path.read_text()),source_path,planner,request)
        report.update(initial_snapshot=request.snapshot.to_dict(),column=column,round_index=number,removed_box_ids=sorted(removed),**context)
    else:
        task=DoorTask(planner,request,'joint_rrt',policy,progress=lambda text:progress(f'列{column} · {text}'))
        task.left_grasp_rolls_deg=(0.,30.,45.);task.right_grasp_rolls_deg=(0.,30.,-30.,45.,-45.)
        task.report['requested_left_grasp_rolls_deg']=list(task.left_grasp_rolls_deg);task.report['requested_right_grasp_rolls_deg']=list(task.right_grasp_rolls_deg)
        try:task.run();task.enter('time_parameterization');task.finalize_timing();task.enter('rnea_dynamics');task.apply_dynamics()
        except Exception as exc:
            import traceback
            error=dict(type=type(exc).__name__,stage=task.phase,message=str(exc),traceback=traceback.format_exc());task.report.update(success=False,error=error)
        finally:
            task.close_stage_timing();task.report.update(initial_snapshot=request.snapshot.to_dict(),final_snapshot=task.store.snapshot().to_dict(),
                plan_cache=dict(enabled=False,hit=False),column=column,round_index=number,removed_box_ids=sorted(removed),**context)
        report=task.report
    torch.cuda.synchronize();report['total_ms']=(time.perf_counter()-round_started)*1000
    report['resource_builds']=resource_delta(resources,resource_snapshot(planner));report['peak_gpu_memory_mb']=torch.cuda.max_memory_allocated()/2**20
    return report,error

def run_sequence(planner,seed,policy,policy_path,assets,output_dir,progress=print):
    """Plan the five rounds on one resident planner; each round replans and removes the released pair."""
    output_dir.mkdir(parents=True,exist_ok=True)
    if any(output_dir.iterdir()):raise ValueError(f'output directory must be empty: {output_dir}')
    removed=set();rounds=[];aggregate=dict(
        success=False,sequence=True,sequence_kind='five_vertical_door_pairs',
        column_order=list(DOOR_COLUMN_ORDER),frames=[],phases=[],payload=[],attachments_by_frame=[],
        time_from_start_s=[],velocities=[],accelerations=[],jerks=[],frame_rounds=[],predicted_scene_events=[],
        stage_timing_ms={},stage_breakdown_ms={},stage_timing_spans=[],timed_segments=[],
        contact_geometry=[],round_frame_ranges=[],dual_recenter_verifications=[],torque_validations=[])
    began=time.perf_counter();identity=source_identity()
    context=dict(source_identity=identity,model_id=planner.model_id,assets=asset_identity(assets),
                 policy_sha256=hashlib.sha256(policy_path.read_bytes()).hexdigest())
    for number,column in enumerate(DOOR_COLUMN_ORDER):
        lower,upper=door_pair(column)
        report,error=plan_round(planner,number,column,removed,seed,policy,context,rounds,progress)
        path=output_dir/f'round-{number+1:02d}-column-{column}.json';path.write_text(json.dumps(report,ensure_ascii=False)+'\n')
        record=dict(round=number+1,column=column,lower_box=lower,upper_box=upper,path=str(path),
                    success=report['success'],total_ms=report['total_ms'],motion_duration_s=report.get('motion_duration_s'),error=error,
                    resource_builds=report['resource_builds'],peak_gpu_memory_mb=report['peak_gpu_memory_mb'])
        rounds.append(record);progress(json.dumps(record,ensure_ascii=False))
        if not report['success']:break
        if report['final_snapshot']['attachments']:
            raise RuntimeError('round ended with attached boxes')
        offset=aggregate['time_from_start_s'][-1] if aggregate['time_from_start_s'] else 0.
        frame_offset=len(aggregate['frames'])
        for key in ('frames','phases','payload','attachments_by_frame','velocities','accelerations','jerks'):
            aggregate[key].extend(report[key])
        aggregate['time_from_start_s'].extend(offset+value for value in report['time_from_start_s'])
        aggregate['frame_rounds'].extend([number]*len(report['frames']))
        for stage,value in report.get('stage_timing_ms',{}).items():
            aggregate['stage_timing_ms'][stage]=aggregate['stage_timing_ms'].get(stage,0.)+value
        for stage,detail in report.get('stage_breakdown_ms',{}).items():
            target=aggregate['stage_breakdown_ms'].setdefault(stage,{})
            for key,value in detail.items():target[key]=target.get(key,0.)+value
        round_start_ms=sum(item['total_ms'] for item in rounds[:-1])
        aggregate['stage_timing_spans'].extend(dict(item,start_ms=item['start_ms']+round_start_ms,
            round=number,column=column) for item in report.get('stage_timing_spans',()))
        aggregate['timed_segments'].extend(dict(item,frame_start=item['frame_start']+frame_offset,
            frame_end=item['frame_end']+frame_offset,round=number,column=column)
            for item in report.get('timed_segments',()))
        aggregate['contact_geometry'].extend(dict(item,round=number,column=column)
            for item in report.get('contact_geometry',()))
        aggregate['dual_recenter_verifications'].append(report.get('dual_recenter_verification'))
        aggregate['torque_validations'].append(report.get('torque_validation'))
        aggregate['round_frame_ranges'].append(dict(round=number,column=column,start=frame_offset,
            end=len(aggregate['frames'])-1,lower_box=lower,upper_box=upper))
        for event in report['predicted_scene_events']:
            event=dict(event);event['frame_index']+=frame_offset;event['round']=number;event['column']=column
            if 'geometric_frame_index' in event:event['geometric_frame_index']+=frame_offset
            aggregate['predicted_scene_events'].append(event)
        removed.update((lower,upper))
    torque=[item for item in aggregate['torque_validations'] if item]
    compact=[item for item in aggregate['dual_recenter_verifications'] if item]
    aggregate.update(rounds=rounds,completed_box_ids=sorted(removed),remaining_box_ids=sorted(set(range(25))-removed),
                     total_ms=(time.perf_counter()-began)*1000,success=len(rounds)==5 and all(item['success'] for item in rounds),
                     motion_duration_s=aggregate['time_from_start_s'][-1] if aggregate['time_from_start_s'] else 0.,
                     joint_names=rounds and json.loads(Path(rounds[0]['path']).read_text())['joint_names'],
                     initial_snapshot=door_request(planner,seed,DOOR_COLUMN_ORDER[0],()).snapshot.to_dict(),
                     final_snapshot=report['final_snapshot'],carry_policy=policy,plan_cache=dict(enabled=False,hit=False),source_identity=identity,
                     torque_validation=dict(success=bool(torque) and all(item.get('success') for item in torque),
                        maximum_utilization=max((item.get('maximum_utilization',0.) for item in torque),default=0.)),
                     dual_recenter_verification=dict(actual_whole_robot_payload_radius_m=max(
                        (item.get('actual_whole_robot_payload_radius_m',0.) for item in compact),default=0.)))
    (output_dir/'five-columns.json').write_text(json.dumps(aggregate,ensure_ascii=False)+'\n')
    (output_dir/'summary.json').write_text(json.dumps({k:aggregate.get(k) for k in
        ('success','column_order','completed_box_ids','remaining_box_ids','total_ms','motion_duration_s')},ensure_ascii=False,indent=2)+'\n')
    return aggregate

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets-root',type=Path,required=True)
    p.add_argument('--seed',type=int,default=11)
    p.add_argument('--wait-for-gpu-seconds',type=float,default=0.)
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=True)
    if any(a.output_dir.iterdir()):p.error('output directory must be empty')
    with open('/tmp/sevenova-curobo-gpu.lock','a') as lock:
        acquire_gpu(lock,a.wait_for_gpu_seconds)
        assets=load_assets(a.assets_root)
        policy_path=ROOT/'config/placement_task_policy.json';policy=json.loads(policy_path.read_text())
        aggregate=run_sequence(DoorPlanner(assets),a.seed,policy,policy_path,assets,a.output_dir,progress=lambda text:print(text,flush=True))
        raise SystemExit(0 if aggregate['success'] else 1)

if __name__=='__main__':main()
