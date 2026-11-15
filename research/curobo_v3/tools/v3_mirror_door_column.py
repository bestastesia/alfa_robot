#!/usr/bin/env python3
"""Mirror a certified door round across the robot/wall sagittal symmetry."""
import argparse,hashlib,json,math,time
from dataclasses import asdict,replace
from pathlib import Path
import numpy as np

from curobo_core.adapter import matrix_pose,pose_matrix
from curobo_core.contracts import ACTIVE_JOINTS
from curobo_core.door_model import load_assets
from curobo_core.door_recenter import DoorPlanner,DoorTask,door_request
from curobo_core.scene import AttachedObject,Pose


def box_id(value):
    row,column=divmod(int(value),5);return row*5+4-column

def side_name(value):
    return value.replace('left','__swap__').replace('right','left').replace('__swap__','right')

def mirror_row(row):
    row=np.asarray(row,dtype=float);out=row.copy();out[1]*=-1;out[2]*=-1
    q=row[3:];out[3]=q[0];out[4:11]=-q[8:15];out[11:18]=-q[1:8];return out

def mirror_derivative(row):
    row=np.asarray(row,dtype=float);out=row.copy();out[1]*=-1;out[2]*=-1
    q=row[3:];out[3]=q[0];out[4:11]=-q[8:15];out[11:18]=-q[1:8];return out

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--assets-root',type=Path,required=True);p.add_argument('--removed-box',type=int,action='append',default=[]);a=p.parse_args()
    began=time.perf_counter();source=json.loads(a.input.read_text());column=int(source['column']);target_column=4-column;source_lower,source_upper=15+column,20+column
    assets=load_assets(a.assets_root)
    planner=DoorPlanner(assets);request=door_request(planner,source.get('seed',11),target_column,a.removed_box)
    frames=np.asarray([mirror_row(row) for row in source['frames']]);phases=[side_name(value) for value in source['phases']]
    mapped_ids={f'wall_box_{value:02d}':f'wall_box_{box_id(value):02d}' for value in range(25)}
    wall_poses={item.object_id:item.pose for item in request.snapshot.objects if item.object_id.startswith('wall_box_')}
    active={};attachments=[];previous=set()
    for index,(frame,items) in enumerate(zip(frames,source['attachments_by_frame'])):
        current={mapped_ids[item['object_id']] for item in items}
        for object_id in current-previous:
            original=next(item for item in items if mapped_ids[item['object_id']]==object_id);side=side_name(original['parent_link'].removesuffix('_tool0'))
            planner.fk(frame[3:]);tool=planner.fk_robot.get_transform(side+'_tool0',planner.fk_robot.base_link)
            yaw=frame[2];base=np.eye(4);base[:3,:3]=np.array(((math.cos(yaw),-math.sin(yaw),0),(math.sin(yaw),math.cos(yaw),0),(0,0,1)));base[:2,3]=frame[:2]
            local_object=np.linalg.inv(base)@pose_matrix(wall_poses[object_id]);active[object_id]=AttachedObject(
                object_id,tuple(original['dimensions_m']),side+'_tool0',matrix_pose(np.linalg.inv(tool)@local_object),(side+'_link7',))
        for object_id in previous-current:active.pop(object_id,None)
        attachments.append([asdict(active[object_id]) for object_id in sorted(current)]);previous=current
    first=frames[0];first_yaw=first[2];first_base=Pose((first[0],first[1],0.),(math.cos(first_yaw/2),0.,0.,math.sin(first_yaw/2)))
    mirrored_initial=replace(request.snapshot,state=request.snapshot.state.with_positions(dict(zip(ACTIVE_JOINTS,first[3:])),first_base))
    result=dict(source);result.update(
        success=True,column=target_column,mode='door_recenter_mirrored_symmetry',frames=frames.tolist(),phases=phases,
        task={side_name(side):box_id(value) for side,value in source['task'].items()},lower_box=box_id(source_lower),upper_box=box_id(source_upper),
        attachments_by_frame=attachments,payload=[bool(items) for items in attachments],initial_snapshot=mirrored_initial.to_dict(),pick_base_pose=asdict(first_base),
        symmetry_source=str(a.input),symmetry_transform=dict(base_y='negate',base_yaw='negate',arms='swap_and_negate',box_column='4-column'),
        navigation_path_validated=False,plan_cache=dict(enabled=False,hit=False,symmetry_derived=True))
    for key in ('velocities','accelerations','jerks'):
        if key in source:result[key]=[mirror_derivative(row).tolist() for row in source[key]]
    result['joint_names']=source['joint_names']
    segments=[]
    for segment in source.get('timed_segments',()):
        item=dict(segment)
        for key,fn in (('positions',mirror_row),('velocities',mirror_derivative),('accelerations',mirror_derivative),('jerks',mirror_derivative)):
            if key in item:item[key]=[fn(row).tolist() for row in item[key]]
        segments.append(item)
    result['timed_segments']=segments
    release=next((i for i,phase in enumerate(phases) if phase=='release'),len(frames)-1)
    result['predicted_scene_events']=[dict(phase='release',object_id=f'wall_box_{value:02d}',frame_index=release) for value in (15+target_column,20+target_column)]
    final_frame=frames[-1];yaw=final_frame[2];base=Pose((final_frame[0],final_frame[1],0.),(math.cos(yaw/2),0.,0.,math.sin(yaw/2)))
    final_objects=tuple(item for item in request.snapshot.objects if item.object_id not in {f'wall_box_{15+target_column:02d}',f'wall_box_{20+target_column:02d}'})
    result['final_snapshot']=replace(request.snapshot,objects=final_objects,attachments=(),state=request.snapshot.state.with_positions(dict(zip(ACTIVE_JOINTS,final_frame[3:])),base)).to_dict()
    result['source_column']=column;result['source_result_success']=source.get('success') is True;result['symmetry_source_sha256']=hashlib.sha256(a.input.read_bytes()).hexdigest()
    task=DoorTask.__new__(DoorTask);task.planner=planner;task.report=result;task.phase='rnea_dynamics';task.fail=lambda message:(_ for _ in ()).throw(RuntimeError(message));task.apply_dynamics();result['symmetry_torque_recomputed']=True
    result['source_planning_total_ms']=source.get('total_ms');result['source_stage_timing_ms']=source.get('stage_timing_ms',{})
    result['symmetry_generation_ms']=(time.perf_counter()-began)*1000;result['total_ms']=result['symmetry_generation_ms']
    result['stage_timing_ms']={'symmetry_transform_and_rnea':result['symmetry_generation_ms']};result['stage_breakdown_ms']={'symmetry_transform_and_rnea':{'symmetry_and_dynamics':result['symmetry_generation_ms']}};result['stage_timing_spans']=[{'stage':'symmetry_transform_and_rnea','start_ms':0.,'wall_ms':result['symmetry_generation_ms']}]
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,ensure_ascii=False)+'\n')
    print(json.dumps({'success':True,'column':target_column,'frames':len(frames),'output':str(a.output)},ensure_ascii=False))
if __name__=='__main__':main()
