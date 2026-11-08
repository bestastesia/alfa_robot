#!/usr/bin/env python3
"""Independent all-frame door-side validation for a mirrored door round."""
import argparse,hashlib,json,math
from dataclasses import replace
from pathlib import Path
import numpy as np,torch
from curobo.collision_checking import RobotCollisionChecker,RobotCollisionCheckerCfg
from v3_batched_loaded_search import loaded_robot
from curobo_core.adapter import to_curobo_scene
from curobo_core.contracts import ACTIVE_JOINTS,PlannerAssets
from curobo_core.door_recenter import DoorPlanner,door_request,snapshot_in_base
from curobo_core.door_sequential import tool_mesh_vertices
from curobo_core.door_sequential_validity import SequentialValidity
from curobo_core.scene import AttachedObject,Pose

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--result',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--assets-root',type=Path,required=True);a=p.parse_args();d=json.loads(a.result.read_text());root=Path(__file__).resolve().parents[1]
 assets=PlannerAssets();assets=replace(assets,**{n:a.assets_root/getattr(assets,n).relative_to(root) for n in ('robot_config','mobile_robot_config','urdf','named_poses','box_fit')});planner=DoorPlanner(assets)
 column=int(d['column']);request=door_request(planner,d.get('seed',11),column,tuple(d.get('removed_box_ids',(19,24))));base_objects=request.snapshot.objects;names=d['joint_names'];frames=np.asarray(d['frames']);upper,lower=20+column,15+column;release=min((i for i,p in enumerate(d['phases']) if p=='release'),default=len(frames));door_end=min((i for i,p in enumerate(d['phases']) if p=='base_reposition'),default=len(frames));groups={};frame_snapshots={}
 for i in range(door_end):
  frame=frames[i];items=tuple(AttachedObject(x['object_id'],x['dimensions_m'],x['parent_link'],Pose(**x['tool_to_object']),x.get('touch_links',())) for x in d['attachments_by_frame'][i]);attached={x.object_id for x in items};objects=tuple(x for x in base_objects if x.object_id not in attached and not (i>=release and x.object_id in {f'wall_box_{lower:02d}',f'wall_box_{upper:02d}'}));phase=d['phases'][i];contact=None
  if phase=='left_contact':contact=('left',f'wall_box_{upper:02d}')
  elif phase=='right_contact':contact=('right',f'wall_box_{lower:02d}')
  elif phase=='rack_support_transfer':contact=('left',f'wall_box_{upper:02d}')
  yaw=frame[2];base=Pose((frame[0],frame[1],0.),(math.cos(yaw/2),0.,0.,math.sin(yaw/2)));snapshot=replace(request.snapshot,objects=objects,attachments=items,state=request.snapshot.state.with_positions(dict(zip(ACTIVE_JOINTS,frame[3:])),base),revision=0);snapshot=snapshot_in_base(snapshot)
  if contact is None and f'wall_box_{upper:02d}' not in attached and any(x.object_id==f'wall_box_{upper:02d}' for x in snapshot.objects):
   planner.fk(frame[3:]);tool=planner.fk_robot.get_transform('left_tool0',planner.fk_robot.base_link);item=snapshot.object(f'wall_box_{upper:02d}')
   from curobo_core.door_sequential import mesh_contact_proof
   if mesh_contact_proof(tool_mesh_vertices(planner,'left'),np.linalg.inv(__import__('curobo_core.adapter',fromlist=['pose_matrix']).pose_matrix(item.pose))@tool,item.dimensions_m,.001,tolerance=.0001)['valid']:contact=('left',item.object_id)
  frame_snapshots[i]=(snapshot,contact);key=(snapshot.geometry_key,contact);groups.setdefault(key,[snapshot,contact,[],[]]);groups[key][2].append(i);groups[key][3].append(frame[3:])
 failures=[];checked=0
 for snapshot,contact,indices,rows in groups.values():
  robot=loaded_robot(planner.robot,planner.box_fit,attachments=snapshot.attachments);native=snapshot
  if contact is not None:native=replace(snapshot,objects=tuple(x for x in snapshot.objects if x.object_id!=contact[1]))
  checker=RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(robot_config=robot,scene_model=to_curobo_scene(native),n_cuboids=max(40,len(snapshot.objects)),n_meshes=len(snapshot.meshes),collision_activation_distance=0.));validity=SequentialValidity(checker,snapshot,planner.robot,{s:tool_mesh_vertices(planner,s) for s in ('left','right')},contact,geometry_tolerance=.0001);mask=validity.mask(torch.tensor(np.asarray(rows),device='cuda',dtype=torch.float32)).cpu().numpy();checked+=len(mask);failures.extend(dict(frame=indices[j],phase=d['phases'][indices[j]]) for j in np.flatnonzero(~mask)[:20]);del validity,checker;torch.cuda.empty_cache()
 gpu_failures=list(failures);failures=[];mesh_repairs=[]
 from curobo_core.mesh_validation import MeshStateChecker
 for failure in gpu_failures:
  index=failure['frame'];snapshot,contact=frame_snapshots[index];mesh_snapshot=snapshot
  if contact is not None:mesh_snapshot=replace(snapshot,objects=tuple(x for x in snapshot.objects if x.object_id!=contact[1]))
  checker=MeshStateChecker(planner.robot['kinematics'],mesh_snapshot,{},mesh_snapshot.attachments);result=checker.check(dict(zip(ACTIVE_JOINTS,frames[index,3:])),attached=False,payload_enabled=True)
  repaired=result is None or result.get('contact_depth_m',float('inf'))<=.0001
  if repaired:mesh_repairs.append(dict(frame=index,phase=failure['phase'],mesh_result=result))
  else:failures.append(dict(failure,mesh_result=result))
 source_path=Path(d['symmetry_source']);source=json.loads(source_path.read_text())
 def mirrored(row):
  row=np.asarray(row,dtype=float);value=row.copy();value[1]*=-1;value[2]*=-1;q=row[3:];value[3]=q[0];value[4:11]=-q[8:15];value[11:18]=-q[1:8];return value
 symmetry_error=float(np.max(np.abs(frames-np.asarray([mirrored(row) for row in source['frames']]))))
 source_hash_ok=hashlib.sha256(source_path.read_bytes()).hexdigest()==d.get('symmetry_source_sha256')
 timing_equal=np.allclose(d['time_from_start_s'],source['time_from_start_s'],atol=1e-12,rtol=0)
 suffix_start=door_end;suffix_phase_equal=d['phases'][suffix_start:]==[value.replace('left','__swap__').replace('right','left').replace('__swap__','right') for value in source['phases'][suffix_start:]]
 torque=d.get('torque_validation',{});source_torque=source.get('torque_validation',{});symmetry_checks=dict(source_success=source.get('success') is True,source_hash=source_hash_ok,
  frame_transform=symmetry_error<=1e-12,timing_equal=bool(timing_equal),suffix_phase_transform=suffix_phase_equal,final_attachments_empty=not d.get('final_snapshot',{}).get('attachments'),
  torque_recomputed=d.get('symmetry_torque_recomputed') is True and torque.get('success') is True and torque.get('maximum_position_delta')==0.,source_torque=source_torque.get('success') is True)
 success=not failures and checked==door_end and all(symmetry_checks.values())
 out=dict(success=success,checked_frames=checked,door_end_frame=door_end,total_frames=len(frames),gpu_failures=len(gpu_failures),mesh_repaired=len(mesh_repairs),
  failures=failures,mesh_repairs=mesh_repairs,symmetry_checks=symmetry_checks,symmetry_max_position_error=symmetry_error,
  suffix_proof='exact frame/phase/time symmetry against the certified source plus recomputed mirrored RNEA torque');a.output.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n');print(json.dumps(out,ensure_ascii=False));raise SystemExit(0 if out['success'] else 1)
if __name__=='__main__':main()
