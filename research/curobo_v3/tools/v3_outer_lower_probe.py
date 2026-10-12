#!/usr/bin/env python3
"""Direct extraction probe for an outer lower box after its upper box vanished."""
import argparse, copy, fcntl, json, math, time
from dataclasses import asdict, replace
from pathlib import Path
import numpy as np, torch, yaml
from scipy.spatial.transform import Rotation
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.types import JointState
from curobo_core.adapter import matrix_pose, pose_matrix, to_curobo_scene
from curobo_core.backend import goal
from curobo_core.contracts import ACTIVE_JOINTS, PlannerAssets
from curobo_core.planner import FullCyclePlanner
from curobo_core.scene import AttachedObject
from curobo_core.sequential import mesh_contact_proof, tool_mesh_vertices
from v3_batched_loaded_search import loaded_robot
from v3_outer_inward_probe import approach_path, local_snapshot, reduced_robot, validity
from v3_plan_cycle import gpu_processes
from v3_rear_place import place_at_rear
from v3_wall_ik_benchmark import canonical_side_suction_quaternion_wxyz

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--column',type=int,choices=(0,4),required=True);p.add_argument('--seed',type=int,default=11);p.add_argument('--start-result',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();base_y=-.5 if a.column==0 else .5;side='right' if a.column==0 else 'left';upper=20+a.column;lower=15+a.column
 with open('/tmp/sevenova-curobo-gpu.lock','a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);occupied=gpu_processes();
  if occupied:raise RuntimeError('GPU occupied: '+'; '.join(occupied))
  started=time.perf_counter();timing={key:0. for key in ("contact_ik","extract","approach_contact","rear_place")};start_document=json.loads(a.start_result.read_text());selected=next(x for x in start_document["attempts"] if x.get("candidate")==start_document["selected_candidate"] and x.get("final_q") is not None);start_q=np.asarray(selected["final_q"],dtype=float)
  planner=FullCyclePlanner(PlannerAssets());snapshot=local_snapshot(planner,a.column,base_y);snapshot=replace(snapshot,objects=tuple(x for x in snapshot.objects if x.object_id!=f'wall_box_{upper:02d}'));item=snapshot.object(f'wall_box_{lower:02d}');planner.set_snapshot(snapshot)
  active=['updown']+[f'{side}_joint{i}' for i in range(1,8)];reference=dict(planner.home);idle='left' if side=='right' else 'right';parked=yaml.safe_load(planner.args.named_poses.read_text())['named_poses']['second_home'];reference.update({f'{idle}_joint{i}':parked[f'{idle}_joint{i}'] for i in range(1,8)})
  ik_started=time.perf_counter();robot=reduced_robot(planner,active,reference);kin=robot.get('robot_cfg',robot)['kinematics'];kin['tool_frames']=[side+'_tool0'];contact_snapshot=replace(snapshot,objects=tuple(x for x in snapshot.objects if x.object_id!=item.object_id));solver=InverseKinematics(InverseKinematicsCfg.create(robot=robot,scene_model=to_curobo_scene(contact_snapshot),collision_cache={'cuboid':max(40,len(snapshot.objects)),'mesh':0},num_seeds=512,self_collision_check=True,use_cuda_graph=False,position_tolerance=.002,orientation_tolerance=math.radians(1),override_iters_for_multi_link_ik=500,optimizer_collision_activation_distance=.005,random_seed=a.seed));timing['contact_ik']+=(time.perf_counter()-ik_started)*1000;state=JointState.from_position(torch.tensor([[reference[n] for n in active]],device='cuda',dtype=torch.float32),joint_names=active);attempts=[]
  try:
   for roll in (0.,math.pi/6,-math.pi/6,math.pi/4,-math.pi/4):
    target=np.eye(4);target[:3,3]=(item.pose.position[0]-item.dimensions_m[0]/2,item.pose.position[1],1.4289049959004567);canonical=Rotation.from_quat(np.roll(canonical_side_suction_quaternion_wxyz(side),-1)).as_matrix();target[:3,:3]=Rotation.from_rotvec((roll,0,0)).as_matrix()@canonical;proof=mesh_contact_proof(tool_mesh_vertices(planner,side),np.linalg.inv(pose_matrix(item.pose))@target,item.dimensions_m,.001,tolerance=.0001)
    if not proof['valid']:attempts.append({'roll':roll,'proof':proof,'success':False});continue
    pose=matrix_pose(target);solver.reset_seed();ik_solve_started=time.perf_counter();result=solver.solve_pose(goal({side+'_tool0':{'position':pose.position,'quaternion':pose.quaternion_wxyz}},frames=[side+'_tool0']),state,return_seeds=32);timing['contact_ik']+=(time.perf_counter()-ik_solve_started)*1000;names=list(result.js_solution.joint_names);values=result.js_solution.position.reshape(-1,len(names));success=result.success.reshape(-1).bool()
    for candidate_index,row in enumerate(values[success].cpu().numpy()):
     q=np.array([reference[n] for n in ACTIVE_JOINTS],dtype=float)
     for name in active:q[ACTIVE_JOINTS.index(name)]=row[names.index(name)]
     planner.fk(q);tool=planner.fk_robot.get_transform(side+'_tool0',planner.fk_robot.base_link).copy();attachment=AttachedObject(item.object_id,item.dimensions_m,side+'_tool0',matrix_pose(np.linalg.inv(tool)@pose_matrix(item.pose)),(side+'_link7',));attached=replace(contact_snapshot,attachments=(attachment,));planner.snapshot=attached
     try:check=validity(planner,attached)
     except ValueError:continue
     operation_started=time.perf_counter();path,details=planner.analytic_extract(q,check,check,sides=(side,),direction=(-1.,0.,0.),distance_m=.36)
     if path is None:path,details=planner.analytic_extract(q,check,check,sides=(side,),direction=(-1.,0.,0.),distance_m=.36,swivel_continuation=True)
     timing["extract"]+=(time.perf_counter()-operation_started)*1000
     attempt={'roll':roll,'native_candidates':int(success.sum()),'candidate':candidate_index,'details':details,'extract_success':path is not None}
     if path is not None:
      operation_started=time.perf_counter();planner.snapshot=snapshot;contact_check=validity(planner,snapshot,contact_pair=(side,item.object_id));outward,outward_details=planner.analytic_extract(q,contact_check,contact_check,sides=(side,),direction=(-1.,0.,0.),distance_m=.05);attempt.update(contact_outward=outward_details,contact_outward_success=outward is not None)
      if outward is not None:
       empty_check=validity(planner,snapshot);active_indices=[0]+list(range(1+("left","right").index(side)*7,8+("left","right").index(side)*7));approach,stats=approach_path(start_q,outward[-1],active_indices,empty_check,a.seed);attempt.update(approach_stats=stats,approach_success=approach is not None,precontact_q=outward[-1].tolist(),start_q=start_q.tolist(),final_q=path[-1].tolist())
       timing["approach_contact"]+=(time.perf_counter()-operation_started)*1000
       if approach is not None:
        rear = place_at_rear(planner, attached, path[-1], side, a.seed)
        timing["rear_place"]+=rear["measured_wall_ms"]
        attempt["attachment"] = asdict(attachment)
        attempt["rear_place"] = rear
        attempt["final_q"] = rear["final_q"]
        attempt["trajectory_frames"] = np.concatenate((approach,outward[::-1],path,
            np.asarray(rear["frames"]))).tolist()
        attempt["trajectory_phases"] = ([side+"_lower_approach"]*len(approach)
            +[side+"_lower_contact"]*len(outward)+[side+"_lower_extract"]*len(path)
            +list(rear["phases"]))
      else:attempt['approach_success']=False
     attempt['success']=bool(attempt.get('extract_success') and attempt.get('approach_success'));attempts.append(attempt)
     if attempt['success']:
      measured=(time.perf_counter()-started)*1000;timing["other"]=max(0.,measured-sum(timing.values()))
      output={'seed':a.seed,'column':a.column,'side':side,'base_y_m':base_y,'success':True,'measured_wall_ms':measured,'stage_timing_ms':timing,'selected':attempt,'attempts':attempts};a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output,indent=2));return
  finally:
   solver.destroy()
  measured=(time.perf_counter()-started)*1000;timing["other"]=max(0.,measured-sum(timing.values()))
  output={'seed':a.seed,'column':a.column,'side':side,'base_y_m':base_y,'success':False,'measured_wall_ms':measured,'stage_timing_ms':timing,'attempts':attempts};a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output,indent=2));raise SystemExit(1)
if __name__=='__main__':main()
