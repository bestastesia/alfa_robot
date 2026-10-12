#!/usr/bin/env python3
"""Bounded outer-upper inward/lower/extract probe after middle columns are empty."""
import argparse
import copy
import fcntl
import json
import math
import time
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import torch
import yaml
from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.types import JointState

from curobo_core.adapter import matrix_pose, pose_matrix, to_curobo_scene
from curobo_core.backend import goal
from curobo_core.contracts import ACTIVE_JOINTS, PlannerAssets
from curobo_core.planner import FullCyclePlanner
from curobo_core.scene import AttachedObject, Pose
from curobo_core.sequential import mesh_contact_proof, sequential_request, tool_mesh_vertices
from curobo_core.sequential_validity import SequentialValidity
from v3_batched_loaded_search import loaded_robot, batched_rrt_connect_multi_goal, densify
from curobo_core.sequential_validity import ReducedValidity
from v3_plan_cycle import gpu_processes
from v3_rear_place import place_at_rear
from v3_search_helpers import audit_state
from v3_wall_ik_benchmark import canonical_side_suction_quaternion_wxyz


def reduced_robot(planner, active_names, reference):
    robot=copy.deepcopy(planner.robot);kin=robot.get("robot_cfg",robot)["kinematics"];names=list(kin["cspace"]["joint_names"])
    kin["lock_joints"].update({name:float(reference[name]) for name in names if name not in active_names})
    indices=[names.index(name) for name in active_names]
    kin["cspace"]={key:[value[i] for i in indices] if isinstance(value,list) and len(value)==len(names) else copy.deepcopy(value) for key,value in kin["cspace"].items()}
    return robot


def local_snapshot(planner,column,base_y):
    request=sequential_request(planner);cleared={16,17,18,21,22,23};objects=[]
    for item in request.snapshot.objects:
        if item.object_id in ("left_wall","right_wall") or item.object_id in {f"wall_box_{i:02d}" for i in cleared}:continue
        if item.object_id=="ground":objects.append(item);continue
        position=list(item.pose.position);position[1]-=base_y;objects.append(replace(item,pose=Pose(tuple(position),item.pose.quaternion_wxyz)))
    return replace(request.snapshot,objects=tuple(objects))


def validity(planner,snapshot,contact_pair=None):
    robot=loaded_robot(planner.robot,planner.box_fit,attachments=snapshot.attachments)
    native=snapshot if contact_pair is None else replace(snapshot,objects=tuple(
        item for item in snapshot.objects if item.object_id!=contact_pair[1]))
    checker=RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(robot_config=robot,
        scene_model=to_curobo_scene(native),n_cuboids=max(40,len(snapshot.objects)),n_meshes=0,collision_activation_distance=0.))
    return SequentialValidity(checker,snapshot,planner.robot,{side:tool_mesh_vertices(planner,side) for side in ("left","right")},
                              contact_pair=contact_pair,geometry_tolerance=.0001)

def approach_path(start,goal,active,check,seed=11):
    reduced=ReducedValidity(check,start,active);lower,upper=check.checker.kinematics.get_joint_limits().position
    path,stats=batched_rrt_connect_multi_goal(torch.tensor(start[active],device="cuda",dtype=torch.float32),
        torch.tensor([goal[active]],device="cuda",dtype=torch.float32),lower[active]+1e-5,upper[active]-1e-5,reduced,2.,seed)
    if path is None:return None,stats
    expanded=reduced.expand(torch.tensor(path,device="cuda",dtype=torch.float32)).cpu().numpy();dense=np.asarray(densify(expanded))
    return (dense if bool(check.mask(torch.tensor(dense,device="cuda",dtype=torch.float32)).all()) else None),stats


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--column",type=int,choices=(0,4),required=True);parser.add_argument("--seed",type=int,default=11);parser.add_argument("--stage",choices=("lateral","lower","extract"),default="lateral");parser.add_argument("--lateral-distance-m",type=float,default=.4);parser.add_argument("--plan-approach",action="store_true");parser.add_argument("--output",type=Path,required=True);args=parser.parse_args()
    base_y=-.5 if args.column==0 else .5;side="right" if args.column==0 else "left";direction=np.array((0.,1.,0.)) if args.column==0 else np.array((0.,-1.,0.));upper=20+args.column
    with open("/tmp/sevenova-curobo-gpu.lock","a") as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);occupied=gpu_processes()
        if occupied:raise RuntimeError("GPU occupied: "+"; ".join(occupied))
        started=time.perf_counter();timing={key:0. for key in ("contact_ik","lateral","lower","extract","approach_contact","rear_place")};planner=FullCyclePlanner(PlannerAssets());snapshot=local_snapshot(planner,args.column,base_y);item=snapshot.object(f"wall_box_{upper:02d}");planner.set_snapshot(snapshot)
        target=np.eye(4);target[:3,3]=(item.pose.position[0]-item.dimensions_m[0]/2,item.pose.position[1],1.7673550912141804);from scipy.spatial.transform import Rotation;target[:3,:3]=Rotation.from_quat(np.roll(canonical_side_suction_quaternion_wxyz(side),-1)).as_matrix()
        proof=mesh_contact_proof(tool_mesh_vertices(planner,side),np.linalg.inv(pose_matrix(item.pose))@target,item.dimensions_m,.001,tolerance=.0001)
        contact_snapshot=replace(snapshot,objects=tuple(x for x in snapshot.objects if x.object_id!=item.object_id))
        active=["updown"]+[f"{side}_joint{i}" for i in range(1,8)]
        reference=dict(planner.home);idle="left" if side=="right" else "right"
        parked=yaml.safe_load(planner.args.named_poses.read_text())["named_poses"]["second_home"]
        reference.update({f"{idle}_joint{i}":parked[f"{idle}_joint{i}"] for i in range(1,8)})
        robot=reduced_robot(planner,active,reference);kin=robot.get("robot_cfg",robot)["kinematics"];kin["tool_frames"]=[side+"_tool0"]
        ik_started=time.perf_counter();solver=InverseKinematics(InverseKinematicsCfg.create(robot=robot,scene_model=to_curobo_scene(contact_snapshot),collision_cache={"cuboid":max(40,len(snapshot.objects)),"mesh":0},num_seeds=512,self_collision_check=True,use_cuda_graph=False,position_tolerance=.002,orientation_tolerance=math.radians(1),override_iters_for_multi_link_ik=500,optimizer_collision_activation_distance=.005,random_seed=args.seed))
        state=JointState.from_position(torch.tensor([[reference[n] for n in active]],device="cuda",dtype=torch.float32),joint_names=active)
        try:
            solver.reset_seed();pose=matrix_pose(target);result=solver.solve_pose(goal({side+"_tool0":{"position":pose.position,"quaternion":pose.quaternion_wxyz}},frames=[side+"_tool0"]),state,return_seeds=32)
            names=list(result.js_solution.joint_names);values=result.js_solution.position.reshape(-1,len(names));success=result.success.reshape(-1).bool();candidates=[]
            for row in values[success].cpu().numpy():
                q=np.array([reference[n] for n in ACTIVE_JOINTS],dtype=float)
                for name in active:
                    q[ACTIVE_JOINTS.index(name)] = row[names.index(name)]
                candidates.append(q)
        finally:solver.destroy()
        timing["contact_ik"]+=(time.perf_counter()-ik_started)*1000
        report={"seed":args.seed,"column":args.column,"side":side,"base_y_m":base_y,"stage_requested":args.stage,"contact_proof":proof,"native_contact_candidates":len(candidates),"attempts":[]}
        for index,q in enumerate(candidates):
            planner.fk(q);tool=planner.fk_robot.get_transform(side+"_tool0",planner.fk_robot.base_link).copy();attachment=AttachedObject(item.object_id,item.dimensions_m,side+"_tool0",matrix_pose(np.linalg.inv(tool)@pose_matrix(item.pose)),(side+"_link7",));attached=replace(contact_snapshot,attachments=(attachment,));planner.snapshot=attached
            try:
                check=validity(planner,attached)
            except ValueError as error:
                report["attempts"].append({"candidate":index,"contact_certificate":str(error),"lateral_success":False})
                continue
            operation_started=time.perf_counter();lateral,details=planner.analytic_extract(q,check,check,sides=(side,),direction=tuple(direction),distance_m=args.lateral_distance_m)
            if lateral is None:lateral,details=planner.analytic_extract(q,check,check,sides=(side,),direction=tuple(direction),distance_m=args.lateral_distance_m,swivel_continuation=True)
            timing["lateral"]+=(time.perf_counter()-operation_started)*1000
            attempt={"candidate":index,"contact_q":q.tolist(),"lateral_distance_m":args.lateral_distance_m,"lateral":details,"lateral_success":lateral is not None}
            if lateral is not None and args.stage in ("lower","extract"):
                operation_started=time.perf_counter();lowered,lower_details=planner.analytic_extract(lateral[-1],check,check,sides=(side,),direction=(0.,0.,-1.),distance_m=.4,lift_end=float(lateral[-1][0]-.4))
                timing["lower"]+=(time.perf_counter()-operation_started)*1000
                failure = copy.deepcopy(planner.last_cartesian_failure) if lowered is None else None
                attempt.update(lower=lower_details,lower_success=lowered is not None,lower_failure=failure)
                if failure is not None and "joint_positions" in failure:
                    failure_state = JointState.from_position(torch.tensor(
                        [failure["joint_positions"]], device="cuda", dtype=torch.float32),
                        joint_names=list(ACTIVE_JOINTS))
                    attempt["lower_native_audit"] = audit_state(
                        check.checker.kinematics, failure_state, to_curobo_scene(attached))
                if lowered is not None and args.stage=="extract":
                    operation_started=time.perf_counter();extracted,extract_details=planner.analytic_extract(lowered[-1],check,check,sides=(side,),direction=(-1.,0.,0.),distance_m=.36)
                    if extracted is None:extracted,extract_details=planner.analytic_extract(lowered[-1],check,check,sides=(side,),direction=(-1.,0.,0.),distance_m=.36,swivel_continuation=True)
                    timing["extract"]+=(time.perf_counter()-operation_started)*1000
                    attempt.update(extract=extract_details,extract_success=extracted is not None,
                                   final_q=extracted[-1].tolist() if extracted is not None else None)
            required=attempt.get(args.stage+"_success", False)
            if required and args.plan_approach:
                operation_started=time.perf_counter();planner.snapshot=snapshot
                contact_check=validity(planner,snapshot,contact_pair=(side,item.object_id))
                outward_path,outward_details=planner.analytic_extract(q,contact_check,contact_check,
                    sides=(side,),direction=(-1.,0.,0.),distance_m=.05)
                attempt.update(contact_outward=outward_details,contact_outward_success=outward_path is not None)
                if outward_path is not None:
                    empty_check=validity(planner,snapshot)
                    active_indices=[0]+list(range(1+("left","right").index(side)*7,8+("left","right").index(side)*7))
                    approach,stats=approach_path(np.array([reference[n] for n in ACTIVE_JOINTS],dtype=float),
                                                outward_path[-1],active_indices,empty_check,args.seed)
                    attempt.update(approach_stats=stats,approach_success=approach is not None,
                                   precontact_q=outward_path[-1].tolist(),
                                   approach_start_q=np.array([reference[n] for n in ACTIVE_JOINTS],dtype=float).tolist())
                    timing["approach_contact"]+=(time.perf_counter()-operation_started)*1000
                    if approach is not None and args.stage=="extract" and extracted is not None:
                        rear = place_at_rear(planner, attached, extracted[-1], side, args.seed)
                        timing["rear_place"]+=rear["measured_wall_ms"]
                        attempt["attachment"] = asdict(attachment)
                        attempt["rear_place"] = rear
                        attempt["final_q"] = rear["final_q"]
                        attempt["trajectory_frames"] = np.concatenate((approach, outward_path[::-1],
                            lateral, lowered, extracted, np.asarray(rear["frames"]))).tolist()
                        attempt["trajectory_phases"] = ([side+"_approach"]*len(approach)
                            +[side+"_contact"]*len(outward_path)+[side+"_lateral"]*len(lateral)
                            +[side+"_lower"]*len(lowered)+[side+"_extract"]*len(extracted)
                            +list(rear["phases"]))
                    required=approach is not None
                else:
                    timing["approach_contact"]+=(time.perf_counter()-operation_started)*1000
                    required=False
            report["attempts"].append(attempt)
            if required:report["selected_candidate"]=index;report["success"]=True;break
        report.setdefault("success",False);report["measured_wall_ms"]=(time.perf_counter()-started)*1000
        timing["other"]=max(0.,report["measured_wall_ms"]-sum(timing.values()));report["stage_timing_ms"]=timing
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps({"column":args.column,"stage":args.stage,"contact_candidates":len(candidates),"success":report["success"],"selected":report.get("selected_candidate"),"last":report["attempts"][-1] if report["attempts"] else None},indent=2))

if __name__=="__main__":main()
