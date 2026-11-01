import copy
from dataclasses import replace
import time

import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import torch
from curobo.inverse_kinematics import InverseKinematics,InverseKinematicsCfg
from curobo._src.solver.seed_ik.seed_ik_solver import SeedIKSolver
from curobo.types import GoalToolPose,JointState,Pose
from curobo_core.adapter import matrix_pose
from curobo_core.scene import SceneSnapshot
from curobo_core.distance_metric import joint_distance_weights
from v3_batched_loaded_search import GpuValidity
from v3_payload_safety import base_world_audit
from v3_wall_ik_benchmark import ACTIVE_JOINTS

WEIGHTS=np.asarray(joint_distance_weights(ACTIVE_JOINTS))


def clock():
    torch.cuda.synchronize()
    return time.perf_counter()


def interpolate_boxes(starts,ends,fractions):
    fractions=np.asarray(fractions,dtype=float)
    result={}
    for side,first in starts.items():
        last=ends[side]
        values=np.repeat(np.eye(4)[None],len(fractions),axis=0)
        values[:,:3,3]=first[:3,3]+fractions[:,None]*(last[:3,3]-first[:3,3])
        values[:,:3,:3]=Slerp([0.,1.],Rotation.from_matrix(np.stack((first[:3,:3],last[:3,:3]))))(fractions).as_matrix()
        result[side]=values
    return result


def prepare_case(planner,record):
    planner.configure_task(record['task'])
    names=record['joint_names']
    indices=[names.index(name) for name in ACTIVE_JOINTS]
    selected=np.flatnonzero(np.asarray(record['phases'])=='conveyor_place')
    if not len(selected):raise ValueError('reference has no placement phase')
    full=np.asarray(record['frames'][int(selected[0])],dtype=np.float32)
    start=full[indices].copy()
    world_base=np.eye(4)
    world_base[:3,:3]=Rotation.from_euler('z',float(full[names.index('base_yaw')])).as_matrix()
    world_base[:2,3]=[full[names.index('base_x')],full[names.index('base_y')]]
    local_base=world_base.copy()
    local_base[0,3]-=record['visual_base_advance']
    snapshot=SceneSnapshot.from_dict(record['visual_scene_snapshot'])
    planner.snapshot=replace(snapshot,state=snapshot.state.with_positions(dict(zip(names,full)),matrix_pose(local_base)))
    checker=planner.checker(record['task'],payload=True)
    if not checker.guidance_payload:raise RuntimeError('placement requires loaded collision checker')
    attachments={side:np.asarray(matrix) for side,matrix in record['visual_tool_to_box'].items()}
    for side in attachments:
        if not np.allclose(attachments[side],checker.task_tool_to_box[side],atol=1e-7):
            raise ValueError('frozen task attachment does not match current model')
    tools=planner._tool_poses(checker,torch.tensor(start,device='cuda'))
    starts={side:tools[side]@attachment for side,attachment in attachments.items()}
    ends={}
    for side,target in record['conveyor_box_targets'].items():
        matrix=np.eye(4)
        matrix[:3,:3]=Rotation.from_quat(np.roll(target['quaternion_wxyz'],-1)).as_matrix()
        matrix[:3,3]=target['position']
        ends[side]=np.linalg.inv(world_base)@matrix
    return dict(start=start,checker=checker,attachments=attachments,starts=starts,ends=ends,
        world_base=world_base,full_start=full,source_rows=np.asarray(record['frames'],dtype=np.float32)[selected][:,indices],
        scene_snapshot=planner.snapshot.to_dict())


def box_poses(checker,rows,attachments):
    tensor=torch.as_tensor(np.asarray(rows),device='cuda',dtype=torch.float32).contiguous()
    state=checker.kinematics.compute_kinematics(JointState.from_position(tensor[None],joint_names=ACTIVE_JOINTS))
    result={}
    for side,attachment in attachments.items():
        pose=state.tool_poses.to_dict()[side+'_tool0']
        matrix=np.repeat(np.eye(4)[None],len(rows),axis=0)
        matrix[:,:3,:3]=Rotation.from_quat(np.roll(pose.quaternion.reshape(-1,4).cpu().numpy(),-1,axis=1)).as_matrix()
        matrix[:,:3,3]=pose.position.reshape(-1,3).cpu().numpy()
        result[side]=matrix@attachment
    return result


def dense_segment(first,last,first_progress,last_progress):
    count=max(1,int(np.ceil(np.max(np.abs((last-first)*WEIGHTS))/np.deg2rad(.25))))
    return np.linspace(first,last,count+1).astype(np.float32),np.linspace(first_progress,last_progress,count+1)


def reference_errors(actual,expected):
    positions=[]
    angles=[]
    for side in expected:
        positions.append(np.linalg.norm(actual[side][:,:3,3]-expected[side][:,:3,3],axis=-1))
        angles.append(np.rad2deg((Rotation.from_matrix(expected[side][:,:3,:3]).inv()*Rotation.from_matrix(actual[side][:,:3,:3])).magnitude()))
    return np.max(positions,axis=0),np.max(angles,axis=0)


def score_path(case,rows,success):
    tracks={side:dict(positions=[],quaternions_wxyz=[]) for side in case['attachments']}
    for offset in range(0,len(rows),512):
        actual=box_poses(case['checker'],rows[offset:offset+512],case['attachments'])
        for side,values in actual.items():
            tracks[side]['positions'].extend(values[:,:3,3].tolist())
            tracks[side]['quaternions_wxyz'].extend(np.roll(Rotation.from_matrix(values[:,:3,:3]).as_quat(),1,axis=1).tolist())
    rotation={}
    extra={}
    for side,track in tracks.items():
        orientations=Rotation.from_quat(np.roll(track['quaternions_wxyz'],-1,axis=1))
        total=float(np.rad2deg((orientations[:-1].inv()*orientations[1:]).magnitude()).sum())
        needed=float(np.rad2deg((orientations[0].inv()*orientations[-1]).magnitude()))
        rotation[side]=total
        extra[side]=max(0.,total-needed)
    metrics=dict(rotation_deg=rotation,extra_rotation_deg=extra,
        total_rotation_deg=sum(rotation.values()),total_extra_rotation_deg=sum(extra.values()),
        weighted_length=float(np.linalg.norm(np.diff(rows,axis=0)*WEIGHTS,axis=-1).sum()),
        endpoint_joints=np.asarray(rows[-1]).tolist(),quality_is_complete=bool(success))
    return metrics,tracks


class SequentialPlacementLm:
    def __init__(self,planner,seeds=32,iterations=32,samples=128,active_sides=('left','right')):
        began=clock()
        self.active_sides=tuple(active_sides)
        if not self.active_sides or not set(self.active_sides)<=set(('left','right')):
            raise ValueError('placement requires at least one active arm')
        self.idle_indices=[index for side,offset in (('left',1),('right',8)) if side not in self.active_sides for index in range(offset,offset+7)]
        robot=copy.deepcopy(planner.robot)
        robot['kinematics']['tool_frames']=[side+'_tool0' for side in self.active_sides]
        config=InverseKinematicsCfg.create(robot=robot,scene_model=None,
            self_collision_check=False,num_seeds=seeds,seed_solver_num_seeds=seeds,max_batch_size=1,
            position_tolerance=.0005,orientation_tolerance=np.deg2rad(.25),use_cuda_graph=True,random_seed=20261031)
        self.owner=InverseKinematics(config)
        original=self.owner.seed_ik_solver
        lm_config=replace(original.config,num_seeds=seeds,max_iterations=iterations,inner_iterations=8,
            position_tolerance=.0005,orientation_tolerance=np.deg2rad(.25))
        original.destroy()
        self.solver=SeedIKSolver(lm_config)
        self.owner.seed_ik_solver=self.solver
        if set(self.solver.joint_names)!=set(ACTIVE_JOINTS) or self.solver.dof!=15:
            raise RuntimeError('LM must solve all15 axes')
        self.to_solver=[ACTIVE_JOINTS.index(name) for name in self.solver.joint_names]
        self.to_active=[self.solver.joint_names.index(name) for name in ACTIVE_JOINTS]
        self.seeds=seeds
        self.iterations=iterations
        self.samples=samples
        self.setup_ms=(clock()-began)*1000

    def plan(self,case):
        began=clock()
        checker=case['checker']
        if not checker.guidance_payload:raise RuntimeError('empty checker cannot validate placement')
        if set(case['attachments'])!=set(self.active_sides):raise ValueError('LM active arms do not match attached boxes')
        validity=GpuValidity(checker,check_ground=True,scene_penetration_tolerance_m=.001)
        start=case['start'].copy()
        rows=[start]
        progress=[0.]
        reports=[]
        timing=dict(lm_ms=0.,candidate_filter_ms=0.,edge_audit_ms=0.)
        failure=None
        failed_rows=[]
        if not base_world_audit(checker)['valid'] or not bool(validity.mask(torch.tensor(start[None],device='cuda')).all()):
            failure='starting state collision'
        lower,upper=[value.cpu().numpy() for value in checker.kinematics.get_joint_limits().position]
        generator=np.random.default_rng(20261031)
        previous=start.copy()
        previous_previous=start.copy()
        maximum_position_error=0.
        maximum_angle_error=0.
        for index,fraction in enumerate(np.linspace(0.,1.,self.samples)[1:],1):
            if failure:break
            boxes=interpolate_boxes(case['starts'],case['ends'],[fraction])
            tools={side:values@np.linalg.inv(case['attachments'][side]) for side,values in boxes.items()}
            target=GoalToolPose.from_poses({side+'_tool0':Pose(
                position=torch.tensor(matrix[:,:3,3],device='cuda',dtype=torch.float32),
                quaternion=torch.tensor(np.roll(Rotation.from_matrix(matrix[:,:3,:3]).as_quat(),1,axis=1),device='cuda',dtype=torch.float32))
                for side,matrix in tools.items()},ordered_tool_frames=[side+'_tool0' for side in self.active_sides])
            noise=generator.normal(size=(self.seeds,15))*np.deg2rad(3.)
            noise[:,0]=generator.normal(size=self.seeds)*.005
            noise[:,self.idle_indices]=0.
            seeds=previous+noise
            seeds[0]=previous
            seeds[1]=previous+(previous-previous_previous)
            seeds=np.clip(seeds,lower+1e-6,upper-1e-6).astype(np.float32)
            stamp=clock()
            current=JointState.from_position(torch.tensor(previous[self.to_solver][None],device='cuda'),joint_names=self.solver.joint_names)
            with torch.enable_grad():
                solved=self.solver.solve_single(target,current_state=current,
                    seed_config=torch.tensor(seeds[:,self.to_solver][None],device='cuda').contiguous(),return_seeds=self.seeds)
            values=solved.js_solution.position.reshape(-1,15)[:,self.to_active].detach().cpu().numpy()
            solve_ok=solved.success.reshape(-1).cpu().numpy().astype(bool)
            timing['lm_ms']+=(clock()-stamp)*1000
            stamp=clock()
            actual=box_poses(checker,values,case['attachments'])
            expected={side:np.repeat(matrix,len(values),axis=0) for side,matrix in boxes.items()}
            position_error,angle_error=reference_errors(actual,expected)
            pose_ok=solve_ok&(position_error<=.002)&(angle_error<=1.)&np.isfinite(values).all(-1)
            collision_ok=validity.mask(torch.tensor(values,device='cuda')).cpu().numpy()
            continuous=np.max(np.abs((values-previous)*WEIGHTS),axis=-1)<=np.deg2rad(15)
            if self.idle_indices:
                continuous&=np.max(np.abs(values[:,self.idle_indices]-start[self.idle_indices]),axis=-1)<=1e-5
            legal=np.flatnonzero(pose_ok&collision_ok&continuous)
            order=legal[np.argsort(np.linalg.norm((values[legal]-previous)*WEIGHTS,axis=-1))]
            report=dict(point=index,progress=float(fraction),lm_valid=int(pose_ok.sum()),collision_valid=int((pose_ok&collision_ok).sum()),continuous_valid=len(legal),edge_rejections=0)
            timing['candidate_filter_ms']+=(clock()-stamp)*1000
            selected=None
            stamp=clock()
            for candidate in order:
                segment,fractions=dense_segment(previous,values[candidate],progress[-1],fraction)
                mask=validity.mask(torch.tensor(segment,device='cuda')).cpu().numpy()
                expected=interpolate_boxes(case['starts'],case['ends'],fractions)
                actual=box_poses(checker,segment,case['attachments'])
                position_error,angle_error=reference_errors(actual,expected)
                if mask.all():
                    selected=values[candidate]
                    rows.extend(segment[1:])
                    progress.extend(fractions[1:])
                    maximum_position_error=max(maximum_position_error,float(np.max(position_error)))
                    maximum_angle_error=max(maximum_angle_error,float(np.max(angle_error)))
                    break
                report['edge_rejections']+=1
            timing['edge_audit_ms']+=(clock()-stamp)*1000
            reports.append(report)
            if selected is None:
                failure='LM has no accurate solution' if not pose_ok.any() else ('all accurate solutions collide' if not (pose_ok&collision_ok).any() else ('joint discontinuity' if not len(legal) else 'interpolated edge collision'))
                failed_rows=values[pose_ok][:8].tolist()
                break
            previous_previous=previous
            previous=selected
        success=bool(failure is None and abs(progress[-1]-1.)<1e-9)
        return dict(success=success,frames=np.asarray(rows).tolist(),progress=progress,failure=failure,
            failed_candidates=failed_rows,point_reports=reports,completed_progress=float(progress[-1]),
            max_reference_position_error_m=maximum_position_error,max_reference_orientation_error_deg=maximum_angle_error,
            edge_pose_policy='diagnostic_only',node_position_tolerance_m=.002,node_orientation_tolerance_deg=1.,
            total_ms=(clock()-began)*1000,**timing,solver='cuRobo SeedIKSolver Levenberg-Marquardt, sequential continuation',
            optimized_joint_names=list(ACTIVE_JOINTS),seeds_per_point=self.seeds,iterations=self.iterations,
            reference_samples=self.samples,endpoint_joint_lock=False,rrt_fallback=False,main_optimizer_ran=False,
            payload_model='frozen40',real_robot_execution=False)
