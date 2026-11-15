import time

import numpy as np
import torch
from scipy.spatial.transform import Rotation
from curobo.types import JointState

from v3_batched_loaded_search import GpuValidity, densify
from v3_payload_safety import exact_payload_audit
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def clock():
    torch.cuda.synchronize()
    return time.perf_counter()


def line_targets(planner,checker,start,direction,distance=.35,count=128):
    direction=np.asarray(direction,dtype=float)
    if direction.shape!=(3,) or not np.isfinite(direction).all() or abs(np.linalg.norm(direction)-1)>1e-6:
        raise ValueError('extraction direction must be a finite unit vector in footprint coordinates')
    poses=planner._tool_poses(checker,torch.tensor(start,device='cuda',dtype=torch.float32))
    output={}
    for side in planner.selected_task:
        matrices=np.repeat(np.asarray(poses[side])[None],count,axis=0)
        matrices[:,:3,3]+=np.linspace(0,distance,count)[:,None]*direction
        output[side]=matrices
    return output


def audit_line(planner,checker,path,start,targets,direction,distance=.35,*,exact_payload_hard_gate=True,base_margin_m=.002):
    began=clock()
    rows=np.asarray(path,dtype=np.float32).reshape(-1,15)
    dense=np.asarray(densify(rows,maximum_weighted_step=np.deg2rad(.25)),dtype=np.float32)
    validity=GpuValidity(checker,check_ground=True,scene_penetration_tolerance_m=.001)
    bad=[]
    metrics=dict(max_line_error_m=0.,max_orientation_error_deg=0.,max_outside_m=0.,max_backward_m=0.,
                 max_sync_error_m=0.,endpoint_error_m=0.)
    tracks={side:dict(positions=[],quaternions_wxyz=[]) for side in targets}
    for offset in range(0,len(dense),512):
        tensor=torch.tensor(dense[offset:offset+512],device='cuda')
        mask=validity.evaluate(tensor,base_margin_m=base_margin_m)[0].cpu().numpy()
        bad.extend((offset+np.flatnonzero(~mask)).tolist())
        state=checker.kinematics.compute_kinematics(JointState.from_position(tensor[None],joint_names=ACTIVE_JOINTS))
        for side in tracks:
            pose=state.tool_poses.to_dict()[side+'_tool0']
            tracks[side]['positions'].extend(pose.position.reshape(-1,3).cpu().tolist())
            tracks[side]['quaternions_wxyz'].extend(pose.quaternion.reshape(-1,4).cpu().tolist())
    progress=[]
    for side,track in tracks.items():
        positions=np.asarray(track['positions'])
        delta=positions-targets[side][0,:3,3]
        along=delta@direction
        progress.append(along)
        metrics['max_line_error_m']=max(metrics['max_line_error_m'],float(np.linalg.norm(delta-along[:,None]*direction,axis=-1).max()))
        rotations=Rotation.from_quat(np.roll(track['quaternions_wxyz'],-1,axis=1))
        angles=(Rotation.from_matrix(targets[side][0,:3,:3]).inv()*rotations).magnitude()
        metrics['max_orientation_error_deg']=max(metrics['max_orientation_error_deg'],float(np.rad2deg(angles.max())))
        metrics['max_outside_m']=max(metrics['max_outside_m'],float(np.maximum(-along,along-distance).max()))
        metrics['max_backward_m']=max(metrics['max_backward_m'],float(np.maximum(-np.diff(along),0).max(initial=0)))
        metrics['endpoint_error_m']=max(metrics['endpoint_error_m'],float(np.linalg.norm(positions[-1]-targets[side][-1,:3,3])))
        track['total_rotation_deg']=float(np.rad2deg((rotations[:-1].inv()*rotations[1:]).magnitude()).sum())
    metrics['max_sync_error_m']=float(np.ptp(np.asarray(progress),axis=0).max())
    exact=exact_payload_audit(checker,dense,checker.task_tool_to_box,planner.box_fit['dimensions_m'],planner.robot) if exact_payload_hard_gate else None
    step=float(np.rad2deg(abs(np.diff(rows[:,1:],axis=0))).max(initial=0))
    failures=[]
    if bad:failures.append('loaded_collision')
    if exact_payload_hard_gate and not exact['valid']:failures.append('exact_payload_collision')
    if not np.allclose(dense[0],start,atol=1e-5,rtol=0):failures.append('start_joint_mismatch')
    if step>15.:failures.append('adjacent_IK_jump_gt15deg')
    if any(metrics[key]>.002 for key in ('max_line_error_m','max_outside_m','max_backward_m','max_sync_error_m','endpoint_error_m')):
        failures.append('cartesian_line_error_gt2mm')
    if metrics['max_orientation_error_deg']>1.:failures.append('orientation_error_gt1deg')
    return dict(success=not failures,failures=failures,metrics=metrics,invalid_frames=bad[:30],exact_payload=exact,
                frames=dense.tolist(),tracks=tracks,max_keyframe_joint_step_deg=step,audit_ms=(clock()-began)*1000,
                exact_payload_hard_gate=exact_payload_hard_gate,base_margin_m=base_margin_m)


def analytic_line(planner,start,targets):
    began=clock()
    start=np.asarray(start,dtype=np.float64)
    planner.fk(start)
    carriage=np.linalg.inv(planner.fk_robot.get_transform('arm_carriage',planner.fk_robot.base_link))
    swivels={side:planner.analytic.swivel(0 if side=='left' else 1,start[1:8] if side=='left' else start[8:]) for side in targets}
    rows=[start.copy()]
    for index in range(1,len(next(iter(targets.values())))):
        row=rows[-1].copy()
        for side in targets:
            arm=0 if side=='left' else 1
            offset=1+arm*7
            values=planner.analytic.solve(arm,carriage@targets[side][index],swivels[side],row[offset:offset+7])
            if not len(values):
                return None,dict(failure='fixed Updown/psi analytic IK unavailable',failed_point=index,solve_ms=(clock()-began)*1000)
            row[offset:offset+7]=values[np.linalg.norm(values-row[offset:offset+7],axis=-1).argmin()]
        rows.append(row)
    return np.asarray(rows,dtype=np.float32),dict(solve_ms=(clock()-began)*1000,fixed_updown_m=float(start[0]),fixed_swivels=swivels)


def plan_analytic_extraction(planner,start,direction=(-1.,0.,0.),distance=.35,count=128,*,exact_payload_hard_gate=True,base_margin_m=.002):
    began=clock()
    checker=planner.checker(planner.selected_task,payload=True)
    targets=line_targets(planner,checker,start,direction,distance,count)
    rows,stats=analytic_line(planner,start,targets)
    if rows is None:
        return dict(success=False,failures=[stats['failure']],solver=stats,total_ms=(clock()-began)*1000)
    audit=audit_line(planner,checker,rows,start,targets,np.asarray(direction),distance,
        exact_payload_hard_gate=exact_payload_hard_gate,base_margin_m=base_margin_m)
    return dict(audit,solver=stats,total_ms=(clock()-began)*1000,method='fixed_updown_psi_analytic',full_payload_extraction=True)
