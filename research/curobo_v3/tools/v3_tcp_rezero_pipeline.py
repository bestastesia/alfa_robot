import copy
import json
from dataclasses import dataclass, replace
import math
from types import SimpleNamespace
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch
from curobo.types import JointState
from curobo.inverse_kinematics import InverseKinematics,InverseKinematicsCfg

from curobo_core.adapter import matrix_pose, pose_matrix
from curobo_core.contracts import ACTIVE_JOINTS
from curobo_core.planner import CycleBlocked
from curobo_core.scene import AttachedObject, Pose, RobotState, SceneObject
from v3_rezero_task_planner import RezeroTaskPlanner
from v3_contact_roll_policy import TOOL_ROLL, aligned_tool_pose, aligned_grasp_transform
from contextlib import ExitStack
from unittest.mock import patch
from v3_all_extraction_roots_test import main as prepare_roots
from v3_batched_loaded_search import GpuValidity, densify, loaded_robot
from v3_tcp_scene_contract import GraspInput, reconstruct_scene, tool_to_box as legacy_tool_to_box


def tool_to_box(mode, dimensions=(.3, .4, .4)):
    return TOOL_ROLL @ legacy_tool_to_box(mode, dimensions)
from v3_multiroot_full_cycle_planner import MultiRootFullCyclePlanner
from v3_single_arm_planner import SingleArmPlanner


@dataclass(frozen=True)
class TcpMotionRequest:
    grasps: tuple
    robot_state: RobotState
    placement_boxes: tuple

    def __post_init__(self):
        sides=[grasp.side for grasp in self.grasps]
        if not sides or len(set(sides))!=len(sides) or not set(sides)<= {'left','right'}:
            raise ValueError('One contact TCP per active arm is required')
        self.robot_state.ordered(ACTIVE_JOINTS)
        if set(side for side,pose in self.placement_boxes)!=set(sides):
            raise ValueError('A final box pose is required for each active arm')
        for grasp in self.grasps:
            transform=np.asarray(grasp.tcp_in_footprint)
            if transform.shape!=(4,4) or not np.isfinite(transform).all() or not np.allclose(transform[3],[0,0,0,1]):
                raise ValueError('Invalid contact TCP transform')
            if not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-6) or np.linalg.det(transform[:3,:3])<0:
                raise ValueError('Contact TCP rotation must be proper orthonormal')
            tool_to_box(grasp.mode)
        if not all(isinstance(pose,Pose) for side,pose in self.placement_boxes):
            raise ValueError('Final box poses must use the validated Pose contract')


class OrientedGraspValidity(GpuValidity):
    def __init__(self,checker,boxes):
        super().__init__(checker,check_ground=True)
        self.boxes=boxes

    def evaluate(self,values):
        valid,stability=super().evaluate(values)
        for offset in range(0,len(values),1024):
            batch=values[offset:offset+1024].contiguous()
            state=self.checker.kinematics.compute_kinematics(JointState.from_position(batch[None],joint_names=list(ACTIVE_JOINTS)))
            spheres=state.robot_spheres.reshape(len(batch),-1,4)
            for matrix,dimensions in self.boxes:
                center=torch.tensor(matrix[:3,3],device=values.device,dtype=values.dtype)
                rotation=torch.tensor(matrix[:3,:3],device=values.device,dtype=values.dtype)
                half=torch.tensor(dimensions,device=values.device,dtype=values.dtype)/2
                delta=((spheres[...,:3]-center)@rotation).abs()-half
                distance=torch.linalg.vector_norm(delta.clamp(min=0),dim=-1)+delta.max(-1).values.clamp(max=0)
                valid[offset:offset+len(batch)] &= ~((spheres[...,3]>0)&(distance<spheres[...,3])).any(-1)
        return valid,stability


class TcpMotionPlanner(RezeroTaskPlanner):
    request=None

    def bind_request(self,request):
        if len({grasp.mode for grasp in request.grasps})!=1:
            raise NotImplementedError('Mixed top/side carry atlas requires a separate adaptation')
        self.request=request
        self.motion_stage='bound'
        self.selected_task={grasp.side:index for index,grasp in enumerate(request.grasps)}
        self.top_grasp=request.grasps[0].mode=='top'
        self.base_advance=0.
        self.home_values=np.asarray(request.robot_state.ordered(ACTIVE_JOINTS),dtype=np.float32)
        self.home.update(dict(zip(ACTIVE_JOINTS,self.home_values)))
        world_from_footprint=pose_matrix(request.robot_state.base_pose)
        generated=reconstruct_scene(tuple(GraspInput(grasp.side,grasp.mode,grasp.tcp_in_footprint@TOOL_ROLL) for grasp in request.grasps),world_from_footprint,self.box_fit['dimensions_m'])
        objects=[item for item in self.original_snapshot.objects if not item.object_id.startswith(('wall_box_','task_fill_'))]
        for grasp in request.grasps:
            object_id=f'wall_box_{self.selected_task[grasp.side]:02d}'
            objects.append(SceneObject(object_id,tuple(self.box_fit['dimensions_m']),matrix_pose(generated['boxes'][grasp.side])))
        for fill in generated['fills']:
            objects.append(SceneObject('task_fill_'+fill['name'],tuple(fill['dimensions']),Pose(tuple(fill['center']))))
        self.snapshot=replace(self.original_snapshot,objects=tuple(objects),state=request.robot_state,revision=self.original_snapshot.revision+1)
        policy=json.loads((Path(__file__).resolve().parents[1]/'config/placement_task_policy.json').read_text())['top' if self.top_grasp else 'side']
        self.target_poses={grasp.side+'_tool0':dict(position=[policy['position']['x'],policy['position']['y_'+grasp.side],1.2 if self.top_grasp else policy['position']['z']],quaternion=policy['quaternion_wxyz']) for grasp in request.grasps}
        self.target_poses={frame:aligned_tool_pose(pose) for frame,pose in self.target_poses.items()}
        self.placement_atlas_override=self.contact_roll_atlases['top' if self.top_grasp else 'side']
        self.contact_inputs={grasp.side:np.asarray(grasp.tcp_in_footprint) for grasp in request.grasps}
        self.extraction_direction=np.asarray(generated['outward_footprint'])
        self.prediction_records=[]
        self.analytic_task=None
        self._cached_task=None
        self._cached_solver=None
        self.request_scene=generated
        return generated

    def configure_task(self,boxes):
        if self.request is None:
            return super().configure_task(boxes)
        if boxes!=self.selected_task:
            raise ValueError('Task handles must match bound TCP inputs')
        return self.request_scene['boxes'],self.request_scene['fills']

    def contact_poses_for_task(self,boxes):
        if self.request is None:
            return super().contact_poses_for_task(boxes)
        return {side+'_tool0':dict(position=matrix[:3,3].tolist(),quaternion=np.roll(Rotation.from_matrix(matrix[:3,:3]).as_quat(),1).tolist()) for side,matrix in self.contact_inputs.items()}

    def task_attachments(self,boxes):
        if self.request is None:
            return super().task_attachments(boxes)
        return tuple(AttachedObject(f'wall_box_{boxes[grasp.side]:02d}',tuple(self.box_fit['dimensions_m']),grasp.side+'_tool0',matrix_pose(tool_to_box(grasp.mode,self.box_fit['dimensions_m'])),(grasp.side+'_link7',grasp.side+'_tool0')) for grasp in self.request.grasps)

    def target_validity(self,checker,allow_contact=False):
        if self.request is None:
            return super().target_validity(checker,allow_contact)
        inverse=np.linalg.inv(pose_matrix(self.snapshot.state.base_pose))
        targets=[(inverse@pose_matrix(self.snapshot.object(f'wall_box_{index:02d}').pose),self.box_fit['dimensions_m']) for index in self.selected_task.values()]
        return OrientedGraspValidity(checker,targets)

    def predict_endpoint(self,contact):
        if self.request is None:
            return super().predict_endpoint(contact)
        self.fk(contact)
        inverse=np.linalg.inv(self.fk_robot.get_transform('arm_carriage',self.fk_robot.base_link))
        result=contact.copy()
        for arm,side in enumerate(('left','right')):
            target=self.fk_robot.get_transform(side+'_tool0',self.fk_robot.base_link).copy()
            target[:3,3]+=.35*self.extraction_direction
            offset=1+7*arm
            seed=contact[offset:offset+7]
            solutions=self.analytic.solve(arm,inverse@target,self.analytic.swivel(arm,seed),seed)
            if not len(solutions):return None
            result[offset:offset+7]=solutions[np.linalg.norm(solutions-seed,axis=1).argmin()]
        return result

    def plan_external_placement(self,feedback,box_targets,held_result,progress=lambda message:None):
        if not held_result.get('success') or not held_result.get('awaiting_navigation'):
            raise ValueError('A successful held transport result is required')
        if set(box_targets)!=set(self.selected_task):
            raise ValueError('Final box targets must cover both active arms')
        positions=np.asarray(feedback.ordered(ACTIVE_JOINTS),dtype=np.float32)
        prior=np.asarray(held_result['frames'][-1])[[held_result['joint_names'].index(name) for name in ACTIVE_JOINTS]]
        if np.max(np.abs(positions-prior))>1e-3:
            raise ValueError('Navigation feedback arm state differs from held transport; replan required')
        began=time.perf_counter()
        self.snapshot=replace(self.snapshot,state=feedback,revision=self.snapshot.revision+1)
        report=copy.deepcopy(held_result)
        report['awaiting_navigation']=False
        names=report['joint_names']
        base=pose_matrix(feedback.base_pose)
        endpoint=np.asarray(report['frames'][-1]).copy()
        endpoint[names.index('base_x')]=feedback.base_pose.position[0]
        endpoint[names.index('base_y')]=feedback.base_pose.position[1]
        endpoint[names.index('base_yaw')]=Rotation.from_matrix(base[:3,:3]).as_euler('xyz')[2]
        report['frames'].append(endpoint.tolist());report['phases'].append('base_reposition');report['payload'].append(True)
        def append(rows,phase,payload):
            for row in rows:
                value=endpoint.copy()
                if len(row)==len(names):value=np.asarray(row)
                else:value[[names.index(name) for name in ACTIVE_JOINTS]]=row
                report['frames'].append(value.tolist());report['phases'].append(phase);report['payload'].append(payload)
        checker=self.checker(self.selected_task,payload=True)
        validity=GpuValidity(checker,check_ground=True)
        configured=loaded_robot(self.robot,self.box_fit,attachments=self.task_attachments(self.selected_task))
        solver=InverseKinematics(InverseKinematicsCfg.create(robot=configured,scene_model=self.scene(self.selected_task),
            collision_cache={'cuboid':40,'mesh':0},num_seeds=64,self_collision_check=True,use_cuda_graph=False,
            position_tolerance=.002,orientation_tolerance=math.radians(1),override_iters_for_multi_link_ik=160,
            optimizer_collision_activation_distance=.005))
        targets={side:pose_matrix(pose)@np.linalg.inv(tool_to_box(next(grasp.mode for grasp in self.request.grasps if grasp.side==side),self.box_fit['dimensions_m'])) for side,pose in box_targets.items()}
        report['conveyor_box_targets']={side:dict(position=matrix_pose(base@pose_matrix(pose)).position,
            quaternion_wxyz=matrix_pose(base@pose_matrix(pose)).quaternion_wxyz) for side,pose in box_targets.items()}
        try:
            self.plan_conveyor_target(positions,targets,checker,validity,solver,report,append)
            report['pending_stage']='await_external_release_confirmation'
            self.motion_stage='await_release'
            report['success']=True
        except CycleBlocked as failure:
            report.update(success=False,blocked_stage=failure.stage,blocked_details=failure.details)
        report['placement_planning_ms']=(time.perf_counter()-began)*1000
        report['final_home_error']=None
        return report

    def confirm_release_and_plan_home(self,released_result,release_confirmed):
        if release_confirmed is not True or self.motion_stage!='await_release':
            raise ValueError('External release confirmation is required before detaching and planning Home')
        report=copy.deepcopy(released_result)
        began=time.perf_counter()
        if not report['success']:raise ValueError('Cannot release after failed placement')
        names=report['joint_names'];last=np.asarray(report['frames'][-1])
        report['frames'].append(last.tolist());report['phases'].append('release');report['payload'].append(False)
        objects=tuple(item for item in self.snapshot.objects if not item.object_id.startswith('wall_box_'))
        self.snapshot=replace(self.snapshot,objects=objects,revision=self.snapshot.revision+1)
        checker=self.checker(self.selected_task)
        validity=GpuValidity(checker,check_ground=True)
        start=last[[names.index(name) for name in ACTIVE_JOINTS]]
        path,stats=self.joint_plan(start,self.home_values,checker,validity,20261014)
        report['conveyor_home_search_stats']=stats
        if path is None:
            report.update(success=False,blocked_stage='release_to_home',blocked_details='Home search exhausted original 2s budget')
            return report
        for row in path:
            value=last.copy();value[[names.index(name) for name in ACTIVE_JOINTS]]=row
            report['frames'].append(value.tolist());report['phases'].append('return_home');report['payload'].append(False)
        report.update(success=True,pending_stage='complete',final_home_error=float(np.max(np.abs(path[-1]-self.home_values))))
        report['release_home_planning_ms']=(time.perf_counter()-began)*1000
        report['total_ms']+=report.get('placement_planning_ms',0.)+report['release_home_planning_ms']
        report['plan_scope']='Simulated TCP task: contact, world extraction, carry, navigation arrival, external placement, confirmed release, Home'
        self.motion_stage='complete'
        return report

    def analytic_extract(self,contact,empty_validity,loaded_validity):
        if self.request is None:
            return super().analytic_extract(contact,empty_validity,loaded_validity)
        if getattr(self,'_lazy_preparing',False):
            key=tuple(np.round(np.asarray(contact,dtype=np.float64),7))
            if self._contact_indices[key] not in self._allowed_contacts:
                return None,'Deferred to a later allowed contact batch'
            if key not in self._extraction_cache:
                began=time.perf_counter()
                self._extraction_cache[key]=self._analytic_extract_world(contact,empty_validity,loaded_validity)
                self.lazy_timings['extraction_work_ms']+=(time.perf_counter()-began)*1000
            return self._extraction_cache[key]
        return self._analytic_extract_world(contact,empty_validity,loaded_validity)

    def _analytic_extract_world(self,contact,empty_validity,loaded_validity):
        self.fk(contact)
        inverse=np.linalg.inv(self.fk_robot.get_transform('arm_carriage',self.fk_robot.base_link))
        targets=self.tool_matrices(contact)
        swivels=[self.analytic.swivel(arm,contact[1+7*arm:8+7*arm]) for arm in range(2)]
        rows=[np.asarray(contact)]
        for distance in np.linspace(.01,.35,35):
            value=rows[-1].copy()
            for arm,side in enumerate(('left','right')):
                target=targets[side].copy();target[:3,3]+=distance*self.extraction_direction
                offset=1+7*arm;seed=rows[-1][offset:offset+7]
                solutions=self.analytic.solve(arm,inverse@target,swivels[arm],seed)
                if not len(solutions):
                    self.last_extract_frames=np.asarray(rows)
                    return None,f'{side} world-outward fixed-psi IK unavailable at {distance:.3f}m'
                value[offset:offset+7]=solutions[np.linalg.norm(solutions-seed,axis=1).argmin()]
            rows.append(value)
        dense=np.asarray(densify(np.asarray(rows)))
        self.last_extract_frames=dense
        validity=empty_validity if self.snapshot.policy.defer_payload_until_extract_end else loaded_validity
        if not bool(validity.mask(torch.tensor(dense,device='cuda',dtype=torch.float32).contiguous()).all()):
            return None,'world-outward extraction collision'
        if not bool(loaded_validity.mask(torch.tensor(dense[-1:],device='cuda',dtype=torch.float32)).all()):
            return None,'world-outward extraction loaded endpoint collision'
        max_line=0.;max_angle=0.
        for value in dense:
            for side,actual in self.tool_matrices(value).items():
                delta=actual[:3,3]-targets[side][:3,3]
                progress=np.dot(delta,self.extraction_direction)
                max_line=max(max_line,float(np.linalg.norm(delta-progress*self.extraction_direction)))
                max_angle=max(max_angle,float(Rotation.from_matrix(targets[side][:3,:3].T@actual[:3,:3]).magnitude()))
        if max_line>.002 or max_angle>math.radians(1):return None,'world extraction line/orientation verification failed'
        return dense,dict(fixed_updown_m=float(contact[0]),fixed_psi_rad=swivels,max_line_error_mm=max_line*1000,max_orientation_error_deg=math.degrees(max_angle),outward_footprint=self.extraction_direction.tolist(),outward_world=[-1.,0.,0.])

    def prepare_roots(self,boxes):
        if self.request is None:return super().prepare_roots(boxes)
        self._lazy_preparing=True
        try:return prepare_roots(self,boxes,True)
        finally:self._lazy_preparing=False

    def cartesian_single(self,contact,distances,along_normal=False):
        if self.request is None:return super().cartesian_single(contact,distances,along_normal)
        self.fk(contact)
        carriage=np.linalg.inv(self.fk_robot.get_transform('arm_carriage',self.fk_robot.base_link))
        initial=self.fk_robot.get_transform(self.active_side+'_tool0',self.fk_robot.base_link).copy()
        arm=0 if self.active_side=='left' else 1
        offset=1+7*arm
        swivel=self.analytic.swivel(arm,contact[offset:offset+7])
        direction=-initial[:3,2] if along_normal else self.extraction_direction
        rows=[np.asarray(contact,dtype=float)]
        for distance in distances:
            target=initial.copy();target[:3,3]+=direction*distance
            solutions=self.analytic.solve(arm,carriage@target,swivel,rows[-1][offset:offset+7])
            if not len(solutions):return None,'Single-arm fixed-updown/psi Cartesian IK unavailable'
            value=rows[-1].copy();value[offset:offset+7]=solutions[np.linalg.norm(solutions-rows[-1][offset:offset+7],axis=1).argmin()]
            rows.append(value)
        dense=np.asarray(densify(np.asarray(rows)))
        for row in dense:
            self.fk(row)
            actual=self.fk_robot.get_transform(self.active_side+'_tool0',self.fk_robot.base_link)
            delta=actual[:3,3]-initial[:3,3]
            if np.linalg.norm(delta-direction*np.dot(delta,direction))>.002 or Rotation.from_matrix(initial[:3,:3].T@actual[:3,:3]).magnitude()>math.radians(1):
                return None,'Single-arm world Cartesian line/orientation verification failed'
        return dense,dict(fixed_updown_m=float(contact[0]),fixed_swivel_rad=swivel,outward_world=[-1.,0.,0.])

    def conveyor_suffix(self,boxes,report,append,progress):
        if self.request is None:return super().conveyor_suffix(boxes,report,append,progress)
        report['awaiting_navigation']=True
        report['pending_stage']='navigation_then_external_placement'
        progress('TCP输入抓取/世界方向抽离/收束完成，等待导航与新footprint放箱目标')

    def plan_to_transport(self,progress=lambda message:None):
        with ExitStack() as stack:
            for module in ('v3_research_assets','v3_all_extraction_roots_test','v3_side_upward_prepare','v3_lazy_extraction_planner','v3_single_arm_planner'):
                stack.enter_context(patch(module+'.atlas_path',self._atlas))
            for module in ('v3_task_cycle_planner','v3_conveyor_cycle_planner','v3_box_rotation_cost'):
                stack.enter_context(patch(module+'.grasp_transform',aligned_grasp_transform))
            return self._plan_to_transport(progress)

    def _plan_to_transport(self,progress=lambda message:None):
        if self.request is None:raise ValueError('Bind a TCP request first')
        began=time.perf_counter()
        self.cached_selection=None
        self.last_partial=dict(success=False,task=self.selected_task,frames=[],phases=[],payload=[],joint_names=list(self.mobile_robot['kinematics']['cspace']['joint_names']))
        self.lazy_timings=dict(contact_generation_ms=0.,endpoint_prediction_ms=0.,prediction_ranking_ms=0.,extraction_work_ms=0.)
        self._extraction_cache={};self._lazy_preparing=False
        try:
            if len(self.selected_task)==1:
                result=SingleArmPlanner._full_cycle(self,self.selected_task,progress)
                return self._finish_pick_result(result,began)
            ranked=self.rank_contacts(self.selected_task)
            contacts=self.analytic_result.js_solution.position.detach().cpu().numpy().reshape(-1,15)
            self._contact_indices={tuple(np.round(contact.astype(np.float64),7)):index for index,contact in enumerate(contacts)}
            search_spent=0.
            for limit,allocation in [(8,350.),(16,450.),(32,600.),(64,600.)]:
                self._allowed_contacts=set(ranked[:limit])
                self.transport_search_budget_ms=min(allocation,max(0.,2000.-search_spent))
                try:
                    result=MultiRootFullCyclePlanner._full_cycle(self,self.selected_task,progress)
                except CycleBlocked as failure:
                    result=dict(failure.partial or self.last_partial,success=False,blocked_stage=failure.stage,blocked_details=failure.details)
                search_spent+=getattr(self,'last_multiroot_search_ms',0.)
                if result['success'] or search_spent>=2000.:break
        except (CycleBlocked,RuntimeError) as failure:
            result=dict(self.last_partial,success=False,blocked_stage=getattr(failure,'stage','contact_generation'),blocked_details=str(failure))
        return self._finish_pick_result(result,began)

    def _finish_pick_result(self,result,began):
        result.update(total_ms=(time.perf_counter()-began)*1000,top_grasp=self.top_grasp,base_advance_m=0.,
            generated_obstacles=self.request_scene['fills'],tool_to_box={grasp.side:tool_to_box(grasp.mode,self.box_fit['dimensions_m']).tolist() for grasp in self.request.grasps},
            plan_scope='Home to held transport only; navigation/place/release/Home suffix pending',final_home_error=None)
        if result.get('frames'):
            rows=np.asarray(result['frames']);names=result['joint_names'];pose=self.request.robot_state.base_pose
            for name,value in [('base_x',pose.position[0]),('base_y',pose.position[1]),('base_yaw',Rotation.from_quat(np.roll(pose.quaternion_wxyz,-1)).as_euler('xyz')[2])]:
                rows[:,names.index(name)]=value
            result['frames']=rows.tolist()
        return result
