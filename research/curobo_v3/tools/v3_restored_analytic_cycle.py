import copy
from contextlib import ExitStack
from dataclasses import replace
import json
from pathlib import Path
import time
from unittest.mock import patch

import numpy as np
import torch
from scipy.spatial.transform import Rotation
from curobo.types import JointState
from curobo_core.adapter import matrix_pose
from curobo_core.backend import CuroboBackend
from curobo_core.scene import digest

from v3_current_urdf_forward import CurrentUrdfForwardPlanner,current_assets
from v3_single_arm_planner import SingleArmPlanner
from v3_conveyor_cycle_planner import ConveyorCyclePlanner
from v3_batched_loaded_search import GpuValidity,densify
from v3_payload_safety import base_world_audit
from v3_strict_cartesian_extraction import plan_analytic_extraction
from v3_contact_roll_policy import aligned_grasp_transform
from v3_wall_ik_benchmark import ACTIVE_JOINTS
from v3_research_assets import atlas_path,file_hash
from v3_multiroot_search import batched_rrt_connect_multi_goal as original_rrt


ROOT=Path(__file__).resolve().parents[1]
REVISION='restored-frozen40-multicandidate-analytic-20261031'


class RestoredAnalyticPlanner(CurrentUrdfForwardPlanner):
    def __init__(self,args=None):
        super().__init__(args or current_assets())
        self.placement_clearance_m=.982-self.conveyor_height_m-self.box_fit['dimensions_m'][2]/2
        self.rotation_quality_hard_gate=False
        self.restore_audits=[]

    def _full_cycle(self,boxes,progress=lambda text:None):
        return SingleArmPlanner._full_cycle(self,boxes,progress)

    def configure_task(self,boxes):
        result=super().configure_task(boxes)
        if not hasattr(self,'_original_current_atlases'):self._original_current_atlases={}
        mode='top' if self.top_grasp else 'side'
        self._original_current_atlases.setdefault(mode,self.transport_atlas_paths[mode])
        self.transport_atlas_paths[mode]=self._original_current_atlases[mode]
        return result

    def validate_current_atlas(self,planner=None):
        path=atlas_path(self)
        identity=(str(path),tuple(sorted(self.selected_task.items())))
        if getattr(self,'_restored_validated_atlas',None)==identity:return
        checker=self.checker(self.selected_task)
        with np.load(path,allow_pickle=False) as stored:
            rows=stored['joints']
            if rows.ndim!=2 or rows.shape[1]!=15 or not len(rows) or not np.isfinite(rows).all():
                raise ValueError('restored profile received malformed current-model atlas')
            heights=stored['updown_grid_m']
            if max(np.min(abs(rows[:,0,None]-heights[None]),axis=1))>1e-5:
                raise ValueError('current-model atlas does not match its declared sampled heights')
        maximum=0.
        for offset in range(0,len(rows),1024):
            state=checker.kinematics.compute_kinematics(JointState.from_position(torch.tensor(rows[offset:offset+1024][None],device='cuda'),joint_names=ACTIVE_JOINTS))
            for side in self.selected_task:
                pose=state.tool_poses.to_dict()[side+'_tool0']
                target=self.target_poses[side+'_tool0']
                error=torch.linalg.vector_norm(pose.position.reshape(-1,3)-torch.tensor(target['position'],device='cuda'),dim=-1)
                cosine=(pose.quaternion.reshape(-1,4)*torch.tensor(target['quaternion'],device='cuda')).sum(-1).abs()
                if not bool((error<=.002).all() and (cosine>=np.cos(np.deg2rad(.5))).all()):
                    raise ValueError('current-model atlas FK differs from restored transport target')
                maximum=max(maximum,float(error.max()))
        self._restored_validated_atlas=identity
        self.atlas_validation=dict(path=str(path),sha256=file_hash(path),rows=len(rows),maximum_position_error_m=maximum,
            scope='fresh current-model FK and declared5mm-height validation; collision/envelope validated at runtime, not old2cm-grid cache')
        loaded=self.checker(self.selected_task,payload=True)
        radii=[]
        for offset in range(0,len(rows),512):
            radii.extend(self.full_envelope_radius(loaded,rows[offset:offset+512]).cpu().tolist())
        radii=np.asarray(radii)
        retained=radii<=1.05
        if not retained.any():raise ValueError('current model has no transport atlas points inside1.05m envelope')
        mode='top' if self.top_grasp else 'side'
        folder=ROOT/'generated/restored_analytic_cycle/compatible_atlas'
        folder.mkdir(parents=True,exist_ok=True)
        signature=digest((file_hash(path),tuple(sorted(self.selected_task))))[:16]
        compatible=folder/f'{mode}_{signature}.npz'
        if not compatible.exists():np.savez_compressed(compatible,joints=rows[retained],radius=radii[retained])
        self.transport_atlas_paths[mode]=compatible
        self._restored_validated_atlas=(str(compatible),tuple(sorted(self.selected_task.items())))

    def audit_path(self,checker,path,target=False):
        if isinstance(path,torch.Tensor):path=path.detach().cpu().numpy()
        rows=np.asarray(densify(np.asarray(path,dtype=np.float32),maximum_weighted_step=np.deg2rad(.25)),dtype=np.float32)
        validity=self.target_validity(checker,allow_contact=target=='contact') if target else GpuValidity(checker,check_ground=True)
        validity.scene_penetration_tolerance_m=.001
        validity.contact_penetration_tolerance_m=.001
        audit=dict(valid=True,base_world=base_world_audit(checker),invalid_frames=0,exact_payload=None,
            collision_model='frozen40_payload',exact_payload_hard_gate=False,base_margin_m=0.)
        for offset in range(0,len(rows),512):
            mask=validity.evaluate(torch.tensor(rows[offset:offset+512],device='cuda'),base_margin_m=0.)[0]
            audit['invalid_frames']+=int((~mask).sum())
        audit['valid']=audit['base_world']['valid'] and audit['invalid_frames']==0
        return audit

    def verified_rrt(self,*args,**kwargs):
        validity=args[4]
        validity.validate_returned_path=getattr(self,'dense_path_rechecks',True)
        validity.scene_penetration_tolerance_m=.001
        validity.contact_penetration_tolerance_m=.001
        if not getattr(self,'dense_path_rechecks',True):
            path,stats=original_rrt(*args,**kwargs)
            return path,dict(stats,quarter_degree_acceptance_checks=0,quarter_degree_rejections=0,
                             extra_path_recheck=False)
        previous=getattr(validity,'path_quality_callback',None)
        checks=[]
        def accept(path):
            if previous is not None and not previous(path):return False
            audit=self.audit_path(validity.checker,path,
                target='contact' if getattr(validity,'allow_contact',False) else bool(hasattr(validity,'targets')))
            checks.append(audit['valid'])
            return audit['valid']
        validity.path_quality_callback=accept
        try:
            path,stats=original_rrt(*args,**kwargs)
            return path,dict(stats,quarter_degree_acceptance_checks=len(checks),quarter_degree_rejections=checks.count(False))
        finally:
            validity.path_quality_callback=previous

    def search_path(self,start,goals,lower,upper,validity,budget,seed):
        validity.validate_returned_path=getattr(self,'dense_path_rechecks',True)
        validity.scene_penetration_tolerance_m=.001
        validity.contact_penetration_tolerance_m=.001
        path,stats=CuroboBackend.search_path(self,start,goals,lower,upper,validity,budget,seed)
        if path is not None and getattr(self,'dense_path_rechecks',True):
            audit=self.audit_path(validity.checker,path,target='contact' if getattr(validity,'allow_contact',False) else bool(hasattr(validity,'targets')))
            stats=dict(stats,restored_collision_audit=audit)
            if not audit['valid']:return None,dict(stats,failure='restored path exact collision acceptance failed')
        return path,stats

    def run_single_search(self,*args,**kwargs):
        validity=args[4]
        validity.scene_penetration_tolerance_m=.001
        validity.contact_penetration_tolerance_m=.001
        path,stats=self.verified_rrt(*args,**kwargs)
        if path is not None and getattr(self,'dense_path_rechecks',True):
            audit=self.audit_path(validity.checker,path,target=bool(hasattr(validity,'targets')))
            stats=dict(stats,restored_collision_audit=audit)
            if not audit['valid']:return None,dict(stats,failure='restored single-arm exact collision acceptance failed')
        return path,stats

    def plan_conveyor_target(self,*args,**kwargs):
        return SingleArmPlanner.plan_conveyor_target(self,*args,**kwargs)

    def _guided_search(self,*args,**kwargs):
        raise RuntimeError('restored analytic profile must not call WARP or LM-guided RRT')

    def analytic_extract(self,contact,empty_validity,loaded_validity):
        key=tuple(np.round(np.asarray(contact,dtype=np.float64),7))
        if getattr(self,'_lazy_preparing',False):
            index=self._contact_indices[key]
            if index not in self._allowed_contacts:return None,'deferred to next historical8/16/32/64 batch'
            if key in self._extraction_cache:return self._extraction_cache[key]
        began=time.perf_counter()
        result=plan_analytic_extraction(self,contact,getattr(self,'extraction_direction',(-1.,0.,0.)),
            exact_payload_hard_gate=False,base_margin_m=0.)
        value=(np.asarray(result['frames']),dict(result,frames=None,tracks=None)) if result['success'] else (None,dict(result,frames=None,tracks=None))
        if getattr(self,'_lazy_preparing',False):
            self._extraction_cache[key]=value
            self.lazy_timings['extraction_work_ms']+=(time.perf_counter()-began)*1000
        return value

    def cartesian_single(self,contact,distances,along_normal=False):
        if along_normal:return SingleArmPlanner.cartesian_single(self,contact,distances,along_normal)
        result=plan_analytic_extraction(self,contact,exact_payload_hard_gate=False,base_margin_m=0.)
        return (np.asarray(result['frames']),dict(result,frames=None,tracks=None)) if result['success'] else (None,dict(result,frames=None,tracks=None))

    def transport_and_turn(self,boxes,transport,stats):
        result=ConveyorCyclePlanner.transport_and_turn(self,boxes,transport,stats)
        checker=self.checker(boxes,payload=True)
        radius=float(self.full_envelope_radius(checker,np.asarray(transport[-1:])).max())
        if not getattr(self,'dense_path_rechecks',True):
            result['transport_path_recheck']='disabled; search collision checks retained'
            if radius>1.05:
                result['success']=False
                result['failure']=dict(stage='transport endpoint envelope',frame_index=0,yaw_deg=0.)
            return result
        audit=self.audit_path(checker,transport)
        audit['transport_endpoint_radius_m']=radius
        audit['valid'] &= radius<=1.05
        result['restored_collision_audit']=audit
        if not audit['valid']:
            result['success']=False
            result['failure']=dict(stage='transport exact payload acceptance',frame_index=0,yaw_deg=0.)
        return result

    def _final_audit(self,result,pick_snapshot):
        if not getattr(self,'dense_path_rechecks',True):return []
        frames=np.asarray(result.get('frames',[]),dtype=np.float32)
        if not len(frames):return []
        names=result['joint_names']
        ids=[names.index(name) for name in ACTIVE_JOINTS]
        phases=np.asarray(result['phases'])
        audits=[]
        for phase in ('approach','contact_approach','extract','transport','conveyor_place','return_home'):
            indices=np.flatnonzero(phases==phase)
            if not len(indices):continue
            self.snapshot=pick_snapshot
            if phase in ('conveyor_place','return_home'):
                frame=frames[indices[0]]
                base=np.eye(4)
                base[:3,:3]=Rotation.from_euler('z',float(frame[names.index('base_yaw')])).as_matrix()
                base[:2,3]=[frame[names.index('base_x')]-self.base_advance,frame[names.index('base_y')]]
                self.snapshot=replace(pick_snapshot,state=pick_snapshot.state.with_positions(dict(zip(names,frame)),matrix_pose(base)))
            payload=phase in ('extract','transport','conveyor_place')
            checker=self.checker(result['task'],payload=payload)
            audit=self.audit_path(checker,frames[indices][:,ids],target='contact' if phase=='contact_approach' else phase=='approach')
            audit.update(phase=phase,first_frame=int(indices[0]),last_frame=int(indices[-1]))
            audits.append(audit)
            if not audit['valid']:
                stop=int(indices[0])
                for key in ('frames','phases','payload'):result[key]=result[key][:stop]
                result.update(success=False,blocked_stage=phase,blocked_details='restored whole-stage exact collision/target acceptance failed')
                break
        self.snapshot=pick_snapshot
        return audits

    def full_cycle(self,boxes,progress=lambda text:None):
        began=time.perf_counter()
        self.configure_task(boxes)
        pick_snapshot=self.snapshot
        self.restore_audits=[]
        with ExitStack() as stack:
            for module in ('v3_task_cycle_planner','v3_conveyor_cycle_planner','v3_box_rotation_cost'):
                stack.enter_context(patch(module+'.grasp_transform',aligned_grasp_transform))
            for module in ('v3_multiroot_full_cycle_planner','curobo_core.backend'):
                stack.enter_context(patch(module+'.batched_rrt_connect_multi_goal',self.verified_rrt))
            stack.enter_context(patch('v3_mixed_placement_planner.validate_atlas',self.validate_current_atlas))
            result=SingleArmPlanner.full_cycle(self,boxes,progress)
        result['restored_phase_audits']=self._final_audit(result,pick_snapshot)
        names=list(self.mobile_robot['kinematics']['cspace']['joint_names'])
        if not result.get('frames'):
            values=dict(self.home)
            values.update(base_x=self.base_advance,base_y=0.,base_yaw=0.)
            result.update(frames=[[float(values.get(name,0)) for name in names]],phases=['home'],payload=[False],joint_names=names)
        result.update(task=dict(boxes),visual_scene_snapshot=pick_snapshot.to_dict(),visual_home=dict(self.home),visual_base_advance=self.base_advance,
            visual_tool_to_box={side:np.asarray(matrix).tolist() for side,matrix in self.checker(boxes,payload=True).task_tool_to_box.items()},
            acceptance_revision=REVISION,full_payload_extraction=True,strict_cartesian_extraction=True,real_robot_execution=False,
            collision_acceptance_policy='frozen40 payload all loaded frames, base cuboids, native robot/self/bounds/ground, targets on approach, world1mm; no added internal2mm padding; no solid payload audit',
            exact_payload_hard_gate=False,
            algorithm='restored',endpoint_selection_policy='historical64 contacts, analytic predicted ranking,8/16/32/64 verified extraction roots and multi-root/multi-goal RRT; no branch quota, graph-DP, LM prediction or WARP guidance',
            return_policy='legacy release then Home and simulated navigation return',wall_ms=(time.perf_counter()-began)*1000)
        result['total_ms']=result['wall_ms']
        if any('out of memory' in str(value).lower() for value in (result.get('blocked_details',''),result.get('error',''))):
            raise torch.OutOfMemoryError('restored profile GPU resource failure; not counted as geometric task failure')
        return result
