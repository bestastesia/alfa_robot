import copy
import json
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from curobo.types import JointState
from curobo_core.backend import goal as goal_message
from curobo_core.planner import CycleBlocked,FullCyclePlanner
from v3_batched_loaded_search import GpuValidity,densify
from v3_box_rotation_cost import BoxRotationCost
from v3_conveyor_cycle_planner import ConveyorCyclePlanner
from v3_endpoint_guided_ik import endpoint_references,solve_numeric,pose_filter
from v3_idle_arm_candidates import solve_idle_candidates
from v3_research_assets import ROOT,atlas_path
from v3_task_cycle_planner import TaskCyclePlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS
from v3_wrist_guided_cycle_planner import WristGuidedCyclePlanner


WEIGHTS=np.array([np.deg2rad(15)/.1]+[1.]*14)


def nearest_endpoint(rows,previous):
    rows=np.asarray(rows).reshape(-1,15)
    if not len(rows):raise ValueError('没有合法终点候选')
    distances=np.linalg.norm((rows-np.asarray(previous))*WEIGHTS,axis=1)
    index=int(np.argmin(distances))
    return rows[index].copy(),index,float(distances[index])


def complete_playback_fields(result):
    if result.get('single_arm') and len(result.get('task',{}))==1:
        result['active_side']=next(iter(result['task']))
    envelope=result.get('transport_endpoint_envelope')
    if envelope is not None:
        envelope['passed']=bool(envelope['radius_m']<=envelope['limit_m'])
    return result


class SerialEndpointPlanner(WristGuidedCyclePlanner):
    search_base_margin_m=0.002

    def __init__(self,args,reference_source=None):
        super().__init__(args,reference_source=reference_source)
        self.endpoint_ik_cache={}
        self.serial_records=[]
        self.weighted_reference=None
        self.extraction_teleport=False

    def record_endpoint(self,name,rows,previous,source):
        selected,index,distance=nearest_endpoint(rows,previous)
        self.serial_records.append(dict(name=name,joints=selected.tolist(),previous_joints=np.asarray(previous).tolist(),
            candidate_count=len(rows),selected_index=index,weighted_distance=distance,source=source,
            chosen_before_connection=True,downstream_validation_used=False))
        return selected

    def _guided_search(self,start,goals,lower,upper,validity,budget,seed,**kwargs):
        roots=start.reshape(-1,15)
        if len(roots)!=1:raise ValueError('串行实验不允许多根搜索反选吸附构型')
        selected,index,distance=nearest_endpoint(goals.detach().cpu().numpy(),roots[0].detach().cpu().numpy())
        fixed=torch.tensor(selected[None],device='cuda',dtype=torch.float32)
        if len(goals)>1:
            self.serial_records.append(dict(name=self.guidance_phase+'_fixed_before_search',joints=selected.tolist(),
                previous_joints=roots[0].detach().cpu().tolist(),candidate_count=len(goals),selected_index=index,
                weighted_distance=distance,source='nearest preceding configuration before RRT; no search-driven goal choice',
                chosen_before_connection=True,downstream_validation_used=False))
        self.weighted_reference=endpoint_references(roots[0].detach().cpu().numpy(),selected)
        validity.search_base_margin_m=self.search_base_margin_m
        endpoints=torch.cat((roots,fixed))
        endpoint_mask=validity.evaluate(endpoints,base_margin_m=self.search_base_margin_m)[0]
        if not bool(endpoint_mask.all()):
            failure='固定起终点不满足搜索底盘安全余量，不改选端点'
            self.guidance_events.append(dict(phase=self.guidance_phase,success=False,failure=failure,
                search_base_margin_m=self.search_base_margin_m,endpoint_margin_valid=endpoint_mask.cpu().tolist(),
                search_ms=0.,total_ms=0.))
            return None,dict(failure=failure,search_base_margin_m=self.search_base_margin_m,
                endpoint_margin_valid=endpoint_mask.cpu().tolist())
        path,stats=super()._guided_search(roots[0],fixed,lower,upper,validity,budget,seed,**kwargs)
        self.guidance_events[-1]['search_base_margin_m']=self.search_base_margin_m
        stats['search_base_margin_m']=self.search_base_margin_m
        return path,stats

    def _lm(self,samples,start,validity):
        rows,ids,stats=solve_numeric(self,'lm',samples,start.detach().cpu().numpy(),self.weighted_reference,
            validity.checker,True,self.endpoint_ik_cache)
        began=time.perf_counter()
        legal,collision=pose_filter(validity.checker,validity,rows,ids,samples)
        if getattr(self,'capture_guidance_nodes',False):
            self.guidance_events[-1]['node_diagnostics']=dict(joints=rows.cpu().tolist(),point_indices=ids.tolist(),
                legal=legal.tolist(),collision_valid=collision.tolist(),targets={s:m.tolist() for s,m in samples.items()})
        self.guidance_events[-1]['endpoint_weighted_reference']=self.weighted_reference.tolist()
        self.guidance_events[-1]['soft_refine_ms']=stats['soft_refine_ms']
        return rows[torch.tensor(legal,device='cuda')],ids[legal],dict(lm_ms=stats['solve_ms']+stats['soft_refine_ms'],
            filter_ms=(time.perf_counter()-began)*1000,solver_setup_ms=stats['solver_setup_ms'],
            legal_nodes=int(legal.sum()),covered_points=len(np.unique(ids[legal])),point_count=128,seeds_per_point=16,batch_points=64)

    def predicted_extraction(self,contact):
        self.fk(contact)
        carriage=np.linalg.inv(self.fk_robot.get_transform('arm_carriage',self.fk_robot.base_link))
        output=np.asarray(contact).copy()
        for side in self.selected_task:
            arm=0 if side=='left' else 1
            offset=1+7*arm
            target=self.fk_robot.get_transform(side+'_tool0',self.fk_robot.base_link).copy()
            target[0,3]-=.35
            swivel=self.analytic.swivel(arm,contact[offset:offset+7])
            solutions=self.analytic.solve(arm,carriage@target,swivel,contact[offset:offset+7])
            if not len(solutions):return None
            output[offset:offset+7]=solutions[np.argmin(np.linalg.norm(solutions-contact[offset:offset+7],axis=1))]
        return output

    def teleport_extraction_endpoint(self,contact,checker,validity):
        self.fk(contact)
        carriage=np.linalg.inv(self.fk_robot.get_transform('arm_carriage',self.fk_robot.base_link))
        targets={side:self.fk_robot.get_transform(side+'_tool0',self.fk_robot.base_link).copy() for side in self.selected_task}
        lower,upper=[value.cpu().numpy() for value in checker.kinematics.get_joint_limits().position]
        options={}
        for side,target in targets.items():
            arm=0 if side=='left' else 1
            offset=1+7*arm;target[0,3]-=.35
            solutions=[]
            for swivel in np.deg2rad(np.arange(-180,180,10)):
                branches=self.analytic.solve(arm,carriage@target,swivel,contact[offset:offset+7])
                for branch in branches:
                    if np.all(branch>=lower[offset:offset+7]) and np.all(branch<=upper[offset:offset+7]):solutions.append(branch)
            if not solutions:return None,dict(reason=side+'固定Updown枚举冗余后终点无解析解')
            rows=np.unique(np.round(solutions,8),axis=0)
            order=np.argsort(np.linalg.norm(rows-contact[offset:offset+7],axis=1))
            options[side]=rows[order[:64]]
        candidates=[]
        if len(options)==2:
            for left in options['left']:
                for right in options['right']:candidates.append(np.r_[contact[0],left,right])
        else:
            side=next(iter(options));offset=1 if side=='left' else 8
            for branch in options[side]:
                row=np.asarray(contact).copy();row[offset:offset+7]=branch;candidates.append(row)
        candidates=np.asarray(candidates,dtype=np.float32)
        distance=np.linalg.norm((candidates-contact)*WEIGHTS,axis=1)
        candidates=candidates[np.argsort(distance)]
        accepted=[]
        for offset in range(0,len(candidates),512):
            batch=torch.tensor(candidates[offset:offset+512],device='cuda',dtype=torch.float32)
            mask=validity.mask(batch)
            state=checker.kinematics.compute_kinematics(JointState.from_position(batch[None],joint_names=ACTIVE_JOINTS))
            for side,target in targets.items():
                pose=state.tool_poses.to_dict()[side+'_tool0']
                position=torch.tensor(target[:3,3],device='cuda',dtype=torch.float32)
                quaternion=torch.tensor(np.roll(Rotation.from_matrix(target[:3,:3]).as_quat(),1),device='cuda',dtype=torch.float32)
                mask&=torch.linalg.vector_norm(pose.position.reshape(-1,3)-position,dim=-1)<=.002
                mask&=(pose.quaternion.reshape(-1,4)*quaternion).sum(-1).abs()>=np.cos(np.deg2rad(1)/2)
            accepted.extend(batch[mask].cpu().tolist())
        if not accepted:return None,dict(reason='固定Updown终点候选均未通过携箱碰撞/Pose验收',raw_pairs=len(candidates))
        selected,index,distance=nearest_endpoint(accepted,contact)
        return selected,dict(raw_pairs=len(candidates),valid_endpoints=len(accepted),fixed_updown_m=float(contact[0]),
            redundancy_step_deg=10,endpoint_weighted_distance=distance,trajectory_checked=False)

    def transport_candidates(self,previous,checker,validity):
        atlas=np.load(atlas_path(self))['joints'].copy()
        if len(self.selected_task)==1:
            variants=[atlas]
            for idle in self.free_arm_variants(previous):
                values=atlas.copy();values[:,self.idle_indices]=idle;variants.append(values)
            atlas=np.unique(np.round(np.concatenate(variants),6),axis=0).astype(np.float32)
        radius_limit=json.loads((ROOT/'config/placement_task_policy.json').read_text())['side']['placement_radius_m']
        selected=[]
        for offset in range(0,len(atlas),512):
            batch=torch.tensor(atlas[offset:offset+512],device='cuda',dtype=torch.float32)
            valid=validity.mask(batch)&(self.full_envelope_radius(checker,batch)<=radius_limit)
            state=checker.kinematics.compute_kinematics(JointState.from_position(batch[None],joint_names=ACTIVE_JOINTS))
            for side in self.selected_task:
                target=self.target_poses[side+'_tool0']
                pose=state.tool_poses.to_dict()[side+'_tool0']
                valid&=torch.linalg.vector_norm(pose.position.reshape(-1,3)-torch.tensor(target['position'],device='cuda'),dim=-1)<=.002
                valid&=(pose.quaternion.reshape(-1,4)*torch.tensor(target['quaternion'],device='cuda')).sum(-1).abs()>=np.cos(np.deg2rad(1)/2)
            selected.extend(batch[valid].cpu().tolist())
        if not selected and len(self.selected_task)==1:
            rows,report=solve_idle_candidates(self,atlas,previous,checker,64,radius_limit)
            self.last_partial['transport_idle_arm_solver']=report
            selected=np.asarray(rows).tolist()
        return np.asarray(selected).reshape(-1,15),radius_limit

    def _full_cycle(self,boxes,progress=lambda text:None):
        began=time.perf_counter()
        names=list(self.mobile_robot['kinematics']['cspace']['joint_names'])
        report=dict(success=False,task=boxes,joint_names=names,frames=[],phases=[],payload=[],attempts=[],
            planner='serial_endpoint_weighted_lm_informed_connect',timing_ms={},serial_endpoint_records=self.serial_records,
            endpoint_selection_policy='strict predecessor-only choice; one frozen contact/extraction/transport/conveyor endpoint, no downstream backtracking',
            single_arm=len(boxes)==1,rotation_quality_hard_gate=False)
        self.last_partial=report
        def append(rows,phase,payload):
            for row in rows:
                mapping=dict(zip(ACTIVE_JOINTS,row)) if len(row)==15 else dict(zip(names,row))
                report['frames'].append([float(mapping.get(name,0)) for name in names]);report['phases'].append(phase);report['payload'].append(payload)
        empty=self.checker(boxes);loaded=self.checker(boxes,payload=True)
        empty_valid=GpuValidity(empty,check_ground=True);loaded_valid=GpuValidity(loaded,check_ground=True)
        append([self.home_values],'home',False)
        if not bool(empty_valid.mask(torch.tensor(self.home_values[None],device='cuda',dtype=torch.float32)).all()):
            raise CycleBlocked('Home','初始构型碰撞',report)
        progress('串行选型1：仅参考Home选择一个吸附构型')
        started=time.perf_counter()
        if len(boxes)==1:
            target=self.contact_poses_for_task(boxes)[self.active_side+'_tool0']
            matrix=np.eye(4);matrix[:3,3]=target['position'];matrix[:3,:3]=Rotation.from_quat(np.roll(target['quaternion'],-1)).as_matrix()
            contacts=self.enumerate_single(matrix,self.home_values,empty)
        else:
            solved=self.build_contacts(boxes)
            contacts=solved.js_solution.position.detach().cpu().numpy().reshape(-1,len(solved.js_solution.joint_names))
            contacts=contacts[:,[solved.js_solution.joint_names.index(name) for name in ACTIVE_JOINTS]]
        report['timing_ms']['contact_candidates_ms']=(time.perf_counter()-started)*1000
        report['contact_ik_count']=len(contacts)
        report['contact_candidates']=np.asarray(contacts).tolist()
        if not len(contacts):raise CycleBlocked('接触IK','当前场景没有合法接触候选',report)
        contact=self.record_endpoint('contact',contacts,self.home_values,'fresh contact enumeration; nearest Home, no extraction/transport lookahead')
        report['selected_candidate']=self.serial_records[-1]['selected_index']+1
        started=time.perf_counter()
        if self.extraction_teleport:
            progress('串行选型2：只求抽离终点，跳过中间抽离轨迹')
            endpoint,detail=self.teleport_extraction_endpoint(contact,loaded,loaded_valid)
            if endpoint is None:raise CycleBlocked('瞬移抽离终点IK',f'没有可显示的合法终点：{detail}；不反向换吸附构型',report)
            predicted=endpoint.copy();extracted=endpoint[None]
            report['extraction_policy']=dict(mode='teleport_only',distance_m=.35,path_generated=False,path_validated=False,
                fixed_updown=True,redundancy_free_at_endpoint=True,endpoint_only=detail)
        else:
            predicted=self.predicted_extraction(contact)
            if predicted is None:raise CycleBlocked('抽离终点预测','选定吸附构型直接解析后撤35cm无解，不回换吸附构型',report)
            extracted,detail=None,None
        if not self.extraction_teleport and len(boxes)==1:
            extracted,detail=self.cartesian_single(contact,np.linspace(.01,.35,35))
            if extracted is not None:
                valid=empty_valid.mask(torch.tensor(extracted,device='cuda',dtype=torch.float32)).all()
                valid&=loaded_valid.mask(torch.tensor(extracted[-1:],device='cuda',dtype=torch.float32)).all()
                if not bool(valid):extracted=None;detail='选定构型抽离路径或携箱终点碰撞'
        elif not self.extraction_teleport:extracted,detail=FullCyclePlanner.analytic_extract(self,contact,empty_valid,loaded_valid)
        report['timing_ms']['single_extraction_ms']=(time.perf_counter()-started)*1000
        if extracted is None:raise CycleBlocked('串行抽离验收',f'仅选定吸附构型失败：{detail}；不反向改选',report)
        endpoint=np.asarray(extracted[-1])
        self.serial_records.append(dict(name='extraction',joints=endpoint.tolist(),previous_joints=contact.tolist(),
            source='teleported endpoint, no extraction path validation' if self.extraction_teleport else 'single verified analytic extraction from chosen contact',prediction_joint_error=float(np.max(np.abs(endpoint-predicted))),downstream_validation_used=False))
        progress('串行选型3：运输终点只按抽离终点距离与本阶段合法性选择')
        started=time.perf_counter()
        candidates,radius_limit=self.transport_candidates(endpoint,loaded,loaded_valid)
        report['timing_ms']['transport_endpoint_ms']=(time.perf_counter()-started)*1000
        if not len(candidates):raise CycleBlocked('串行运输终点','没有满足运输目标Pose/整机碰撞/圆柱的终点',report)
        transport_goal=self.record_endpoint('transport',candidates,endpoint,'frozen placement-Pose analytic atlas, nearest verified extraction endpoint; current-stage collision/Pose/cylinder only')
        progress('连接Home→3cm预接触：起终共同加权LM先验＋首合法Informed Connect')
        self.guidance_phase='approach'
        if len(boxes)==1:
            backwards,detail=self.cartesian_single(contact,np.linspace(.005,.03,6),True)
            if backwards is None or not empty_valid.mask(torch.tensor(backwards,device='cuda',dtype=torch.float32)).all():
                raise CycleBlocked('预接触解析段','选定接触构型的反向3cm接近不可行',report)
            approach,stats=self.single_path(self.home_values,[backwards[-1]],empty,strict=self.target_validity(empty))
            if approach is None:raise CycleBlocked('串行Home到位','固定吸附构型的预接触位搜索失败，不改选',report)
            append(approach,'approach',False);append(backwards[::-1],'contact_approach',False)
        else:
            approach,stats=TaskCyclePlanner.plan_contact_approach(self,contact,empty,empty_valid,20261023)
            if approach is None:raise CycleBlocked('串行Home到位',f'固定预接触位失败，不改选：{stats}',report)
            split=self.contact_approach_split
            append(approach[:split],'approach',False);append(approach[split:],'contact_approach',False)
        report['approach_search_stats']=stats
        append([contact],'attach',True);append(extracted,'extraction_teleport' if self.extraction_teleport else 'extract',True)
        self.guidance_phase='transport'
        if len(boxes)==1:transport,stats=self.single_path(endpoint,[transport_goal],loaded,True)
        else:
            validity=GpuValidity(loaded,check_ground=True)
            validity.weights[0]=np.deg2rad(15)/.1
            validity.box_rotation_cost=BoxRotationCost(loaded,ACTIVE_JOINTS,torch.tensor(endpoint,device='cuda',dtype=torch.float32),torch.tensor(transport_goal,device='cuda',dtype=torch.float32),self.top_grasp)
            lower,upper=loaded.kinematics.get_joint_limits().position
            path,stats=self._guided_search(torch.tensor(endpoint,device='cuda',dtype=torch.float32),torch.tensor(transport_goal[None],device='cuda',dtype=torch.float32),lower,upper,validity,2.,20261024)
            transport=np.asarray(densify(path)) if path is not None else None
        if transport is None:raise CycleBlocked('串行运输搜索','固定抽离/运输终点未连通，不回换接触或运输构型',report)
        if not loaded_valid.mask(torch.tensor(transport,device='cuda',dtype=torch.float32)).all():raise CycleBlocked('串行运输验收','运输密化碰撞',report)
        report['transport_search_stats']=stats
        report['transport_endpoint_envelope']=dict(radius_m=float(self.full_envelope_radius(loaded,transport[-1:]).item()),limit_m=radius_limit,includes_idle_arm=True,includes_payload=True)
        append(transport,'transport',True)
        self.conveyor_suffix(boxes,report,append,progress)
        values=np.asarray(report['frames'])[:,[names.index(name) for name in ACTIVE_JOINTS]]
        report['final_home_error']=float(np.abs(values[-1]-self.home_values).max())
        report.update(success=True,total_ms=(time.perf_counter()-began)*1000)
        return report

    def plan_conveyor_target(self,start,targets,checker,validity,solver,report,append):
        self.guidance_phase='conveyor_place'
        if len(self.selected_task)==1:
            candidates=self.enumerate_single(targets[self.active_side],start,checker)
        else:
            poses={side+'_tool0':dict(position=matrix[:3,3].tolist(),quaternion=np.roll(Rotation.from_matrix(matrix[:3,:3]).as_quat(),1).tolist()) for side,matrix in targets.items()}
            state=JointState.from_position(torch.tensor(np.asarray(start)[None],device='cuda',dtype=torch.float32),joint_names=ACTIVE_JOINTS)
            with torch.enable_grad():solved=solver.solve_pose(goal_message(poses),current_state=state,return_seeds=64)
            rows=solved.js_solution.position.reshape(-1,len(solved.js_solution.joint_names))
            rows=rows[:,[solved.js_solution.joint_names.index(name) for name in ACTIVE_JOINTS]]
            candidates=rows[solved.success.reshape(-1)].detach().cpu().numpy()
        margin=self.conveyor_search_margin_m
        candidates=np.asarray(candidates).reshape(-1,15)
        if len(candidates):
            tensor=torch.tensor(candidates,device='cuda',dtype=torch.float32)
            mask=validity.mask(tensor)&validity.scene_margin_mask(tensor,margin)
            candidates=candidates[mask.cpu().numpy()]
        report['conveyor_goal_ik']=dict(collision_valid=len(candidates),mode='predecessor-only endpoint selection, frozen before search')
        if not len(candidates):raise CycleBlocked('串行龙头车终点','运输构型为种子未找到合法放箱终点，不回换运输构型',report)
        target=self.record_endpoint('conveyor_place',candidates,start,'goal IK seeded from transport, nearest transport configuration; fixed before path search')
        validity.search_scene_margin_m=margin
        validity.search_edge_resolution=self.conveyor_search_edge_resolution
        lower,upper=checker.kinematics.get_joint_limits().position
        if len(self.selected_task)==1:
            previous=self._placement_search
            self._placement_search=True
            try:
                dense,stats=self.single_path(start,[target],checker,scene_margin_m=margin)
            finally:
                self._placement_search=previous
        else:
            path,stats=self._guided_search(torch.tensor(start,device='cuda',dtype=torch.float32),torch.tensor(target[None],device='cuda',dtype=torch.float32),lower,upper,validity,2.,20261025)
            dense=np.asarray(densify(path)) if path is not None else None
        report['conveyor_search_stats']=stats
        if dense is None:raise CycleBlocked('串行龙头车放箱搜索','固定运输→放箱构型未连通，不回溯换终点',report)
        if not validity.mask(torch.tensor(dense,device='cuda',dtype=torch.float32)).all():raise CycleBlocked('串行放箱验收','放箱路径密化碰撞',report)
        names=report['joint_names']
        frames=np.repeat(np.asarray(report['frames'][-1])[None],len(dense),axis=0)
        frames[:,[names.index(name) for name in ACTIVE_JOINTS]]=dense
        append(frames,'conveyor_place',True)
        return dense[-1]

    def full_cycle(self,boxes,progress=lambda text:None):
        self.configure_task(boxes)
        self.serial_records=[];self.guidance_events=[]
        self.checker_resource_events.clear()
        self.cached_selection=None
        result=ConveyorCyclePlanner.full_cycle(self,boxes,progress)
        result['serial_endpoint_records']=copy.deepcopy(self.serial_records)
        result['guided_search_events']=copy.deepcopy(self.guidance_events)
        result['guided_search_policy']='Serial predecessor-only terminal selection, fixed single endpoints, endpoint-weighted LM guidance + first finely valid Informed Connect; no joint corridor iteration'
        result['checker_resource_events']=list(self.checker_resource_events)
        result['search_base_margin_m']=self.search_base_margin_m
        return complete_playback_fields(result)
