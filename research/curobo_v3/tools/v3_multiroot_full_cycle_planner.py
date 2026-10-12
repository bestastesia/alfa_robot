import copy
import math
import time
from types import SimpleNamespace

import numpy as np
import torch
from v3_cuboid_full_cycle_planner import BallFullCyclePlanner
from curobo_core.planner import CycleBlocked
from v3_all_extraction_roots_test import main as prepare_roots
from v3_multiroot_search import batched_rrt_connect_multi_goal
from v3_batched_loaded_search import GpuValidity,densify
from v3_box_rotation_cost import BoxRotationCost
from v3_wall_ik_benchmark import ACTIVE_JOINTS


class MultiRootFullCyclePlanner(BallFullCyclePlanner):
    def __init__(self,args):
        super().__init__(args)
        self.cached_selection=None
        self.reusing_contact_result=False
        self.validate_turns=False

    def prepare_roots(self, boxes):
        return prepare_roots(self, boxes, True)

    def ik_solver(self,boxes):
        if self.reusing_contact_result:
            return SimpleNamespace(solve_pose=lambda *args,**kwargs:self.saved_contact_result)
        return super().ik_solver(boxes)

    def analytic_extract(self,contact,empty_validity,loaded_validity):
        if self.cached_selection is not None:
            return np.asarray(self.cached_selection['root']['extraction_frames']),self.cached_selection['extraction_detail']
        return super().analytic_extract(contact,empty_validity,loaded_validity)

    def plan_contact_approach(self,contact,checker,validity,seed):
        if self.cached_selection is not None:
            self.contact_approach_split=self.cached_selection['approach_split']
            return self.cached_selection['approach'],self.cached_selection['approach_stats']
        return super().plan_contact_approach(contact,checker,validity,seed)

    def transport_and_turn(self,boxes,transport,stats):
        names=list(self.mobile_robot['kinematics']['cspace']['joint_names'])
        frames=[]
        for row in transport:
            values=dict(zip(ACTIVE_JOINTS,row));frames.append([float(values.get(name,0)) for name in names])
        yaw=names.index('base_yaw');endpoint=frames[-1].copy()
        for step in range(1,361):
            row=endpoint.copy();row[yaw]=math.pi*step/360;frames.append(row)
        bad=[]
        if self.validate_turns:
            checker=self.checker(boxes,payload=True,mobile=True)
            validity=GpuValidity(checker,check_ground=True,joint_names=names,weights=[2.,2.,1.,5.]+[1.]*14)
            mask=validity.mask(torch.tensor(frames,device='cuda',dtype=torch.float32))
            bad=(~mask).nonzero().reshape(-1)
        failure=None
        if len(bad):
            index=int(bad[0]);failure={'stage':'Yaw180' if index>=len(transport) else '搬运','frame_index':index,'yaw_deg':math.degrees(frames[index][yaw])}
        return {'success':failure is None,'failure':failure,'frames':frames,'transport_frames':len(transport),'joint_names':names,'rrt_stats':stats,'total_ms':0,'timing_ms':{},'stl_checks':False}

    def plan(self,boxes,start_values):
        assert self.cached_selection is not None
        assert np.allclose(start_values,self.cached_selection['root']['start'],atol=1e-5)
        self.last_partial['transport_rotation_metrics']=self.cached_selection['rotation']
        return self.cached_selection['transport_result']

    def _full_cycle(self,boxes,progress=lambda text:None):
        began=time.perf_counter();self.cached_selection=None
        detail_times={'approach_plan_validate':0.,'loaded_turn_setup_validate':0.,'empty_turn_setup_validate':0.,'home_plan_validate':0.,'rotation_and_bookkeeping':0.}
        with torch.no_grad():
            empty=self.checker(boxes)
            empty_validity=GpuValidity(empty,check_ground=True)
            if not empty_validity.mask(torch.tensor(self.home_values[None],device='cuda',dtype=torch.float32)).all():
                raise CycleBlocked('Home','初始构型碰撞',self.last_partial)
            torch.cuda.synchronize()
            detail_times['initial_home_checker_setup_validate']=(time.perf_counter()-began)*1000
            progress('生成接触候选，保留全部合法解析抽离终点并匹配目标')
            preparation,roots,goals,contact_result=self.prepare_roots(boxes)
            self.saved_contact_result=contact_result
            loaded=self.checker(boxes,payload=True)
            validity=GpuValidity(loaded,check_ground=True,weights=[math.radians(15)/.1]+[1.]*14)
            validity.collision_weights=torch.tensor([5.]+[1.]*14,device='cuda')
            common=BoxRotationCost(loaded,ACTIVE_JOINTS,roots[0],goals[0],self.top_grasp)
            root_orientations=common.orientations(roots)
            goal_orientations=common.orientations(goals)
            for side in root_orientations:
                assert ((root_orientations[side]*common.initial[side]).sum(-1).abs()>math.cos(math.radians(.1)/2)).all()
                assert ((goal_orientations[side]*goal_orientations[side][:1]).sum(-1).abs()>math.cos(math.radians(.1)/2)).all()
            validity.box_rotation_cost=common
            quality_rejections=[]
            def quality(values):
                metric=BoxRotationCost(loaded,ACTIVE_JOINTS,values[0],values[-1],self.top_grasp)
                metrics=metric.metrics(values)
                accepted=all(item['total_rotation_deg']-item['required_rotation_deg']<=90 and item['backtracking_deg']<=15 and item['max_deviation_deg']<=45 for item in metrics.values())
                if not accepted:quality_rejections.append(metrics)
                return accepted
            validity.path_quality_callback=quality
            lower,upper=loaded.kinematics.get_joint_limits().position
            attempts=[];active_roots=list(range(len(roots)));active_goals=list(range(len(goals)))
            search_total=0.;validation_total=0.;selection=None
            while active_roots and active_goals and search_total<2000:
                progress(f'联合搜索：{len(active_roots)}抽离根／{len(active_goals)}放置目标')
                segment=time.perf_counter()
                path,stats=batched_rrt_connect_multi_goal(roots[active_roots].contiguous(),goals[active_goals].contiguous(),lower+1e-5,upper-1e-5,validity,(2000-search_total)/1000,621648851+len(attempts),stop_at_first_valid=True)
                torch.cuda.synchronize();search_total+=(time.perf_counter()-segment)*1000
                if path is None:break
                dense=np.asarray(densify(path))
                root_index=active_roots[int(torch.linalg.vector_norm(roots[active_roots]-torch.tensor(dense[0],device='cuda'),dim=-1).argmin())]
                goal_index=active_goals[int(torch.linalg.vector_norm(goals[active_goals]-torch.tensor(dense[-1],device='cuda'),dim=-1).argmin())]
                root=preparation['root_records'][root_index]
                progress(f"验证候选{root['contact_index']+1}的Home到位与携箱转身")
                segment=time.perf_counter()
                approach,approach_stats=super().plan_contact_approach(np.asarray(root['contact']),empty,empty_validity,20261003+root['contact_index'])
                torch.cuda.synchronize();detail_times['approach_plan_validate']+=(time.perf_counter()-segment)*1000
                attempt={'root':root_index,'contact':root['contact_index'],'goal':goal_index,'search':stats,'approach':approach_stats}
                if approach is None:
                    active_roots.remove(root_index);attempt['rejected']='Home到位';attempts.append(attempt);validation_total+=(time.perf_counter()-segment)*1000;continue
                split=self.contact_approach_split
                sub_started=time.perf_counter()
                transport_result=self.transport_and_turn(boxes,dense,stats)
                torch.cuda.synchronize();detail_times['loaded_turn_setup_validate']+=(time.perf_counter()-sub_started)*1000
                if not transport_result['success']:
                    active_goals.remove(goal_index);attempt['rejected']='携箱转身';attempt['failure']=transport_result['failure'];attempts.append(attempt);validation_total+=(time.perf_counter()-segment)*1000;continue
                names=transport_result['joint_names'];last=np.asarray(transport_result['frames'][-1]);turn=np.repeat(last[None],361,axis=0);turn[:,names.index('base_yaw')]=np.linspace(math.pi,0,361)
                sub_started=time.perf_counter()
                if self.validate_turns:
                    empty_mobile=self.checker(boxes,mobile=True)
                    mobile_validity=GpuValidity(empty_mobile,check_ground=True,joint_names=names,weights=[2.,2.,1.,5.]+[1.]*14)
                    if not mobile_validity.mask(torch.tensor(turn,device='cuda',dtype=torch.float32)).all():
                        active_goals.remove(goal_index);attempt['rejected']='空载转回';attempts.append(attempt);validation_total+=(time.perf_counter()-segment)*1000;continue
                torch.cuda.synchronize();detail_times['empty_turn_setup_validate']+=(time.perf_counter()-sub_started)*1000
                home_start=np.array([turn[-1,names.index(name)] for name in ACTIVE_JOINTS])
                sub_started=time.perf_counter()
                home_path,home_stats=self.joint_plan(home_start,self.home_values,empty,empty_validity,20261004)
                torch.cuda.synchronize();detail_times['home_plan_validate']+=(time.perf_counter()-sub_started)*1000
                if home_path is None:
                    active_goals.remove(goal_index);attempt['rejected']='回Home';attempts.append(attempt);validation_total+=(time.perf_counter()-segment)*1000;continue
                dense_tensor=torch.tensor(dense,device='cuda',dtype=torch.float32).contiguous()
                sub_started=time.perf_counter()
                rotation=BoxRotationCost(loaded,ACTIVE_JOINTS,dense_tensor[0],dense_tensor[-1],self.top_grasp).metrics(dense_tensor)
                detail=next(item['detail'] for item in preparation['attempts'] if item['contact_index']==root['contact_index'])
                selection={'root':root,'approach':approach,'approach_stats':approach_stats,'approach_split':split,'extraction_detail':detail,'transport_result':transport_result,'rotation':rotation,'home_path':home_path,'home_stats':home_stats}
                attempts.append(attempt);validation_total+=(time.perf_counter()-segment)*1000;break
            if selection is None:
                self.last_partial={'success':False,'task':boxes,'frames':[],'multiroot_preparation':preparation,'selection_attempts':attempts,'quality_rejections':len(quality_rejections)}
                raise CycleBlocked('全链路候选验证','搜索上限内未找到可到位、转身和回Home的低旋转组合',self.last_partial)
            self.cached_selection=selection;self.comparison_contact=selection['root']['contact'];self.reusing_contact_result=True
            detail_times['rotation_and_bookkeeping']=validation_total-sum(detail_times[key] for key in ['approach_plan_validate','loaded_turn_setup_validate','empty_turn_setup_validate','home_plan_validate'])
            assembly_started=time.perf_counter()
            try:
                result=super()._full_cycle(boxes,progress)
            finally:
                self.reusing_contact_result=False;self.comparison_contact=None
            result['multiroot_preparation']=preparation
            detail_times['assembly_revalidate_actual']=(time.perf_counter()-assembly_started)*1000
            result['validation_detail_ms']=detail_times
            result['selection_attempts']=attempts
            result['quality_rejections']=len(quality_rejections)
            result['selected_original_contact_index']=selection['root']['contact_index']
            result['selected_candidate']=selection['root']['contact_index']+1
            result['pipeline_timing_ms']={'prepare_checkers':preparation['setup_ms'],'contact_ik':preparation['contact_ik_ms'],'all_extract_and_validation':preparation['extraction_ms'],'goal_matching':preparation['matching_ms'],'multiroot_search':search_total,'selected_prefix_suffix_validation':validation_total,'assembly_and_repeat_validation':(time.perf_counter()-began)*1000-(preparation['setup_ms']+preparation['contact_ik_ms']+preparation['extraction_ms']+preparation['matching_ms']+search_total+validation_total)}
            result['total_ms']=(time.perf_counter()-began)*1000
            result['planner']='multiroot_connect'
            result['turn_collision_validation']='skipped per user instruction; turn actions retained'
            return result
