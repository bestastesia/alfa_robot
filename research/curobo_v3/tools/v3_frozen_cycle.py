import copy
import time

import numpy as np
import torch

from curobo_core.planner import CycleBlocked
from v3_lm_conveyor import SequentialPlacementLm
from v3_restored_analytic_cycle import RestoredAnalyticPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS


PROFILE='20261106-lm-extraction-rrt-transport-lm-placement-frozen40-v2'


class FrozenCyclePlanner(RestoredAnalyticPlanner):
    pipeline_label='连续LM抽离＋RRT运输＋连续LM放箱'
    pipeline_description='旧多候选预测排序与8/16/32/64分批验收，128点15轴连续LM沿原抽离方向移动35cm，Updown自由，空闲臂保持起点；多根多目标RRT运输；导航瞬移后连续LM放箱，抽离与放箱均无RRT回退；释放后从实际放箱末态用RRT回Home。全程原40球模型。'
    conveyor_placement_description='导航到位瞬移完成；底盘冻结，使用15轴连续LM放箱，启用携箱40球与龙头车碰撞检查'
    navigation_export_enabled=False
    record_lm_extraction_candidates=False

    def __init__(self,args=None,*,record_lm_extraction_candidates=False):
        super().__init__(args)
        self.placement_lm_solvers={}
        self.record_lm_extraction_candidates=record_lm_extraction_candidates
        self.lm_extraction_candidate_reviews=[]

    def placement_solver(self):
        sides=tuple(sorted(self.selected_task))
        if sides not in self.placement_lm_solvers:
            self.placement_lm_solvers[sides]=SequentialPlacementLm(self,seeds=32,iterations=32,samples=128,active_sides=sides)
        return self.placement_lm_solvers[sides]

    def conveyor_ik_solver(self,boxes):
        return None

    def analytic_extract(self,contact,empty_validity,loaded_validity):
        key=tuple(np.round(np.asarray(contact,dtype=np.float64),7))
        if getattr(self,'_lazy_preparing',False):
            index=self._contact_indices[key]
            if index not in self._allowed_contacts:return None,'deferred to next historical8/16/32/64 batch'
            if key in self._extraction_cache:return self._extraction_cache[key]
        began=time.perf_counter()
        checker=loaded_validity.checker if loaded_validity is not None else self.checker(self.selected_task,payload=True)
        start=np.asarray(contact,dtype=np.float32)
        attachments={side:np.asarray(matrix) for side,matrix in checker.task_tool_to_box.items()}
        tools=self._tool_poses(checker,torch.tensor(start,device='cuda',dtype=torch.float32))
        starts={side:tools[side]@matrix for side,matrix in attachments.items()}
        displacement=.35*np.asarray(getattr(self,'extraction_direction',(-1.,0.,0.)),dtype=float)
        ends={side:matrix.copy() for side,matrix in starts.items()}
        for matrix in ends.values():matrix[:3,3]+=displacement
        engine=self.placement_solver()
        result=engine.plan(dict(start=start,checker=checker,attachments=attachments,starts=starts,ends=ends))
        if self.record_lm_extraction_candidates and not result['success']:
            if not hasattr(self,'lm_extraction_candidate_reviews'):self.lm_extraction_candidate_reviews=[]
            if len(self.lm_extraction_candidate_reviews)<64:
                self.lm_extraction_candidate_reviews.append(copy.deepcopy(dict(result,
                    contact=np.asarray(contact).tolist(),start=start.tolist(),joint_names=list(ACTIVE_JOINTS),
                    task=dict(self.selected_task),translation_local={side:displacement.tolist() for side in attachments})))
        detail=dict(result,frames=None,tracks=None,solver_setup_ms=engine.setup_ms,
            translation_local={side:displacement.tolist() for side in attachments},updown_free=True)
        value=(np.asarray(result['frames']) if result['success'] else None,detail)
        if getattr(self,'_lazy_preparing',False):
            self._extraction_cache[key]=value
            self.lazy_timings['extraction_work_ms']+=(time.perf_counter()-began)*1000
        return value

    def cartesian_single(self,contact,distances,along_normal=False):
        if along_normal:return super().cartesian_single(contact,distances,along_normal)
        return self.analytic_extract(contact,None,None)

    def plan_conveyor_target(self,start,targets,checker,validity,solver,report,append):
        attachments={side:np.asarray(matrix) for side,matrix in checker.task_tool_to_box.items()}
        tools=self._tool_poses(checker,torch.tensor(np.asarray(start),device='cuda',dtype=torch.float32))
        case=dict(start=np.asarray(start,dtype=np.float32),checker=checker,attachments=attachments,
            starts={side:tools[side]@matrix for side,matrix in attachments.items()},
            ends={side:targets[side]@matrix for side,matrix in attachments.items()})
        engine=self.placement_solver()
        result=engine.plan(case)
        report['conveyor_place_mode']='continuous15axis LM, box-center line and orientation interpolation, free terminal joints, no RRT fallback'
        report['conveyor_place_lm']={key:value for key,value in result.items() if key!='frames'}
        report['conveyor_place_lm']['solver_setup_ms']=engine.setup_ms
        names=report['joint_names']
        frames=np.repeat(np.asarray(report['frames'][-1])[None],len(result['frames']),axis=0)
        frames[:,[names.index(name) for name in ACTIVE_JOINTS]]=result['frames']
        append(frames,'conveyor_place',True)
        if not result['success']:
            raise CycleBlocked('龙头车放置LM',f"进度{result['completed_progress']*100:.2f}%：{result['failure']}",report)
        return np.asarray(result['frames'][-1],dtype=np.float32)

    def full_cycle(self,boxes,progress=lambda text:None):
        self.lm_extraction_candidate_reviews=[]
        result=super().full_cycle(boxes,progress)
        result.update(algorithm='frozen_cycle',cycle_profile=PROFILE,
            lm_extraction_candidate_reviews=self.lm_extraction_candidate_reviews,
            endpoint_selection_policy='historical64 contacts, unchanged analytic predicted ranking,8/16/32/64 LM-verified extraction roots and multi-root/multi-goal RRT; predictions are ranking only',
            strict_cartesian_extraction=False,
            strategy=dict(contact='historical multi-candidate',extraction='sequential15axis LM35cm128points32seeds32iterations, free Updown, idle arm held at start',
                transport='multi-root multi-goal RRT',placement='sequential15axis LM128points32seeds32iterations',
                return_home='RRT from actual LM terminal state',payload_model='frozen40',rrt_extraction_fallback=False,rrt_placement_fallback=False))
        return result
