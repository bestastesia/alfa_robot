import argparse,json,math,os,sys,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import torch
from scipy.spatial.transform import Rotation

root=Path(__file__).resolve().parents[1]
from v3_multiroot_full_cycle_planner import MultiRootFullCyclePlanner
from v3_batched_loaded_search import GpuValidity
from v3_wall_ik_benchmark import ACTIVE_JOINTS
from curobo.types import JointState
from v3_interactive_60_tasks import DEFAULT_ROBOT_CONFIG,DEFAULT_URDF,DEFAULT_NAMED_POSES,DEFAULT_BOX_FIT,DEFAULT_MOBILE_ROBOT_CONFIG

output=root/'generated/mixed_placement_live'
from v3_all_extraction_roots_test import main as normal_prepare
from v3_side_upward_prepare import main as upward_prepare
def mixed_prepare(planner,boxes,prepare_only):
    return (normal_prepare if planner.top_grasp else upward_prepare)(planner,boxes,prepare_only)
from curobo_core.distance_metric import joint_distance_weights
from v3_research_assets import planner_identity, validate_atlas

class AnalyticContacts(MultiRootFullCyclePlanner):
    def prepare_roots(self, boxes):
        validate_atlas(self)
        return mixed_prepare(self, boxes, True)
    def full_cycle(self,boxes,progress=lambda text:None):
        self.cached_selection=None
        self.last_partial={'success':False,'task':boxes,'frames':[],'phases':[],'payload':[],
                           'joint_names':list(self.mobile_robot['kinematics']['cspace']['joint_names'])}
        result=super().full_cycle(boxes,progress)
        result.setdefault('frames',[])
        result.setdefault('phases',[])
        result.setdefault('payload',[])
        result.setdefault('joint_names',list(self.mobile_robot['kinematics']['cspace']['joint_names']))
        result['placement_tool_poses']=self.target_poses
        result['analytic_contact_generation']=getattr(self,'analytic_contact_report',{})
        result['atlas_validation']=getattr(self,'atlas_validation',{})
        generation=result['analytic_contact_generation']
        timing=result.setdefault('pipeline_timing_ms',{})
        timing['analytic_contact_enumeration']=generation.get('enumeration_ms',0.0)
        timing['analytic_contact_collision_fk_filter']=generation.get('collision_and_fk_filter_ms',0.0)
        timing['analytic_contact_total']=generation.get('total_ms',0.0)
        timing['contact_ik_return_cached']=timing.pop('contact_ik',0.0)
        result['timing_note']='Contact generation is inside preparation setup; component timings overlap and must not be summed as disjoint stages.'
        return result
    def configure_task(self,boxes):
        result=super().configure_task(boxes)
        policy=json.loads((root/'config/placement_task_policy.json').read_text())['top' if self.top_grasp else 'side']
        self.target_poses={side+'_tool0':dict(position=[policy['position']['x'],policy['position']['y_'+side],policy['position']['z']],quaternion=policy['quaternion_wxyz']) for side in ['left','right']}
        return result
    def ik_solver(self,boxes):
        if self.reusing_contact_result:return super().ik_solver(boxes)
        if getattr(self,'analytic_task',None)!=planner_identity(self, boxes):
            self.analytic_result=self.build_contacts(boxes)
            self.analytic_task=planner_identity(self, boxes)
        return SimpleNamespace(solve_pose=lambda *args,**kwargs:self.analytic_result)

    def build_contacts(self,boxes):
        began=time.perf_counter();checker=self.checker(boxes);validity=GpuValidity(checker,check_ground=True)
        lower,upper=[value.cpu().numpy() for value in checker.kinematics.get_joint_limits().position]
        targets=self.contact_poses_for_task(boxes);transforms={}
        for side in ['left','right']:
            target=targets[side+'_tool0'];matrix=np.eye(4);matrix[:3,3]=target['position'];matrix[:3,:3]=Rotation.from_quat(np.roll(target['quaternion'],-1)).as_matrix();transforms[side]=matrix
        batches=[];scanned=0;counts=[];enumeration_ms=0.;filter_ms=0.
        for updown in np.arange(-1.,.0001,.05):
            updown=float(round(updown,2))
            if updown<lower[0]-1e-7 or updown>upper[0]+1e-7:continue
            segment=time.perf_counter();reference=self.home_values.copy();reference[0]=updown;self.fk(reference)
            carriage=np.linalg.inv(self.fk_robot.get_transform('arm_carriage',self.fk_robot.base_link));arms=[]
            for side_index,side in enumerate(['left','right']):
                values=[];offset=1+side_index*7
                for swivel in range(-180,180,10):
                    solutions=self.analytic.solve(side_index,carriage@transforms[side],math.radians(swivel),reference[offset:offset+7])
                    assert len(solutions)<16,'解析bridge容量已满，不能声称遍历完整'
                    for solution in solutions:
                        if np.all(solution>=lower[offset:offset+7]-1e-7) and np.all(solution<=upper[offset:offset+7]+1e-7):values.append(solution)
                arms.append(np.array(values).reshape(-1,7))
            count=len(arms[0])*len(arms[1]);scanned+=count
            enumeration_ms+=(time.perf_counter()-segment)*1000;kept=0
            for offset in range(0,count,2048):
                indices=np.arange(offset,min(count,offset+2048));left=indices//len(arms[1]);right=indices%len(arms[1])
                joints=np.column_stack([np.full(len(indices),updown),arms[0][left],arms[1][right]]).astype(np.float32)
                segment=time.perf_counter();tensor=torch.tensor(joints,device='cuda').contiguous();mask=validity.mask(tensor)
                fk=checker.kinematics.compute_kinematics(JointState.from_position(tensor[None],joint_names=ACTIVE_JOINTS))
                for side in ['left','right']:
                    pose=fk.tool_poses.to_dict()[side+'_tool0'];position=torch.tensor(targets[side+'_tool0']['position'],device='cuda');quat=torch.tensor(targets[side+'_tool0']['quaternion'],device='cuda')
                    mask &= torch.linalg.vector_norm(pose.position.reshape(-1,3)-position,dim=-1)<=.002
                    mask &= (pose.quaternion.reshape(-1,4)*quat).sum(-1).abs()>=math.cos(math.radians(1)/2)
                valid=joints[mask.cpu().numpy()];kept+=len(valid)
                if len(valid):batches.append(valid)
                torch.cuda.synchronize();filter_ms+=(time.perf_counter()-segment)*1000
            counts.append(dict(updown_m=updown,paired=count,valid=kept))
        if not batches:raise RuntimeError('解析遍历接触IK没有合法组合')
        values=np.concatenate(batches);_,unique=np.unique(np.round(values,6),axis=0,return_index=True);values=values[np.sort(unique)]
        weights=np.array(joint_distance_weights(ACTIVE_JOINTS));cost=np.linalg.norm((values-self.home_values)*weights,axis=1);indices=np.argsort(cost,kind='stable')[:64];selected=values[indices]
        self.analytic_contact_report=dict(updown_step_m=.05,swivel_step_deg=10,raw_pair_count=scanned,valid_unique_count=len(values),selected_count=len(selected),cost='Home weighted L2, updown10cm=joint15deg, no joint4 penalty, no cylinder requirement at contact',enumeration_ms=enumeration_ms,collision_and_fk_filter_ms=filter_ms,total_ms=(time.perf_counter()-began)*1000,cost_min=float(cost[indices].min()),cost_max=float(cost[indices].max()),counts_by_updown=counts)
        output.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(output/f"contacts_L{boxes['left']}_R{boxes['right']}.npz",joints=selected,cost=cost[indices])
        return SimpleNamespace(js_solution=JointState.from_position(torch.tensor(selected[None],device='cuda'),joint_names=ACTIVE_JOINTS),success=torch.ones((1,len(selected)),device='cuda',dtype=torch.bool))
