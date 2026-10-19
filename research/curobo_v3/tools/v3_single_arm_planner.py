import math
import json
import time

import numpy as np
import torch
from scipy.spatial.transform import Rotation

from curobo_core.planner import CycleBlocked
from curobo.types import JointState
from v3_batched_loaded_search import GpuValidity, densify
from v3_box_rotation_cost import BoxRotationCost
from v3_conveyor_cycle_planner import ConveyorCyclePlanner
from v3_lazy_extraction_planner import LazyExtractionPlanner
from v3_multiroot_search import batched_rrt_connect_multi_goal
from v3_research_assets import atlas_path
from v3_research_assets import ROOT
from v3_wall_ik_benchmark import ACTIVE_JOINTS


class ActiveRotationCost(BoxRotationCost):
    def __init__(self, checker, start, goal, top, side):
        self.active_side = side
        super().__init__(checker, ACTIVE_JOINTS, start.float(), goal.float(), top)

    def orientations(self, values):
        return {self.active_side: super().orientations(values)[self.active_side]}


class IdleArmCost:
    def __init__(self, indices, home, rotation=None):
        self.indices = indices
        self.home = torch.tensor(home, device='cuda', dtype=torch.float32)[indices]
        self.rotation = rotation

    def edge_cost(self, starts, ends):
        deviation = .5 * ((starts[:, self.indices] - self.home).square().sum(-1) +
                          (ends[:, self.indices] - self.home).square().sum(-1))
        length = torch.linalg.vector_norm(ends - starts, dim=-1)
        motion = torch.linalg.vector_norm(ends[:, self.indices] - starts[:, self.indices], dim=-1)
        penalty = 2. * motion + 1.5 * deviation * length
        return penalty if self.rotation is None else penalty + self.rotation.edge_cost(starts, ends)


class SingleArmPlanner(LazyExtractionPlanner):
    def __init__(self, args):
        super().__init__(args)
        for box_id in range(25):
            side = 'left' if box_id % 5 >= 3 else 'right'
            label = f"单臂 {'左手' if side == 'left' else '右手'} {box_id:02d}(排{5-box_id//5}/列{5-box_id%5})"
            self.task_options.append((label, {side: box_id}))
        self.task_map = dict(self.task_options)

    @property
    def active_side(self):
        return next(iter(self.selected_task))

    @property
    def idle_indices(self):
        return list(range(8, 15)) if self.active_side == 'left' else list(range(1, 8))

    def configure_task(self, boxes):
        result = super().configure_task(boxes)
        if len(boxes) == 1:
            self.target_poses = {side + '_tool0': self.target_poses[side + '_tool0'] for side in boxes}
        return result

    def tool_matrices(self, values):
        matrices = super().tool_matrices(values)
        return {side: matrix for side, matrix in matrices.items() if side in self.selected_task}

    def free_arm_variants(self, reference):
        variants = [reference[self.idle_indices].copy(), self.home_values[self.idle_indices].copy()]
        for shoulder, elbow in [(-20, 15), (20, -15), (-35, 30), (35, -30)]:
            arm = self.home_values[self.idle_indices].copy()
            arm[1] += math.radians(shoulder)
            arm[3] += math.radians(elbow)
            variants.append(arm)
        return variants

    def full_envelope_radius(self, checker, values):
        values = torch.as_tensor(values, device='cuda', dtype=torch.float32).reshape(-1,15)
        state = checker.kinematics.compute_kinematics(JointState.from_position(values[None],joint_names=ACTIVE_JOINTS))
        spheres = state.robot_spheres.reshape(len(values),-1,4)
        footprint = torch.tensor(self.fk_robot.get_transform('base_footprint',self.fk_robot.base_link)[:2,3],device='cuda',dtype=torch.float32)
        radius = torch.linalg.vector_norm(spheres[...,:2]-footprint,dim=-1)+spheres[...,3]
        return radius.masked_fill(spheres[...,3]<=0,float('-inf')).max(dim=-1).values

    def enumerate_single(self, matrix, reference, checker, limit=64):
        side_index = 0 if self.active_side == 'left' else 1
        offset = 1 + side_index * 7
        lower, upper = [values.cpu().numpy() for values in checker.kinematics.get_joint_limits().position]
        candidates = []
        idle_variants = self.free_arm_variants(reference)
        for updown in np.round(np.arange(-1., .0001, .05), 2):
            row = np.asarray(reference, dtype=np.float64).copy()
            row[0] = updown
            self.fk(row)
            carriage = np.linalg.inv(self.fk_robot.get_transform('arm_carriage', self.fk_robot.base_link))
            for swivel_deg in range(-180, 180, 10):
                solutions = self.analytic.solve(side_index, carriage @ matrix, math.radians(swivel_deg), row[offset:offset+7])
                for solution in solutions:
                    for idle in idle_variants:
                        values = row.copy()
                        values[offset:offset+7] = solution
                        values[self.idle_indices] = idle
                        if np.all(values >= lower) and np.all(values <= upper):
                            candidates.append(values)
        if not candidates:
            self.single_enumeration_report = dict(raw_candidates=0, checked_candidates=0, accepted_candidates=0)
            return np.empty((0, 15), dtype=np.float32)
        candidates = np.unique(np.round(candidates, 7), axis=0).astype(np.float32)
        weights = np.array([math.radians(15)/.1] + [1.]*14)
        costs = np.sum(((candidates-reference)*weights)**2, axis=1) + 3*np.sum((candidates[:,self.idle_indices]-self.home_values[self.idle_indices])**2, axis=1)
        order = np.argsort(costs)
        validity = GpuValidity(checker, check_ground=True)
        accepted = []
        checked = 0
        for batch_start in range(0, len(order), 512):
            values = candidates[order[batch_start:batch_start+512]]
            checked += len(values)
            valid = validity.mask(torch.tensor(values, device='cuda')).cpu().numpy()
            accepted.extend(values[valid])
            if len(accepted) >= limit:
                break
        self.single_enumeration_report = dict(raw_candidates=len(candidates), checked_candidates=checked, accepted_candidates=min(limit,len(accepted)))
        return np.asarray(accepted[:limit], dtype=np.float32).reshape(-1, 15)

    def cartesian_single(self, contact, distances, along_normal=False):
        self.fk(contact)
        carriage = np.linalg.inv(self.fk_robot.get_transform('arm_carriage', self.fk_robot.base_link))
        target = self.fk_robot.get_transform(self.active_side+'_tool0', self.fk_robot.base_link).copy()
        side_index = 0 if self.active_side == 'left' else 1
        offset = 1 + side_index*7
        swivel = self.analytic.swivel(side_index, contact[offset:offset+7])
        rows = [np.asarray(contact, dtype=np.float64)]
        for distance in distances:
            pose = target.copy()
            pose[:3,3] -= pose[:3,2]*distance if along_normal else np.array([distance,0.,0.])
            solutions = self.analytic.solve(side_index, carriage@pose, swivel, rows[-1][offset:offset+7])
            if not len(solutions):
                return None, '工作臂固定Updown/冗余角解析无解'
            row = rows[-1].copy()
            row[offset:offset+7] = solutions[np.argmin(np.sum((solutions-rows[-1][offset:offset+7])**2,axis=1))]
            rows.append(row)
        dense = np.asarray(densify(np.asarray(rows)))
        for row in dense:
            self.fk(row)
            pose = self.fk_robot.get_transform(self.active_side+'_tool0', self.fk_robot.base_link)
            delta = pose[:3,3]-target[:3,3]
            direction = target[:3,2] if along_normal else np.array([1.,0.,0.])
            lateral = np.linalg.norm(delta-direction*np.dot(delta,direction))
            angle = Rotation.from_matrix(pose[:3,:3].T@target[:3,:3]).magnitude()
            if lateral>.002 or angle>math.radians(1):
                return None, '单臂笛卡尔密化直线或朝向验收失败'
        return dense, dict(fixed_updown_m=float(contact[0]), fixed_swivel_rad=swivel)

    def single_validity(self, checker, start, goal, rotation=False):
        validity = GpuValidity(checker, check_ground=True)
        metric = ActiveRotationCost(checker, torch.tensor(start,device='cuda',dtype=torch.float32), torch.tensor(goal,device='cuda',dtype=torch.float32), self.top_grasp, self.active_side) if rotation else None
        validity.box_rotation_cost = IdleArmCost(self.idle_indices, self.home_values, metric)
        idle = self.idle_indices
        home = torch.tensor(self.home_values[idle], device='cuda')
        def sample_transform(samples, generator, lower, upper):
            near = home + torch.randn((len(samples),7),device='cuda',generator=generator)*.18
            selected = torch.rand((len(samples),1),device='cuda',generator=generator)<.8
            samples[:,idle] = torch.where(selected,near,samples[:,idle])
            return torch.clamp(samples,lower,upper)
        validity.sample_transform = sample_transform
        return validity

    def single_path(self, start, goals, checker, rotation=False, strict=None, budget=2., seed=20261015):
        goals = np.asarray(goals).reshape(-1,15)
        validity = self.single_validity(checker,start,goals[0],rotation)
        if strict is not None:
            validity.evaluate = strict.evaluate
        lower,upper = checker.kinematics.get_joint_limits().position
        path,stats = batched_rrt_connect_multi_goal(torch.tensor(start,device='cuda',dtype=torch.float32),torch.tensor(goals,device='cuda',dtype=torch.float32),lower+1e-5,upper-1e-5,validity,budget,seed,stop_at_first_valid=True)
        if path is None:
            return None,stats
        dense = np.asarray(densify(path))
        if not validity.mask(torch.tensor(dense,device='cuda',dtype=torch.float32)).all():
            return None,dict(failure='单臂完整15轴密化碰撞验收失败')
        return dense,stats

    def transport_path(self, start, goals, checker, candidates):
        return self.single_path(start,goals,checker,True)

    def joint_plan(self, start, target, checker, validity, seed):
        if len(self.selected_task)==2:
            return super().joint_plan(start,target,checker,validity,seed)
        return self.single_path(start,np.asarray(target).reshape(1,15),checker,strict=validity)

    def plan_conveyor_target(self, start, targets, checker, validity, solver, report, append):
        if len(self.selected_task)==2:
            return super().plan_conveyor_target(start,targets,checker,validity,solver,report,append)
        candidates = self.enumerate_single(targets[self.active_side],start,checker)
        report['conveyor_goal_ik'] = dict(mode='single active TCP analytic updown/swivel enumeration; idle arm free in 15D search',collision_valid=len(candidates))
        if not len(candidates):
            raise CycleBlocked('龙头车放置目标IK','单臂目标没有合法候选',report)
        path,stats = self.single_path(start,candidates,checker)
        report['conveyor_place_search_stats'] = stats
        if path is None:
            raise CycleBlocked('龙头车放置RRT','单臂15轴联合搜索超时',report)
        names = report['joint_names']
        frames = np.repeat(np.asarray(report['frames'][-1])[None],len(path),axis=0)
        frames[:,[names.index(name) for name in ACTIVE_JOINTS]] = path
        append(frames,'conveyor_place',True)
        return path[-1]

    def _full_cycle(self, boxes, progress=lambda text:None):
        if len(boxes)==2:
            return super()._full_cycle(boxes,progress)
        began = time.perf_counter()
        names = list(self.mobile_robot['kinematics']['cspace']['joint_names'])
        report = dict(success=False,task=boxes,joint_names=names,frames=[],phases=[],payload=[],attempts=[],planner='single_arm_15d_connect',timing_ms={},single_arm=True,active_side=self.active_side,idle_arm_policy='15D joint search; Home deviation and motion soft costs; 80% idle sampling near Home, 20% corridor/global')
        self.last_partial = report
        timing = report['timing_ms']
        def append(rows,phase,payload):
            for row in rows:
                mapping = dict(zip(ACTIVE_JOINTS,row)) if len(row)==15 else dict(zip(names,row))
                report['frames'].append([float(mapping.get(name,0)) for name in names])
                report['phases'].append(phase)
                report['payload'].append(payload)
        empty = self.checker(boxes)
        loaded = self.checker(boxes,payload=True)
        empty_valid = GpuValidity(empty,check_ground=True)
        loaded_valid = GpuValidity(loaded,check_ground=True)
        append([self.home_values],'home',False)
        if not empty_valid.mask(torch.tensor(self.home_values[None],device='cuda')).all():
            raise CycleBlocked('Home','单臂初始姿态碰撞',report)
        progress('单臂：枚举工作臂接触解及空闲臂收拢候选')
        started = time.perf_counter()
        target = self.contact_poses_for_task(boxes)[self.active_side+'_tool0']
        matrix = np.eye(4)
        matrix[:3,:3] = Rotation.from_quat(np.roll(target['quaternion'],-1)).as_matrix()
        matrix[:3,3] = target['position']
        contacts = self.enumerate_single(matrix,self.home_values,empty)
        timing['contact_candidates_ms'] = (time.perf_counter()-started)*1000
        report['contact_ik_count'] = len(contacts)
        report['contact_candidates'] = contacts.tolist()
        report['single_contact_enumeration'] = dict(self.single_enumeration_report)
        if not len(contacts):
            raise CycleBlocked('接触IK','单臂接触无合法候选',report)
        selected = None
        timing['extraction_and_approach_ms'] = 0.
        for index,contact in enumerate(contacts):
            progress(f'单臂候选{index+1}/{len(contacts)}：抽离验收及Home到预接触')
            started = time.perf_counter()
            extracted,detail = self.cartesian_single(contact,np.linspace(.01,.35,35))
            if extracted is None or not empty_valid.mask(torch.tensor(extracted,device='cuda',dtype=torch.float32)).all() or not loaded_valid.mask(torch.tensor(extracted[-1:],device='cuda',dtype=torch.float32)).all():
                report['attempts'].append(dict(candidate=index+1,rejected='抽离运动学/机器人碰撞/携箱终点碰撞',detail=detail))
                timing['extraction_and_approach_ms'] += (time.perf_counter()-started)*1000
                continue
            backwards,detail = self.cartesian_single(contact,np.linspace(.005,.03,6),True)
            if backwards is None or not empty_valid.mask(torch.tensor(backwards,device='cuda',dtype=torch.float32)).all():
                timing['extraction_and_approach_ms'] += (time.perf_counter()-started)*1000
                continue
            approach,stats = self.single_path(self.home_values,[backwards[-1]],empty,strict=self.target_validity(empty))
            timing['extraction_and_approach_ms'] += (time.perf_counter()-started)*1000
            if approach is not None:
                selected=(contact,extracted,backwards,approach)
                report['selected_candidate']=index+1
                break
        if selected is None:
            raise CycleBlocked('抽离与到位','单臂64候选未找到通过组合',report)
        contact,extracted,backwards,approach=selected
        append(approach,'approach',False)
        append(backwards[::-1],'contact_approach',False)
        append([contact],'attach',True)
        append(extracted,'extract',True)
        progress('单臂：匹配携箱放置终点并进行15轴联合搜索')
        started=time.perf_counter()
        atlas=np.load(atlas_path(self))['joints']
        candidates=[atlas.copy()]
        for idle in self.free_arm_variants(extracted[-1]):
            values=atlas.copy()
            values[:,self.idle_indices]=idle
            candidates.append(values)
        candidates=np.unique(np.round(np.concatenate(candidates),6),axis=0).astype(np.float32)
        mask=[]
        policy=json.loads((ROOT/'config/placement_task_policy.json').read_text())
        radius_limit=policy['side']['placement_radius_m']
        for offset in range(0,len(candidates),512):
            batch=torch.tensor(candidates[offset:offset+512],device='cuda')
            radius=self.full_envelope_radius(loaded,batch)
            mask.extend((loaded_valid.mask(batch)&(radius<=radius_limit)).cpu().tolist())
        report['transport_endpoint_filter']=dict(candidate_count=len(candidates),full_envelope_radius_limit_m=radius_limit,passed=int(np.sum(mask)),scope='whole robot including idle arm and active payload, distance to base_footprint axis')
        candidates=candidates[np.asarray(mask)]
        if not len(candidates):
            raise CycleBlocked('运输终点','单臂携箱放置无合法候选',report)
        weights=np.array([math.radians(15)/.1]+[1.]*14)
        order=np.argsort(np.linalg.norm((candidates-extracted[-1])*weights,axis=1))
        goals=candidates[order[:64]]
        transport,stats=self.transport_path(extracted[-1],goals,loaded,candidates)
        timing['transport_ms']=(time.perf_counter()-started)*1000
        if transport is None:
            raise CycleBlocked('运输RRT','单臂15轴携箱搜索超时',report)
        final_radius=float(self.full_envelope_radius(loaded,transport[-1:]).item())
        report['transport_endpoint_envelope']=dict(radius_m=final_radius,limit_m=radius_limit,passed=final_radius<=radius_limit,includes_idle_arm=True,includes_payload=True)
        if final_radius>radius_limit:
            raise CycleBlocked('运输终点圆柱验收','整机及携箱包络超过允许半径',report)
        report['transport_rotation_metrics']=ActiveRotationCost(loaded,torch.tensor(transport[0],device='cuda'),torch.tensor(transport[-1],device='cuda'),self.top_grasp,self.active_side).metrics(torch.tensor(transport,device='cuda',dtype=torch.float32))
        report['transport_search_stats']=stats
        append(transport,'transport',True)
        self.conveyor_suffix(boxes,report,append,progress)
        values=np.asarray(report['frames'])[:,[names.index(name) for name in ACTIVE_JOINTS]]
        report['final_home_error']=float(np.abs(values[-1]-self.home_values).max())
        report['idle_arm_motion']=dict(max_home_deviation_deg=float(np.rad2deg(np.abs(values[:,self.idle_indices]-self.home_values[self.idle_indices])).max()),cumulative_joint_motion_deg=float(np.rad2deg(np.abs(np.diff(values[:,self.idle_indices],axis=0))).sum()))
        report.update(success=True,total_ms=(time.perf_counter()-began)*1000,rotation_quality_hard_gate=False,box_rotation_cost_enabled=True)
        return report

    def full_cycle(self, boxes, progress=lambda text:None):
        if len(boxes)==2:
            return super().full_cycle(boxes,progress)
        self.checker_resource_events.clear()
        result=ConveyorCyclePlanner.full_cycle(self,boxes,progress)
        result['checker_resource_events']=list(self.checker_resource_events)
        return result
