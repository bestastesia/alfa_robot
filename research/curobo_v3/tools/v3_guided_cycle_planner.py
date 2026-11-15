import copy
import json
import math
from pathlib import Path
import time
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import torch

from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.types import GoalToolPose, JointState, Pose
from v3_adaptive_raised_cycle_planner import AdaptiveRaisedCyclePlanner
from v3_batched_loaded_search import densify
from v3_multiroot_search import batched_rrt_connect_multi_goal
from v3_route_guided_compare import RouteGuidance, TcpRouteCost
from v3_route_ik_benchmark import sample_route
from v3_tangent_sphere_demo import plan as proxy_plan
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def guidance_summary(events):
    lines=['','|搜索阶段|参考ms|LM ms|筛选ms|搜索ms|合法先验|覆盖|结果|','|--|--:|--:|--:|--:|--:|--:|--|']
    for entry in events:
        lines.append(f"|{entry['phase']}|{entry.get('reference_ms',0):.1f}|{entry.get('lm_ms',0):.1f}|{entry.get('filter_ms',0):.1f}|{entry.get('search_ms',0):.1f}|{entry.get('legal_nodes',0)}|{entry.get('covered_points',0)}/{entry.get('point_count',128)}|{'通过' if entry['success'] else entry.get('failure','搜索失败')}|")
    for entry in events:
        if entry.get('actual_rotation_deg'):
            rotations=entry['actual_rotation_deg']
            lines += ['',f"{entry['phase']}实际tool累计转：左{rotations['left']:.1f}°／右{rotations['right']:.1f}°；参考外部场景余量{entry.get('reference_margin_m',0)*1000:.2f}mm。"]
    return '\n'.join(lines)


class SumCosts:
    def __init__(self, previous, reference):
        self.previous, self.reference = previous, reference

    def edge_cost(self, starts, ends):
        value = self.reference.edge_cost(starts, ends)
        return value if self.previous is None else value+self.previous.edge_cost(starts, ends)


class GuidedCyclePlanner(AdaptiveRaisedCyclePlanner):
    def __init__(self, args):
        super().__init__(args)
        self.guidance_events = []
        self.guidance_solver = None
        self.guidance_goal = None
        self.guidance_phase = 'joint_search'
        source = Path(__file__).resolve().parents[1] / 'config/forward_precontact_proxy.json'
        self.proxy_parameters = json.loads(source.read_text())

    def collision_checker(self, boxes, payload=False, mobile=False):
        checker = super().collision_checker(boxes, payload, mobile)
        checker.guidance_scene = self.scene(boxes, mobile=mobile)
        checker.guidance_payload = payload
        return checker

    def _tool_poses(self, checker, values):
        state = checker.kinematics.compute_kinematics(JointState.from_position(values.reshape(1, -1), joint_names=ACTIVE_JOINTS))
        output = {}
        for side in ('left', 'right'):
            pose = state.tool_poses.to_dict()[side+'_tool0']
            matrix = np.eye(4)
            matrix[:3, 3] = pose.position.reshape(-1, 3)[0].detach().cpu().numpy()
            matrix[:3, :3] = Rotation.from_quat(np.roll(pose.quaternion.reshape(-1, 4)[0].detach().cpu().numpy(), -1)).as_matrix()
            output[side] = matrix.tolist()
        return output

    def _reference(self, checker, validity, start, goal):
        scene = checker.guidance_scene
        if scene.mesh:
            raise ValueError('指导路线暂不支持未转换的场景网格')
        objects = [dict(name=item.name, dimensions=list(item.dims), position=list(item.pose[:3]), quaternion_wxyz=list(item.pose[3:])) for item in scene.cuboid or []]
        layer = getattr(checker, 'base_cuboid_layer', None)
        if layer is not None and hasattr(layer, 'scene'):
            objects.extend(dict(name=item.name, dimensions=list(item.dims), position=list(item.pose[:3]), quaternion_wxyz=list(item.pose[3:])) for item in layer.scene.cuboid or [])
        if hasattr(validity, 'targets'):
            objects.extend(dict(name='precontact_target_'+side, dimensions=list(item['dimensions']), position=list(item['center']), quaternion_wxyz=[1.,0.,0.,0.]) for side,item in validity.targets.items())
        snapshot = dict(tcp_world=self._tool_poses(checker, start), targets=self._tool_poses(checker, goal))
        params = self.proxy_parameters
        reference = proxy_plan(snapshot, objects, params['radius'], params['margin'], 0., params['mode']=='forward', params['normal_limit_m_s2'])
        samples, progress = sample_route(dict(tracks=reference['paths']), 128)
        return reference, samples, progress

    def _lm(self, samples, start, validity):
        setup_ms = 0.
        if self.guidance_solver is None:
            began = time.perf_counter()
            config = InverseKinematicsCfg.create(robot=copy.deepcopy(self.robot), scene_model=None,
                self_collision_check=False, num_seeds=16, seed_solver_num_seeds=16, max_batch_size=64,
                position_tolerance=.002, orientation_tolerance=math.radians(1), use_cuda_graph=True,
                override_iters_for_multi_link_ik=160, random_seed=20261021)
            config.exit_early = True
            config.exit_early_batch_success_threshold = 0.
            self.guidance_solver = InverseKinematics(config)
            setup_ms = (time.perf_counter()-began)*1000
        batches = []
        point_ids = []
        state = JointState.from_position(start.reshape(1, -1).repeat(64, 1), joint_names=ACTIVE_JOINTS)
        torch.cuda.synchronize()
        began = time.perf_counter()
        for offset in (0, 64):
            goals = GoalToolPose.from_poses({side+'_tool0': Pose(
                position=torch.tensor(matrices[offset:offset+64, :3, 3], device='cuda', dtype=torch.float32),
                quaternion=torch.tensor(np.roll(Rotation.from_matrix(matrices[offset:offset+64, :3, :3]).as_quat(), 1, axis=1), device='cuda', dtype=torch.float32))
                for side,matrices in samples.items()}, ordered_tool_frames=['left_tool0','right_tool0'])
            if self.guidance_goal is None:
                self.guidance_goal = copy.deepcopy(goals)
            else:
                self.guidance_goal.position.copy_(goals.position)
                self.guidance_goal.quaternion.copy_(goals.quaternion)
            with torch.enable_grad():
                solved = self.guidance_solver.solve_pose(self.guidance_goal, current_state=state, return_seeds=16)
            indices = [solved.js_solution.joint_names.index(name) for name in ACTIVE_JOINTS]
            batches.append(solved.js_solution.position.reshape(-1, len(solved.js_solution.joint_names))[:, indices].detach().clone())
            point_ids.extend(np.repeat(np.arange(offset, offset+64),16).tolist())
        rows = torch.cat(batches)
        identifiers = np.asarray(point_ids)
        torch.cuda.synchronize()
        solve_ms = (time.perf_counter()-began)*1000
        began = time.perf_counter()
        masks = []
        diagnostic_collision=[]
        for offset in range(0, len(rows), 512):
            batch = rows[offset:offset+512]
            valid = validity.mask(batch)
            collision_valid=valid.clone()
            lower, upper = validity.checker.kinematics.get_joint_limits().position
            valid &= ((batch>=lower)&(batch<=upper)).all(-1)
            state_fk = validity.checker.kinematics.compute_kinematics(JointState.from_position(batch[None], joint_names=ACTIVE_JOINTS))
            for side in ('left', 'right'):
                targets = samples[side][identifiers[offset:offset+512]]
                pose = state_fk.tool_poses.to_dict()[side+'_tool0']
                position = torch.tensor(targets[:, :3, 3], device='cuda', dtype=torch.float32)
                quaternion = torch.tensor(np.roll(Rotation.from_matrix(targets[:, :3, :3]).as_quat(),1,axis=1),device='cuda',dtype=torch.float32)
                valid &= torch.linalg.vector_norm(pose.position.reshape(-1,3)-position,dim=-1)<=.002
                valid &= (pose.quaternion.reshape(-1,4)*quaternion).sum(-1).abs()>=math.cos(math.radians(1)/2)
            masks.extend(valid.cpu().tolist())
            if getattr(self,'capture_guidance_nodes',False):
                diagnostic_collision.extend(collision_valid.cpu().tolist())
        mask = np.asarray(masks)
        if getattr(self,'capture_guidance_nodes',False):
            self.guidance_events[-1]['node_diagnostics']=dict(joints=rows.detach().cpu().tolist(),point_indices=identifiers.tolist(),
                legal=mask.tolist(),collision_valid=diagnostic_collision,
                targets={side:matrices.tolist() for side,matrices in samples.items()})
        torch.cuda.synchronize()
        return rows[torch.tensor(mask,device='cuda')], identifiers[mask], dict(solver_setup_ms=setup_ms,
            lm_ms=solve_ms, filter_ms=(time.perf_counter()-began)*1000, legal_nodes=int(mask.sum()),
            covered_points=int(len(np.unique(identifiers[mask]))), point_count=128, batch_points=64, seeds_per_point=16)

    def _guided_search(self, start, goals, lower, upper, validity, budget, seed, **kwargs):
        started = time.perf_counter()
        event = dict(phase=self.guidance_phase, payload=getattr(validity.checker,'guidance_payload',False),
            roots=int(start.reshape(-1,15).shape[0]), goals=len(goals), success=False)
        self.guidance_events.append(event)
        roots = start.reshape(-1,15)
        distances = torch.cdist(roots*validity.weights, goals*validity.weights)
        pair = int(distances.argmin())
        representative_start = roots[pair//len(goals)]
        representative_goal = goals[pair%len(goals)]
        old_transform = getattr(validity,'post_sampling_transform', None)
        old_cost = getattr(validity,'box_rotation_cost', None)
        try:
            reference, samples, progress = self._reference(validity.checker,validity,representative_start,representative_goal)
            event.update(reference_ms=reference['total_ms'], reference_tracks=reference['paths'])
            rows, identifiers, stats = self._lm(samples, representative_start, validity)
            event.update(stats)
            if not len(rows):
                event['failure']='参考路线没有通过整机碰撞及Pose验收的LM先验'
                return None, dict(guidance=event)
            guidance = RouteGuidance(rows.cpu().numpy(),identifiers,representative_start,goals,validity.weights)
            validity.post_sampling_transform = guidance
            validity.box_rotation_cost = SumCosts(old_cost,TcpRouteCost(validity.checker,samples,validity.weights))
            began = time.perf_counter()
            path, search = batched_rrt_connect_multi_goal(start,goals,lower,upper,validity,budget,seed,
                informed_sampling=True,stop_at_first_valid=True)
            torch.cuda.synchronize()
            event.update(search_ms=(time.perf_counter()-began)*1000, search=search, success=path is not None)
            if path is not None:
                dense = np.asarray(densify(path))
                if not bool(validity.mask(torch.tensor(dense,device='cuda',dtype=torch.float32)).all()):
                    event.update(success=False,failure='首解再次密化后碰撞')
                    return None,dict(guidance=event)
                event['frames']=len(dense)
            search['guidance_event_index']=len(self.guidance_events)-1
            return path,search
        except (ValueError,RuntimeError) as error:
            event['failure']=str(error)
            return None,dict(guidance=event)
        finally:
            validity.post_sampling_transform=old_transform
            validity.box_rotation_cost=old_cost
            event['total_ms']=(time.perf_counter()-started)*1000

    def search_path(self,*args,**kwargs):
        return self._guided_search(*args,**kwargs)

    def run_single_search(self,*args,**kwargs):
        return self._guided_search(*args,**kwargs)

    def joint_plan(self,*args,**kwargs):
        previous = self.guidance_phase
        self.guidance_phase = 'return_home' if np.allclose(args[1],self.home_values,atol=1e-5) else 'approach'
        try:
            return super().joint_plan(*args,**kwargs)
        finally:
            self.guidance_phase=previous

    def single_path(self,*args,**kwargs):
        previous=self.guidance_phase
        if self._placement_search:
            self.guidance_phase='conveyor_place'
        elif kwargs.get('rotation',False) or (len(args)>3 and args[3]):
            self.guidance_phase='transport'
        elif np.allclose(np.asarray(args[1]).reshape(-1,15)[0],self.home_values,atol=1e-5):
            self.guidance_phase='return_home'
        else:
            self.guidance_phase='approach'
        try:
            return super().single_path(*args,**kwargs)
        finally:
            self.guidance_phase=previous

    def plan_conveyor_target(self,*args,**kwargs):
        previous=self.guidance_phase
        self.guidance_phase='conveyor_place'
        try:
            return super().plan_conveyor_target(*args,**kwargs)
        finally:
            self.guidance_phase=previous

    def full_cycle(self,boxes,progress=lambda text:None):
        self.configure_task(boxes)
        self.guidance_events=[]
        self.guidance_phase='transport'
        with patch('v3_multiroot_full_cycle_planner.batched_rrt_connect_multi_goal',self._guided_search):
            result=super().full_cycle(boxes,progress)
        result['guided_search_events']=copy.deepcopy(self.guidance_events)
        result['guided_search_policy']='All four joint-search stages: proxy TCP route ->128 poses ->LM16/batch64 ->native full collision+FK ->first finely valid Informed Connect; Cartesian approach/extraction unchanged'
        return result
