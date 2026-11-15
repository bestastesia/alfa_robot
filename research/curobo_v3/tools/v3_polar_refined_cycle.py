import copy
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation
import torch
from curobo.types import JointState
from curobo_core.planner import CycleBlocked

from v3_current_urdf_forward import CurrentUrdfForwardPlanner, current_assets
from v3_shoulder_wrist_reference import ArmReferenceGeometry
from v3_spherical_wrist_route import WristCurve
from curobo_core.new_warp_reference import PolarCurve, polar_plan, smooth
from curobo_core.new_warp_prior import enumerate_smooth, nearest_chain
from curobo_core.new_warp_objective import FixedLiftObjective, path_metrics
from v3_endpoint_guided_ik import pose_filter
from v3_batched_loaded_search import GpuValidity, densify
from v3_multiroot_search import batched_rrt_connect_multi_goal
from v3_route_guided_compare import RouteGuidance, TcpRouteCost
from v3_guided_cycle_planner import SumCosts
from v3_serial_endpoint_planner import nearest_endpoint
from v3_wall_ik_benchmark import ACTIVE_JOINTS


ROOT = Path(__file__).resolve().parents[1]


def clock():
    torch.cuda.synchronize()
    return time.perf_counter()


class PolarRefinedCyclePlanner(CurrentUrdfForwardPlanner):
    conveyor_release_drop_m = .10
    pipeline_label = '球面＋解析＋整段30轮＋Informed Connect'
    pipeline_description = '六段统一：球面软参考（允许碰撞/缺口）→128点解析→整段30轮→合法先验引导Informed Connect→0.25°加密复核。参考不是执行轨迹，不因参考碰撞提前停止；最终路径仍严格验碰/限位/端点。'
    warmup_description = '当前确认零位与191mm TCP；已预热碰撞检查与30轮联合调整。当前流程的路径先验不运行LM。'
    result_directory = ROOT / 'generated/polar_refined_full_cycle_20261027/live'

    def __init__(self, args=None, reference_source=None):
        super().__init__(args or current_assets(), reference_source)
        self.placement_clearance_m -= self.conveyor_release_drop_m
        self.arm_geometry = ArmReferenceGeometry(self.args.urdf, self.wrist_geometry)

    def warmup_resources(self):
        if getattr(self, 'warmup_report', {}).get('complete'):
            return self.warmup_report
        began = clock()
        checker = self.checker(self.selected_task)
        validity = GpuValidity(checker, check_ground=True)
        home = np.asarray(self.home_values)
        for count in (128, 512):
            validity.mask(torch.tensor(np.tile(home, (count, 1)), device='cuda', dtype=torch.float32))
        poses = self._tool_poses(checker, torch.tensor(home, device='cuda', dtype=torch.float32))
        samples = {side: np.tile(matrix, (128, 1, 1)) for side, matrix in poses.items()}
        self._joint_adjust(checker, np.tile(home, (128, 1)), samples, home, home, np.full(128, home[0]))
        self.warmup_report = dict(complete=True, lm_used=False, wall_ms=(clock() - began) * 1000,
                                 joint_adjustment_iterations=30, caches_planning_results=False)
        return self.warmup_report

    def _joint_adjust(self, checker, chain, samples, start, goal, lift):
        began = clock()
        document = dict(start=start, goal=goal, methods={'lm': {'endpoint_soft': {'node_diagnostics': {'targets': samples}}}})
        objective = FixedLiftObjective(checker, document, lift)
        basis = objective.basis.detach().cpu().numpy()
        values = np.linalg.lstsq(basis[:, 1:-1], chain - basis[:, :1] * start - basis[:, -1:] * goal, rcond=None)[0]
        lower, upper = [item.cpu().numpy() for item in checker.kinematics.get_joint_limits().position]
        values = np.clip(values, lower, upper)
        setup_ms = (clock() - began) * 1000
        began = clock()
        with torch.enable_grad():
            interior = torch.tensor(values[None], device='cuda', dtype=torch.float32, requires_grad=True)
            optimizer = torch.optim.Adam([interior], lr=.018)
            for iteration in range(30):
                cost = sum(objective.terms(interior).values()).sum()
                optimizer.zero_grad()
                cost.backward()
                torch.nn.utils.clip_grad_norm_([interior], 30.)
                optimizer.step()
                with torch.no_grad():
                    interior.clamp_(objective.lower, objective.upper)
            rows = objective.decode(interior)[0].detach().clone()
        return rows, dict(joint_setup_ms=setup_ms, joint_adjustment_ms=(clock() - began) * 1000, joint_iterations=30)

    def _polar_reference(self, checker, validity, start, goal):
        began = clock()
        initial = self._tool_poses(checker, torch.tensor(start, device='cuda', dtype=torch.float32))
        targets = self._tool_poses(checker, torch.tensor(goal, device='cuda', dtype=torch.float32))
        scene = checker.guidance_scene
        if scene.mesh:
            raise ValueError('当前球面引导仅支持已转换的长方体场景')
        objects = [dict(name=item.name, dimensions=list(item.dims), position=list(item.pose[:3]), quaternion_wxyz=list(item.pose[3:])) for item in scene.cuboid or [] if not item.name.startswith('robot_internal/')]
        strict = validity if hasattr(validity, 'targets') else getattr(validity.evaluate, '__self__', None)
        if strict is not None and hasattr(strict, 'targets'):
            objects.extend(dict(name='precontact_target_' + side, dimensions=list(item['dimensions']), position=list(item['center']), quaternion_wxyz=np.roll(Rotation.from_matrix(np.asarray(item.get('rotation', np.eye(3)))).as_quat(), 1).tolist()) for side, item in strict.targets.items())
        kinematics = checker.kinematics.compute_kinematics(JointState.from_position(torch.tensor(start[None], device='cuda', dtype=torch.float32), joint_names=ACTIVE_JOINTS))
        spheres = kinematics.robot_spheres.reshape(-1, 4).detach().cpu().numpy()
        models, originals = {}, {}
        for side in ('left', 'right'):
            indices = checker.kinematics.config.kinematics_config.get_sphere_index_from_link_name(side + '_link7').cpu().numpy()
            selected = spheres[indices]
            selected = selected[selected[:, 3] > 0]
            transform = np.asarray(initial[side])
            models[side] = dict(centers_tool=(selected[:, :3] - transform[:3, 3]) @ transform[:3, :3], radii=selected[:, 3])
            originals[side] = WristCurve(initial[side], targets[side], self.wrist_geometry['arms'][side]['center_tool'])
        context = self.arm_geometry.context(start, goal, ACTIVE_JOINTS)
        case = dict(start=start, goal=goal, event=dict(review_scene_objects=objects))
        components = (case, context, originals, models, None, (-1., 0.))
        route = polar_plan(None, str(self.selected_task), self.guidance_phase, components=components, allow_colliding_reference=True)
        if not route['guidance_available']:
            if getattr(self, 'capture_reference_failures', False):
                self.guidance_events[-1]['reference_failure_snapshot'] = dict(
                    start=start.tolist(), goal=goal.tolist(), objects=objects,
                    context={side: {key: value.tolist() if isinstance(value, np.ndarray) else value for key, value in entry.items()} for side, entry in context.items()},
                    originals={side: curve.serialize() for side, curve in originals.items()},
                    models={side: {key: value.tolist() if isinstance(value, np.ndarray) else value for key, value in entry.items()} for side, entry in models.items()},
                    checked_candidates=route['checked_candidates'])
            raise ValueError('球面参考未生成有限几何数据')
        parameter = np.linspace(0., 1., 128)
        samples = {}
        paths = {}
        ratio = route['lift_ratio']
        for side in ('left', 'right'):
            track = route['paths'][side]
            curve = PolarCurve(originals[side], context[side], track['strength'], track['bend'], ratio)
            states = curve.evaluate(parameter)
            matrices = np.tile(np.eye(4), (128, 1, 1))
            matrices[:, :3, 3] = states['positions']
            matrices[:, :3, :3] = states['rotations'].as_matrix()
            samples[side] = matrices
            paths[side] = dict(positions=states['positions'].tolist(), centers=states['wrists'].tolist(), quaternions_wxyz=np.roll(states['rotations'].as_quat(), 1, axis=1).tolist())
        lift = start[0] + (goal[0] - start[0]) * smooth(ratio * parameter / (1 + (ratio - 1) * parameter))
        return samples, lift, dict(reference_ms=(clock() - began) * 1000, reference_tracks=paths,
            reference_kind='polar_shared_lift_smooth_radius', review_scene_objects=objects, reference_margin_m=0.,
            reference_collision_free=route['accepted'], reference_used_soft_fallback=route['used_colliding_reference'],
            reference_is_execution_path=False,
            reference_scene_scope='external collision is guidance preference only; not a prerequisite for RRT')

    def _guided_search(self, start, goals, lower, upper, validity, budget, seed, **kwargs):
        began = clock()
        roots = start.reshape(-1, 15)
        if len(roots) != 1:
            raise ValueError('当前串行流程只允许一个起点')
        initial = roots[0].detach().cpu().numpy()
        goal, selected, distance = nearest_endpoint(goals.detach().cpu().numpy(), initial)
        event = dict(phase=self.guidance_phase, success=False, roots=1, goals=1, payload=getattr(validity.checker, 'guidance_payload', False),
                     pipeline=self.pipeline_label, point_count=128, lm_used=False, lm_ms=0., solver_setup_ms=0.)
        self.guidance_events.append(event)
        old_transform = getattr(validity, 'post_sampling_transform', None)
        old_cost = getattr(validity, 'box_rotation_cost', None)
        validity.search_base_margin_m = .002
        try:
            event['step'] = 'endpoint_check'
            if not bool(validity.evaluate(torch.tensor(np.stack([initial, goal]), device='cuda', dtype=torch.float32), base_margin_m=.002)[0].all()):
                raise ValueError('固定起终点未通过底盘2mm余量验收')
            event['step'] = 'polar_reference'
            samples, lift, reference = self._polar_reference(validity.checker, validity, initial, goal)
            event.update(reference)
            event['step'] = 'analytic_prior'
            case = dict(start=initial, goal=goal)
            enumerated = enumerate_smooth(SimpleNamespace(arm=self.arm_geometry), self.analytic, case, samples, lift)
            event['analytic_ms'] = enumerated['statistics']['enumeration_ms']
            rows = torch.tensor(enumerated['rows'], device='cuda', dtype=torch.float32)
            filter_began = clock()
            legal, collision = pose_filter(validity.checker, validity, rows, enumerated['ids'], samples)
            legal = np.asarray(legal, dtype=bool)
            event['analytic_filter_ms'] = (clock() - filter_began) * 1000
            chain, present = nearest_chain(enumerated['rows'], enumerated['ids'], legal, initial, goal)
            chain[:, 0] = lift
            event['analytic_covered_points'] = int(present.sum())
            event['analytic_missing_points'] = np.flatnonzero(~present).tolist()
            event['step'] = 'joint_adjustment'
            adjusted, adjustment = self._joint_adjust(validity.checker, chain, samples, initial, goal, lift)
            event.update(adjustment)
            filter_began = clock()
            mask = validity.mask(adjusted)
            mask &= ((adjusted >= lower - 1e-6) & (adjusted <= upper + 1e-6)).all(-1)
            ids = np.flatnonzero(mask.cpu().numpy())
            selected_rows = adjusted[mask]
            event.update(filter_ms=event['analytic_filter_ms'] + (clock() - filter_began) * 1000,
                         legal_nodes=len(ids), covered_points=len(ids), adjusted_joint_rows=adjusted.cpu().tolist(),
                         adjusted_legal_mask=mask.cpu().tolist())
            fixed = torch.tensor(goal[None], device='cuda', dtype=torch.float32)
            validity.post_sampling_transform = RouteGuidance(selected_rows.cpu().numpy(), ids, roots[0], fixed, validity.weights) if len(ids) else old_transform
            event['rrt_without_legal_prior'] = not len(ids)
            validity.box_rotation_cost = SumCosts(old_cost, TcpRouteCost(validity.checker, samples, validity.weights))
            event['step'] = 'rrt_search'
            search_began = clock()
            path, statistics = batched_rrt_connect_multi_goal(roots[0], fixed, lower, upper, validity, budget, seed,
                informed_sampling=True, stop_at_first_valid=True)
            event.update(search_ms=(clock() - search_began) * 1000, search=statistics)
            if path is None:
                raise ValueError('已找到粗连接，但未通过细验' if statistics.get('solutions_found', 0) else '搜索预算内未找到粗连接')
            dense = np.asarray(densify(path, maximum_weighted_step=np.deg2rad(.25)), dtype=np.float32)
            event['step'] = 'fine_validation'
            fine_began = clock()
            checks = []
            for offset in range(0, len(dense), 512):
                checks.extend(validity.mask(torch.tensor(dense[offset:offset + 512], device='cuda')).cpu().tolist())
            endpoint_valid, _ = pose_filter(validity.checker, validity, torch.tensor(dense[[0, -1]], device='cuda'), np.array([0, 127]), samples)
            event['fine_validation_ms'] = (clock() - fine_began) * 1000
            if not all(checks) or not endpoint_valid.all():
                raise ValueError('RRT已找到连接，但未通过0.25°加密细验或端点Pose复核')
            metrics = path_metrics(validity.checker, dense, samples)
            event.update(success=True, frames=len(dense), actual_tracks=metrics['tracks'],
                         actual_rotation_deg={side: track['total_rotation_deg'] for side, track in metrics['tracks'].items()},
                         search_base_margin_m=.002)
            statistics['guidance_event_index'] = len(self.guidance_events) - 1
            return dense, statistics
        except (ValueError, RuntimeError) as error:
            event['failure'] = str(error)
            return None, dict(failure=str(error), guidance_event_index=len(self.guidance_events) - 1)
        finally:
            validity.post_sampling_transform, validity.box_rotation_cost = old_transform, old_cost
            event['total_ms'] = (clock() - began) * 1000

    def cartesian_segment(self, contact, stage, target_joints=None):
        short = stage in ('contact_forward', 'precontact_reverse')
        checker = self.checker(self.selected_task)
        poses = self._tool_poses(checker, torch.tensor(contact, device='cuda', dtype=torch.float32))
        swivels = {side: self.analytic.swivel(arm, contact[1 + arm * 7:8 + arm * 7]) for arm, side in enumerate(('left', 'right'))}
        case = dict(case_id=stage, task=dict(self.selected_task), contact_joints=np.asarray(contact).tolist(), stage=stage,
            stage_distance_m=.03 if short else .35, sample_count=128, payload=not short,
            tcp_start_local={side: poses[side] for side in self.selected_task},
            translation_local={side: (np.asarray(getattr(self, 'extraction_direction', [-1., 0., 0.])) * .35).tolist() for side in self.selected_task},
            fixed_swivels_rad=swivels)
        self.cartesian_engine.prepare(case)
        if target_joints is None:
            candidates = self.cartesian_engine.endpoint_bank()
            if not len(candidates):
                raise CycleBlocked('抽离终点IK', '没有合法携箱终点，停止，不改选上游', self.last_partial)
            target = self.record_endpoint('extraction_target', candidates, contact, 'nearest actual contact; endpoint only')
        else:
            target = np.asarray(target_joints)
        previous = self.guidance_phase
        self.guidance_phase = 'contact_approach' if short else 'extract'
        lower, upper = checker.kinematics.get_joint_limits().position
        validity = GpuValidity(checker, check_ground=True)
        validity.weights[0] = np.deg2rad(15) / .1
        try:
            path, stats = self._guided_search(torch.tensor(contact, device='cuda', dtype=torch.float32),
                torch.tensor(target[None], device='cuda', dtype=torch.float32), lower, upper, validity, 2., 20261024)
        finally:
            self.guidance_phase = previous
        event = self.guidance_events[-1]
        event['collision_scope'] = 'empty intermediate; loaded endpoint' if not short else 'empty; grasp target excluded'
        self.last_partial.setdefault('cartesian_segments', {})[stage] = dict(success=path is not None, connected=path is not None,
            details=stats, inclusive_ms=event['total_ms'], pipeline=self.pipeline_label)
        if path is None:
            raise CycleBlocked('解析30轮接近' if short else '解析30轮抽离', event.get('failure', '未连接'), self.last_partial)
        if not short:
            loaded = self.checker(self.selected_task, payload=True)
            if not bool(GpuValidity(loaded, check_ground=True).mask(torch.tensor(path[-1:], device='cuda')).all()):
                raise CycleBlocked('抽离携箱终点验收', '终点携箱碰撞', self.last_partial)
        return path

    def full_cycle(self, boxes, progress=lambda text: None):
        result = super().full_cycle(boxes, lambda text: progress(text.replace('LM', '解析＋整段30轮')))
        result['planner'] = self.pipeline_label
        result['pipeline_policy'] = self.pipeline_description
        result['guided_search_policy'] = self.pipeline_description
        result['path_prior_lm_used'] = False
        result['joint_adjustment_rounds_per_stage'] = 30
        result['conveyor_release_adjustment'] = dict(delta_z_m=-self.conveyor_release_drop_m,
            box_center_z_m=self.conveyor_height_m + self.box_fit['dimensions_m'][2] / 2 + self.placement_clearance_m)
        if 'adaptive_raised_policy' in result:
            result['adaptive_raised_policy']['conveyor_box_center_z_m'] = result['conveyor_release_adjustment']['box_center_z_m']
        if 'extraction_policy' in result:
            result['extraction_policy']['mode'] = 'polar_analytic_joint30_guided_motion'
        if 'precontact_policy' in result:
            result['precontact_policy']['mode'] = 'direct_pose_ik_then_forward_polar_analytic_joint30'
        if not result.get('success') and self.guidance_events and not self.guidance_events[-1]['success']:
            last = self.guidance_events[-1]
            result['blocked_details'] = f"{last['phase']} / {last.get('step', 'planning')}：{last.get('failure', '未完成')}"
        return result


def guidance_summary(events):
    lines = ['|阶段|球面ms|解析ms|筛选ms|联合30轮ms|搜索ms|细验ms|结果|', '|---|---:|---:|---:|---:|---:|---:|---|']
    for entry in events:
        lines.append(f"|{entry['phase']}|{entry.get('reference_ms',0):.1f}|{entry.get('analytic_ms',0):.1f}|{entry.get('filter_ms',0):.1f}|{entry.get('joint_adjustment_ms',0):.1f}|{entry.get('search_ms',0):.1f}|{entry.get('fine_validation_ms',0):.1f}|{'通过' if entry['success'] else entry.get('failure','未通过')}|")
    return '\n'.join(lines)
