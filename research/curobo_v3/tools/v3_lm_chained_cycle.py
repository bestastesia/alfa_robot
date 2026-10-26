import gc
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from curobo_core.planner import CycleBlocked
from v3_batched_loaded_search import GpuValidity, densify
from v3_box_rotation_cost import BoxRotationCost
from v3_cartesian_cases import line_samples
from v3_cartesian_compare import CartesianComparison
from v3_serial_endpoint_demo import SETTINGS
from v3_serial_endpoint_planner import SerialEndpointPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS


class LmChainedCyclePlanner(SerialEndpointPlanner):
    serial_endpoint_policy = True
    lm_chained_cycle_policy = True

    def __init__(self, args, reference_source=None):
        self.active_payload_layout = None
        self.resource_release_events = []
        super().__init__(args, reference_source=reference_source or SETTINGS)
        self.cartesian_engine = CartesianComparison(payload_policy='legacy', planner=self)

    def configure_task(self, boxes):
        result = super().configure_task(boxes)
        layout = (self.top_grasp, tuple(sorted(boxes)))
        if self.active_payload_layout is not None and layout != self.active_payload_layout:
            before = torch.cuda.memory_allocated()
            engine = getattr(self, 'cartesian_engine', None)
            if engine is not None:
                engine.__dict__.pop('loaded', None)
            cache = getattr(self, 'persistent_checkers', {})
            stale = [key for key, resource in cache.items() if getattr(resource['checker'], 'guidance_payload', False)]
            for key in stale:
                del cache[key]
            gc.collect()
            torch.cuda.empty_cache()
            self.resource_release_events.append(dict(previous_layout=self.active_payload_layout, next_layout=layout,
                released_payload_checkers=len(stale), allocated_before_mib=before / 2**20,
                allocated_after_mib=torch.cuda.memory_allocated() / 2**20))
        self.active_payload_layout = layout
        return result

    def collision_checker(self, boxes, payload=False, mobile=False):
        checker = super().collision_checker(boxes, payload, mobile)
        layer = getattr(checker, 'base_cuboid_layer', None)
        if layer is not None:
            layer = getattr(layer, 'local_layer', layer)
            layer.query_cache_limit = 4
            while len(layer.query_cache) > 4:
                layer.query_cache.pop(next(iter(layer.query_cache)))
        return checker

    def full_cycle(self, boxes, progress=lambda text: None):
        started = time.perf_counter()
        release_start = len(self.resource_release_events)
        result = super().full_cycle(boxes, progress)
        result['resource_release_events'] = self.resource_release_events[release_start:]
        result['measured_wall_ms'] = (time.perf_counter() - started) * 1000
        failures = [str(result.get('blocked_details', ''))] + [str(event.get('failure', '')) for event in result.get('guided_search_events', [])]
        if any('out of memory' in failure.lower() for failure in failures):
            raise torch.OutOfMemoryError('本次六段规划发生显存不足，停止批测；不记作路线无解')
        return result

    def cartesian_segment(self, contact, stage, target_joints=None):
        if stage == 'extraction' and ('lm', True) in self.endpoint_ik_cache:
            self.cartesian_engine.solvers[64] = self.endpoint_ik_cache[('lm', True)]
        checker = self.checker(self.selected_task)
        poses = self._tool_poses(checker, torch.tensor(contact, device='cuda', dtype=torch.float32))
        short = stage in ('precontact_reverse', 'contact_forward')
        distance = .03 if short else .35
        self.fk(contact)
        swivels = {side: self.analytic.swivel(arm, contact[1 + arm * 7:8 + arm * 7]) for arm, side in enumerate(('left', 'right'))}
        case = dict(case_id=stage, task=dict(self.selected_task), contact_joints=np.asarray(contact).tolist(),
            stage=stage, stage_distance_m=distance, sample_count=16 if short else 128,
            payload=stage == 'extraction', tcp_start_local={side: poses[side] for side in self.selected_task},
            translation_local={side: (-np.asarray(poses[side])[:3, 2] * distance if stage == 'precontact_reverse'
                else np.array([-distance, 0., 0.])).tolist() for side in self.selected_task}, fixed_swivels_rad=swivels)
        if target_joints is not None:
            targets = self._tool_poses(checker, torch.tensor(target_joints, device='cuda', dtype=torch.float32))
            case['translation_local'] = {side: (np.asarray(targets[side])[:3, 3] - np.asarray(poses[side])[:3, 3]).tolist() for side in self.selected_task}
            case['fixed_goal_joints'] = np.asarray(target_joints).tolist()
        result = self.cartesian_engine.run(case, 'lm_informed_connect')
        result['shared_64_point_ik_solver'] = stage == 'extraction' and self.cartesian_engine.solvers.get(64) is self.endpoint_ik_cache.get(('lm', True))
        self.last_partial.setdefault('cartesian_segments', {})[stage] = result
        details = result['details']
        phase = 'contact_approach' if short else 'extract'
        event = dict(phase=phase, payload=case['payload'], success=result['success'], roots=1, goals=1,
            reference_kind='cartesian_soft_reference', point_count=case['sample_count'],
            reference_ms=0., lm_ms=details.get('lm_ms', 0.) + details.get('soft_refine_ms', 0.),
            filter_ms=details.get('filter_ms', 0.), search_ms=details.get('search_ms', 0.),
            total_ms=result['inclusive_ms'], legal_nodes=details.get('legal_nodes', 0), covered_points=details.get('covered_points', 0),
            search_base_margin_m=.002, intermediate_deviation_is_observation_only=True,
            collision_scope='empty intermediate states; loaded endpoint' if case['payload'] else 'empty; grasp target excluded')
        if result['connected']:
            tracks = result['metrics']['tcp_tracks']
            if stage == 'precontact_reverse':
                tracks = {side: dict(track, positions=track['positions'][::-1], quaternions_wxyz=track['quaternions_wxyz'][::-1]) for side, track in tracks.items()}
            event['actual_tracks'] = {side: dict(track, total_rotation_deg=track['cumulative_rotation_deg']) for side, track in tracks.items()}
            event['actual_rotation_deg'] = {side: track['cumulative_rotation_deg'] for side, track in tracks.items()}
            event['frames'] = len(result['frames'])
            samples = line_samples(case)
            event['reference_tracks'] = {side: dict(positions=matrices[:, :3, 3].tolist(),
                quaternions_wxyz=np.roll(Rotation.from_matrix(matrices[:, :3, :3]).as_quat(), 1, axis=1).tolist(), total_rotation_deg=0.) for side, matrices in samples.items()}
        else:
            event['failure'] = details.get('failure', 'LM引导未连接')
        self.guidance_events.append(event)
        if not result['success']:
            raise CycleBlocked('LM接近段' if short else 'LM抽离', details.get('failure', '碰撞/起终点验收失败'), self.last_partial)
        return np.asarray(result['frames'])

    def initial_motion(self, boxes, empty, report, append, progress):
        progress('生成新接触候选，仅参考Home选吸附构型')
        started = time.perf_counter()
        if len(boxes) == 1:
            target = self.contact_poses_for_task(boxes)[self.active_side + '_tool0']
            matrix = np.eye(4)
            matrix[:3, 3] = target['position']
            matrix[:3, :3] = Rotation.from_quat(np.roll(target['quaternion'], -1)).as_matrix()
            contacts = self.enumerate_single(matrix, self.home_values, empty)
        else:
            solved = self.build_contacts(boxes)
            contacts = solved.js_solution.position.detach().cpu().numpy().reshape(-1, len(solved.js_solution.joint_names))
            contacts = contacts[:, [solved.js_solution.joint_names.index(name) for name in ACTIVE_JOINTS]]
        report['timing_ms']['contact_candidates_ms'] = (time.perf_counter() - started) * 1000
        report['contact_ik_count'] = len(contacts)
        report['contact_candidates'] = np.asarray(contacts).tolist()
        if not len(contacts):
            raise CycleBlocked('接触IK', '无合法接触构型', report)
        contact = self.record_endpoint('contact', contacts, self.home_values, 'fresh contact candidates; nearest Home, no downstream re-selection')
        report['selected_candidate'] = self.serial_records[-1]['selected_index'] + 1
        progress('LM软直线参考反推3cm预接触，允许Updown/冗余变化')
        reverse_path = self.cartesian_segment(contact, 'precontact_reverse')
        precontact = reverse_path[-1].copy()
        self.serial_records.append(dict(name='precontact', joints=precontact.tolist(), previous_joints=contact.tolist(),
            source='actual endpoint of newly solved reverse LM segment', downstream_validation_used=False))
        progress('Home→实际预接触构型：球腕参考＋LM先验＋Informed Connect')
        approach, stats = self.joint_plan(self.home_values, precontact, empty, self.target_validity(empty), 20261023)
        report['approach_search_stats'] = stats
        if approach is None:
            raise CycleBlocked('Home到LM预接触', f'固定新预接触构型未连通：{stats}', report)
        append(approach, 'approach', False)
        append(reverse_path[::-1], 'contact_approach', False)
        append([contact], 'attach', True)
        return contact

    def _full_cycle(self, boxes, progress=lambda text: None):
        began = time.perf_counter()
        names = list(self.mobile_robot['kinematics']['cspace']['joint_names'])
        report = dict(success=False, task=boxes, joint_names=names, frames=[], phases=[], payload=[], attempts=[], timing_ms={},
            planner='six_stage_lm_guided_informed_connect', serial_endpoint_records=self.serial_records,
            endpoint_selection_policy='predecessor-only; fresh contact then actual LM outputs feed following phases; no backtracking',
            single_arm=len(boxes) == 1, rotation_quality_hard_gate=False, lm_chained_cycle=True, seam_checks=[])
        self.last_partial = report

        def append(rows, phase, payload):
            converted = []
            for row in rows:
                mapping = dict(zip(ACTIVE_JOINTS, row)) if len(row) == 15 else dict(zip(names, row))
                converted.append([float(mapping.get(name, 0)) for name in names])
            if converted and report['frames']:
                indices = [names.index(name) for name in ACTIVE_JOINTS]
                error = float(np.abs(np.asarray(converted[0])[indices] - np.asarray(report['frames'][-1])[indices]).max())
                report['seam_checks'].append(dict(phase=phase, maximum_joint_difference=error, passed=error <= 1e-5))
                if error > 1e-5:
                    raise CycleBlocked('阶段关节衔接', f'{phase}起点没有接上上一段实际终点：{error}', report)
            report['frames'].extend(converted)
            report['phases'].extend([phase] * len(converted))
            report['payload'].extend([payload] * len(converted))

        empty = self.checker(boxes)
        loaded = self.checker(boxes, payload=True)
        empty_valid = GpuValidity(empty, check_ground=True)
        loaded_valid = GpuValidity(loaded, check_ground=True)
        append([self.home_values], 'home', False)
        if not bool(empty_valid.mask(torch.tensor(self.home_values[None], device='cuda', dtype=torch.float32)).all()):
            raise CycleBlocked('Home', '初始构型碰撞', report)
        contact = self.initial_motion(boxes, empty, report, append, progress)
        progress('从实际吸附构型开始LM引导后撤35cm，生成真实关节运动')
        extracted = self.cartesian_segment(contact, 'extraction')
        endpoint = extracted[-1].copy()
        append(extracted, 'extract', True)
        self.serial_records.append(dict(name='extraction', joints=endpoint.tolist(), previous_joints=contact.tolist(),
            source='actual LM extraction endpoint; no teleport', downstream_validation_used=False))
        report['extraction_policy'] = dict(mode='lm_guided_motion', distance_m=.35, path_generated=True, path_validated=True,
            payload_path_validated=False, endpoint_payload_validated=True, collision_scope='legacy empty intermediate; loaded endpoint')
        progress('按实际抽离终点选择运输构型并重新规划')
        started = time.perf_counter()
        candidates, radius_limit = self.transport_candidates(endpoint, loaded, loaded_valid)
        report['timing_ms']['transport_endpoint_ms'] = (time.perf_counter() - started) * 1000
        if not len(candidates):
            raise CycleBlocked('实际抽离后运输终点', '没有合法运输构型，不回选上游终点', report)
        target = self.record_endpoint('transport', candidates, endpoint, 'nearest actual LM extraction end; current transport-stage checks only')
        self.guidance_phase = 'transport'
        if len(boxes) == 1:
            transport, stats = self.single_path(endpoint, [target], loaded, True)
        else:
            validity = GpuValidity(loaded, check_ground=True)
            validity.weights[0] = np.deg2rad(15) / .1
            validity.box_rotation_cost = BoxRotationCost(loaded, ACTIVE_JOINTS, torch.tensor(endpoint, device='cuda', dtype=torch.float32),
                torch.tensor(target, device='cuda', dtype=torch.float32), self.top_grasp)
            lower, upper = loaded.kinematics.get_joint_limits().position
            path, stats = self._guided_search(torch.tensor(endpoint, device='cuda', dtype=torch.float32),
                torch.tensor(target[None], device='cuda', dtype=torch.float32), lower, upper, validity, 2., 20261024)
            transport = np.asarray(densify(path)) if path is not None else None
        report['transport_search_stats'] = stats
        if transport is None:
            raise CycleBlocked('实际LM抽离后运输搜索', f'未连通，不回换上游构型：{stats}', report)
        if not bool(loaded_valid.mask(torch.tensor(transport, device='cuda', dtype=torch.float32)).all()):
            raise CycleBlocked('运输密化验收', '携箱碰撞', report)
        report['transport_endpoint_envelope'] = dict(radius_m=float(self.full_envelope_radius(loaded, transport[-1:]).item()),
            limit_m=radius_limit, includes_idle_arm=True, includes_payload=True)
        append(transport, 'transport', True)
        self.conveyor_suffix(boxes, report, append, progress)
        values = np.asarray(report['frames'])[:, [names.index(name) for name in ACTIVE_JOINTS]]
        report['final_home_error'] = float(np.abs(values[-1] - self.home_values).max())
        report.update(success=True, total_ms=(time.perf_counter() - began) * 1000)
        return report
