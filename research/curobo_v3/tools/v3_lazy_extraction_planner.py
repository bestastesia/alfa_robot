import time

import numpy as np
import torch

from curobo_core.distance_metric import joint_distance_weights
from v3_box_rotation_cost import BoxRotationCost
from v3_persistent_collision_planner import PersistentCollisionPlanner
from v3_research_assets import atlas_path
from v3_wall_ik_benchmark import ACTIVE_JOINTS


class LazyExtractionPlanner(PersistentCollisionPlanner):
    rotation_quality_hard_gate = False
    lazy_schedule = ((8, 350.), (16, 450.), (32, 600.), (64, 600.))
    lazy_search_budget_ms = 2000.

    def predict_endpoint(self, contact):
        self.fk(contact)
        carriage = np.linalg.inv(self.fk_robot.get_transform('arm_carriage', self.fk_robot.base_link))
        predicted = contact.copy()
        for side_index, side in enumerate(['left', 'right']):
            target = self.fk_robot.get_transform(side + '_tool0', self.fk_robot.base_link).copy()
            target[0, 3] -= .35
            offset = 1 + side_index * 7
            seed = contact[offset:offset + 7]
            swivel = self.analytic.swivel(side_index, seed)
            solutions = self.analytic.solve(side_index, carriage @ target, swivel, seed)
            if not len(solutions):
                return None
            predicted[offset:offset + 7] = solutions[np.argmin(np.sum((solutions - seed) ** 2, axis=1))]
        return predicted

    def generate_contacts(self, boxes):
        began = time.perf_counter()
        self.analytic_task = None
        self.ik_solver(boxes)
        torch.cuda.synchronize()
        self.lazy_timings['contact_generation_ms'] = (time.perf_counter() - began) * 1000
        return self.analytic_result.js_solution.position.detach().cpu().numpy().reshape(-1, 15).astype(np.float64)

    def rank_contacts(self, boxes):
        return self.baseline_ranking(boxes, self.generate_contacts(boxes))

    def lazy_batches(self, boxes, ranked):
        """Yield (allowed contact indices, candidate limit, search allocation ms, kind) per lazy batch."""
        for limit, allocation_ms in self.lazy_schedule:
            yield ranked[:limit], limit, allocation_ms, 'baseline'

    def baseline_ranking(self, boxes, contacts):
        began = time.perf_counter()
        predictions = [self.predict_endpoint(contact) for contact in contacts]
        self.lazy_timings['endpoint_prediction_ms'] = (time.perf_counter() - began) * 1000
        eligible = [index for index, endpoint in enumerate(predictions) if endpoint is not None]
        self.prediction_records = [dict(contact_index=index, endpoint_exists=endpoint is not None,
                                        endpoint=None if endpoint is None else endpoint.tolist())
                                   for index, endpoint in enumerate(predictions)]
        if not eligible:
            return list(range(len(contacts)))
        began = time.perf_counter()
        starts = torch.tensor(np.asarray([predictions[index] for index in eligible]), device='cuda', dtype=torch.float32)
        atlas = np.load(atlas_path(self), allow_pickle=False)
        goals = torch.tensor(atlas['joints'], device='cuda', dtype=torch.float32)
        weights = torch.tensor(joint_distance_weights(ACTIVE_JOINTS), device='cuda')
        distances = torch.linalg.vector_norm((starts[:, None] - goals[None]) * weights, dim=-1)
        nearest_distances, nearest = torch.topk(distances, min(8, len(goals)), largest=False, dim=1)
        neighbors = goals[nearest]
        fractions = torch.linspace(0, 1, 17, device='cuda')
        samples = (starts[:, None, None] + fractions[None, None, :, None] * (neighbors[:, :, None] - starts[:, None, None])).reshape(-1, 15)
        checker = self.checker(boxes, payload=True)
        metric = BoxRotationCost(checker, ACTIVE_JOINTS, starts[0], goals[0], self.top_grasp)
        parts = {side: [] for side in ['left', 'right']}
        for offset in range(0, len(samples), 1024):
            orientations = metric.orientations(samples[offset:offset + 1024].contiguous())
            for side in parts:
                parts[side].append(orientations[side])
        penalty = torch.zeros_like(nearest_distances)
        for side in parts:
            quaternion = torch.cat(parts[side]).reshape(len(starts), nearest.shape[1], 17, 4)
            adjacent = (quaternion[:, :, :-1] * quaternion[:, :, 1:]).sum(-1).abs().clamp(max=1)
            total = (2 * torch.acos(adjacent)).sum(-1)
            necessary = 2 * torch.acos((quaternion[:, :, 0] * quaternion[:, :, -1]).sum(-1).abs().clamp(max=1))
            penalty += torch.relu(total - necessary)
        distance_order = torch.argsort(nearest_distances[:, 0]).cpu().tolist()
        rotation_order = torch.argsort(penalty.min(dim=1).values).cpu().tolist()
        ranked = []
        for distance_index, rotation_index in zip(distance_order, rotation_order):
            for local_index in [distance_index, rotation_index]:
                original_index = eligible[local_index]
                if original_index not in ranked:
                    ranked.append(original_index)
        ranked.extend(index for index in range(len(contacts)) if index not in ranked)
        torch.cuda.synchronize()
        self.lazy_timings['prediction_ranking_ms'] = (time.perf_counter() - began) * 1000
        return ranked

    def analytic_extract(self, contact, empty_validity, loaded_validity):
        if not getattr(self, '_lazy_preparing', False):
            return super().analytic_extract(contact, empty_validity, loaded_validity)
        key = tuple(np.round(np.asarray(contact, dtype=np.float64), 7))
        original_index = self._contact_indices[key]
        if original_index not in self._allowed_contacts:
            return None, '延后到后续候选批次；尚未做抽离验收'
        if key not in self._extraction_cache:
            torch.cuda.synchronize()
            began = time.perf_counter()
            frames, detail = super().analytic_extract(contact, empty_validity, loaded_validity)
            torch.cuda.synchronize()
            self.lazy_timings['extraction_work_ms'] += (time.perf_counter() - began) * 1000
            self._extraction_cache[key] = (frames, detail)
        return self._extraction_cache[key]

    def prepare_roots(self, boxes):
        if not getattr(self, 'lazy_enabled', True):
            return super().prepare_roots(boxes)
        self._lazy_preparing = True
        try:
            return super().prepare_roots(boxes)
        finally:
            self._lazy_preparing = False

    def full_cycle(self, boxes, progress=lambda text: None):
        if not getattr(self, 'lazy_enabled', True):
            self.analytic_task = None
            self.transport_search_budget_ms = 2000.
            return super().full_cycle(boxes, progress)
        began = time.perf_counter()
        self.cached_selection = None
        self.last_partial = dict(success=False, task=boxes, frames=[], phases=[], payload=[],
                                 joint_names=list(self.mobile_robot['kinematics']['cspace']['joint_names']))
        self.lazy_timings = dict(contact_generation_ms=0., endpoint_prediction_ms=0., prediction_ranking_ms=0., extraction_work_ms=0.)
        self._extraction_cache = {}
        self._lazy_preparing = False
        ranked = self.rank_contacts(boxes)
        contacts = self.analytic_result.js_solution.position.detach().cpu().numpy().reshape(-1, 15)
        self._contact_indices = {tuple(np.round(contact.astype(np.float64), 7)): index for index, contact in enumerate(contacts)}
        batches = []
        search_spent_ms = 0.
        for allowed, limit, allocation_ms, kind in self.lazy_batches(boxes, ranked):
            self._allowed_contacts = set(allowed)
            self.transport_search_budget_ms = min(allocation_ms, max(0., self.lazy_search_budget_ms - search_spent_ms))
            progress(f'预测排序完成：验收前{min(limit,len(ranked))}／{len(ranked)}个候选，再做低旋转联合搜索')
            batch_started = time.perf_counter()
            self.last_multiroot_search_ms = 0.
            result = super().full_cycle(boxes, progress)
            search_ms = result.get('multiroot_search_ms', result.get('pipeline_timing_ms', {}).get('multiroot_search', self.last_multiroot_search_ms))
            search_spent_ms += search_ms
            batches.append(dict(candidate_limit=limit, actual_extractions=len(self._extraction_cache), success=result['success'],
                                total_ms=(time.perf_counter() - batch_started) * 1000, search_ms=search_ms,
                                search_budget_ms=self.transport_search_budget_ms, blocked_stage=result.get('blocked_stage')))
            if kind != 'baseline':
                batches[-1].update(kind=kind, allowed_contacts=list(allowed))
            if result['success'] or result.get('blocked_stage') not in ['抽离候选验收', '全链路候选验证'] or len(self._allowed_contacts) == len(ranked) or search_spent_ms >= self.lazy_search_budget_ms:
                break
        result['lazy_extraction'] = dict(timings_ms=dict(self.lazy_timings), batches=batches, ranking=ranked,
                                         predictions=self.prediction_records, checked_candidates=len(self._extraction_cache),
                                         candidate_pool=len(ranked), total_transport_search_ms=search_spent_ms,
                                         policy='64 fresh contacts; predicted endpoints; interleaved distance/rotation ranking; 8/16/32/64 lazy validation; shared 2s transport search budget')
        result['total_ms'] = (time.perf_counter() - began) * 1000
        result['rotation_acceptance_policy'] = 'soft cost only; extra rotation, backtracking and arc deviation are diagnostics, not rejection gates'
        result['rotation_quality_hard_gate'] = self.rotation_quality_hard_gate
        result['box_rotation_cost_enabled'] = True
        result['timing_note'] += ' Lazy total_ms includes prediction and all fallback batches; pipeline_timing_ms describes the final batch only.'
        return result
