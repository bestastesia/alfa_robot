import time

import numpy as np
import torch

from curobo_core.planner import CycleBlocked
from v3_frozen_cycle import FrozenCyclePlanner
from v3_batched_loaded_search import densify


class PredictedSelectedLmPlanner(FrozenCyclePlanner):
    dense_path_rechecks = False

    def placement_solver(self):
        engine = super().placement_solver()
        engine.validate_intermediate_edges = False
        return engine

    def prepare_predicted_root(self, index, contact, validity):
        if index not in self._allowed_contacts:
            return None, 'deferred prediction batch'
        endpoint = self.prediction_records[index]['endpoint']
        if endpoint is None:
            return None, 'analytic endpoint prediction unavailable'
        values = torch.tensor([endpoint], device='cuda', dtype=torch.float32)
        if not bool(validity.mask(values).all()):
            return None, 'predicted endpoint collision or joint limit'
        return dict(contact_index=index, contact=np.asarray(contact).tolist(), start=endpoint,
                    extraction_frames=None, prediction_only=True), dict(method='analytic endpoint prediction only; full LM not run')

    def validate_selected_extraction(self, root, empty_validity, loaded_validity):
        if root.get('extraction_frames') is not None and root.get('prediction_only', True):
            raise RuntimeError('candidate preparation unexpectedly computed a full extraction')
        key = tuple(np.round(np.asarray(root['contact'], dtype=np.float64), 7))
        began = time.perf_counter()
        if key not in self._extraction_cache:
            self._extraction_cache[key] = super().analytic_extract(np.asarray(root['contact']), empty_validity, loaded_validity)
            self.lazy_timings['extraction_work_ms'] += (time.perf_counter() - began) * 1000
        frames, detail = self._extraction_cache[key]
        predicted = root.get('predicted_start', root['start'])
        review = dict(contact_index=root['contact_index'], predicted_endpoint=predicted, lm_detail=detail,
                      lm_frames=None if frames is None else frames.tolist())
        self.selected_lm_review = review
        if frames is None:
            raise CycleBlocked('选中候选LM抽离', '选中组合的完整LM抽离失败；未改选或拼接', dict(self.last_partial, selected_lm_review=review))
        difference = np.asarray(frames[-1]) - np.asarray(predicted)
        review.update(actual_endpoint=frames[-1].tolist(), updown_difference_m=float(abs(difference[0])),
                      max_arm_difference_deg=float(np.rad2deg(abs(difference[1:])).max()))
        root.setdefault('predicted_start', list(root['start']))
        root['start'] = frames[-1].tolist()
        root['extraction_frames'] = frames.tolist()
        root['prediction_only'] = False

    def reconcile_selected_transport(self, root, path, stats, validity, lower, upper):
        start = np.asarray(root['start'], dtype=np.float32)
        if np.max(abs(start - path[0])) <= 1e-5:
            return path, stats
        began = time.perf_counter()
        actual = torch.tensor(start[None], device='cuda', dtype=torch.float32).contiguous()
        target = torch.tensor(np.asarray(path[-1:]), device='cuda', dtype=torch.float32).contiguous()
        connection, detail = self.verified_rrt(actual, target, lower + 1e-5, upper - 1e-5,
            validity, 2., 20261107 + root['contact_index'], stop_at_first_valid=True)
        torch.cuda.synchronize()
        self.selected_lm_review['actual_transport_reconnect_ms'] = (time.perf_counter() - began) * 1000
        self.selected_lm_review['actual_transport_reconnect_success'] = connection is not None
        if connection is None:
            return None, dict(stats, actual_endpoint_reconnect=detail)
        return np.asarray(densify(connection)), dict(stats, actual_endpoint_reconnect=detail,
            endpoint_policy='selected transport goal retained; RRT starts at actual LM endpoint')

    def analytic_extract(self, contact, empty_validity, loaded_validity):
        if self.cached_selection is not None:
            root = self.cached_selection['root']
            return np.asarray(root['extraction_frames']), self.selected_lm_review['lm_detail']
        return super().analytic_extract(contact, empty_validity, loaded_validity)

    def full_cycle(self, boxes, progress=lambda message: None):
        if len(boxes) != 2:
            raise ValueError('预测选型实验当前仅开放双臂任务；未把单臂旧流程伪装为新流程')
        self.selected_lm_review = None
        result = super().full_cycle(boxes, progress)
        result['endpoint_selection_policy'] = 'predicted endpoint roots → multi-root multi-goal RRT → selected full LM → reconnect actual endpoint to selected transport goal when different'
        result['selected_lm_review'] = self.selected_lm_review
        result['cycle_profile'] = 'predicted-selected-lm-fast-20261107'
        result['collision_acceptance_policy'] = 'RRT search node/edge checks and LM candidate node checks; no returned-path densification recheck or final whole-cycle recheck'
        return result
