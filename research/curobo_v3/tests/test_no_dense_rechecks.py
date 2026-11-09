from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import sys
import time

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import v3_lm_conveyor as lm
import v3_restored_analytic_cycle as restored
from v3_predicted_selected_lm import PredictedSelectedLmPlanner


def test_lm_checks_candidates_but_never_interpolated_edges(monkeypatch):
    engine = object.__new__(lm.SequentialPlacementLm)
    engine.active_sides = ('left',)
    engine.idle_indices = list(range(8, 15))
    engine.to_solver = engine.to_active = list(range(15))
    engine.samples = engine.seeds = 2
    engine.iterations = 32
    engine.validate_intermediate_edges = False
    rows = torch.zeros((1, 2, 15))
    rows[0, 0, 1] = .01
    engine.solver = SimpleNamespace(joint_names=lm.ACTIVE_JOINTS,
        solve_single=Mock(return_value=SimpleNamespace(js_solution=SimpleNamespace(position=rows), success=torch.tensor([[True, False]]))))
    validity = Mock()
    validity.mask.side_effect = [torch.tensor([True]), torch.tensor([True, True])]
    checker = SimpleNamespace(guidance_payload=True, kinematics=SimpleNamespace(get_joint_limits=lambda: SimpleNamespace(position=(-torch.ones(15), torch.ones(15)))))
    monkeypatch.setattr(lm, 'GpuValidity', lambda *args, **kwargs: validity)
    monkeypatch.setattr(lm, 'base_world_audit', lambda checker: dict(valid=True))
    monkeypatch.setattr(lm, 'clock', time.perf_counter)
    monkeypatch.setattr(lm, 'box_poses', lambda *args: {})
    monkeypatch.setattr(lm, 'reference_errors', lambda *args: (np.zeros(2), np.zeros(2)))
    dense = Mock(side_effect=AssertionError('interpolated edge must not be evaluated'))
    monkeypatch.setattr(lm, 'dense_segment', dense)
    tensor = torch.tensor
    monkeypatch.setattr(torch, 'tensor', lambda *args, **kwargs: tensor(*args, **dict(kwargs, device='cpu')))
    result = engine.plan(dict(start=np.zeros(15), checker=checker, attachments={'left':np.eye(4)}, starts={'left':np.eye(4)}, ends={'left':np.eye(4)}))
    assert result['success'] and len(result['frames']) == 2
    assert result['intermediate_edge_checks'] is False
    assert validity.mask.call_count == 2
    dense.assert_not_called()


def test_rrt_retains_search_validity_without_extra_audit(monkeypatch):
    planner = object.__new__(PredictedSelectedLmPlanner)
    planner.audit_path = Mock(side_effect=AssertionError('extra audit forbidden'))
    validity = SimpleNamespace(path_quality_callback=None)
    path = np.zeros((2, 15))
    search = Mock(return_value=(path, {'success': True}))
    monkeypatch.setattr(restored, 'original_rrt', search)
    result, stats = planner.verified_rrt(None, None, None, None, validity)
    assert result is path
    assert search.call_args.args[4] is validity
    assert validity.validate_returned_path is False
    assert not stats['extra_path_recheck']
    assert planner._final_audit({}, None) == []
    planner.audit_path.assert_not_called()
