from pathlib import Path
import sys
from unittest.mock import Mock

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from curobo_core.planner import CycleBlocked
from v3_frozen_cycle import FrozenCyclePlanner
from v3_predicted_selected_lm import PredictedSelectedLmPlanner


def test_both_grasp_modes_prepare_predictions_without_full_lm():
    tools = Path(__file__).resolve().parents[1] / 'tools'
    for name in ('v3_all_extraction_roots_test.py', 'v3_side_upward_prepare.py'):
        source = (tools / name).read_text()
        assert "if hasattr(p,'prepare_predicted_root'):" in source
        assert source.index('p.prepare_predicted_root') < source.index('p.analytic_extract')


def test_prediction_does_not_claim_an_extraction_path(monkeypatch):
    planner = object.__new__(PredictedSelectedLmPlanner)
    planner._allowed_contacts = {0}
    planner.prediction_records = [dict(endpoint=[0.] * 15)]
    validity = Mock()
    validity.mask.return_value = torch.tensor([True])
    tensor = torch.tensor
    monkeypatch.setattr(torch, 'tensor', lambda *args, **kwargs: tensor(*args, **dict(kwargs, device='cpu')))
    root, detail = planner.prepare_predicted_root(0, np.zeros(15), validity)
    assert root['extraction_frames'] is None
    assert root['prediction_only']


@pytest.mark.parametrize('mismatch', [False, True])
def test_selected_lm_is_computed_once_and_actual_endpoint_becomes_root(monkeypatch, mismatch):
    planner = object.__new__(PredictedSelectedLmPlanner)
    planner._extraction_cache = {}
    planner.lazy_timings = dict(extraction_work_ms=0.)
    planner.last_partial = {}
    frames = np.zeros((2, 15))
    if mismatch:
        frames[-1, 2] = .1
    solve = Mock(return_value=(frames, dict(success=True)))
    monkeypatch.setattr(FrozenCyclePlanner, 'analytic_extract', solve)
    root = dict(contact_index=0, contact=[0.] * 15, start=[0.] * 15, extraction_frames=None)
    planner.validate_selected_extraction(root, None, None)
    planner.validate_selected_extraction(root, None, None)
    np.testing.assert_array_equal(root['extraction_frames'], frames)
    np.testing.assert_array_equal(root['start'], frames[-1])
    np.testing.assert_array_equal(root['predicted_start'], np.zeros(15))
    assert solve.call_count == 1


def test_actual_endpoint_reconnect_retains_selected_goal(monkeypatch):
    planner = object.__new__(PredictedSelectedLmPlanner)
    planner.selected_lm_review = {}
    actual = np.zeros(15, dtype=np.float32)
    actual[1] = .2
    goal = np.ones(15, dtype=np.float32)
    planner.verified_rrt = Mock(return_value=(np.stack([actual, goal]), {'success': True}))
    tensor = torch.tensor
    monkeypatch.setattr(torch, 'tensor', lambda *args, **kwargs: tensor(*args, **dict(kwargs, device='cpu')))
    monkeypatch.setattr(torch.cuda, 'synchronize', lambda: None)
    root = dict(start=actual.tolist(), contact_index=3)
    path, stats = planner.reconcile_selected_transport(root, np.stack([np.zeros(15), goal]), {}, object(), -torch.ones(15), torch.ones(15))
    call = planner.verified_rrt.call_args.args
    np.testing.assert_allclose(call[0].numpy()[0], actual)
    np.testing.assert_allclose(call[1].numpy()[0], goal)
    np.testing.assert_allclose(path[0], actual)
    np.testing.assert_allclose(path[-1], goal)
    assert planner.selected_lm_review['actual_transport_reconnect_success']
