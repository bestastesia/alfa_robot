from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from v3_new_warp import NewWarpPlanner, TimedNewWarpPlanner
from v3_polar_refined_cycle import PolarRefinedCyclePlanner


def test_absolute_release_height_is_applied_once_and_historical_profile_remains_available():
    for height in (.982, 1.082):
        planner = NewWarpPlanner(release_box_center_z_m=height)
        actual = planner.conveyor_height_m + planner.box_fit['dimensions_m'][2] / 2 + planner.placement_clearance_m
        assert np.isclose(actual, height)
        assert planner.analytic.__class__.__name__ == 'RezeroAnalytic'
        assert planner.strategy_policy['joint_adjustment_rounds'] == 30


def test_timed_strategy_keeps_new_warp_reference_and_joint_adjustment():
    planner = TimedNewWarpPlanner()
    assert planner._guided_search.__func__ is PolarRefinedCyclePlanner._guided_search
    assert planner._joint_adjust.__func__ is PolarRefinedCyclePlanner._joint_adjust
    assert planner.warmup_resources.__func__ is PolarRefinedCyclePlanner.warmup_resources
    assert planner.timed_validation_resolution_deg == .25
    planner.trajectory_optimization_enabled = False
    sentinel = dict(success=False)
    with patch.object(NewWarpPlanner, 'full_cycle', return_value=sentinel) as geometric:
        assert planner.full_cycle({'right': 2}) is sentinel
    geometric.assert_called_once()


def test_old_height_results_cannot_be_loaded_as_new_height_evidence():
    from curobo_core.scene import digest
    planner = NewWarpPlanner()
    valid = dict(strategy=dict(policy_identity=digest((planner.strategy_policy, .982))), model_id=planner.model_id)
    planner.validate_saved_result(valid)
    with pytest.raises(ValueError, match='释放高度'):
        planner.validate_saved_result(dict(valid, strategy=dict(policy_identity=digest((planner.strategy_policy, 1.082)))))
    with pytest.raises(ValueError, match='模型'):
        planner.validate_saved_result(dict(valid, model_id='old'))


@pytest.mark.skipif(not torch.cuda.is_available(), reason='requires GPU tensors')
def test_joint_adjustment_keeps_shared_lift_and_true_joint_endpoints():
    from curobo_core.new_warp_objective import FixedLiftObjective, spline_basis
    objective = FixedLiftObjective.__new__(FixedLiftObjective)
    objective.start = torch.zeros(15, device='cuda')
    objective.goal = torch.full((15,), .1, device='cuda')
    objective.basis = torch.tensor(spline_basis(32, 128), device='cuda', dtype=torch.float32)
    objective.lift = torch.linspace(0., .1, 128, device='cuda')
    interior = torch.randn((1, 30, 15), device='cuda', requires_grad=True)
    result = objective.decode(interior)
    torch.testing.assert_close(result[0, :, 0], objective.lift)
    torch.testing.assert_close(result[0, 0], objective.start)
    torch.testing.assert_close(result[0, -1], objective.goal)
    result.sum().backward()
    assert bool(torch.isfinite(interior.grad).all())


@pytest.mark.skipif(not torch.cuda.is_available(), reason='requires GPU validity tensors')
def test_colliding_reference_and_no_legal_prior_still_reach_rrt():
    from types import SimpleNamespace
    planner = NewWarpPlanner.__new__(NewWarpPlanner)
    planner.guidance_phase = 'transport'
    planner.guidance_events = []
    planner.arm_geometry = object()
    planner.analytic = object()
    samples = {side: np.tile(np.eye(4), (128, 1, 1)) for side in ('left', 'right')}
    planner._polar_reference = lambda *args: (samples, np.zeros(128), dict(reference_collision_free=False, reference_used_soft_fallback=True))
    planner._joint_adjust = lambda *args: (torch.zeros((128, 15), device='cuda'), dict(joint_iterations=30))
    checker = SimpleNamespace(guidance_payload=True)
    validity = SimpleNamespace(checker=checker, weights=torch.ones(15, device='cuda'),
        evaluate=lambda rows, **kwargs: (torch.ones(len(rows), dtype=torch.bool, device='cuda'), None),
        mask=lambda rows: torch.zeros(len(rows), dtype=torch.bool, device='cuda'))
    enumerated = dict(rows=np.empty((0, 15), np.float32), ids=np.empty(0, int), statistics=dict(enumeration_ms=0.))
    start = torch.zeros(15, device='cuda')
    with patch('v3_polar_refined_cycle.enumerate_smooth', return_value=enumerated), \
         patch('v3_polar_refined_cycle.pose_filter', return_value=(np.empty(0, bool), np.empty(0, bool))), \
         patch('v3_polar_refined_cycle.batched_rrt_connect_multi_goal', return_value=(None, dict(solutions_found=0))) as search:
        path, stats = planner._guided_search(start, start[None], start - 1, start + 1, validity, 2., 1)
    search.assert_called_once()
    assert path is None
    assert planner.guidance_events[-1]['rrt_without_legal_prior']
    assert planner.guidance_events[-1]['step'] == 'rrt_search'
    assert validity.post_sampling_transform is None
