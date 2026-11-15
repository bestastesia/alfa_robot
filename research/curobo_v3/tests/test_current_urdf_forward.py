from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import yourdfpy
import torch
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))

from v3_cartesian_compare import CartesianComparison
from v3_current_urdf_forward import CurrentUrdfForwardPlanner, current_assets, verify_current_model, transport_heights
from v3_batched_loaded_search import GpuValidity
from v3_research_assets import atlas_path
from v3_rezero_analytic import RezeroAnalytic
from v3_zero_model_candidate import TOOL_OFFSET_M


def test_shared_demo_accepts_both_current_assets_and_pr65_planner_selection():
    import inspect
    from v3_visualized_task_demo import main
    from v3_timed_single_arm_planner import TimedSingleArmPlanner

    signature = inspect.signature(main)
    signature.bind(default_assets=current_assets(), planner_class=TimedSingleArmPlanner)
    assert signature.parameters['planner_class'].default is None


def test_current_model_uses_matching_fixed_urdf_and_191mm_tools():
    assets = current_assets()
    verify_current_model(assets)
    model = yourdfpy.URDF.load(assets.urdf, load_meshes=False)
    assert model.base_link == 'base_footprint'
    for side in ('left', 'right'):
        np.testing.assert_allclose(model.get_transform(side + '_tool0', side + '_link7')[:3, 3],
                                   [0., 0., TOOL_OFFSET_M], atol=1e-10)


def test_current_grid_uses_rezero_adapter_not_legacy_compiled_grid():
    planner = SimpleNamespace(analytic=RezeroAnalytic())
    with patch('v3_cartesian_compare.ctypes.CDLL', side_effect=AssertionError('legacy grid loaded')):
        engine = CartesianComparison(planner=planner)
    assert engine.library is None


def test_batched_analytic_grid_roundtrips_current_urdf_in_both_arms():
    model = yourdfpy.URDF.load(current_assets().urdf, load_meshes=False)
    analytic = RezeroAnalytic()
    for arm, side in enumerate(('left', 'right')):
        seed = np.deg2rad([30., -40., 20., -70., 25., -35., 50.])
        model.update_cfg({name: 0. for name in model.actuated_joint_names})
        model.update_cfg({f'{side}_joint{index}': angle for index, angle in enumerate(seed, 1)})
        carriage = np.linalg.inv(model.get_transform('arm_carriage', model.base_link))
        target = carriage @ model.get_transform(side + '_tool0', model.base_link)
        rows, counts = analytic.solve_grid(arm, np.repeat(target[None], 2, axis=0),
                                          [analytic.swivel(arm, seed)], seed)
        assert (counts > 0).all()
        for solution in rows[0, 0, :counts[0, 0]]:
            assert np.all(solution >= analytic.lower[arm] - 1e-8)
            assert np.all(solution <= analytic.upper[arm] + 1e-8)
            model.update_cfg({f'{side}_joint{index}': angle for index, angle in enumerate(solution, 1)})
            actual = carriage @ model.get_transform(side + '_tool0', model.base_link)
            np.testing.assert_allclose(actual, target, atol=1e-6)


def test_model_specific_transport_atlas_overrides_both_legacy_modes():
    planner = SimpleNamespace(top_grasp=False,
                              transport_atlas_paths={'side': Path('/new/side.npz'), 'top': Path('/new/top.npz')})
    assert atlas_path(planner) == Path('/new/side.npz')
    planner.top_grasp = True
    assert atlas_path(planner) == Path('/new/top.npz')


def test_refined_transport_grid_covers_narrow_lift_window_without_expanding_bounds():
    original = np.arange(-.2, .001, .02)
    heights = transport_heights('side', original, np.zeros((1, 15)))
    assert np.isclose(heights.min(), original.min())
    assert np.isclose(heights.max(), original.max())
    assert np.diff(heights).max() <= .005 + 1e-9
    assert np.isclose(heights, -.145).any()
    np.testing.assert_array_equal(transport_heights('top', original, np.zeros((1, 15))), [0.])


@pytest.mark.skipif(not torch.cuda.is_available(), reason='requires cuRobo GPU collision checker')
def test_rebuilt_transport_library_has_legal_loaded_endpoints_in_refined_window():
    planner = CurrentUrdfForwardPlanner()
    planner.configure_task({'left': 22, 'right': 17})
    checker = planner.checker(planner.selected_task, payload=True)
    candidates, radius_limit = planner.transport_candidates(planner.home_values, checker, GpuValidity(checker, check_ground=True))
    assert len(candidates) > 0
    assert np.all(candidates[:, 0] >= -.2 - 1e-6)
    assert np.all(candidates[:, 0] <= 1e-6)
    assert not np.isclose(candidates[:, 0, None], np.arange(-.2, .001, .02), atol=1e-6).any()
    assert bool((planner.full_envelope_radius(checker, candidates) <= radius_limit).all())


@pytest.mark.skipif(not torch.cuda.is_available(), reason='requires cuRobo CUDA graph warmup')
def test_lm_warmup_restores_scene_and_shares_long_solver_without_caching_results():
    planner = CurrentUrdfForwardPlanner()
    snapshot = planner.snapshot
    selected = dict(planner.selected_task)
    targets = {name: {key: np.asarray(value).copy() for key, value in pose.items()}
               for name, pose in planner.target_poses.items()}
    report = planner.warmup_resources()
    assert planner.snapshot is snapshot
    assert planner.selected_task == selected
    for name, pose in targets.items():
        for key, value in pose.items():
            np.testing.assert_array_equal(planner.target_poses[name][key], value)
    assert report['complete'] and not report['caches_planning_results']
    assert [event['batch_points'] for event in report['events']] == [16, 64]
    assert planner.endpoint_ik_cache[('lm', True)] is planner.cartesian_engine.solvers[64]
    assert planner.warmup_resources() is report
