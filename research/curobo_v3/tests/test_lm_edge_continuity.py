import copy
import gzip
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))


def test_l4r5_accepts_curved_edge_with_loaded_collision_and_continuity_checks():
    import torch
    from v3_batched_loaded_search import GpuValidity
    from v3_frozen_cycle import FrozenCyclePlanner
    from v3_frozen_cycle_validate import check_cycle
    from v3_lm_conveyor import prepare_case, WEIGHTS, box_poses, interpolate_boxes, reference_errors, SequentialPlacementLm

    path = Path(__file__).resolve().parents[1] / 'generated/wrist_refit_lm_continuity_20261101/076.json.gz'
    if not path.exists() or not torch.cuda.is_available():
        pytest.skip('recorded regression and CUDA required')
    result = json.loads(gzip.decompress(path.read_bytes()))
    assert result['success']
    lm = result['conveyor_place_lm']
    assert lm['success'] and lm['completed_progress'] == 1.
    assert lm['edge_pose_policy'] == 'diagnostic_only'
    assert lm['max_reference_position_error_m'] > .002
    check_cycle(result, result['visual_home'])
    replay = copy.deepcopy(result)
    first = result['phases'].index('conveyor_place')
    navigation = result['phases'].index('base_reposition')
    replay['frames'][first] = result['frames'][navigation]
    planner = FrozenCyclePlanner()
    case = prepare_case(planner, replay)
    actual = SequentialPlacementLm(planner, active_sides=tuple(sorted(result['task']))).plan(case)
    assert actual['success'] and actual['edge_pose_policy'] == 'diagnostic_only'
    rows = np.asarray(actual['frames'])
    progress = np.asarray(actual['progress'])
    assert len(rows) == len(progress)
    assert np.max(np.abs(np.diff(rows, axis=0)*WEIGHTS)) <= np.deg2rad(.251)
    validity = GpuValidity(case['checker'], check_ground=True, scene_penetration_tolerance_m=.001)
    assert validity.mask(torch.tensor(rows, device='cuda', dtype=torch.float32).contiguous()).all()
    position, angle = reference_errors(box_poses(case['checker'], rows, case['attachments']), interpolate_boxes(case['starts'], case['ends'], progress))
    assert position.max() > .002
    for fraction in np.linspace(0., 1., lm['reference_samples']):
        index = np.argmin(abs(progress-fraction))
        assert abs(progress[index]-fraction) < 1e-8
        assert position[index] <= lm['node_position_tolerance_m'] + 1e-6
        assert angle[index] <= lm['node_orientation_tolerance_deg'] + 1e-3
