import json
from pathlib import Path

import numpy as np
import yaml


MODEL = Path(__file__).resolve().parents[1] / 'generated/zero_model_candidate_20261018'


def test_both_planning_models_use_approved_wrist_spheres():
    approved = json.loads((MODEL / 'wrist_collision_20261101.json').read_text())
    assert set(approved['collision_spheres']) == {f'{side}_link{number}' for side in ('left', 'right') for number in (5, 6, 7)}
    for name in ('fixed.yaml', 'mobile.yaml'):
        config = yaml.safe_load((MODEL / name).read_text())['kinematics']
        for link, spheres in approved['collision_spheres'].items():
            assert config['collision_spheres'][link] == spheres
            assert np.isfinite([sphere['center'] for sphere in spheres]).all()
            assert all(sphere['radius'] > 0 for sphere in spheres)
        for side in ('left', 'right'):
            first, second = side+'_link5', side+'_link7'
            assert second not in config['self_collision_ignore'].get(first, [])
            assert first not in config['self_collision_ignore'].get(second, [])
            assert config['self_collision_buffer'][first] == 0
            assert config['self_collision_buffer'][second] == 0
