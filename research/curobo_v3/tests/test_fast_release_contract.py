import hashlib
import json
from pathlib import Path
import subprocess

import pytest

from v3_fast_preflight import ROOT, verify_hashes


def test_release_manifest_matches_frozen_assets():
    lock=json.loads((ROOT/'config/fast_release_manifest.json').read_text())
    verify_hashes(ROOT,lock['assets_sha256'])
    assert lock['baseline']['pr']==80
    assert lock['upstream_src_comparison']['verified_files']==434
    assert lock['upstream_src_comparison']['mismatch']==[]


def test_dependency_mismatch_is_not_silent(tmp_path):
    path=tmp_path/'model.json'
    path.write_text('frozen')
    expected={'model.json':hashlib.sha256(path.read_bytes()).hexdigest()}
    verify_hashes(tmp_path,expected)
    path.write_text('changed')
    with pytest.raises(ValueError,match='mismatch'):verify_hashes(tmp_path,expected)


def test_pr80_collision_sources_remain_identical():
    repository=ROOT.parents[1]
    for relative in ('tools/v3_fixed_body_boxes.py','tools/v3_base_cuboid_layer.py',
        'tools/v3_persistent_collision_planner.py','tools/v3_contact_validity.py',
        'tools/v3_predicted_selected_lm.py','tools/v3_lm_conveyor.py',
        'tools/v3_multiroot_full_cycle_planner.py','config/rezero_home.json'):
        path=ROOT/relative
        baseline=subprocess.check_output(['git','show',f'92bd8cdb:research/curobo_v3/{relative}'],cwd=repository)
        assert path.read_bytes()==baseline,relative


def test_no_legacy_generator_dependency_in_new_runtime():
    for relative in ('tools/curobo_core/lm_refinement.py','tools/v3_lm_refinement.py'):
        source=(ROOT/relative).read_text()
        assert 'v3_straight_joint_refine_experiment' not in source
        assert 'raw_lm.npz' not in source
        assert '/mnt/mydisk' not in source
