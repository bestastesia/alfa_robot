import ast
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
        'tools/v3_predicted_selected_lm.py',
        'tools/v3_multiroot_full_cycle_planner.py','config/rezero_home.json'):
        path=ROOT/relative
        baseline=subprocess.check_output(['git','show',f'92bd8cdb:research/curobo_v3/{relative}'],cwd=repository)
        if relative == 'tools/v3_contact_validity.py':
            # New physical side/offset grasps require an explicit face normal.
            # Restore ONLY the legacy default expression before comparing the entire collision module.
            before,after=ast.parse(baseline),ast.parse(path.read_text())
            old_normal=next(n for n in ast.walk(before) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='inward' for t in n.targets))
            new_normal=next(n for n in ast.walk(after) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='inward' for t in n.targets))
            assert isinstance(new_normal.value,ast.IfExp)
            assert ast.unparse(new_normal.value.test)=="'inward_normal' in target"
            assert ast.unparse(new_normal.value.body)=="torch.as_tensor(target['inward_normal'], device=position.device, dtype=position.dtype)"
            assert ast.dump(new_normal.value.orelse,include_attributes=False)==ast.dump(old_normal.value,include_attributes=False)
            new_normal.value=old_normal.value
            assert ast.dump(before,include_attributes=False)==ast.dump(after,include_attributes=False),relative
        else:
            assert path.read_bytes()==baseline,relative


def test_lm_performance_change_preserves_pr82_solver_and_gates():
    # This file is no longer byte-identical: FK/target preparation is optimized.
    # Keep the old solver budgets, acceptance logic, selection and output contract
    # explicitly anchored to PR82 instead of dropping the collision regression.
    baseline=subprocess.check_output(['git','show','ee6dba3:research/curobo_v3/tools/v3_lm_conveyor.py'],cwd=ROOT.parents[1])
    before=ast.parse(baseline); after=ast.parse((ROOT/'tools/v3_lm_conveyor.py').read_text())
    def method(tree,name):
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SequentialPlacementLm')
        return next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
    assert ast.dump(method(before,'__init__'))==ast.dump(method(after,'__init__'))
    def gates(tree):
        result=[]
        for node in ast.walk(method(tree,'plan')):
            if isinstance(node,ast.Return) and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Name) and node.value.func.id=='dict':
                result.append(ast.dump(node))
            if isinstance(node,(ast.Assign,ast.AugAssign)):
                targets=node.targets if isinstance(node,ast.Assign) else [node.target]
                if any(isinstance(target,ast.Name) and target.id in
                       ('pose_ok','continuous','legal','order','noise','seeds') for target in targets):
                    result.append(ast.dump(node))
            if isinstance(node,ast.If) and ast.unparse(node.test) in ('mask.all()', 'selected is None'):
                # Only the local progress-array variable was renamed to avoid shadowing.
                normalized=ast.parse(ast.unparse(node).replace('edge_fractions','fractions'))
                result.append(ast.dump(normalized))
        return sorted(result)
    assert gates(before)==gates(after)


def test_no_legacy_generator_dependency_in_new_runtime():
    for relative in ('tools/curobo_core/lm_refinement.py','tools/v3_lm_refinement.py'):
        source=(ROOT/relative).read_text()
        assert 'v3_straight_joint_refine_experiment' not in source
        assert 'raw_lm.npz' not in source
        assert '/mnt/mydisk' not in source


def test_process_observation_does_not_change_native_physical_predicate():
    tree = ast.parse((ROOT/'tools/v3_two_box_timing.py').read_text())
    value = next(n.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == 'physical_success' for t in n.targets))
    expected = ast.parse("bool(geometry_valid and stage['array_checks']['valid'] and native['dynamic_valid'] "
        "and stage['independent_rnea']['valid'] and seam['valid'] "
        "and stage.get('trajectory_optimization', {}).get('solver_success', True))", mode='eval').body
    assert ast.dump(value) == ast.dump(expected)
