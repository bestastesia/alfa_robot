"""CPU regression: wrong assigned arm must not become a physical-carton rejection."""
import ast
from dataclasses import asdict, replace
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import yourdfpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from curobo_core.adapter import matrix_pose
from curobo_core.box_pick import BoxPick, actual_owner_ids
from curobo_core.contact_reach import urdf_reach_model, contact_reach_witnesses
from curobo_core.layout_contract import carry_configuration, validate_pair
from curobo_core.ordered_pick import replay_box_poses
from curobo_core.scene import Pose, SceneObject, SceneSnapshot, RobotState
from v3_rezero_analytic import RezeroAnalytic

NAMES = ['updown']+[f'{s}_joint{i}' for s in ('left', 'right') for i in range(1, 8)]
SWAPPED = {'left': 'wall_box_01', 'right': 'wall_box_00'}


def method(path, name, namespace):
    tree = ast.parse(path.read_text())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


def test_assignment_fallback_preserves_physical_cartons(tmp_path):
    before = {n for n in sys.modules if n.split('.')[0] in ('torch', 'curobo')}
    raw = dict(frame_id='world', front_distance_m=.66, boxes=[
        dict(size=size, side=side, center_m=[1.1680520510673524, y, .79], side_min_height_m=.79)
        for side, size, y in [('left', '40x40x30', -.21625), ('right', '46.5x25.5x58.2', .21625)]])
    original = {b['side']: BoxPick(**b, carry_joint_home=True) for b in raw['boxes']}
    swapped = {s: replace(original[other], side=s) for s, other in [('left', 'right'), ('right', 'left')]}
    robot = yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
    ik = RezeroAnalytic()
    model = urdf_reach_model(robot, ik.offsets, ik.signs)
    assert len(contact_reach_witnesses(original, model, (-1., 0.))) == 2
    assert contact_reach_witnesses(swapped, model, (-1., 0.)) == []
    # Real deterministic analytic IK, no cached q or hand-tuned joint pose.
    seed = np.array([carry_configuration(-.5)[n] for n in NAMES])
    q = None
    for lift in np.round(np.arange(-1., .0001, .05), 2):
        robot.update_cfg(dict(zip(NAMES, np.r_[lift, seed[1:]])))
        carriage = np.linalg.inv(robot.get_transform('arm_carriage', robot.base_link))
        arms = [np.concatenate([ik.solve(a, carriage@swapped[s].contact_tool(), np.deg2rad(t), seed[1+7*a:8+7*a])
                                for t in range(-180, 180, 10)]) for a, s in enumerate(('left', 'right'))]
        if all(len(v) for v in arms):
            q = np.r_[lift, arms[0][0], arms[1][0]]
            break
    assert q is not None
    robot.update_cfg(dict(zip(NAMES, q)))
    for arm, pick in swapped.items():
        np.testing.assert_allclose(robot.get_transform(arm+'_tool0', robot.base_link), pick.contact_tool(), atol=1e-6)

    ns = dict(np=np, Path=Path, json=json, asdict=asdict, replace=replace, validate_pair=validate_pair)
    plan = method(ROOT/'tools/v3_box_layout_planner.py', 'plan_layout', ns)
    calls = []
    def solve(picks, *args, **kwargs):
        owners = actual_owner_ids(kwargs)
        calls.append(owners)
        assert {owners[s]: (p.size, p.center_m) for s, p in picks.items()} == {
            'wall_box_00': (original['left'].size, original['left'].center_m),
            'wall_box_01': (original['right'].size, original['right'].center_m)}
        return dict(success=owners == SWAPPED, task={s: asdict(p) for s, p in picks.items()}, owner_to_physical_id=owners)
    result = plan(NS(_plan_layout=solve), original, tmp_path/'fallback')
    assert calls == [actual_owner_ids({})]*3+[SWAPPED]
    assert result['requested_tasks'] == {s: asdict(p) for s, p in original.items()}
    assert len(result['grasp_trials']) == 4 and result['success']

    # Execute the real constructor/configuration bodies with only GPU construction replaced.
    ns.update(actual_owner_ids=actual_owner_ids, payload_fit=lambda fit, dims: dict(dimensions_m=dims),
              SceneObject=SceneObject, Pose=Pose, matrix_pose=matrix_pose)
    configure = method(ROOT/'tools/v3_box_layout_planner.py', 'configure_layout', ns)
    snapshot = SceneSnapshot('cpu-fixture', RobotState(tuple(NAMES), tuple(seed), 0), ())
    class Base:
        def __init__(self):
            self.original_snapshot = self.snapshot = snapshot
            self.environment = (); self.frozen_box_fit = {}
        configure_layout = configure
    ns['__name__'] = __name__
    # super() needs its real class cell: compile just this class and its actual constructor.
    tree = ast.parse((ROOT/'tools/v3_ordered_layout.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'OrderedLayoutPlanner')
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__']
    ns['BoxLayoutPlanner'] = Base
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(ROOT/'tools/v3_ordered_layout.py'), 'exec'), ns)
    owner = ns['OrderedLayoutPlanner'](swapped, SWAPPED)
    assert owner.selected_task == {'left': 1, 'right': 0}
    assert {o.object_id: o.dimensions_m for o in owner.snapshot.objects} == {
        'wall_box_00': original['left'].dimensions_m, 'wall_box_01': original['right'].dimensions_m}
    owner.configure_layout(original)
    assert owner.selected_task == {'left': 0, 'right': 1} and owner.owner_to_physical_id == actual_owner_ids({})

    # First-attachment audit and replay must identify cartons from immutable raw labels.
    result.update(request=raw, joint_names=NAMES, timing=dict(stages=[dict(phase='extract', trajectory_file=str(tmp_path/'extract.npz'))]))
    np.savez(tmp_path/'extract.npz', position=q[None], joint_names=np.array(NAMES))
    for phase in ('approach', 'extract'):
        poses = replay_box_poses(robot, result, phase)
        for label, pick in original.items():
            np.testing.assert_allclose(poses[label][:3, 3], pick.center_m, atol=1e-6)
    spec = importlib.util.spec_from_file_location('assignment_suction_audit', ROOT/'tools/experiments/box_layout_20261119/default_handover_regression/source_contact_audit.py')
    audit = importlib.util.module_from_spec(spec); spec.loader.exec_module(audit)
    (tmp_path/'result.json').write_text(json.dumps(result)); (tmp_path/'request.json').write_text(json.dumps(raw))
    check = audit.audit_record(tmp_path/'result.json', tmp_path/'request.json')
    assert check['valid'] and {m['physical_id']: m['actual_arm'] for m in check['measurements']} == {
        'wall_box_00': 'right', 'wall_box_01': 'left'}
    for bad in ({'left': 'wall_box_00', 'right': 'wall_box_00'}, {'left': 'wall_box_01'}):
        try: actual_owner_ids(dict(owner_to_physical_id=bad))
        except ValueError: pass
        else: raise AssertionError('invalid physical bijection admitted')
    assert before == {n for n in sys.modules if n.split('.')[0] in ('torch', 'curobo')}
