import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy

from curobo_core.contracts import PlannerAssets
from v3_conveyor_cycle_planner import ConveyorCyclePlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def audit_conveyor_geometry(snapshot, base_advance):
    from curobo_core.adapter import pose_matrix
    from curobo_core.conveyor import conveyor_pair
    from curobo_core.scene import Pose, SceneSnapshot
    expected = conveyor_pair(pose=Pose((-2.3 - base_advance, 0., 0.))).world_parts()
    snapshot = SceneSnapshot.from_dict(snapshot)
    parts = [item for item in (*snapshot.objects, *snapshot.meshes) if item.object_id.startswith('conveyor_pair/')]
    assert {item.object_id for item in parts} == {item.object_id for item in expected}
    for original in expected:
        actual = next(item for item in parts if item.object_id == original.object_id)
        vertices = np.asarray(original.vertices_m)
        lower, upper = vertices.min(0), vertices.max(0)
        if hasattr(actual, 'vertices_m'):
            assert np.allclose(actual.vertices_m, vertices) and actual.faces == original.faces
            assert np.allclose(pose_matrix(actual.pose), pose_matrix(original.pose))
        else:
            assert len(np.unique(vertices, axis=0)) == 8 and len(original.faces) == 12
            assert np.all(np.isclose(vertices, lower) | np.isclose(vertices, upper))
            assert np.allclose(actual.dimensions_m, upper - lower)
            transform = pose_matrix(original.pose)
            transform[:3, 3] += transform[:3, :3] @ ((lower + upper) / 2)
            assert np.allclose(pose_matrix(actual.pose), transform)
    return {'parts': len(parts), 'exact_frozen_stl_equivalent': True}


def audit(result, assets):
    if not result['success']:
        raise AssertionError(result.get('blocked_details'))
    names = result['joint_names']
    phases = result['phases']
    rows = np.asarray(result['frames'])
    assert len(rows) == len(phases) == len(result['payload'])
    assert np.isfinite(rows).all()
    assert 'conveyor_retreat' not in phases and 'turn_loaded' not in phases and 'turn_empty' not in phases
    begin = phases.index('base_reposition')
    release = phases.index('release')
    home = phases.index('return_home')
    returned = phases.index('base_return')
    assert begin < release < home < returned
    base_indices = [names.index(name) for name in ('base_x', 'base_y', 'base_yaw')]
    assert np.max(np.abs(rows[begin:returned, base_indices] - rows[begin, base_indices])) < 1e-9
    arms = [names.index(name) for name in ACTIVE_JOINTS[1:]]
    assert np.rad2deg(np.abs(np.diff(rows[:, arms], axis=0))).max() <= .50001
    assert result['final_home_error'] < 1e-6
    release_event = next(event for event in result['predicted_scene_events'] if event['phase'] == 'release')
    assert release_event['frame_index'] == release
    assert not release_event['snapshot']['attachments']
    assert not {f'wall_box_{box:02d}' for box in result['task'].values()} & {item['object_id'] for item in release_event['snapshot']['objects']}
    geometry = audit_conveyor_geometry(release_event['snapshot'], result.get('base_advance_m', 0.))
    model = yourdfpy.URDF.load(assets.mobile_robot_config.parent / 'model/robot_meter_meshes_mobile18.urdf', load_meshes=False)
    cfg = dict(zip(names, rows[release]))
    model.update_cfg(np.asarray([cfg.get(name, 0.) for name in model.actuated_joint_names]))
    errors = {}
    for side in ('left', 'right'):
        matrix = model.get_transform(side + '_tool0', model.base_link) @ np.asarray(result['tool_to_box'][side])
        target = result['conveyor_box_targets'][side]
        position = np.linalg.norm(matrix[:3, 3] - target['position'])
        orientation = (Rotation.from_quat(np.roll(target['quaternion_wxyz'], -1)).inv() * Rotation.from_matrix(matrix[:3, :3])).magnitude()
        assert position <= .002 and orientation <= np.deg2rad(1)
        assert np.allclose(target['position'], [-1.45, -.425 if side == 'left' else .425, 1.182], atol=.002)
        errors[side] = {'position_mm': float(position * 1000), 'orientation_deg': float(np.rad2deg(orientation))}
    return {'passed': True, 'placement_fk': errors, 'base_frozen': True, 'release_home_navigation_order': True,
            'conveyor_geometry': geometry}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--replay', type=Path, nargs='+')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    assets = PlannerAssets()
    results = []
    if args.replay:
        results = [json.loads(path.read_text()) for path in args.replay]
    else:
        planner = ConveyorCyclePlanner(assets)
        for boxes in ({'left': 18, 'right': 16}, {'left': 4, 'right': 2}):
            result = planner.full_cycle(boxes, lambda message: print(message, flush=True))
            path = args.output_dir / f"L{boxes['left']}_R{boxes['right']}.json"
            path.write_text(json.dumps(result, ensure_ascii=False))
            results.append(result)
            if not result['success']:
                raise SystemExit(f"Stopped: {result.get('blocked_stage')}: {result.get('blocked_details')}")
    summaries = [{'task': result['task'], 'total_ms': result['total_ms'], 'home_error': result['final_home_error'],
                  'goal_ik': result['conveyor_goal_ik'], 'place_rrt': result['conveyor_search_stats'],
                  'home_rrt': result['conveyor_home_search_stats'], 'audit': audit(result, assets)} for result in results]
    (args.output_dir / 'summary.json').write_text(json.dumps(summaries, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(summaries, ensure_ascii=False))


if __name__ == '__main__':
    main()
