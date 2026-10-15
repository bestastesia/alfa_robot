import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import torch
import yourdfpy

from curobo_core.contracts import PlannerAssets
from curobo_core.scene import Pose
from v3_batched_loaded_search import GpuValidity, loaded_robot
from v3_conveyor_regression import audit_conveyor_geometry
from v3_single_arm_planner import SingleArmPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def validate(result, planner, model):
    boxes = result['task']
    planner.configure_task(boxes)
    original = planner.snapshot
    names = result['joint_names']
    rows = np.asarray(result['frames'])
    values = rows[:, [names.index(name) for name in ACTIVE_JOINTS]]
    phases = np.asarray(result['phases'])
    assert result['success'] and result['final_home_error'] < 1e-6
    assert not result['rotation_quality_hard_gate']
    assert np.rad2deg(np.abs(np.diff(values[:, 1:], axis=0))).max() < .50001
    counts = {}
    for phase, payload in [('approach', False), ('contact_approach', False), ('extract', False), ('transport', True),
                           ('conveyor_place', True), ('return_home', False)]:
        indices = np.flatnonzero(phases == phase)
        assert len(indices), phase
        if phase in ('conveyor_place', 'return_home'):
            row = rows[indices[0]]
            yaw = row[names.index('base_yaw')]
            base = Pose((row[names.index('base_x')] - planner.base_advance, row[names.index('base_y')], 0.),
                        (np.cos(yaw / 2), 0., 0., np.sin(yaw / 2)))
            planner.snapshot = replace(original, state=original.state.with_positions(dict(zip(names, row)), base))
        else:
            planner.snapshot = original
        checker = planner.checker(boxes, payload=payload)
        validity = planner.target_validity(checker) if phase == 'approach' else GpuValidity(checker, check_ground=True)
        mask = validity.mask(torch.tensor(values[indices], device='cuda', dtype=torch.float32))
        assert bool(mask.all()), (boxes, phase, int((~mask).sum()))
        counts[phase] = len(indices)
    planner.snapshot = original
    empty = planner.checker(boxes)
    loaded = planner.checker(boxes, payload=True)
    assert loaded.kinematics.total_spheres - empty.kinematics.total_spheres == 40
    extracted = values[phases == 'extract']
    side = next(iter(boxes))
    poses = []
    for row in extracted:
        planner.fk(row)
        poses.append(planner.fk_robot.get_transform(side + '_tool0', planner.fk_robot.base_link).copy())
    poses = np.asarray(poses)
    line_error = np.linalg.norm(poses[:, 1:3, 3] - poses[0, 1:3, 3], axis=1).max()
    assert line_error < .002 and np.ptp(extracted[:, 0]) < 1e-6
    assert np.max((Rotation.from_matrix(poses[0, :3, :3]).inv() * Rotation.from_matrix(poses[:, :3, :3])).magnitude()) < np.deg2rad(1)
    transport_index = np.flatnonzero(phases == 'transport')[-1]
    mapping = dict(zip(names, rows[transport_index]))
    model.update_cfg(np.array([mapping.get(name, 0.) for name in model.actuated_joint_names]))
    configured = loaded_robot(planner.robot, planner.box_fit, attachments=planner.task_attachments(boxes))
    maximum = 0.
    for link, balls in configured['kinematics']['collision_spheres'].items():
        transform = model.get_transform(link, model.base_link)
        for sphere in balls:
            if sphere['radius'] > 0:
                center = transform[:3, :3] @ sphere['center'] + transform[:3, 3]
                maximum = max(maximum, np.linalg.norm(center[:2] - model.get_transform('base_footprint', model.base_link)[:2, 3]) + sphere['radius'])
    gpu_radius = result['transport_endpoint_envelope']['radius_m']
    assert maximum <= 1.05 and abs(maximum - gpu_radius) < 1e-4
    release_index = list(phases).index('release')
    mapping = dict(zip(names, rows[release_index]))
    model.update_cfg(np.array([mapping.get(name, 0.) for name in model.actuated_joint_names]))
    actual = model.get_transform(side + '_tool0', model.base_link) @ np.asarray(result['tool_to_box'][side])
    target = result['conveyor_box_targets'][side]
    assert np.linalg.norm(actual[:3, 3] - target['position']) < .002
    assert (Rotation.from_quat(np.roll(target['quaternion_wxyz'], -1)).inv() * Rotation.from_matrix(actual[:3, :3])).magnitude() < np.deg2rad(1)
    release = next(event for event in result['predicted_scene_events'] if event['phase'] == 'release')
    assert not release['snapshot']['attachments']
    audit_conveyor_geometry(release['snapshot'], result['base_advance_m'])
    return {'validated_frames': counts, 'payload_added_spheres': 40, 'line_error_mm': float(line_error * 1000),
            'independent_envelope_m': float(maximum), 'gpu_envelope_m': gpu_radius}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    assets = PlannerAssets()
    planner = SingleArmPlanner(assets)
    planner.warmup_resources()
    assert len(planner.task_options) == 119
    model = yourdfpy.URDF.load(assets.mobile_robot_config.parent / 'model/robot_meter_meshes_mobile18.urdf', load_meshes=False)
    summaries = []
    for box_id in range(25):
        boxes = {'left' if box_id % 5 >= 3 else 'right': box_id}
        planner.configure_task(boxes)
        result = planner.full_cycle(boxes, lambda text: print(text, flush=True))
        (args.output_dir / f'{next(iter(boxes))}{box_id:02d}.json').write_text(json.dumps(result, ensure_ascii=False))
        if not result['success']:
            if boxes != {'right': 2} or result.get('blocked_stage') != '接触IK':
                raise SystemExit(f"Unexpected failure: {boxes}: {result.get('blocked_details')}")
            summaries.append({'task': boxes, 'success': False, 'known_boundary': True,
                              'stage': result['blocked_stage'], 'enumeration': result.get('single_contact_enumeration')})
        else:
            summaries.append({'task': boxes, 'success': True, 'total_ms': result['total_ms'], 'audit': validate(result, planner, model)})
        print(json.dumps(summaries[-1], ensure_ascii=False), flush=True)
    planner.configure_task({'left': 18, 'right': 22})
    dual = planner.full_cycle({'left': 18, 'right': 22})
    (args.output_dir / 'dual_L18_R22.json').write_text(json.dumps(dual, ensure_ascii=False))
    assert dual['success'] and not dual['rotation_quality_hard_gate']
    (args.output_dir / 'summary.json').write_text(json.dumps({'single_tasks': summaries,
        'dual_regression': {'success': dual['success'], 'total_ms': dual['total_ms']}}, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
