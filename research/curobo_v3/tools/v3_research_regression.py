import argparse
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy

from curobo_core.contracts import PlannerAssets, ACTIVE_JOINTS
from v3_mixed_placement_planner import AnalyticContacts


def audit(result, assets):
    assert result['success'], result.get('blocked_details')
    frames = np.asarray(result['frames'])
    names = result['joint_names']
    assert len(frames) == len(result['phases']) == len(result['payload'])
    assert np.isfinite(frames).all()
    assert set(('home','approach','contact_approach','attach','extract','transport','turn_loaded','release','turn_empty','return_home')) <= set(result['phases'])
    arm_indices = [names.index(name) for name in ACTIVE_JOINTS[1:]]
    assert np.rad2deg(np.abs(np.diff(frames[:, arm_indices], axis=0))).max() <= .50001
    assert result['final_home_error'] < 1e-6
    expected_base = .3 if result['top_grasp'] else 0
    assert abs(frames[-1, names.index('base_x')] - expected_base) < 1e-6
    urdf = yourdfpy.URDF.load(assets.mobile_robot_config.parent / 'model/robot_meter_meshes_mobile18.urdf', load_meshes=False)
    index = result['phases'].index('turn_loaded') - 1
    row = dict(zip(names, frames[index]))
    urdf.update_cfg(np.asarray([row.get(name, 0.0) for name in urdf.actuated_joint_names]))
    errors = {}
    for side in ('left','right'):
        frame = side + '_tool0'
        actual = urdf.get_transform(frame, urdf.base_link)
        target = result['placement_tool_poses'][frame]
        position = np.asarray(target['position']) + (expected_base, 0, 0)
        rotation = Rotation.from_quat(np.roll(target['quaternion'], -1)).as_matrix()
        position_error = float(np.linalg.norm(actual[:3, 3] - position))
        orientation_error = float(Rotation.from_matrix(actual[:3, :3].T @ rotation).magnitude())
        assert position_error <= .002 and orientation_error <= np.deg2rad(1)
        attached = actual @ np.asarray(result['tool_to_box'][side])
        assert np.isfinite(attached).all()
        errors[side] = {'position_error_mm': position_error * 1000,
                        'orientation_error_deg': float(np.rad2deg(orientation_error)),
                        'attached_box_pose': attached.tolist()}
    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--repeat', type=int, default=1)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    assets = PlannerAssets()
    planner = AnalyticContacts(assets)
    assert len(planner.task_options) == 94
    summaries = []
    for repeat in range(args.repeat):
        for boxes in ({'left':24,'right':20}, {'left':9,'right':1}, {'left':22,'right':17}):
            started = time.perf_counter()
            result = planner.full_cycle(boxes, lambda message: print(message, flush=True))
            result['regression_wall_ms'] = (time.perf_counter() - started) * 1000
            name = f"{repeat}_L{boxes['left']}_R{boxes['right']}.json"
            (args.output_dir / name).write_text(json.dumps(result, ensure_ascii=False))
            errors = audit(result, assets)
            summary = {key: result.get(key) for key in ('task','success','total_ms','final_home_error','selected_candidate','pipeline_timing_ms','validation_detail_ms','analytic_contact_generation','turn_collision_validation')}
            summary['analytic_contact_generation'].pop('counts_by_updown', None)
            summary.update(frame_count=len(result['frames']), fk_audit=errors, atlas_validation=planner.atlas_validation)
            summaries.append(summary)
            (args.output_dir / 'summary.json').write_text(json.dumps(summaries, ensure_ascii=False, indent=2) + '\n')
            print(json.dumps({'task': boxes,'success': True,'repeat': repeat,'total_ms': result['total_ms']}), flush=True)


if __name__ == '__main__':
    main()
