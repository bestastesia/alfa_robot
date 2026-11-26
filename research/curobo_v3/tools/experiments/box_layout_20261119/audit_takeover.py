"""CPU-only takeover check of saved native results; finite evidence, not workspace coverage.

Run from any directory: python /absolute/path/to/this_file.py
Native sampled collision/dynamic RNEA are retained gates, not independently repeated.
"""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import yourdfpy
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools'))
from v3_box_layout import verify_layout_result


def audit():
    previous = ROOT / 'generated/two_box_layout/joint_contract_20261123'
    batch = previous / 'default_full31'
    manifest = json.loads((previous / 'source_default_full31/manifest.json').read_text())
    changed = [name for name, digest in manifest['files'].items()
               if not (ROOT / name).is_file()
               or hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != digest]
    if changed:
        raise ValueError(f'previously validated source changed: {changed}')
    robot = yourdfpy.URDF.load(ROOT / "generated/fast_runtime_assets/fixed.urdf", load_meshes=False)
    cases = []
    expected_angles = np.deg2rad([40., -46., 0., -100., -30., -75., 90.])
    for item in json.loads((batch / 'catalog.json').read_text())['cases']:
        path = batch / item['name'] / 'result.json'
        record = json.loads(path.read_text())
        if (record['request'] != item['request'] or not record['success']
                or not record['physical_success'] or not record['task_success']):
            raise ValueError(f"original request or success gates rejected: {item['name']}")
        request = record['request']
        if (request.get('front_distance_m') != .66
                or any(b.get('side_min_height_m') != .79 for b in request['boxes'])):
            raise ValueError('fixed distance/threshold policy changed')
        if any(set(b) & {'suction_offset_m', 'suction_roll_deg', 'side_face'} for b in request['boxes']):
            raise ValueError('saved input contains manually selected grasps')
        originals = {b['side']: b for b in request['boxes']}
        upper = max(originals, key=lambda side: originals[side]['center_m'][2])
        contacts = {}
        for side, original in originals.items():
            selected = record['task'][side]
            if (not np.array_equal(selected['center_m'], original['center_m'])
                    or selected['size'] != original['size']
                    or selected.get('swap_length_height', False) != original.get('swap_length_height', False)
                    or selected['side_min_height_m'] != .79):
                raise ValueError('selected grasp changed the physical source carton or threshold')
            dims = {'40x40x30': (.3, .4, .4), '46.5x25.5x58.2': (.255, .465, .582)}[original['size']]
            if original.get('swap_length_height', False): dims = (dims[0], dims[2], dims[1])
            # Derive attachment directly from face geometry, independently of BoxPick.
            tool = np.eye(4)
            if original['center_m'][2] < .79:
                tool[:3, :3] = np.diag([-1., 1., -1.])
                tool[2, 3] = dims[2] / 2
            else:
                axis, sign, yaw = {'x-': (0, -1, 0.), 'x+': (0, 1, 180.),
                                   'y-': (1, -1, 90.), 'y+': (1, 1, -90.)}[selected['side_face']]
                tool[:3, :3] = Rotation.from_euler('z', yaw, degrees=True).as_matrix() @ np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
                tool[axis, 3] = sign * dims[axis] / 2
            tool[:3, 3] += selected['suction_offset_m']
            tool[:3, :3] = tool[:3, :3] @ Rotation.from_euler('z', selected['suction_roll_deg'], degrees=True).as_matrix()
            attachment = np.linalg.inv(tool)
            stage_id = 'upper_extract' if side == upper else 'lower_extract'
            extraction = next(stage for stage in record['timing']['stages'] if stage['phase'] == 'extract'
                              and (record.get('workflow') != 'ordered_upper_first' or stage['stage_id'] == stage_id))
            with np.load(extraction['trajectory_file'], allow_pickle=False) as arrays:
                endpoints = arrays['position'][[0, -1]].copy()
            boxes = []
            for row in endpoints:
                robot.update_cfg(dict(zip(record['joint_names'], row)))
                boxes.append(robot.get_transform(side + '_tool0', robot.base_link) @ attachment)
            source_error = float(np.linalg.norm(boxes[0][:3, 3] - original['center_m']))
            source_angle = float(Rotation.from_matrix(boxes[0][:3, :3]).magnitude())
            pull_error = float(np.linalg.norm(boxes[1][:3, 3] - boxes[0][:3, 3] + [dims[0], 0., 0.]))
            if source_error > .002 or source_angle > np.deg2rad(1.) or pull_error > .002:
                raise ValueError(f'actual source contact/extraction failed: {item["name"]}/{side}')
            contacts[side] = dict(source_error_m=source_error, source_angle_rad=source_angle,
                                  extraction_error_m=pull_error, mode='top' if original['center_m'][2] < .79 else 'side')
        # Reopen actual arrays and execute synchronous/ordered CPU rest/qva/FK checks.
        check = verify_layout_result(record)
        with np.load(record['timing']['stages'][-1]['trajectory_file'], allow_pickle=False) as arrays:
            final = arrays['position'][-1].copy()
        names = record['joint_names']
        error = max(float(np.max(np.abs(
            [final[names.index(f'{side}_joint{i}')] for i in range(1, 8)]
            - expected_angles * sign))) for side, sign in (('left', 1), ('right', -1)))
        if error > 1e-5:
            raise ValueError('actual final array disagrees with user angle table')
        updown = float(final[names.index('updown')])
        if not -1. <= updown <= 0.:
            raise ValueError('actual lift is outside URDF limits')
        cases.append(dict(case=item['name'], workflow=record.get('workflow', 'synchronous'),
                          valid=check['valid'], final_updown_m=updown, user_angle_error_rad=error,
                          original_source_checks=contacts,
                          result_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), check=check))
    if len(cases) != 31 or not all(c['valid'] for c in cases):
        raise ValueError('saved 31-case baseline is incomplete')
    output = dict(valid=True, production_source_files_unchanged=len(manifest['files']), cases=cases,
                  passed=len(cases), ordered=sum(c['workflow'] == 'ordered_upper_first' for c in cases),
                  max_user_angle_error_rad=max(c['user_angle_error_rad'] for c in cases),
                  observed_updown_m=sorted({c['final_updown_m'] for c in cases}),
                  whole_goal_complete=False,
                  scope='CPU reopening saved native arrays, input integrity, prior source hashes and exact user angles; '
                        'finite 31 cases. Original native sampled collision/dynamic RNEA gates retained; '
                        'not independently recomputed, not a proof of arbitrary legal-position coverage.')
    path = ROOT / 'generated/two_box_layout/takeover_20261124/baseline_audit.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(output, indent=2) + '\n')
    print(json.dumps({k: v for k, v in output.items() if k not in ('cases', 'scope')}, ensure_ascii=False))
    return output


if __name__ == '__main__':
    audit()
