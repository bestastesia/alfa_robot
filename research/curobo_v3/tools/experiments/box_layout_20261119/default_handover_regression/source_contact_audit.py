"""CPU-only source-contact audit. API: audit_record(record, raw_request); CLI: --summary.
Only reads inputs; all output is confined to this script's directory. No planner imports.
"""
import os
import sys
sys.dont_write_bytecode = True
os.environ['OPENBLAS_NUM_THREADS'] = '1'
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import yourdfpy

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT/'generated/two_box_layout/actual_suction_contract_audit_20261124'
BASE = ROOT/'generated/two_box_layout'
sys.path.insert(0, str(ROOT/'tools'))
from curobo_core.suction_grasp import contact_cap_spheres
from curobo_core.box_pick import actual_owner_ids

NAMES = ['updown']+[f'{s}_joint{i}' for s in ('left', 'right') for i in range(1, 8)]
IDS = {'left': 'wall_box_00', 'right': 'wall_box_01'}
DIMS = {'40x40x30': [.3, .4, .4], '46.5x25.5x58.2': [.255, .465, .582]}
MODELS = [ROOT/'generated/fast_runtime_assets'/f'fixed.{ext}' for ext in ('urdf', 'yaml')]
ROBOT = yourdfpy.URDF.load(MODELS[0], load_meshes=False, load_collision_meshes=False)
SAMPLES = [
    ('takeover_20261124/default_y85/result.json', 'default_handover_final36_20261124/requests/32_broader_same_side_y85.json'),
    ('joint_contract_20261124_top_handover/attempt_01/record.json', 'joint_contract_20261124_top_handover/y85_top70_high130_request.json'),
    ('joint_contract_20261124_top_handover/attempt_04/record.json', 'joint_contract_20261124_top_handover/y85_both_top20_high70_request.json'),
    ('role_generalization_20261124/default_admission_attempt_07/default/result.json', 'role_generalization_20261124/raw_request.json'),
    ('sku_order_handover_20261124/default_attempt_01/default/result.json', 'sku_order_handover_20261124/raw_request.json'),
]


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def load(path):
    return json.loads(Path(path).read_text())


def contact(raw, arm, q, offset=(0., 0., 0.)):
    """FK and cap footprint against the original axis-aligned physical box."""
    if arm not in IDS or not np.isfinite(q).all() or np.shape(q) != (15,):
        raise ValueError('invalid actual arm or native first q')
    if raw.get('side_min_height_m', .79) != .79:
        raise ValueError('required height differs from the fixed 0.79m contract')
    center = np.asarray(raw['center_m'], dtype=float)
    dims = np.array(DIMS[raw['size']])
    if raw.get('swap_length_height', False):
        dims[[1, 2]] = dims[[2, 1]]
    offset = np.asarray(offset, dtype=float)
    if center.shape != (3,) or offset.shape != (3,) or not np.isfinite([center, offset]).all():
        raise ValueError('invalid raw center/offset')
    ROBOT.update_cfg(dict(zip(NAMES, q)))
    tool = ROBOT.get_transform(arm+'_tool0', ROBOT.base_link)
    C = np.eye(4)
    C[:3, 3] = center  # Keep raw box axes/world pose; never derive C from an attachment.
    T = np.linalg.inv(C)@tool
    normal = T[:3, 2]
    axis = int(np.argmax(np.abs(normal)))
    sign = float(np.sign(normal[axis]))
    angle = float(np.degrees(np.arccos(np.clip(abs(normal[axis]), -1., 1.))))
    actual = 'Top' if axis == 2 and sign < 0 else ('Side' if axis < 2 else 'Bottom')
    expected = 'Top' if center[2] < .79 else 'Side'
    half = dims/2
    face_point = np.zeros(3)
    face_point[axis] = -sign*half[axis]
    plane = float(abs(T[axis, 3]-face_point[axis]))
    tcp_error = float(np.linalg.norm(T[:3, 3]-(face_point+offset)))
    caps = contact_cap_spheres(arm)
    centers = caps[:, :3]@T[:3, :3].T+T[:3, 3]
    tangent = [i for i in range(3) if i != axis]
    excess = np.abs(centers[:, tangent])+caps[:, 3, None]-half[tangent]
    gates = dict(source_mode=actual == expected, normal_angle_1deg=angle <= 1.,
                 tangent_offset=bool(abs(offset[axis]) <= 1e-12),
                 tcp_source_position_2mm=tcp_error <= .002,
                 inherited_contact_plane_1mm=plane <= .001,
                 cap_footprint_1mm=float(excess.max()) <= .001)
    return dict(valid=all(gates.values()), rejected_layers=[k for k, v in gates.items() if not v],
                raw_center_m=center.tolist(), raw_dimensions_m=dims.tolist(), raw_side_center_z_m=float(center[2]),
                expected_mode=expected, actual_mode=actual, actual_arm=arm,
                normal_box=normal.tolist(), normal_angle_deg=angle,
                actual_T_box_to_tool=T.tolist(), actual_T_world_tool=tool.tolist(),
                selected_tangent_offset_m=offset.tolist(), expected_face_point_box_m=(face_point+offset).tolist(),
                tcp_source_position_error_m=tcp_error, contact_plane_error_m=plane,
                cap_count=len(caps), cap_spheres_tool=caps.tolist(), cap_centers_box=centers.tolist(),
                tangent_axes=tangent, footprint_excess_m=float(excess.max()),
                footprint_excess_per_cap_m=excess.tolist(), gates=gates)


def audit_record(record, raw_request):
    """Reopen real extract position[0]; identify each physical ID's first attachment."""
    record, raw_request = Path(record), Path(raw_request)
    d, request = load(record), load(raw_request)
    raw = {IDS[b['side']]: b for b in request['boxes']}
    if len(raw) != 2 or len(request['boxes']) != 2 or request.get('frame_id', 'world') != 'world':
        raise ValueError('need exactly two distinct physical labels in original world request')
    if d['joint_names'] != NAMES or d.get('stage_interface', {}).get('joint_names', NAMES) != NAMES:
        raise ValueError('native joint order differs from updown,L1..7,R1..7')
    if not np.allclose(d.get('world_from_robot_base', np.eye(4)), np.eye(4), atol=1e-12, rtol=0):
        raise ValueError('nonidentity world/base requires explicit FK composition')
    workflow = d.get('workflow', 'synchronous')
    native = d['timing']['stages']
    stages = d['stage_interface']['stages'] if workflow == 'handover_same_side' else native
    if workflow not in ('handover_same_side', 'ordered_upper_first', 'synchronous'):
        raise ValueError('unsupported workflow '+workflow)
    seen, measurements = set(), []
    for i, stage in enumerate(stages):
        sid = stage.get('stage_id', stage.get('phase'))
        if workflow == 'synchronous':
            if sid != 'extract':
                continue
            owners = actual_owner_ids(d)
            attachments = [dict(object_id=pid, parent_link=arm+'_tool0') for arm, pid in owners.items()]
            owner_source = 'record.owner_to_physical_id actual arm bijection (legacy defaults to label order)'
        else:
            scene = stage['scene'] if workflow == 'handover_same_side' else stage['physical_scene']
            if not np.allclose(scene['state']['base_pose']['position'], [0, 0, 0], atol=1e-12, rtol=0) or scene['state']['base_pose']['quaternion_wxyz'] != [1., 0., 0., 0.]:
                raise ValueError('nonidentity stage world/base')
            attachments = scene['attachments']
            owner_source = 'stage_interface.scene.attachments.parent_link' if workflow == 'handover_same_side' else 'native.physical_scene.attachments.parent_link'
        for att in attachments:
            pid = att['object_id']
            if pid in seen:
                continue  # Handover/support switch is not a new source pickup.
            if pid not in raw or 'extract' not in sid:
                raise ValueError('unexpected first attachment '+str((sid, pid)))
            matches = [s for s in native if s.get('stage_id', s.get('phase')) == sid]
            if len(matches) != 1:
                raise ValueError('native stage_id must match uniquely: '+sid)
            path = Path(matches[0]['trajectory_file'])
            with np.load(path, allow_pickle=False) as arrays:
                q = arrays['position'][0].astype(float)
                if 'joint_names' in arrays and arrays['joint_names'].tolist() != NAMES:
                    raise ValueError('array joint order mismatch')
            parent = att['parent_link']
            if parent not in ('left_tool0', 'right_tool0'):
                raise ValueError('unexpected actual parent_link '+parent)
            arm = parent.removesuffix('_tool0')
            label = raw[pid]['side']
            offset = d['task'][arm if workflow == 'synchronous' else label].get('suction_offset_m', [0., 0., 0.])
            measurement = contact(raw[pid], arm, q, offset)
            measurement.update(physical_id=pid, physical_label=label, stage_id=sid, stage_index=i,
                               owner_provenance=owner_source, parent_link=parent,
                               native_array=str(path), native_array_sha256=sha(path), native_position_0=q.tolist())
            measurements.append(measurement)
            seen.add(pid)
    if seen != set(raw) or len(measurements) != 2:
        raise ValueError('must independently observe exactly one source pickup per physical ID')
    return dict(valid=all(m['valid'] for m in measurements), workflow=workflow,
                record=str(record), record_sha256=sha(record), raw_request=str(raw_request),
                raw_request_sha256=sha(raw_request), models_sha256={str(p): sha(p) for p in MODELS},
                measurements=measurements)


def checked(record, request):
    try:
        return audit_record(record, request)
    except Exception as error:
        return dict(valid=False, record=str(record), raw_request=str(request),
                    rejected_layers=['input_or_provenance'], error=f'{type(error).__name__}: {error}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--summary', type=Path, help='One snapshot of final36 run_001/summary.json; passed slots only.')
    parser.add_argument('--self-check', action='store_true')
    parser.add_argument('--output', default=None, help='Report filename, confined to this directory.')
    args = parser.parse_args()
    report = dict(scope='CPU actual first-source contact only; no collision, dynamics, RNEA, CUDA, hardware or universal proof.',
                  interpreter=sys.executable, height_rule='raw original side center z < 0.79 -> Top; >= 0.79 -> Side',
                  tolerances=dict(normal_deg=1., tcp_source_m=.002, contact_plane_m=.001, cap_footprint_m=.001),
                  auditor_sha256=sha(__file__), cap_reader_sha256=sha(ROOT/'tools/curobo_core/suction_grasp.py'))
    rows = []
    if args.summary:
        summary_bytes = args.summary.read_bytes()
        summary = json.loads(summary_bytes)
        catalog_path = Path(summary['catalog_file'])
        catalog = load(catalog_path)
        workers = {r['index']: r for r in summary['cases']}
        report.update(summary=str(args.summary), summary_sha256=hashlib.sha256(summary_bytes).hexdigest(),
                      summary_state=summary['state'], source_digest=summary['source_digest'],
                      catalog=str(catalog_path), catalog_sha256=sha(catalog_path))
        for case in catalog['cases']:
            worker = workers.get(case['index'], {})
            row = dict(index=case['index'], name=case['name'], worker_state=worker.get('state', 'missing'), state='unverified', valid=False)
            if worker.get('state') == 'passed':
                try:
                    if worker['name'] != case['name'] or sha(case['request_file']) != case['request_sha256'] or load(case['request_file']) != case['request']:
                        raise ValueError('catalog/raw request mismatch')
                    row.update(checked(worker['output'], case['request_file']))
                    row['state'] = 'verified' if row['valid'] else 'rejected'
                except Exception as error:
                    row.update(state='rejected', error=str(error))
            rows.append(row)
    else:
        rows = [checked(BASE/record, BASE/request) for record, request in SAMPLES]
    if args.self_check:
        approved = next(r for r in rows if r['valid'])
        m = approved['measurements'][0]
        raw = next(b for b in load(approved['raw_request'])['boxes'] if b['side'] == m['physical_label'])
        q = np.asarray(m['native_position_0'])
        assert contact(raw, m['actual_arm'], q, m['selected_tangent_offset_m'])['valid']
        moved = copy.deepcopy(raw)
        moved['center_m'][2] += .05
        assert not contact(moved, m['actual_arm'], q, m['selected_tangent_offset_m'])['valid']
        wrong_arm = 'right' if m['actual_arm'] == 'left' else 'left'
        assert not contact(raw, wrong_arm, q, m['selected_tangent_offset_m'])['valid']
        changed = dict(raw, side_min_height_m=1.5)
        try:
            contact(changed, m['actual_arm'], q)
        except ValueError:
            pass
        else:
            raise AssertionError('tampered required height accepted')
        report['self_check'] = 'approved real q passes; raw center +50mm, swapped physical owner, required height 1.5m rejected (memory only)'
    if args.summary:
        report['all_slots_verified'] = bool(rows) and all(r.get('state') == 'verified' for r in rows)
    report.update(rows=rows, verified=sum(r.get('valid', False) for r in rows),
                  rejected=sum(not r.get('valid', False) and r.get('state') != 'unverified' for r in rows),
                  unverified=sum(r.get('state') == 'unverified' for r in rows),
                  prohibited_modules_imported=[m for m in ('torch', 'curobo') if m in sys.modules])
    path = (OUT/(args.output or ('batch_current.json' if args.summary else 'five_samples.json'))).resolve()
    if path.parent != OUT:
        raise ValueError('output must stay in audit directory')
    path.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: report[k] for k in ('verified', 'rejected', 'unverified', 'prohibited_modules_imported')}))
    print(path)
    return int(report['rejected'] > 0 or bool(report['prohibited_modules_imported']))


if __name__ == '__main__':
    sys.exit(main())
