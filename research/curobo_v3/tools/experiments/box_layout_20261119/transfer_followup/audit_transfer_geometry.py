"""Independent CPU FK/identity/seam audit; no torch or GPU initialization."""
from pathlib import Path
import hashlib
import json
import sys
import importlib.util
import importlib.metadata

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy

ROOT = Path(__file__).resolve().parents[4]
BASE = ROOT / 'generated/two_box_layout/joint_contract_20261123'
OUT = ROOT / 'generated/two_box_layout/joint_contract_20261124_transfer'
sys.path.insert(0, str(ROOT / 'tools'))
from curobo_core.box_pick import BoxPick, source_slot_observation, source_slot_process_checks
from curobo_core.payload_geometry import box_sphere_depth
from curobo_core.suction_grasp import contact_cap_spheres

NAMES = ['updown'] + [f'{s}_joint{i}' for s in ('left', 'right') for i in range(1, 8)]


def source_hashes():
    manifest = json.loads((BASE / 'source_default_full31/manifest.json').read_text())
    bad = [p for p, h in manifest['files'].items()
           if not (ROOT / p).is_file() or hashlib.sha256((ROOT / p).read_bytes()).hexdigest() != h]
    assert not bad, f'existing manifest content changed: {bad}'
    return dict(existing_manifest_files=len(manifest['files']), mismatches=bad,
                original_source_stamp=manifest['source_stamp'], production_content_hashes_match=True)


def inputs():
    hashes = source_hashes()
    report = json.loads((BASE / 'transfer_owner_transport_probe.json').read_text())
    assert report['success'] and report['physical_source_box_id'] == 'right'
    box = next(b for b in report['request']['boxes'] if b['side'] == 'right')
    owner = BoxPick(**dict(box, side='left'))
    with np.load(BASE / 'transfer_owner_transport_geometry.npz', allow_pickle=False) as data:
        arrays = {k: data[k].copy() for k in data.files}
    return hashes, report, owner, arrays


def fk(robot, rows):
    result = {s: [] for s in ('left', 'right')}
    for q in rows:
        robot.update_cfg(dict(zip(NAMES, q)))
        for s in result:
            result[s].append(robot.get_transform(s + '_tool0', robot.base_link).copy())
    return {s: np.asarray(T) for s, T in result.items()}


def main():
    hashes, prior, owner, old = inputs()
    report = json.loads((OUT / 'transfer_probe.json').read_text())
    assert report['success'], report.get('failure')
    with np.load(OUT / 'transfer_geometry.npz', allow_pickle=False) as data:
        arrays = {k: data[k].copy() for k in data.files}
    assert report['physical_box_id'] == prior['physical_source_box_id'] == 'right'
    assert report['physical_box_count'] == 1 and report['physical_payload_kg'] == 10.
    assert np.array_equal(arrays['left_attachment'], owner.tool_to_box())
    for key in old:
        assert np.array_equal(arrays[key], old[key]), f'old input mutated: {key}'
    robot = yourdfpy.URDF.load(ROOT / 'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
    phases = ('approach', 'contact', 'extract', 'owner_transport', 'receiver_approach', 'support_transfer', 'donor_release', 'donor_continue')
    previous = None
    seams = {}
    for key in phases:
        q = arrays[key]
        assert q.ndim == 2 and q.shape[1] == 15 and len(q) and np.isfinite(q).all()
        seam = 0. if previous is None else float(np.max(np.abs(q[0] - previous)))
        assert seam <= 1e-5, (key, seam)
        seams[key] = seam
        previous = q[-1]
    assert np.array_equal(arrays['receiver_approach'][0], old['receiver_contact_seed'][0])
    assert np.array_equal(arrays['receiver_approach'][-1], old['receiver_contact_seed'][-1])
    first = fk(robot, arrays['extract'][[0, -1]])['left'] @ arrays['left_attachment']
    pull = first[-1, :3, 3] - first[0, :3, 3]
    assert np.linalg.norm(pull - [-.255, 0., 0.]) <= .002
    assert np.linalg.norm(first[0, :3, 3] - owner.center_m) <= .002
    previous_box = None
    pose_seams = {}
    fk_error = 0.
    all_boxes = []
    for key in phases[2:]:
        tools = fk(robot, arrays[key])
        side = 'right' if key in ('donor_release', 'donor_continue') else 'left'
        A = arrays['right_measured_attachment'] if side == 'right' else arrays['left_attachment']
        C = tools[side] @ A
        stored = arrays[key + '_box']
        fk_error = max(fk_error, float(np.max(np.abs(C - stored))))
        assert np.max(np.abs(C - stored)) <= 2e-5
        if previous_box is not None:
            delta = float(np.max(np.abs(C[0] - previous_box)))
            assert delta <= 2e-5, (key, delta)
            pose_seams[key] = delta
        previous_box = C[-1]
        if key != 'extract':
            all_boxes.append(C)
    handoff = fk(robot, arrays['support_transfer'])
    assert np.max(np.abs(np.diff(arrays['support_transfer'], axis=0))) == 0.
    actual_C = handoff['left'][-1] @ arrays['left_attachment']
    measured = np.linalg.inv(handoff['right'][-1]) @ actual_C
    assert np.max(np.abs(measured - arrays['right_measured_attachment'])) <= 2e-5
    switch_jump = float(np.max(np.abs(handoff['right'][-1] @ arrays['right_measured_attachment'] - actual_C)))
    assert switch_jump <= 2e-5
    tools = fk(robot, arrays['donor_release'])
    motion = tools['left'][:, :3, 3] - tools['left'][0, :3, 3]
    axis = -tools['left'][0, :3, 2]
    axial = motion @ axis
    lateral = np.linalg.norm(motion - axial[:, None] * axis, axis=1)
    assert axial[-1] >= .05 and axial.min() >= -1e-6 and np.diff(axial).min() >= -1e-5
    assert lateral.max() <= .002
    angles = Rotation.from_matrix(tools['left'][0, :3, :3].T @ tools['left'][:, :3, :3]).magnitude()
    assert angles.max() <= np.deg2rad(1.)
    release_C = tools['right'] @ arrays['right_measured_attachment']
    box_drift = float(np.max(np.abs(release_C - actual_C)))
    assert box_drift <= 2e-5
    caps = contact_cap_spheres('left')
    centers = caps[:, :3] @ tools['left'][-1, :3, :3].T + tools['left'][-1, :3, 3]
    depths = box_sphere_depth(centers[None], caps[None, :, 3], release_C[-1:, :3, 3], release_C[-1:, :3, :3], np.array(owner.dimensions_m)/2)
    assert depths.max() < 0., depths
    tau = arrays['support_extreme_torques_nm']
    limits = arrays['scaled_torque_limits_nm']
    assert tau.shape == (2, 14) and np.all(np.abs(tau) <= limits)
    alpha = arrays['support_alpha']
    assert alpha[0] == 0. and alpha[-1] == 1. and np.all(np.diff(alpha) >= 0.)
    assert np.allclose(arrays['support_torques_nm'], (1-alpha[:, None])*tau[0] + alpha[:, None]*tau[1])
    assert np.all(np.abs(arrays['support_torques_nm']) <= limits)
    robot.update_cfg(dict(zip(NAMES, arrays['support_transfer'][-1])))
    expected_delta = np.zeros(14)
    for s, sign in [('left', -1.), ('right', 1.)]:
        for j in range(1, 8):
            joint = robot.joint_map[f'{s}_joint{j}']
            T = robot.get_transform(joint.child, robot.base_link)
            gravity = -(T[:3, :3] @ joint.axis) @ np.cross(actual_C[:3, 3] - T[:3, 3], [0., 0., -98.1])
            expected_delta[(0 if s == 'left' else 7) + j - 1] = sign*gravity
    torque_error = float(np.max(np.abs(tau[1] - tau[0] - expected_delta)))
    assert torque_error <= .01
    process = source_slot_process_checks({'left': source_slot_observation(owner, np.concatenate(all_boxes))})
    assert process['valid'], process
    pending = next(b for b in prior['request']['boxes'] if b['side'] == 'left')
    assert report['pending_original_box']['center_m'] == pending['center_m']
    assert report['pending_original_box']['size'] == pending['size']
    assert np.array_equal(arrays['pending_box_matrix'][:3, 3], pending['center_m'])
    continued = fk(robot, arrays['donor_continue'])
    continuation = float(np.linalg.norm(continued['left'][-1, :3, 3]-continued['left'][0, :3, 3]))
    assert continuation >= .0099
    result = dict(runtime=dict(sys_executable=sys.executable, sys_version=sys.version,
        curobo_module_path=importlib.util.find_spec('curobo').origin,
        curobo_distribution_version=importlib.metadata.version('nvidia-curobo'),
        numpy_version=np.__version__, scipy_version=importlib.metadata.version('scipy')),
        valid=True, donor_continuation_m=continuation, hashes=hashes, physical_box_id='right', q_seams=seams,
                  box_pose_seams_max_element=pose_seams, fk_max_element_error=fk_error,
                  actual_world_extraction_m=pull.tolist(), support_switch_world_pose_jump_max_element=switch_jump,
                  donor_retreat_axial_m=float(axial[-1]), donor_retreat_lateral_max_m=float(lateral.max()),
                  donor_final_all_cap_clearance_min_m=float(-depths.max()), held_box_drift_max_element=box_drift,
                  independent_support_swap_error_nm=torque_error, source_process=process,
                  scope='CPU FK, one-box identity, seams, world-pose continuity, extraction, support convexity and donor-cap separation. GPU report supplies robot/world collision audit. Sampled geometry only; no native dynamics or full two-box execution.')
    (OUT / 'transfer_cpu_audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
