"""CPU integration check of the three saved Y85 geometry components, without retiming."""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy

ROOT = Path(__file__).resolve().parents[3]
DATA = ROOT / 'generated/two_box_layout'
NAMES = ['updown'] + [f'{side}_joint{i}' for side in ('left', 'right') for i in range(1, 8)]


def audit():
    paths = [DATA / name for name in (
        'joint_contract_20261123/transfer_owner_transport_geometry.npz',
        'joint_contract_20261124_transfer/transfer_geometry.npz',
        'joint_contract_20261124_suffix/suffix_geometry.npz')]
    components = []
    for path in paths:
        with np.load(path, allow_pickle=False) as data:
            components.append({key: data[key].copy() for key in data.files})
    for phase in ('approach', 'contact', 'extract', 'owner_transport'):
        assert np.array_equal(components[0][phase], components[1][phase]), 'prefix was changed during transfer'
    interface = json.loads((paths[1].parent / 'takeover_interface.json').read_text())
    suffix = json.loads((paths[2].parent / 'suffix_probe.json').read_text())
    assert interface['physical_box_id'] == 'right' and interface['physical_payload_kg'] == 10.
    assert suffix['success'] and not suffix['native_timed']
    request = json.loads((paths[0].parent / 'broader_same_side_y85_request.json').read_text())
    originals = {box['side']: box for box in request['boxes']}
    original_poses = {key: np.eye(4) for key in originals}
    for key, box in originals.items(): original_poses[key][:3, 3] = box['center_m']
    attachment = {
        ('right', 'left'): np.asarray(interface['donor_original_tool_to_box']),
        ('right', 'right'): np.asarray(interface['receiver_measured_tool_to_box']),
        ('left', 'left'): np.asarray(suffix['attachment_left_actual'])}
    # Keys are immutable physical box IDs; values are the actual arm supporting each box.
    stages = [(0, 'approach', {}), (0, 'contact', {}),
              (0, 'extract', {'right': 'left'}), (0, 'owner_transport', {'right': 'left'}),
              (1, 'receiver_approach', {'right': 'left'}), (1, 'support_transfer', {'right': 'left'}),
              (1, 'donor_release', {'right': 'right'}), (1, 'donor_continue', {'right': 'right'}),
              (2, 'upper_park', {'right': 'right'}), (2, 'lower_approach', {'right': 'right'}),
              (2, 'lower_contact', {'right': 'right'}),
              (2, 'lower_extract', {'right': 'right', 'left': 'left'}),
              (2, 'final_transport', {'right': 'right', 'left': 'left'})]
    robot = yourdfpy.URDF.load(ROOT / 'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
    previous_q = None
    previous_boxes = original_poses
    records, rows, phase_ids = [], [], []
    for index, (component, phase, owners) in enumerate(stages):
        q = components[component][phase]
        assert q.ndim == 2 and q.shape[1] == 15 and np.isfinite(q).all()
        assert len(set(owners.values())) == len(owners), 'two physical boxes assigned to one arm'
        pending = set(originals) - set(owners)
        assert pending.isdisjoint(owners) and pending | set(owners) == set(originals)
        seam = 0. if previous_q is None else float(np.max(np.abs(q[0] - previous_q)))
        assert seam <= 1e-5, (phase, seam)
        endpoint_boxes = []
        for value in q[[0, -1]]:
            robot.update_cfg(dict(zip(NAMES, value)))
            endpoint_boxes.append({box_id: (robot.get_transform(owners[box_id] + '_tool0', robot.base_link)
                @ attachment[(box_id, owners[box_id])] if box_id in owners else original_poses[box_id].copy())
                for box_id in originals})
        position_seam = max(float(np.linalg.norm(endpoint_boxes[0][key][:3, 3] - previous_boxes[key][:3, 3]))
                            for key in originals)
        angle_seam = max(float(Rotation.from_matrix(previous_boxes[key][:3, :3].T
                            @ endpoint_boxes[0][key][:3, :3]).magnitude()) for key in originals)
        assert position_seam <= .002 and angle_seam <= np.deg2rad(1.), (phase, position_seam, angle_seam)
        pull = None
        if phase in ('extract', 'lower_extract'):
            key = 'right' if phase == 'extract' else 'left'
            delta = endpoint_boxes[-1][key][:3, 3] - endpoint_boxes[0][key][:3, 3]
            expected = .255 if key == 'right' else .300
            assert np.linalg.norm(delta + [expected, 0., 0.]) <= .002
            assert np.linalg.norm(endpoint_boxes[0][key][:3, 3] - originals[key]['center_m']) <= .002
            pull = delta.tolist()
        if phase == 'support_transfer':
            assert np.array_equal(q, np.repeat(q[:1], len(q), axis=0)), 'support changed during motion'
            robot.update_cfg(dict(zip(NAMES, q[-1])))
            switched = robot.get_transform('right_tool0', robot.base_link) @ attachment[('right', 'right')]
            assert np.max(np.abs(switched - endpoint_boxes[-1]['right'])) < 1e-12
        records.append(dict(phase=phase, physical_box_to_owner=owners, pending_box_ids=sorted(pending),
            total_supported_mass_kg=10 * len(owners), frames=len(q), q_seam_max_error_rad=seam,
            box_position_seam_max_m=position_seam, box_angle_seam_max_rad=angle_seam,
            extraction_world_delta_m=pull, component_file=str(paths[component])))
        previous_q, previous_boxes = q[-1], endpoint_boxes[-1]
        rows.append(q[1:] if index else q)
        phase_ids.extend([index] * (len(q) - bool(index)))
    final = np.concatenate(rows)
    expected = np.deg2rad([40., -46., 0., -100., -30., -75., 90.])
    angle_error = float(np.max(np.abs(final[-1, 1:] - np.r_[expected, -expected])))
    assert angle_error < 1e-5 and -1. <= final[-1, 0] <= 0.
    output = DATA / 'takeover_20261124'
    output.mkdir(exist_ok=True)
    np.savez_compressed(output / 'y85_geometry_chain.npz', position=final, phase_index=np.array(phase_ids))
    report = dict(valid=True, geometry_only=True, native_success=False, production_dispatch=False,
        whole_goal_complete=False, phases=records, frames=len(final), final_angle_error_rad=angle_error,
        final_updown_m=float(final[-1, 0]), original_request=request,
        component_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths},
        scope='CPU cross-component identity, actual endpoint FK, 13 geometry stages and seams; '
              'per-component sampled collision/static checks remain their GPU/CPU evidence. '
              'No native q/v/a chain, dynamic handover or universal workspace proof.')
    (output / 'y85_geometry_chain_audit.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: report[key] for key in ('valid', 'frames', 'final_angle_error_rad', 'final_updown_m', 'native_success')}))


if __name__ == '__main__':
    audit()
