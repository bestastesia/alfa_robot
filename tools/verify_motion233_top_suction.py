#!/usr/bin/env python3
"""Validate a completed suction cycle, payload transforms and the frozen pad footprint."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import yaml
import yourdfpy

from verify_motion233_curobo_rebase import ROOT, cycle
sys.path.insert(0, str(ROOT/'research/curobo_v3/tools'))
from curobo_core.adapter import (TOP_PAD_SIZE_M, pose_matrix, task_attachments, top_lift_blockers,
                                 tool_to_box)
from curobo_core.contracts import PlanRequest
from curobo_core.scene import Pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suction-mode', choices=('side', 'top'), default='top')
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--runtime-root', type=Path, required=True)
    args = parser.parse_args()
    document = json.loads(args.result.read_text())
    request, result = PlanRequest.from_dict(document['request']), document['results'][-1]
    cycle(result)
    mode = args.suction_mode
    offsets = result.get('contact_offsets_m', {})
    assert result['request_id'] == request.identity and result['suction_mode'] == mode
    assert result['suction_attempts'][-1]['success']
    if mode == 'top':
        assert not top_lift_blockers(request.snapshot, dict(request.tasks))
    config = yaml.safe_load((args.runtime_root/'generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml').read_text())
    urdf_path = Path(config['kinematics']['urdf_path'])
    robot = yourdfpy.URDF.load(urdf_path, load_meshes=False, build_scene_graph=True)
    root = ET.parse(urdf_path)
    pad_measurements = {}
    for side in ('left', 'right'):
        visual = root.find(f"link[@name='{side}_link7']/visual")
        mesh = trimesh.load(visual.find('geometry/mesh').attrib['filename'])
        origin = visual.find('origin').attrib
        vertices = mesh.vertices @ Rotation.from_euler('xyz', np.fromstring(origin['rpy'], sep=' ')).as_matrix().T + np.fromstring(origin['xyz'], sep=' ')
        face = vertices[vertices[:, 2] > .151-1e-5]
        assert len(face) >= 4
        size = np.ptp(face[:, :2], axis=0)
        np.testing.assert_allclose(size, TOP_PAD_SIZE_M, atol=1e-6)
        pad_measurements[side] = size.tolist()
    attachments = task_attachments(request.snapshot, dict(request.tasks), mode, offsets)
    attach_index = result['phases'].index('attach')
    extract_index = max(i for i, p in enumerate(result['phases']) if p == 'extract')
    errors = {}
    for side, box_id in request.tasks:
        name = side+'_tool0'
        box = request.snapshot.object(f'wall_box_{box_id:02d}')
        attachment = next(a for a in attachments if a.parent_link == name)
        recorded = next(a for a in result['predicted_scene_events'][0]['snapshot']['attachments'] if a['parent_link'] == name)
        np.testing.assert_allclose(pose_matrix(Pose(**recorded['tool_to_object'])), pose_matrix(attachment.tool_to_object), atol=1e-12)
        poses = []
        for index in (attach_index, extract_index):
            robot.update_cfg(dict(zip(result['joint_names'], result['frames'][index])))
            poses.append(robot.get_transform(name, robot.base_link).copy())
        contact, extracted = poses
        actual_box = contact @ pose_matrix(attachment.tool_to_object)
        position_error = np.linalg.norm(actual_box[:3, 3]-box.pose.position)
        assert position_error < .004
        np.testing.assert_allclose(extracted[:3, 3]-contact[:3, 3], (0., 0., .35) if mode == 'top' else (-.35, 0., 0.), atol=1e-6)
        assert Rotation.from_matrix(contact[:3,:3].T @ extracted[:3,:3]).magnitude() < 1e-5
        corners = np.array([[x, y, 0.] for x in (-.0875, .0875) for y in (-.1775, .1775)])
        corners = corners @ contact[:3, :3].T + contact[:3, 3]
        axes = [0, 1] if mode == 'top' else [1, 2]
        center, half = np.array(box.pose.position)[axes], np.array(box.dimensions_m)[axes]/2
        lower, upper = center-half, center+half
        edge_margin = float(min((corners[:, axes]-lower).min(), (upper-corners[:, axes]).min()))
        assert edge_margin >= 0, f'{side} suction plate overhang'
        reference = dict(request.targets)[name]
        effective = result['carry_tool_targets'][name]
        nominal_box = pose_matrix(reference) @ tool_to_box(side, box.dimensions_m, 'side')
        top_box = pose_matrix(Pose(effective['position'], effective['quaternion'])) @ pose_matrix(attachment.tool_to_object)
        np.testing.assert_allclose(top_box, nominal_box, atol=1e-12)
        errors[side] = {'attachment_position_error_m': float(position_error), 'minimum_actual_pad_edge_clearance_m': edge_margin}
    report = {'passed': True, 'suction_mode': mode, 'contact_offsets_m': offsets, 'task': dict(request.tasks),
              'pad_dimensions_measured_m': pad_measurements, 'geometry': errors,
              'linear_extract_m': .35, 'box_destination_preserved': True, 'frames': len(result['frames']),
              'total_ms': result['total_ms'], 'scope': 'f044 research policy; not vacuum-force or hardware certification'}
    args.result.with_suffix('.audit.json').write_text(json.dumps(report, indent=2)+'\n')
    print(f'PASS: {mode} contact, pad footprint, 35cm extraction, attachment geometry, box destination and full cycle.')


if __name__ == '__main__':
    main()
