#!/usr/bin/env python3
"""Build the door asset root of the re-zeroed alfa_v3_curobo model (PR #80/#82, MOTION-443).

One self-contained root from this checkout's frozen model: fixed/mobile robot configs and URDFs
(mesh paths relocated into the root), named poses, the 40-ball box fit, the fixed-body cuboids,
the PR #71 dynamics overlay with the eRob torque profiles, and the placement policy's recenter
atlas re-solved for the 135 mm tool point on the same updown/swivel/branch grid. DOOR_MODEL.json
names every file (curobo_core.door_model.load_assets) and records the sources' sha256.

Usage: v3_door_rezero_assets.py --output <asset root>. CPU only.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import yourdfpy

from curobo_core.adapter import pose_matrix
from curobo_core.contracts import ACTIVE_JOINTS
from curobo_core.door_model import MANIFEST, REZERO_REVISION
from curobo_core.scene import Pose
from v3_cartesian_math import JointFrame, branch
from v3_fast_assets import local_generated_path
from v3_rezero_analytic import RezeroAnalytic
from v3_zero_model_candidate import COORDINATE_REVISION, JOINT_SIGNS, OFFSETS_DEG, TOOL_OFFSET_M

ROOT = Path(__file__).resolve().parents[1]
SOURCES = dict(
    fixed='generated/zero_model_candidate_20261018/fixed.yaml',
    mobile='generated/zero_model_candidate_20261018/mobile.yaml',
    named_poses='generated/zero_model_candidate_20261018/named_poses.yaml',
    model_manifest='generated/zero_model_candidate_20261018/manifest.json',
    dynamics='generated/pr71_current_dynamics/robot.yaml',
    torque_profiles='generated/v3_dynamics_20261019/erob_joint_profiles.json',
    fixed_body_boxes='generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json',
    box_fit='generated/v3_suction_v322_6bb184b/frozen_collision_model/box_voxel_edge_corner_final.json',
    policy='config/placement_task_policy.json',
    analytic_library='generated/v3_analytic_071cb95/libv3_analytic_bridge.so')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def relocate_urdf(source, destination, meshes):
    """Copy a URDF with every mesh copied into `meshes` (unique basenames) and referenced absolutely."""
    tree = ET.parse(source)
    for mesh in tree.findall('.//mesh'):
        path = local_generated_path(mesh.get('filename'))
        if not path.is_absolute():
            path = Path(source).parent/path
        target = meshes/path.name
        if target.exists():
            if sha256(target) != sha256(path):
                raise ValueError(f'two different meshes named {path.name}')
        else:
            shutil.copyfile(path, target)
        mesh.set('filename', str(target.resolve()))
    tree.write(destination, encoding='utf-8', xml_declaration=True)
    return tree.getroot()


def relocate_config(source, name, model, meshes):
    """Robot config `name`.yaml with its URDF relocated next to it; returns (config, URDF root element)."""
    document = yaml.safe_load(Path(source).read_text())
    kin = document.get('robot_cfg', document)['kinematics']
    urdf = model/f'{name}.urdf'
    element = relocate_urdf(local_generated_path(kin['urdf_path']), urdf, meshes)
    kin['urdf_path'], kin['asset_root_path'] = str(urdf.resolve()), str(model.resolve())
    config = model/f'{name}.yaml'
    config.write_text(yaml.safe_dump(document, sort_keys=False))
    return config, element


def resolve_atlas(urdf, policy, legacy_path):
    """The policy atlas in re-zeroed joints at the policy tool targets: same updown, swivels, branches."""
    legacy = np.load(legacy_path, allow_pickle=False)
    joints, metadata = legacy['joints'].astype(np.float64), legacy['metadata']
    if not np.allclose(metadata[:, 0], joints[:, 0], atol=1e-6):
        raise ValueError('atlas metadata updown differs from the joint rows')
    side_policy = policy['side']
    targets = [pose_matrix(Pose((side_policy['position']['x'], side_policy['position']['y_'+side],
                                 side_policy['position']['z']), tuple(side_policy['quaternion_wxyz'])))
               for side in ('left', 'right')]
    frame = JointFrame(np.deg2rad([OFFSETS_DEG['left'], OFFSETS_DEG['right']]), JOINT_SIGNS)
    solver = RezeroAnalytic()
    model = yourdfpy.URDF.load(urdf, load_meshes=False, build_scene_graph=True)
    output = np.empty_like(joints)
    output[:, 0] = joints[:, 0]
    failures, position_error, angle_error, step = [], 0., 0., 0.
    for index, row in enumerate(joints):
        seed = row.copy()
        for side in range(2):
            seed[1+7*side:8+7*side] = frame.from_legacy(side, row[1+7*side:8+7*side])
        model.update_cfg(dict(zip(ACTIVE_JOINTS, seed)))
        carriage = np.linalg.inv(model.get_transform('arm_carriage', model.base_link))
        for side in range(2):
            arm = slice(1+7*side, 8+7*side)
            solutions = solver.solve(side, carriage@targets[side], np.deg2rad(metadata[index, 1+side]), seed[arm])
            same = [solution for solution in solutions if frame.branch(side, solution) == branch(row[arm])]
            if not same:
                failures.append(dict(row=index, side=side, solutions=len(solutions)))
                output[index, arm] = np.nan
                continue
            output[index, arm] = same[0]  # RezeroAnalytic sorts by distance to the seed
            step = max(step, float(np.abs(same[0]-seed[arm]).max()))
        if np.isnan(output[index]).any():
            continue
        model.update_cfg(dict(zip(ACTIVE_JOINTS, output[index])))
        for side, target in zip(('left', 'right'), targets):
            actual = model.get_transform(side+'_tool0', model.base_link)
            position_error = max(position_error, float(np.linalg.norm(actual[:3, 3]-target[:3, 3])))
            angle_error = max(angle_error, float(Rotation.from_matrix(actual[:3, :3].T@target[:3, :3]).magnitude()))
    return output, metadata, dict(rows=len(joints), failures=failures, maximum_position_error_m=position_error,
                                  maximum_angle_error_rad=angle_error, maximum_joint_step_from_converted_rad=step)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if COORDINATE_REVISION != REZERO_REVISION:
        raise SystemExit('door model and analytic adapter disagree on the joint coordinates')
    sources = {name: ROOT/relative for name, relative in SOURCES.items()}
    manifest = json.loads(sources['model_manifest'].read_text())
    if manifest['coordinate_revision'] != REZERO_REVISION or manifest['tool_offset_m'] != TOOL_OFFSET_M:
        raise SystemExit('frozen model manifest is not the re-zeroed 135 mm model')
    if args.output.exists():
        raise SystemExit(f'refusing to overwrite {args.output}')
    model, meshes, dynamics, atlas = (args.output/name for name in ('model', 'model/meshes', 'dynamics', 'atlas'))
    for directory in (meshes, dynamics, atlas):
        directory.mkdir(parents=True)
    fixed, element = relocate_config(sources['fixed'], 'fixed', model, meshes)
    mobile, _ = relocate_config(sources['mobile'], 'mobile', model, meshes)
    if element.get('alfa_coordinate_revision') != REZERO_REVISION:
        raise SystemExit('fixed URDF is not in the re-zeroed coordinates')
    dynamics_kin = yaml.safe_load(sources['dynamics'].read_text())
    dynamics_kin = dynamics_kin.get('robot_cfg', dynamics_kin)['kinematics']
    overlay = relocate_urdf(local_generated_path(dynamics_kin['urdf_path']), dynamics/'robot_dynamics_overlay.urdf', meshes)
    if overlay.get('alfa_coordinate_revision') != REZERO_REVISION:
        raise SystemExit('dynamics overlay is not in the re-zeroed coordinates')
    shutil.copyfile(sources['torque_profiles'], dynamics/'erob_joint_profiles.json')
    for name, target in (('named_poses', 'named_poses.yaml'), ('box_fit', 'box_fit.json'),
                         ('fixed_body_boxes', 'frozen_cuboids.json')):
        shutil.copyfile(sources[name], model/target)
    policy = json.loads(sources['policy'].read_text())
    legacy_atlas = ROOT/policy['side']['atlas_path']
    joints, metadata, report = resolve_atlas(model/'fixed.urdf', policy, legacy_atlas)
    if report['failures']:
        raise SystemExit(f"atlas rows without a same-branch solution: {report['failures'][:5]}")
    np.savez_compressed(atlas/'all_solutions.npz', joints=joints.astype(np.float32), metadata=metadata,
                        coordinate_revision=REZERO_REVISION, tool_offset_m=TOOL_OFFSET_M,
                        source_sha256=sha256(legacy_atlas), policy_sha256=sha256(sources['policy']))
    document = dict(
        revision=REZERO_REVISION, tool_offset_m=TOOL_OFFSET_M,
        files=dict(robot_config='model/fixed.yaml', mobile_robot_config='model/mobile.yaml', urdf='model/fixed.urdf',
                   named_poses='model/named_poses.yaml', box_fit='model/box_fit.json',
                   fixed_body_boxes='model/frozen_cuboids.json', dynamics_root='dynamics',
                   recenter_atlas='atlas/all_solutions.npz'),
        sources={name: dict(path=str(path), sha256=sha256(path)) for name, path in
                 {**sources, 'legacy_atlas': legacy_atlas}.items()},
        recenter_atlas=dict(report, method='RezeroAnalytic at the policy tool targets, legacy row updown and swivel, '
                                           'same library branch, nearest to the converted legacy row'))
    (args.output/MANIFEST).write_text(json.dumps(document, ensure_ascii=False, indent=1)+'\n')
    print(json.dumps(dict(output=str(args.output), fixed=str(fixed), mobile=str(mobile),
                          meshes=len(list(meshes.iterdir())), atlas=report), ensure_ascii=False))


if __name__ == '__main__':
    main()
