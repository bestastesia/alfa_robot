import numpy as np
from scipy.spatial.transform import Rotation
import torch
from curobo.types import JointState

from curobo_core.contracts import ACTIVE_JOINTS
from curobo_core.lm_refinement import CartesianReference, refine_lm_path
from v3_fast_path import fit_path
from curobo_core.scene import digest


def world_pose_in_robot_frame(position, quaternion_wxyz, robot_pose):
    matrix = np.eye(4)
    matrix[:3, 3] = position
    matrix[:3, :3] = Rotation.from_quat(np.roll(quaternion_wxyz, -1)).as_matrix()
    return np.linalg.inv(robot_pose) @ matrix


def phase_references(planner, source, phase, rows, checker):
    if phase not in ('extract', 'conveyor_place'):
        raise ValueError('LM refinement is restricted to extraction and placement')
    if set(source['task']) != {'left', 'right'}:
        raise ValueError('LM refinement not yet validated for single-arm tasks')
    if not checker.guidance_payload:
        raise ValueError('LM refinement requires a loaded phase checker')
    for side in source['task']:
        if not np.allclose(source['visual_tool_to_box'][side], checker.task_tool_to_box[side], atol=1e-7):
            raise ValueError('Phase checker attachment differs from current task')
    with torch.no_grad():
        state = checker.kinematics.compute_kinematics(JointState.from_position(
            torch.tensor(np.asarray(rows[:1]), device='cuda', dtype=torch.float32), joint_names=list(ACTIVE_JOINTS)))
        poses = state.tool_poses.to_dict()
        starts = {}
        for side in source['task']:
            pose = poses[side + '_tool0']
            matrix = np.eye(4)
            matrix[:3, 3] = pose.position.reshape(3).cpu().numpy()
            matrix[:3, :3] = Rotation.from_quat(np.roll(pose.quaternion.reshape(4).cpu().numpy(), -1)).as_matrix()
            starts[side] = matrix
    references = {}
    if phase == 'extract':
        detail = (source.get('selected_lm_review') or {}).get('lm_detail') or {}
        translations = detail.get('translation_local')
        if not translations:
            raise ValueError('Missing task extraction direction/distance; do not assume 35cm -X')
        for side, start in starts.items():
            displacement = np.asarray(translations[side], dtype=float)
            if displacement.shape != (3,) or not np.isfinite(displacement).all():
                raise ValueError('Invalid phase-local extraction displacement')
            end = start.copy()
            end[:3, 3] += displacement
            references[side] = CartesianReference(start, end, np.eye(4))
    else:
        names = source['joint_names']
        indices = np.flatnonzero(np.asarray(source['phases']) == phase)
        row = source['frames'][max(0, int(indices[0]) - 1)]
        base = np.eye(4)
        base[:3, :3] = Rotation.from_euler('z', row[names.index('base_yaw')]).as_matrix()
        base[:2, 3] = [row[names.index('base_x')], row[names.index('base_y')]]
        for side, tool in starts.items():
            attachment = np.asarray(source['visual_tool_to_box'][side], dtype=float)
            if not np.allclose(attachment, checker.task_tool_to_box[side], atol=1e-7):
                raise ValueError('Phase checker attachment differs from current task')
            target = source['conveyor_box_targets'][side]
            end = world_pose_in_robot_frame(target['position'], target['quaternion_wxyz'], base)
            references[side] = CartesianReference(tool @ attachment, end, attachment)
    return references


def optimize_lm_stage(planner, source, phase, rows, validity, config):
    checker = validity.checker
    references = phase_references(planner, source, phase, rows, checker)
    controls, start, goal, diagnostics = refine_lm_path(checker, rows, references, validity, fit_path,
        count=config['controls'], iterations=config['iterations'])
    diagnostics['reference_kind'] = 'task TCP translation and fixed orientation' if phase == 'extract' else 'box-center line and same-progress SLERP'
    diagnostics['references'] = {side:dict(start=item.start.tolist(), end=item.end.tolist(), attachment=item.attachment.tolist()) for side,item in references.items()}
    diagnostics['robot_config_digest'] = digest(planner.robot)
    diagnostics['scene_digest'] = digest([(item.name, item.pose, item.dims) for item in checker.guidance_scene.cuboid or []])
    diagnostics['attachment_digest'] = digest(source['visual_tool_to_box'])
    return controls, start, goal, None, diagnostics
