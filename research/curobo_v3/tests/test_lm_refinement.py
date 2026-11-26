from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import torch

from curobo_core.contracts import ACTIVE_JOINTS
from curobo_core.lm_refinement import CartesianReference, reference_residuals, validate_inputs
from v3_lm_refinement import world_pose_in_robot_frame


def identity_reference(direction=(.1, .2, .3)):
    start = np.eye(4)
    end = start.copy()
    end[:3, 3] = direction
    return CartesianReference(start, end, np.eye(4))


@pytest.mark.parametrize('direction', [(-.35, 0, 0), (0, .21, .17), (.1, -.1, -.2)])
def test_arbitrary_direction_and_quaternion_sign(direction):
    fraction = torch.linspace(0, 1, 9)
    displacement = torch.tensor(direction, dtype=torch.float32)
    point = fraction[:, None] * displacement
    rotation = torch.tensor([1., 0, 0, 0]).expand(9, -1)
    reference = tuple(torch.tensor(item, dtype=torch.float32) for item in (
        [0., 0, 0], direction, [0., 0, 0], [1., 0, 0, 0], [1., 0, 0, 0], [1., 0, 0, 0]))
    result = reference_residuals(point, rotation, reference)
    alternate = reference_residuals(point, -rotation, reference)
    torch.testing.assert_close(result[0], fraction)
    assert float(result[1].abs().max()) < 1e-7
    assert float(result[2].max()) < 1e-10
    torch.testing.assert_close(result[2], alternate[2])


def test_placement_tracks_box_not_tool_line():
    fraction = torch.linspace(0, 1, 9)
    angle = fraction * torch.pi / 4
    quaternion = torch.stack((torch.cos(angle / 2), torch.zeros_like(angle), torch.zeros_like(angle), torch.sin(angle / 2)), -1)
    center = torch.stack((fraction * .5, torch.zeros_like(fraction), torch.ones_like(fraction)), -1)
    offset_world = torch.stack((.2 * torch.cos(angle), .2 * torch.sin(angle), torch.zeros_like(angle)), -1)
    point = center - offset_world
    reference = tuple(torch.tensor(item, dtype=torch.float32) for item in (
        [0., 0, 1.], [.5, 0, 0], [.2, 0, 0], [1., 0, 0, 0], [1., 0, 0, 0], [np.cos(np.pi / 8), 0, 0, np.sin(np.pi / 8)]))
    actual, error, orientation, _, _ = reference_residuals(point, quaternion, reference)
    torch.testing.assert_close(actual, fraction)
    assert float(error.abs().max()) < 1e-7
    assert float(orientation.max()) < 1e-10
    assert abs(float(point[4, 1]) - float((point[0, 1] + point[-1, 1]) / 2)) > .005


def test_world_placement_goal_uses_navigation_pose():
    base = np.eye(4)
    base[:3, :3] = Rotation.from_euler('z', np.pi).as_matrix()
    base[:3, 3] = [2., 3., 0.]
    local = world_pose_in_robot_frame([1., 3., 1.], [1., 0, 0, 0], base)
    np.testing.assert_allclose(local[:3, 3], [1., 0, 1.], atol=1e-8)
    np.testing.assert_allclose((base @ local)[:3, 3], [1., 3., 1.], atol=1e-8)


def test_reference_contract_allows_fixed_pose_but_rejects_unsupported_stationary_rotation():
    held = CartesianReference(np.eye(4), np.eye(4), np.eye(4))
    assert not held.start.flags.writeable
    rotated = np.eye(4);rotated[:3,:3] = Rotation.from_euler('z', 10, degrees=True).as_matrix()
    with pytest.raises(ValueError, match='fixed orientation'):
        CartesianReference(np.eye(4), rotated, np.eye(4))
    for value in (np.full((4, 4), np.nan), np.zeros((3, 3)), np.diag([2., 1., 1., 1.])):
        with pytest.raises(ValueError):CartesianReference(value, identity_reference().end, np.eye(4))
    reference = identity_reference()
    assert not reference.start.flags.writeable


def test_joint_payload_phase_and_arm_contracts():
    checker = SimpleNamespace(kinematics=SimpleNamespace(joint_names=ACTIVE_JOINTS), guidance_payload=True)
    validity = SimpleNamespace(checker=checker)
    references = {side:identity_reference() for side in ('left', 'right')}
    rows = np.zeros((3, 15))
    assert validate_inputs(checker, rows, references, validity, 64, 32).shape == (3, 15)
    for bad in (np.zeros((3, 14)), np.full((3, 15), np.nan)):
        with pytest.raises(ValueError):validate_inputs(checker, bad, references, validity, 64, 32)
    with pytest.raises(ValueError, match='two independent'):
        validate_inputs(checker, rows, {'left':references['left']}, validity, 64, 32)
    checker.guidance_payload = False
    with pytest.raises(ValueError, match='loaded'):
        validate_inputs(checker, rows, references, validity, 64, 32)


@pytest.mark.parametrize('phase,expected_payload', [('extract', True), ('transport', True), ('contact_approach', False)])
def test_stage_setup_uses_correct_payload(monkeypatch, phase, expected_payload):
    import v3_fast_path as module
    calls = []
    checker = SimpleNamespace(guidance_scene=object())
    planner = SimpleNamespace(configure_task=lambda task:None,
        checker=lambda task,payload:calls.append(payload) or checker)
    monkeypatch.setattr(module.SceneSnapshot, 'from_dict', lambda document:document)
    monkeypatch.setattr(module, 'GpuValidity', lambda value,check_ground:SimpleNamespace(checker=value))
    source = dict(task={'left':1,'right':2}, visual_scene_snapshot=object(),
        joint_names=list(ACTIVE_JOINTS), phases=['home',phase,phase], frames=np.zeros((3,15)).tolist())
    _, columns, validity, _ = module.stage_setup(planner, source, phase)
    assert calls == [expected_payload]
    assert len(columns) == 15 and validity.checker is checker


def test_layout_extraction_reference_uses_box_center_but_legacy_tcp_is_unchanged(monkeypatch):
    import v3_lm_refinement as module
    starts, attachments, poses = {}, {}, {}
    for index, side in enumerate(('left', 'right')):
        tool = np.eye(4)
        tool[:3, :3] = Rotation.from_euler('xyz', [.1, .2, .3]).as_matrix()
        tool[:3, 3] = [1., index, .8]
        attachment = np.eye(4)
        attachment[:3, :3] = Rotation.from_euler('y', np.pi/2).as_matrix()
        attachment[:3, 3] = [0., 0., .2+index*.05]
        starts[side], attachments[side] = tool, attachment
        poses[side+'_tool0'] = SimpleNamespace(position=torch.tensor(tool[:3, 3]),
            quaternion=torch.tensor(np.roll(Rotation.from_matrix(tool[:3, :3]).as_quat(), 1)))
    tensor = torch.tensor
    monkeypatch.setattr(module, 'torch', SimpleNamespace(no_grad=torch.no_grad, float32=torch.float32,
        tensor=lambda values, device, dtype: tensor(values, dtype=dtype)))
    checker = SimpleNamespace(guidance_payload=True, task_tool_to_box=attachments,
        kinematics=SimpleNamespace(compute_kinematics=lambda state:SimpleNamespace(
            tool_poses=SimpleNamespace(to_dict=lambda:poses))))
    source = dict(task={'left':0, 'right':1}, visual_tool_to_box=attachments,
        selected_lm_review=dict(lm_detail=dict(translation_local={s:[-.3, 0., 0.] for s in attachments})))
    legacy = module.phase_references(None, source, 'extract', np.zeros((2, 15)), checker)
    source['extraction_reference_frame'] = 'box_center'
    layout = module.phase_references(None, source, 'extract', np.zeros((2, 15)), checker)
    for side in attachments:
        np.testing.assert_allclose(legacy[side].start, starts[side], atol=1e-12)
        np.testing.assert_array_equal(legacy[side].attachment, np.eye(4))
        np.testing.assert_allclose(layout[side].start, starts[side]@attachments[side], atol=1e-12)
        np.testing.assert_array_equal(layout[side].attachment, attachments[side])
        np.testing.assert_allclose(layout[side].end[:3, 3]-layout[side].start[:3, 3], [-.3, 0., 0.], atol=1e-12)


def test_held_reference_has_finite_xyz_and_orientation_residuals_and_gradients():
    point = torch.tensor([[0., 0., 0.], [.001, -.002, .003]], requires_grad=True)
    quaternion = torch.tensor([[1., 0., 0., 0.], [1., 0., 0., 0.]], requires_grad=True)
    reference = tuple(torch.tensor(value, dtype=torch.float32) for value in (
        [0., 0., 0.], [0., 0., 0.], [0., 0., 0.], [1., 0., 0., 0.], [1., 0., 0., 0.], [1., 0., 0., 0.]))
    fraction, error, orientation, actual, desired = reference_residuals(point, quaternion, reference)
    torch.testing.assert_close(fraction, torch.zeros(2))
    torch.testing.assert_close(error, point)
    torch.testing.assert_close(actual, desired)
    (error.square().sum()+orientation.sum()).backward()
    assert torch.isfinite(point.grad).all() and torch.isfinite(quaternion.grad).all()
