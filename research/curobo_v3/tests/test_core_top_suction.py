"""Suction geometry checks; run in the pinned SDK environment, no GPU planning."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest
from importlib.util import find_spec

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from curobo_core.adapter import (carry_targets, pose_matrix, suction_quaternion,
                                 task_attachments, task_contact_positions, top_lift_blockers,
                                 contact_offset, side_face_variants, tool_to_box)
from curobo_core.cache import resource_key
from curobo_core.contracts import ACTIVE_JOINTS, PlanRequest
from curobo_core.scene import Pose, RobotState, SceneObject, SceneSnapshot, MeshObject


class TopSuctionTest(unittest.TestCase):
    def setUp(self):
        self.box = SceneObject('wall_box_02', (.3, .4, .4), Pose((1.408052051, 0., .2)))
        self.snapshot = SceneSnapshot('model', RobotState.from_mapping(dict.fromkeys(ACTIVE_JOINTS, 0.), 1), (self.box,))
        self.tasks = {'left': 2}
        self.targets = {s + '_tool0': {'position': (.7, .4 if s == 'left' else -.4, 1.1),
                                      'quaternion': tuple(suction_quaternion(s, 'side'))}
                        for s in ('left', 'right')}

    def test_top_contact_and_attached_box_are_consistent(self):
        for mode in ('side', 'top'):
            pos = task_contact_positions(self.snapshot, self.tasks, suction_mode=mode)['left']
            contact = Pose(pos, tuple(suction_quaternion('left', mode)))
            attachment, = task_attachments(self.snapshot, self.tasks, mode)
            actual = pose_matrix(contact) @ pose_matrix(attachment.tool_to_object)
            np.testing.assert_allclose(actual, pose_matrix(self.box.pose), atol=1e-12)
            if mode == 'top':
                self.assertAlmostEqual(pos[2], .4)
                self.assertAlmostEqual(pos[0], self.box.pose.position[0] - .0575)
                self.assertGreaterEqual(pos[0] - .0875, self.box.pose.position[0] - .15 + .005 - 1e-12)
                np.testing.assert_allclose(pose_matrix(contact)[:3, 2], (0., 0., -1.), atol=1e-12)
                # The box remains upright even though the tool now points down.
                self.assertAlmostEqual(actual[2, 2], 1.)

    def test_switching_suction_preserves_box_destination_and_empty_arm_target(self):
        converted = carry_targets(self.snapshot, self.tasks, self.targets, 'top')
        def destination(targets, mode):
            p = targets['left_tool0']
            return pose_matrix(Pose(p['position'], p['quaternion'])) @ pose_matrix(
                task_attachments(self.snapshot, self.tasks, mode)[0].tool_to_object)
        np.testing.assert_allclose(destination(converted, 'top'), destination(self.targets, 'side'), atol=1e-12)
        self.assertEqual(converted['right_tool0'], self.targets['right_tool0'])
        self.assertNotEqual(resource_key(self.snapshot, self.tasks, {}, 'top'),
                            resource_key(self.snapshot, self.tasks, {}, 'side'))

    def test_top_lift_rejects_cover_box_and_overhead_mesh(self):
        self.assertEqual(top_lift_blockers(self.snapshot, self.tasks), [])
        cover = replace(self.box, object_id='cover', pose=Pose((1.408052051, 0., .61)))
        self.assertEqual(top_lift_blockers(replace(self.snapshot, objects=(self.box, cover)), self.tasks), ['cover'])
        roof = MeshObject('roof', ((1.2, -.2, .5), (1.6, -.2, .5), (1.4, .2, .6), (1.2, .2, .5)), ((0, 1, 2), (0, 2, 3)))
        self.assertEqual(top_lift_blockers(replace(self.snapshot, meshes=(roof,)), self.tasks), ['roof'])
        with self.assertRaises(ValueError):
            top_lift_blockers(self.snapshot, self.tasks, -1)

    def test_side_face_search_preserves_box_pose_and_stays_inside_contact_face(self):
        for offsets in side_face_variants(self.snapshot, self.tasks):
            self.assertAlmostEqual(abs(offsets['left'][2]), .05375)
            position = task_contact_positions(self.snapshot, self.tasks, offsets=offsets)['left']
            contact = pose_matrix(Pose(position, tuple(suction_quaternion('left', 'side'))))
            attachment, = task_attachments(self.snapshot, self.tasks, offsets=offsets)
            np.testing.assert_allclose(contact @ pose_matrix(attachment.tool_to_object), pose_matrix(self.box.pose), atol=1e-12)
            corners = np.array([[x,y,0.] for x in (-.0875,.0875) for y in (-.1775,.1775)])
            corners = corners @ contact[:3,:3].T + contact[:3,3]
            self.assertTrue(np.all(np.abs(corners[:,1:] - self.box.pose.position[1:]) <= np.array(self.box.dimensions_m[1:])/2-.005+1e-12))
            converted = carry_targets(self.snapshot, self.tasks, self.targets, 'side', offsets)
            new = converted['left_tool0']; old = self.targets['left_tool0']
            np.testing.assert_allclose(pose_matrix(Pose(new['position'],new['quaternion'])) @ pose_matrix(attachment.tool_to_object),
                pose_matrix(Pose(old['position'],old['quaternion'])) @ tool_to_box('left',self.box.dimensions_m,'side'), atol=1e-12)
        for invalid in ((.01,0,0),(0,.1,0),(0,0,.15),(0,0,float('nan'))):
            with self.assertRaises(ValueError):
                contact_offset(self.box.dimensions_m,'side',invalid)

    @unittest.skipUnless(find_spec("curobo") is not None, "requires pinned cuRobo SDK; checked in SDK environment")
    def test_grasp_offset_invalidates_cached_robot_payload(self):
        from curobo_core.backend import CuroboBackend
        backend = object.__new__(CuroboBackend)
        backend.snapshot, backend.box_fit = self.snapshot, {}
        backend.suction_mode, backend.contact_offsets = 'side', {}
        backend._cached_task, backend._cached_solver, backend._cached_checkers = None, None, {}
        backend.prepare_task(self.tasks)
        before = backend._cached_task
        backend._cached_solver = object()
        backend._cached_checkers['payload'] = object()
        backend.contact_offsets = next(side_face_variants(self.snapshot, self.tasks))
        backend.prepare_task(self.tasks)
        self.assertNotEqual(before, backend._cached_task)
        self.assertIsNone(backend._cached_solver)
        self.assertEqual(backend._cached_checkers, {})

    @unittest.skipUnless(find_spec("curobo") is not None, "requires pinned cuRobo SDK; checked in SDK environment")
    def test_stability_uses_the_selected_attachment_not_side_grasp_axes(self):
        import torch
        from types import SimpleNamespace
        from v3_batched_loaded_search import GpuValidity
        validity = object.__new__(GpuValidity)
        validity.active_sides = ('left',)
        validity.payload_up = {'left': pose_matrix(task_attachments(self.snapshot, self.tasks, 'top')[0].tool_to_object)[:3, 2]}
        q = torch.tensor(suction_quaternion('left', 'top')).reshape(1, 4)
        state = SimpleNamespace(robot_spheres=torch.zeros(1, 4),
                                tool_poses=SimpleNamespace(to_dict=lambda: {'left_tool0': SimpleNamespace(quaternion=q)}))
        self.assertAlmostEqual(float(validity._stability_cost(state, 1)[0]), 0.)
        validity.payload_up = {'left': pose_matrix(task_attachments(self.snapshot, self.tasks, 'side')[0].tool_to_object)[:3, 2]}
        self.assertAlmostEqual(float(validity._stability_cost(state, 1)[0]), 1.)

    def test_request_modes_are_serialized_and_legacy_side_identity_is_preserved(self):
        targets = tuple((n, Pose(p['position'], p['quaternion'])) for n, p in self.targets.items())
        request = PlanRequest(self.snapshot, tuple(self.tasks.items()), targets)
        self.assertEqual(PlanRequest.from_dict(request.to_dict()), request)
        side = replace(request, suction_mode='side')
        self.assertNotIn('suction_mode', side.to_dict())
        self.assertEqual(PlanRequest.from_dict(side.to_dict()), side)
        self.assertNotEqual(side.identity, request.identity)
        with self.assertRaises(ValueError):
            replace(request, suction_mode='bottom')


if __name__ == '__main__':
    unittest.main()
