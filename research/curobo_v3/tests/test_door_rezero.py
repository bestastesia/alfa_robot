"""CPU contracts of the re-zeroed door model (MOTION-443): joint frame, batch IK, tool plane, fixed body."""
from pathlib import Path
import sys
import unittest

import fcl
import numpy as np
import torch
import trimesh
import yourdfpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from curobo_core.door_model import REZERO_REVISION, FixedBodyBoxes, clip_at_tool_plane, model_revision
from curobo_core.door_sequential_validity import SearchInflation, SequentialValidity
from curobo_core.mesh_validation import _mesh_geometry
from v3_cartesian_math import LEGACY_FRAME, JointFrame, branch
from v3_cartesian_transport import RezeroBatchAnalyticIk
from v3_fast_assets import local_generated_path
from v3_verify_door_mesh_audit import FrameAudit, cap_targets, split_at_tool_plane
from v3_rezero_analytic import RezeroAnalytic
from v3_zero_model_candidate import COORDINATE_REVISION, JOINT_SIGNS, OFFSETS_DEG, TOOL_OFFSET_M

MODEL = ROOT/'generated/zero_model_candidate_20261018'
LIBRARY = ROOT/'generated/v3_analytic_071cb95/libv3_analytic_bridge.so'
FRAME = JointFrame(np.deg2rad([OFFSETS_DEG['left'], OFFSETS_DEG['right']]), JOINT_SIGNS)


class JointFrameTests(unittest.TestCase):
    def test_revision_matches_the_analytic_adapter(self):
        self.assertEqual(REZERO_REVISION, COORDINATE_REVISION)

    def test_round_trip_and_library_branch(self):
        rng = np.random.default_rng(1)
        for side in (0, 1):
            arms = rng.uniform(-3., 3., (500, 7))
            np.testing.assert_allclose(FRAME.from_legacy(side, FRAME.legacy(side, arms)), arms, atol=1e-12)
            for arm in arms[:50]:
                self.assertEqual(FRAME.branch(side, arm), branch(FRAME.legacy(side, arm)))
        # q2 of the library is offset-15deg minus q2 (left): the sign changes at q2 = 15 deg, not at 0.
        arm = np.zeros(7)
        arm[1] = np.deg2rad(10.)
        self.assertEqual(FRAME.branch(0, arm)[0], 1)
        arm[1] = np.deg2rad(20.)
        self.assertEqual(FRAME.branch(0, arm)[0], -1)

    def test_identity_frame_keeps_the_legacy_branch(self):
        arm = np.array([.1, -.2, .3, .4, -.5, .6, -.7])
        self.assertEqual(LEGACY_FRAME.branch(0, arm), branch(arm))
        np.testing.assert_array_equal(LEGACY_FRAME.legacy(1, arm), arm)

    def test_negated_mirror_rows_stay_mirrored(self):
        """Column 0 negates every arm joint and swaps arms: a mirror in library angles too."""
        rng = np.random.default_rng(2)
        for left in rng.uniform(-3., 3., (200, 7)):
            np.testing.assert_allclose(FRAME.legacy(1, -left), -FRAME.legacy(0, left), atol=1e-12)


@unittest.skipUnless(LIBRARY.is_file(), 'analytic library not built (tools/prepare_checkout.py)')
class RezeroBatchIkTests(unittest.TestCase):
    def test_batch_solutions_reach_the_135mm_tool_point_in_the_requested_branch(self):
        scalar = RezeroAnalytic()
        batch = RezeroBatchAnalyticIk(FRAME, TOOL_OFFSET_M, scalar.lower, scalar.upper)
        model = yourdfpy.URDF.load(MODEL/'robot_rezero_candidate.urdf', load_meshes=False, build_scene_graph=True)
        rng = np.random.default_rng(3)
        for side, name in ((0, 'left'), (1, 'right')):
            joints = [f'{name}_joint{index}' for index in range(1, 8)]
            arms, targets, swivels, branches = [], [], [], []
            while len(arms) < 24:
                arm = rng.uniform(scalar.lower[side]*.8, scalar.upper[side]*.8)
                model.update_cfg(dict(zip(joints, arm)))
                carriage = model.get_transform('arm_carriage', model.base_link)
                targets.append(np.linalg.inv(carriage)@model.get_transform(f'{name}_tool0', model.base_link))
                swivels.append(scalar.swivel(side, arm))
                branches.append(FRAME.branch(side, arm))
                arms.append(arm)
            for arm, target, swivel, arm_branch in zip(arms, targets, swivels, branches):
                output, found = batch.solve(side, target[None], np.array([swivel]), arm[None], arm_branch)
                self.assertTrue(found[0])
                self.assertEqual(FRAME.branch(side, output[0]), arm_branch)
                model.update_cfg(dict(zip(joints, output[0])))
                carriage = model.get_transform('arm_carriage', model.base_link)
                actual = np.linalg.inv(carriage)@model.get_transform(f'{name}_tool0', model.base_link)
                np.testing.assert_allclose(actual, target, atol=2e-6)
                scalar_rows = scalar.solve(side, target, swivel, arm)
                self.assertLess(np.abs(scalar_rows-output[0]).max(axis=1).min(), 1e-6)


class ToolPlaneTests(unittest.TestCase):
    def test_clip_keeps_the_hull_behind_the_tool_plane(self):
        corners = np.array([[x, y, z] for x in (-.0875, .0875) for y in (-.1775, .1775) for z in (-.151, .016)])
        clipped = clip_at_tool_plane(corners)
        self.assertAlmostEqual(clipped[:, 2].max(), 0.)
        self.assertAlmostEqual(clipped[:, 2].min(), -.151)
        front = clipped[np.isclose(clipped[:, 2], 0.)]
        np.testing.assert_allclose(np.abs(front[:, :2]).max(0), [.0875, .1775])
        self.assertEqual(len(front), 4)

    def test_a_tool_behind_the_plane_is_unchanged(self):
        corners = np.array([[x, y, z] for x in (-.1, .1) for y in (-.1, .1) for z in (-.15, 0.)])
        np.testing.assert_array_equal(clip_at_tool_plane(corners), corners)

    def test_model_revision_is_read_from_the_urdf(self):
        self.assertEqual(model_revision(MODEL/'robot_rezero_candidate.urdf'), REZERO_REVISION)


class FixedBodyTests(unittest.TestCase):
    def setUp(self):
        model = yourdfpy.URDF.load(MODEL/'robot_rezero_candidate.urdf', load_meshes=False, build_scene_graph=True)
        definition = ROOT/'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json'
        self.boxes = FixedBodyBoxes(definition, model)
        validity = SequentialValidity.__new__(SequentialValidity)
        validity.fixed_body = self.boxes
        validity.fixed_body_boxes = tuple(torch.as_tensor(value, dtype=torch.float32)
                                          for value in (self.boxes.centers, self.boxes.rotations, self.boxes.halves))
        validity.fixed_body_query = [0]
        validity.fixed_body_world_clear = {1.: True, 1.1: False}
        validity.inflation = SearchInflation.__new__(SearchInflation)
        validity.inflation.scale = 1.1
        self.validity = validity

    def test_spheres_collide_within_the_activation_distance(self):
        center, rotation, half = self.boxes.centers[0], self.boxes.rotations[0], self.boxes.halves[0]
        rows = []
        for gap in (.0049, .0051, -.01):  # sphere surface this far below the bottom face (no other cuboid there)
            rows.append([[*(center-rotation@[0., 0., half[2]+.02+gap]), .02]])
        collision = self.validity.fixed_body_collision(torch.tensor(rows, dtype=torch.float32))
        self.assertEqual(collision.tolist(), [True, False, True])

    def test_world_overlap_follows_each_rows_scale(self):
        far = torch.tensor([[[5., 5., 5., .02]]]*2, dtype=torch.float32)
        scale = torch.tensor([1., 1.1])
        self.assertEqual(self.validity.fixed_body_collision(far, scale).tolist(), [False, True])
        self.assertEqual(self.validity.fixed_body_collision(far).tolist(), [False, False])


class InflationTests(unittest.TestCase):
    def test_radii_follow_the_world_margin_formula_and_keep_disabled_spheres(self):
        inflation = SearchInflation.__new__(SearchInflation)
        inflation.scale, inflation.tolerance = 1.1, .001
        radius = torch.tensor([.05, -1., 0.])
        np.testing.assert_allclose(inflation.radii(radius).numpy(), [(.051*1.1)-.001, -1., 0.], rtol=1e-6)


class _StaticRobot:
    """Every link frame at the origin: the audit's pieces are placed by their meshes alone."""
    actuated_joint_names = []
    base_link = 'base_link'

    def update_cfg(self, values):
        pass

    def get_transform(self, frame, base):
        return np.eye(4)


class MeshAuditCapTests(unittest.TestCase):
    def test_cap_targets_are_the_last_held_and_the_next_box_of_each_side(self):
        rows = [[], [dict(parent_link='right_tool0', object_id='upper')], [],
                [dict(parent_link='left_tool0', object_id='lower'), dict(parent_link='right_tool0', object_id='upper')], []]
        for frame in cap_targets(rows):
            self.assertEqual(frame, dict(left={'lower'}, right={'upper'}))
        self.assertEqual(cap_targets([[dict(parent_link='left_tool0', object_id='a')], [], [dict(parent_link='left_tool0', object_id='b')]]),
                         [dict(left={'a'}, right=set()), dict(left={'a', 'b'}, right=set()), dict(left={'b'}, right=set())])

    def test_link7_mesh_splits_at_the_135mm_tcp_plane(self):
        model = yourdfpy.URDF.load(MODEL/'robot_rezero_candidate.urdf', load_meshes=False, build_scene_graph=True)
        joint = model.joint_map['left_tool0_fixed']
        collision = model.link_map[joint.parent].collisions[0]
        mesh = trimesh.load(local_generated_path(collision.geometry.mesh.filename), force='mesh', process=False)
        tool = np.linalg.inv(collision.origin)@joint.origin
        body, cap = split_at_tool_plane(mesh, tool)

        def front(piece):
            return (np.asarray(piece.vertices)-tool[:3, 3])@tool[:3, 2]
        self.assertLessEqual(front(body).max(), 1e-9)
        self.assertGreaterEqual(front(cap).min(), -1e-9)
        self.assertAlmostEqual(front(cap).max(), .016, places=5)
        self.assertTrue(body.is_watertight and cap.is_watertight)
        self.assertAlmostEqual(body.volume+cap.volume, mesh.volume, places=9)

    def audit(self):
        audit = FrameAudit.__new__(FrameAudit)
        audit.robot, audit.locked, audit.parents, audit.self_pairs = _StaticRobot(), {}, {}, []
        audit.tolerance, audit.world_cache = 1e-4, {}
        audit.request = fcl.CollisionRequest(num_max_contacts=1, enable_contact=True)
        audit.links = []
        for name, low, high, cap_side in (('left_link7:0', -.1, 0., None), ('left_link7:0:cap', 0., .016, 'left')):
            mesh = trimesh.creation.box(bounds=((-.05, -.05, low), (.05, .05, high)))
            audit.links.append(dict(name=name, link='left_link7', cap_side=cap_side, object=fcl.CollisionObject(_mesh_geometry(mesh)),
                                    vertices=np.asarray(mesh.vertices), origin=np.eye(4)))
        audit.caps = ['left_link7:0:cap']
        return audit

    def check(self, audit, bottom, caps):
        box = dict(object_id='target', dimensions_m=[.4, .4, .2-bottom],
                   pose=dict(position=[0., 0., (.2+bottom)/2], quaternion_wxyz=[1., 0., 0., 0.]))
        world = audit.world(bottom, [box], [], np.eye(4))
        return audit.check({}, world, [], np.eye(4), -10., 'base_link', caps)

    def test_the_cap_may_enter_only_its_own_box(self):
        audit = self.audit()
        contacts, failures = self.check(audit, .008, dict(left={'target'}, right=set()))
        self.assertEqual(failures, [])
        self.assertEqual([(item['kind'], item['pair']) for item in contacts], [('cap_contact', ['left_link7:0:cap', 'target'])])
        _, failures = self.check(audit, .008, dict(left=set(), right={'target'}))
        self.assertEqual([(item['kind'], item['pair']) for item in failures], [('robot_world', ['left_link7:0:cap', 'target'])])

    def test_the_body_behind_the_tcp_plane_never_enters_its_own_box(self):
        _, failures = self.check(self.audit(), -.01, dict(left={'target'}, right=set()))
        self.assertEqual([(item['kind'], item['pair'][0]) for item in failures], [('robot_world', 'left_link7:0')])


if __name__ == '__main__':
    unittest.main()
