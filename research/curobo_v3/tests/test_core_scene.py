import ast
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from curobo_core.adapter import relative_pose, world_objects
from curobo_core.cache import resource_key
from curobo_core.contracts import ACTIVE_JOINTS, PlanRequest
from curobo_core.scene import (
    AttachedObject, CollisionPolicy, Pose, RobotState, SceneObject,
    SceneSnapshot, SceneStore,
)


class SceneTest(unittest.TestCase):
    def test_single_arm_request_requires_only_its_active_target(self):
        request = PlanRequest(self.scene, (("left", 24),), (("left_tool0", Pose()),))
        self.assertEqual(PlanRequest.from_dict(request.to_dict()).identity, request.identity)
        with self.assertRaises(ValueError):
            PlanRequest(self.scene, (("left", 24),), (("right_tool0", Pose()),))
        with self.assertRaises(ValueError):
            PlanRequest(self.scene, (("left", 24), ("left", 20)), (("left_tool0", Pose()),))

    def setUp(self):
        positions = {name: 0.0 for name in reversed(ACTIVE_JOINTS)}
        self.state = RobotState.from_mapping(positions, 123)
        self.left = SceneObject("wall_box_24", (0.3, 0.4, 0.4), Pose((1.5, 0.8, 1.8)))
        self.right = SceneObject("wall_box_20", (0.3, 0.4, 0.4), Pose((1.5, -0.8, 1.8)))
        self.scene = SceneSnapshot("model-sha", self.state, (self.left, self.right))

    def attachment(self, item, side):
        return AttachedObject(item.object_id, item.dimensions_m, side + "_tool0", Pose((0, 0, 0.15)))

    def test_named_state_orders_without_guessing_missing_axes(self):
        state = RobotState.from_mapping({name: index for index, name in enumerate(ACTIVE_JOINTS)}, 123)
        self.assertEqual(state.ordered(tuple(reversed(ACTIVE_JOINTS))), tuple(reversed(range(15))))
        with self.assertRaisesRegex(ValueError, "missing joints"):
            RobotState(("updown",), (0.0,), 123).ordered(ACTIVE_JOINTS)

    def test_snapshot_copies_caller_containers(self):
        objects = [self.left]
        snapshot = replace(self.scene, objects=objects)
        objects.clear()
        self.assertEqual(snapshot.objects, (self.left,))

    def test_partial_trajectory_update_preserves_unplanned_state(self):
        state = RobotState.from_mapping({"updown": -0.3, "head_joint": 0.2}, 123)
        updated = state.with_positions({"updown": -0.6, "base_yaw": math.pi}, Pose((1, 0, 0)))
        self.assertEqual(updated.ordered(("head_joint", "updown", "base_yaw")), (0.2, -0.6, math.pi))
        self.assertEqual(updated.base_pose.position, (1, 0, 0))
        self.assertEqual(updated.stamp_ns, 123)
        self.assertEqual(state.positions, (-0.3, 0.2))

    def test_ground_geometry_cannot_silently_disagree_with_plane(self):
        ground = SceneObject("ground", (10, 10, 0.2), Pose((0, 0, -0.1)))
        snapshot = replace(self.scene, objects=self.scene.objects + (ground,))
        self.assertEqual(len(world_objects(snapshot)), 2)
        raised = replace(ground, pose=Pose((0, 0, 0.1)))
        with self.assertRaisesRegex(ValueError, "ground plane"):
            world_objects(replace(snapshot, objects=self.scene.objects + (raised,)))

    def test_geometry_update_keeps_old_snapshot_immutable(self):
        store = SceneStore(self.scene)
        moved = replace(self.left, pose=Pose((2.0, 0.8, 1.8)))
        updated = store.upsert(moved, self.scene.revision)
        self.assertEqual(self.scene.object("wall_box_24").pose.position[0], 1.5)
        self.assertEqual(updated.object("wall_box_24").pose.position[0], 2.0)
        self.assertNotEqual(updated.geometry_key, self.scene.geometry_key)
        self.assertFalse(store.is_current(self.scene))

    def test_both_attachments_commit_once_and_remove_world_copies(self):
        store = SceneStore(self.scene)
        attached = store.attach_many((self.attachment(self.left, "left"),
                                     self.attachment(self.right, "right")), 0)
        self.assertEqual(attached.revision, 1)
        self.assertEqual(attached.objects, ())
        self.assertEqual(len(attached.attachments), 2)
        self.assertEqual(len(self.scene.objects), 2)

    def test_invalid_dual_attachment_does_not_partially_commit(self):
        store = SceneStore(self.scene)
        missing = AttachedObject("missing", (0.3, 0.4, 0.4), "right_tool0", Pose())
        with self.assertRaises(StopIteration):
            store.attach_many((self.attachment(self.left, "left"), missing), 0)
        self.assertIs(store.snapshot(), self.scene)

    def test_release_can_delete_or_restore_world_object(self):
        store = SceneStore(self.scene)
        store.attach(self.attachment(self.left, "left"), 0)
        deleted = store.release(self.left.object_id, 1)
        self.assertEqual(deleted.attachments, ())
        self.assertEqual(deleted.objects, (self.right,))
        store = SceneStore(self.scene)
        store.attach(self.attachment(self.left, "left"), 0)
        released = store.release(self.left.object_id, 1, Pose((0, 0, 0.2)))
        self.assertEqual(released.object(self.left.object_id).pose.position, (0.0, 0.0, 0.2))

    def test_stale_writes_rejected_and_duplicate_ownership_rejected(self):
        store = SceneStore(self.scene)
        store.update_state(replace(self.state, stamp_ns=124), 0)
        with self.assertRaisesRegex(ValueError, "revision"):
            store.upsert(self.left, 0)
        with self.assertRaisesRegex(ValueError, "both world and attached"):
            replace(self.scene, attachments=(self.attachment(self.left, "left"),))

    def test_cache_changes_for_geometry_policy_base_attachment_or_spheres(self):
        task = {"left": 24, "right": 20}
        spheres = {"centers": [[0, 0, 0]], "radii": [0.1]}
        key = resource_key(self.scene, task, spheres)
        variants = [replace(self.scene, objects=(replace(self.left, pose=Pose()), self.right)),
                    replace(self.scene, policy=CollisionPolicy(max_box_tilt_deg=30)),
                    replace(self.scene, state=replace(self.state, base_pose=Pose((0.1, 0, 0))))]
        store = SceneStore(self.scene)
        variants.append(store.attach(self.attachment(self.left, "left"), 0))
        for variant in variants:
            self.assertNotEqual(resource_key(variant, task, spheres), key)
        self.assertNotEqual(resource_key(self.scene, task, {"radii": [0.2]}), key)
        self.assertEqual(resource_key(replace(self.scene, state=replace(self.state, stamp_ns=125)), task, spheres), key)

    def test_world_to_base_conversion_moves_robot_reference_only(self):
        base = Pose((1.0, 2.0, 0.0), (math.sqrt(0.5), 0, 0, math.sqrt(0.5)))
        world = Pose((1.0, 3.0, 0.0))
        local = relative_pose(world, base)
        self.assertTrue(np.allclose(local.position, (1, 0, 0), atol=1e-12))
        snapshot = replace(self.scene, state=replace(self.state, base_pose=base))
        objects = world_objects(snapshot, mobile=True)
        self.assertEqual(objects[0].pose, self.left.pose)
        self.assertEqual(snapshot.object(self.left.object_id).pose, self.left.pose)

    def test_contact_exclusion_is_explicit_and_does_not_delete_snapshot_objects(self):
        self.assertEqual(len(world_objects(self.scene, ("wall_box_24",))), 1)
        strict = replace(self.scene, policy=CollisionPolicy(exclude_task_objects_before_contact=False))
        self.assertEqual(len(world_objects(strict, ("wall_box_24",))), 2)
        self.assertEqual(len(self.scene.objects), 2)

    def test_serialization_preserves_versions_and_request_targets(self):
        request = PlanRequest(self.scene, (("left", 24), ("right", 20)),
                              (("left_tool0", Pose()), ("right_tool0", Pose())))
        restored = PlanRequest.from_dict(json.loads(json.dumps(request.to_dict())))
        self.assertEqual(restored, request)
        self.assertEqual(restored.identity, request.identity)
        self.assertNotEqual(replace(request, snapshot=replace(self.scene, revision=1)).identity, request.identity)

    def test_invalid_data_and_task_selection_rejected(self):
        for make in (lambda: Pose((math.nan, 0, 0)), lambda: Pose(quaternion_wxyz=(0, 0, 0, 0)),
                     lambda: RobotState(("updown", "updown"), (0, 0), 1)):
            with self.assertRaises(ValueError):
                make()
        with self.assertRaises(ValueError):
            PlanRequest(self.scene, (("left", 24), ("right", 24)),
                        (("left_tool0", Pose()), ("right_tool0", Pose())))

    def test_core_has_no_moveit_ros_or_viewer_imports(self):
        for path in (ROOT / "tools/curobo_core").glob("*.py"):
            tree = ast.parse(path.read_text())
            for node in ast.walk(tree):
                modules = [alias.name for alias in node.names] if isinstance(node, ast.Import) else (
                    [node.module or ""] if isinstance(node, ast.ImportFrom) else [])
                for module in modules:
                    self.assertNotIn(module.split(".")[0],
                                     ("viser", "rerun", "rclpy", "moveit", "v3_interactive_60_tasks"))


if __name__ == "__main__":
    unittest.main()
