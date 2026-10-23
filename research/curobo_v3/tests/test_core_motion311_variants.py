from dataclasses import dataclass
from pathlib import Path
import sys
import types
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from curobo_core.scene import Pose, SceneObject
from v3_scene_variants import (OFFSET_LIMIT_M, OUT_OF_RANGE_LIMIT_M, SHIFTED_IDS, VARIANTS, DockingOffsetMixin,
                               all_tasks, box_cell, build_registry, draw_cells, registry_digest, task_key, task_split)
from v3_search_recorder import SEED_STRIDE, SearchRecorder


def mirror_box(box_id):
    row, column = divmod(box_id, 5)
    return row * 5 + 4 - column


def mirror_task(boxes):
    if len(boxes) == 2:
        return {"left": mirror_box(boxes["right"]), "right": mirror_box(boxes["left"])}
    side, box_id = next(iter(boxes.items()))
    return {("right" if side == "left" else "left"): mirror_box(box_id)}


class SplitTest(unittest.TestCase):
    def test_cells_group_mirrored_columns(self):
        self.assertEqual(box_cell(0), box_cell(4))
        self.assertEqual(box_cell(21), box_cell(23))
        self.assertEqual(box_cell(12), (2, 2))

    def test_task_universe(self):
        keys = [task_key(boxes) for boxes in all_tasks()]
        self.assertEqual(len(keys), 119)
        self.assertEqual(len(set(keys)), 119)
        self.assertEqual(sum(len(boxes) == 2 for boxes in all_tasks()), 94)

    def test_cell_draw_is_deterministic_and_constrained(self):
        test, dev = draw_cells()
        self.assertEqual((test, dev), draw_cells())
        self.assertEqual(sorted(column for _, column in test), [0, 1, 2])
        self.assertTrue(any(row <= 1 for row, _ in test) and any(row >= 2 for row, _ in test))
        self.assertEqual(sorted(row <= 1 for row, _ in dev), [False, True])
        self.assertFalse(set(test) & set(dev))

    def test_mirrored_tasks_share_a_split(self):
        test, dev = draw_cells()
        known = {task_key(boxes): boxes for boxes in all_tasks()}
        for boxes in all_tasks():
            mirrored = task_key(mirror_task(boxes))
            if mirrored in known:
                self.assertEqual(task_split(boxes, test, dev), task_split(known[mirrored], test, dev), boxes)


class RegistryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.registry = build_registry()

    def test_registry_is_reproducible_and_hashed(self):
        self.assertEqual(self.registry["sha256"], build_registry()["sha256"])
        self.assertEqual(self.registry["sha256"], registry_digest(self.registry))

    def test_instances_respect_split_rules(self):
        test_cells = {tuple(cell) for cell in self.registry["test_cells"]}
        splits, names = {}, set()
        for item in self.registry["instances"]:
            self.assertNotIn(item["instance"], names)
            names.add(item["instance"])
            splits.setdefault(item["task_key"], set()).add(item["split"])
            cells = {tuple(cell) for cell in item["cells"]}
            self.assertEqual(item["split"] == "test", bool(cells & test_cells), item["instance"])
            limit = OUT_OF_RANGE_LIMIT_M if item["group"] == "out_of_range" else OFFSET_LIMIT_M
            self.assertLessEqual(np.max(np.abs(item["offset_m"])), limit + 1e-12)
            if item["group"] == "out_of_range":
                self.assertGreater(np.max(np.abs(item["offset_m"])), OFFSET_LIMIT_M)
            if item["offset_m"] == [0.0, 0.0]:
                self.assertIn(item["group"], ("train", "dev"))
        self.assertTrue(all(len(value) == 1 for value in splits.values()))

    def test_variant_counts(self):
        for item in self.registry["instances"]:
            group = "train_dev" if item["group"] in ("train", "dev") else item["group"]
            expected = VARIANTS[group][item["kind"]] + (group == "train_dev")
            self.assertLess(item["variant"], expected)


@dataclass(frozen=True)
class FakeSnapshot:
    objects: tuple
    revision: int = 0


class FakePlanner:
    def configure_task(self, boxes):
        self.snapshot = FakeSnapshot(tuple(SceneObject(name, (.1, .1, .1), Pose((1., 2., 3.))) for name in
                                           ("ground", "front_wall", "wall_box_03", "task_fill_0", "conveyor_left")))
        return "configured"


class ShiftedPlanner(DockingOffsetMixin, FakePlanner):
    pass


class OffsetTest(unittest.TestCase):
    def test_only_pick_side_objects_move(self):
        planner = ShiftedPlanner()
        planner.scene_offset = (.02, -.01)
        self.assertEqual(planner.configure_task({"right": 3}), "configured")
        moved = {item.object_id: item.pose.position for item in planner.snapshot.objects}
        for name in ("ground", "front_wall", "wall_box_03", "task_fill_0"):
            self.assertAlmostEqual(moved[name][0], 1.02)
            self.assertAlmostEqual(moved[name][1], 1.99)
            self.assertEqual(moved[name][2], 3.)
        self.assertEqual(moved["conveyor_left"], (1., 2., 3.))
        self.assertEqual(planner.snapshot.revision, 1)
        self.assertTrue({"ground", "front_wall"} <= SHIFTED_IDS)

    def test_zero_offset_keeps_snapshot(self):
        planner = ShiftedPlanner()
        planner.configure_task({"right": 3})
        reference = planner.snapshot
        planner.configure_task({"right": 3})
        self.assertEqual(planner.snapshot, reference)


class RecorderTest(unittest.TestCase):
    def test_seed_offset_and_restore(self):
        seen = []

        def search(start, goals, lower, upper, validity, budget_s, seed, **kwargs):
            seen.append(seed)
            return None, {"iterations": 3, "search_ms": 1.0}

        module = types.SimpleNamespace(batched_rrt_connect_multi_goal=search)
        with SearchRecorder(2, {"fake": module}) as recorder:
            module.batched_rrt_connect_multi_goal(np.zeros(15), np.zeros((4, 15)), 0, 1, None, .5, 20261015)
        self.assertIs(module.batched_rrt_connect_multi_goal, search)
        self.assertEqual(seen, [20261015 + 2 * SEED_STRIDE])
        self.assertEqual(recorder.calls[0]["goals"], 4)
        self.assertFalse(recorder.calls[0]["success"])
        with SearchRecorder(0, {"fake": module}):
            module.batched_rrt_connect_multi_goal(np.zeros(15), np.zeros((1, 15)), 0, 1, None, .5, 7)
        self.assertEqual(seen[-1], 7)


if __name__ == "__main__":
    unittest.main()
