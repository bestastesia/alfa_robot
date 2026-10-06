from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from curobo_core.adapter import task_contact_positions
from curobo_core.cache import resource_key
from curobo_core.contracts import ACTIVE_JOINTS, PlanRequest
from curobo_core.fixtures import WallLayout, tasks
from curobo_core.scene import Pose, RobotState, SceneObject, SceneSnapshot


class WallLayoutTest(unittest.TestCase):
    def test_default_matches_f044_geometry_and_catalog(self):
        layout = WallLayout()
        legacy = tuple(SceneObject(f'wall_box_{i:02d}', (.3, .4, .4), Pose((
            .508 + .9 + .15, (i % 5 - 2) * .41, .20 + (i // 5) * .41))) for i in range(25))
        self.assertEqual(layout.objects(.508), legacy)
        catalog = tasks(layout)
        self.assertEqual(len(catalog), 60)
        self.assertEqual(catalog[0], ('L24(排1/列1) · R20(排1/列5)', {'left': 24, 'right': 20}))
        self.assertEqual(catalog[-1][1], {'left': 12, 'right': 11})

    def test_non_square_missing_boxes_offsets_and_side_assignment(self):
        layout = WallLayout(3, 4, .75, .05, .82, .01, [0, 2, 5, 8, 11])
        objects = layout.objects(.508)
        self.assertEqual([item.object_id for item in objects],
                         ['wall_box_00', 'wall_box_02', 'wall_box_05', 'wall_box_08', 'wall_box_11'])
        self.assertAlmostEqual(objects[-1].pose.position[0] - .15 - .508, .75)
        self.assertAlmostEqual(objects[-1].pose.position[1], .05 + .615)
        self.assertAlmostEqual(objects[-1].pose.position[2], 1.84)
        self.assertEqual(tasks(layout)[0][1], {'left': 11, 'right': 8})
        for _, pair in tasks(layout):
            self.assertTrue(all(i in layout.active_box_ids for i in pair.values()))
            self.assertGreaterEqual(pair['left'] % 4, 2)
            self.assertLess(pair['right'] % 4, 2)
        self.assertEqual(tasks(WallLayout(active_box_ids=[])), [])
        self.assertEqual(tasks(WallLayout(active_box_ids=[0, 4]))[0][1], {'left': 4, 'right': 0})

    def test_invalid_layout_fails_before_allocating_scene(self):
        for changes in ({'rows': 0}, {'rows': True}, {'columns': 1.5}, {'rows': 1001},
                        {'distance_m': 0}, {'gap_m': -0.1}, {'center_y_m': math.nan},
                        {'active_box_ids': [1, 1]}, {'active_box_ids': [25]},
                        {'active_box_ids': [-1]}, {'active_box_ids': [True]}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                WallLayout(**changes)

    def test_contact_and_cache_use_snapshot_not_box_number(self):
        state = RobotState.from_mapping(dict.fromkeys(ACTIVE_JOINTS, 0.0), 1)
        layout = WallLayout(3, 4, active_box_ids=[8, 11])
        snapshot = SceneSnapshot('model', state, layout.objects(.508))
        pair = {'left': 11, 'right': 8}
        before = task_contact_positions(snapshot, pair)
        moved = replace(snapshot.objects[-1], pose=Pose((2.0, .123, .456)))
        updated = replace(snapshot, objects=(snapshot.objects[0], moved))
        after = task_contact_positions(updated, pair)
        self.assertEqual(after['left'], (1.85, .123, .456))
        self.assertNotEqual(before['left'], after['left'])
        self.assertEqual(before['right'], after['right'])
        self.assertNotEqual(resource_key(snapshot, pair, {}), resource_key(updated, pair, {}))
        request = PlanRequest(updated, tuple(pair.items()), (('left_tool0', Pose()), ('right_tool0', Pose())))
        self.assertEqual(PlanRequest.from_dict(request.to_dict()), request)
        with self.assertRaisesRegex(ValueError, 'absent'):
            PlanRequest(updated, (('left', 10), ('right', 8)), request.targets)
        with self.assertRaisesRegex(ValueError, 'invalid task'):
            PlanRequest(updated, (('left', True), ('right', 8)), request.targets)
        with self.assertRaises(ValueError):
            task_contact_positions(snapshot, pair, math.nan)


if __name__ == '__main__':
    unittest.main()
