from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from curobo_core.adapter import task_contact_positions
from curobo_core.cache import resource_key
from curobo_core.contracts import ACTIVE_JOINTS, PlanRequest
from curobo_core.fixtures import WallLayout, tasks, wall_sequence
from curobo_core.scene import Pose, RobotState, SceneObject, SceneSnapshot


class WallLayoutTest(unittest.TestCase):
    def test_default_matches_f044_geometry_and_includes_all_rows(self):
        layout = WallLayout()
        legacy = tuple(SceneObject(f'wall_box_{i:02d}', (.3, .4, .4), Pose((
            .508 + .9 + .15, (i % 5 - 2) * .41, .20 + (i // 5) * .41))) for i in range(25))
        self.assertEqual(layout.objects(.508), legacy)
        catalog = tasks(layout)
        self.assertEqual(len(catalog), 117)
        self.assertTrue(any("排5" in label for label, _ in catalog))
        self.assertEqual(catalog[0], ('L24(排1/列1) · R20(排1/列5)', {'left': 24, 'right': 20}))
        self.assertEqual(catalog[-1][1], {'left': 2})

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
            if 'left' in pair:
                self.assertGreaterEqual(pair['left'] % 4, 2)
            if 'right' in pair:
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


class SequenceTest(unittest.TestCase):
    def test_rounds_cover_every_box_once_in_row_order(self):
        for layout in (WallLayout(), WallLayout(3, 4, active_box_ids=[0, 2, 5, 8, 11]),
                       WallLayout(4, 1), WallLayout(active_box_ids=[])):
            sequence = wall_sequence(layout)
            ids = [i for task in sequence for i in task.values()]
            self.assertEqual(sorted(ids), list(layout.active_box_ids))
            self.assertEqual(len(set(ids)), len(ids))
            rows = [next(iter(t.values())) // layout.columns for t in sequence]
            self.assertEqual(rows, sorted(rows, reverse=True))
            for task in sequence:
                self.assertEqual(len({i // layout.columns for i in task.values()}), 1)
        sequence = wall_sequence(WallLayout())
        self.assertEqual(len(sequence), 15)
        self.assertEqual(sum(len(t) == 2 for t in sequence), 10)
        self.assertEqual(sequence[:3], [{'left': 24, 'right': 20}, {'left': 23, 'right': 21}, {'left': 22}])
        self.assertEqual(sequence[-3:], [{'left': 4, 'right': 0}, {'left': 3, 'right': 1}, {'left': 2}])

    def test_single_arm_request_and_sequence_stop_without_consuming_failed_boxes(self):
        from curobo_core.sequence import plan_sequence
        layout = WallLayout(1, 3)
        snapshot = SceneSnapshot('model', RobotState.from_mapping(dict.fromkeys(ACTIVE_JOINTS, 0.), 1),
                                 layout.objects(.508))
        class Planner:
            wall_layout = layout
            default_target_poses = {s + '_tool0': {'position': (0., 0., 0.), 'quaternion': (1., 0., 0., 0.)}
                                    for s in ('left', 'right')}
            fail_single = True
            def _plan_request(self, request, progress):
                if self.fail_single and len(request.tasks) == 1:
                    self.single_request = request
                    return {'success': False, 'error': {'code': 'test-blocked'}}
                removed = {i for _, i in request.tasks}
                released = replace(request.snapshot, objects=tuple(o for o in request.snapshot.objects
                                   if int(o.object_id.removeprefix('wall_box_')) not in removed), revision=3)
                zeros = [[0.] * 15] * 2
                return {'success': True, 'joint_names': list(ACTIVE_JOINTS), 'frames': zeros,
                        'phases': ['home', 'return_home'], 'payload': [False, False],
                        'time_from_start_s': [0., 1.], 'velocities': zeros,
                        'accelerations': zeros, 'jerks': zeros,
                        'predicted_scene_events': [{'phase': 'release', 'snapshot': released.to_dict()}]}
            def set_snapshot(self, current):
                self.snapshot = current
        planner = Planner()
        result = plan_sequence(planner, snapshot, lambda _: None)
        self.assertFalse(result['success'])
        self.assertEqual(result['blocked_round'], 2)
        self.assertEqual(result['completed_box_ids'], [2, 0])
        self.assertEqual(result['remaining_box_ids'], [1])
        self.assertEqual(planner.single_request.tasks, (('left', 1),))
        self.assertEqual(len(planner.single_request.snapshot.objects), 1)
        self.assertEqual(planner.single_request.snapshot.revision, 4)
        self.assertEqual(len(result['frames']), 2)
        self.assertEqual(result['frame_rounds'], [0, 0])
        self.assertEqual(len(snapshot.objects), 3)
        planner = Planner()
        planner.fail_single = False
        complete = plan_sequence(planner, snapshot, lambda _: None)
        self.assertTrue(complete['success'])
        self.assertEqual(complete['completed_box_ids'], [2, 0, 1])
        self.assertEqual(complete['remaining_box_ids'], [])
        self.assertEqual(complete['frame_rounds'], [0, 0, 1, 1])
        self.assertEqual(complete['time_from_start_s'], [0., 1., 1., 2.])
        self.assertEqual(complete['motion_duration_s'], 2.)
        self.assertEqual(len(complete['velocities']), len(complete['frames']))
        self.assertEqual(planner.snapshot.objects, ())

        class MixedTimingPlanner(Planner):
            fail_single = False

            def _plan_request(self, request, progress):
                result = super()._plan_request(request, progress)
                if len(request.tasks) == untimed_task_size:
                    for key in ("time_from_start_s", "velocities", "accelerations", "jerks"):
                        result.pop(key)
                return result

        class BadTimingPlanner(Planner):
            fail_single = False

            def _plan_request(self, request, progress):
                result = super()._plan_request(request, progress)
                result["velocities"] = []
                return result

        with self.assertRaisesRegex(RuntimeError, "timing arrays"):
            plan_sequence(BadTimingPlanner(), snapshot, lambda _: None)

        for untimed_task_size in (1, 2):
            with self.subTest(untimed_task_size=untimed_task_size):
                mixed = plan_sequence(MixedTimingPlanner(), snapshot, lambda _: None)
                self.assertTrue(mixed["success"])
                self.assertEqual(len(mixed["frames"]), 4)
                for key in ("time_from_start_s", "velocities", "accelerations", "jerks", "motion_duration_s"):
                    self.assertNotIn(key, mixed)
                self.assertEqual(sum("time_from_start_s" in r["result"] for r in mixed["rounds"]), 1)


if __name__ == '__main__':
    unittest.main()
