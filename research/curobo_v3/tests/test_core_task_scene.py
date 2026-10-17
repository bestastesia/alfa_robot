from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from curobo_core.fixtures import tasks
from v3_task_scene import row_obstacles, grasp_transform
from v3_placement_target import placement_tool_poses


class TaskSceneTest(unittest.TestCase):
    def test_task_partition_and_unique_pairs(self):
        original = tasks()
        bottom = tasks([1, 0])
        self.assertEqual(len(original), 60)
        self.assertEqual(len(bottom), 34)
        identities = {tuple(sorted(pair.items())) for _, pair in original + bottom}
        self.assertEqual(len(identities), 94)
        for _, pair in original + bottom:
            self.assertNotEqual(pair['left'], pair['right'])
            self.assertLessEqual(abs(pair['left'] // 5 - pair['right'] // 5), 1)

    def test_fill_never_intersects_selected_target_volumes(self):
        for _, pair in tasks() + tasks([1, 0]):
            centers, obstacles = row_obstacles(0.2, pair)
            for obstacle in obstacles:
                center = np.asarray(obstacle['center'])
                half = np.asarray(obstacle['dimensions']) / 2
                self.assertTrue(np.all(half > 0))
                self.assertGreaterEqual(center[2] - half[2], 0)
                for target in centers.values():
                    overlaps = np.minimum(center + half, target + (.15, .2, .2)) - np.maximum(
                        center - half, target - (.15, .2, .2))
                    self.assertFalse(np.all(overlaps > 1e-8))

    def test_top_grasp_rigid_transform_keeps_box_upright(self):
        tool = np.eye(4)
        tool[:3, :3] = np.diag([1, -1, -1])
        box = tool @ grasp_transform('left', True)
        self.assertTrue(np.allclose(box[:3, :3], np.eye(3)))
        self.assertAlmostEqual(box[2, 3], -.2)
        self.assertTrue(np.allclose(placement_tool_poses()['left_tool0']['position'], (.6, .425, 1)))
        self.assertTrue(np.allclose(placement_tool_poses()['right_tool0']['position'], (.6, -.425, 1)))


if __name__ == '__main__':
    unittest.main()
