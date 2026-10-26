import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from probe_touch_ik import ProbeTouchIK
from probe_touch_motion import selected_endpoint, interpolate_motion, motion_payload


class FixedJointTest(unittest.TestCase):
    def test_all_seven_fixed_axes_recover_known_target_with_different_seed(self):
        solver = ProbeTouchIK(-.3)
        goal = np.deg2rad([65., -75., 20., -90., 0., 40., 90.])
        target = solver.forward(goal)
        seed = goal + np.deg2rad([2., 1., -3., 1., 2., -1., 2.])
        for index in range(7):
            with self.subTest(joint=index + 1):
                rows = solver.solve_fixed(target, seed, index, goal[index])
                self.assertGreater(len(rows), 0)
                self.assertLess(min(np.linalg.norm(row['joints'] - goal) for row in rows), .02 if index == 3 else 1e-3)
                for row in rows:
                    self.assertLessEqual(abs(row['joints'][index] - goal[index]), 1e-6)
                    self.assertLessEqual(row['position_error_m'], 2e-6)
                    self.assertLessEqual(row['rotation_error_rad'], 2e-6)

    def test_invalid_fixed_angle_is_rejected(self):
        solver = ProbeTouchIK(-.3)
        with self.assertRaises(ValueError):
            solver.solve_fixed(np.eye(4), np.zeros(7), 1, np.deg2rad(150.))

    def test_zero_then_selected_endpoint_preserves_original_lock(self):
        locked = dict(joint_index=2, angle_rad=.4)
        goal = np.array([.1, -.2, .4, -.6, .2, .5, .1])
        zero, context = selected_endpoint(None, locked)
        np.testing.assert_array_equal(zero, np.zeros(7))
        self.assertEqual(context['saved_lock'], locked)
        selected, destination = selected_endpoint(dict(joints=goal), locked)
        self.assertEqual(locked['angle_rad'], .4)
        points, timing = interpolate_motion(zero, selected, 5.)
        self.assertEqual(points[0]['positions'], [0.] * 7)
        np.testing.assert_array_equal(points[-1]['positions'], goal)
        payload = motion_payload(points, timing, 0, np.eye(4), motion_context=destination)
        self.assertEqual(payload['motion_context']['endpoint_lock']['angle_rad'], .4)
        with self.assertRaises(ValueError):
            selected_endpoint(dict(joints=np.zeros(7)), locked)


if __name__ == '__main__':
    unittest.main()
