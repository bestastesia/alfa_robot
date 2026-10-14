from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from v3_playback import playback_frame_interval


class PlaybackTest(unittest.TestCase):
    def test_angular_speed_limit_keeps_linear_joint_ratio(self):
        result = {'joint_names': ['updown', 'left_joint1', 'right_joint7'],
                  'frames': [[0, 0, 0], [.01, np.deg2rad(.5), np.deg2rad(.2)], [.02, np.deg2rad(.7), np.deg2rad(.8)]]}
        interval = playback_frame_interval(result)
        maximum_speed = np.max(np.abs(np.rad2deg(np.diff(np.asarray(result['frames'])[:, 1:], axis=0)))) / interval
        self.assertAlmostEqual(maximum_speed, 30)
        self.assertAlmostEqual(.01 / interval, .5)

    def test_navigation_teleport_is_not_a_motor_speed_limit(self):
        result = {'joint_names': ['base_x', 'base_yaw', 'left_joint1'],
                  'frames': [[0, 0, 0], [-.1, np.pi, 0], [-.1, np.pi, np.deg2rad(.5)]]}
        self.assertAlmostEqual(playback_frame_interval(result), .5 / 30)

    def test_still_result_and_invalid_speed(self):
        result = {'joint_names': ['left_joint1'], 'frames': [[0], [0]]}
        self.assertEqual(playback_frame_interval(result), .03)
        with self.assertRaises(ValueError):
            playback_frame_interval(result, 0)


if __name__ == '__main__':
    unittest.main()
