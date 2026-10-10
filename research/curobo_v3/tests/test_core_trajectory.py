import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from curobo_core.trajectory import (assemble_timed_cycle, expand_timed_trajectory,
                                    joint_state_trajectory, resample_path, time_parameterize_path,
                                    time_parameterize_stops, validate_timed_cycle)


class _Tensor:
    def __init__(self, value):
        self.value = np.asarray(value)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.value


class TrajectoryHelpersTest(unittest.TestCase):
    def test_weighted_resampling_preserves_endpoints_and_ignores_duplicates(self):
        path = np.array([[0., 0.], [0., 0.], [1., 0.], [1., 2.]], dtype=np.float32)
        result = resample_path(path, 5, weights=[2., 1.])
        np.testing.assert_allclose(result[0], path[0])
        np.testing.assert_allclose(result[-1], path[-1])
        np.testing.assert_allclose(result[2], [1., 0.])






    def test_stop_timing_preserves_polyline_and_limits(self):
        path = np.array([[0., 0.], [.5, -.25], [1., 0.]], dtype=np.float32)
        trajectory, validation = time_parameterize_stops(
            path, ["a", "b"], [1., 1.], [2., 2.], [10., 10.], sample_dt=.01)
        np.testing.assert_allclose(trajectory["positions"], path)
        np.testing.assert_allclose(trajectory["velocities"], 0.)
        np.testing.assert_allclose(trajectory["accelerations"], 0.)
        self.assertTrue(np.max(np.abs(trajectory["jerks"])) <= 10.)
        np.testing.assert_allclose(validation[[0, -1]], path[[0, -1]])
        self.assertEqual(trajectory["maximum_linear_deviation"], 0.)

    def test_validate_timed_cycle_rejects_uncovered_motion(self):
        result = validate_timed_cycle(
            ["home", "approach"], [0., 1.], [[0.], [.5]], [[0.], [0.]], [[0.], [0.]],
            [1.], [1.], [1.], [])
        self.assertFalse(result["success"])
        self.assertEqual(result["uncovered_motion_frames"], [1])
        result = validate_timed_cycle(
            ["home", "approach"], [0., 1.], [[0.], [.5]], [[0.], [0.]], [[0.], [0.]],
            [1.], [1.], [1.], [{"frame_start": 1, "frame_end": 1}])
        self.assertTrue(result["success"])

    def test_assemble_timed_cycle_keeps_zero_duration_event_frames(self):
        segment = {
            "joint_names": ["a"], "frame_start": 1, "frame_end": 2,
            "time_from_start_s": [0., 1.], "velocities": [[0.], [0.]],
            "accelerations": [[0.], [0.]], "jerks": [[1.], [2.]],
        }
        result = assemble_timed_cycle(4, [segment])
        self.assertEqual(result["time_from_start_s"], [0., 0., 1., 1.])
        self.assertEqual(result["jerks"], [[0.], [1.], [2.], [0.]])
        self.assertEqual(result["motion_duration_s"], 1.)

    def test_fixed_path_time_parameterization_is_rest_to_rest_and_bounded(self):
        path = np.array([[0., 0.], [.5, -.25], [1., 0.]], dtype=np.float32)
        trajectory, validation = time_parameterize_path(
            path, ["a", "b"], [1., 1.], [2., 2.], [10., 10.], sample_dt=.01)
        np.testing.assert_allclose(trajectory["positions"], path)
        velocity = np.asarray(trajectory["velocities"])
        acceleration = np.asarray(trajectory["accelerations"])
        np.testing.assert_allclose(velocity[[0, -1]], 0., atol=1e-7)
        np.testing.assert_allclose(acceleration[[0, -1]], 0., atol=1e-7)
        self.assertTrue(np.all(np.diff(trajectory["time_from_start_s"]) > 0))
        self.assertTrue(np.max(np.abs(trajectory["velocities"])) <= 1.0)
        self.assertTrue(np.max(np.abs(trajectory["accelerations"])) <= 2.0)
        self.assertTrue(np.max(np.abs(trajectory["jerks"])) <= 10.0)
        np.testing.assert_allclose(validation[[0, -1]], path[[0, -1]], atol=1e-7)

    def test_expand_timed_trajectory_zero_fills_fixed_joints(self):
        source = {
            "joint_names": ["arm"], "dt_s": .1, "time_from_start_s": [0., .1],
            "positions": [[1.], [2.]], "velocities": [[3.], [4.]],
            "accelerations": [[5.], [6.]], "jerks": [[7.], [8.]],
        }
        result = expand_timed_trajectory(source, ["base", "arm"])
        self.assertEqual(result["positions"], [[0., 1.], [0., 2.]])
        self.assertEqual(result["velocities"], [[0., 3.], [0., 4.]])

    def test_joint_state_serialization_reorders_all_derivatives(self):
        class State:
            joint_names = ["b", "a"]
            position = _Tensor([[[[1., 2.], [3., 4.]]]])
            velocity = _Tensor([[[[.1, .2], [.3, .4]]]])
            acceleration = _Tensor([[[[.01, .02], [.03, .04]]]])
            jerk = _Tensor([[[[.001, .002], [.003, .004]]]])
            dt = _Tensor([0.025])

        result = joint_state_trajectory(State(), ["a", "b"])
        self.assertEqual(result["time_from_start_s"], [0.0, 0.025])
        self.assertEqual(result["positions"], [[2., 1.], [4., 3.]])
        self.assertEqual(result["velocities"], [[.2, .1], [.4, .3]])
        self.assertEqual(result["accelerations"], [[.02, .01], [.04, .03]])
        self.assertEqual(result["jerks"], [[.002, .001], [.004, .003]])


if __name__ == "__main__":
    unittest.main()
