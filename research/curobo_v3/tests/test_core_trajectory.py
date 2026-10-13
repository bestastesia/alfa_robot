import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from curobo_core.trajectory import (assemble_timed_cycle, expand_timed_trajectory,
                                    joint_state_trajectory, resample_path, time_parameterize_path,
                                    time_parameterize_stops, trajectory_fallback_path,
                                    validate_timed_cycle)


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






    def test_sparse_fallback_does_not_stop_at_dense_collision_samples(self):
        sparse = np.array([[0., 0.], [.5, .25], [1., 0.]], dtype=np.float32)
        dense = np.vstack((np.linspace(sparse[0], sparse[1], 101),
                           np.linspace(sparse[1], sparse[2], 101)[1:]))
        path = trajectory_fallback_path(
            dense, {"trajectory_fallback_waypoints": sparse.tolist()})
        self.assertEqual(len(path), 3)
        trajectory, validation = time_parameterize_stops(
            path, ["a", "b"], [1., 1.], [2., 2.], [10., 10.], sample_dt=.01)
        samples = trajectory["sampled_trajectory"]
        speed = np.linalg.norm(samples["velocities"], axis=1)
        self.assertEqual(np.count_nonzero(np.isclose(speed, 0., atol=1e-7)), 3)
        self.assertEqual(len(samples["positions"]), len(validation))
        self.assertEqual(len(samples["velocities"]), len(validation))
        self.assertEqual(len(samples["accelerations"]), len(validation))
        self.assertEqual(len(samples["jerks"]), len(validation))
        for field, limit in (("velocities", 1.), ("accelerations", 2.), ("jerks", 10.)):
            self.assertLessEqual(np.max(np.abs(samples[field])), limit * (1 + 1e-6))

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
        samples = trajectory["sampled_trajectory"]
        np.testing.assert_array_equal(samples["positions"], validation)
        self.assertGreater(len(samples["positions"]), len(path))
        self.assertTrue(np.all(np.diff(samples["time_from_start_s"]) > 0))
        self.assertLessEqual(np.max(np.diff(samples["time_from_start_s"])), .01 + 1e-12)
        self.assertGreater(np.max(np.abs(samples["velocities"])), 0.)
        for field, limit in (("velocities", 1.), ("accelerations", 2.), ("jerks", 10.)):
            self.assertEqual(np.asarray(samples[field]).shape, validation.shape)
            self.assertLessEqual(np.max(np.abs(samples[field])), limit * (1 + 1e-6))

        # Exercise the real recording helper without importing the CUDA planner.
        import ast
        source = Path(__file__).resolve().parents[1] / "tools/curobo_core/planner.py"
        method = next(n for n in ast.walk(ast.parse(source.read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == "record_timed_trajectory")
        report = {"frames": path.tolist(), "phases": ["attach", "transport", "transport"],
                  "payload": [True] * 3, "timed_segments": []}
        namespace = {"np": np, "report": report, "mobile_names": ["a", "b"],
                     "expand_timed_trajectory": expand_timed_trajectory}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
        self.assertTrue(namespace["record_timed_trajectory"]("transport", 1, trajectory))
        self.assertEqual(report["phases"], ["attach"] + ["transport"] * (len(validation) - 1))
        self.assertEqual(report["payload"], [True] * len(validation))
        np.testing.assert_array_equal(report["frames"], validation)
        segment = report["timed_segments"][0]
        self.assertEqual(segment["frame_end"], len(validation) - 1)
        np.testing.assert_array_equal(segment["velocities"], samples["velocities"])

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

    def test_joint_state_serialization_rejects_invalid_contracts(self):
        from types import SimpleNamespace
        state = dict(joint_names=["a"], position=_Tensor([[0.], [1.]]),
                     velocity=_Tensor([[0.], [0.]]), acceleration=_Tensor([[0.], [0.]]),
                     jerk=_Tensor([[0.], [0.]]), dt=_Tensor([.025]))
        for field, value in (("velocity", _Tensor([[0.]])), ("jerk", None),
                             ("position", _Tensor([[[0.], [1.]], [[2.], [3.]]])),
                             ("dt", None), ("dt", _Tensor([.025, -1.])),
                             ("dt", _Tensor([.025, .05])), ("dt", _Tensor([float("nan")])),
                             ("joint_names", ["a", "a"])):
            with self.subTest(field=field, value=value):
                with self.assertRaises(ValueError):
                    joint_state_trajectory(SimpleNamespace(**{**state, field: value}), ["a"])

    def test_joint_state_serialization_removes_repeated_optimizer_padding(self):
        from types import SimpleNamespace
        state = SimpleNamespace(
            joint_names=["a"], position=_Tensor([[0.], [1.], [1.], [1.]]),
            velocity=_Tensor([[0.], [.5], [0.], [0.]]),
            acceleration=_Tensor([[0.], [.1], [0.], [0.]]),
            jerk=_Tensor([[0.], [.01], [0.], [0.]]), dt=_Tensor([.025]))
        result = joint_state_trajectory(state, ["a"])
        self.assertEqual(result["time_from_start_s"], [0., .025])
        self.assertEqual(result["positions"], [[0.], [1.]])
        self.assertEqual(result["velocities"], [[0.], [.5]])

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
