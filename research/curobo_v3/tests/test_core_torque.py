import sys
from pathlib import Path
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from curobo_core.torque import (TorqueModel, _time_warp, combine_inertials,
                                constrain_timed_result, cuboid_inertia, geometry_digest,
                                load_joint_torque_profiles, scale_derivatives,
                                trajectory_identity_digest)


class TorqueHelpersTest(unittest.TestCase):
    def test_combine_inertials_preserves_mass_com_and_positive_tensor(self):
        mass, center, tensor = combine_inertials(
            2., [0., 0., 0.], np.eye(3), 1., [3., 0., 0.], np.eye(3) * .5)
        self.assertEqual(mass, 3.)
        np.testing.assert_allclose(center, [1., 0., 0.])
        self.assertTrue(np.all(np.linalg.eigvalsh(tensor) > 0))

    def test_joint_torque_profile_expands_and_validates_catalog_order(self):
        path = Path(__file__).resolve().parents[1] / "generated/v3_dynamics_20261019/erob_joint_profiles.json"
        document, profiles = load_joint_torque_profiles(path)
        self.assertEqual(document["source"]["manual_version"], "3.42")
        self.assertEqual(profiles["left_joint3"]["model"], "eRob142H120I")
        self.assertEqual(profiles["left_joint3"]["start_stop_peak_torque_nm"], 459)
        self.assertEqual(profiles["right_joint4"]["model"], "eRob142H160I")
        self.assertEqual(profiles["right_joint4"]["start_stop_peak_torque_nm"], 484)

    def test_payload_model_cache_includes_attachment_transform(self):
        model = object.__new__(TorqueModel)
        model.payload_mass_kg = 1.0
        model._models = {}
        model._create = lambda sides, transforms: transforms["left"][2][3]
        first = {"left": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, .15], [0, 0, 0, 1]]}
        second = {"left": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, .20], [0, 0, 0, 1]]}
        self.assertEqual(model._model(("left",), first), .15)
        self.assertEqual(model._model(("left",), second), .20)
        self.assertEqual(len(model._models), 2)

    def test_timed_result_applies_torque_retiming(self):
        class Values:
            def __init__(self, values): self.values = np.asarray(values)
            def detach(self): return self
            def cpu(self): return self
            def numpy(self): return self.values

        class Model:
            names = ["joint"]
            payload_mass_kg = 2.0
            torque_limit_scale = .9
            base_kp = type("Kinematics", (), {"joint_limits": type("Limits", (), {
                "velocity": [None, Values([2.])], "acceleration": [None, Values([4.])],
                "jerk": [None, Values([8.])]})()})()

            def constrain(self, segment, active_sides=(), tool_to_box=None, deadline=None):
                output = scale_derivatives(segment, 2.)
                audit = {"maximum_utilization": 1.0}
                return output, audit, {"success": True, "time_scale": 2., "after": audit}

            def torque(self, positions, velocities, accelerations, active_sides=(), tool_to_box=None):
                return np.asarray(positions) + velocities + accelerations, 0.0

        segment = {"name": "transport", "frame_start": 0, "frame_end": 1,
                   "joint_names": ["joint"], "duration_s": 1., "time_from_start_s": [0., 1.],
                   "positions": [[0.], [1.]], "velocities": [[2.], [2.]],
                   "accelerations": [[4.], [4.]], "jerks": [[8.], [8.]]}
        result = {"task": {"left": 1}, "tool_to_box": {}, "joint_names": ["joint"],
                  "frames": [[0.], [1.]], "phases": ["transport", "transport"],
                  "payload": [False, False], "timed_segments": [segment],
                  "time_from_start_s": [0., 1.], "velocities": segment["velocities"],
                  "accelerations": segment["accelerations"], "jerks": segment["jerks"]}
        result["baseline_trajectory_contract"] = {
            "collision_validation_success": True,
            "timed_geometry_sha256": geometry_digest(result["frames"]),
            "identity_sha256": trajectory_identity_digest(result),
            "joint_names": result["joint_names"],
        }
        updated = constrain_timed_result(result, Model())
        self.assertEqual(updated["motion_duration_s"], 2.)
        self.assertEqual(updated["velocities"], [[1.], [1.]])
        self.assertTrue(updated["torque_validation"]["success"])
        self.assertTrue(updated["torque_validation"]["geometry_frozen"])
        self.assertEqual(updated["torque_validation"]["maximum_position_delta"], 0.0)
        self.assertEqual(updated["torque_validation"]["retimed_segments"], 1)
        self.assertEqual(updated["timed_segments"][0]["torque_joint_names"], ["joint"])
        self.assertEqual(len(updated["timed_segments"][0]["torques_nm"]), 2)
        result["frames"][0][0] = .25
        with self.assertRaises(ValueError):
            constrain_timed_result(result, Model())


    def test_nonuniform_time_warp_keeps_positions_and_scales_constant_profile(self):
        segment = {"duration_s": 2., "time_from_start_s": [0., 1., 2.],
                   "positions": [[0.], [1.], [2.]],
                   "velocities": [[2.], [2.], [2.]],
                   "accelerations": [[4.], [4.], [4.]],
                   "jerks": [[8.], [8.], [8.]]}
        result = _time_warp(segment, [2., 2., 2.])
        self.assertEqual(result["positions"], segment["positions"])
        np.testing.assert_allclose(result["time_from_start_s"], [0., 2., 4.])
        np.testing.assert_allclose(result["velocities"], 1.)
        np.testing.assert_allclose(result["accelerations"], 1.)
        np.testing.assert_allclose(result["jerks"], 1.)

    def test_local_retiming_changes_only_time_and_derivatives(self):
        model = object.__new__(TorqueModel)
        model.torque_limit_scale = .9
        model._derivative_ratio = lambda segment: 0.
        def audit(segment, *_):
            values = np.max(np.abs(np.asarray(segment["velocities"])), axis=1)
            return {"maximum_utilization": float(values.max()),
                    "maximum_gravity_utilization": 0.,
                    "frame_maximum_utilization": values.tolist(),
                    "frame_maximum_gravity_utilization": [0.] * len(values)}
        model.audit = audit
        segment = {"duration_s": 1., "time_from_start_s": [0., .25, .5, .75, 1.],
                   "positions": [[0.], [.25], [.5], [.75], [1.]],
                   "velocities": [[0.], [.6], [1.2], [.6], [0.]],
                   "accelerations": [[0.]] * 5, "jerks": [[0.]] * 5}
        result, _, constraint = model.constrain(segment)
        self.assertTrue(constraint["success"])
        self.assertEqual(constraint["retiming_mode"], "local_window")
        self.assertEqual(result["positions"], segment["positions"])
        self.assertGreater(result["time_from_start_s"][-1], 1.)

    def test_uniform_retiming_uses_unscaled_end_time_when_duration_is_absent(self):
        source = {"time_from_start_s": [0., 1.], "positions": [[0.], [1.]],
                  "velocities": [[2.], [2.]], "accelerations": [[4.], [4.]],
                  "jerks": [[8.], [8.]]}
        self.assertEqual(scale_derivatives(source, 2.)["duration_s"], 2.)

    def test_uniform_retiming_scales_derivatives(self):
        source = {"duration_s": 1., "time_from_start_s": [0., 1.], "positions": [[0.], [1.]],
                  "velocities": [[2.], [2.]], "accelerations": [[4.], [4.]],
                  "jerks": [[8.], [8.]]}
        result = scale_derivatives(source, 2.)
        self.assertEqual(result["duration_s"], 2.)
        self.assertEqual(result["velocities"], [[1.], [1.]])
        self.assertEqual(result["accelerations"], [[1.], [1.]])
        self.assertEqual(result["jerks"], [[1.], [1.]])
        self.assertEqual(cuboid_inertia(12., [1., 2., 3.]).diagonal().tolist(), [13., 10., 5.])


if __name__ == "__main__":
    unittest.main()
