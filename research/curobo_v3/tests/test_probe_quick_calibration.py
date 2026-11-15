import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from probe_touch_ik import ProbeTouchIK
from probe_calibration import record_contact, measured_angles
from ipc_left_prefix.coordinate_mapping import EXECUTION_REVISION, motor_to_model, POLICY


class QuickCalibrationTest(unittest.TestCase):
    def test_quick_exact_pose_fk_and_nearby_seed(self):
        solver = ProbeTouchIK(-.3)
        seed = np.deg2rad([40., -50., 15., -60., 20., 30., 60.])
        target = solver.forward(seed)
        target[0, 3] += .002
        row = solver.solve_nearest(target, seed)
        self.assertIsNotNone(row)
        self.assertLess(row['position_error_m'], 2e-6)
        self.assertLess(row['rotation_error_rad'], 2e-6)
        self.assertLess(row['joint_distance_rad'], .1)

    def test_unreachable_target_returns_no_solution(self):
        solver = ProbeTouchIK(-.3)
        target = np.eye(4)
        target[:3, 3] = [20., 20., 20.]
        self.assertIsNone(solver.solve_nearest(target, np.zeros(7)))

    def test_local_numeric_pose_and_fixed_joint(self):
        from scipy.spatial.transform import Rotation
        solver = ProbeTouchIK(-.3)
        seed = np.deg2rad([40., -50., 15., -60., 20., 30., 60.])
        target = solver.forward(seed)
        target[0,3] += .002
        target[:3,:3] = Rotation.from_rotvec([.02,0,0]).as_matrix() @ target[:3,:3]
        for locked in (None, dict(joint_index=0, angle_rad=seed[0])):
            row = solver.solve_local(target, seed, locked)
            self.assertIsNotNone(row)
            self.assertLess(row['position_error_m'], 2e-6)
            self.assertLess(row['rotation_error_rad'], 2e-6)
            self.assertLess(row['joint_distance_rad'], .2)
            if locked:
                self.assertEqual(row['joints'][0], seed[0])

    def test_local_numeric_unreachable_fails(self):
        target = np.eye(4)
        target[:3,3] = [20,20,20]
        self.assertIsNone(ProbeTouchIK(-.3).solve_local(target, np.zeros(7)))

    def status(self, angle=1.):
        return dict(feedback_deg=[0., angle, 0., 0., 0., 0., 0.], feedback_age_s=.01,
            busy=False, coordinate_revision=EXECUTION_REVISION)

    def test_records_feedback_not_target_and_appends(self):
        solver = ProbeTouchIK(-.3)
        target = np.eye(4)
        target[:3, 3] = [20., 30., 40.]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'contacts.jsonl'
            with patch('probe_calibration.time.sleep'), patch('probe_calibration.time.monotonic', side_effect=[0., .25, 1., 1.25]):
                for _sample in range(2):
                    record_contact(lambda: self.status(), solver, path, 'fixed_point', target)
            records = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(len(records), 2)
            sample = records[0]
            self.assertEqual(sample['raw_motor_deg'][1], 1.)
            self.assertAlmostEqual(sample['model_joint_deg'][1], 1.-POLICY['motor_minus_model_deg'][1])
            np.testing.assert_allclose(sample['nominal_tip_in_footprint'], solver.forward(motor_to_model(np.deg2rad(sample['raw_motor_deg']))))
            self.assertFalse(np.allclose(np.asarray(sample['nominal_tip_in_footprint'])[:3, 3], target[:3, 3]))
            self.assertEqual(sample['point_id'], 'fixed_point')

    def test_stale_moving_and_wrong_revision_rejected(self):
        for changes in [dict(feedback_age_s=2.), dict(busy=True), dict(coordinate_revision='old')]:
            with self.assertRaises(ValueError):
                measured_angles(dict(self.status(), **changes))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'contacts.jsonl'
            readings = iter([self.status(1.), self.status(3.)])
            with patch('probe_calibration.time.sleep'), patch('probe_calibration.time.monotonic', side_effect=[0., .25]):
                with self.assertRaises(ValueError):
                    record_contact(lambda: next(readings), ProbeTouchIK(-.3), path, 'fixed', np.eye(4))
            self.assertFalse(path.exists())


if __name__ == '__main__':
    unittest.main()
