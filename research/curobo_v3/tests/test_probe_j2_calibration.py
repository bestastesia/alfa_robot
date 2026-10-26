from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from ipc_left_prefix.coordinate_mapping import motor_to_model, model_to_motor, EXECUTION_REVISION, POLICY
from ipc_left_prefix.smooth_timing import rt_curves, validate_preview
from probe_touch_motion import interpolate_motion, motion_payload, selected_endpoint


class J2CalibrationTest(unittest.TestCase):
    def test_fitted_corrections_replace_original_j2_offset(self):
        motor = np.deg2rad([10., 1., 20., 30., 40., 50., 60.])
        model = motor_to_model(motor)
        np.testing.assert_allclose(np.rad2deg(model-motor),
            [0.,.23638073957006833,-.5736353095123823,-1.1948599250492464,.2689652652013817,-.7586338532082705,0.],atol=1e-12)
        np.testing.assert_array_equal(model[[0,6]], motor[[0,6]])
        np.testing.assert_allclose(model_to_motor(model), motor, atol=1e-15)

    def test_payload_motor_coordinates_match_model_preview(self):
        start = motor_to_model(np.deg2rad([0., 1., 0., 0., 0., 0., 0.]))
        goal = np.deg2rad([2., 5., -4., 1., 0., 3., 2.])
        points, timing = interpolate_motion(start, goal)
        payload = motion_payload(points, timing, 10, np.eye(4))
        validate_preview(payload)
        self.assertEqual(payload['coordinate_revision'], EXECUTION_REVISION)
        self.assertAlmostEqual(np.rad2deg(payload['points'][0]['positions'][1]), 1.)
        self.assertAlmostEqual(np.rad2deg(payload['points'][-1]['positions'][1]), 5.+POLICY['motor_minus_model_deg'][1])
        original = rt_curves(points)
        sent = rt_curves(payload['points'])
        stamps = np.linspace(0, timing['duration_s'], 101)
        for order in (0, 1, 2):
            expected = np.column_stack([curve.derivative(order)(stamps) for curve in original])
            actual = np.column_stack([curve.derivative(order)(stamps) for curve in sent])
            if order == 0:
                actual = motor_to_model(actual)
            np.testing.assert_allclose(actual, expected, atol=1e-9)

    def test_model_zero_and_fixed_axis_use_same_mapping(self):
        locked = dict(joint_index=1, angle_rad=float(motor_to_model(np.deg2rad([0, 21, 0, 0, 0, 0, 0]))[1]))
        zero, context = selected_endpoint(None, locked)
        np.testing.assert_allclose(np.rad2deg(model_to_motor(zero)), POLICY['motor_minus_model_deg'])
        self.assertAlmostEqual(np.rad2deg(context['saved_lock']['angle_rad']), 21.-POLICY['motor_minus_model_deg'][1])
        goal = np.zeros(7)
        goal[1] = locked['angle_rad']
        selected, _context = selected_endpoint(dict(joints=goal), locked)
        self.assertAlmostEqual(np.rad2deg(model_to_motor(selected)[1]), 21.)


if __name__ == '__main__':
    unittest.main()
