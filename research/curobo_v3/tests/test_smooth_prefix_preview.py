import hashlib
import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/ipc_left_prefix'))
from smooth_timing import smooth_points,rt_curves,validate_preview


class SmoothPreviewTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library=json.loads((Path(__file__).resolve().parents[1]/'generated/left_prefix_control_20261018/prefixes.json').read_text())

    def test_saved_paths_have_ten_hz_points_and_verified_derivative_peaks(self):
        for name,record in self.library['tasks'].items():
            points,report=smooth_points(record['model_positions'])
            self.assertLessEqual(len(points),1000,name)
            self.assertLessEqual(report['peak_velocity_deg_s'],15.+1e-6,name)
            self.assertLessEqual(report['peak_acceleration_deg_s2'],30.+1e-6,name)
            self.assertLessEqual(report['peak_jerk_deg_s3'],150.+1e-6,name)
            self.assertLessEqual(report['maximum_joint_deviation_deg'],.5,name)
            np.testing.assert_allclose(np.diff([point['time_s']for point in points]),.1,atol=1e-8)
            np.testing.assert_allclose(points[0]['positions'],record['model_positions'][0])
            np.testing.assert_allclose(points[-1]['positions'],record['model_positions'][-1])
            for index in (0,-1):
                self.assertEqual(points[index]['velocities'],[0.]*7)
                self.assertEqual(points[index]['accelerations'],[0.]*7)
            curves=rt_curves(points);stamps=np.linspace(1.,report['duration_s']-1.,1000)
            moving=np.column_stack([curve.derivative(1)(stamps)for curve in curves])
            self.assertGreater(float(np.max(np.abs(moving),axis=1).min()),1e-6,name)

    def test_first_path_is_smoothed_before_final_stop(self):
        points,report=smooth_points(self.library['tasks']['L18_R22']['model_positions'])
        self.assertLess(report['duration_s'],35.)
        self.assertLess(report['peak_acceleration_deg_s2'],20.)
        self.assertGreater(len(points),300)
        self.assertLess(len(points),400)

    def test_preview_without_collision_check_cannot_execute(self):
        points,report=smooth_points(self.library['tasks']['L18_R22']['model_positions'])
        report.update(collision_checked=False,left_task=True)
        payload=dict(task='L18_R22',points=points,report=report)
        payload['preview_id']=hashlib.sha256(json.dumps(payload,sort_keys=True).encode()).hexdigest()
        with self.assertRaisesRegex(ValueError,'碰撞'):
            validate_preview(payload)
        report['collision_checked']=True
        with self.assertRaisesRegex(ValueError,'校验'):
            validate_preview(payload)

    def test_duration_gate_does_not_increase_velocity_to_force_acceptance(self):
        points,report=smooth_points(self.library['tasks']['L7_R5']['model_positions'])
        self.assertGreater(report['duration_s'],60.)
        self.assertFalse(report['within_duration'])
        report.update(collision_checked=True,left_task=True)
        payload=dict(task='L7_R5',points=points,report=report)
        payload['preview_id']=hashlib.sha256(json.dumps(payload,sort_keys=True).encode()).hexdigest()
        with self.assertRaisesRegex(ValueError,'超过'):
            validate_preview(payload)


if __name__=='__main__':unittest.main()
