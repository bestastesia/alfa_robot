import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

import numpy as np
import yourdfpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from probe_touch_ik import ROOT, ProbeTouchIK
from probe_touch_motion import interpolate_motion, motion_payload
from v3_zero_model_candidate import COORDINATE_REVISION, OLD, JOINT_SIGNS


class LeftJ5ZeroTest(unittest.TestCase):
    def test_only_left_j5_shift_preserves_all_link_geometry(self):
        old = yourdfpy.URDF.load(OLD, load_meshes=False)
        new = yourdfpy.URDF.load(ROOT / 'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf', load_meshes=False)
        rng = np.random.default_rng(20261020)
        offsets = {'left': [-90,15,0,0,90,0,90], 'right': [90,-15,0,0,-90,0,-90]}
        for _trial in range(30):
            values = {name: float(rng.uniform(-.5, .5)) for name in new.actuated_joint_names}
            prior = dict(values)
            prior['left_joint5'] += np.pi
            for side in ('left', 'right'):
                angles = np.array([prior[f'{side}_joint{index}'] for index in range(1,8)])
                legacy = np.deg2rad(offsets[side]) + JOINT_SIGNS * angles
                prior.update({f'{side}_joint{index}': value for index, value in enumerate(legacy,1)})
            old.update_cfg(prior)
            new.update_cfg(values)
            for link in new.link_map:
                np.testing.assert_allclose(old.get_transform(link, old.base_link), new.get_transform(link, new.base_link), atol=1e-7)

    def test_old_preview_rejected_without_publishing(self):
        sys.path.insert(0, str(ROOT / 'tools/ipc_left_prefix'))
        from server import State
        points, report = interpolate_motion(np.zeros(7), np.ones(7) * .01)
        payload = motion_payload(points, report, 1, np.eye(4))
        state = State(dict(tasks={'sample': {}}, required_coordinate_revision=COORDINATE_REVISION), 'left7_joint_replay')
        state.positions = [0.] * 7
        state.motor_bounds = [(-np.pi, np.pi)] * 7
        state.publisher = Mock()
        state.prepared[payload['preview_id']] = dict(payload, coordinate_revision='old')
        with self.assertRaisesRegex(ValueError, '旧关节零位'):
            state.send_preview(payload['preview_id'], 'forward')
        state.publisher.assert_not_called()

    def test_new_zero_analytic_fk_roundtrip(self):
        solver = ProbeTouchIK(-.3)
        seed = np.deg2rad([65, -75, 20, -90, 0, 40, 90])
        target = solver.forward(seed)
        rows = solver.solve(target, seed)
        self.assertTrue(rows)
        self.assertLess(min(row['joint_distance_rad'] for row in rows), 1e-4)
        self.assertTrue(all(row['position_error_m'] < 2e-6 for row in rows))


if __name__ == '__main__':
    unittest.main()
