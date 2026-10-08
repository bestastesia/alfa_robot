from dataclasses import replace
from pathlib import Path
import sys
import unittest
from importlib.util import find_spec

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from curobo_core.contracts import ACTIVE_JOINTS, PlanRequest
from curobo_core.scene import Pose, RobotState, SceneObject, SceneSnapshot, digest


class SearchSeedTest(unittest.TestCase):
    def setUp(self):
        state = RobotState.from_mapping(dict.fromkeys(ACTIVE_JOINTS, 0.), 1)
        box = SceneObject('wall_box_02', (.3, .4, .4), Pose((1.4, 0., .2)))
        self.request = PlanRequest(SceneSnapshot('model', state, (box,)), (('left', 2),),
                                   (('left_tool0', Pose()), ('right_tool0', Pose())), 'side')

    def test_legacy_identity_and_seeded_roundtrip(self):
        legacy = self.request.to_dict()
        self.assertNotIn('search_seed', legacy)
        self.assertEqual(PlanRequest.from_dict(legacy).identity, digest(legacy))
        seeded = replace(self.request, search_seed=104729)
        self.assertEqual(PlanRequest.from_dict(seeded.to_dict()), seeded)
        self.assertNotEqual(seeded.identity, self.request.identity)
        self.assertEqual(seeded.snapshot.geometry_key, self.request.snapshot.geometry_key)
        for invalid in (True, -1, 2**32, 1.5, '104729'):
            with self.assertRaises(ValueError):
                replace(self.request, search_seed=invalid)

    @unittest.skipUnless(find_spec("curobo") is not None, "requires pinned cuRobo SDK; checked in SDK environment")
    def test_single_task_shoulder_window_preserves_physical_limits_and_pair(self):
        import math
        import numpy as np
        import torch
        from curobo_core.backend import CuroboBackend
        backend = object.__new__(CuroboBackend)
        backend.home_values = np.zeros(15, dtype=np.float32)
        backend.home_values[1] = math.radians(155)
        backend.home_values[8] = math.radians(-155)
        lower, upper = torch.full((15,), -math.pi), torch.full((15,), math.pi)
        original_lower, original_upper = lower.clone(), upper.clone()
        for boxes in ({"left": 2}, {"right": 7}):
            low, high = backend.motion_bounds(lower, upper, boxes)
            self.assertAlmostEqual(float(low[1]), math.radians(-25), places=6)
            self.assertAlmostEqual(float(high[8]), math.radians(25), places=6)
            self.assertEqual(float(high[1]), float(upper[1]))
            self.assertEqual(float(low[8]), float(lower[8]))
            others = [i for i in range(15) if i not in (1, 8)]
            self.assertTrue(torch.equal(low[others], lower[others]))
            self.assertTrue(torch.equal(high[others], upper[others]))
        low, high = backend.motion_bounds(lower, upper, {"left": 24, "right": 20})
        self.assertTrue(torch.equal(low, original_lower) and torch.equal(high, original_upper))
        self.assertTrue(torch.equal(lower, original_lower) and torch.equal(upper, original_upper))
        self.assertEqual(backend.motion_weights({"left": 2}).tolist(), [5.] + [1.] * 7 + [3.] * 7)
        self.assertEqual(backend.motion_weights({"right": 7}).tolist(), [5.] + [3.] * 7 + [1.] * 7)
        self.assertEqual(backend.motion_weights({"left": 24, "right": 20}).tolist(), [5.] + [1.] * 14)

    @unittest.skipUnless(find_spec("curobo") is not None, "requires pinned cuRobo SDK; checked in SDK environment")
    def test_rrt_shortcut_is_explicit_and_loaded_default_unchanged(self):
        from unittest.mock import patch
        from curobo_core.backend import CuroboBackend
        backend = object.__new__(CuroboBackend)
        with patch("curobo_core.backend.batched_rrt_multi_goal", return_value=(None, {})) as search:
            for kind in ("rrt", "informed_rrt"):
                backend.planner_kind = kind
                backend.search_path(None, None, None, None, None, 2., 104729)
                self.assertFalse(search.call_args.kwargs["apply_shortcut"])
                backend.search_path(None, None, None, None, None, 2., 104729, apply_shortcut=True)
                self.assertTrue(search.call_args.kwargs["apply_shortcut"])
                self.assertEqual(search.call_args.kwargs["informed_sampling"], kind == "informed_rrt")
                self.assertEqual(search.call_args.args[-2:], (2., 104729))

    @unittest.skipUnless(find_spec("curobo") is not None, "requires pinned cuRobo SDK; checked in SDK environment")
    def test_legacy_stage_seeds_and_explicit_wrap(self):
        from curobo_core.backend import CuroboBackend
        backend = object.__new__(CuroboBackend)
        backend.search_seed = None
        self.assertEqual(backend.search_seed_for(), 20261003)
        self.assertEqual(backend.search_seed_for(1), 20261004)
        backend.search_seed = 104729
        self.assertEqual(backend.search_seed_for(8), 104737)
        backend.search_seed = 2**32-1
        self.assertEqual(backend.search_seed_for(1), 0)


if __name__ == '__main__':
    unittest.main()
