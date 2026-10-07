from dataclasses import replace
from pathlib import Path
import sys
import unittest

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
