"""GPU regression for native scene updates; run only in a prepared isolated runtime."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))


@unittest.skipUnless(torch.cuda.is_available(), "requires CUDA")
class OnlineWorldUpdateTest(unittest.TestCase):
    def test_add_move_remove_obstacle_updates_warm_checker_without_stale_world(self):
        from curobo_core.backend import CuroboBackend
        from curobo_core.contracts import PlannerAssets
        from curobo_core.rolling_curobo import SegmentReplanner
        from curobo_core.scene import Pose, SceneObject

        backend = CuroboBackend(PlannerAssets())
        adapter = SegmentReplanner(backend, {})
        snapshot = backend.snapshot
        q = torch.tensor(backend.home_values, dtype=torch.float32, device="cuda").reshape(1, -1)
        initial = adapter.scene(snapshot)
        self.assertTrue(initial.mask(q).item(), "fixture must start collision-free")
        checker = initial.checker
        # A large cube certainly overlaps the robot; moving it away and removing
        # it must clear collision buffers, not merely update a version counter.
        blocking = SceneObject("online_world_test", (10., 10., 10.), Pose((0., 0., 2.)))
        far = replace(blocking, pose=Pose((50., 0., 2.)))
        for revision, (objects, expected) in enumerate(
                ((snapshot.objects + (blocking,), False),
                 (snapshot.objects + (far,), True),
                 (snapshot.objects + (blocking,), False),
                 (snapshot.objects, True)), 1):
            world = replace(snapshot, objects=objects, revision=revision)
            validity = adapter.scene(world)
            self.assertIs(validity.checker, checker)
            self.assertEqual(bool(validity.mask(q).item()), expected)


if __name__ == "__main__":
    unittest.main()
