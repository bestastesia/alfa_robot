import math
import sys
from pathlib import Path
import unittest

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from v3_batched_loaded_search import GpuValidity, densify


@unittest.skipUnless(torch.cuda.is_available(), "requires CUDA")
class MarginEdgeResolutionTest(unittest.TestCase):
    def test_margin_edge_check_matches_final_dense_collision_resolution(self):
        validity = GpuValidity.__new__(GpuValidity)
        validity.weights = torch.ones(1, device="cuda")
        validity.search_edge_resolution = .05
        validity.search_scene_margin_m = .003
        validity.evaluate = lambda values, base_margin_m=0.: (
            ~((values[:, 0] > .02) & (values[:, 0] < .03)),
            torch.zeros(len(values), device=values.device),
        )
        validity.scene_margin_mask = lambda values, margin: torch.ones(
            len(values), device=values.device, dtype=torch.bool)

        start = torch.tensor([[0.]], device="cuda")
        goal = torch.tensor([[.05]], device="cuda")
        dense = torch.tensor(densify(np.asarray([[0.], [.05]]), [1.]), device="cuda")

        self.assertFalse(validity.mask(dense).all())
        self.assertFalse(validity.edges(start, goal).item())
        self.assertLessEqual(math.radians(.5), .05)

    def test_margin_itself_is_checked_at_final_dense_resolution(self):
        validity = GpuValidity.__new__(GpuValidity)
        validity.weights = torch.ones(1, device="cuda")
        validity.search_edge_resolution = .05
        validity.search_scene_margin_m = .003
        validity.evaluate = lambda values, base_margin_m=0.: (
            torch.ones(len(values), device=values.device, dtype=torch.bool),
            torch.zeros(len(values), device=values.device),
        )
        validity.scene_margin_mask = lambda values, margin: ~((values[:, 0] > .02) & (values[:, 0] < .03))

        start = torch.tensor([[0.]], device="cuda")
        goal = torch.tensor([[.05]], device="cuda")

        self.assertFalse(validity.edges(start, goal).item())


if __name__ == "__main__":
    unittest.main()
