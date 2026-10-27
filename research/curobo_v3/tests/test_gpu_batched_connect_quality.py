import sys
from pathlib import Path
import unittest

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from v3_batched_loaded_search import batched_rrt_connect_multi_goal


class FreeValidity:
    def __init__(self, quality):
        self.weights = torch.ones(1, device="cuda")
        self.joint_names = ["joint"]
        self.stability_weight = 0.
        self.path_quality_callback = quality

    def mask(self, values):
        return torch.ones(len(values), device=values.device, dtype=torch.bool)

    def edges(self, starts, goals, return_stability=False, **_):
        valid = torch.ones(len(starts), device=starts.device, dtype=torch.bool)
        stability = torch.zeros(len(starts), device=starts.device)
        return (valid, stability) if return_stability else valid


@unittest.skipUnless(torch.cuda.is_available(), "requires CUDA")
class BatchedConnectQualityTest(unittest.TestCase):
    def test_final_path_quality_gate_is_enforced(self):
        start = torch.tensor([0.], device="cuda")
        goals = torch.tensor([[.01]], device="cuda")
        lower = torch.tensor([-1.], device="cuda")
        upper = -lower

        rejected, _ = batched_rrt_connect_multi_goal(
            start, goals, lower, upper, FreeValidity(lambda _: False), .1, 1,
            stop_at_first_valid=True)
        accepted, _ = batched_rrt_connect_multi_goal(
            start, goals, lower, upper, FreeValidity(lambda _: True), .1, 1,
            stop_at_first_valid=True)

        self.assertIsNone(rejected)
        self.assertIsNotNone(accepted)


if __name__ == "__main__":
    unittest.main()
