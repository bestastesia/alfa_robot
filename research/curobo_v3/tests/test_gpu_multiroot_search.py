import sys
from pathlib import Path
import unittest

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from v3_multiroot_search import batched_rrt_connect_multi_goal


class FreeValidity:
    def __init__(self):
        self.weights = torch.ones(2, device='cuda')
        self.joint_names = ['test_joint_a', 'test_joint_b']
        self.stability_weight = 0.0

    def mask(self, values):
        return torch.ones(len(values), device=values.device, dtype=torch.bool)

    def edges(self, starts, goals, return_stability=False):
        valid = torch.ones(len(starts), device=starts.device, dtype=torch.bool)
        return (valid, torch.zeros(len(starts), device=starts.device)) if return_stability else valid


@unittest.skipUnless(torch.cuda.is_available(), 'requires CUDA')
class MultiRootTest(unittest.TestCase):
    def test_path_starts_at_one_actual_root_and_ends_at_one_goal(self):
        roots = torch.tensor([[-1, 0], [-1, .5]], device='cuda', dtype=torch.float32)
        goals = torch.tensor([[1, 0], [1, .5]], device='cuda', dtype=torch.float32)
        lower = torch.tensor([-2, -2], device='cuda', dtype=torch.float32)
        path, stats = batched_rrt_connect_multi_goal(roots, goals, lower, -lower, FreeValidity(),
                                                   2, 42, stop_at_first_valid=True)
        self.assertIsNotNone(path)
        result = torch.as_tensor(path, device='cuda')
        self.assertLess(float(torch.linalg.vector_norm(roots - result[0], dim=-1).min()), 1e-5)
        self.assertLess(float(torch.linalg.vector_norm(goals - result[-1], dim=-1).min()), 1e-5)


if __name__ == '__main__':
    unittest.main()
