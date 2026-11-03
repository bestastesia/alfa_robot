import sys
from pathlib import Path
import unittest

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from v3_multiroot_search import batched_rrt_connect_multi_goal


class CountingValidity:
    def __init__(self):
        self.weights = torch.ones(2, device='cuda')
        self.joint_names = ['state_a', 'state_b']
        self.stability_weight = 0.0
        self.edge_batches = []

    def mask(self, values):
        return torch.ones(len(values), device=values.device, dtype=torch.bool)

    def edges(self, starts, goals, return_stability=False):
        self.edge_batches.append(len(starts))
        valid = torch.ones(len(starts), device=starts.device, dtype=torch.bool)
        return (valid, torch.zeros(len(starts), device=starts.device)) if return_stability else valid


@unittest.skipUnless(torch.cuda.is_available(), 'requires CUDA')
class CartesianSearchSettingsTest(unittest.TestCase):
    def test_small_batches_reach_a_goal_and_respect_the_batch_size(self):
        start = torch.tensor([-1., 0.], device='cuda')
        goals = torch.tensor([[1., 0.], [1., .5]], device='cuda')
        lower = torch.tensor([-2., -2.], device='cuda')
        validity = CountingValidity()
        path, stats = batched_rrt_connect_multi_goal(start, goals, lower, -lower, validity, 2, 7,
                                                   stop_at_first_valid=True, batch=16, connect_candidates=4, step=.2)
        self.assertIsNotNone(path)
        result = torch.as_tensor(path, device='cuda')
        self.assertLess(float(torch.linalg.vector_norm(result[0] - start)), 1e-5)
        self.assertLess(float(torch.linalg.vector_norm(goals - result[-1], dim=-1).min()), 1e-5)
        self.assertLessEqual(max(validity.edge_batches), 16)


if __name__ == '__main__':
    unittest.main()
