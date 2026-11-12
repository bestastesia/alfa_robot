"""GPU: pruning the subtrees below dense-invalid tree edges lets the multi-goal RRT pass a thin wall (MOTION-443)."""
from pathlib import Path
import sys
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))


class ThinWallValidity:
    """States collide inside a 2 cm wall at x = 0 where |y| < 0.3: the RRT's 0.05 coarse edge checks
    can step over it, its 0.5 deg dense checks cannot."""
    joint_names = ['x', 'y', 'z']
    stability_weight = 0.

    def __init__(self):
        self.weights = torch.ones(3, device='cuda')

    def evaluate(self, values):
        inside = (values[:, 0] > 0.) & (values[:, 0] < .02) & (values[:, 1].abs() < .3)
        return ~inside, torch.zeros(len(values), device=values.device)

    def mask(self, values):
        return self.evaluate(values)[0]

    def edges(self, starts, goals, resolution=.05, return_stability=False):
        from v3_batched_loaded_search import GpuValidity
        return GpuValidity.edges(self, starts, goals, resolution, return_stability)


@unittest.skipUnless(torch.cuda.is_available(), 'requires CUDA')
class PruningTests(unittest.TestCase):
    def test_pruned_search_finds_a_densely_valid_path_around_the_wall(self):
        from v3_batched_loaded_search import batched_rrt_connect_multi_goal, densify
        start = torch.tensor([-1., 0., 0.], device='cuda')
        goals = torch.tensor([[1., 0., 0.], [1., .2, .1]], device='cuda')
        lower, upper = torch.full((3,), -2., device='cuda'), torch.full((3,), 2., device='cuda')
        for seed in range(4):
            validity = ThinWallValidity()
            path, stats = batched_rrt_connect_multi_goal(start, goals, lower, upper, validity, 10., seed,
                                                         stop_at_first_valid=True, prune_invalid_edges=True)
            self.assertIsNotNone(path, f'seed {seed}: {stats}')
            dense = torch.tensor(densify(path, [1., 1., 1.]), device='cuda', dtype=torch.float32)
            self.assertTrue(bool(validity.mask(dense).all()))
            self.assertGreater(stats['pruned_nodes'], 0)


if __name__ == '__main__':
    unittest.main()
