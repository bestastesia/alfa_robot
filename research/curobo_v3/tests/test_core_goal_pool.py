from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import v3_goal_pool as pool


class GoalPoolTest(unittest.TestCase):
    def setUp(self):
        generator = torch.Generator().manual_seed(311)
        self.rows = torch.randn(3000, 15, generator=generator)
        self.ends = torch.randn(12, 15, generator=generator)

    def test_fast_pool_matches_direct(self):
        expected = pool.direct_goal_pool(self.ends, self.rows)
        actual = pool.exact_goal_pool(self.ends, self.rows)
        self.assertTrue(torch.equal(actual, expected))

    def test_boundary_ties_keep_original_order(self):
        rows = self.rows.clone()
        rows[:800] = 0.
        ends = torch.zeros(3, 15)
        self.assertTrue(torch.equal(pool.exact_goal_pool(ends, rows), pool.direct_goal_pool(ends, rows)))

    def test_trailing_boundary_ties_use_original_topk(self):
        rows = torch.randn(3000, 15, generator=torch.Generator().manual_seed(0))
        rows[-600:] = 0.
        ends = torch.zeros(1, 15)
        self.assertTrue(torch.equal(pool.exact_goal_pool(ends, rows), pool.direct_goal_pool(ends, rows)))

    def test_dense_boundary_uses_original_kernel(self):
        rows = torch.zeros_like(self.rows)
        direct = pool.direct_goal_pool
        with patch.object(pool, "direct_goal_pool", wraps=direct) as fallback:
            actual = pool.exact_goal_pool(self.ends, rows)
            fallback.assert_called_once()
        self.assertTrue(torch.equal(actual, direct(self.ends, rows)))

    def test_small_and_empty_pools(self):
        for rows, ends in ((self.rows[:200], self.ends), (self.rows[:0], self.ends), (self.rows, self.ends[:0])):
            self.assertTrue(torch.equal(pool.exact_goal_pool(ends, rows), pool.direct_goal_pool(ends, rows)))


if __name__ == "__main__":
    unittest.main()
