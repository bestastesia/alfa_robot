"""CPU-only coverage for exact farthest-first shortcut batching."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from v3_timed_single_arm_planner import reverse_shortcut_blocks


class TimedShortcutTest(unittest.TestCase):
    def test_reverse_blocks_cover_all_candidates_farthest_first(self):
        blocks = list(reverse_shortcut_blocks(0, 42, 16))
        self.assertEqual(blocks[0], (27, 42))
        self.assertEqual(blocks[-1], (1, 10))
        indices = [index for start, end in blocks for index in range(start, end + 1)]
        self.assertEqual(sorted(indices), list(range(1, 43)))
        self.assertEqual(len(indices), len(set(indices)))
        with self.assertRaises(ValueError):
            list(reverse_shortcut_blocks(2, 2))


if __name__ == "__main__":
    unittest.main()
