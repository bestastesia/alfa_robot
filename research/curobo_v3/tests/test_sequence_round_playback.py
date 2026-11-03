from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from v3_task_full_cycle_demo import sequence_round_index, sequence_round_result, sequence_round_start


class SequenceRoundPlaybackTest(unittest.TestCase):
    def test_selected_round_resolves_result_and_global_start(self):
        first = {"task": {"left": 1}, "frames": [[1], [2]]}
        second = {"task": {"right": 2}, "frames": [[3], [4], [5]]}
        sequence = {
            "rounds": [{"task": first["task"], "result": first},
                       {"task": second["task"], "result": second}],
            "frame_rounds": [0, 0, 1, 1, 1],
        }
        self.assertEqual(sequence_round_index(sequence, second["task"]), 1)
        self.assertIs(sequence_round_result(sequence, second["task"]), second)
        self.assertEqual(sequence_round_start(sequence, second["task"]), 2)


if __name__ == "__main__":
    unittest.main()
