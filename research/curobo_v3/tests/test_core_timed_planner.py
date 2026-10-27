import sys
from pathlib import Path
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from v3_single_arm_planner import SingleArmPlanner
from v3_timed_single_arm_planner import TimedSingleArmPlanner


class TimedPlannerRetryTest(unittest.TestCase):
    def test_conveyor_seed_retries_without_relaxing_validity(self):
        planner = object.__new__(TimedSingleArmPlanner)
        validity = object()
        with patch.object(SingleArmPlanner, "search_path", autospec=True,
                          side_effect=[(None, {"first": True}),
                                       ("path", {"second": True})]) as search:
            path, stats = planner.search_path("start", "goals", "lower", "upper",
                                              validity, 2.0, 20261013)
        self.assertEqual(path, "path")
        self.assertEqual([call.args[-1] for call in search.call_args_list],
                         [20261013, 20261014])
        self.assertIs(search.call_args_list[0].args[-3], validity)
        self.assertIs(search.call_args_list[1].args[-3], validity)
        self.assertEqual(stats["selected_seed"], 20261014)
        self.assertEqual([item["success"] for item in stats["seed_attempts"]],
                         [False, True])

    def test_other_search_seed_is_not_retried(self):
        planner = object.__new__(TimedSingleArmPlanner)
        with patch.object(SingleArmPlanner, "search_path", autospec=True,
                          return_value=(None, {})) as search:
            path, stats = planner.search_path(None, None, None, None, None, 2.0, 7)
        self.assertIsNone(path)
        self.assertEqual(search.call_count, 1)
        self.assertIsNone(stats["selected_seed"])


if __name__ == "__main__":
    unittest.main()
