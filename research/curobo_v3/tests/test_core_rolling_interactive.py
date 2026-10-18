"""CPU-only checks for the live Viser stream and fresh per-stage commands."""
import json
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from v3_rolling_interactive import STAGES, command_for, stage_planning_rows
from v3_rolling_replan import PROGRESS_PREFIX, progress_line


class RollingInteractiveTest(unittest.TestCase):
    def test_live_stream_and_full_round_commands_are_fresh_and_stage_specific(self):
        line = progress_line("solve_finished", stage="transport", compute_wall_s=.123)
        self.assertTrue(line.startswith(PROGRESS_PREFIX))
        self.assertEqual(json.loads(line[len(PROGRESS_PREFIX):])["compute_wall_s"], .123)

        args = SimpleNamespace(runner=Path("runner.py"), execution_timeout_s=120.)
        controls = dict(obstacle_stage="transport", scenario="static", center=(1., 2., 3.),
                        size=(.1, .2, .3), velocity=(0., 0., 0.), link_delay=.02,
                        braking=.5, seed=7)
        cycle, output = Path("fresh.json"), Path("result.json")
        commands = {stage: command_for(args, cycle, controls, stage, output) for stage in STAGES}
        self.assertTrue(all("--stream-progress" in command and str(cycle) in command
                            for command in commands.values()))
        self.assertEqual(float(commands["contact_approach"][commands["contact_approach"].index("--compute-bound-s") + 1]), .2)
        self.assertIn("static", commands["transport"])
        self.assertIn("clear", commands["approach"])
        self.assertNotIn("--obstacle-center", commands["approach"])

        total, additive, diagnostics = stage_planning_rows({
            "pipeline_timing_ms": {"prepare_checkers": 2., "all_extract_and_validation": 3.,
                                   "goal_matching": 4., "multiroot_search": 5.,
                                   "selected_prefix_suffix_validation": 6.,
                                   "assembly_and_repeat_validation": 7.},
            "lazy_extraction": {"timings_ms": {"contact_generation_ms": 1.,
                                                "endpoint_prediction_ms": 8.,
                                                "prediction_ranking_ms": 9.}},
        }, "transport")
        self.assertEqual(total, 9.)
        self.assertEqual(len(additive), 2)
        self.assertEqual(diagnostics, [])


if __name__ == "__main__":
    unittest.main()
