"""CPU-only contract for the persistent six-stage rolling runner."""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from v3_rolling_round import STAGES, stage_args


class RollingRoundTest(unittest.TestCase):
    def test_stage_sessions_share_one_round_configuration(self):
        args = SimpleNamespace(
            initial_cycle=Path("cycle.json"), obstacle_stage="transport", scenario="static",
            obstacle_center=(1., 2., 3.), obstacle_velocity=(0., 0., 0.), obstacle_size=(.1, .1, .1),
            link_delay_s=.02, braking_duration_s=.5, execution_timeout_s=120., seed=7,
            output_dir=Path("out"), stream_progress=True)
        stages = [stage_args(args, stage) for stage in STAGES]
        self.assertEqual([stage.stage for stage in stages], list(STAGES))
        self.assertEqual([stage.scenario for stage in stages].count("static"), 1)
        self.assertTrue(all(stage.initial_cycle == Path("cycle.json") and stage.seed == 7 for stage in stages))
        self.assertEqual(stages[0].slices, 8)
        self.assertEqual(stages[2].slices, 32)
        self.assertEqual([stage.compute_bound_s for stage in stages], [.95, .05, .1, .65, .8, 1.1])


if __name__ == "__main__":
    unittest.main()
