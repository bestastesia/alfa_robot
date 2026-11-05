from pathlib import Path
import json
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from motion306_learning.rankers import B1CostConfig

ROOT = Path(__file__).resolve().parents[1]


class Motion386CostMatrixTest(unittest.TestCase):
    def test_matrix_configs_load_and_change_only_soft_costs(self):
        matrix = json.loads((ROOT / "config/motion386_cost_matrix.json").read_text())
        self.assertEqual(matrix["schema"], "motion386-cost-matrix-v1")
        self.assertEqual(len(matrix["arms"]), 6)
        baseline = B1CostConfig.load(ROOT / matrix["baseline"])
        configs = {row["id"]: B1CostConfig.load(ROOT / row["config"]) for row in matrix["arms"]}
        for config in configs.values():
            self.assertEqual(config.selection_budget_ms, baseline.selection_budget_ms)
            self.assertEqual(config.direct_path_samples, baseline.direct_path_samples)
            self.assertEqual(config.approach_direct_risk_penalty_s, 0.)
            self.assertEqual(config.carry_direct_risk_penalty_s, 0.)
            self.assertEqual(config.place_direct_risk_penalty_s, baseline.place_direct_risk_penalty_s)
            self.assertEqual(config.home_direct_risk_penalty_s, baseline.home_direct_risk_penalty_s)
        self.assertEqual(configs["limit_guard"].joint_limit_penalty_s, 2.)
        self.assertEqual(configs["updown_stable"].carry_updown_penalty_s_per_m, 10.)
        self.assertEqual((configs["pool_robust"].contact_carry_mean_excess_weight,
                          configs["pool_robust"].contact_scarcity_penalty_s), (.5, 1.))
        self.assertEqual(configs["home_bias"].carry_home_proxy_weight, .25)
        self.assertEqual(configs["return_priority"].place_home_weight, 1.5)

    def test_matrix_paths_are_unique_and_exist(self):
        matrix = json.loads((ROOT / "config/motion386_cost_matrix.json").read_text())
        paths = [row["config"] for row in matrix["arms"]]
        self.assertEqual(len(paths), len(set(paths)))
        self.assertTrue(all((ROOT / path).is_file() for path in paths))


if __name__ == "__main__":
    unittest.main()
