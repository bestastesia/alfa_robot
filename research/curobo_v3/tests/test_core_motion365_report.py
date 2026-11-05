from pathlib import Path
import json
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from experiments.motion365.b1_ablation_report import build, load_arm


def row(identity, success, cost=1.):
    return dict(instance=identity, success=success, status="success" if success else "task_failure",
                reason=None if success else "failed", selection_total_ms=10., selection_budget_exceeded=False,
                raw_t_path_s=cost if success else None, executed_t_path_s=cost if success else None,
                motion_duration_s=cost if success else None, outcomes=[{"risk_ms": 2.}])


class AblationReportTest(unittest.TestCase):
    def test_summary_and_pair_direction_are_explicit(self):
        arms = {"a": ({"x": row("x", True, 3.), "y": row("y", False)}, {"source_sha256": "a"}),
                "b": ({"x": row("x", True, 2.), "y": row("y", True, 4.)}, {"source_sha256": "b"})}
        report = build(arms)
        self.assertEqual(report["summaries"]["b"]["successes"], 2)
        pair = report["paired"]["a__b"]
        self.assertEqual((pair["a_label"], pair["b_label"], pair["b_only_success"], pair["b_minus_a_success"]),
                         ("a", "b", 1, 1))
        self.assertEqual(pair["both_success_costs"]["executed_t_path_s"]["median_b_minus_a"], -1.)

    def test_loader_requires_unique_nonempty_rows_and_reads_config(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            instances = root / "instances"
            instances.mkdir()
            (root / "config.json").write_text(json.dumps({"source_sha256": "source"}))
            (instances / "a.json").write_text(json.dumps(row("x", True)))
            label, rows, config = load_arm("test=" + str(instances))
            self.assertEqual((label, sorted(rows), config["source_sha256"]), ("test", ["x"], "source"))
            with self.assertRaises(ValueError):
                load_arm("bad-spec")


if __name__ == "__main__":
    unittest.main()
