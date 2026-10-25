import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from experiments.motion311 import ab_report as report


def selected_entry():
    return dict(arm="B", class_="success", selected_contact=2,
                selection=dict(contact=dict(order=[2]), abstained=False,
                               carry=[dict(choice=4)], place=[dict(outcomes=["success"], attempt_ranks=[0])]),
                timings=dict(lazy=dict(batches=[dict(kind="selection", allowed_contacts=[2], success=True)])),
                searches=[dict(module="multiroot", starts=1, goals=1, success=True)], trace=[])


class SelectionReportTest(unittest.TestCase):
    def test_first_selected_triple_requires_single_candidate_success(self):
        self.assertIs(report.first_selected_triple(selected_entry()), True)

    def test_stage_fallback_is_not_first_try(self):
        entry = selected_entry()
        entry["timings"]["lazy"]["batches"] = [dict(kind="selection", success=False), dict(success=True)]
        self.assertIs(report.first_selected_triple(entry), False)
        entry = selected_entry()
        entry["selection"]["place"][0]["outcomes"] = ["龙头车放置RRT", "success"]
        self.assertIs(report.first_selected_triple(entry), False)

    def test_baseline_and_missing_diagnostics_are_not_single_choice_evidence(self):
        entry = selected_entry()
        entry["selection"]["abstained"] = True
        self.assertIsNone(report.first_selected_triple(entry))
        entry = selected_entry()
        entry["selection"]["carry"][0]["choice"] = None
        self.assertIsNone(report.first_selected_triple(entry))
        self.assertIsNone(report.first_selected_triple(dict(arm="A", class_="success")))
        self.assertIsNone(report.first_selected_triple(dict(arm="B", class_="success")))

    def test_single_arm_contact_retry_is_not_first_try(self):
        entry = selected_entry()
        entry["timings"] = {}
        entry["selected_contact"] = 3  # single-arm result is one-based
        entry["trace"] = [dict(call="cartesian_single", along_normal=False, ok=True),
                          dict(call="single_path", goals=1, ok=True)]
        entry["searches"] = [dict(module="single_arm", starts=1, goals=1, success=True)]
        self.assertIs(report.first_selected_triple(entry), True)
        entry["trace"].insert(0, dict(call="cartesian_single", along_normal=False, ok=False))
        self.assertIs(report.first_selected_triple(entry), False)

    def test_missing_digests_do_not_prove_frame_equivalence(self):
        record = dict(instance="one", arms={"A": dict(class_="budget_exhausted", frames_digest=None),
                                            "B": dict(class_="budget_exhausted", selection=dict(fallback={"first_stage": "rrt"}), frames_digest=None)})
        result = report.g3([record])
        self.assertIsNone(result["frames_equal"])
        self.assertEqual(result["checks"], 0)
        self.assertEqual(len(result["unverifiable"]), 1)
        for entry in record["arms"].values():
            entry["frames_digest"] = "partial"
        result = report.g3([record])
        self.assertIsNone(result["frames_equal"])
        self.assertEqual(result["partial_frames_equal"], 1.)
        self.assertEqual(result["partial_checks"], 1)

    def test_explicit_b_compares_with_offline_bx_without_a(self):
        entry = selected_entry()
        entry["contact_value"] = "explicit"
        record = dict(instance="one", arms={"B": entry})
        offline = dict(rows=dict(contact=[dict(instance="one", B=dict(choice=8), Bx=dict(choice=2))]))
        result = report.g3([record], offline)
        self.assertEqual(result["compared"], 1)
        self.assertEqual(result["contact_choice_equal_offline"], 1.)

    def test_place_resolution_skip_is_not_first_choice_success(self):
        entry = selected_entry()
        entry["selection"]["place"][0]["attempt_ranks"] = [1]
        self.assertIs(report.first_selected_triple(entry), False)
        del entry["selection"]["place"][0]["attempt_ranks"]
        self.assertIsNone(report.first_selected_triple(entry))

    def test_equivalence_checks_nonwinning_values(self):
        from experiments.motion311.v2_latency_bench import value_delta
        self.assertEqual(value_delta([1., 2., 3.], [1., 2.5, 3.5]), .5)
        self.assertEqual(value_delta([1., float("inf")], [1., float("inf")]), 0.)
        self.assertEqual(value_delta([1., float("inf")], [1., 2.]), float("inf"))

    def test_offline_abstention_mismatch_is_counted(self):
        record = dict(instance="one", arms={"A": dict(class_="success"), "B": selected_entry()})
        offline = dict(rows=dict(contact=[dict(instance="one", B=dict(choice=None))]))
        result = report.g3([record], offline)
        self.assertEqual(result["compared"], 1)
        self.assertEqual(result["contact_choice_equal_offline"], 0.)


if __name__ == "__main__":
    unittest.main()
