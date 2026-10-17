import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from v3_rolling_acceptance_report import build_report, STAGES, SCENARIOS, FAULTS


class AcceptanceReportTest(unittest.TestCase):
    def test_complete_coverage_keeps_success_and_fail_closed_results(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); runs=[]
            common = dict(runtime_source_sha256={"a.py": "hash"}, initial_cycle_sha256="input",
                          startup={"model_assets_sha256": {"robot": "hash"}}, hardware_commands=False,
                          statistics={}, events=[], solves=[], trajectories=[], final_feedback={})
            for stage in STAGES:
                for scenario in SCENARIOS:
                    for seed in (1, 2, 3):
                        success = scenario == "clear"
                        name = f"{stage}-{scenario}-{seed}"
                        report = {**common, "stage": stage, "scenario": scenario, "seed": seed, "fault": "none",
                                  "success": success, "safety_action": None if success else "abort",
                                  "failure_code": None if success else "NO_VALID_REPLACEMENT",
                                  "constrained_revalidation_count": 1 if success and stage == "extract" else 0,
                                  "direct_splice_count": 1 if success else 0,
                                  "braking_replan_count": 0, "revalidated_splice_count": 0}
                        if success and stage in {"approach", "transport", "conveyor_place", "return_home"}:
                            report["solves"] = [{"previous_suffix_valid": False, "optimization_success": True}]
                        (root/f"{name}.json").write_text(json.dumps(report));(root/f"{name}.audit.log").write_text('{"verified":true}\n')
                        runs.append({"name": name, "verified": True})
            for fault in FAULTS:
                name=f"fault-{fault}";report={**common,"stage":"approach","scenario":"clear","seed":1,"fault":fault,
                    "success":False,"failure_code":"DEADLINE_MISS","safety_action":"abort"}
                (root/f"{name}.json").write_text(json.dumps(report));runs.append({"name":name,"verified":True})
            matrix=root/"matrix.json";matrix.write_text(json.dumps({"complete":True,"passed":len(runs),"runs":runs}))
            result=build_report({"success":True,"rounds":15,"boxes":25},[matrix],
                                {"head":"541fdbc7250b50ffd5daadae6c8385c54f7b3a92","unchanged_files":{str(i):"h" for i in range(9)}},3)
            self.assertTrue(result["simulation_contract_complete"])
            self.assertFalse(result["overall_issue_done"])
            self.assertGreater(result["normal_cases"]["rejected_fail_closed"],0)
            (root/"approach-clear-1.audit.log").unlink()
            with self.assertRaisesRegex(ValueError,"audit"):
                build_report({"success":True,"rounds":15,"boxes":25},[matrix],
                             {"head":"541fdbc7250b50ffd5daadae6c8385c54f7b3a92","unchanged_files":{str(i):"h" for i in range(9)}},3)


if __name__ == "__main__": unittest.main()
