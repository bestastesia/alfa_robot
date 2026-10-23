import json
import os
import subprocess
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from v3_rolling_acceptance_report import build_report, STAGES, SCENARIOS, FAULTS
from v3_rolling_matrix import invoke


class MatrixProcessSafetyTest(unittest.TestCase):
    def test_timeout_kills_the_whole_subprocess_group(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); pid_file = root / "pid"
            code = ("import os,pathlib,time,subprocess,sys; "
                    f"pathlib.Path({str(pid_file)!r}).write_text(str(os.getpid())); "
                    "subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); time.sleep(60)")
            self.assertEqual(invoke([sys.executable, "-c", code], root/"run.log", .1), -1)
            pid = int(pid_file.read_text())
            with self.assertRaises(ProcessLookupError):
                os.kill(pid, 0)
            self.assertIn("terminated timed-out process group", (root/"run.log").read_text())


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
            duplicate = json.loads(matrix.read_text()); duplicate["runs"].append(duplicate["runs"][0]);
            duplicate["passed"] += 1; matrix.write_text(json.dumps(duplicate))
            with self.assertRaisesRegex(ValueError, "duplicate"):
                build_report({"success":True,"rounds":15,"boxes":25},[matrix],
                             {"head":"541fdbc7250b50ffd5daadae6c8385c54f7b3a92","unchanged_files":{str(i):"h" for i in range(9)}},3)
            duplicate["runs"].pop(); duplicate["passed"] -= 1; matrix.write_text(json.dumps(duplicate))
            (root/"approach-clear-1.audit.log").unlink()
            with self.assertRaisesRegex(ValueError,"audit"):
                build_report({"success":True,"rounds":15,"boxes":25},[matrix],
                             {"head":"541fdbc7250b50ffd5daadae6c8385c54f7b3a92","unchanged_files":{str(i):"h" for i in range(9)}},3)


if __name__ == "__main__": unittest.main()
