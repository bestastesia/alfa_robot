#!/usr/bin/env python3
"""Aggregate raw, independently-audited matrices into a simulation acceptance report."""
import argparse
import json
from pathlib import Path

from v3_rolling_matrix import summarize_reports

STAGES = {"approach", "contact_approach", "extract", "transport", "conveyor_place", "return_home"}
SCENARIOS = {"clear", "static", "moving", "appear", "depart"}
FAULTS = {"compute_delay", "link_delay", "stale_scene", "feedback_error", "scene_changed",
          "worker_failure", "solver_failure", "admission_delay"}
FREE_STAGES = {"approach", "transport", "conveyor_place", "return_home"}


def build_report(baseline, matrices, preservation, required_seeds=3):
    if not baseline.get("success") or baseline.get("rounds") != 15 or baseline.get("boxes") != 25:
        raise ValueError("baseline does not prove 15 rounds / 25 boxes")
    if preservation.get("head") != "541fdbc7250b50ffd5daadae6c8385c54f7b3a92" or len(
            preservation.get("unchanged_files", {})) < 9:
        raise ValueError("PR65 core preservation evidence is incomplete")
    reports, rows = [], []
    for matrix_path in matrices:
        matrix = json.loads(matrix_path.read_text())
        if not matrix.get("complete") or matrix.get("passed") != len(matrix.get("runs", [])):
            raise ValueError(f"matrix is incomplete or contains an unaudited case: {matrix_path}")
        for row in matrix["runs"]:
            if not row.get("verified"):
                raise ValueError(f"unverified case: {row.get('name')}")
            result_path = matrix_path.parent / f"{row['name']}.json"
            audit_path = result_path.with_suffix(".audit.log")
            report = json.loads(result_path.read_text())
            if report.get("fault") == "none":
                if not audit_path.is_file():
                    raise ValueError(f"missing independent audit: {result_path}")
                lines = [line for line in audit_path.read_text().splitlines() if line.startswith("{")]
                if not lines or not json.loads(lines[-1]).get("verified"):
                    raise ValueError(f"missing independent audit: {result_path}")
            reports.append(report)
            rows.append({"name": row["name"], "matrix": str(matrix_path), "success": report["success"],
                         "failure_code": report.get("failure_code"), "seed": report["seed"],
                         "stage": report["stage"], "scenario": report["scenario"], "fault": report["fault"]})
    normal = [report for report in reports if report.get("fault") == "none"]
    coverage = {(stage, scenario): set() for stage in STAGES for scenario in SCENARIOS}
    for report in normal:
        coverage.setdefault((report["stage"], report["scenario"]), set()).add(report["seed"])
        if not report["success"] and (not report.get("failure_code") or not report.get("safety_action")):
            raise ValueError("normal rejection lacks fail-closed code/action")
        if report["success"] and ("final_feedback" not in report or report.get("hardware_commands")):
            raise ValueError("successful simulation lacks final feedback or sent hardware commands")
    missing = {f"{stage}/{scenario}": sorted(seeds) for (stage, scenario), seeds in coverage.items()
               if len(seeds) < required_seeds}
    if missing:
        raise ValueError(f"stage/scenario seed coverage incomplete: {missing}")
    fault_coverage = {report["fault"] for report in reports if report.get("fault") != "none" and not report["success"]}
    if fault_coverage != FAULTS:
        raise ValueError(f"fault coverage differs: {sorted(fault_coverage)}")
    free_responses = {stage: sum(
        solve.get("previous_suffix_valid") is False and solve.get("optimization_success")
        for report in normal if report["stage"] == stage and report["success"]
        for solve in report.get("solves", [])) for stage in FREE_STAGES}
    if any(count < 1 for count in free_responses.values()):
        raise ValueError(f"no demonstrated affected-suffix response for free stage: {free_responses}")
    constrained_checks = sum(report.get("constrained_revalidation_count", 0) for report in normal if report["success"])
    if constrained_checks < 1:
        raise ValueError("constrained stages have no rolling revalidation evidence")
    hashes = {json.dumps(report.get("runtime_source_sha256"), sort_keys=True) for report in reports}
    models = {json.dumps(report.get("startup", {}).get("model_assets_sha256"), sort_keys=True) for report in reports}
    inputs = {report.get("initial_cycle_sha256") for report in reports}
    if len(hashes) != 1 or len(models) != 1 or len(inputs) != 1:
        raise ValueError("matrices do not share one frozen source/model/input")
    success = sum(report["success"] for report in normal)
    rejection_codes = {}
    for report in normal:
        if not report["success"]:
            rejection_codes[report["failure_code"]] = rejection_codes.get(report["failure_code"], 0) + 1
    strategy = {key: sum(report.get(key, 0) for report in normal) for key in
                ("direct_splice_count", "braking_replan_count", "revalidated_splice_count")}
    return {
        "simulation_contract_complete": True,
        "overall_issue_done": False,
        "baseline": baseline,
        "pr65_preservation": preservation,
        "normal_cases": {"count": len(normal), "completed": success, "rejected_fail_closed": len(normal)-success,
                         "completion_rate": success/len(normal), "rejection_codes": rejection_codes},
        "actual_splice_strategies": strategy,
        "free_stage_affected_suffix_responses": free_responses,
        "constrained_revalidation_count": constrained_checks,
        "fault_cases": {"count": len(FAULTS), "covered": sorted(fault_coverage)},
        "latency_by_stage_scenario": summarize_reports(normal),
        "cases": rows,
        "external_gates": ["MOTION-296 acceptance", "15-axis conditional boundary replacement/stop interface",
                           "real obstacle and execution feedback sources", "clock synchronization and safety review"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-summary", type=Path, required=True)
    parser.add_argument("--preservation", type=Path, required=True)
    parser.add_argument("--matrix", type=Path, action="append", required=True)
    parser.add_argument("--required-seeds", type=int, default=3)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = build_report(json.loads(args.baseline_summary.read_text()), args.matrix,
                          json.loads(args.preservation.read_text()), args.required_seeds)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"simulation_contract_complete": True, **result["normal_cases"],
                      "actual_splice_strategies": result["actual_splice_strategies"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
