#!/usr/bin/env python3
"""Fail closed unless a fixed-path timed + RNEA full-wall result is controller-export ready."""
import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--controller", type=Path, required=True)
    args = parser.parse_args()
    result = json.loads(args.result.read_text())
    controller = json.loads(args.controller.read_text())
    if not (result.get("success") and result.get("dynamics_enabled") and
            result.get("torque_constraints_enabled")):
        raise SystemExit("full-wall pipeline flags are incomplete")
    attempts = accepted = fallbacks = 0
    for record in result["rounds"]:
        cycle = record["result"]
        if not cycle.get("success") or not cycle["timed_validation"]["success"]:
            raise SystemExit(f"failed timed round: {record['task']}")
        contract = cycle.get("baseline_trajectory_contract", {})
        if not contract.get("collision_validation_success"):
            raise SystemExit(f"missing baseline certificate: {record['task']}")
        latency = cycle.get("latency_validation", {})
        if not latency.get("success") or latency.get("end_to_end_ms", float("inf")) > 5000:
            raise SystemExit(f"round exceeded latency budget: {record['task']} {latency}")
        torque = cycle.get("torque_validation", {})
        if not torque.get("success") or not torque.get("geometry_frozen") or torque.get("maximum_position_delta") != 0:
            raise SystemExit(f"failed/flexible dynamics round: {record['task']}")
        if not cycle.get("controller_ready"):
            raise SystemExit(f"round controller preview failed: {record['task']}")
        for segment in cycle["timed_segments"]:
            attempts += bool(segment.get("trajopt_attempted"))
            accepted += bool(segment.get("trajopt_accepted"))
            fallbacks += bool(segment.get("trajopt_attempted") and not segment.get("trajopt_accepted"))
            if len(segment.get("torques_nm", ())) != len(segment["positions"]):
                raise SystemExit(f"missing torque samples: {record['task']} {segment['name']}")
    if result.get("trajectory_optimizer_enabled"):
        if attempts != 60 or accepted + fallbacks != attempts:
            raise SystemExit(f"unexpected experimental TrajOpt coverage: {attempts}/{accepted}/{fallbacks}")
    elif attempts:
        raise SystemExit(f"production result unexpectedly attempted TrajOpt: {attempts}")
    if not controller.get("controller_ready") or len(controller.get("segments", ())) != 90:
        raise SystemExit("controller export is incomplete")
    for segment in controller["segments"]:
        times = np.asarray(segment["time_from_start_s"])
        count = len(times)
        if count < 2 or np.any(np.diff(times) <= 0):
            raise SystemExit(f"non-increasing controller clock: {segment['id']}")
        for field in ("positions", "velocities", "accelerations", "jerks",
                      "rnea_required_torque_nm", "gravity_torque_nm"):
            values = np.asarray(segment[field])
            if len(values) != count or not np.isfinite(values).all():
                raise SystemExit(f"invalid controller {field}: {segment['id']}")
    print(json.dumps({"success": True, "rounds": len(result["rounds"]),
                      "boxes": len(result["completed_box_ids"]),
                      "trajopt_attempted": attempts, "trajopt_accepted": accepted,
                      "trajopt_fallback": fallbacks,
                      "controller_segments": len(controller["segments"]),
                      "motion_duration_s": result["motion_duration_s"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
