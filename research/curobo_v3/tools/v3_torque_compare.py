#!/usr/bin/env python3
"""Audit and optionally retime a timed wall sequence against arm effort limits."""
import argparse
import copy
import json
from pathlib import Path
import tempfile
import time

import numpy as np
from curobo_core.torque import (TorqueModel, constrain_timed_result,
                                  write_dynamics_robot_config)


def rebuild_sequence(output):
    for key in ("frames", "phases", "payload", "frame_rounds", "time_from_start_s",
                "velocities", "accelerations", "jerks"):
        output[key] = []
    for number, record in enumerate(output["rounds"]):
        result = record["result"]
        offset = output["time_from_start_s"][-1] if output["time_from_start_s"] else 0.0
        for key in ("frames", "phases", "payload", "velocities", "accelerations", "jerks"):
            output[key].extend(result[key])
        output["time_from_start_s"].extend(offset + value for value in result["time_from_start_s"])
        output["frame_rounds"].extend([number] * len(result["frames"]))
    output["motion_duration_s"] = output["time_from_start_s"][-1]
    return output


def kinematic_summary(document):
    names = list(document["joint_names"])
    arrays = {field: np.asarray(document[field], dtype=float)
              for field in ("frames", "velocities", "accelerations", "jerks")}
    labels = {"frames": "position", "velocities": "velocity",
              "accelerations": "acceleration", "jerks": "jerk"}
    summary = {"motion_duration_s": float(document["motion_duration_s"]), "per_joint": {}}
    for index, name in enumerate(names):
        summary["per_joint"][name] = {
            "position_min": float(np.min(arrays["frames"][:, index])),
            "position_max": float(np.max(arrays["frames"][:, index])),
            **{f"peak_abs_{labels[field]}": float(np.max(np.abs(values[:, index])))
               for field, values in arrays.items()},
        }
    for field, values in arrays.items():
        frame, joint = np.unravel_index(np.argmax(np.abs(values)), values.shape)
        summary[f"peak_abs_{labels[field]}"] = {
            "value": float(abs(values[frame, joint])), "joint": names[joint], "frame": int(frame)}
    return summary


def torque_summary(rounds, joint_names, effort_limits, torque_limit_scale, constrained):
    records = []
    for round_record in rounds:
        for segment in round_record["segments"]:
            audit = (segment["constraint"].get("after", segment["before"])
                     if constrained else segment["before"])
            records.append((round_record["round"], segment["phase"], audit))
    per_joint_peak = np.max([item[2]["per_joint_peak_nm"] for item in records], axis=0)
    per_joint_gravity = np.max([item[2]["per_joint_gravity_peak_nm"] for item in records], axis=0)
    per_joint_dynamic = np.max([item[2]["per_joint_dynamic_peak_nm"] for item in records], axis=0)
    peak = max(records, key=lambda item: item[2]["maximum_utilization"])
    gravity_peak = max(records, key=lambda item: item[2]["maximum_gravity_utilization"])
    return {
        "maximum_utilization": peak[2]["maximum_utilization"],
        "maximum_configured_limit_utilization": peak[2]["maximum_utilization"] / torque_limit_scale,
        "peak_joint": peak[2]["peak_joint"], "peak_round": peak[0], "peak_phase": peak[1],
        "peak_torque_nm": peak[2]["peak_torque_nm"], "effort_limit_nm": peak[2]["effort_limit_nm"],
        "maximum_gravity_utilization": gravity_peak[2]["maximum_gravity_utilization"],
        "maximum_gravity_round": gravity_peak[0], "maximum_gravity_phase": gravity_peak[1],
        "per_joint": {name: {"effort_limit_nm": float(limit),
                             "configured_limit_nm": float(limit * torque_limit_scale),
                             "peak_torque_nm": float(per_joint_peak[index]),
                             "peak_gravity_nm": float(per_joint_gravity[index]),
                             "peak_dynamic_nm": float(per_joint_dynamic[index]),
                             "utilization": float(per_joint_peak[index] / limit),
                             "configured_limit_utilization": float(
                                 per_joint_peak[index] / (limit * torque_limit_scale)),
                             "gravity_utilization": float(per_joint_gravity[index] / limit)}
                      for index, (name, limit) in enumerate(zip(joint_names, effort_limits))},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--robot-config", type=Path, required=True)
    parser.add_argument("--dynamics-urdf", type=Path, required=True)
    parser.add_argument("--asset-root", type=Path, required=True)
    parser.add_argument("--payload-mass-kg", type=float, required=True)
    parser.add_argument("--payload-dimensions-m", type=float, nargs=3, default=(.3, .4, .4))
    parser.add_argument("--torque-limit-scale", type=float, default=.9)
    parser.add_argument("--joint-torque-profile", type=Path)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    source = json.loads(args.input.read_text())
    with tempfile.TemporaryDirectory() as directory:
        config = Path(directory) / "robot.yml"
        write_dynamics_robot_config(args.robot_config, args.dynamics_urdf, args.asset_root, config)
        model = TorqueModel(config, args.payload_mass_kg, args.payload_dimensions_m,
                            args.torque_limit_scale, args.joint_torque_profile)
        began = time.perf_counter()
        rounds, failed = [], False
        constrained = copy.deepcopy(source)
        for number, record in enumerate(source["rounds"], 1):
            result = record["result"]
            constrained_result = constrain_timed_result(
                constrained["rounds"][number-1]["result"], model, copy_result=False, require_contract=False)
            validation = constrained_result["torque_validation"]
            failed |= not validation["success"]
            constrained["rounds"][number-1]["result"] = constrained_result
            rounds.append({"round": number, "task": result["task"],
                           "segments": validation["segments"],
                           "before_motion_s": result["motion_duration_s"],
                           "after_motion_s": constrained_result["motion_duration_s"],
                           "after_timed_validation": constrained_result["timed_validation"]})
        constrained = rebuild_sequence(constrained)
        report = {
            "success": not failed,
            "source": str(args.input), "dynamics_urdf": str(args.dynamics_urdf),
            "payload_mass_kg": args.payload_mass_kg,
            "payload_dimensions_m": args.payload_dimensions_m,
            "torque_limit_scale": args.torque_limit_scale,
            "before_motion_s": source["motion_duration_s"],
            "after_motion_s": constrained["motion_duration_s"],
            "wall_ms": (time.perf_counter() - began) * 1000,
            "joint_names": model.names, "effort_limits": model.effort.tolist(),
            "enforced_joints": [name for name, enabled in zip(model.names, model.enforced) if enabled],
            "effort_semantics": ("manufacturer average allowable output torque"
                                 if model.torque_profile_source else
                                 "URDF joint-output effort; not motor-shaft torque"),
            "torque_profile_source": model.torque_profile_source,
            "joint_models": model.joint_models,
            "urdf_effort_limits": model.urdf_effort.tolist(),
            "rated_torque_limits": model.rated_effort.tolist(),
            "average_torque_limits": model.effort.tolist(),
            "start_stop_peak_limits": model.start_stop_effort.tolist(),
            "instantaneous_torque_limits": model.instantaneous_effort.tolist(),
            "utilization_denominator": ("manufacturer average allowable torque"
                                        if model.torque_profile_source else "raw URDF effort"),
            "configured_limit_denominator": "selected sustained limit multiplied by torque_limit_scale",
            "updown_effort_enforced": False,
            "retimed_segments": sum(segment["constraint"].get("time_scale", 1.0) > 1.000001
                                      for item in rounds for segment in item["segments"]),
            "failed_segments": sum(not segment["constraint"]["success"]
                                     for item in rounds for segment in item["segments"]),
            "kinematics_before": kinematic_summary(source),
            "kinematics_after": kinematic_summary(constrained),
            "rounds": rounds,
        }
        report["torque_before"] = torque_summary(
            rounds, model.names, model.effort, model.torque_limit_scale, False)
        report["torque_after"] = torque_summary(
            rounds, model.names, model.effort, model.torque_limit_scale, True)
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        if args.output:
            if report["success"]:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(constrained, ensure_ascii=False) + "\n")
            else:
                args.output.unlink(missing_ok=True)
        print(json.dumps({key: report[key] for key in
                          ("success", "payload_mass_kg", "before_motion_s", "after_motion_s", "wall_ms")},
                         ensure_ascii=False))
        raise SystemExit(0 if report["success"] else 2)


if __name__ == "__main__":
    main()
