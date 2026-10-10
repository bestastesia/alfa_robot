"""Offline sequential prediction: commit a round only after its full cycle passes."""
import time
import numpy as np

from .contracts import PlanRequest
from .fixtures import wall_sequence
from .scene import Pose, SceneSnapshot, SceneStore


def plan_sequence(planner, snapshot, progress, search_seed=None):
    from dataclasses import replace

    ids = tuple(int(o.object_id.removeprefix("wall_box_")) for o in snapshot.objects
                if o.object_id.startswith("wall_box_"))
    rounds = wall_sequence(replace(planner.wall_layout, active_box_ids=ids))
    report = {"success": False, "sequence": True, "search_seed": search_seed, "frames": [], "phases": [], "payload": [],
              "frame_rounds": [], "rounds": [{"task": task, "status": "pending"} for task in rounds],
              "initial_snapshot": snapshot.to_dict(), "completed_box_ids": [], "total_ms": 0.0}
    started = time.perf_counter()
    current = snapshot
    for number, record in enumerate(report["rounds"]):
        task = record["task"]
        record["removed_before"] = list(report["completed_box_ids"])
        request = PlanRequest(current, tuple(task.items()), tuple(
            (side + "_tool0", Pose(tuple(planner.default_target_poses[side + "_tool0"]["position"]),
                                   tuple(planner.default_target_poses[side + "_tool0"]["quaternion"])))
            for side in ("left", "right")), search_seed=search_seed)
        progress(f"整墙第{number+1}/{len(rounds)}轮 {task}")
        result = planner._plan_request(request, lambda text: progress(f"第{number+1}/{len(rounds)}轮 · {text}"))
        record.update(request=request.to_dict(), result=result,
                      status="completed" if result["success"] else "failed")
        if not result["success"]:
            report["error"] = result["error"]
            report["blocked_round"] = number + 1
            break
        if report["frames"] and not np.allclose(report["frames"][-1], result["frames"][0], atol=1e-7, rtol=0):
            raise RuntimeError("sequence boundary state mismatch")
        if report.get("joint_names", result["joint_names"]) != result["joint_names"]:
            raise RuntimeError("sequence joint order changed")
        report["joint_names"] = result["joint_names"]
        for key in ("frames", "phases", "payload"):
            report[key].extend(result[key])
        previous_frames = len(report["frames"]) - len(result["frames"])
        if ("time_from_start_s" in result and
                len(report.get("time_from_start_s", [])) == previous_frames):
            if any(len(result.get(key, [])) != len(result["frames"]) for key in
                   ("time_from_start_s", "velocities", "accelerations", "jerks")):
                raise RuntimeError("sequence round timing arrays do not match its frames")
            if "time_from_start_s" not in report:
                report.update(time_from_start_s=[], velocities=[], accelerations=[], jerks=[])
            offset = report["time_from_start_s"][-1] if report["time_from_start_s"] else 0.0
            report["time_from_start_s"].extend(offset + value for value in result["time_from_start_s"])
            for key in ("velocities", "accelerations", "jerks"):
                report[key].extend(result[key])
            report["motion_duration_s"] = report["time_from_start_s"][-1]
        else:
            # Keep per-round timing, but never publish a partial sequence clock.
            for key in ("time_from_start_s", "velocities", "accelerations", "jerks", "motion_duration_s"):
                report.pop(key, None)
        report["frame_rounds"].extend([number] * len(result["frames"]))
        released = next(e["snapshot"] for e in result["predicted_scene_events"] if e["phase"] == "release")
        store = SceneStore(SceneSnapshot.from_dict(released))
        values = dict(zip(result["joint_names"], result["frames"][-1]))
        current = store.update_state(store.snapshot().state.with_positions(values, Pose()), store.snapshot().revision)
        record["final_snapshot"] = current.to_dict()
        report["completed_box_ids"].extend(task.values())
    else:
        report["success"] = True
        report["error"] = None
    report["final_snapshot"] = current.to_dict()
    report["total_ms"] = (time.perf_counter() - started) * 1000
    report["grasp_search_rounds"] = [i+1 for i, r in enumerate(report["rounds"])
                                    if len(r.get("result", {}).get("suction_attempts", [])) > 1]
    report["fallback_rounds"] = [i+1 for i, r in enumerate(report["rounds"])
                                 if r.get("result", {}).get("home_search_stats", {}).get("fallback")]
    report["remaining_box_ids"] = sorted(set(ids) - set(report["completed_box_ids"]))
    planner.set_snapshot(current)
    return report
