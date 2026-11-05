"""Record selected contact/carry/place IK endpoint interpolation in Rerun.

This is a URDF/FK visualization, not a collision-validated RRT trajectory.
"""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
import rerun as rr
import rerun.blueprint as rrb


ACTIVE_JOINTS = ["updown", *[f"left_joint{i}" for i in range(1, 8)],
                 *[f"right_joint{i}" for i in range(1, 8)]]
STAGES = ("接触 IK", "回中/携箱 IK", "放置 IK")
COLORS = {
    "v2b": [70, 137, 213], "limit_guard": [75, 177, 126], "updown_stable": [224, 164, 69],
    "pool_robust": [186, 112, 181], "home_bias": [73, 176, 181],
    "return_priority": [217, 105, 87], "balanced_combo": [151, 160, 171],
}


def load_selected(data, task):
    functions = [row["id"] for row in data["functions"]]
    selected = {}
    for row in data["rows"]:
        if row["instance"] != task or row["stage"] >= 3 or not row.get("reached"):
            continue
        selected.setdefault(row["function_id"], {"label": row["function_label"], "stages": {}})
        selected[row["function_id"]]["stages"][row["stage"]] = np.asarray(
            [row[f"q{i}"] for i in range(15)], dtype=np.float64)
    if any(set(selected.get(function, {}).get("stages", {})) != {0, 1, 2} for function in functions):
        raise ValueError("task lacks three selected IK endpoints for at least one function")
    return functions, selected


def load_diagnostics(path, task):
    with path.open(encoding="utf-8-sig") as stream:
        rows = [row for row in csv.DictReader(stream) if row["instance"] == task]
    return {row["function_id"]: row for row in rows}


def interpolate(stages, hold, travel):
    frames = []
    for stage in range(3):
        frames.extend([(stages[stage], stage, 0.)] * hold)
        if stage < 2:
            for index in range(1, travel + 1):
                alpha = index / travel
                frames.append(((1. - alpha) * stages[stage] + alpha * stages[stage + 1], stage, alpha))
    return frames


def joints(row):
    return dict(zip(ACTIVE_JOINTS, np.asarray(row, dtype=np.float64)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--diagnostic-csv", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--task", default="L07-R01-dev-v0-S30x40x40")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hold", type=int, default=20)
    parser.add_argument("--travel", type=int, default=60)
    args = parser.parse_args()
    if args.hold < 1 or args.travel < 2:
        raise ValueError("hold must be positive and travel at least two")

    repo_root = Path(__file__).resolve().parents[5]
    rerun_package = repo_root / "ros2_ws/src/alfa_robot_rerun"
    if str(rerun_package) not in sys.path:
        sys.path.insert(0, str(rerun_package))
    from alfa_robot_rerun.visualize_rerun import UrdfRobot, log_robot_state, log_robot_static_model

    data = json.loads(args.data.read_text())
    functions, selected = load_selected(data, args.task)
    diagnostics = load_diagnostics(args.diagnostic_csv, args.task)
    if set(diagnostics) != set(functions):
        raise ValueError("diagnostics do not cover all functions")
    robot = UrdfRobot(args.urdf.read_text())
    args.output.parent.mkdir(parents=True, exist_ok=True)

    rr.init("motion386_selected_ik_endpoint_motion", spawn=False)
    rr.save(str(args.output))
    rr.send_blueprint(rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/world", contents=["/world/**"], name="三 IK 端点运动",
                              eye_controls=rrb.EyeControls3D(position=[-3.2, 3.2, 2.5],
                                                            look_target=[0., 0., .9], eye_up=[0., 0., 1.])),
            rrb.TextDocumentView(origin="/status", name="函数与阶段"),
            column_shares=[.78, .22]),
        rrb.TimePanel(timeline="endpoint_motion", expanded=True, fps=30, play_state="Playing"),
        collapse_panels=True))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/floor", rr.Boxes3D(centers=[[0., 0., -.025]], half_sizes=[[1.7, 1.7, .025]],
                                     colors=[[82, 91, 103, 50]]), static=True)
    log_robot_static_model(robot, "world/robot")

    timeline = 0
    boundaries = []
    for function in functions:
        entry = selected[function]
        endpoints = np.stack([entry["stages"][stage] for stage in range(3)])
        diagnostic = diagnostics[function]
        color = COLORS[function]
        # Endpoint markers and tool paths are refreshed once at each function boundary.
        transforms = [robot.fk(joints(row)) for row in endpoints]
        for side_index, side in enumerate(("left", "right")):
            points = np.stack([transform[f"{side}_tool0"][:3, 3] for transform in transforms])
            rr.set_time("endpoint_motion", sequence=timeline)
            rr.log(f"world/endpoint_paths/{side}", rr.LineStrips3D([points], colors=[color], radii=[.006]))
            rr.log(f"world/endpoint_markers/{side}", rr.Points3D(points, colors=[color] * 3, radii=[.025] * 3,
                                                                  labels=[f"{side} {stage}" for stage in STAGES]))
        sequence = interpolate(endpoints, args.hold, args.travel)
        start = timeline
        for local_index, (row, source_stage, alpha) in enumerate(sequence):
            rr.set_time("endpoint_motion", sequence=timeline)
            log_robot_state(robot, joints(row), "world/robot")
            current = STAGES[source_stage] if alpha == 0. or source_stage == 2 else f"{STAGES[source_stage]} → {STAGES[source_stage + 1]}"
            if local_index == 0 or local_index % 20 == 0:
                rr.log("status", rr.TextDocument(
                    f"## {entry['label']}\n\n"
                    f"**当前：** {current}\n\n"
                    f"**顺畅度：** {float(diagnostic['smoothness_score']):.2f} 分  "
                    f"（排名 {diagnostic['smoothness_rank']}，等效代价 {float(diagnostic['smoothness_cost_s']):.3f} s）\n\n"
                    f"**代表任务：** `{args.task}`\n\n"
                    "⚠️ 这是已选三个 IK 端点的线性插值和真实 URDF/FK 显示。"
                    "不是 RRT 轨迹，插值过程未作碰撞验收，也不是 B2 重选结果。",
                    media_type=rr.MediaType.MARKDOWN))
            timeline += 1
        boundaries.append({"function_id": function, "label": entry["label"], "start": start,
                           "end": timeline - 1, "frames": len(sequence)})
    rr.get_global_data_recording().flush()
    metadata = {
        "schema": "motion386-selected-ik-rerun-v1",
        "rrd": str(args.output.resolve()),
        "task": args.task,
        "timeline": "endpoint_motion",
        "frames": timeline,
        "boundaries": boundaries,
        "scope": "real URDF/FK with linear interpolation between selected contact/carry/placement IK endpoints",
        "not_claimed": ["RRT trajectory", "collision-validated interpolation", "B2 group selection"],
    }
    metadata_path = args.output.with_suffix(".json")
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(metadata, ensure_ascii=False))


if __name__ == "__main__":
    main()
