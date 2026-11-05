"""Record synchronized v2b and B2 executed trajectories side by side in Rerun."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import rerun as rr
import rerun.blueprint as rrb


def load_trajectory(path):
    document = np.load(path)
    return {key: np.asarray(document[key]) for key in document.files}


def b1_metrics(path):
    row = json.loads(path.read_text())
    return {"selection_ms": row["selection_total_ms"], "planning_ms": row["planning_wall_ms"],
            "executed_t_lb_s": row["executed_t_path_s"], "duration_s": row["motion_duration_s"],
            "acceleration_ratio": row["report"]["timed_validation"]["acceleration_limit_ratio"],
            "jerk_ratio": row["report"]["timed_validation"]["jerk_limit_ratio"]}


def b2_metrics(path):
    row = json.loads(path.read_text())
    return {"selection_ms": row["selection_ms"], "planning_ms": sum(item["wall_ms"] for item in row["outcomes"]),
            "executed_t_lb_s": row["outcomes"][-1]["info"]["executed_t_path_s"],
            "duration_s": row["motion_duration_s"],
            "acceleration_ratio": row["timed_validation"]["acceleration_limit_ratio"],
            "jerk_ratio": row["timed_validation"]["jerk_limit_ratio"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--v2b-trajectory", type=Path, required=True)
    parser.add_argument("--v2b-result", type=Path, required=True)
    parser.add_argument("--b2-trajectory", type=Path, required=True)
    parser.add_argument("--b2-result", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--task", required=True)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[5]
    sys.path.insert(0, str(repo_root / "ros2_ws/src/alfa_robot_rerun"))
    from alfa_robot_rerun.visualize_rerun import UrdfRobot, log_robot_state, log_robot_static_model

    v2b, b2 = load_trajectory(args.v2b_trajectory), load_trajectory(args.b2_trajectory)
    for name, trajectory in (("v2b", v2b), ("B2", b2)):
        required = {"frames", "phases", "time_from_start_s", "joint_names"}
        if not required <= set(trajectory):
            raise ValueError(f"{name} trajectory lacks {sorted(required - set(trajectory))}")
        if len(trajectory["frames"]) != len(trajectory["phases"]):
            raise ValueError(f"{name} trajectory shapes differ")
    if v2b["joint_names"].astype(str).tolist() != b2["joint_names"].astype(str).tolist():
        raise ValueError("paired trajectories use different joint order")
    names = v2b["joint_names"].astype(str).tolist()
    metrics_a, metrics_b = b1_metrics(args.v2b_result), b2_metrics(args.b2_result)
    robot = UrdfRobot(args.urdf.read_text())
    args.output.parent.mkdir(parents=True, exist_ok=True)

    rr.init("motion386_v2b_vs_b2_executed", spawn=False)
    rr.save(str(args.output))
    rr.send_blueprint(rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/world", contents=["/world/**"], name="v2b vs B2 实际轨迹",
                              eye_controls=rrb.EyeControls3D(position=[-4.8, 4.8, 3.0],
                                                            look_target=[0., 0., .9], eye_up=[0., 0., 1.])),
            rrb.TextDocumentView(origin="/status", name="配对指标"), column_shares=[.76, .24]),
        rrb.TimePanel(timeline="paired_frame", expanded=True, fps=40, play_state="Playing"),
        collapse_panels=True))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/floor", rr.Boxes3D(centers=[[0., 0., -.025]], half_sizes=[[1.8, 3.2, .025]],
                                     colors=[[82, 91, 103, 50]]), static=True)
    rr.log("world/v2b", rr.Transform3D(translation=[0., 1.8, 0.]), static=True)
    rr.log("world/b2", rr.Transform3D(translation=[0., -1.8, 0.]), static=True)
    log_robot_static_model(robot, "world/v2b/robot", log_view_coordinates=False)
    log_robot_static_model(robot, "world/b2/robot", log_view_coordinates=False)
    rr.log("world/v2b/label", rr.Points3D([[0., 0., 2.15]], colors=[[70, 137, 213]], radii=[.05],
                                           labels=["B1 v2b"]), static=True)
    rr.log("world/b2/label", rr.Points3D([[0., 0., 2.15]], colors=[[73, 176, 181]], radii=[.05],
                                          labels=["B2 group smooth"]), static=True)

    count = max(len(v2b["frames"]), len(b2["frames"]))
    prior = None
    for index in range(count):
        ai, bi = min(index, len(v2b["frames"]) - 1), min(index, len(b2["frames"]) - 1)
        phase_a, phase_b = str(v2b["phases"][ai]), str(b2["phases"][bi])
        rr.set_time("paired_frame", sequence=index)
        log_robot_state(robot, dict(zip(names, v2b["frames"][ai])), "world/v2b/robot")
        log_robot_state(robot, dict(zip(names, b2["frames"][bi])), "world/b2/robot")
        state = (phase_a, phase_b)
        if state != prior or index % 80 == 0:
            rr.log("status", rr.TextDocument(
                f"## 同源配对：{args.task}\n\n"
                f"| 指标 | B1 v2b | B2 |\n|---|---:|---:|\n"
                f"| 当前阶段 | `{phase_a}` | `{phase_b}` |\n"
                f"| 选择耗时 | {metrics_a['selection_ms']:.1f} ms | {metrics_b['selection_ms']:.1f} ms |\n"
                f"| 规划耗时 | {metrics_a['planning_ms']:.1f} ms | {metrics_b['planning_ms']:.1f} ms |\n"
                f"| 执行 T_lb | {metrics_a['executed_t_lb_s']:.2f} s | {metrics_b['executed_t_lb_s']:.2f} s |\n"
                f"| 运动时长 | {metrics_a['duration_s']:.2f} s | {metrics_b['duration_s']:.2f} s |\n"
                f"| 加速度比 | {metrics_a['acceleration_ratio']:.4f} | {metrics_b['acceleration_ratio']:.4f} |\n"
                f"| jerk 比 | {metrics_a['jerk_ratio']:.5f} | {metrics_b['jerk_ratio']:.5f} |\n\n"
                "两条轨迹均由同一阶段规划器生成并通过碰撞与定时校核。",
                media_type=rr.MediaType.MARKDOWN))
        prior = state
    rr.get_global_data_recording().flush()
    print(json.dumps({"rrd": str(args.output.resolve()), "frames": count, "task": args.task,
                      "v2b": metrics_a, "b2": metrics_b}, ensure_ascii=False))


if __name__ == "__main__":
    main()
