"""Repeat one explicit 13-DoF segment service request and write stable A/B metrics."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from robot_motion_interfaces.msg import AttachedBox, MotionContext
from robot_motion_interfaces.srv import PlanJointSegment
from sensor_msgs.msg import JointState

from .planner_core import AUTHORITY_JOINT_NAMES


def parse_positions(value: str) -> tuple[float, ...]:
    result = tuple(float(item) for item in value.split(","))
    if len(result) != len(AUTHORITY_JOINT_NAMES) or not all(math.isfinite(x) for x in result):
        raise argparse.ArgumentTypeError("expected 13 finite comma-separated positions")
    return result


def payload(side: str, box_id: int, grasp_mode: str = "front") -> AttachedBox:
    message = AttachedBox()
    message.id = f"carried_{side}_box_{box_id}"
    message.box_id = box_id
    message.side = side
    message.grasp_mode = grasp_mode
    message.link_name = f"{side}_tool0"
    message.center_in_link.orientation.w = 1.0
    if grasp_mode == "top_suction":
        message.center_in_link.position.z = 0.2
        message.size.x = 0.3
        message.size.y = 0.4
        message.size.z = 0.4
    elif grasp_mode == "front":
        message.center_in_link.position.z = 0.15
        message.size.x = 0.4
        message.size.y = 0.4
        message.size.z = 0.3
    else:
        raise ValueError(f"unsupported grasp_mode:{grasp_mode}")
    message.touch_links = [message.link_name, f"{side}joint6"]
    return message


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))]


def trajectory_metrics(trajectory) -> tuple[int, float, float]:
    motion = 0.0
    for previous, current in zip(trajectory.points, trajectory.points[1:]):
        motion += sum(abs(b - a) for a, b in zip(previous.positions, current.positions))
    duration = 0.0
    if trajectory.points:
        stamp = trajectory.points[-1].time_from_start
        duration = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return len(trajectory.points), motion, duration


def duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def robot_state_json(positions: tuple[float, ...]) -> dict:
    names = list(AUTHORITY_JOINT_NAMES)
    values = [float(value) for value in positions]
    return {
        "joint_names": names,
        "joint_values": values,
        "joint_map": dict(zip(names, values)),
    }


def attached_boxes_json(
    left_box_id: int,
    right_box_id: int,
    left_grasp_mode: str,
    right_grasp_mode: str,
) -> list[dict]:
    result = []
    for side, box_id, grasp_mode in (
        ("left", left_box_id, left_grasp_mode),
        ("right", right_box_id, right_grasp_mode),
    ):
        top_suction = grasp_mode == "top_suction"
        result.append(
            {
                "id": f"carried_{side}_box_{box_id}",
                "link_name": f"{side}_tool0",
                "grasp_mode": grasp_mode,
                "center_in_link": [0.0, 0.0, 0.2 if top_suction else 0.15],
                "size": [0.3, 0.4, 0.4] if top_suction else [0.4, 0.4, 0.3],
            }
        )
    return result


def replay_scene_json(scene_config: Path | None) -> tuple[dict, dict]:
    if scene_config is None:
        return {}, {"enabled": False, "boxes": []}
    document = yaml.safe_load(scene_config.read_text()) or {}
    cuboids = document.get("cuboid", {})
    container_panels = []
    static_boxes = []
    for obstacle_id, obstacle in cuboids.items():
        size = [float(value) for value in obstacle.get("dims", [])]
        pose = [float(value) for value in obstacle.get("pose", [])]
        if len(size) != 3 or len(pose) < 3:
            continue
        item = {"id": str(obstacle_id), "center": pose[:3], "size": size}
        if str(obstacle_id).startswith("container_"):
            container_panels.append(item)
        else:
            static_boxes.append(item)
    return (
        {"enabled": bool(container_panels), "panels": container_panels},
        {"enabled": bool(static_boxes), "boxes": static_boxes},
    )


def trajectory_stage_json(parsed, index: int, response) -> dict:
    points = []
    for point in response.trajectory.points:
        points.append(
            {
                "time_from_start_sec": duration_seconds(point.time_from_start),
                "positions": [float(value) for value in point.positions],
                "velocities": [float(value) for value in point.velocities],
            }
        )
    return {
        "type": "stage",
        "stage": f"{parsed.label}/{response.planner_method}",
        "stage_index": index,
        "trajectory": {
            "joint_names": list(response.trajectory.joint_names),
            "point_count": len(points),
            "points": points,
        },
        "target_names": list(AUTHORITY_JOINT_NAMES),
        "start_state": robot_state_json(parsed.start),
        "goal_state": robot_state_json(parsed.goal),
        "attached_boxes": attached_boxes_json(
            parsed.left_box_id,
            parsed.right_box_id,
            parsed.left_grasp_mode,
            parsed.right_grasp_mode,
        ),
        "static_box_obstacles": parsed.replay_static_obstacles,
        "extra": {
            "stage_kind": "local_curobo_segment_repair",
            "valid": bool(response.success),
            "planning_method": response.planner_method,
            "scene_id": parsed.scene_id,
            "left_box_id": parsed.left_box_id,
            "right_box_id": parsed.right_box_id,
            "solve_ms": float(response.solve_time_ms),
            "queue_ms": float(response.queue_time_ms),
            "endpoint_check_ms": float(response.endpoint_check_time_ms),
            "graph_ms": float(response.graph_time_ms),
            "trajopt_ms": float(response.trajopt_time_ms),
            "interpolation_ms": float(response.interpolation_time_ms),
            "total_ms": float(response.total_time_ms),
            "message": response.message,
        },
    }


def write_replay_jsonl(path: Path, parsed, stages: list[dict], summary: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = {
        "type": "header",
        "schema": "moveit_box_stack_flow_v1",
        "planning_group": "dual_arm_with_base",
        "box_front_x": parsed.box_front_x,
        "scene_y_shift": parsed.scene_y_shift,
        "fixed_updown": parsed.start[0],
        "velocity_scale": 0.25,
        "acceleration_scale": 1.0,
        "execute": False,
        "show_box_stack": False,
        "container_obstacle": parsed.replay_container,
        "static_box_obstacles": parsed.replay_static_obstacles,
    }
    records = [header, *stages, {"type": "summary", **summary}]
    path.write_text("".join(json.dumps(record, ensure_ascii=False) + "\n" for record in records))


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--service", required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--scene-id", default="")
    parser.add_argument("--start", type=parse_positions, required=True)
    parser.add_argument("--goal", type=parse_positions, required=True)
    parser.add_argument("--repeat", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--max-attempts", type=int, default=3)
    parser.add_argument("--force-graph", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--left-box-id", type=int, default=1)
    parser.add_argument("--right-box-id", type=int, default=3)
    parser.add_argument(
        "--left-grasp-mode", choices=("front", "top_suction"), default="front"
    )
    parser.add_argument(
        "--right-grasp-mode", choices=("front", "top_suction"), default="front"
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--record-jsonl",
        type=Path,
        default=None,
        help="Write successful returned trajectories for the existing Rerun flow viewer",
    )
    parser.add_argument(
        "--scene-config",
        type=Path,
        default=None,
        help="cuRobo scene YAML used to draw the matching collision cuboids in Rerun",
    )
    parser.add_argument("--box-front-x", type=float, default=0.925)
    parser.add_argument("--scene-y-shift", type=float, default=-0.4)
    parsed = parser.parse_args(args)
    if parsed.repeat < 1:
        raise ValueError("--repeat must be positive")
    parsed.replay_container, parsed.replay_static_obstacles = replay_scene_json(
        parsed.scene_config
    )

    rclpy.init()
    node = rclpy.create_node(f"segment_benchmark_{parsed.label}")
    client = node.create_client(PlanJointSegment, parsed.service)
    if not client.wait_for_service(timeout_sec=30.0):
        raise RuntimeError(f"service unavailable: {parsed.service}")
    rows = []
    replay_stages = []
    try:
        for index in range(parsed.repeat):
            request = PlanJointSegment.Request()
            request.context = MotionContext(
                request_id=f"{parsed.label}_{index}",
                frame_id="world",
                scene_id=parsed.scene_id,
                state_id="explicit_segment_benchmark",
            )
            request.start_state = JointState(
                name=list(AUTHORITY_JOINT_NAMES), position=list(parsed.start)
            )
            request.goal_state = JointState(
                name=list(AUTHORITY_JOINT_NAMES), position=list(parsed.goal)
            )
            request.attached_boxes = [
                payload("left", parsed.left_box_id, parsed.left_grasp_mode),
                payload("right", parsed.right_box_id, parsed.right_grasp_mode),
            ]
            request.force_graph = parsed.force_graph
            request.max_attempts = parsed.max_attempts
            request.timeout_s = parsed.timeout
            started = time.perf_counter()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=parsed.timeout + 1.0)
            wall_ms = (time.perf_counter() - started) * 1000.0
            response = future.result() if future.done() else None
            points, motion, duration = trajectory_metrics(response.trajectory) if response else (0, 0.0, 0.0)
            row = {
                "label": parsed.label,
                "index": index,
                "success": bool(response and response.success),
                "planner_method": response.planner_method if response else "client_timeout",
                "wall_ms": wall_ms,
                "service_total_ms": response.total_time_ms if response else 0.0,
                "solve_ms": response.solve_time_ms if response else 0.0,
                "queue_ms": response.queue_time_ms if response else 0.0,
                "endpoint_check_ms": response.endpoint_check_time_ms if response else 0.0,
                "graph_ms": response.graph_time_ms if response else 0.0,
                "trajopt_ms": response.trajopt_time_ms if response else 0.0,
                "interpolation_ms": response.interpolation_time_ms if response else 0.0,
                "trajectory_points": points,
                "trajectory_joint_motion": motion,
                "trajectory_execution_s": duration,
                "message": response.message if response else "client_timeout",
            }
            rows.append(row)
            if response and response.success and response.trajectory.points:
                replay_stages.append(
                    trajectory_stage_json(parsed, len(replay_stages), response)
                )
            print(json.dumps(row, ensure_ascii=False), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    parsed.output_dir.mkdir(parents=True, exist_ok=True)
    sample_path = parsed.output_dir / f"{parsed.label}_samples.csv"
    with sample_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    successful = [row for row in rows if row["success"]]
    summary = {
        "label": parsed.label,
        "samples": len(rows),
        "success_rate": len(successful) / len(rows),
    }
    for field in (
        "wall_ms",
        "service_total_ms",
        "solve_ms",
        "queue_ms",
        "endpoint_check_ms",
        "graph_ms",
        "trajopt_ms",
        "interpolation_ms",
    ):
        values = [float(row[field]) for row in successful]
        summary[f"{field}_p50"] = percentile(values, 0.50)
        summary[f"{field}_p95"] = percentile(values, 0.95)
        summary[f"{field}_p99"] = percentile(values, 0.99)
    (parsed.output_dir / f"{parsed.label}_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2)
    )
    if parsed.record_jsonl is not None:
        if not replay_stages:
            raise RuntimeError("no successful trajectory available for --record-jsonl")
        write_replay_jsonl(parsed.record_jsonl, parsed, replay_stages, summary)
        print(json.dumps({"replay_jsonl": str(parsed.record_jsonl)}, ensure_ascii=False))
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
