"""Repeatable real-GPU parameter benchmark for explicit 13-DoF cspace tasks."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path

import yaml

from .curobo_backend import CuroboBackend
from .planner_core import AttachedPayload, validate_backend_trajectory


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    ratio = position - lower
    return ordered[lower] * (1.0 - ratio) + ordered[upper] * ratio


def _summary(values: list[float]) -> dict[str, float | int | None]:
    return {
        "count": len(values),
        "mean": statistics.fmean(values) if values else None,
        "p50": _percentile(values, 0.50),
        "p95": _percentile(values, 0.95),
        "p99": _percentile(values, 0.99),
        "min": min(values) if values else None,
        "max": max(values) if values else None,
    }


def _payload(side: str, box_id: int, grasp_mode: str = "front") -> AttachedPayload:
    if grasp_mode == "top_suction":
        center_xyz = (0.0, 0.0, 0.2)
        size_xyz = (0.3, 0.4, 0.4)
    elif grasp_mode == "front":
        center_xyz = (0.0, 0.0, 0.15)
        size_xyz = (0.4, 0.4, 0.3)
    else:
        raise ValueError(f"unsupported grasp_mode:{grasp_mode}")
    return AttachedPayload(
        object_id=f"carried_{side}_box_{box_id}",
        box_id=box_id,
        side=side,
        link_name=f"{side}_tool0",
        center_xyz=center_xyz,
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        size_xyz=size_xyz,
    )


def _joint_motion(rows: tuple[tuple[float, ...], ...]) -> float:
    return sum(
        abs(b - a)
        for first, second in zip(rows, rows[1:])
        for a, b in zip(first, second)
    )


def _max_joint_step(rows: tuple[tuple[float, ...], ...]) -> float:
    return max(
        (abs(b - a) for first, second in zip(rows, rows[1:]) for a, b in zip(first, second)),
        default=0.0,
    )


def _smoothness_cost(rows: tuple[tuple[float, ...], ...]) -> float:
    return sum(
        (after - 2.0 * current + before) ** 2
        for previous, current_row, next_row in zip(rows, rows[1:], rows[2:])
        for before, current, after in zip(previous, current_row, next_row)
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tasks", required=True)
    parser.add_argument("--robot-config", default="")
    parser.add_argument("--curobo-python-root", required=True)
    parser.add_argument("--dependency-venv", default="")
    parser.add_argument("--select", default="all")
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--force-graph", action="store_true")
    parser.add_argument("--disable-cuda-graph", action="store_true")
    parser.add_argument("--disable-self-collision-check", action="store_true")
    parser.add_argument("--max-attempts", type=int, default=3)
    parser.add_argument("--num-trajopt-seeds", type=int, default=1)
    parser.add_argument("--trajopt-num-iters", type=int, default=100)
    parser.add_argument("--trajopt-inner-iters", type=int, default=25)
    parser.add_argument("--trajopt-history", type=int, default=27)
    parser.add_argument("--trajopt-n-knots", type=int, default=16)
    parser.add_argument("--trajopt-interpolation-steps", type=int, default=4)
    parser.add_argument("--trajopt-finetune-attempts", type=int, default=3)
    parser.add_argument("--trajopt-finetune-dt-scale", type=float, default=0.75)
    parser.add_argument("--collision-activation-distance", type=float, default=0.01)
    parser.add_argument("--warmup-iterations", type=int, default=5)
    args = parser.parse_args()

    tasks_path = Path(args.tasks).expanduser().resolve()
    document = yaml.safe_load(tasks_path.read_text())
    robot_config_path = args.robot_config or str(document.get("robot_config_path", ""))
    if not robot_config_path:
        raise ValueError("robot config is required via --robot-config or robot_config_path in tasks YAML")
    task_names = list(document["tasks"])
    if args.select != "all":
        requested = [value.strip() for value in args.select.split(",") if value.strip()]
        missing = [value for value in requested if value not in document["tasks"]]
        if missing:
            raise ValueError(f"unknown_tasks:{missing}")
        task_names = requested

    records: list[dict] = []
    initialization: dict[str, float] = {}
    endpoint_feasibility: dict[str, dict[str, list[bool]]] = {}
    payload_sphere_world_bounds: dict[str, dict[str, object]] = {}
    for task_name in task_names:
        task = document["tasks"][task_name]
        scene_path = Path(task["scene_config_path"])
        if not scene_path.is_absolute():
            scene_path = tasks_path.parent / scene_path
        initialized = time.perf_counter()
        backend = CuroboBackend(
            robot_config_path=robot_config_path,
            scene_config_path=str(scene_path),
            curobo_python_root=args.curobo_python_root,
            dependency_venv=args.dependency_venv,
            warmup_iterations=args.warmup_iterations,
            use_cuda_graph=not args.disable_cuda_graph,
            self_collision_check=not args.disable_self_collision_check,
            num_trajopt_seeds=args.num_trajopt_seeds,
            trajopt_num_iters=args.trajopt_num_iters,
            trajopt_inner_iters=args.trajopt_inner_iters,
            trajopt_history=args.trajopt_history,
            trajopt_n_knots=args.trajopt_n_knots,
            trajopt_interpolation_steps=args.trajopt_interpolation_steps,
            trajopt_finetune_attempts=args.trajopt_finetune_attempts,
            trajopt_finetune_dt_scale=args.trajopt_finetune_dt_scale,
            optimizer_collision_activation_distance=args.collision_activation_distance,
            left_payload_link="left_tool0",
            right_payload_link="right_tool0",
        )
        initialization[task_name] = (time.perf_counter() - initialized) * 1000.0
        start = tuple(float(value) for value in task["start"])
        goal = tuple(float(value) for value in task["goal"])
        payloads = (
            _payload(
                "left",
                int(task["left_box_id"]),
                str(task.get("left_grasp_mode", "front")),
            ),
            _payload(
                "right",
                int(task["right_box_id"]),
                str(task.get("right_grasp_mode", "front")),
            ),
        )
        endpoint_feasibility[task_name] = {
            label: list(backend.check_positions_feasible((start, goal), selected))
            for label, selected in (
                ("robot_only", ()),
                ("left_payload_only", payloads[:1]),
                ("right_payload_only", payloads[1:]),
                ("both_payloads", payloads),
            )
        }
        payload_sphere_world_bounds[task_name] = {
            "start": backend.payload_sphere_world_bounds(start, payloads),
            "goal": backend.payload_sphere_world_bounds(goal, payloads),
        }
        try:
            for repeat_index in range(args.repeat):
                wall_started = time.perf_counter()
                result = backend.plan_cspace(
                    start,
                    goal,
                    payloads,
                    scene_id=str(task["scene_id"]),
                    force_graph=args.force_graph,
                    max_attempts=args.max_attempts,
                    timeout_s=120.0,
                )
                wall_ms = (time.perf_counter() - wall_started) * 1000.0
                trajectory = None
                feasible = False
                joint_motion = None
                duration_s = None
                max_joint_step = None
                smoothness_cost = None
                if result.success and result.trajectory is not None:
                    trajectory = validate_backend_trajectory(
                        result.trajectory, start, goal, backend.joint_limits
                    )
                    feasible = all(backend.check_positions_feasible(
                        trajectory.positions, payloads
                    ))
                    joint_motion = _joint_motion(trajectory.positions)
                    duration_s = trajectory.times_s[-1]
                    max_joint_step = _max_joint_step(trajectory.positions)
                    smoothness_cost = _smoothness_cost(trajectory.positions)
                reference_motion = float(task["reference_joint_motion"])
                records.append(
                    {
                        "task": task_name,
                        "repeat": repeat_index,
                        "success": result.success,
                        "planner_method": result.planner_method,
                        "message": result.message,
                        "wall_ms": wall_ms,
                        "solve_ms": result.solve_time_ms,
                        "ik_ms": 0.0,
                        "endpoint_check_ms": result.endpoint_check_time_ms,
                        "graph_ms": result.graph_time_ms,
                        "trajopt_ms": result.trajopt_time_ms,
                        "interpolation_ms": result.interpolation_time_ms,
                        "trajectory_feasible": feasible,
                        "trajectory_points": 0 if trajectory is None else len(trajectory.positions),
                        "trajectory_joint_motion": joint_motion,
                        "reference_joint_motion": reference_motion,
                        "joint_motion_ratio": None if joint_motion is None else joint_motion / reference_motion,
                        "trajectory_duration_s": duration_s,
                        "max_joint_step": max_joint_step,
                        "smoothness_cost": smoothness_cost,
                        "reference_duration_s": float(task["reference_duration_s"]),
                        "quality_gate": bool(
                            result.success
                            and feasible
                            and joint_motion is not None
                            and joint_motion <= reference_motion * 1.10
                        ),
                    }
                )
        finally:
            backend.destroy()

    by_task = {}
    for task_name in task_names:
        selected = [record for record in records if record["task"] == task_name]
        successful = [record for record in selected if record["success"]]
        quality = [record for record in selected if record["quality_gate"]]
        by_task[task_name] = {
            "samples": len(selected),
            "successes": len(successful),
            "success_rate": len(successful) / len(selected) if selected else 0.0,
            "quality_gate_passes": len(quality),
            "quality_gate_rate": len(quality) / len(selected) if selected else 0.0,
            "wall_ms": _summary([record["wall_ms"] for record in successful]),
            "solve_ms": _summary([record["solve_ms"] for record in successful]),
            "endpoint_check_ms": _summary([record["endpoint_check_ms"] for record in successful]),
            "graph_ms": _summary([record["graph_ms"] for record in successful]),
            "trajopt_ms": _summary([record["trajopt_ms"] for record in successful]),
            "interpolation_ms": _summary([record["interpolation_ms"] for record in successful]),
            "joint_motion_ratio": _summary([
                record["joint_motion_ratio"] for record in successful
                if record["joint_motion_ratio"] is not None
            ]),
            "trajectory_duration_s": _summary([
                record["trajectory_duration_s"] for record in successful
                if record["trajectory_duration_s"] is not None
            ]),
            "max_joint_step": _summary([
                record["max_joint_step"] for record in successful
                if record["max_joint_step"] is not None
            ]),
            "smoothness_cost": _summary([
                record["smoothness_cost"] for record in successful
                if record["smoothness_cost"] is not None
            ]),
        }

    report = {
        "label": args.label,
        "configuration": vars(args),
        "initialization_ms": initialization,
        "endpoint_feasibility": endpoint_feasibility,
        "payload_sphere_world_bounds": payload_sphere_world_bounds,
        "summary_by_task": by_task,
        "records": records,
    }
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / f"{args.label}.json"
    csv_path = output_dir / f"{args.label}.csv"
    json_path.write_text(json.dumps(report, indent=2) + "\n")
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    print(json.dumps({"json": str(json_path), "csv": str(csv_path), "summary_by_task": by_task}, indent=2))


if __name__ == "__main__":
    main()
