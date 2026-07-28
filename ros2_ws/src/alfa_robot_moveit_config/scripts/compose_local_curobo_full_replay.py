#!/usr/bin/env python3
"""Compose a continuous diagnostic full-flow replay ending in a real cuRobo repair."""

from __future__ import annotations

import argparse
import copy
import json
import math
import re
from pathlib import Path
from typing import Any


def read_jsonl(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    header: dict[str, Any] = {}
    stages: list[dict[str, Any]] = []
    summary: dict[str, Any] = {}
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        if row.get("type") == "header":
            header = row
        elif row.get("type") == "stage":
            stages.append(row)
        elif row.get("type") == "summary":
            summary = row
    return header, stages, summary


def point_positions(stage: dict[str, Any], first: bool) -> list[float]:
    points = stage.get("trajectory", {}).get("points", [])
    if not points:
        raise ValueError(f"stage has no trajectory points: {stage.get('stage')}")
    point = points[0] if first else points[-1]
    return [float(value) for value in point.get("positions", [])]


def continuity_delta(left: dict[str, Any], right: dict[str, Any]) -> float:
    lhs = point_positions(left, first=False)
    rhs = point_positions(right, first=True)
    if len(lhs) != len(rhs):
        raise ValueError("trajectory joint count differs across stage boundary")
    return max((abs(a - b) for a, b in zip(lhs, rhs)), default=0.0)


def requested_goal_error(stage: dict[str, Any]) -> float:
    joint_names = list(stage.get("trajectory", {}).get("joint_names", []))
    goal_map = stage.get("goal_state", {}).get("joint_map", {})
    endpoint = point_positions(stage, first=False)
    if len(endpoint) != len(joint_names) or any(name not in goal_map for name in joint_names):
        raise ValueError("repair stage cannot be compared with its requested goal")
    return max(
        (abs(value - float(goal_map[name])) for name, value in zip(joint_names, endpoint)),
        default=0.0,
    )


def label_full_stage(stage: dict[str, Any], extract_index: int) -> str:
    original = str(stage.get("stage", ""))
    if "pre_attach_loaded_to_ik" in original:
        return "01_预抓取到抓取位_箱体未挂载"
    if is_loaded_plan_stage(stage):
        return "03_自然选择的负重目标轨迹"
    return f"02_吸附完成并负重脱离_step_{extract_index:02d}"


def is_loaded_plan_stage(stage: dict[str, Any]) -> bool:
    original = str(stage.get("stage", ""))
    stage_kind = str(stage.get("extra", {}).get("stage_kind", ""))
    return (
        "selected_loaded_plan" in original
        or "loaded_plan_attempt" in original
        or stage_kind == "monitor_loaded_plan_attempt_replay"
    )


def snapshot_replay_stages(snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    """Read either the finalized top-level replay or an embedded candidate replay."""
    stages = list(snapshot.get("replay_stages", []))
    if stages:
        return stages

    records = list(snapshot.get("records", []))
    selected = [record for record in records if record.get("loaded_plan_selected")]
    candidates = selected or [record for record in records if record.get("success")]
    for record in candidates:
        stages = list(record.get("replay_stages", []))
        if stages:
            return stages
    return []


def final_target_hold_stage(repair: dict[str, Any], sample_count: int) -> dict[str, Any]:
    hold = copy.deepcopy(repair)
    goal_state = hold.get("goal_state", {})
    goal_map = goal_state.get("joint_map", {})
    joint_names = list(hold.get("trajectory", {}).get("joint_names", []))
    goal_positions = [float(goal_map[name]) for name in joint_names]
    hold["stage"] = "06_shortcut请求目标状态保持"
    hold["start_state"] = copy.deepcopy(goal_state)
    hold["trajectory"] = {
        "joint_names": joint_names,
        "point_count": sample_count,
        "points": [
            {
                "time_from_start_sec": 0.1 * index,
                "positions": goal_positions,
                "velocities": [0.0 for _ in goal_positions],
            }
            for index in range(sample_count)
        ],
    }
    hold["extra"] = {
        "stage_kind": "diagnostic_requested_goal_hold",
        "valid": True,
        "planning_method": "final_target_hold",
        "message": "visual-only hold at the exact requested shortcut goal",
        "requested_goal_joint_names": joint_names,
        "requested_goal_positions": goal_positions,
    }
    return hold


def shortcut_collision_evidence(summary: dict[str, Any]) -> tuple[int, str, str]:
    failures = [str(value) for value in summary.get("failures", [])]
    for failure in failures:
        point_match = re.search(r"trajectory point (\d+)", failure)
        pair_match = re.search(r"(carried_[A-Za-z0-9_]+) overlaps ([A-Za-z0-9_]+)", failure)
        if point_match and pair_match:
            return int(point_match.group(1)), pair_match.group(1), pair_match.group(2)
    raise ValueError("repair summary has no production shortcut collision evidence")


def unrepaired_shortcut_stage(
    repair: dict[str, Any],
    flow_index: int,
    collision_point: int,
    collision_box: str,
    collision_obstacle: str,
) -> dict[str, Any]:
    joint_names = list(repair.get("trajectory", {}).get("joint_names", []))
    start_map = repair.get("start_state", {}).get("joint_map", {})
    goal_map = repair.get("goal_state", {}).get("joint_map", {})
    if not joint_names or any(name not in start_map or name not in goal_map for name in joint_names):
        raise ValueError("repair stage cannot provide shortcut endpoints")

    start = [float(start_map[name]) for name in joint_names]
    goal = [float(goal_map[name]) for name in joint_names]
    deltas = [
        (to - begin)
        if name == "updown"
        else math.atan2(math.sin(to - begin), math.cos(to - begin))
        for name, begin, to in zip(joint_names, start, goal)
    ]
    max_delta = max((abs(delta) for delta in deltas), default=0.0)
    steps = max(2, math.ceil(max_delta / math.radians(5.0)) + 1)
    if collision_point < 0 or collision_point >= steps:
        raise ValueError(
            f"production collision point {collision_point} is outside shortcut [0,{steps})"
        )

    points = []
    statuses = []
    for point_index in range(steps):
        ratio = point_index / (steps - 1)
        points.append(
            {
                "time_from_start_sec": ratio,
                "positions": [begin + delta * ratio for begin, delta in zip(start, deltas)],
                "velocities": [0.0 for _ in joint_names],
            }
        )
        if point_index < collision_point:
            statuses.append("fcl_clear")
        elif point_index == collision_point:
            statuses.append("fcl_collision")
        else:
            statuses.append("not_evaluated_after_first_collision")

    shortcut = copy.deepcopy(repair)
    shortcut["stage"] = f"流程_{flow_index:02d}/04_原始shortcut全程_首次碰撞点标红"
    shortcut["trajectory"] = {
        "joint_names": joint_names,
        "point_count": steps,
        "points": points,
    }
    shortcut["extra"] = {
        "stage_kind": "diagnostic_unrepaired_shortcut",
        "planning_method": "joint_interpolation_rejected",
        "valid": False,
        "rerun_state_reset": True,
        "collision_check_status_by_point": statuses,
        "collision_pairs_by_point": {
            str(collision_point): [f"{collision_box} vs {collision_obstacle}"]
        },
        "collision_box_ids_by_point": {str(collision_point): [collision_box]},
        "collision_obstacle_ids_by_point": {str(collision_point): [collision_obstacle]},
        "first_collision_point": collision_point,
        "collision_point_indexing": "zero_based",
        "collision_evidence": "production PlanningScene/FCL full-scene validation message",
        "message": (
            f"trajectory point {collision_point}: {collision_box} overlaps "
            f"{collision_obstacle}; later points were not evaluated by the fail-fast validator"
        ),
    }
    return shortcut


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--full-snapshot", type=Path, required=True)
    parser.add_argument("--bridge-jsonl", type=Path)
    parser.add_argument("--repair-jsonl", type=Path, required=True)
    parser.add_argument(
        "--repair-stage-index",
        type=int,
        default=0,
        help="Select one successful repair stage when benchmark JSONL contains repeats",
    )
    parser.add_argument("--output-jsonl", type=Path, required=True)
    parser.add_argument(
        "--include-selected-loaded-plan",
        action="store_true",
        help="Keep the naturally selected loaded plan before bridging into the diagnostic collision case",
    )
    parser.add_argument("--continuity-tolerance", type=float, default=1e-6)
    parser.add_argument("--goal-tolerance", type=float, default=1e-6)
    parser.add_argument("--final-hold-samples", type=int, default=30)
    parser.add_argument(
        "--staged-repair-indices",
        default="",
        help=(
            "Comma-separated successful repair indices to append as independent Rerun stages. "
            "This intentionally permits a state reset and does not create a bridge trajectory."
        ),
    )
    parser.add_argument(
        "--repeat-full-flow-per-repair",
        action="store_true",
        help=(
            "Repeat the complete source flow for every selected repair and insert the full "
            "rejected shortcut before each repaired trajectory."
        ),
    )
    args = parser.parse_args()

    snapshot = json.loads(args.full_snapshot.read_text())
    full_stages = snapshot_replay_stages(snapshot)
    if not args.include_selected_loaded_plan:
        full_stages = [stage for stage in full_stages if not is_loaded_plan_stage(stage)]
    if not full_stages:
        raise SystemExit("full snapshot has no pre-loaded replay stages")

    repair_header, repair_stages, repair_summary = read_jsonl(args.repair_jsonl)
    if args.staged_repair_indices:
        indices = [int(value.strip()) for value in args.staged_repair_indices.split(",")]
        if not indices:
            raise SystemExit("--staged-repair-indices is empty")
        selected_repairs = []
        for index in indices:
            if index < 0 or index >= len(repair_stages):
                raise SystemExit(
                    f"repair stage index {index} is outside [0, {len(repair_stages) - 1}]"
                )
            repair = copy.deepcopy(repair_stages[index])
            if repair.get("extra", {}).get("planning_method") != "shortcut_local_curobo":
                raise SystemExit(f"repair stage {index} is not shortcut_local_curobo")
            selected_repairs.append(repair)

        output_stages = []
        reset_deltas = []

        def append_full_flow(flow_index: int) -> None:
            extract_index = 0
            for stage in full_stages:
                item = copy.deepcopy(stage)
                extra = dict(item.get("extra", {}))
                extra["original_stage"] = item.get("stage", "")
                item["stage"] = f"流程_{flow_index:02d}/{label_full_stage(item, extract_index)}"
                if not is_loaded_plan_stage(item):
                    extract_index += 1
                item["extra"] = extra
                output_stages.append(item)

        if not args.repeat_full_flow_per_repair:
            append_full_flow(1)

        collision_point = -1
        collision_box = ""
        collision_obstacle = ""
        if args.repeat_full_flow_per_repair:
            collision_point, collision_box, collision_obstacle = shortcut_collision_evidence(
                repair_summary
            )

        previous = None
        for display_index, repair in enumerate(selected_repairs, start=1):
            if args.repeat_full_flow_per_repair:
                if previous is not None:
                    reset_deltas.append(continuity_delta(previous, full_stages[0]))
                append_full_flow(display_index)
                shortcut = unrepaired_shortcut_stage(
                    repair,
                    display_index,
                    collision_point,
                    collision_box,
                    collision_obstacle,
                )
                reset_deltas.append(continuity_delta(output_stages[-1], shortcut))
                output_stages.append(shortcut)
                previous = shortcut
            else:
                previous = output_stages[-1]
            reset_deltas.append(continuity_delta(previous, repair))
            goal_error = requested_goal_error(repair)
            if goal_error > args.goal_tolerance:
                raise SystemExit(
                    f"repair endpoint differs from the requested goal: {goal_error} > {args.goal_tolerance}"
                )
            extra = dict(repair.get("extra", {}))
            extra.update(
                {
                    "stage_kind": "diagnostic_shortcut_local_curobo_repair",
                    "diagnostic_note": "independent successful repair replay; no bridge inserted",
                    "rerun_state_reset": True,
                    "requested_goal_max_error_rad": goal_error,
                }
            )
            repair["stage"] = (
                f"流程_{display_index:02d}/05_shortcut碰撞后_local_curobo局部修补成功"
            )
            repair["extra"] = extra
            output_stages.append(repair)
            previous = repair

        header = repair_header
        header.update(
            {
                "type": "header",
                "schema": "moveit_box_stack_flow_v1",
                "show_box_stack": True,
                "removed_box_ids": [
                    int(snapshot["left_box_id"]),
                    int(snapshot["right_box_id"]),
                ],
                "box_front_x": float(snapshot.get("box_front_x", 0.925)),
                "scene_y_shift": float(snapshot.get("scene_y_shift", -0.4)),
                "diagnostic_composite": True,
                "independent_stage_replay": True,
                "execute": False,
            }
        )
        records = [header]
        for index, stage in enumerate(output_stages):
            stage["type"] = "stage"
            stage["stage_index"] = index
            records.append(stage)
        records.append(
            {
                "type": "summary",
                "diagnostic_composite": True,
                "independent_stage_replay": True,
                "stage_count": len(output_stages),
                "repair_stage_indices": indices,
                "state_reset_max_deltas": reset_deltas,
                "full_flow_repeated_per_repair": args.repeat_full_flow_per_repair,
                "shortcut_first_collision_point": (
                    collision_point if args.repeat_full_flow_per_repair else None
                ),
                "repair": repair_summary,
                "warning": (
                    "repair trajectories are independent Rerun stages; no interpolated bridge "
                    "was inserted between stage sources"
                ),
            }
        )
        args.output_jsonl.parent.mkdir(parents=True, exist_ok=True)
        args.output_jsonl.write_text(
            "".join(json.dumps(record, ensure_ascii=False) + "\n" for record in records)
        )
        print(
            json.dumps(
                {
                    "output_jsonl": str(args.output_jsonl),
                    "stage_count": len(output_stages),
                    "repair_stage_indices": indices,
                    "state_reset_max_deltas": reset_deltas,
                    "full_flow_repeated_per_repair": args.repeat_full_flow_per_repair,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return

    if args.bridge_jsonl is None:
        raise SystemExit("--bridge-jsonl is required unless --staged-repair-indices is used")
    header, bridge_stages, bridge_summary = read_jsonl(args.bridge_jsonl)
    if len(bridge_stages) != 1:
        raise SystemExit("bridge JSONL must contain exactly one stage")
    if not repair_stages:
        raise SystemExit("repair JSONL contains no stages")
    if args.repair_stage_index < 0 or args.repair_stage_index >= len(repair_stages):
        raise SystemExit(
            f"repair stage index {args.repair_stage_index} is outside "
            f"[0, {len(repair_stages) - 1}]"
        )
    bridge = bridge_stages[0]
    repair = repair_stages[args.repair_stage_index]
    if repair.get("extra", {}).get("planning_method") != "shortcut_local_curobo":
        raise SystemExit("repair stage is not shortcut_local_curobo")

    extract_to_bridge = continuity_delta(full_stages[-1], bridge)
    bridge_to_repair = continuity_delta(bridge, repair)
    goal_error = requested_goal_error(repair)
    if max(extract_to_bridge, bridge_to_repair) > args.continuity_tolerance:
        raise SystemExit(
            "stage boundary is discontinuous: "
            f"extract_to_bridge={extract_to_bridge} bridge_to_repair={bridge_to_repair}"
        )
    if goal_error > args.goal_tolerance:
        raise SystemExit(
            f"repair endpoint differs from the requested goal: {goal_error} > {args.goal_tolerance}"
        )

    output_stages: list[dict[str, Any]] = []
    extract_index = 0
    for stage in full_stages:
        item = copy.deepcopy(stage)
        extra = dict(item.get("extra", {}))
        extra["original_stage"] = item.get("stage", "")
        item["stage"] = label_full_stage(item, extract_index)
        if not any(
            marker in str(extra["original_stage"])
            for marker in ("pre_attach_loaded_to_ik", "selected_loaded_plan")
        ):
            extract_index += 1
        item["extra"] = extra
        output_stages.append(item)

    bridge = copy.deepcopy(bridge)
    bridge_extra = dict(bridge.get("extra", {}))
    bridge_extra.update(
        {
            "stage_kind": "diagnostic_loaded_bridge",
            "diagnostic_note": "FCL-validated bridge into the known collision case",
        }
    )
    bridge["stage"] = "04_负重安全过渡到真碰撞工况"
    bridge["extra"] = bridge_extra
    output_stages.append(bridge)

    repair = copy.deepcopy(repair)
    repair_extra = dict(repair.get("extra", {}))
    repair_extra.update(
        {
            "stage_kind": "diagnostic_shortcut_local_curobo_repair",
            "diagnostic_note": "automatic local repair after shortcut collision detection",
            "requested_goal_max_error_rad": goal_error,
        }
    )
    repair["stage"] = "05_shortcut碰撞后_local_curobo局部修补"
    repair["extra"] = repair_extra
    output_stages.append(repair)
    if args.final_hold_samples > 0:
        output_stages.append(final_target_hold_stage(repair, args.final_hold_samples))

    header.update(
        {
            "type": "header",
            "schema": "moveit_box_stack_flow_v1",
            "show_box_stack": True,
            "removed_box_ids": [
                int(snapshot["left_box_id"]),
                int(snapshot["right_box_id"]),
            ],
            "box_front_x": float(snapshot.get("box_front_x", 0.925)),
            "scene_y_shift": float(snapshot.get("scene_y_shift", -0.4)),
            "diagnostic_composite": True,
            "execute": False,
        }
    )
    records: list[dict[str, Any]] = [header]
    for index, stage in enumerate(output_stages):
        stage["type"] = "stage"
        stage["stage_index"] = index
        records.append(stage)
    records.append(
        {
            "type": "summary",
            "diagnostic_composite": True,
            "stage_count": len(output_stages),
            "extract_to_bridge_max_delta": extract_to_bridge,
            "bridge_to_repair_max_delta": bridge_to_repair,
            "requested_goal_max_error_rad": goal_error,
            "bridge": bridge_summary,
            "repair": repair_summary,
            "repair_stage_index": args.repair_stage_index,
            "warning": (
                "collision case is injected after a real full-flow extract; "
                "this is not a naturally selected production goal"
            ),
        }
    )
    args.output_jsonl.parent.mkdir(parents=True, exist_ok=True)
    args.output_jsonl.write_text(
        "".join(json.dumps(record, ensure_ascii=False) + "\n" for record in records)
    )
    print(
        json.dumps(
            {
                "output_jsonl": str(args.output_jsonl),
                "stage_count": len(output_stages),
                "extract_to_bridge_max_delta": extract_to_bridge,
                "bridge_to_repair_max_delta": bridge_to_repair,
                "requested_goal_max_error_rad": goal_error,
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
