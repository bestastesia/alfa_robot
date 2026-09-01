#!/usr/bin/env python3
"""Batch-test deterministic radial extraction followed by loaded shortcut/RRT."""
from __future__ import annotations

import argparse
import csv
import gzip
import json
import os
import shlex
import subprocess
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

import yaml


REPO = Path(__file__).resolve().parents[3]
ROS_WS = REPO / "ros2_ws"
PROTOTYPE_EXECUTABLE = (
    REPO / "build" / "alfa_robot_benchmarks" / "analytic_radial_extract_prototype"
)
MONITOR_DIR = ROS_WS / "src" / "alfa_robot_moveit_config" / "scripts"
if str(MONITOR_DIR) not in sys.path:
    sys.path.insert(0, str(MONITOR_DIR))

import extract_stage_monitor_console as monitor  # noqa: E402
import process_lifecycle  # noqa: E402


ROW_SPECS = {
    1: (1, 3),
    2: (4, 6),
    3: (7, 9),
    4: (10, 12),
    5: (13, 15),
}


def latest_launch_template() -> Path:
    candidates = list(
        (REPO / "data" / "ik_benchmark" / "extract_stage_monitor").glob(
            "L*_R*/launch_command.sh"
        )
    )
    if not candidates:
        raise FileNotFoundError("no extract monitor launch_command.sh found")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def launch_tokens(template: Path, overrides: dict[str, str]) -> list[str]:
    command_line = next(
        line.strip()
        for line in reversed(template.read_text().splitlines())
        if line.strip().startswith("ros2 launch ")
    )
    tokens = shlex.split(command_line)
    prefixes = tuple(f"{name}:=" for name in overrides)
    tokens = [token for token in tokens if not token.startswith(prefixes)]
    tokens.extend(f"{name}:={value}" for name, value in overrides.items())
    return tokens


def gzip_file(path: Path) -> None:
    if not path.exists():
        return
    with path.open("rb") as source, gzip.open(path.with_suffix(path.suffix + ".gz"), "wb") as target:
        target.write(source.read())
    path.unlink()


def write_prototype_parameter_file(output: Path, env: dict[str, str]) -> Path:
    dump_dir = output / "prototype_parameter_dump"
    dump_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["ros2", "param", "dump", "/dual_arm_planner", "--output-dir", str(dump_dir)],
        cwd=ROS_WS,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        timeout=10.0,
        check=True,
    )
    dumped = dump_dir / "dual_arm_planner.yaml"
    payload = yaml.safe_load(dumped.read_text())
    planner_parameters = payload["/dual_arm_planner"]["ros__parameters"]
    required = {
        name: planner_parameters[name]
        for name in ("robot_description", "robot_description_semantic")
    }
    parameter_file = output / "analytic_radial_extract_prototype.params.yaml"
    parameter_file.write_text(
        yaml.safe_dump({"/**": {"ros__parameters": required}}, sort_keys=False),
        encoding="utf-8",
    )
    return parameter_file


def candidate_identity(candidate: dict[str, object]) -> tuple[float, ...] | None:
    state = candidate.get("state")
    if not isinstance(state, dict):
        return None
    joint_map = state.get("joint_map")
    if not isinstance(joint_map, dict):
        return None
    names = [
        *(f"left_joint{index}" for index in range(1, 7)),
        *(f"right_joint{index}" for index in range(1, 7)),
    ]
    if any(name not in joint_map for name in names):
        return None
    return (
        round(float(candidate.get("h", joint_map.get("updown", 0.0))), 8),
        *(round(float(joint_map[name]), 8) for name in names),
    )


def ranked_ik_candidates(snapshot: dict[str, object]) -> tuple[list[dict[str, object]], int, int]:
    source = snapshot.get("all_ik_candidate_records")
    if not isinstance(source, list):
        source = snapshot.get("records", [])
    eligible = [
        candidate
        for candidate in source
        if isinstance(candidate, dict)
        and bool(candidate.get("legal", False))
        and bool(candidate.get("collision_free", False))
    ]
    eligible.sort(key=lambda candidate: float(candidate.get("score", float("inf"))))
    unique: list[dict[str, object]] = []
    identities: set[tuple[float, ...]] = set()
    for candidate in eligible:
        identity = candidate_identity(candidate)
        if identity is None or identity in identities:
            continue
        identities.add(identity)
        unique.append(candidate)
    return unique, len(source), len(eligible)


def failure_key(row: dict[str, object]) -> str:
    if not row["configure_ok"]:
        return "configure_failed"
    if not row["ik_ok"]:
        return "ik_failed"
    if not row.get("pregrasp_valid", False):
        return f"pregrasp_failed:{row.get('pregrasp_failure_reason', '')}"
    if int(row["radial_start_frame"]) < 0:
        return f"radial_no_valid_prefix:{row['radial_failure_reason']}"
    if not row["loaded_valid"]:
        return f"loaded_failed:{row['loaded_failure_reason']}"
    return "success"


def ratio_table(rows: list[dict[str, object]], key: str) -> list[tuple[object, int, int]]:
    groups: dict[object, list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        groups[row[key]].append(row)
    return [
        (group, sum(bool(item["success"]) for item in items), len(items))
        for group, items in sorted(groups.items())
    ]


def write_markdown(
    path: Path,
    rows: list[dict[str, object]],
    elapsed_s: float,
    grasp_mode: str,
    min_distance_cm: int,
    max_distance_cm: int,
) -> None:
    successes = sum(bool(row["success"]) for row in rows)
    lateral_count = len({float(row["scene_y_shift"]) for row in rows})
    distance_count = max_distance_cm - min_distance_cm + 1
    if len(rows) == distance_count * lateral_count * 3:
        matrix_line = (
            f"`{distance_count}距离({min_distance_cm / 100.0:.2f}～"
            f"{max_distance_cm / 100.0:.2f}m) × {lateral_count}横移(-0.10～+0.10m) "
            f"× 3排 = {len(rows)}组`"
        )
    elif len(rows) == distance_count * lateral_count * 2:
        matrix_line = (
            f"`{distance_count}距离({min_distance_cm / 100.0:.2f}～"
            f"{max_distance_cm / 100.0:.2f}m) × {lateral_count}横移(-0.10～+0.10m) "
            f"× 2排 = {len(rows)}组`"
        )
    else:
        matrix_line = f"固定筛选测试组，共 `{len(rows)}组`"
    strategy_lines = (
        [
            "- IK：全部合法、无碰撞解按代价升序去重，最多依次尝试64个；首个完整链路成功即停止。",
            "- 第一段：顶吸工具轴保持竖直向下，末端按 1cm 沿 x 靠近机器人，同时第一径向连线向 0°转正并缩短。",
            "- 第二段：完成第30步后，从每个合法帧检查一次到负重位的完整携箱 Shortcut；首个通过即成功。",
        ]
        if grasp_mode == "top_suction"
        else [
            "- IK：全部合法、无碰撞解按代价升序去重，最多依次尝试64个；首个完整链路成功即停止。",
            "- 第一段：径向解析、定点转腕和径向续推；径向转动达到30°后逐检查点尝试完整携箱 Shortcut。",
            "- 第二段：到 `[0,-90,120,-75,0,0]° / updown=0.1m`，Shortcut失败时才由局部RRT修补。",
        ]
    )
    lines = [
        f"# {grasp_mode}径向抽离→负重规划网格测试",
        "",
        f"- 矩阵：{matrix_line}",
        *strategy_lines,
        f"- 成功：`{successes}/{len(rows)} = {100.0 * successes / max(1, len(rows)):.2f}%`",
        f"- 批量墙钟：`{elapsed_s:.1f}s`，平均 `{1000.0 * elapsed_s / max(1, len(rows)):.1f}ms/组`。",
        "",
        "## 分排",
        "",
        "| 排 | 成功 | 总数 | 成功率 |",
        "|---:|---:|---:|---:|",
    ]
    for group, success, total in ratio_table(rows, "row"):
        lines.append(f"| {group} | {success} | {total} | {100.0 * success / total:.1f}% |")
    lines.extend(["", "## 分距离", "", "| X距离/m | 成功 | 总数 | 成功率 |", "|---:|---:|---:|---:|"])
    for group, success, total in ratio_table(rows, "front_x"):
        lines.append(f"| {float(group):.2f} | {success} | {total} | {100.0 * success / total:.1f}% |")
    lines.extend(["", "## 分横移", "", "| 横移/m | 成功 | 总数 | 成功率 |", "|---:|---:|---:|---:|"])
    for group, success, total in ratio_table(rows, "scene_y_shift"):
        lines.append(f"| {float(group):+.2f} | {success} | {total} | {100.0 * success / total:.1f}% |")
    failures = Counter(failure_key(row) for row in rows)
    lines.extend(["", "## 结果分类", "", "| 分类 | 数量 |", "|---|---:|"])
    for reason, count in failures.most_common():
        lines.append(f"| `{reason}` | {count} |")
    path.write_text("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--launch-template", type=Path, default=None)
    parser.add_argument("--service-timeout", type=float, default=30.0)
    parser.add_argument("--case-names-file", type=Path, default=None)
    parser.add_argument("--rows", default="1,2,3")
    parser.add_argument("--grasp-mode", choices=("front", "top_suction"), default="front")
    parser.add_argument("--top-suction-z-offset-m", type=float, default=0.20)
    parser.add_argument("--resume-radial-after-orientation", action="store_true")
    parser.add_argument("--interleave-loaded-shortcuts", action="store_true")
    parser.add_argument("--disable-updown-compensation", action="store_true")
    parser.add_argument("--shortcut-start-radial-rotation-deg", type=float, default=30.0)
    parser.add_argument("--top-shortcut-start-step", type=int, default=30)
    parser.add_argument("--lateral-step-cm", type=int, default=2)
    parser.add_argument("--ik-candidate-limit", type=int, default=64)
    parser.add_argument("--candidate-workers", type=int, default=8)
    parser.add_argument("--min-distance-cm", type=int, default=75)
    parser.add_argument("--max-distance-cm", type=int, default=86)
    args = parser.parse_args()

    row_indices = tuple(int(value.strip()) for value in args.rows.split(",") if value.strip())
    if not row_indices or any(row not in ROW_SPECS for row in row_indices):
        raise ValueError(f"--rows must only contain {sorted(ROW_SPECS)}")
    if args.lateral_step_cm <= 0 or 20 % args.lateral_step_cm != 0:
        raise ValueError("--lateral-step-cm must be a positive divisor of 20")
    if args.ik_candidate_limit <= 0:
        raise ValueError("--ik-candidate-limit must be positive")
    if args.candidate_workers <= 0:
        raise ValueError("--candidate-workers must be positive")
    if args.top_suction_z_offset_m <= 0.0:
        raise ValueError("--top-suction-z-offset-m must be positive")
    if args.min_distance_cm <= 0 or args.max_distance_cm < args.min_distance_cm:
        raise ValueError("distance range must be positive and ordered")
    rows_to_test = tuple((row, *ROW_SPECS[row]) for row in row_indices)
    top_suction = args.grasp_mode == "top_suction"

    selected_case_names = None
    if args.case_names_file is not None:
        selected_case_names = {
            line.strip()
            for line in args.case_names_file.read_text().splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        }
        if not selected_case_names:
            raise ValueError("--case-names-file contains no cases")
    total_cases = (
        len(selected_case_names)
        if selected_case_names is not None
        else (args.max_distance_cm - args.min_distance_cm + 1)
        * (20 // args.lateral_step_cm + 1)
        * len(rows_to_test)
    )

    output = args.output.resolve()
    cases_dir = output / "cases"
    cases_dir.mkdir(parents=True, exist_ok=True)
    template = (args.launch_template or latest_launch_template()).resolve()
    domain = process_lifecycle.configure_ros_domain("auto")
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain)
    env["ROS_HOME"] = str(output / "ros_home")
    env["ROS_LOG_DIR"] = str(output / "ros_log")
    Path(env["ROS_HOME"]).mkdir(parents=True, exist_ok=True)
    Path(env["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)

    initial_snapshot = output / "initial_snapshot.json"
    tokens = launch_tokens(
        template,
        {
            "box_front_x": "0.75",
            "scene_y_shift": "-0.10",
            "extract_demo_left_box_id": str(rows_to_test[0][1]),
            "extract_demo_right_box_id": str(rows_to_test[0][2]),
            "extract_monitor_left_top_suction": str(top_suction).lower(),
            "extract_monitor_right_top_suction": str(top_suction).lower(),
            "extract_monitor_top_suction": str(top_suction).lower(),
            "top_suction_z_offset": str(args.top_suction_z_offset_m),
            "carried_box_grasp_lateral_offset": "-0.01",
            "extract_monitor_snapshot_path": str(initial_snapshot),
            "record_jsonl_path": str(output / "flow_unused.jsonl"),
        },
    )
    planner_log = (output / "planner.log").open("w")
    planner = subprocess.Popen(
        tokens,
        cwd=ROS_WS,
        env=env,
        stdout=planner_log,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )
    rows: list[dict[str, object]] = []
    started = time.monotonic()
    try:
        monitor.wait_for_service(
            "/dual_arm_planner/configure_extract_monitor",
            planner,
            args.service_timeout,
            output / "planner.log",
        )
        prototype_parameter_file = write_prototype_parameter_file(output, env)
        with monitor.ExtractMonitorServiceClient(
            configure_service="/dual_arm_planner/configure_extract_monitor",
            trigger_service="/dual_arm_planner/run_extract_monitor_next",
            timeout=args.service_timeout,
            node_name="radial_loaded_grid_client",
        ) as client:
            case_index = 0
            for distance_cm in range(args.min_distance_cm, args.max_distance_cm + 1):
                for shift_cm in range(-10, 11, args.lateral_step_cm):
                    for row_index, left_id, right_id in rows_to_test:
                        case_index += 1
                        label = f"row{row_index}_x{distance_cm:02d}_y{shift_cm:+03d}"
                        if selected_case_names is not None and label not in selected_case_names:
                            case_index -= 1
                            continue
                        case_dir = cases_dir / label
                        case_dir.mkdir(parents=True, exist_ok=True)
                        snapshot = case_dir / "snapshot.json"
                        result_path = case_dir / "result.json"
                        front_x = distance_cm / 100.0
                        scene_y_shift = shift_cm / 100.0
                        configure_ok, configure_message, configure_ms = client.configure(
                            left_id,
                            right_id,
                            snapshot,
                            args.service_timeout,
                            top_suction,
                            top_suction,
                            runtime_config={
                                "box_front_x": front_x,
                                "scene_y_shift": scene_y_shift,
                                "extract_rollout_mode": "greedy",
                                "loaded_lateral_shift_enabled": True,
                                "extract_box_pose_rrt_max_iterations": 160,
                            },
                        )
                        ik_ok = False
                        ik_ms = 0.0
                        trigger_message = ""
                        if configure_ok:
                            ik_ok, trigger_message, ik_ms = client.trigger(args.service_timeout)

                        record: dict[str, object] = {
                            "case": label,
                            "row": row_index,
                            "left_box_id": left_id,
                            "right_box_id": right_id,
                            "front_x": front_x,
                            "scene_y_shift": scene_y_shift,
                            "configure_ok": configure_ok,
                            "configure_ms": configure_ms,
                            "ik_ok": ik_ok,
                            "ik_wall_ms": ik_ms,
                            "radial_start_frame": -1,
                            "radial_failure_frame": -1,
                            "radial_failure_reason": "",
                            "orientation_progressed": False,
                            "orientation_complete": False,
                            "orientation_valid_steps": 0,
                            "orientation_used": False,
                            "pregrasp_valid": False,
                            "pregrasp_failure_reason": "",
                            "loaded_valid": False,
                            "loaded_start_phase": "",
                            "loaded_start_step": -1,
                            "loaded_method": "",
                            "loaded_failure_reason": "",
                            "loaded_planning_ms": 0.0,
                            "loaded_points": 0,
                            "ik_candidates_source_count": 0,
                            "ik_candidates_eligible_count": 0,
                            "ik_candidates_available": 0,
                            "ik_candidates_considered": 0,
                            "ik_candidates_attempted": 0,
                            "selected_candidate_index": -1,
                            "selected_candidate_score": 0.0,
                            "candidate_attempts": [],
                            "success": False,
                            "result_path": str(result_path),
                        }
                        if ik_ok and snapshot.exists():
                            snapshot_payload = json.loads(snapshot.read_text())
                            ranked_records, source_count, eligible_count = ranked_ik_candidates(
                                snapshot_payload
                            )
                            records = ranked_records[: args.ik_candidate_limit]
                            snapshot_payload["records"] = records
                            snapshot.write_text(
                                json.dumps(snapshot_payload, separators=(",", ":"))
                            )
                            candidate_count = len(records)
                            record["ik_candidates_source_count"] = source_count
                            record["ik_candidates_eligible_count"] = eligible_count
                            record["ik_candidates_available"] = len(ranked_records)
                            record["ik_candidates_considered"] = candidate_count
                            selected_result = None
                            temporary_result = case_dir / ".candidate_result.json"
                            try:
                                run = subprocess.run(
                                    [
                                        str(PROTOTYPE_EXECUTABLE),
                                        "--snapshot", str(snapshot),
                                        "--output", str(temporary_result),
                                        "--frames", "50",
                                        "--target-radius", "0.79",
                                        "--resume-radial-after-orientation",
                                        f"{'true' if args.resume_radial_after_orientation else 'false'}",
                                        "--interleave-loaded-shortcuts",
                                        f"{'true' if args.interleave_loaded_shortcuts else 'false'}",
                                        "--updown-compensation-enabled",
                                        f"{'false' if args.disable_updown_compensation else 'true'}",
                                        "--shortcut-start-radial-rotation-deg",
                                        f"{args.shortcut_start_radial_rotation_deg}",
                                        "--path-mode",
                                        f"{'top_horizontal_retract' if top_suction else 'radial'}",
                                        "--top-retreat-step-x", "0.01",
                                        "--top-shortcut-start-step",
                                        f"{args.top_shortcut_start_step}",
                                        "--candidate-index", "0",
                                        "--candidate-count", str(candidate_count),
                                        "--ros-args", "--params-file",
                                        str(prototype_parameter_file),
                                    ],
                                    cwd=REPO,
                                    env=env,
                                    stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL,
                                    timeout=max(args.service_timeout, 10.0 * candidate_count),
                                    check=False,
                                )
                                if run.returncode in (0, 3) and temporary_result.exists():
                                    selected_result = json.loads(temporary_result.read_text())
                                    record["candidate_attempts"] = list(
                                        selected_result.get("candidate_attempts", [])
                                    )
                                    record["ik_candidates_attempted"] = len(
                                        record["candidate_attempts"]
                                    )
                                    record["selected_candidate_index"] = int(
                                        selected_result.get("candidate_index", -1)
                                    )
                                    record["selected_candidate_score"] = float(
                                        selected_result.get("candidate_score", 0.0)
                                    )
                                else:
                                    record["candidate_attempts"].append(
                                        {
                                            "candidate_index": -1,
                                            "score": 0.0,
                                            "success": False,
                                            "failure_reason": f"prototype_exit_{run.returncode}",
                                        }
                                    )
                            except subprocess.TimeoutExpired:
                                record["candidate_attempts"].append(
                                    {
                                        "candidate_index": -1,
                                        "score": 0.0,
                                        "success": False,
                                        "failure_reason": "prototype_timeout",
                                    }
                                )
                            finally:
                                temporary_result.unlink(missing_ok=True)
                            if selected_result is not None:
                                result_path.write_text(
                                    json.dumps(selected_result, ensure_ascii=False, indent=2)
                                    + "\n"
                                )
                                result = selected_result
                                orientation = result.get("orientation_only_transition", {})
                                pregrasp = result.get("pregrasp_transition", {})
                                loaded = result.get("loaded_transition", {})
                                record.update(
                                    radial_start_frame=int(loaded.get("start_radial_frame", -1)),
                                    radial_failure_frame=int(result.get("first_failure_frame", -1)),
                                    radial_failure_reason=str(result.get("first_failure_reason", "")),
                                    orientation_progressed=bool(orientation.get("progressed", False)),
                                    orientation_complete=bool(orientation.get("complete", False)),
                                    orientation_valid_steps=int(orientation.get("valid_steps", 0)),
                                    orientation_used=bool(
                                        orientation.get("used_for_loaded_transition", False)
                                    ),
                                    pregrasp_valid=bool(pregrasp.get("valid", False)),
                                    pregrasp_failure_reason=str(
                                        pregrasp.get("failure_reason", "")
                                    ),
                                    loaded_valid=bool(loaded.get("valid", False)),
                                    loaded_start_phase=str(loaded.get("start_phase", "")),
                                    loaded_start_step=int(loaded.get("start_phase_step", -1)),
                                    loaded_method=str(loaded.get("method", "")),
                                    loaded_failure_reason=str(loaded.get("failure_reason", "")),
                                    loaded_planning_ms=float(loaded.get("planning_ms", 0.0)),
                                    loaded_points=int(loaded.get("point_count", 0)),
                                )
                                record["success"] = (
                                    record["pregrasp_valid"]
                                    and record["radial_start_frame"] >= 0
                                    and record["loaded_valid"]
                                )
                            elif not records:
                                record["pregrasp_failure_reason"] = (
                                    "no_eligible_collision_free_ik_candidate"
                                )
                            else:
                                record["pregrasp_failure_reason"] = (
                                    "all_considered_ik_candidates_failed"
                                )
                        else:
                            record["loaded_failure_reason"] = (
                                trigger_message if configure_ok else configure_message
                            )[-500:]
                        rows.append(record)
                        gzip_file(snapshot)
                        if case_index % 10 == 0 or case_index == total_cases:
                            succeeded = sum(bool(item["success"]) for item in rows)
                            print(
                                f"progress={case_index}/{total_cases} success={succeeded}/{case_index} "
                                f"elapsed={time.monotonic() - started:.1f}s",
                                flush=True,
                            )
    finally:
        monitor.terminate_process(planner)
        planner_log.close()

    elapsed_s = time.monotonic() - started
    (output / "summary.json").write_text(
        json.dumps(
            {
                "matrix": {
                    "front_x_m": [
                        value / 100.0
                        for value in range(args.min_distance_cm, args.max_distance_cm + 1)
                    ],
                    "scene_y_shift_m": [
                        value / 100.0
                        for value in range(-10, 11, args.lateral_step_cm)
                    ],
                    "rows": list(row_indices),
                    "grasp_mode": args.grasp_mode,
                    "shortcut_start_radial_rotation_deg":
                    args.shortcut_start_radial_rotation_deg,
                    "top_shortcut_start_step": args.top_shortcut_start_step,
                    "count": len(rows),
                },
                "elapsed_s": elapsed_s,
                "success_count": sum(bool(row["success"]) for row in rows),
                "rows": rows,
            },
            indent=2,
            ensure_ascii=False,
        )
        + "\n"
    )
    with (output / "summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    write_markdown(
        output / "summary.md",
        rows,
        elapsed_s,
        args.grasp_mode,
        args.min_distance_cm,
        args.max_distance_cm,
    )
    print(output / "summary.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
