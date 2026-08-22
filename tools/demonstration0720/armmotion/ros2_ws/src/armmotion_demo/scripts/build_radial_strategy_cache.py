#!/usr/bin/env python3
from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path
from typing import Any

from armmotion_demo.trajectory_cache import (
    CACHE_NOMINAL_ARM_SPACING_CM,
    CACHE_SCHEMA_VERSION,
)
from generate_trajectory_cache import (
    JOINT_NAMES,
    TASK_ROWS,
    canonical_targets,
    write_cache_record,
)


TRAJECTORY_JOINT_NAMES = (
    "updown",
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="将径向策略网格结果转换为正式轨迹缓存")
    parser.add_argument("--side-root", type=Path, required=True)
    parser.add_argument("--top-root", type=Path, required=True)
    parser.add_argument("--template-cache-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    return parser.parse_args()


def load_gzip_json(path: Path) -> dict[str, Any]:
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        return json.load(stream)


def frame_positions(frame: dict[str, Any]) -> list[float]:
    return [
        float(frame["compensated_updown"]),
        *[float(value) for value in frame["left_joints"]],
        *[float(value) for value in frame["right_joints"]],
    ]


def valid_frame(frame: dict[str, Any]) -> bool:
    return (
        bool(frame.get("solved", True))
        and bool(frame.get("bounds_ok", True))
        and bool(frame.get("collision_free", True))
        and "left_joints" in frame
        and "right_joints" in frame
    )


def valid_prefix(frames: list[dict[str, Any]]) -> list[dict[str, Any]]:
    prefix = []
    for frame in frames:
        if not valid_frame(frame):
            break
        prefix.append(frame)
    return prefix


def selected_strategy_frames(result: dict[str, Any]) -> list[dict[str, Any]]:
    loaded = result["loaded_transition"]
    phase = str(loaded["start_phase"])
    step = int(loaded.get("start_phase_step", 0))
    radial = valid_prefix(list(result.get("frames", [])))
    last_radial = int(loaded.get("start_radial_frame", len(radial) - 1))
    radial = radial[: last_radial + 1]
    if phase == "top_upright_checkpoint":
        selected = valid_prefix(list(result.get("frames", [])))[: step + 1]
    elif phase == "radial_checkpoint":
        selected = radial[:step]
    elif phase == "orientation_only":
        orientation = valid_prefix(
            list(result.get("orientation_only_transition", {}).get("frames", []))
        )
        selected = [*radial, *orientation[:step]]
    elif phase == "radial_resume":
        orientation_transition = result.get("orientation_only_transition", {})
        orientation = valid_prefix(list(orientation_transition.get("frames", [])))
        orientation_count = int(
            orientation_transition.get("used_frame_count", len(orientation))
        )
        resume = valid_prefix(
            list(result.get("radial_resume_transition", {}).get("frames", []))
        )
        selected = [*radial, *orientation[:orientation_count], *resume[:step]]
    else:
        raise ValueError(f"不支持的负重起始阶段: {phase}")
    if not selected or not all(valid_frame(frame) for frame in selected):
        raise ValueError(f"策略前缀无效: phase={phase} step={step}")
    return selected


def points_from_frames(frames: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "positions": frame_positions(frame),
            "time_from_start_sec": 0.1 * index,
            "velocities": [],
        }
        for index, frame in enumerate(frames)
    ]


def trajectory_stage(name: str, points: list[dict[str, Any]]) -> dict[str, Any]:
    if not points:
        raise ValueError(f"轨迹阶段为空: {name}")
    return {
        "stage": name,
        "trajectory": {
            "joint_names": list(TRAJECTORY_JOINT_NAMES),
            "point_count": len(points),
            "points": points,
        },
    }


def max_delta(lhs: list[float], rhs: list[float]) -> tuple[float, float]:
    return (
        abs(float(lhs[0]) - float(rhs[0])),
        max(abs(float(a) - float(b)) for a, b in zip(lhs[1:], rhs[1:])),
    )


def assert_continuous(stages: list[dict[str, Any]]) -> None:
    for previous, current in zip(stages, stages[1:]):
        lhs = previous["trajectory"]["points"][-1]["positions"]
        rhs = current["trajectory"]["points"][0]["positions"]
        updown_delta, joint_delta = max_delta(lhs, rhs)
        if updown_delta > 1e-9 or joint_delta > 1e-9:
            raise ValueError(
                f"阶段边界不连续: {previous['stage']} -> {current['stage']} "
                f"updown={updown_delta:.3e} joint={joint_delta:.3e}"
            )


def renamed_template_stage(stage: dict[str, Any], pair: tuple[int, int], suffix: str) -> dict[str, Any]:
    copied = json.loads(json.dumps(stage))
    copied["stage"] = f"extract_monitor_L{pair[0]}_R{pair[1]}/{suffix}"
    copied["trajectory"]["point_count"] = len(copied["trajectory"]["points"])
    return copied


def load_place_templates(root: Path) -> dict[str, list[dict[str, Any]]]:
    templates = {}
    for mode, row in (("front", 1), ("top_suction", 4)):
        path = root / f"x_75cm_y_p00cm_row_{row}.json.gz"
        record = load_gzip_json(path)
        stages = list(record["snapshot"]["replay_stages"])
        selected = [
            stage
            for stage in stages
            if str(stage.get("stage", "")).endswith(
                ("selected_loaded_to_place", "selected_place_to_loaded")
            )
        ]
        if len(selected) != 2:
            raise ValueError(f"放置模板阶段数量错误: {path}")
        templates[mode] = selected
    return templates


def case_metadata(path: Path) -> tuple[int, int, int]:
    label = path.parent.name
    tokens = label.split("_")
    row = int(tokens[0].removeprefix("row"))
    distance_cm = int(tokens[1].removeprefix("x"))
    lateral_offset_cm = int(tokens[2].removeprefix("y"))
    return row, distance_cm, lateral_offset_cm


def cache_record_from_result(
    result: dict[str, Any],
    *,
    row: int,
    distance_cm: int,
    lateral_offset_cm: int,
    place_templates: dict[str, list[dict[str, Any]]],
) -> dict[str, Any]:
    pair = next(pair for pair, pair_row in TASK_ROWS.items() if pair_row == row)
    mode = "front" if row <= 3 else "top_suction"
    pregrasp = result.get("pregrasp_transition", {})
    if not pregrasp.get("valid", False):
        raise ValueError(f"预抓取轨迹无效: {pregrasp.get('failure_reason', '')}")
    approach_frames = valid_prefix(list(pregrasp.get("frames", [])))
    strategy_frames = selected_strategy_frames(result)
    loaded_frames = valid_prefix(list(result["loaded_transition"].get("frames", [])))
    if not approach_frames or not loaded_frames:
        raise ValueError("预抓取或负重轨迹为空")

    approach_points = points_from_frames(approach_frames)
    extract_points = points_from_frames(strategy_frames)
    loaded_points = points_from_frames(loaded_frames)
    approach_points[-1]["positions"] = list(extract_points[0]["positions"])
    loaded_points[0]["positions"] = list(extract_points[-1]["positions"])

    place = renamed_template_stage(
        place_templates[mode][0], pair, "selected_loaded_to_place"
    )
    return_loaded = renamed_template_stage(
        place_templates[mode][1], pair, "selected_place_to_loaded"
    )
    loaded_points[-1]["positions"] = list(
        place["trajectory"]["points"][0]["positions"]
    )
    prefix = f"extract_monitor_L{pair[0]}_R{pair[1]}"
    stages = [
        trajectory_stage(
            f"{prefix}/selected_pre_attach_pre_contact_to_ik", approach_points
        ),
        trajectory_stage(f"{prefix}/selected_extract_new_radial_strategy", extract_points),
        trajectory_stage(f"{prefix}/selected_loaded_plan", loaded_points),
        place,
        return_loaded,
    ]
    assert_continuous(stages)

    pregrasp_positions = approach_points[0]["positions"]
    joint_values = dict(zip(TRAJECTORY_JOINT_NAMES, pregrasp_positions))
    pregrasp_state = {
        "joints": {
            name: float(joint_values.get(name, 0.0))
            for name in JOINT_NAMES
        },
        "updown_m": float(pregrasp_positions[0]),
    }
    return {
        "schema_version": CACHE_SCHEMA_VERSION,
        "distance_cm": distance_cm,
        "lateral_offset_cm": lateral_offset_cm,
        "nominal_arm_spacing_cm": CACHE_NOMINAL_ARM_SPACING_CM,
        "row": row,
        "source_pair": list(pair),
        "strategy": "analytic_radial_to_loaded_v1",
        "pregrasp_state": pregrasp_state,
        "canonical_targets": canonical_targets(
            distance_cm / 100.0, lateral_offset_cm, row
        ),
        "snapshot": {
            "type": "trajectory_cache",
            "success": True,
            "box_front_x": distance_cm / 100.0,
            "scene_y_shift": lateral_offset_cm / 100.0,
            "left_box_id": pair[0],
            "right_box_id": pair[1],
            "replay_stages": stages,
        },
    }


def main() -> int:
    args = parse_args()
    place_templates = load_place_templates(args.template_cache_root.resolve())
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    generated = []
    failed = []
    attempted: dict[tuple[int, int, int], dict[str, Any]] = {}
    for source_root in (args.side_root.resolve(), args.top_root.resolve()):
        summary_path = source_root / "summary.json"
        if summary_path.is_file():
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            attempted.update(
                {
                    (
                        int(item["row"]),
                        int(round(float(item["front_x"]) * 100.0)),
                        int(round(float(item["scene_y_shift"]) * 100.0)),
                    ): item
                    for item in summary.get("rows", [])
                }
            )
        for result_path in sorted(source_root.glob("cases/*/result.json")):
            row, distance_cm, lateral_offset_cm = case_metadata(result_path)
            result = json.loads(result_path.read_text(encoding="utf-8"))
            if not result.get("loaded_transition", {}).get("valid", False):
                failed.append((row, distance_cm, lateral_offset_cm, "loaded_transition_invalid"))
                continue
            try:
                record = cache_record_from_result(
                    result,
                    row=row,
                    distance_cm=distance_cm,
                    lateral_offset_cm=lateral_offset_cm,
                    place_templates=place_templates,
                )
                generated.append(write_cache_record(record, output_root))
            except Exception as exc:
                failed.append((row, distance_cm, lateral_offset_cm, str(exc)))

    expected = 12 * 21 * 5
    generated_keys = {
        (
            int(path.name.rsplit("row_", 1)[1].split(".", 1)[0]),
            int(path.name.split("x_", 1)[1].split("cm", 1)[0]),
            int(path.name.split("_y_", 1)[1][1:].split("cm", 1)[0])
            * (-1 if "_y_m" in path.name else 1),
        )
        for path in generated
    }
    failed_keys = {(row, distance, offset) for row, distance, offset, _ in failed}
    for row in range(1, 6):
        for distance_cm in range(75, 87):
            for lateral_offset_cm in range(-10, 11):
                key = (row, distance_cm, lateral_offset_cm)
                if key in generated_keys or key in failed_keys:
                    continue
                item = attempted.get(key, {})
                reason = str(item.get("loaded_failure_reason", "case_result_missing"))
                if not item.get("ik_ok", False):
                    reason = "ik_failed:" + reason
                failed.append((*key, reason))
    rows = []
    for row in range(1, 6):
        count = sum(f"row_{row}" in path.name for path in generated)
        rows.append(f"| {row} | {count} | {12 * 21} | {count / (12 * 21):.1%} |")
    report = [
        "# 新径向策略轨迹缓存",
        "",
        "- 网格：12距离(75～86cm) × 21横移(-10～+10cm) × 5排。",
        "- 前三排：侧吸径向转正/缩短 + 分阶段 Shortcut。",
        "- 后两排：顶吸竖直工具轴 + 1cm水平回抽 + 第30步起 Shortcut。",
        f"- 成功缓存：{len(generated)}/{expected} = {len(generated) / expected:.2%}。",
        f"- 未生成：{len(failed)}。运行时继续使用原有最近成功缓存命中规则。",
        "",
        "| 排 | 成功缓存 | 总数 | 覆盖率 |",
        "|---:|---:|---:|---:|",
        *rows,
        "",
        "## 未生成明细",
        "",
    ]
    report.extend(
        f"- row{row} x={distance}cm y={offset:+d}cm: {reason}"
        for row, distance, offset, reason in failed
    )
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text("\n".join(report) + "\n", encoding="utf-8")
    print(f"generated={len(generated)}/{expected} failed={len(failed)} output={output_root}")
    return 0 if generated else 1


if __name__ == "__main__":
    raise SystemExit(main())
