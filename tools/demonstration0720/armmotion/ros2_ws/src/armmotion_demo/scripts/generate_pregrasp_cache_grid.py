#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import statistics
import time
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from typing import Any

from armmotion_demo.common import (
    parse_task_code,
    planning_task_from_suction_surface_poses,
    suction_surface_poses_for_task,
)
from armmotion_demo.planner_adapter import PlannerAdapter
from armmotion_demo.trajectory_cache import (
    CACHE_LATERAL_OFFSET_STEP_CM,
    CACHE_MAX_DISTANCE_CM,
    CACHE_MAX_LATERAL_OFFSET_CM,
    CACHE_MIN_DISTANCE_CM,
    CACHE_MIN_LATERAL_OFFSET_CM,
    CACHE_NOMINAL_ARM_SPACING_CM,
    cache_filename,
)
from generate_trajectory_cache import build_cache_record, write_cache_record


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="生成距离×横向偏移×五排预抓取后段轨迹缓存")
    parser.add_argument("--source-ws", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--cache-output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument(
        "--distance-cm",
        nargs="+",
        type=int,
        default=list(range(CACHE_MIN_DISTANCE_CM, CACHE_MAX_DISTANCE_CM + 1)),
    )
    parser.add_argument(
        "--lateral-offset-cm",
        nargs="+",
        type=int,
        default=list(
            range(
                CACHE_MIN_LATERAL_OFFSET_CM,
                CACHE_MAX_LATERAL_OFFSET_CM + 1,
                CACHE_LATERAL_OFFSET_STEP_CM,
            )
        ),
    )
    parser.add_argument("--rows", nargs="+", type=int, default=list(range(1, 6)))
    parser.add_argument("--spacing-cm", type=int, default=CACHE_NOMINAL_ARM_SPACING_CM)
    parser.add_argument("--attempts", type=int, default=3)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--retry-failures", action="store_true")
    return parser.parse_args()


def case_key(distance_cm: int, lateral_offset_cm: int, row: int) -> str:
    sign = "+" if lateral_offset_cm >= 0 else "-"
    return f"x{distance_cm:02d}_y{sign}{abs(lateral_offset_cm):02d}_row{row}"


def load_results(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def _aggregate(results: list[dict[str, Any]], field: str) -> dict[int, tuple[int, int]]:
    output: dict[int, tuple[int, int]] = {}
    for value in sorted({int(item[field]) for item in results}):
        subset = [item for item in results if int(item[field]) == value]
        output[value] = (sum(bool(item["success"]) for item in subset), len(subset))
    return output


def write_report(
    run_root: Path,
    results: list[dict[str, Any]],
    *,
    expected: int,
    cache_output: Path,
) -> None:
    ordered = sorted(
        results,
        key=lambda item: (
            int(item["row"]),
            int(item["lateral_offset_cm"]),
            int(item["distance_cm"]),
        ),
    )
    (run_root / "results.json").write_text(
        json.dumps(ordered, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    with (run_root / "results.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "case_key",
                "distance_cm",
                "lateral_offset_cm",
                "row",
                "success",
                "attempts_used",
                "wall_s",
                "cache_file",
                "failure_reason",
            ),
        )
        writer.writeheader()
        writer.writerows(
            {name: item.get(name, "") for name in writer.fieldnames}
            for item in ordered
        )

    success_count = sum(bool(item["success"]) for item in ordered)
    completed = len(ordered)
    durations = [float(item["wall_s"]) for item in ordered]
    rows = sorted({int(item["row"]) for item in ordered})
    distances = sorted({int(item["distance_cm"]) for item in ordered})
    offsets = sorted({int(item["lateral_offset_cm"]) for item in ordered})
    lines = [
        "# 82cm 双臂间距轨迹缓存生成结果",
        "",
        f"- 生成时间：{datetime.now().isoformat(timespec='seconds')}",
        f"- 参数空间：{len(rows)}排 × {len(offsets)}个横向偏移 × {len(distances)}个距离",
        f"- 计划任务：{expected}",
        f"- 已完成：{completed}",
        f"- 成功缓存：{success_count}",
        f"- 失败：{completed - success_count}",
        f"- 成功率：{success_count / completed:.2%}" if completed else "- 成功率：0.00%",
        f"- 缓存目录：`{cache_output}`",
        f"- 平均计算耗时：{statistics.fmean(durations):.3f}s" if durations else "- 平均计算耗时：无",
        "",
        "## 分组统计",
        "",
        "| 维度 | 值 | 成功 | 总数 | 成功率 |",
        "|---|---:|---:|---:|---:|",
    ]
    for label, field in (
        ("层", "row"),
        ("距离/cm", "distance_cm"),
        ("横向偏移/cm", "lateral_offset_cm"),
    ):
        for value, (success, total) in _aggregate(ordered, field).items():
            lines.append(f"| {label} | {value} | {success} | {total} | {success / total:.1%} |")

    for row in rows:
        lookup = {
            (int(item["lateral_offset_cm"]), int(item["distance_cm"])): item
            for item in ordered
            if int(item["row"]) == row
        }
        lines.extend(
            [
                "",
                f"## 第 {row} 排完整矩阵",
                "",
                "表格单元格：`成功(耗时s)` 或 `失败(阶段/原因摘要)`。",
                "",
                "| 横向偏移/cm | " + " | ".join(str(value) for value in distances) + " |",
                "|---:|" + "---:|" * len(distances),
            ]
        )
        for offset in offsets:
            cells = []
            for distance in distances:
                item = lookup.get((offset, distance))
                if item is None:
                    cells.append("未计算")
                elif item["success"]:
                    cells.append(f"成功({float(item['wall_s']):.2f})")
                else:
                    reason = str(item.get("failure_reason", "未知"))
                    cells.append("失败(" + reason[:30].replace("|", "/") + ")")
            lines.append(f"| {offset:+d} | " + " | ".join(cells) + " |")

    failures = [item for item in ordered if not item["success"]]
    lines.extend(["", "## 失败明细", ""])
    if not failures:
        lines.append("无。")
    else:
        lines.extend(
            [
                "| 任务 | 尝试次数 | 耗时/s | 原因 |",
                "|---|---:|---:|---|",
            ]
        )
        for item in failures:
            reason = str(item.get("failure_reason", "未知")).replace("|", "/")
            lines.append(
                f"| {item['case_key']} | {item['attempts_used']} | "
                f"{float(item['wall_s']):.3f} | {reason} |"
            )
    (run_root / "cache_generation_report.md").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    run_root = args.output_root.resolve()
    cache_output = args.cache_output.resolve()
    results_path = run_root / "results.ndjson"
    expected = len(args.distance_cm) * len(args.lateral_offset_cm) * len(args.rows)
    if args.spacing_cm != CACHE_NOMINAL_ARM_SPACING_CM:
        raise ValueError(
            f"当前缓存合同固定间距 {CACHE_NOMINAL_ARM_SPACING_CM}cm，收到 {args.spacing_cm}cm"
        )
    if args.attempts < 1:
        raise ValueError("--attempts 必须至少为1")
    if run_root.exists() and not args.resume:
        raise FileExistsError(f"输出目录已存在；若为续跑请加 --resume: {run_root}")
    run_root.mkdir(parents=True, exist_ok=True)
    cache_output.mkdir(parents=True, exist_ok=True)
    results = load_results(results_path) if args.resume else []
    previous_failures: dict[str, dict[str, Any]] = {}
    if args.retry_failures:
        if not args.resume:
            raise ValueError("--retry-failures 必须与 --resume 一起使用")
        previous_failures = {
            str(item["case_key"]): item
            for item in results
            if not item["success"]
        }
        results = [item for item in results if item["success"]]
        with results_path.open("w", encoding="utf-8") as stream:
            for item in results:
                stream.write(
                    json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n"
                )
    completed = {str(item["case_key"]) for item in results}
    half_spacing_m = args.spacing_cm / 200.0

    planner = PlannerAdapter(
        source_ws=args.source_ws.resolve(),
        output_root=run_root / "planner",
        rate_hz=30.0,
        max_joint_speed_deg_s=10.0,
        max_joint_acceleration_deg_s2=60.0,
        max_updown_speed_m_s=0.15,
        max_updown_acceleration_m_s2=0.05,
        speed_scale=3.0,
        timeout_s=args.timeout,
        trajectory_cache_enabled=False,
        trajectory_cache_required=False,
        trajectory_cache_fallback_on_planning_failure=False,
    )
    planner.start()
    try:
        for row in args.rows:
            for lateral_offset_cm in args.lateral_offset_cm:
                for distance_cm in args.distance_cm:
                    key = case_key(distance_cm, lateral_offset_cm, row)
                    if key in completed:
                        continue
                    fixture = parse_task_code(
                        f"B{row}",
                        distance_cm / 100.0,
                        distance_cm / 100.0,
                    )
                    left, right = suction_surface_poses_for_task(fixture)
                    offset_m = lateral_offset_cm / 100.0
                    left = replace(left, y=half_spacing_m + offset_m)
                    right = replace(right, y=-half_spacing_m + offset_m)
                    task = planning_task_from_suction_surface_poses(
                        key,
                        left,
                        right,
                        fixture.left_grasp_mode,
                        fixture.right_grasp_mode,
                    )
                    started = time.monotonic()
                    success = False
                    failure_reason = ""
                    cache_path: Path | None = None
                    attempts_used = 0
                    for attempt in range(1, args.attempts + 1):
                        attempts_used = attempt
                        try:
                            plan = planner.compute(task)
                            snapshot = json.loads(
                                plan.snapshot_path.read_text(encoding="utf-8")
                            )
                            record = build_cache_record(
                                snapshot,
                                distance_cm=distance_cm,
                                lateral_offset_cm=lateral_offset_cm,
                                row=row,
                            )
                            cache_path = write_cache_record(record, cache_output)
                            success = True
                            break
                        except Exception as exc:
                            failure_reason = str(exc)
                    result = {
                        "case_key": key,
                        "distance_cm": distance_cm,
                        "lateral_offset_cm": lateral_offset_cm,
                        "row": row,
                        "spacing_cm": args.spacing_cm,
                        "success": success,
                        "attempts_used": attempts_used,
                        "wall_s": time.monotonic() - started,
                        "cache_file": str(cache_path) if cache_path else "",
                        "failure_reason": failure_reason if not success else "",
                    }
                    previous = previous_failures.get(key)
                    if previous is not None:
                        result["attempts_used"] += int(previous.get("attempts_used", 0))
                        result["wall_s"] += float(previous.get("wall_s", 0.0))
                    with results_path.open("a", encoding="utf-8") as stream:
                        stream.write(
                            json.dumps(result, ensure_ascii=False, separators=(",", ":"))
                            + "\n"
                        )
                    results.append(result)
                    completed.add(key)
                    write_report(
                        run_root,
                        results,
                        expected=expected,
                        cache_output=cache_output,
                    )
                    status = "成功" if success else "失败"
                    print(
                        f"CACHE_GRID [{len(results)}/{expected}] {key} {status} "
                        f"attempts={attempts_used} wall={result['wall_s']:.3f}s",
                        flush=True,
                    )
    finally:
        planner.close()

    write_report(
        run_root,
        results,
        expected=expected,
        cache_output=cache_output,
    )
    requested_cache_paths = [
        cache_output / cache_filename(distance_cm, lateral_offset_cm, row)
        for row in args.rows
        for lateral_offset_cm in args.lateral_offset_cm
        for distance_cm in args.distance_cm
    ]
    actual_cache_count = sum(path.is_file() for path in requested_cache_paths)
    expected_successes = sum(bool(item["success"]) for item in results)
    if actual_cache_count != expected_successes:
        raise RuntimeError(
            f"缓存文件数不一致: files={actual_cache_count} success={expected_successes}"
        )
    print(f"缓存生成完成: {actual_cache_count}/{expected} -> {cache_output}", flush=True)
    print(f"结果文档: {run_root / 'cache_generation_report.md'}", flush=True)
    return 0 if actual_cache_count == expected else 1


if __name__ == "__main__":
    raise SystemExit(main())
