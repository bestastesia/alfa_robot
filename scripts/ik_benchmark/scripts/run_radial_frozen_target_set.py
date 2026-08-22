#!/usr/bin/env python3
"""Re-run a recorded radial-extraction target set against frozen IK snapshots."""
from __future__ import annotations

import argparse
import csv
import gzip
import json
import os
import subprocess
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--snapshot-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=50)
    parser.add_argument("--target-radius", type=float, default=0.79)
    parser.add_argument("--orientation-only-step-deg", type=float, default=2.0)
    parser.add_argument(
        "--shortcut-start-radial-rotation-deg", type=float, default=30.0
    )
    parser.add_argument("--resume-radial-after-orientation", action="store_true")
    parser.add_argument("--interleave-loaded-shortcuts", action="store_true")
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    manifest = json.loads(args.manifest.resolve().read_text())
    snapshot_root = args.snapshot_root.resolve()
    output = args.output.resolve()
    cases_dir = output / "cases"
    cases_dir.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    rows: list[dict[str, object]] = []
    started = time.monotonic()

    for index, baseline in enumerate(manifest["cases"], 1):
        case = str(baseline["case"])
        case_dir = cases_dir / case
        case_dir.mkdir(parents=True, exist_ok=True)
        source_snapshot = snapshot_root / case / "snapshot.json.gz"
        snapshot = case_dir / "snapshot.json"
        result_path = case_dir / "result.json"
        with gzip.open(source_snapshot, "rb") as source, snapshot.open("wb") as target:
            target.write(source.read())
        case_started = time.monotonic()
        run = subprocess.run(
            [
                "ros2", "launch", "alfa_robot_benchmarks",
                "analytic_radial_extract_prototype.launch.py",
                f"snapshot:={snapshot}",
                f"output:={result_path}",
                f"frames:={args.frames}",
                f"target_radius:={args.target_radius}",
                f"orientation_only_step_deg:={args.orientation_only_step_deg}",
                "shortcut_start_radial_rotation_deg:="
                f"{args.shortcut_start_radial_rotation_deg}",
                "resume_radial_after_orientation:="
                f"{'true' if args.resume_radial_after_orientation else 'false'}",
                "interleave_loaded_shortcuts:="
                f"{'true' if args.interleave_loaded_shortcuts else 'false'}",
            ],
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=args.timeout,
            check=False,
        )
        case_wall_ms = (time.monotonic() - case_started) * 1000.0
        result = json.loads(result_path.read_text())
        orientation = result.get("orientation_only_transition", {})
        resumed = result.get("radial_resume_transition", {})
        loaded = result.get("loaded_transition", {})
        baseline_success = str(baseline["baseline_outcome"]) == "success"
        rows.append(
            {
                "target_id": baseline["target_id"],
                "case": case,
                "row": baseline["row"],
                "front_x": baseline["front_x"],
                "scene_y_shift": baseline["scene_y_shift"],
                "baseline_success": baseline_success,
                "new_success": bool(loaded.get("valid", False)),
                "orientation_valid_steps": int(orientation.get("valid_steps", 0)),
                "orientation_steps_requested": int(orientation.get("steps_requested", 0)),
                "orientation_complete": bool(orientation.get("complete", False)),
                "orientation_used": bool(
                    orientation.get("used_for_loaded_transition", False)
                ),
                "resume_valid_steps": int(resumed.get("valid_steps", 0)),
                "resume_steps_requested": int(resumed.get("steps_requested", 0)),
                "resume_complete": bool(resumed.get("complete", False)),
                "resume_used": bool(resumed.get("used_for_loaded_transition", False)),
                "loaded_start_phase": str(loaded.get("start_phase", "")),
                "loaded_method": str(loaded.get("method", "")),
                "loaded_failure_reason": str(loaded.get("failure_reason", "")),
                "shortcut_attempt_count": len(loaded.get("shortcut_attempts", [])),
                "shortcut_check_ms": float(loaded.get("shortcut_check_ms", 0.0)),
                "strategy_total_ms": float(result.get("strategy_total_ms", 0.0)),
                "process_wall_ms": case_wall_ms,
                "prototype_exit_code": run.returncode,
                "result_path": str(result_path),
            }
        )
        with snapshot.open("rb") as source, gzip.open(
            snapshot.with_suffix(".json.gz"), "wb"
        ) as target:
            target.write(source.read())
        snapshot.unlink()
        print(
            f"progress={index}/{len(manifest['cases'])} "
            f"success={sum(bool(row['new_success']) for row in rows)}",
            flush=True,
        )

    elapsed_s = time.monotonic() - started
    retained = sum(row["baseline_success"] and row["new_success"] for row in rows)
    recovered = sum(not row["baseline_success"] and row["new_success"] for row in rows)
    regressed = sum(row["baseline_success"] and not row["new_success"] for row in rows)
    still_failed = sum(not row["baseline_success"] and not row["new_success"] for row in rows)
    baseline_count = sum(bool(row["baseline_success"]) for row in rows)
    success_count = sum(bool(row["new_success"]) for row in rows)
    summary = {
        "source_manifest": str(args.manifest.resolve()),
        "frozen_snapshot_source": str(snapshot_root),
        "resume_radial_after_orientation": args.resume_radial_after_orientation,
        "interleave_loaded_shortcuts": args.interleave_loaded_shortcuts,
        "shortcut_start_radial_rotation_deg":
        args.shortcut_start_radial_rotation_deg,
        "elapsed_s": elapsed_s,
        "baseline_success_count": baseline_count,
        "success_count": success_count,
        "retained": retained,
        "recovered": recovered,
        "regressed": regressed,
        "still_failed": still_failed,
        "rows": rows,
    }
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n"
    )
    with (output / "summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    shortcut_check_times = [
        float(attempt.get("check_ms", 0.0))
        for row in rows
        for attempt in json.loads(Path(str(row["result_path"])).read_text())
        .get("loaded_transition", {})
        .get("shortcut_attempts", [])
    ]
    lines = [
        "# 定点转腕与径向续推方案冻结快照A/B测试",
        "",
        f"- 测试组：{len(rows)}组。",
        "- 输入：复用原始测试保存的同一IK snapshot，不重新选IK。",
        f"- 径向Shortcut同步检测阈值：双臂径向连线累计转动达到{args.shortcut_start_radial_rotation_deg:.1f}°后开始；若此前已失败，保留最后合法径向帧兜底。",
        f"- 基线：{baseline_count}/{len(rows)} = {100.0 * baseline_count / len(rows):.2f}%",
        f"- 新方案：{success_count}/{len(rows)} = {100.0 * success_count / len(rows):.2f}%",
        f"- 保持成功：{retained}；新增恢复：{recovered}；回归失败：{regressed}；仍失败：{still_failed}。",
        f"- 墙钟：{elapsed_s:.1f}s。",
        f"- 策略最快/最慢：{min(float(row['strategy_total_ms']) for row in rows):.3f}ms / "
        f"{max(float(row['strategy_total_ms']) for row in rows):.3f}ms。",
        (
            f"- 单次Shortcut检查最快/最慢：{min(shortcut_check_times):.3f}ms / "
            f"{max(shortcut_check_times):.3f}ms。"
            if shortcut_check_times
            else "- 未执行逐步Shortcut检查。"
        ),
        "",
        "| ID | 案例 | 基线 | 新方案 | 转腕 | 径向续推 | 接管起点 | 方法 |",
        "|---|---|---|---|---:|---:|---|---|",
    ]
    for row in rows:
        lines.append(
            f"| {row['target_id']} | `{row['case']}` | "
            f"{'成功' if row['baseline_success'] else '失败'} | "
            f"{'成功' if row['new_success'] else '失败'} | "
            f"{row['orientation_valid_steps']}/{row['orientation_steps_requested']} | "
            f"{row['resume_valid_steps']}/{row['resume_steps_requested']} | "
            f"`{row['loaded_start_phase']}` | `{row['loaded_method']}` |"
        )
    (output / "README.md").write_text("\n".join(lines) + "\n")
    print(output / "README.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
