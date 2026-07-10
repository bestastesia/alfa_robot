#!/usr/bin/env python3
"""Visualize 9-orientation reachability as directional reachability classes.

Collision is treated as failure. Center-orientation failure is split into
center rejected (IK exists but collision-checked IK is rejected) and center
solve failed (IK does not exist even without collision checking).
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import rerun as rr

from alfa_robot_rerun.visualize_rerun import UrdfRobot, log_robot_static_model, log_robot_state, render_current_urdf


def point_key(row: dict[str, str]) -> tuple[float, float, float]:
    return (round(float(row["x"]), 6), round(float(row["y"]), 6), round(float(row["z"]), 6))


def classify_points(csv_path: Path) -> tuple[dict[str, list[tuple[float, float, float]]], Counter[str]]:
    grouped: dict[tuple[float, float, float], list[dict[str, str]]] = defaultdict(list)
    with csv_path.open(newline="") as file:
        reader = csv.DictReader(file)
        for row in reader:
            if row.get("orient_label") == "SUMMARY":
                continue
            grouped[point_key(row)].append(row)

    classes: dict[str, list[tuple[float, float, float]]] = {
        "all_9": [],
        "center_and_majority": [],
        "center_only_minority": [],
        "center_rejected": [],
        "center_solve_failed": [],
    }
    reason_counts: Counter[str] = Counter()
    for point, rows in grouped.items():
        success_count = sum(1 for row in rows if row.get("reason") == "success" or row.get("success") == "1")
        center_rows = [row for row in rows if row.get("orient_label") == "center" or row.get("orient_idx") == "4"]
        center_reason = center_rows[0].get("reason", "") if center_rows else "missing_center"
        center_success = bool(center_rows) and (
            center_reason == "success" or center_rows[0].get("success") == "1")
        for row in rows:
            if row.get("reason") != "success" and row.get("success") != "1":
                reason_counts[row.get("reason") or "unknown"] += 1

        if success_count == len(rows) and rows:
            classes["all_9"].append(point)
        elif center_success and success_count > len(rows) / 2:
            classes["center_and_majority"].append(point)
        elif center_success:
            classes["center_only_minority"].append(point)
        elif center_reason == "collision":
            classes["center_rejected"].append(point)
        else:
            classes["center_solve_failed"].append(point)
    return classes, reason_counts


def log_points(path: str, points: list[tuple[float, float, float]], color: list[int],
               radius: float, label: str) -> None:
    if not points:
        return
    rr.log(
        path,
        rr.Points3D(
            positions=np.asarray(points, dtype=np.float32),
            colors=[color],
            radii=radius,
            labels=None,
        ),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize nine-orient CSV as directional reachability classes")
    parser.add_argument("csv", type=Path)
    parser.add_argument("--save", type=Path, default=None)
    parser.add_argument("--connect", action="store_true")
    parser.add_argument("--no-robot", action="store_true")
    parser.add_argument("--no-meshes", action="store_true")
    parser.add_argument("--robot-path", default="world/robot")
    args = parser.parse_args()

    classes, reason_counts = classify_points(args.csv)

    rr.init("nine_orient_three_class", recording_id=args.csv.stem)
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        rr.save(str(args.save))
    elif args.connect:
        rr.connect()
    else:
        rr.spawn()

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    if not args.no_robot:
        robot = UrdfRobot(render_current_urdf())
        log_robot_static_model(robot, args.robot_path, log_meshes=not args.no_meshes)
        log_robot_state(robot, {}, args.robot_path)

    log_points("world/reachability/01_九向全可达", classes["all_9"], [0, 180, 80], 0.013, "all_9")
    log_points("world/reachability/02_中心可达且半数以上", classes["center_and_majority"], [0, 130, 255], 0.011, "center_and_majority")
    log_points("world/reachability/03_中心可达但半数以下", classes["center_only_minority"], [255, 180, 0], 0.010, "center_only_minority")
    log_points("world/reachability/04_中心被拒绝", classes["center_rejected"], [255, 80, 0], 0.008, "center_rejected")
    log_points("world/reachability/05_中心求解失败", classes["center_solve_failed"], [130, 130, 130], 0.006, "center_solve_failed")

    summary = [
        f"csv: {args.csv}",
        f"九向全可达: {len(classes['all_9'])}",
        f"中心可达且半数以上: {len(classes['center_and_majority'])}",
        f"中心可达但半数以下: {len(classes['center_only_minority'])}",
        f"中心被拒绝: {len(classes['center_rejected'])}",
        f"中心求解失败: {len(classes['center_solve_failed'])}",
        "颜色: 绿=九向全可达, 蓝=中心可达且半数以上, 黄=中心可达但半数以下, 红=中心被拒绝, 灰=中心求解失败",
        f"失败原因统计: {dict(reason_counts)}",
    ]
    rr.log("summary", rr.TextLog("\n".join(summary)))
    print("\n".join(summary))
    if args.save:
        print(f"saved: {args.save}")


if __name__ == "__main__":
    main()
