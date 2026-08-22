#!/usr/bin/env python3
"""Render analytic orientation reachability as an RGB point cloud."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import rerun as rr

from alfa_robot_rerun.visualize_rerun import (
    UrdfRobot,
    log_robot_state,
    log_robot_static_model,
    render_current_urdf,
)


def load_rows(csv_path: Path) -> tuple[np.ndarray, np.ndarray, list[dict[str, str]]]:
    with csv_path.open(newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))
    if not rows:
        raise SystemExit(f"CSV contains no points: {csv_path}")
    positions = np.asarray(
        [[float(row["x"]), float(row["y"]), float(row["z"])] for row in rows],
        dtype=np.float32,
    )
    colors = np.asarray(
        [[int(row["r"]), int(row["g"]), int(row["b"])] for row in rows],
        dtype=np.uint8,
    )
    zero_mask = np.all(colors == 0, axis=1)
    colors[zero_mask] = [24, 24, 24]
    return positions, colors, rows


def inclusive_angles(lower: float, upper: float, step: float) -> list[float]:
    lower, upper = sorted((lower, upper))
    count = math.floor((upper - lower) / step + 1e-10)
    values = [lower + index * step for index in range(count + 1)]
    if upper - values[-1] > 1e-9:
        values.append(upper)
    else:
        values[-1] = upper
    return values


def format_angles(values: list[float]) -> str:
    return ",".join(f"{value:g}" for value in values)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--save", type=Path, default=None)
    parser.add_argument("--connect", action="store_true")
    parser.add_argument("--point-radius", type=float, default=0.009)
    parser.add_argument("--robot-opacity", type=float, default=0.35)
    parser.add_argument("--updown", type=float, default=0.45)
    parser.add_argument("--no-meshes", action="store_true")
    args = parser.parse_args()
    if not 0.0 <= args.robot_opacity <= 1.0:
        parser.error("--robot-opacity must be within [0, 1]")

    positions, colors, rows = load_rows(args.csv)
    rr.init("analytic_orientation_rgb_reachability", recording_id=args.csv.stem)
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        rr.save(str(args.save))
    elif args.connect:
        rr.connect()
    else:
        rr.spawn()

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    robot = UrdfRobot(render_current_urdf())
    robot_alpha = round(255 * args.robot_opacity)
    log_robot_static_model(
        robot,
        "world/robot",
        log_meshes=not args.no_meshes,
        mesh_albedo_factor=[255, 255, 255, robot_alpha],
    )
    log_robot_state(robot, {"updown": args.updown}, "world/robot")

    mins = positions.min(axis=0)
    maxs = positions.max(axis=0)
    rr.log(
        "world/reachability/sample_bounds",
        rr.Boxes3D(
            centers=[(mins + maxs) * 0.5],
            half_sizes=[(maxs - mins) * 0.5],
            colors=[[255, 255, 255, 20]],
            fill_mode=rr.components.FillMode.TransparentFillMajorWireframe,
        ),
    )
    rr.log(
        "world/reachability/orientation_rgb",
        rr.Points3D(positions, colors=colors, radii=args.point_radius),
    )

    total_orientations = sum(int(row["total_orientations"]) for row in rows)
    total_successes = sum(int(row["total_success"]) for row in rows)
    any_reachable = sum(int(row["total_success"]) > 0 for row in rows)
    all_reachable = sum(
        int(row["total_success"]) == int(row["total_orientations"]) for row in rows
    )
    zero_reachable = len(rows) - any_reachable
    channel_means = colors.astype(float).mean(axis=0)
    min_angle_deg = float(rows[0].get("min_angle_deg", 0.0))
    max_angle_deg = float(rows[0].get("max_angle_deg", 90.0))
    angle_step_deg = float(rows[0].get("angle_step_deg", 5.0))
    angle_values = inclusive_angles(min_angle_deg, max_angle_deg, angle_step_deg)
    lower = min(min_angle_deg, max_angle_deg)
    upper = max(min_angle_deg, max_angle_deg)
    span = upper - lower
    band_angles = [
        [value for value in angle_values if value <= lower + span / 3.0 + 1e-9],
        [
            value
            for value in angle_values
            if lower + span / 3.0 + 1e-9 < value <= lower + 2.0 * span / 3.0 + 1e-9
        ],
        [value for value in angle_values if value > lower + 2.0 * span / 3.0 + 1e-9],
    ]
    joint_sign_filter_enabled = "joint_sign_rejected_orientations" in rows[0]
    raw_reachable = sum(int(row.get("raw_reachable_orientations", 0)) for row in rows)
    joint_sign_rejected = sum(
        int(row.get("joint_sign_rejected_orientations", 0)) for row in rows
    )
    joint_filter_summary = ""
    if joint_sign_filter_enabled:
        joint_filter_summary = (
            "- 解析分支硬约束：`joint3 > 0` 且 `joint4 < 0`\n"
            f"- 原始有解姿态：`{raw_reachable:,}`；被关节符号约束拒绝："
            f"`{joint_sign_rejected:,}`\n"
        )
    summary = (
        "# 右臂解析 IK 姿态可达性 RGB 点云\n\n"
        "- 姿态约定：工具法向 `-90°=-Z向下`、`0°=+X向前`、`+90°=+Z向上`\n"
        f"- 本次范围：`{min_angle_deg:g}～{max_angle_deg:g}°`，分辨率："
        f"`{angle_step_deg:g}°`，共`{len(angle_values)}`个姿态\n"
        f"- 红色：`{format_angles(band_angles[0])}°` 成功比例\n"
        f"- 绿色：`{format_angles(band_angles[1])}°` 成功比例\n"
        f"- 蓝色：`{format_angles(band_angles[2])}°` 成功比例\n"
        "- 每通道全可达=`255`；RGB全零点用深灰显示，避免黑色背景不可见\n"
        f"- 点数：`{len(rows):,}`\n"
        f"- IK成功：`{total_successes:,}/{total_orientations:,}`\n"
        f"- 至少一个姿态可达：`{any_reachable:,}`\n"
        f"- {len(angle_values)}姿态全部可达：`{all_reachable:,}`\n"
        f"- 全部不可达：`{zero_reachable:,}`\n"
        f"- 平均显示RGB：`({channel_means[0]:.1f}, {channel_means[1]:.1f}, {channel_means[2]:.1f})`\n"
        f"{joint_filter_summary}"
        "- 口径：纯解析IK与关节限位，不包含MoveIt碰撞过滤"
    )
    rr.log("world/reachability/legend", rr.TextDocument(summary))
    print(summary.replace("`", ""))
    if args.save:
        print(f"Rerun: {args.save}")


if __name__ == "__main__":
    main()
