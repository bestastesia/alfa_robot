#!/usr/bin/env python3
"""Refine six boundary regions from a 9-orientation reachability CSV."""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


AXES = ("x", "y", "z")


@dataclass(frozen=True)
class EdgeRegion:
    name: str
    axis: str
    direction: str
    boundary_value: float
    boundary_count: int
    boundary_other_stats: dict[str, dict[str, float]]
    ranges: dict[str, tuple[float, float, float]]
    sample_count: int
    output_csv: Path


def parse_bool(value: str | None) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def is_summary_reachable(row: dict[str, str]) -> bool:
    if row.get("orient_label") != "SUMMARY":
        return False
    point_reachable = row.get("point_reachable")
    if point_reachable not in (None, ""):
        return parse_bool(point_reachable)
    return row.get("n_success") not in (None, "") and row.get("n_success") == row.get("n_total")


def load_reachable_summary_rows(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        required = {"x", "y", "z", "orient_label"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise SystemExit(f"CSV missing required columns: {sorted(missing)}")
        if "point_reachable" not in set(reader.fieldnames or []) and not {"n_success", "n_total"}.issubset(set(reader.fieldnames or [])):
            raise SystemExit("CSV must contain point_reachable or n_success/n_total")

        rows = []
        for row in reader:
            if is_summary_reachable(row):
                rows.append({axis: float(row[axis]) for axis in AXES})

    if not rows:
        raise SystemExit("CSV has no reachable SUMMARY rows")
    return rows


def count_samples(min_value: float, max_value: float, step: float) -> int:
    if step <= 0.0:
        raise ValueError("step must be positive")
    return int(math.floor((max_value - min_value) / step + 1e-9)) + 1


def round_range(value: float) -> float:
    return round(value, 6)


def stats(values: list[float]) -> dict[str, float]:
    return {
        "min": min(values),
        "max": max(values),
        "mean": sum(values) / len(values),
    }


def build_region(
    rows: list[dict[str, float]],
    axis: str,
    direction: str,
    edge_width: float,
    cross_half_width: float,
    step: float,
    output_dir: Path,
    tolerance: float,
) -> EdgeRegion:
    values = [row[axis] for row in rows]
    boundary = max(values) if direction == "max" else min(values)
    boundary_rows = [row for row in rows if abs(row[axis] - boundary) <= tolerance]
    if not boundary_rows:
        raise RuntimeError(f"no boundary rows for {axis}_{direction}")

    ranges: dict[str, tuple[float, float, float]] = {}
    other_stats: dict[str, dict[str, float]] = {}
    for current_axis in AXES:
        if current_axis == axis:
            if direction == "max":
                low, high = boundary, boundary + edge_width
            else:
                low, high = boundary - edge_width, boundary
        else:
            axis_stats = stats([row[current_axis] for row in boundary_rows])
            other_stats[current_axis] = axis_stats
            center = axis_stats["mean"]
            low, high = center - cross_half_width, center + cross_half_width
        ranges[current_axis] = (round_range(low), round_range(high), step)

    sample_count = 1
    for low, high, axis_step in ranges.values():
        sample_count *= count_samples(low, high, axis_step)

    name = f"{axis}_{direction}"
    return EdgeRegion(
        name=name,
        axis=axis,
        direction=direction,
        boundary_value=boundary,
        boundary_count=len(boundary_rows),
        boundary_other_stats=other_stats,
        ranges=ranges,
        sample_count=sample_count,
        output_csv=output_dir / f"{name}.csv",
    )


def nine_orient_command(args: argparse.Namespace, region: EdgeRegion) -> list[str]:
    script = args.nine_orient_script
    command = [
        sys.executable,
        str(script),
        "--ros-args",
        "-p", f"side:={args.side}",
        "-p", f"reference_frame:={args.reference_frame}",
        "-p", f"output_csv:={region.output_csv}",
        "-p", f"append:=false",
        "-p", f"avoid_collisions:={str(args.avoid_collisions).lower()}",
        "-p", f"ik_timeout:={args.ik_timeout}",
        "-p", f"service_timeout:={args.service_timeout}",
        "-p", f"seed_wait_sec:={args.seed_wait_sec}",
        "-p", f"yaw_pitch_delta:={args.yaw_pitch_delta}",
    ]

    if args.center_orientation_xyzw:
        command.extend(["-p", f"center_orientation_xyzw:={args.center_orientation_xyzw}"])

    if args.side == "left":
        command.extend([
            "-p", f"left_group:={args.group}",
            "-p", f"left_tip:={args.tip}",
        ])
    elif args.side == "right":
        command.extend([
            "-p", f"right_group:={args.group}",
            "-p", f"right_tip:={args.tip}",
        ])
    else:
        raise ValueError(f"unsupported side: {args.side}")

    for axis in AXES:
        low, high, step = region.ranges[axis]
        command.extend(["-p", f"min_{axis}:={low}", "-p", f"max_{axis}:={high}"])
    command.extend(["-p", f"step:={args.step}"])
    return command


def manifest_entry(args: argparse.Namespace, region: EdgeRegion) -> dict:
    command = nine_orient_command(args, region)
    return {
        "name": region.name,
        "axis": region.axis,
        "direction": region.direction,
        "boundary_value": region.boundary_value,
        "boundary_count": region.boundary_count,
        "boundary_other_stats": region.boundary_other_stats,
        "ranges": {
            axis: {"min": low, "max": high, "step": step}
            for axis, (low, high, step) in region.ranges.items()
        },
        "sample_count": region.sample_count,
        "ik_call_count": region.sample_count * 9,
        "output_csv": str(region.output_csv),
        "command": command,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Build and optionally run six 9-orientation edge refinement grids")
    parser.add_argument("--input", type=Path, required=True, help="Coarse nine_orient_reachability CSV")
    parser.add_argument("--output-dir", type=Path, default=Path("data/ik_range/nine_orient_refined_edges"))
    parser.add_argument("--nine-orient-script", type=Path, default=Path("ros2_ws/src/alfa_robot_moveit_config/scripts/nine_orient_reachability.py"))
    parser.add_argument("--edge-width", type=float, default=0.1)
    parser.add_argument("--cross-half-width", type=float, default=0.1)
    parser.add_argument("--step", type=float, default=0.01)
    parser.add_argument("--tolerance", type=float, default=1e-9)
    parser.add_argument("--run", action="store_true", help="Run nine_orient_reachability for all six regions")
    parser.add_argument("--side", choices=["left", "right"], default="left")
    parser.add_argument("--reference-frame", default="left_arm_base")
    parser.add_argument("--group", default="left_arm")
    parser.add_argument("--tip", default="left_tool0")
    parser.add_argument("--avoid-collisions", action="store_true")
    parser.add_argument("--ik-timeout", type=float, default=0.05)
    parser.add_argument("--service-timeout", type=float, default=10.0)
    parser.add_argument("--seed-wait-sec", type=float, default=2.0)
    parser.add_argument("--yaw-pitch-delta", type=float, default=15.0)
    parser.add_argument("--center-orientation-xyzw", default="")
    args = parser.parse_args()

    rows = load_reachable_summary_rows(args.input)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    regions = [
        build_region(rows, axis, direction, args.edge_width, args.cross_half_width, args.step, args.output_dir, args.tolerance)
        for axis in AXES
        for direction in ("max", "min")
    ]
    manifest = {
        "input": str(args.input),
        "reachable_summary_rows": len(rows),
        "edge_width": args.edge_width,
        "cross_half_width": args.cross_half_width,
        "step": args.step,
        "regions": [manifest_entry(args, region) for region in regions],
    }
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"Wrote {manifest_path}")
    for entry in manifest["regions"]:
        ranges = entry["ranges"]
        print(
            f"{entry['name']}: boundary={entry['boundary_value']:.6f}, "
            f"boundary_points={entry['boundary_count']}, samples={entry['sample_count']}, "
            f"ik_calls={entry['ik_call_count']}, "
            f"x={ranges['x']['min']}..{ranges['x']['max']}, "
            f"y={ranges['y']['min']}..{ranges['y']['max']}, "
            f"z={ranges['z']['min']}..{ranges['z']['max']}"
        )

    if args.run:
        for region in regions:
            command = nine_orient_command(args, region)
            print("Running:")
            print(" ".join(command))
            subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
