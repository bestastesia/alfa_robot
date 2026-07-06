#!/usr/bin/env python3
"""Refine six boundary regions from an IK range CSV."""

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


def parse_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def load_success_rows(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        required = {"x", "y", "z", "is_success"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise SystemExit(f"CSV missing required columns: {sorted(missing)}")
        rows = []
        for row in reader:
            if parse_bool(row["is_success"]):
                rows.append({axis: float(row[axis]) for axis in AXES})
    if not rows:
        raise SystemExit("CSV has no successful IK rows")
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


def ik_range_command(args: argparse.Namespace, region: EdgeRegion) -> list[str]:
    command = [
        sys.executable,
        str(args.range_script),
        "--version",
        args.version,
        "--group",
        args.group,
        "--solver",
        args.solver,
        "--base-frame",
        args.base_frame,
        "--tip-link",
        args.tip_link,
        "--timeout",
        str(args.timeout),
        "--forward-axis",
        args.forward_axis,
        "--target-axis",
        args.target_axis,
        "--spin-samples",
        str(args.spin_samples),
        "--spin-min",
        str(args.spin_min),
        "--spin-max",
        str(args.spin_max),
        "--output",
        str(region.output_csv),
    ]
    if args.urdf is not None:
        command.extend(["--urdf", str(args.urdf)])
    if args.srdf is not None:
        command.extend(["--srdf", str(args.srdf)])
    if args.success_only:
        command.append("--success-only")
    if args.fix_joint6:
        command.append("--fix-joint6")
    for axis in AXES:
        low, high, step = region.ranges[axis]
        command.extend([f"--{axis}", str(low), str(high), str(step)])
    return command


def manifest_entry(args: argparse.Namespace, region: EdgeRegion) -> dict:
    command = ik_range_command(args, region)
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
        "output_csv": str(region.output_csv),
        "command": command,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Build and optionally run six IK edge refinement grids from a coarse CSV")
    parser.add_argument("--input", type=Path, required=True, help="Coarse ik_range_grid CSV")
    parser.add_argument("--output-dir", type=Path, default=Path("data/ik_range/refined_edges"))
    parser.add_argument("--range-script", type=Path, default=Path(__file__).with_name("ik_range_grid.py"))
    parser.add_argument("--edge-width", type=float, default=0.1)
    parser.add_argument("--cross-half-width", type=float, default=0.1)
    parser.add_argument("--step", type=float, default=0.01)
    parser.add_argument("--tolerance", type=float, default=1e-9)
    parser.add_argument("--run", action="store_true", help="Run ik_range_grid for all six regions")
    parser.add_argument("--version", choices=["current", "v2", "v3", "v4"], default="current")
    parser.add_argument("--group", default="left_arm")
    parser.add_argument("--solver", default="kdl")
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument("--srdf", type=Path, default=None)
    parser.add_argument("--base-frame", default="left_arm_base")
    parser.add_argument("--tip-link", default="left_tool0")
    parser.add_argument("--timeout", type=float, default=0.02)
    parser.add_argument("--forward-axis", default="z")
    parser.add_argument("--target-axis", default="x")
    parser.add_argument("--spin-samples", type=int, default=12)
    parser.add_argument("--spin-min", type=float, default=0.0)
    parser.add_argument("--spin-max", type=float, default=2.0 * math.pi)
    parser.add_argument("--success-only", action="store_true")
    parser.add_argument("--fix-joint6", action="store_true")
    args = parser.parse_args()

    rows = load_success_rows(args.input)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    regions = [
        build_region(rows, axis, direction, args.edge_width, args.cross_half_width, args.step, args.output_dir, args.tolerance)
        for axis in AXES
        for direction in ("max", "min")
    ]
    manifest = {
        "input": str(args.input),
        "success_rows": len(rows),
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
            f"x={ranges['x']['min']}..{ranges['x']['max']}, "
            f"y={ranges['y']['min']}..{ranges['y']['max']}, "
            f"z={ranges['z']['min']}..{ranges['z']['max']}"
        )

    if args.run:
        for region in regions:
            command = ik_range_command(args, region)
            print("Running:")
            print(" ".join(command))
            subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
