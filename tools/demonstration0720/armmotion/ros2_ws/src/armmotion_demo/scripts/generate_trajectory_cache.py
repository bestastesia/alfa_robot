#!/usr/bin/env python3
from __future__ import annotations

import argparse
import gzip
import json
import math
from pathlib import Path

from armmotion_demo.trajectory_cache import (
    CACHE_NOMINAL_ARM_SPACING_CM,
    CACHE_SCHEMA_VERSION,
    cache_filename,
)


TASK_ROWS = {
    (1, 3): 1,
    (4, 6): 2,
    (7, 9): 3,
    (10, 12): 4,
    (13, 15): 5,
}
JOINT_NAMES = (
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
    "turn",
)


def slim_stage(stage: dict) -> dict:
    return {
        "stage": stage.get("stage", ""),
        "trajectory": stage.get("trajectory", {}),
    }


WORLD_TO_BASE_Z_M = 0.202094
BOTTOM_ROW_CENTER_WORLD_Z_M = 0.21
ROW_PITCH_M = 0.41
BOX_DEPTH_M = 0.30
BOX_HEIGHT_M = 0.40
SIDE_RPY = (math.pi, -math.pi / 2.0, 0.0)
TOP_RPY = (math.pi, 0.0, 0.0)


def pregrasp_state(snapshot: dict) -> dict:
    stages = snapshot.get("replay_stages", [])
    if not stages:
        raise ValueError("snapshot replay_stages 为空")
    trajectory = stages[0].get("trajectory", {})
    names = trajectory.get("joint_names", [])
    points = trajectory.get("points", [])
    if not points:
        raise ValueError("snapshot 首段轨迹为空")
    positions = points[-1].get("positions", [])
    values = dict(zip(names, positions))
    return {
        "joints": {name: float(values.get(name, 0.0)) for name in JOINT_NAMES},
        "updown_m": float(values.get("updown", 0.3)),
    }


def pose_dict(x: float, y: float, z: float, rpy: tuple[float, float, float]) -> dict:
    return {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "roll": float(rpy[0]),
        "pitch": float(rpy[1]),
        "yaw": float(rpy[2]),
    }


def canonical_targets(
    distance_m: float,
    lateral_offset_cm: int,
    row: int,
) -> dict:
    center_z = (
        BOTTOM_ROW_CENTER_WORLD_Z_M
        - WORLD_TO_BASE_Z_M
        + (5 - row) * ROW_PITCH_M
    )
    mode = "front" if row <= 2 else "top_suction"
    if mode == "front":
        x = distance_m
        z = center_z
        rpy = SIDE_RPY
    else:
        x = distance_m + 0.5 * BOX_DEPTH_M
        z = center_z + 0.5 * BOX_HEIGHT_M
        rpy = TOP_RPY
    half_spacing_m = CACHE_NOMINAL_ARM_SPACING_CM / 200.0
    lateral_offset_m = lateral_offset_cm / 100.0
    return {
        "left": {
            "pose_6d": pose_dict(x, half_spacing_m + lateral_offset_m, z, rpy),
            "grasp_mode": mode,
        },
        "right": {
            "pose_6d": pose_dict(x, -half_spacing_m + lateral_offset_m, z, rpy),
            "grasp_mode": mode,
        },
    }


def build_cache_record(
    snapshot: dict,
    *,
    distance_cm: int,
    lateral_offset_cm: int,
    row: int,
) -> dict:
    pair = next((pair for pair, pair_row in TASK_ROWS.items() if pair_row == row), None)
    if pair is None:
        raise ValueError(f"未知任务层: {row}")
    return {
        "schema_version": CACHE_SCHEMA_VERSION,
        "distance_cm": distance_cm,
        "lateral_offset_cm": lateral_offset_cm,
        "nominal_arm_spacing_cm": CACHE_NOMINAL_ARM_SPACING_CM,
        "row": row,
        "source_pair": list(pair),
        "pregrasp_state": pregrasp_state(snapshot),
        "canonical_targets": canonical_targets(
            distance_cm / 100.0,
            lateral_offset_cm,
            row,
        ),
        "snapshot": {
            "type": "trajectory_cache",
            "success": True,
            "box_front_x": float(snapshot["box_front_x"]),
            "scene_y_shift": lateral_offset_cm / 100.0,
            "left_box_id": pair[0],
            "right_box_id": pair[1],
            "replay_stages": [
                slim_stage(stage) for stage in snapshot["replay_stages"][1:]
            ],
        },
    }


def write_cache_record(record: dict, output_root: Path) -> Path:
    output_root.mkdir(parents=True, exist_ok=True)
    target = output_root / cache_filename(
        int(record["distance_cm"]),
        int(record["lateral_offset_cm"]),
        int(record["row"]),
    )
    with gzip.open(target, "wt", encoding="utf-8", compresslevel=9) as stream:
        json.dump(record, stream, ensure_ascii=False, separators=(",", ":"))
    return target


def main() -> int:
    parser = argparse.ArgumentParser(description="从距离×横移×五排验证快照生成运行时轨迹缓存")
    parser.add_argument("source_root", type=Path)
    parser.add_argument("output_root", type=Path)
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)

    generated = []
    pattern = "x_*cm/y_*cm/sequence_*/??_L*_R*/stage_snapshot.json"
    for path in sorted(args.source_root.glob(pattern)):
        snapshot = json.loads(path.read_text(encoding="utf-8"))
        try:
            row = int(path.parent.name.split("_", 1)[0])
        except (TypeError, ValueError):
            continue
        try:
            lateral_token = path.parents[2].name.removeprefix("y_").removesuffix("cm")
            lateral_offset_cm = int(lateral_token.replace("p", "+").replace("m", "-"))
        except (TypeError, ValueError):
            continue
        distance_cm = int(round(float(snapshot["box_front_x"]) * 100.0))
        record = build_cache_record(
            snapshot,
            distance_cm=distance_cm,
            lateral_offset_cm=lateral_offset_cm,
            row=row,
        )
        generated.append(write_cache_record(record, args.output_root))

    print(
        f"生成轨迹缓存 {len(generated)} 条，"
        f"总大小 {sum(path.stat().st_size for path in generated)} bytes"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
