#!/usr/bin/env python3
from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path


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


def initial_state(snapshot: dict) -> dict:
    stages = snapshot.get("replay_stages", [])
    if not stages:
        raise ValueError("snapshot replay_stages 为空")
    trajectory = stages[0].get("trajectory", {})
    names = trajectory.get("joint_names", [])
    points = trajectory.get("points", [])
    if not points:
        raise ValueError("snapshot 首段轨迹为空")
    positions = points[0].get("positions", [])
    values = dict(zip(names, positions))
    return {
        "joints": {name: float(values.get(name, 0.0)) for name in JOINT_NAMES},
        "updown_m": float(values.get("updown", 0.3)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="从六档五排验证快照生成运行时轨迹缓存")
    parser.add_argument("source_root", type=Path)
    parser.add_argument("output_root", type=Path)
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)

    generated = []
    for path in sorted(args.source_root.glob("x_*cm/sequence_*/??_L*_R*/stage_snapshot.json")):
        snapshot = json.loads(path.read_text(encoding="utf-8"))
        pair = (int(snapshot["left_box_id"]), int(snapshot["right_box_id"]))
        if pair not in TASK_ROWS:
            continue
        distance_cm = int(round(float(snapshot["box_front_x"]) * 100.0))
        row = TASK_ROWS[pair]
        record = {
            "schema_version": 1,
            "distance_cm": distance_cm,
            "row": row,
            "source_pair": list(pair),
            "initial_state": initial_state(snapshot),
            "snapshot": {
                "type": "trajectory_cache",
                "success": True,
                "box_front_x": float(snapshot["box_front_x"]),
                "scene_y_shift": float(snapshot.get("scene_y_shift", 0.0)),
                "left_box_id": pair[0],
                "right_box_id": pair[1],
                "replay_stages": [slim_stage(stage) for stage in snapshot["replay_stages"]],
            },
        }
        target = args.output_root / f"x_{distance_cm:02d}cm_row_{row}.json.gz"
        with gzip.open(target, "wt", encoding="utf-8", compresslevel=9) as stream:
            json.dump(record, stream, ensure_ascii=False, separators=(",", ":"))
        generated.append(target)

    if len(generated) != 30:
        raise RuntimeError(f"期望生成30条缓存，实际 {len(generated)}")
    print(f"生成轨迹缓存 {len(generated)} 条，总大小 {sum(path.stat().st_size for path in generated)} bytes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
