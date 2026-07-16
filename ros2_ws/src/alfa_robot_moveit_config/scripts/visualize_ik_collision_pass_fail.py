#!/usr/bin/python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import extract_sequence_rerun as sequence
import extract_stage_monitor_console as monitor


DEFAULT_RUN_ROOT = Path(
    "/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/"
    "extract_sequence_rerun/sequence_20260711_053356"
)
DEFAULT_TASKS = "05_L6_R13,06_L11_R8,07_L11_R13,10_L16_R18"


def load_snapshot(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"快照格式错误：{path}")
    return data


def log_records(
    snapshot: dict[str, Any],
    records: list[dict[str, Any]],
    accepted: bool,
    helpers: Any,
    robot: Any,
    display_args: Any,
    task_index: int,
    task_count: int,
    sample_start: int,
) -> int:
    scene_y_shift = float(snapshot.get("scene_y_shift", -0.4))
    box_front_x = float(snapshot.get("box_front_x", 0.925))
    left_id = int(snapshot.get("left_box_id", 0))
    right_id = int(snapshot.get("right_box_id", 0))
    attached_boxes = snapshot.get("attached_boxes", [])
    sample = sample_start
    phase = "碰撞通过" if accepted else "碰撞拒绝"
    color = [40, 220, 80, 255] if accepted else [255, 35, 35, 255]

    for record_index, record in enumerate(records):
        joint_map = record.get("state", {}).get("joint_map", {})
        if not isinstance(joint_map, dict) or not joint_map:
            continue
        joints = {str(name): float(value) for name, value in joint_map.items()}
        reason = "collision_free" if accepted else str(
            record.get("scene_rejection_reason", "ik_candidate_scene_rejected")
        )
        helpers.set_sample_time(sample)
        monitor.log_default_container(scene_y_shift)
        monitor.log_box_stack(box_front_x, left_id, right_id, scene_y_shift)
        monitor.log_static_box_obstacles(snapshot.get("static_box_obstacles"))
        sequence.log_robot_state_display(helpers, robot, joints, "monitor/robot", display_args)
        sequence.log_attached_boxes_display(robot, joints, attached_boxes, display_args)
        monitor.rr.log(
            "monitor/diagnostics/ik_collision_result",
            monitor.rr.Points3D(
                positions=[[0.0, 0.0, 0.0]],
                radii=[0.09],
                colors=[color],
                labels=[phase if accepted else reason],
            ),
        )
        monitor.rr.log(
            "monitor/info",
            monitor.rr.TextLog(
                f"任务 {task_index}/{task_count}: L{left_id}/R{right_id} | "
                f"{phase} {record_index + 1}/{len(records)} | "
                f"h={float(record.get('h', joints.get('updown', 0.0))):.4f} "
                f"score={float(record.get('score', 0.0)):.4f} | {reason}"
            ),
        )
        sample += 1
    return sample


def main() -> int:
    parser = argparse.ArgumentParser(
        description="按任务顺序回放 IK 碰撞通过与拒绝结果：任务A通过、任务A失败、任务B通过、任务B失败"
    )
    parser.add_argument("--run-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--tasks", default=DEFAULT_TASKS)
    parser.add_argument("--save", type=Path, required=True)
    args = parser.parse_args()

    task_names = [item.strip() for item in args.tasks.split(",") if item.strip()]
    snapshots = []
    for task_name in task_names:
        path = args.run_root / task_name / "stage_snapshot.json"
        if not path.exists():
            raise FileNotFoundError(path)
        snapshots.append(load_snapshot(path))

    import rerun as rr

    monitor.rr = rr
    helpers = monitor.load_rerun_helpers()
    args.save.parent.mkdir(parents=True, exist_ok=True)
    rr.init("ik_collision_pass_fail")
    rr.save(str(args.save))
    robot = helpers.UrdfRobot(helpers.render_current_urdf())
    rr.log("monitor", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    helpers.log_robot_static_model(robot, "monitor/robot", log_meshes=True)
    display_args = SimpleNamespace(
        display_base_x=0.0,
        display_base_y=0.0,
        display_base_yaw_deg=0.0,
        display_turn_offset_deg=0.0,
    )

    sample = 0
    summary = []
    for task_index, snapshot in enumerate(snapshots, start=1):
        accepted_records = [item for item in snapshot.get("records", []) if isinstance(item, dict)]
        rejected_records = [
            item for item in snapshot.get("scene_rejected_records", []) if isinstance(item, dict)
        ]
        sample = log_records(
            snapshot,
            accepted_records,
            True,
            helpers,
            robot,
            display_args,
            task_index,
            len(snapshots),
            sample,
        )
        sample = log_records(
            snapshot,
            rejected_records,
            False,
            helpers,
            robot,
            display_args,
            task_index,
            len(snapshots),
            sample,
        )
        summary.append(
            f"L{snapshot.get('left_box_id')}/R{snapshot.get('right_box_id')}: "
            f"通过={len(accepted_records)} 拒绝={len(rejected_records)}"
        )

    print(f"已保存：{args.save.resolve()}")
    print(f"总帧数：{sample}")
    for line in summary:
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
