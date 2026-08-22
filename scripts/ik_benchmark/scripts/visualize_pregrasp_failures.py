#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
MONITOR_DIR = SCRIPT_DIR.parents[2] / "ros2_ws" / "src" / "alfa_robot_moveit_config" / "scripts"
if str(MONITOR_DIR) not in sys.path:
    sys.path.insert(0, str(MONITOR_DIR))

import extract_stage_monitor_console as monitor  # noqa: E402
import rerun as rr  # noqa: E402
from alfa_robot_rerun import visualize_rerun as helpers  # noqa: E402
from visualize_analytic_radial_extract import (  # noqa: E402
    collision_object_ids,
    joint_map,
    log_box_set,
    log_collision_scene,
    planned_attached_boxes,
    quaternion_matrix,
)


def log_targets(frame: dict[str, Any], failed: bool) -> None:
    positions = [
        [float(value) for value in frame[f"{side}_target_position_world"]]
        for side in ("left", "right")
    ]
    vectors = []
    for side in ("left", "right"):
        rotation = quaternion_matrix(frame[f"{side}_target_orientation_xyzw"])
        vectors.append((rotation[:, 2] * 0.16).tolist())
    color = [255, 40, 40, 255] if failed else [40, 220, 100, 255]
    rr.log(
        "monitor/pregrasp/targets",
        rr.Points3D(
            positions,
            colors=[color, color],
            radii=0.025,
            labels=["left pregrasp target", "right pregrasp target"],
        ),
    )
    rr.log(
        "monitor/pregrasp/tool_z",
        rr.Arrows3D(
            origins=positions,
            vectors=vectors,
            colors=[color, color],
            radii=0.008,
        ),
    )


def fallback_joint_map(data: dict[str, Any]) -> dict[str, float] | None:
    frames = data.get("frames", [])
    if not frames:
        return None
    return joint_map(frames[0], float(data["updown"]))


def main() -> None:
    parser = argparse.ArgumentParser(description="合并显示预抓取到IK优解的失败姿态")
    parser.add_argument("results", nargs="+", type=Path)
    parser.add_argument("--save", type=Path, required=True)
    args = parser.parse_args()

    rr.init("pregrasp_to_ik_failures")
    rr.save(str(args.save))
    monitor.rr = rr
    robot = helpers.UrdfRobot(helpers.render_current_urdf())
    helpers.log_robot_static_model(
        robot,
        "monitor/robot",
        log_meshes=True,
        mesh_albedo_factor=[255, 255, 255, 165],
    )

    sample = 0
    for case_index, result_path in enumerate(args.results, start=1):
        data = json.loads(result_path.read_text(encoding="utf-8"))
        transition = data.get("pregrasp_transition", {})
        frames = list(transition.get("frames", []))
        if not frames:
            continue
        original_boxes, _ = planned_attached_boxes(data, data["frames"][0], 0.0)
        default_joints = fallback_joint_map(data)
        for frame_index, frame in enumerate(frames):
            helpers.set_sample_time(sample)
            failed = not (
                frame.get("solved", False)
                and frame.get("bounds_ok", False)
                and frame.get("collision_free", False)
            )
            positions = joint_map(frame, float(data["updown"])) or default_joints
            if positions is not None:
                helpers.log_robot_state(robot, positions, "monitor/robot")
            contacts = [str(value) for value in frame.get("contacts", [])]
            log_collision_scene(data, collision_object_ids(contacts))
            log_box_set(
                "monitor/target_boxes/original",
                original_boxes,
                [],
                [70, 190, 255, 95],
                "original target",
            )
            log_targets(frame, failed)
            rr.log(
                "monitor/status",
                rr.TextLog(
                    f"case={case_index}/{len(args.results)} {result_path.parent.name}\n"
                    f"pregrasp_frame={frame_index + 1}/{len(frames)} ratio={float(frame.get('ratio', 0.0)):.2f}\n"
                    f"solved={frame.get('solved')} bounds={frame.get('bounds_ok')} "
                    f"collision_free={frame.get('collision_free')}\n"
                    f"failure={frame.get('solve_failure', '') or transition.get('failure_reason', '')}\n"
                    f"contacts={contacts}\n"
                    "红点/红箭头=本帧要求的预抓取目标；解析无解时机器人显示吸附IK参考姿态。"
                ),
            )
            sample += 1
        for _ in range(3):
            helpers.set_sample_time(sample)
            rr.log(
                "monitor/status",
                rr.TextLog(
                    f"{result_path.parent.name} 失败结束：{transition.get('failure_reason', '')}"
                ),
            )
            sample += 1

    print(f"saved={args.save} cases={len(args.results)} samples={sample}")


if __name__ == "__main__":
    main()
