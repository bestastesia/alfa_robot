from __future__ import annotations

import argparse
import json
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import rerun as rr
from geometry_msgs.msg import PoseStamped

from alfa_robot_execution_bridge.joints import RT_CONTROL_JOINT_NAMES
from alfa_robot_rerun import visualize_rerun as rerun_helpers

from .common import (
    MotionSample,
    camera_view_poses_for_task,
    loaded_joint_map,
    parse_task_code,
    planning_task_from_suction_surface_poses,
    suction_surface_poses_for_task,
)
from .controller_interpolated_rerun import segment_raw_trace, static_signature
from .planner_adapter import PlannerAdapter
from .trajectory_cache import CACHE_MAX_DISTANCE_CM, CACHE_MIN_DISTANCE_CM


WORLD_TO_BASE_Z_M = 0.202094


PHASES = (
    ("CAMERA_VIEW", 0),
    ("PREGRASP", 1),
    ("APPROACH", 2),
    ("PLACE_EXTRACT_AND_LOADED", 3),
    ("PLACE_TO_RELEASE", 4),
    ("HOME", 6),
)


def _pose_stamped(value) -> PoseStamped:
    from robot_motion_runtime.dual_grasp_strategy import quaternion_xyzw

    message = PoseStamped()
    message.header.frame_id = "base_link"
    message.pose.position.x = float(value.x)
    message.pose.position.y = float(value.y)
    message.pose.position.z = float(value.z)
    orientation = quaternion_xyzw(value)
    message.pose.orientation.x = orientation[0]
    message.pose.orientation.y = orientation[1]
    message.pose.orientation.z = orientation[2]
    message.pose.orientation.w = orientation[3]
    return message


def _loaded_sample() -> MotionSample:
    joints = loaded_joint_map(name for name in RT_CONTROL_JOINT_NAMES if name != "updown")
    joints["turn"] = 0.0
    return MotionSample(
        time_s=0.0,
        joints=joints,
        updown_m=0.3,
        context={"stage": "action_grid/initial", "updown": 0.3},
    )


def _log_recapture_targets(left_pose, right_pose, *, failed: bool) -> None:
    poses = (left_pose, right_pose)
    color = [255, 40, 40, 255] if failed else [80, 220, 255, 255]
    rr.log(
        "monitor/scene/recapture_targets",
        rr.Points3D(
            positions=[
                [pose.x, pose.y, pose.z + WORLD_TO_BASE_Z_M]
                for pose in poses
            ],
            radii=[0.04, 0.04],
            colors=[color, color],
            labels=["left_camera_view_35cm", "right_camera_view_35cm"],
        ),
    )


def _log_failure(
    *,
    robot: Any,
    task_label: str,
    left_camera,
    right_camera,
    reason: str,
    timeline_s: float,
) -> None:
    rr.set_time("execution_time", duration=timeline_s)
    initial = _loaded_sample()
    joint_map = dict(initial.joints)
    joint_map["updown"] = initial.updown_m
    rerun_helpers.log_robot_state(robot, joint_map, "monitor/robot")
    _log_recapture_targets(left_camera, right_camera, failed=True)
    rr.log(
        "monitor/info",
        rr.TextLog(f"{task_label} | CAMERA_VIEW失败 | {reason}"),
    )


def _log_sample(
    *,
    monitor: Any,
    robot: Any,
    task_label: str,
    phase_label: str,
    segment_number: int,
    sample_number: int,
    sample_count: int,
    sample,
    timeline_s: float,
    last_static_signature: str | None,
) -> str | None:
    rr.set_time("execution_time", duration=timeline_s)
    joint_map = {
        name: float(value)
        for name, value in zip(RT_CONTROL_JOINT_NAMES, sample.controller_state.positions)
    }
    rerun_helpers.log_robot_state(robot, joint_map, "monitor/robot")
    static_obstacles = sample.context.get("static_box_obstacles")
    signature = static_signature(static_obstacles)
    if signature != last_static_signature:
        monitor.log_static_box_obstacles(static_obstacles)
        last_static_signature = signature
    monitor.log_attached_boxes(robot, joint_map, sample.context.get("attached_boxes", []))
    rr.log(
        "monitor/info",
        rr.TextLog(
            f"{task_label} | {phase_label} | segment={segment_number} "
            f"sample={sample_number}/{sample_count}"
        ),
    )
    return last_static_signature


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="生成30组缓存Action的完整原始轨迹Rerun")
    parser.add_argument("--source-ws", type=Path)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--save", type=Path)
    parser.add_argument("--planner-timeout-s", type=float, default=180.0)
    parser.add_argument("--trajectory-rate-hz", type=float, default=30.0)
    parser.add_argument("--speed-scale", type=float, default=3.0)
    return parser


def _default_source_ws() -> Path:
    configured = os.environ.get("ARMMOTION_SOURCE_WS")
    if configured:
        return Path(configured).resolve()
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "ros2_ws/src/alfa_robot_moveit_config"
        if candidate.is_dir():
            return parent / "ros2_ws"
    raise FileNotFoundError("无法定位主 ros2_ws，请设置 ARMMOTION_SOURCE_WS")


def main() -> int:
    args = build_parser().parse_args()
    source_ws = (args.source_ws or _default_source_ws()).resolve()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = (
        args.output_root
        or source_ws.parent / "data/ik_benchmark/motion_action_cache_30_full_flow"
    ).resolve()
    run_root = output_root / f"run_{stamp}"
    run_root.mkdir(parents=True, exist_ok=False)
    save_path = (args.save or run_root / "motion_action_cache_30_full_flow.rrd").resolve()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = str(160 + os.getpid() % 40)

    rr.init("motion_action_cache_30_full_flow", spawn=False)
    rr.save(str(save_path))
    robot = rerun_helpers.UrdfRobot(rerun_helpers.render_current_urdf())
    rr.log("monitor", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rerun_helpers.log_robot_static_model(robot, "monitor/robot", log_meshes=True)

    planner = PlannerAdapter(
        source_ws=source_ws,
        output_root=run_root / "planner",
        rate_hz=float(args.trajectory_rate_hz),
        max_joint_speed_deg_s=10.0,
        max_joint_acceleration_deg_s2=60.0,
        max_updown_speed_m_s=0.15,
        max_updown_acceleration_m_s2=0.05,
        speed_scale=float(args.speed_scale),
        timeout_s=float(args.planner_timeout_s),
    )
    monitor = planner.monitor_helpers
    monitor.rr = rr
    timeline_s = 0.0
    summaries: list[dict[str, Any]] = []
    try:
        task_total = (CACHE_MAX_DISTANCE_CM - CACHE_MIN_DISTANCE_CM + 1) * 5
        task_index = 0
        for distance_cm in range(CACHE_MIN_DISTANCE_CM, CACHE_MAX_DISTANCE_CM + 1):
            distance_m = distance_cm / 100.0
            for row in range(1, 6):
                task_index += 1
                fixture = parse_task_code(f"B{row}", distance_m, distance_m)
                left_surface, right_surface = suction_surface_poses_for_task(fixture)
                left_camera, right_camera = camera_view_poses_for_task(fixture)
                task_label = f"x_{distance_cm:02d}cm_row_{row}"
                task = planning_task_from_suction_surface_poses(
                    task_label,
                    left_surface,
                    right_surface,
                    fixture.left_grasp_mode,
                    fixture.right_grasp_mode,
                )
                print(f"[{task_index}/{task_total}] {task_label} 开始", flush=True)
                started = time.monotonic()
                try:
                    recapture, recapture_metrics = planner.plan_recapture(
                        _pose_stamped(left_camera),
                        _pose_stamped(right_camera),
                        _loaded_sample(),
                        preferred_updown=0.3,
                        context_stage=f"{task_label}/camera_view",
                    )
                    plan = planner.compute(task, initial_sample=recapture[-1])
                except Exception as exc:
                    compute_ms = (time.monotonic() - started) * 1000.0
                    reason = str(exc)
                    _log_failure(
                        robot=robot,
                        task_label=task_label,
                        left_camera=left_camera,
                        right_camera=right_camera,
                        reason=reason,
                        timeline_s=timeline_s,
                    )
                    summaries.append(
                        {
                            "task": task_label,
                            "distance_cm": distance_cm,
                            "row": row,
                            "success": False,
                            "compute_ms": compute_ms,
                            "failure_reason": reason,
                        }
                    )
                    print(
                        f"[{task_index}/{task_total}] {task_label} 失败 "
                        f"compute={compute_ms:.1f}ms reason={reason}",
                        flush=True,
                    )
                    timeline_s += 1.0
                    continue
                compute_ms = (time.monotonic() - started) * 1000.0
                snapshot = json.loads(plan.snapshot_path.read_text(encoding="utf-8"))
                rr.set_time("execution_time", duration=timeline_s)
                _log_recapture_targets(left_camera, right_camera, failed=False)
                monitor.log_container_panels(snapshot.get("container_panels"))
                segments_by_phase = {
                    0: [recapture],
                    1: plan.stages[1],
                    2: plan.stages[2],
                    3: plan.stages[3],
                    4: plan.stages[4],
                    6: plan.stages[6],
                }
                task_start_s = timeline_s
                last_static_signature: str | None = None
                point_count = 0
                for phase_label, phase_number in PHASES:
                    for segment_number, segment in enumerate(
                        segments_by_phase[phase_number], start=1
                    ):
                        display_samples, _ = segment_raw_trace(segment)
                        segment_start_s = timeline_s
                        for sample_number, sample in enumerate(display_samples, start=1):
                            last_static_signature = _log_sample(
                                monitor=monitor,
                                robot=robot,
                                task_label=task_label,
                                phase_label=phase_label,
                                segment_number=segment_number,
                                sample_number=sample_number,
                                sample_count=len(display_samples),
                                sample=sample,
                                timeline_s=segment_start_s
                                + sample.controller_state.time_from_start,
                                last_static_signature=last_static_signature,
                            )
                        point_count += len(display_samples)
                        timeline_s = (
                            segment_start_s
                            + display_samples[-1].controller_state.time_from_start
                        )
                summaries.append(
                    {
                        "task": task_label,
                        "distance_cm": distance_cm,
                        "row": row,
                        "success": True,
                        "compute_ms": compute_ms,
                        "recapture_metrics": recapture_metrics,
                        "planner_metrics": plan.metrics,
                        "trajectory_points": point_count,
                        "execution_duration_s": timeline_s - task_start_s,
                    }
                )
                print(
                    f"[{task_index}/{task_total}] {task_label} 完成 "
                    f"compute={compute_ms:.1f}ms points={point_count}",
                    flush=True,
                )
                timeline_s += 0.5
    finally:
        planner.close()
        rr.disconnect()

    summary_path = run_root / "summary.json"
    summary_path.write_text(
        json.dumps(summaries, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"完成：Rerun={save_path}", flush=True)
    print(f"统计={summary_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
