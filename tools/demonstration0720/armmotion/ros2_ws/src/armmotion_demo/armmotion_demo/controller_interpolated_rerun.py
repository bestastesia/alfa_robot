from __future__ import annotations

import argparse
import bisect
import json
import os
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import rerun as rr

from alfa_robot_execution_bridge.joints import RT_CONTROL_JOINT_NAMES
from alfa_robot_execution_bridge.trajectory_interpolation import (
    InterpolatedState,
    TrajectorySample,
    downsample_controller_trace,
    sample_fixed_rate,
)
from alfa_robot_rerun import visualize_rerun as rerun_helpers

from .common import MotionSample, STAGE_COUNT, STAGE_LABELS, parse_task_code
from .planner_adapter import PlannerAdapter


DEFAULT_TASKS = "B1,A3,B5"


@dataclass(frozen=True)
class DisplaySample:
    controller_state: InterpolatedState
    updown_m: float
    context: dict[str, Any]


def segment_controller_trace(
    segment: list[MotionSample],
    *,
    control_rate_hz: float,
    display_rate_hz: float,
) -> tuple[list[DisplaySample], int]:
    if len(segment) < 2:
        raise ValueError("轨迹段至少需要两个采样点")
    trajectory = [
        TrajectorySample(
            time_from_start=float(sample.time_s),
            positions=[
                float(sample.updown_m)
                if name == "updown"
                else float(sample.joints[name])
                for name in RT_CONTROL_JOINT_NAMES
            ],
            velocities=[
                float(sample.updown_velocity_m_s)
                if name == "updown"
                else float(sample.joint_velocities.get(name, 0.0))
                for name in RT_CONTROL_JOINT_NAMES
            ],
            accelerations=[
                float(sample.updown_acceleration_m_s2)
                if name == "updown"
                else float(sample.joint_accelerations.get(name, 0.0))
                for name in RT_CONTROL_JOINT_NAMES
            ],
        )
        for sample in segment
    ]
    controller_trace = sample_fixed_rate(trajectory, control_rate_hz)
    display_trace = downsample_controller_trace(controller_trace, display_rate_hz)
    source_times = [float(sample.time_s) for sample in segment]
    updown_index = RT_CONTROL_JOINT_NAMES.index("updown")
    result: list[DisplaySample] = []
    for state in display_trace:
        source_index = bisect.bisect_left(source_times, state.time_from_start)
        if source_index <= 0:
            context = dict(segment[0].context)
        elif source_index >= len(segment):
            context = dict(segment[-1].context)
        else:
            start = segment[source_index - 1]
            goal = segment[source_index]
            context = dict(goal.context)
        updown_m = float(state.positions[updown_index])
        context["updown"] = updown_m
        result.append(
            DisplaySample(
                controller_state=state,
                updown_m=updown_m,
                context=context,
            )
        )
    return result, len(controller_trace)


def segment_raw_trace(segment: list[MotionSample]) -> tuple[list[DisplaySample], int]:
    result = []
    for sample in segment:
        state = InterpolatedState(
            time_from_start=float(sample.time_s),
            positions=[
                float(sample.updown_m)
                if name == "updown"
                else float(sample.joints[name])
                for name in RT_CONTROL_JOINT_NAMES
            ],
            velocities=[
                float(sample.updown_velocity_m_s)
                if name == "updown"
                else float(sample.joint_velocities.get(name, 0.0))
                for name in RT_CONTROL_JOINT_NAMES
            ],
            accelerations=[
                float(sample.updown_acceleration_m_s2)
                if name == "updown"
                else float(sample.joint_accelerations.get(name, 0.0))
                for name in RT_CONTROL_JOINT_NAMES
            ],
        )
        context = dict(sample.context)
        context["updown"] = float(sample.updown_m)
        result.append(
            DisplaySample(
                controller_state=state,
                updown_m=float(sample.updown_m),
                context=context,
            )
        )
    return result, len(result)


def parse_tasks(value: str) -> list[str]:
    tasks = [item.strip().upper() for item in value.split(",") if item.strip()]
    if not tasks:
        raise ValueError("至少需要一个任务编号")
    return tasks


def default_source_ws() -> Path:
    configured = os.environ.get("ARMMOTION_SOURCE_WS")
    if configured:
        return Path(configured).resolve()
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "ros2_ws/src/alfa_robot_moveit_config"
        if candidate.is_dir():
            return parent / "ros2_ws"
    raise FileNotFoundError("无法定位主 ros2_ws，请设置 ARMMOTION_SOURCE_WS")


def static_signature(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def log_display_sample(
    *,
    monitor: Any,
    robot: Any,
    task_code: str,
    stage_number: int,
    segment_number: int,
    sample_number: int,
    sample_count: int,
    sample: DisplaySample,
    timeline_s: float,
    last_static_signature: str | None,
    trajectory_label: str,
) -> str:
    rr.set_time("execution_time", duration=timeline_s)
    joint_map = {
        name: float(value)
        for name, value in zip(
            RT_CONTROL_JOINT_NAMES,
            sample.controller_state.positions,
        )
    }
    rerun_helpers.log_robot_state(robot, joint_map, "monitor/robot")
    static_obstacles = sample.context.get("static_box_obstacles")
    signature = static_signature(static_obstacles)
    if signature != last_static_signature:
        monitor.log_static_box_obstacles(static_obstacles)
        last_static_signature = signature
    monitor.log_attached_boxes(
        robot,
        joint_map,
        sample.context.get("attached_boxes", []),
    )
    rr.log(
        "monitor/info",
        rr.TextLog(
            f"{task_code} | 第{stage_number}步 {STAGE_LABELS[stage_number]} | "
            f"segment={segment_number} sample={sample_number}/{sample_count} | "
            f"14轴={trajectory_label}"
        ),
    )
    return last_static_signature


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="用当前算法计算任务；默认直接记录原始轨迹，可选模拟控制器插值。",
    )
    parser.add_argument("--tasks", default=DEFAULT_TASKS, help="任务编号，逗号分隔")
    parser.add_argument("--front-distance", type=float, default=0.9)
    parser.add_argument("--top-distance", type=float, default=0.7)
    parser.add_argument("--trajectory-rate-hz", type=float, default=30.0)
    parser.add_argument("--control-rate-hz", type=float, default=250.0)
    parser.add_argument("--rerun-rate-hz", type=float, default=90.0)
    parser.add_argument(
        "--raw-trajectory",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="默认直接记录算法输出轨迹点；关闭后才模拟250Hz控制器插值并90Hz抽样。",
    )
    parser.add_argument("--speed-scale", type=float, default=3.0)
    parser.add_argument("--max-joint-speed-deg-s", type=float, default=10.0)
    parser.add_argument("--max-joint-acceleration-deg-s2", type=float, default=60.0)
    parser.add_argument("--max-updown-speed-m-s", type=float, default=0.15)
    parser.add_argument("--planner-timeout-s", type=float, default=180.0)
    parser.add_argument("--source-ws", type=Path, default=None)
    parser.add_argument("--output-root", type=Path, default=None)
    parser.add_argument("--save", type=Path, default=None)
    parser.add_argument("--ros-domain-id", default="auto")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    task_codes = parse_tasks(args.tasks)
    source_ws = (args.source_ws or default_source_ws()).resolve()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = (
        args.output_root
        or source_ws.parent / "data/ik_benchmark/controller_interpolated_rerun"
    ).resolve()
    run_root = output_root / f"run_{stamp}"
    run_root.mkdir(parents=True, exist_ok=False)
    save_path = (args.save or run_root / "controller_interpolated_selected_tasks.rrd").resolve()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    if args.ros_domain_id != "inherit":
        if args.ros_domain_id == "auto":
            os.environ["ROS_DOMAIN_ID"] = str(140 + os.getpid() % 80)
        else:
            os.environ["ROS_DOMAIN_ID"] = str(int(args.ros_domain_id))
    trajectory_mode = (
        f"raw={args.trajectory_rate_hz:.1f}Hz"
        if args.raw_trajectory
        else (
            f"controller={args.control_rate_hz:.1f}Hz "
            f"rerun={args.rerun_rate_hz:.1f}Hz"
        )
    )
    print(
        f"开始：tasks={task_codes} mode={trajectory_mode} "
        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 'inherit')}",
        flush=True,
    )

    rr.init("armmotion_controller_interpolated_rerun", spawn=False)
    rr.save(str(save_path))
    robot = rerun_helpers.UrdfRobot(rerun_helpers.render_current_urdf())
    rr.log("monitor", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rerun_helpers.log_robot_static_model(robot, "monitor/robot", log_meshes=True)

    planner = PlannerAdapter(
        source_ws=source_ws,
        output_root=run_root / "planner",
        rate_hz=float(args.trajectory_rate_hz),
        max_joint_speed_deg_s=float(args.max_joint_speed_deg_s),
        max_joint_acceleration_deg_s2=float(args.max_joint_acceleration_deg_s2),
        max_updown_speed_m_s=float(args.max_updown_speed_m_s),
        max_updown_acceleration_m_s2=0.05,
        speed_scale=float(args.speed_scale),
        timeout_s=float(args.planner_timeout_s),
    )
    monitor = planner.monitor_helpers
    monitor.rr = rr
    summaries: list[dict[str, Any]] = []
    timeline_s = 0.0
    try:
        for task_index, task_code in enumerate(task_codes, start=1):
            task = parse_task_code(
                task_code,
                float(args.front_distance),
                float(args.top_distance),
            )
            print(
                f"[{task_index}/{len(task_codes)}] {task.code} L{task.left_box_id}/R{task.right_box_id} 计算开始",
                flush=True,
            )
            started = time.monotonic()
            plan = planner.compute(task)
            compute_ms = (time.monotonic() - started) * 1000.0
            snapshot = json.loads(plan.snapshot_path.read_text(encoding="utf-8"))
            rr.set_time("execution_time", duration=timeline_s)
            monitor.log_container_panels(snapshot.get("container_panels"))
            last_static_signature: str | None = None
            input_points = 0
            controller_points = 0
            rerun_points = 0
            task_start_s = timeline_s
            for stage_number in range(1, STAGE_COUNT + 1):
                for segment_number, segment in enumerate(
                    plan.stages[stage_number],
                    start=1,
                ):
                    input_points += len(segment)
                    if args.raw_trajectory:
                        display_samples, control_count = segment_raw_trace(segment)
                        trajectory_label = (
                            f"算法输出的{args.trajectory_rate_hz:.1f}Hz原始轨迹点"
                        )
                    else:
                        display_samples, control_count = segment_controller_trace(
                            segment,
                            control_rate_hz=float(args.control_rate_hz),
                            display_rate_hz=float(args.rerun_rate_hz),
                        )
                        trajectory_label = "位置/速度/加速度五次样条250Hz插值后90Hz抽样"
                    controller_points += control_count
                    rerun_points += len(display_samples)
                    segment_start_s = timeline_s
                    for sample_number, sample in enumerate(display_samples, start=1):
                        last_static_signature = log_display_sample(
                            monitor=monitor,
                            robot=robot,
                            task_code=task.code,
                            stage_number=stage_number,
                            segment_number=segment_number,
                            sample_number=sample_number,
                            sample_count=len(display_samples),
                            sample=sample,
                            timeline_s=segment_start_s
                            + sample.controller_state.time_from_start,
                            last_static_signature=last_static_signature,
                            trajectory_label=trajectory_label,
                        )
                    timeline_s = segment_start_s + display_samples[-1].controller_state.time_from_start
            execution_duration_s = timeline_s - task_start_s
            summaries.append(
                {
                    "task": task.code,
                    "left_box_id": task.left_box_id,
                    "right_box_id": task.right_box_id,
                    "compute_ms": compute_ms,
                    "planner_metrics": plan.metrics,
                    "trajectory_input_points": input_points,
                    "trajectory_output_points": controller_points,
                    "rerun_points": rerun_points,
                    "raw_trajectory": bool(args.raw_trajectory),
                    "execution_duration_s": execution_duration_s,
                    "snapshot": str(plan.snapshot_path),
                }
            )
            print(
                f"[{task_index}/{len(task_codes)}] {task.code} 完成："
                f"compute={compute_ms:.1f}ms duration={execution_duration_s:.3f}s "
                f"points={input_points}->{rerun_points}",
                flush=True,
            )
            timeline_s += 0.5
    finally:
        planner.close()
        rr.disconnect()

    summary_path = run_root / "summary.json"
    summary_path.write_text(
        json.dumps(summaries, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"完成：Rerun={save_path}", flush=True)
    print(f"统计={summary_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
