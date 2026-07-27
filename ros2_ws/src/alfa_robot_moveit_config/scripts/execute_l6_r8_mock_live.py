#!/usr/bin/python3
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import rclpy
from alfa_robot_rerun import visualize_rerun as rerun_helpers
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


SCRIPT_DIR = Path(__file__).resolve().parent


def find_repo_root() -> Path:
    env_root = os.environ.get("ALFA_ROBOT_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve()
    for candidate in [SCRIPT_DIR, *SCRIPT_DIR.parents]:
        if (candidate / "ros2_ws" / "src").is_dir():
            return candidate
        if candidate.name == "ros2_ws":
            return candidate.parent
    return Path.cwd().resolve()


REPO_ROOT = find_repo_root()
ROS_WS = REPO_ROOT / "ros2_ws"
DEFAULT_MOCK_OUTPUT_ROOT = REPO_ROOT / "data/ik_benchmark/live_mock_execution"
DEFAULT_REAL_OUTPUT_ROOT = REPO_ROOT / "data/ik_benchmark/live_real_execution"

# real direct（不经 execution_bridge 转发，直接发真实控制器 action）流程的运行时安全上限。
# 超过这些值需要显式设置对应的 ALFA_ALLOW_UNSAFE_*_OVERRIDE 环境变量才能绕过。
MAX_SAFE_REAL_HZ = 10.0
MAX_SAFE_REAL_JOINT_SPEED_DEG_S = 20.0
MAX_SAFE_REAL_UPDOWN_SPEED_M_S = 0.05

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
BRIDGE_SRC = ROS_WS / "src/alfa_robot_execution_bridge"
if str(BRIDGE_SRC) not in sys.path:
    sys.path.insert(0, str(BRIDGE_SRC))
from alfa_robot_execution_bridge.joints import (  # noqa: E402
    EXECUTION_JOINT_NAMES,
    REAL_CONTROLLER_JOINT_NAMES,
    ethercat_to_ros_position,
    physical_to_logical_updown,
    ros_to_ethercat_position,
)
from alfa_robot_execution_bridge.updown import (  # noqa: E402
    DEFAULT_UPDOWN_ACCELERATION_MPS2,
    DEFAULT_UPDOWN_DECELERATION_MPS2,
    make_updown_command_data,
    synchronized_updown_velocity_mps,
)
import extract_stage_monitor_console as monitor  # noqa: E402
import process_lifecycle  # noqa: E402


# /joint_states 由传感器/驱动侧以 BEST_EFFORT 发布（rclcpp SensorDataQoS 语义）。
# 订阅方若用默认 RELIABLE，会因 QoS 不兼容而收不到任何消息（DDS 直接拒绝匹配）。
# 这里显式用 BEST_EFFORT + KEEP_LAST(depth=10) 与发布方对齐，读取当前关节状态。
JOINT_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def bash_source_command(command: str) -> list[str]:
    return [
        "bash",
        "-lc",
        "source /opt/ros/humble/setup.bash && "
        f"source {ROS_WS}/install/setup.bash && "
        f"cd {ROS_WS} && "
        f"{command}",
    ]


def terminate_process(process: subprocess.Popen[str] | None, timeout: float = 5.0) -> None:
    process_lifecycle.terminate_process_tree(process, interrupt_timeout=timeout)


def wait_for_enter_confirmation(prompt: str) -> None:
    """real direct 流程的人工回车确认关卡。"""
    if not sys.stdin.isatty():
        raise SystemExit(
            f"拒绝执行：{prompt} 需要交互式人工回车确认，"
            "但当前标准输入不是 tty，无法确认人员/设备安全状态。"
        )
    input(f"{prompt}\n确认后直接按回车继续；Ctrl+C 取消：")


def seconds_to_duration(seconds: float):
    msg = JointTrajectoryPoint().time_from_start
    seconds = max(0.0, float(seconds))
    whole = math.floor(seconds)
    msg.sec = int(whole)
    msg.nanosec = int(round((seconds - whole) * 1e9))
    if msg.nanosec >= 1_000_000_000:
        msg.sec += 1
        msg.nanosec -= 1_000_000_000
    return msg


def duration_to_seconds(duration: Any) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def moveit_to_execution_name(name: str) -> str | None:
    if name.startswith("left_joint"):
        return "left_joint" + name.removeprefix("left_joint")
    if name.startswith("right_joint"):
        return "right_joint" + name.removeprefix("right_joint")
    if name.startswith("left_joint"):
        return name
    if name.startswith("right_joint"):
        return name
    if name == "turn":
        return "turn"
    return None


def execution_to_moveit_name(name: str) -> str:
    if name.startswith("left_joint"):
        return "left_joint" + name.removeprefix("left_joint")
    if name.startswith("right_joint"):
        return "right_joint" + name.removeprefix("right_joint")
    return name


def execution_to_rerun_joint_map(positions: list[float], updown: float = 0.3) -> dict[str, float]:
    joint_map = {"updown": updown, "pitch": 0.0, "turn": 0.0}
    for name, value in zip(EXECUTION_JOINT_NAMES, positions):
        joint_map[execution_to_moveit_name(name)] = float(value)
    return joint_map


def extract_position_from_stage_point(stage: dict[str, Any], point: dict[str, Any], previous: dict[str, float]) -> dict[str, float]:
    joints = dict(previous)
    names = list(stage.get("trajectory", {}).get("joint_names", []))
    positions = list(point.get("positions", []))
    for name, value in zip(names, positions):
        target = moveit_to_execution_name(name)
        if target is not None:
            joints[target] = float(value)
    joints["turn"] = 0.0
    return joints


def extract_updown_from_stage_point(stage: dict[str, Any], point: dict[str, Any], previous: float) -> float:
    names = list(stage.get("trajectory", {}).get("joint_names", []))
    positions = list(point.get("positions", []))
    for name, value in zip(names, positions):
        if name == "updown":
            return float(value)
    return float(previous)


def resample_segment(
    start: dict[str, float],
    goal: dict[str, float],
    *,
    start_time: float,
    hz: float,
    max_joint_speed_deg_s: float,
    minimum_duration_s: float = 0.0,
) -> list[tuple[float, dict[str, float]]]:
    max_delta = max(abs(goal[name] - start[name]) for name in EXECUTION_JOINT_NAMES)
    duration = max(
        1.0 / hz,
        max_delta / math.radians(max_joint_speed_deg_s),
        float(minimum_duration_s),
    )
    steps = max(1, int(math.ceil(duration * hz)))
    out: list[tuple[float, dict[str, float]]] = []
    for step in range(1, steps + 1):
        ratio = step / steps
        positions = {
            name: start[name] + (goal[name] - start[name]) * ratio
            for name in EXECUTION_JOINT_NAMES
        }
        out.append((start_time + step / hz, positions))
    return out


def make_point(time_s: float, positions: list[float]) -> JointTrajectoryPoint:
    point = JointTrajectoryPoint()
    point.time_from_start = seconds_to_duration(time_s)
    point.positions = list(positions)
    return point


def make_trajectory(
    samples: list[tuple[float, dict[str, float]]],
    joint_names: list[str] | None = None,
    *,
    apply_ethercat_signs: bool = False,
) -> JointTrajectory:
    joint_names = joint_names or EXECUTION_JOINT_NAMES
    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)
    if not samples:
        return trajectory
    start_time = samples[0][0]
    for time_s, joint_map in samples:
        if apply_ethercat_signs:
            positions = [ros_to_ethercat_position(name, joint_map[name]) for name in joint_names]
        else:
            positions = [joint_map[name] for name in joint_names]
        trajectory.points.append(make_point(time_s - start_time, positions))
    return trajectory


LOADED_LEFT_POSE_FAMILY_DEG = [
    [0.0, -45.0, 120.0, -75.0, 0.0, 0.0],
    [0.0, -75.0, 135.0, 0.0, 60.0, 0.0],
    [33.87, 75.82, -135.08, 0.0, -59.25, -33.87],
]
LOADED_RIGHT_POSE_FAMILY_DEG = [
    [0.0, -45.0, 120.0, -75.0, 0.0, 0.0],
    [0.0, -75.0, 135.0, 0.0, 60.0, 0.0],
    [-30.93, 74.17, -134.92, 0.0, -60.74, 30.93],
]
FRONT_SUCTION_BOX_IDS = {1, 3, 4, 6}


def normalize_grasp_mode(value: str) -> str:
    aliases = {
        "": "auto",
        "auto": "auto",
        "front": "front",
        "side": "front",
        "side_suction": "front",
        "top": "top_suction",
        "top_suction": "top_suction",
        "down": "top_suction",
    }
    key = str(value).strip().lower()
    if key not in aliases:
        raise ValueError(f"invalid grasp mode: {value}")
    return aliases[key]


def grasp_mode_for_box(box_id: int) -> str:
    return "front" if int(box_id) in FRONT_SUCTION_BOX_IDS else "top_suction"


def pair_vehicle_mode(left_mode: str, right_mode: str) -> str:
    return "top_suction" if "top_suction" in (left_mode, right_mode) else "front"


def effective_box_front_x(args: argparse.Namespace, mode: str) -> float:
    if mode != "top_suction":
        return float(args.box_front_x)
    if args.top_box_front_x is not None:
        return float(args.top_box_front_x)
    return float(args.box_front_x) - float(args.top_approach_forward)


def loaded_joint_map(index: int = 0) -> dict[str, float]:
    pose_index = max(0, min(int(index), len(LOADED_LEFT_POSE_FAMILY_DEG) - 1))
    left = [math.radians(v) for v in LOADED_LEFT_POSE_FAMILY_DEG[pose_index]]
    right = [math.radians(v) for v in LOADED_RIGHT_POSE_FAMILY_DEG[pose_index]]
    values = left + right + [0.0]
    return dict(zip(EXECUTION_JOINT_NAMES, values))


def trajectory_from_snapshot(
    snapshot: dict[str, Any],
    hz: float,
    max_joint_speed_deg_s: float,
    *,
    initial: dict[str, float] | None = None,
) -> list[tuple[float, dict[str, float], dict[str, Any]]]:
    current = dict(initial) if initial is not None else {name: 0.0 for name in EXECUTION_JOINT_NAMES}
    samples: list[tuple[float, dict[str, float], dict[str, Any]]] = []
    time_s = 0.0
    for stage_index, stage in enumerate(snapshot.get("replay_stages", [])):
        points = monitor.ensure_points_start_at_stage_start(stage, list(stage.get("trajectory", {}).get("points", [])))
        for point_index, point in enumerate(points):
            goal = extract_position_from_stage_point(stage, point, current)
            for sample_time, sample in resample_segment(
                current,
                goal,
                start_time=time_s,
                hz=hz,
                max_joint_speed_deg_s=max_joint_speed_deg_s,
            ):
                samples.append((
                    sample_time,
                    sample,
                    {
                        "stage_index": stage_index,
                        "stage": stage.get("stage", ""),
                        "point_index": point_index,
                        "attached_boxes": stage.get("attached_boxes", []),
                        "static_box_obstacles": stage.get("static_box_obstacles", []),
                    },
                ))
            if samples:
                time_s = samples[-1][0]
                current = dict(samples[-1][1])
    return samples


def _same_execution_sample(
    lhs: dict[str, float],
    lhs_updown: float,
    rhs: dict[str, float],
    rhs_updown: float,
    tolerance: float = 1e-9,
) -> bool:
    return (
        all(abs(float(lhs.get(name, 0.0)) - float(rhs.get(name, 0.0))) <= tolerance for name in EXECUTION_JOINT_NAMES)
        and abs(float(lhs_updown) - float(rhs_updown)) <= tolerance
    )


def resample_state_segment(
    start: dict[str, float],
    start_updown: float,
    goal: dict[str, float],
    goal_updown: float,
    *,
    start_time: float,
    hz: float,
    max_joint_speed_deg_s: float,
    max_updown_speed_m_s: float,
    context: dict[str, Any],
) -> list[tuple[float, dict[str, float], dict[str, Any]]]:
    max_joint_delta = max(abs(goal[name] - start[name]) for name in EXECUTION_JOINT_NAMES)
    joint_duration_s = max_joint_delta / math.radians(max_joint_speed_deg_s) if max_joint_speed_deg_s > 0.0 else 0.0
    updown_duration_s = (
        abs(float(goal_updown) - float(start_updown)) / max_updown_speed_m_s
        if max_updown_speed_m_s > 0.0
        else 0.0
    )
    duration_s = max(1.0 / max(hz, 1e-9), joint_duration_s, updown_duration_s)
    steps = max(1, int(math.ceil(duration_s * hz)))
    out: list[tuple[float, dict[str, float], dict[str, Any]]] = []
    for step in range(1, steps + 1):
        ratio = step / steps
        sample = {
            name: start[name] + (goal[name] - start[name]) * ratio
            for name in EXECUTION_JOINT_NAMES
        }
        sample_context = dict(context)
        sample_context["updown"] = float(start_updown) + (float(goal_updown) - float(start_updown)) * ratio
        out.append((start_time + step / hz, sample, sample_context))
    return out


def trajectory_from_snapshot_preserve_timing(
    snapshot: dict[str, Any],
    *,
    initial: dict[str, float] | None = None,
    initial_updown: float = 0.3,
    hz: float = 10.0,
    max_joint_speed_deg_s: float = 20.0,
    max_updown_speed_m_s: float = 0.05,
) -> list[tuple[float, dict[str, float], dict[str, Any]]]:
    """Flatten planner replay stages without destroying planner timing.

    Timed C++ plan segments keep their original time_from_start. Zero-duration
    keyframe stages, mainly extraction rollout records, are expanded at the requested
    rate with explicit joint/updown speed limits.
    """
    current = dict(initial) if initial is not None else {name: 0.0 for name in EXECUTION_JOINT_NAMES}
    current_updown = float(initial_updown)
    samples: list[tuple[float, dict[str, float], dict[str, Any]]] = []
    stage_offset_s = 0.0

    for stage_index, stage in enumerate(snapshot.get("replay_stages", [])):
        points = monitor.ensure_points_start_at_stage_start(stage, list(stage.get("trajectory", {}).get("points", [])))
        if not points:
            continue
        local_zero_s = float(points[0].get("time_from_start_sec", 0.0))
        local_times = [max(0.0, float(point.get("time_from_start_sec", 0.0)) - local_zero_s) for point in points]
        stage_duration_s = max(local_times) if local_times else 0.0

        if stage_duration_s <= 1e-9:
            for point_index, point in enumerate(points):
                sample = extract_position_from_stage_point(stage, point, current)
                updown = extract_updown_from_stage_point(stage, point, current_updown)
                context = {
                    "stage_index": stage_index,
                    "stage": stage.get("stage", ""),
                    "point_index": point_index,
                    "attached_boxes": stage.get("attached_boxes", []),
                    "static_box_obstacles": stage.get("static_box_obstacles", []),
                    "updown": updown,
                }
                if _same_execution_sample(current, current_updown, sample, updown):
                    continue
                generated = resample_state_segment(
                    current,
                    current_updown,
                    sample,
                    updown,
                    start_time=stage_offset_s,
                    hz=hz,
                    max_joint_speed_deg_s=max_joint_speed_deg_s,
                    max_updown_speed_m_s=max_updown_speed_m_s,
                    context=context,
                )
                samples.extend(generated)
                stage_offset_s = generated[-1][0]
                current = sample
                current_updown = updown
            continue

        for point_index, point in enumerate(points):
            local_s = local_times[point_index]
            sample_time_s = stage_offset_s + local_s
            sample = extract_position_from_stage_point(stage, point, current)
            updown = extract_updown_from_stage_point(stage, point, current_updown)
            context = {
                "stage_index": stage_index,
                "stage": stage.get("stage", ""),
                "point_index": point_index,
                "attached_boxes": stage.get("attached_boxes", []),
                "static_box_obstacles": stage.get("static_box_obstacles", []),
                "updown": updown,
            }
            if samples:
                previous_time_s, previous_sample, previous_context = samples[-1]
                previous_updown = float(previous_context.get("updown", current_updown))
                if sample_time_s <= previous_time_s + 1e-9:
                    if _same_execution_sample(previous_sample, previous_updown, sample, updown):
                        current = sample
                        current_updown = updown
                        continue
                    raise RuntimeError(
                        "snapshot timing is non-increasing at a discontinuous stage boundary: "
                        f"stage={stage.get('stage', '')!r} point={point_index} "
                        f"time={sample_time_s:.6f}s previous={previous_time_s:.6f}s"
                    )
            samples.append((sample_time_s, sample, context))
            current = sample
            current_updown = updown
        stage_offset_s += stage_duration_s
    return samples


def split_task_execution_phases(
    samples: list[tuple[float, dict[str, float], dict[str, Any]]],
) -> tuple[
    list[tuple[float, dict[str, float], dict[str, Any]]],
    list[tuple[float, dict[str, float], dict[str, Any]]],
    list[tuple[float, dict[str, float], dict[str, Any]]],
]:
    """Split replay samples into pre-contact, 5cm contact, and post-contact phases."""
    pre_contact = [sample for sample in samples if int(sample[2].get("stage_index", -1)) == 0]
    contact = [sample for sample in samples if int(sample[2].get("stage_index", -1)) == 1]
    post_contact = [sample for sample in samples if int(sample[2].get("stage_index", -1)) >= 2]
    if not pre_contact or not contact or not post_contact:
        raise RuntimeError(
            "snapshot is missing required execution phases: "
            f"pre_contact={len(pre_contact)} contact={len(contact)} post_contact={len(post_contact)}"
        )

    def prepend_boundary(
        phase: list[tuple[float, dict[str, float], dict[str, Any]]],
        previous: tuple[float, dict[str, float], dict[str, Any]],
    ) -> list[tuple[float, dict[str, float], dict[str, Any]]]:
        first_time = phase[0][0]
        if previous[0] > first_time + 1e-9:
            raise RuntimeError("execution phase boundary time moved backwards")
        boundary_context = dict(phase[0][2])
        boundary_context["updown"] = float(previous[2].get("updown", boundary_context.get("updown", 0.3)))
        return [(previous[0], dict(previous[1]), boundary_context), *phase]

    contact = prepend_boundary(contact, pre_contact[-1])
    post_contact = prepend_boundary(post_contact, contact[-1])
    return pre_contact, contact, post_contact


def split_post_contact_place_cycle(
    post_contact: list[tuple[float, dict[str, float], dict[str, Any]]],
    *,
    required: bool,
) -> tuple[
    list[tuple[float, dict[str, float], dict[str, Any]]],
    list[tuple[float, dict[str, float], dict[str, Any]]],
    list[tuple[float, dict[str, float], dict[str, Any]]],
]:
    """Split post-contact replay into extract, loaded-to-place, and return phases."""

    def stage_ends_with(sample, suffix: str) -> bool:
        return str(sample[2].get("stage", "")).endswith(suffix)

    loaded_to_place = [
        sample for sample in post_contact
        if stage_ends_with(sample, "/selected_loaded_to_place")
    ]
    place_to_loaded = [
        sample for sample in post_contact
        if stage_ends_with(sample, "/selected_place_to_loaded")
    ]
    if not loaded_to_place and not place_to_loaded:
        if required:
            raise RuntimeError("snapshot is missing the required loaded/place/loaded cycle")
        return post_contact, [], []
    if not loaded_to_place or not place_to_loaded:
        raise RuntimeError(
            "snapshot contains an incomplete place cycle: "
            f"loaded_to_place={len(loaded_to_place)} place_to_loaded={len(place_to_loaded)}"
        )

    place_stage_names = {
        str(sample[2].get("stage", "")) for sample in loaded_to_place + place_to_loaded
    }
    extract_to_loaded = [
        sample for sample in post_contact
        if str(sample[2].get("stage", "")) not in place_stage_names
    ]
    if not extract_to_loaded:
        raise RuntimeError("snapshot place cycle has no preceding extract-to-loaded phase")

    def prepend_boundary(phase, previous):
        boundary_context = dict(phase[0][2])
        boundary_context["updown"] = float(
            previous[2].get("updown", boundary_context.get("updown", 0.3))
        )
        return [(previous[0], dict(previous[1]), boundary_context), *phase]

    loaded_to_place = prepend_boundary(loaded_to_place, extract_to_loaded[-1])
    place_to_loaded = prepend_boundary(place_to_loaded, loaded_to_place[-1])
    return extract_to_loaded, loaded_to_place, place_to_loaded


def phase_updown_samples(
    samples: list[tuple[float, dict[str, float], dict[str, Any]]],
) -> list[tuple[float, float]]:
    if not samples:
        return []
    start_time = samples[0][0]
    return [
        (time_s - start_time, float(context["updown"]))
        for time_s, _, context in samples
        if "updown" in context
    ]


def load_rerun_helpers():
    return rerun_helpers


class LiveExecutionClient(Node):
    def __init__(
        self,
        *,
        action_name: str,
        helpers: Any,
        robot: Any,
        snapshot: dict[str, Any],
        run_dir: Path,
        box_front_x: float,
        scene_y_shift: float,
        hz: float,
        updown_topic: str,
        max_updown_speed_m_s: float,
        updown_acceleration_m_s2: float,
        updown_deceleration_m_s2: float,
    ) -> None:
        super().__init__("alfa_l6_r8_live_executor")
        self.client = ActionClient(self, FollowJointTrajectory, action_name)
        self.updown_pub = self.create_publisher(Float64MultiArray, updown_topic, 10) if updown_topic else None
        self.helpers = helpers
        self.robot = robot
        self.snapshot = snapshot
        # 集装箱壳几何在一次流程内恒定，取 snapshot 顶层 container_panels（与 MoveIt
        # 规划场景同源，含 yaw），可视化不再硬编码 center_x/width/height。
        self.container_panels = snapshot.get("container_panels")
        self.run_dir = run_dir
        self.box_front_x = box_front_x
        self.scene_y_shift = scene_y_shift
        self.hz = hz
        self.updown_topic = updown_topic
        self.max_updown_speed_m_s = max_updown_speed_m_s
        self.updown_acceleration_m_s2 = updown_acceleration_m_s2
        self.updown_deceleration_m_s2 = updown_deceleration_m_s2
        self.sample = 0
        self.last_context: dict[str, Any] = {}
        self.feedback_count = 0
        self._lock = threading.Lock()

    def actual_positions_to_execution_order(self, feedback, *, from_ethercat_signs: bool) -> list[float]:
        joint_names = list(getattr(feedback, "joint_names", []))
        positions = list(feedback.actual.positions)
        if joint_names and len(joint_names) == len(positions):
            name_to_position = dict(zip(joint_names, positions))
            if from_ethercat_signs:
                return [
                    ethercat_to_ros_position(name, float(name_to_position.get(name, 0.0)))
                    for name in EXECUTION_JOINT_NAMES
                ]
            return [float(name_to_position.get(name, 0.0)) for name in EXECUTION_JOINT_NAMES]
        if len(positions) == len(EXECUTION_JOINT_NAMES):
            return [float(value) for value in positions]
        return [0.0] * len(EXECUTION_JOINT_NAMES)

    def log_static_scene(self) -> None:
        monitor.rr.log("monitor", monitor.rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        self.helpers.log_robot_static_model(self.robot, "monitor/robot", log_meshes=True)
        monitor.log_container_panels(self.container_panels)

    def log_positions(self, positions: list[float], context: dict[str, Any] | None = None) -> None:
        context = context or self.last_context
        with self._lock:
            self.helpers.set_sample_time(self.sample)
            monitor.log_container_panels(self.container_panels)
            monitor.log_static_box_obstacles(context.get("static_box_obstacles"))
            joint_map = execution_to_rerun_joint_map(positions, updown=float(context.get("updown", 0.3)))
            self.helpers.log_robot_state(self.robot, joint_map, "monitor/robot")
            monitor.log_attached_boxes(self.robot, joint_map, context.get("attached_boxes", []))
            monitor.rr.log(
                "monitor/info",
                monitor.rr.TextLog(
                    f"{context.get('label', 'execute')} | "
                    f"stage={context.get('stage', '')} | "
                    f"feedback={self.feedback_count}"
                ),
            )
            self.sample += 1

    def send_and_wait(
        self,
        trajectory: JointTrajectory,
        contexts: list[dict[str, Any]],
        label: str,
        *,
        feedback_uses_ethercat_signs: bool = False,
        updown_samples: list[tuple[float, float]] | None = None,
    ) -> bool:
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("execution action server is not available")
            return False
        if not trajectory.points:
            self.get_logger().error("empty trajectory")
            return False

        context_by_time = [
            (duration_to_seconds(point.time_from_start), context)
            for point, context in zip(trajectory.points, contexts)
        ]

        def feedback_callback(msg) -> None:
            feedback = msg.feedback
            elapsed = duration_to_seconds(feedback.actual.time_from_start)
            context = contexts[-1] if contexts else {"label": label}
            for sample_time, sample_context in context_by_time:
                if sample_time <= elapsed + 1e-6:
                    context = sample_context
                else:
                    break
            context = dict(context)
            context["label"] = label
            self.feedback_count += 1
            self.log_positions(
                self.actual_positions_to_execution_order(
                    feedback,
                    from_ethercat_signs=feedback_uses_ethercat_signs,
                ),
                context,
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        if self.updown_pub is not None and updown_samples:
            velocity_mps = synchronized_updown_velocity_mps(
                updown_samples,
                self.max_updown_speed_m_s,
            )
            if velocity_mps is not None:
                self._publish_updown_target(updown_samples[-1][1], velocity_mps)
                self.get_logger().info(
                    f"{label}: updown PP target={updown_samples[-1][1]:.4f}m "
                    f"velocity={velocity_mps:.4f}m/s "
                    f"accel={self.updown_acceleration_m_s2:.4f}m/s^2 "
                    f"decel={self.updown_deceleration_m_s2:.4f}m/s^2"
                )
        future = self.client.send_goal_async(goal, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{label}: goal rejected")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        ok = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        if not ok:
            self.get_logger().error(f"{label}: failed {result.error_code} {result.error_string}")
        return ok

    def _publish_updown_target(self, updown_m: float, velocity_mps: float) -> None:
        if self.updown_pub is None:
            return
        msg = Float64MultiArray()
        msg.data = make_updown_command_data(
            updown_m,
            velocity_mps,
            self.updown_acceleration_m_s2,
            self.updown_deceleration_m_s2,
        )
        self.updown_pub.publish(msg)


def build_planner_args(args: argparse.Namespace, run_dir: Path, snapshot_path: Path) -> SimpleNamespace:
    left_mode = normalize_grasp_mode(getattr(args, "left_grasp_mode", "auto"))
    right_mode = normalize_grasp_mode(getattr(args, "right_grasp_mode", "auto"))
    if left_mode == "auto":
        left_mode = grasp_mode_for_box(args.left_box_id)
    if right_mode == "auto":
        right_mode = grasp_mode_for_box(args.right_box_id)
    mode = pair_vehicle_mode(left_mode, right_mode)
    box_front_x = effective_box_front_x(args, mode)
    lateral_shift_enabled = left_mode == "front" or right_mode == "front"

    return SimpleNamespace(
        box_front_x=box_front_x,
        top_box_front_x=box_front_x,
        top_approach_forward=0.0,
        scene_y_shift=args.scene_y_shift,
        fixed_updown=args.fixed_updown,
        turn_rad=0.0,
        grasp_mode=mode,
        left_grasp_mode=left_mode,
        right_grasp_mode=right_mode,
        front_z_reach_lower=args.front_z_reach_lower,
        front_z_reach_upper=args.front_z_reach_upper,
        top_z_reach_lower=getattr(args, "top_z_reach_lower", 0.0),
        top_z_reach_upper=getattr(args, "top_z_reach_upper", 0.45),
        top_suction_x_offset=getattr(args, "top_suction_x_offset", 0.15),
        top_suction_z_offset=getattr(args, "top_suction_z_offset", 0.25),
        ik_top_position_tolerance=getattr(args, "ik_top_position_tolerance", 0.04),
        ik_top_orientation_tolerance_deg=getattr(args, "ik_top_orientation_tolerance_deg", 7.0),
        ik_h_candidate_count=getattr(args, "ik_h_candidate_count", 64),
        ik_h_lower=getattr(args, "ik_h_lower", 0.0),
        ik_h_upper=getattr(args, "ik_h_upper", 0.7),
        ik_h_step=getattr(args, "ik_h_step", 0.01),
        ik_full_h_range_scan=getattr(args, "ik_full_h_range_scan", False),
        ik_seed_count=getattr(args, "ik_seed_count", 32),
        ik_workers=getattr(args, "ik_workers", 1),
        ik_candidate_timeout=getattr(args, "ik_candidate_timeout", 0.01),
        ik_try_target_orders=getattr(args, "ik_try_target_orders", False),
        ik_use_reversed_target_order=getattr(args, "ik_use_reversed_target_order", True),
        optimized_ik_check_collision=getattr(args, "optimized_ik_check_collision", True),
        left_box_id=args.left_box_id,
        right_box_id=args.right_box_id,
        extract_workers=args.extract_workers,
        extract_success_quorum=getattr(args, "extract_success_quorum", 3),
        extract_quality_success_quorum=getattr(args, "extract_quality_success_quorum", 1),
        extract_quality_loaded_distance_sum=getattr(args, "extract_quality_loaded_distance_sum", 5.0),
        candidate_limit=args.candidate_limit,
        extract_step_x=getattr(args, "extract_step_x", 0.03),
        extract_max_joint_delta=getattr(args, "extract_max_joint_delta", 10.0 * math.pi / 180.0),
        extract_rrt=getattr(args, "extract_rrt", False),
        extract_rrt_planning_group=getattr(args, "extract_rrt_planning_group", "dual_arm"),
        extract_rrt_planning_time=getattr(args, "extract_rrt_planning_time", 0.35),
        extract_rrt_planning_attempts=getattr(args, "extract_rrt_planning_attempts", 1),
        extract_rrt_endpoint_per_arm_limit=getattr(args, "extract_rrt_endpoint_per_arm_limit", 8),
        extract_rrt_goal_limit=getattr(args, "extract_rrt_goal_limit", 8),
        extract_rollout_mode=getattr(args, "extract_rollout_mode", "box_pose_rrt"),
        extract_box_pose_rrt_edge_scene_collision=getattr(args, "extract_box_pose_rrt_edge_scene_collision", True),
        extract_box_pose_rrt_max_iterations=getattr(args, "extract_box_pose_rrt_max_iterations", 160),
        extract_box_pose_rrt_paths_per_arm=getattr(args, "extract_box_pose_rrt_paths_per_arm", 8),
        extract_box_pose_rrt_path_pair_limit=getattr(args, "extract_box_pose_rrt_path_pair_limit", 64),
        extract_box_pose_rrt_parent_candidates=getattr(args, "extract_box_pose_rrt_parent_candidates", 8),
        extract_box_pose_rrt_parent_diverse_candidates=getattr(args, "extract_box_pose_rrt_parent_diverse_candidates", 0),
        extract_box_pose_rrt_parent_endpoint_score_weight=getattr(args, "extract_box_pose_rrt_parent_endpoint_score_weight", 0.05),
        extract_box_pose_rrt_parent_node_score_weight=getattr(args, "extract_box_pose_rrt_parent_node_score_weight", 0.0),
        extract_box_pose_rrt_parent_density_weight=getattr(args, "extract_box_pose_rrt_parent_density_weight", 0.0),
        extract_box_pose_rrt_max_lateral=getattr(args, "extract_box_pose_rrt_max_lateral", 0.0),
        extract_box_pose_rrt_step_lateral=getattr(args, "extract_box_pose_rrt_step_lateral", 0.02),
        extract_box_pose_rrt_front_free_motion=getattr(args, "extract_box_pose_rrt_front_free_motion", True),
        extract_box_pose_rrt_front_goal_requires_max_pitch=getattr(args, "extract_box_pose_rrt_front_goal_requires_max_pitch", False),
        extract_box_pose_rrt_best_first_fallback=getattr(args, "extract_box_pose_rrt_best_first_fallback", True),
        extract_box_pose_rrt_best_first_first=getattr(args, "extract_box_pose_rrt_best_first_first", False),
        extract_box_pose_rrt_top_best_first_first=getattr(args, "extract_box_pose_rrt_top_best_first_first", False),
        extract_box_pose_rrt_best_first_max_expansions=getattr(args, "extract_box_pose_rrt_best_first_max_expansions", 800),
        extract_box_pose_rrt_best_first_heuristic_weight=getattr(args, "extract_box_pose_rrt_best_first_heuristic_weight", 1.0),
        dedup_joint_threshold_deg=args.dedup_joint_threshold_deg,
        dedup_h_threshold=args.dedup_h_threshold,
        extract_ik_stratified_limit_enabled=getattr(args, "extract_ik_stratified_limit_enabled", False),
        extract_ik_stratified_h_bucket=getattr(args, "extract_ik_stratified_h_bucket", 0.05),
        extract_ik_stratified_top_score_count=getattr(args, "extract_ik_stratified_top_score_count", 12),
        extract_ik_candidate_reserve_limit=getattr(args, "extract_ik_candidate_reserve_limit", 64),
        extract_ik_candidate_reserve_stratified=getattr(args, "extract_ik_candidate_reserve_stratified", True),
        extract_ik_candidate_reserve_interleave_stride=getattr(args, "extract_ik_candidate_reserve_interleave_stride", 4),
        extract_ik_loaded_distance_order_weight=getattr(args, "extract_ik_loaded_distance_order_weight", 0.0),
        ik_only_raw=False,
        extract_monitor_build_final_replay=True,
        place_cycle_enabled=getattr(args, "place_cycle_enabled", True),
        place_updown=getattr(args, "place_updown", 0.10),
        place_left_pose_deg=getattr(
            args, "place_left_pose_deg", "[0.0,-55.0,-50.0,-60.0,0.0,0.0]"
        ),
        place_right_pose_deg=getattr(
            args, "place_right_pose_deg", "[0.0,-55.0,-50.0,-60.0,0.0,0.0]"
        ),
        loaded_candidate_limit=args.loaded_candidate_limit,
        lateral_shift_enabled=lateral_shift_enabled,
        lateral_shift_distance=args.lateral_shift_distance,
        lateral_shift_step=args.lateral_shift_step,
        lateral_shift_column=args.lateral_shift_column,
        pre_lower_left_box_id=args.pre_lower_left_box_id,
        pre_lower_right_box_id=args.pre_lower_right_box_id,
        pre_lower_updown_delta=args.pre_lower_updown_delta,
        loaded_updown=args.loaded_updown,
        loaded_planner_id=getattr(args, "loaded_planner_id", ""),
        loaded_planning_mode=getattr(args, "loaded_planning_mode", "shortcut"),
        loaded_planning_time=args.loaded_planning_time,
        loaded_planning_attempts=args.loaded_planning_attempts,
        loaded_workers=args.loaded_workers,
        loaded_sort_by_pose_distance=getattr(args, "loaded_sort_by_pose_distance", True),
        loaded_stop_on_first_success=getattr(args, "loaded_stop_on_first_success", False),
        loaded_preferred_pose_index=args.loaded_preferred_pose_index,
        loaded_left_pose_family_deg="[0.0,-45.0,120.0,-75.0,0.0,0.0]",
        loaded_right_pose_family_deg="[0.0,-45.0,120.0,-75.0,0.0,0.0]",
    )


def compute_snapshot(args: argparse.Namespace, run_dir: Path) -> Path:
    snapshot_path = run_dir / "stage_snapshot.json"
    launch_log = run_dir / "planner.log"
    planner_args = build_planner_args(args, run_dir, snapshot_path)
    launch_command = monitor.build_launch_command(planner_args, run_dir, snapshot_path)
    with launch_log.open("w") as log_file:
        planner = subprocess.Popen(
            bash_source_command(launch_command),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )
    try:
        monitor.wait_for_service(
            "/dual_arm_planner/run_extract_monitor_full_selected",
            planner,
            args.service_timeout,
            launch_log,
        )
        prewarm_ok, prewarm_output, prewarm_ms = monitor.call_configure_extract_monitor_service(
            "/dual_arm_planner/configure_extract_monitor",
            args.left_box_id,
            args.right_box_id,
            snapshot_path,
            args.service_timeout,
            normalize_grasp_mode(args.left_grasp_mode) == "top_suction"
            or (normalize_grasp_mode(args.left_grasp_mode) == "auto" and grasp_mode_for_box(args.left_box_id) == "top_suction"),
            normalize_grasp_mode(args.right_grasp_mode) == "top_suction"
            or (normalize_grasp_mode(args.right_grasp_mode) == "auto" and grasp_mode_for_box(args.right_box_id) == "top_suction"),
        )
        print(prewarm_output, flush=True)
        if not prewarm_ok:
            raise RuntimeError(f"IK solver 预热失败：{prewarm_output}")
        print(f"planner 启动完成，IK solver 已预热：{prewarm_ms:.1f}ms", flush=True)
        print("开始计算 L6/R8：IK → 抽离 → 横向让位 → 负重规划", flush=True)
        start = time.monotonic()
        success, output, elapsed_ms = monitor.call_trigger_service(
            "/dual_arm_planner/run_extract_monitor_full_selected",
            args.compute_timeout,
        )
        print(output, flush=True)
        print(f"计算完成：success={success} service={elapsed_ms:.1f}ms wall={(time.monotonic()-start)*1000.0:.1f}ms", flush=True)
        if not success:
            raise RuntimeError(output)
        returned = monitor.extract_snapshot_path_from_service_output(output)
        if returned is not None and returned != snapshot_path:
            raise RuntimeError(f"connected to stale planner: expected={snapshot_path}, got={returned}")
        return snapshot_path
    finally:
        terminate_process(planner)
        monitor.wait_until_planner_services_gone(15.0)


def start_execution_bridge(run_dir: Path, hz: float, config_name: str) -> subprocess.Popen[str]:
    log_path = run_dir / "execution_bridge.log"
    command = (
        "ros2 run alfa_robot_execution_bridge execution_bridge_node "
        "--ros-args "
        f"--params-file {ROS_WS}/install/alfa_robot_execution_bridge/share/alfa_robot_execution_bridge/config/{config_name} "
        f"-p update_hz:={hz}"
    )
    with log_path.open("w") as log_file:
        return subprocess.Popen(
            bash_source_command(command),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )


def wait_for_action_server(action_name: str, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        result = subprocess.run(
            bash_source_command("ros2 action list"),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        if action_name in result.stdout.splitlines():
            return
        time.sleep(0.2)
    raise TimeoutError(f"action {action_name} not available")


def read_joint_position_once(topic: str, joint_name: str, timeout_s: float) -> float:
    holder: dict[str, float] = {}
    if not rclpy.ok():
        rclpy.init()
        owns_context = True
    else:
        owns_context = False
    node = rclpy.create_node("alfa_read_joint_position_once")

    def on_msg(msg: JointState) -> None:
        if joint_name not in msg.name:
            return
        index = list(msg.name).index(joint_name)
        if index < len(msg.position):
            holder["value"] = float(msg.position[index])

    subscription = node.create_subscription(JointState, topic, on_msg, JOINT_STATE_QOS)
    try:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline and "value" not in holder:
            rclpy.spin_once(node, timeout_sec=0.1)
        if "value" not in holder:
            raise TimeoutError(f"timed out waiting for {joint_name!r} on {topic}")
        return holder["value"]
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        if owns_context and rclpy.ok():
            rclpy.shutdown()


def read_current_execution_joint_map(topic: str, timeout_s: float) -> dict[str, float]:
    holder: dict[str, dict[str, float]] = {}
    if not rclpy.ok():
        rclpy.init()
        owns_context = True
    else:
        owns_context = False
    node = rclpy.create_node("alfa_read_execution_joints_once")

    def on_msg(msg: JointState) -> None:
        values: dict[str, float] = {}
        for index, name in enumerate(msg.name):
            if index >= len(msg.position):
                continue
            execution_name = moveit_to_execution_name(str(name))
            if execution_name in EXECUTION_JOINT_NAMES:
                values[execution_name] = ethercat_to_ros_position(
                    execution_name, float(msg.position[index])
                )
        if all(name in values for name in EXECUTION_JOINT_NAMES):
            holder["values"] = values

    subscription = node.create_subscription(JointState, topic, on_msg, JOINT_STATE_QOS)
    try:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline and "values" not in holder:
            rclpy.spin_once(node, timeout_sec=0.1)
        if "values" not in holder:
            raise TimeoutError(f"timed out waiting for {len(EXECUTION_JOINT_NAMES)} execution joints on {topic}")
        return holder["values"]
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        if owns_context and rclpy.ok():
            rclpy.shutdown()


def parse_args(default_executor_mode: str = "mock") -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compute L6/R8 then execute it with live Rerun feedback.")
    parser.add_argument("--executor-mode", choices=["mock", "real"], default=default_executor_mode)
    parser.add_argument(
        "--start-execution-bridge",
        action="store_true",
        help="real 模式下也启动 alfa_robot_execution_bridge 并发 /alfa_execution；默认直接发真实控制器 action",
    )
    parser.add_argument("--output-root", type=Path, default=None)
    parser.add_argument("--left-box-id", type=int, default=6)
    parser.add_argument("--right-box-id", type=int, default=8)
    parser.add_argument("--left-grasp-mode", default="auto", choices=["auto", "front", "top_suction", "top", "side"])
    parser.add_argument("--right-grasp-mode", default="auto", choices=["auto", "front", "top_suction", "top", "side"])
    parser.add_argument("--box-front-x", type=float, default=0.925)
    parser.add_argument("--top-approach-forward", type=float, default=0.30)
    parser.add_argument("--top-box-front-x", type=float, default=None)
    parser.add_argument("--scene-y-shift", type=float, default=0.0)
    parser.add_argument("--fixed-updown", type=float, default=0.3)
    parser.add_argument(
        "--loaded-updown",
        type=float,
        default=0.3,
        help="初始化及任务结束时的负重高度；与当前 IK 参考 fixed_updown 分离",
    )
    parser.add_argument("--front-z-reach-lower", type=float, default=0.45)
    parser.add_argument("--front-z-reach-upper", type=float, default=1.25)
    parser.add_argument("--top-z-reach-lower", type=float, default=0.0)
    parser.add_argument("--top-z-reach-upper", type=float, default=0.45)
    parser.add_argument("--top-suction-x-offset", type=float, default=0.15)
    parser.add_argument("--top-suction-z-offset", type=float, default=0.25)
    parser.add_argument("--ik-top-position-tolerance", type=float, default=0.04)
    parser.add_argument("--ik-top-orientation-tolerance-deg", type=float, default=7.0)
    parser.add_argument("--ik-h-candidate-count", type=int, default=64)
    parser.add_argument("--ik-h-lower", type=float, default=0.0)
    parser.add_argument("--ik-h-upper", type=float, default=0.7)
    parser.add_argument("--ik-h-step", type=float, default=0.01)
    # full-h-range-scan 会把整个 [0,0.7] 按 0.01 步长扫 ~70 个高度,再对每个高度做
    # 左×右解析解笛卡尔积,产出上百个候选喂给重型 box_pose_rrt 抽离 -> 真机 180s 超时。
    # 离线 13/13 基线(extract_stage_monitor_console/extract_sequence_rerun)用的是
    # false: 只在"由目标反推的可达 h 区间"里按 h_search_margin 取少量高度,候选 ~20 个。
    # 这里对齐那套被验证过的原方案,默认 false。
    parser.add_argument("--ik-full-h-range-scan", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--ik-seed-count", type=int, default=32)
    parser.add_argument("--ik-workers", type=int, default=1)
    parser.add_argument("--ik-candidate-timeout", type=float, default=0.01)
    parser.add_argument("--ik-try-target-orders", action="store_true")
    parser.add_argument("--ik-use-reversed-target-order", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--optimized-ik-check-collision", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--candidate-limit", type=int, default=64)
    parser.add_argument("--extract-workers", type=int, default=16)
    parser.add_argument("--extract-success-quorum", type=int, default=3)
    parser.add_argument("--extract-quality-success-quorum", type=int, default=1)
    parser.add_argument("--extract-quality-loaded-distance-sum", type=float, default=5.0)
    parser.add_argument("--extract-step-x", type=float, default=0.03)
    parser.add_argument("--extract-max-joint-delta", type=float, default=10.0 * math.pi / 180.0)
    parser.add_argument(
        "--extract-rollout-mode",
        choices=["greedy", "box_pose_rrt", "moveit_rrt_legacy", "top_lift_legacy"],
        default="box_pose_rrt",
    )
    parser.add_argument("--extract-rrt", action="store_true")
    parser.add_argument("--extract-box-pose-rrt-edge-scene-collision", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--extract-box-pose-rrt-max-iterations", type=int, default=160)
    parser.add_argument("--extract-box-pose-rrt-paths-per-arm", type=int, default=8)
    parser.add_argument("--extract-box-pose-rrt-path-pair-limit", type=int, default=64)
    parser.add_argument("--extract-box-pose-rrt-parent-candidates", type=int, default=8)
    parser.add_argument("--extract-box-pose-rrt-parent-diverse-candidates", type=int, default=0)
    parser.add_argument("--extract-box-pose-rrt-parent-endpoint-score-weight", type=float, default=0.05)
    parser.add_argument("--extract-box-pose-rrt-parent-node-score-weight", type=float, default=0.0)
    parser.add_argument("--extract-box-pose-rrt-parent-density-weight", type=float, default=0.0)
    parser.add_argument("--extract-box-pose-rrt-max-lateral", type=float, default=0.0)
    parser.add_argument("--extract-box-pose-rrt-step-lateral", type=float, default=0.02)
    parser.add_argument("--extract-box-pose-rrt-front-free-motion", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--extract-box-pose-rrt-front-goal-requires-max-pitch", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--extract-box-pose-rrt-best-first-fallback", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--extract-box-pose-rrt-best-first-first", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--extract-box-pose-rrt-top-best-first-first", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--extract-box-pose-rrt-best-first-max-expansions", type=int, default=800)
    parser.add_argument("--extract-box-pose-rrt-best-first-heuristic-weight", type=float, default=1.0)
    parser.add_argument("--extract-rrt-planning-group", default="dual_arm")
    parser.add_argument("--extract-rrt-planning-time", type=float, default=0.35)
    parser.add_argument("--extract-rrt-planning-attempts", type=int, default=1)
    parser.add_argument("--extract-rrt-endpoint-per-arm-limit", type=int, default=8)
    parser.add_argument("--extract-rrt-goal-limit", type=int, default=8)
    parser.add_argument("--extract-ik-stratified-limit-enabled", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--extract-ik-stratified-h-bucket", type=float, default=0.05)
    parser.add_argument("--extract-ik-stratified-top-score-count", type=int, default=12)
    parser.add_argument("--extract-ik-candidate-reserve-limit", type=int, default=64)
    parser.add_argument("--extract-ik-candidate-reserve-stratified", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--extract-ik-candidate-reserve-interleave-stride", type=int, default=4)
    parser.add_argument("--extract-ik-loaded-distance-order-weight", type=float, default=0.0)
    parser.add_argument("--loaded-candidate-limit", type=int, default=8)
    parser.add_argument("--loaded-workers", type=int, default=8)
    parser.add_argument("--loaded-planner-id", default="")
    parser.add_argument("--loaded-planning-mode", choices=["rrt", "shortcut"], default="shortcut")
    parser.add_argument(
        "--loaded-preferred-pose-index",
        type=int,
        default=0,
        help="负重姿态族索引；0 是当前实机确认的安全姿态，1 是旧的 [-75,135,60] 姿态",
    )
    parser.add_argument("--loaded-planning-time", type=float, default=1.0)
    parser.add_argument("--loaded-planning-attempts", type=int, default=8)
    parser.add_argument("--loaded-sort-by-pose-distance", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--loaded-stop-on-first-success", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--place-cycle-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--place-updown", type=float, default=0.10)
    parser.add_argument(
        "--place-left-pose-deg",
        default="[0.0,-55.0,-50.0,-60.0,0.0,0.0]",
    )
    parser.add_argument(
        "--place-right-pose-deg",
        default="[0.0,-55.0,-50.0,-60.0,0.0,0.0]",
    )
    parser.add_argument("--lateral-shift-distance", type=float, default=0.5)
    parser.add_argument("--lateral-shift-step", type=float, default=0.01)
    parser.add_argument("--lateral-shift-column", type=int, default=2)
    parser.add_argument("--pre-lower-left-box-id", type=int, default=0)
    parser.add_argument("--pre-lower-right-box-id", type=int, default=0)
    parser.add_argument("--pre-lower-updown-delta", type=float, default=0.0)
    parser.add_argument("--dedup-joint-threshold-deg", type=float, default=1.0)
    parser.add_argument("--dedup-h-threshold", type=float, default=0.005)
    parser.add_argument("--service-timeout", type=float, default=120.0)
    # 计算本身(run_extract_monitor_full_selected)的独立超时,与"等planner启动"分开:
    # planner 起 move_group+IK预热要 30~40s,不能用它卡计算;而计算(候选收敛后)应几秒内出,
    # 给它一个较紧的上限让真卡死时快速失败,不空等 service-timeout 那么久。默认 30s。
    parser.add_argument("--compute-timeout", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--max-joint-speed-deg-s", type=float, default=10.0)
    parser.add_argument("--max-updown-speed-m-s", type=float, default=0.05)
    parser.add_argument(
        "--updown-acceleration-m-s2",
        type=float,
        default=DEFAULT_UPDOWN_ACCELERATION_MPS2,
    )
    parser.add_argument(
        "--updown-deceleration-m-s2",
        type=float,
        default=DEFAULT_UPDOWN_DECELERATION_MPS2,
    )
    parser.add_argument(
        "--trajectory-timing-source",
        choices=["snapshot", "python"],
        default="snapshot",
        help="snapshot=保留 C++ planner 写入的 time_from_start；python=旧逻辑按最大关节速度二次重采样",
    )
    parser.add_argument(
        "--fixed-updown-from-joint-states",
        action="store_true",
        help="规划前从 --updown-joint-state-topic 读取 updown，并覆盖 --fixed-updown",
    )
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument(
        "--updown-joint-state-topic",
        default="/joint_states",
        help="updown 现已合并进 /joint_states(与12臂+turn同一个topic);"
             "该topic上的updown是电机物理值,读取后经 physical_to_logical_updown 转逻辑值",
    )
    parser.add_argument("--joint-state-timeout-s", type=float, default=3.0)
    parser.add_argument(
        "--home-start",
        choices=["current", "zero"],
        default="current",
        help="负重姿态过渡的起点；实机默认 current，避免旧逻辑强行全0起步",
    )
    parser.add_argument(
        "--send-updown",
        action="store_true",
        help="real 直连时按同一时间轴向 updown topic 发布位置命令",
    )
    parser.add_argument("--updown-command-topic", default="/canopen/updown_position_controller/commands")
    parser.add_argument(
        "--action-name",
        default=None,
        help="不设置时：mock 用 /alfa_execution/execute_joint_trajectory；real 默认用 /dual_arm_trajectory_controller/follow_joint_trajectory",
    )
    parser.add_argument(
        "--real-controller-order",
        choices=["right_first", "left_first"],
        default="right_first",
        help="real 直连控制器 joint_names 顺序；工控机当前 dual_arm_trajectory_controller 为 right_first",
    )
    parser.add_argument(
        "--real-apply-direction-signs",
        dest="real_apply_direction_signs",
        action="store_true",
        default=True,
        help="real 直连时按 EtherCAT 方向标定表翻转目标；默认开启",
    )
    parser.add_argument(
        "--no-real-apply-direction-signs",
        dest="real_apply_direction_signs",
        action="store_false",
        help="real 直连时不做方向映射，仅允许独立小角度诊断，不允许 L6/R8 实机流程使用",
    )
    parser.add_argument("--save", type=Path, default=None, help="保存为 .rrd；不设置时默认打开实时 Rerun viewer")
    parser.add_argument("--connect", action="store_true", help="连接已有 Rerun viewer，而不是新开 viewer")
    parser.add_argument(
        "--ros-domain-id",
        default="auto",
        help="本次 ROS_DOMAIN_ID；auto 隔离自启动测试，inherit 表示沿用当前终端。",
    )
    args = parser.parse_args()
    make_updown_command_data(
        0.0,
        args.max_updown_speed_m_s,
        args.updown_acceleration_m_s2,
        args.updown_deceleration_m_s2,
    )
    real_direct = args.executor_mode == "real" and not args.start_execution_bridge
    if (
        real_direct
        and not args.real_apply_direction_signs
        and os.environ.get("ALFA_ALLOW_UNSAFE_DIRECTION_OVERRIDE") != "I_UNDERSTAND_DIRECTION_RISK"
    ):
        raise SystemExit(
            "禁止 real direct L6/R8 流程关闭方向映射：这会导致实机方向反。"
            "如需诊断，必须使用独立小角度脚本；若确需绕过，显式设置 "
            "ALFA_ALLOW_UNSAFE_DIRECTION_OVERRIDE=I_UNDERSTAND_DIRECTION_RISK。"
        )
    if (
        real_direct
        and args.max_joint_speed_deg_s > MAX_SAFE_REAL_JOINT_SPEED_DEG_S
        and os.environ.get("ALFA_ALLOW_UNSAFE_SPEED_OVERRIDE") != "I_UNDERSTAND_SPEED_RISK"
    ):
        raise SystemExit(
            f"禁止 real direct L6/R8 流程使用 --max-joint-speed-deg-s > {MAX_SAFE_REAL_JOINT_SPEED_DEG_S}："
            f"当前值 {args.max_joint_speed_deg_s}。"
            "若确需更高速度，先在仿真/mock模式验证，再显式设置 "
            "ALFA_ALLOW_UNSAFE_SPEED_OVERRIDE=I_UNDERSTAND_SPEED_RISK。"
        )
    if (
        real_direct
        and args.hz > MAX_SAFE_REAL_HZ
        and os.environ.get("ALFA_ALLOW_UNSAFE_HZ_OVERRIDE") != "I_UNDERSTAND_HZ_RISK"
    ):
        raise SystemExit(
            f"禁止 real direct L6/R8 流程使用 --hz > {MAX_SAFE_REAL_HZ}："
            f"当前值 {args.hz}。"
            "若确需更高频率，先在仿真/mock模式验证，再显式设置 "
            "ALFA_ALLOW_UNSAFE_HZ_OVERRIDE=I_UNDERSTAND_HZ_RISK。"
        )
    if (
        real_direct
        and args.send_updown
        and args.max_updown_speed_m_s > MAX_SAFE_REAL_UPDOWN_SPEED_M_S
        and os.environ.get("ALFA_ALLOW_UNSAFE_UPDOWN_SPEED_OVERRIDE") != "I_UNDERSTAND_UPDOWN_SPEED_RISK"
    ):
        raise SystemExit(
            f"禁止 real direct L6/R8 流程使用 --max-updown-speed-m-s > {MAX_SAFE_REAL_UPDOWN_SPEED_M_S}："
            f"当前值 {args.max_updown_speed_m_s}。"
            "若确需更高速度，先在仿真/mock模式验证，再显式设置 "
            "ALFA_ALLOW_UNSAFE_UPDOWN_SPEED_OVERRIDE=I_UNDERSTAND_UPDOWN_SPEED_RISK。"
        )
    if (
        real_direct
        and args.loaded_preferred_pose_index != 0
        and os.environ.get("ALFA_ALLOW_UNSAFE_LOADED_POSE_OVERRIDE") != "I_UNDERSTAND_LOADED_POSE_RISK"
    ):
        raise SystemExit(
            "禁止 real direct L6/R8 流程使用非 0 的 --loaded-preferred-pose-index："
            f"当前值 {args.loaded_preferred_pose_index}。"
            "index=0 是当前唯一在实机上确认过方向的负重姿态族。若确需切换，先离机验证，再显式设置 "
            "ALFA_ALLOW_UNSAFE_LOADED_POSE_OVERRIDE=I_UNDERSTAND_LOADED_POSE_RISK。"
        )
    if args.output_root is None:
        args.output_root = DEFAULT_REAL_OUTPUT_ROOT if args.executor_mode == "real" else DEFAULT_MOCK_OUTPUT_ROOT
    if args.action_name is None:
        if args.executor_mode == "real" and not args.start_execution_bridge:
            args.action_name = "/dual_arm_trajectory_controller/follow_joint_trajectory"
        else:
            args.action_name = "/alfa_execution/execute_joint_trajectory"
    return args


def main(default_executor_mode: str = "mock") -> int:
    args = parse_args(default_executor_mode)
    domain = process_lifecycle.configure_ros_domain(args.ros_domain_id)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = args.output_root / f"L{args.left_box_id}_R{args.right_box_id}_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    save_path = args.save
    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)

    os.environ["ROS_HOME"] = str(run_dir / "ros_home")
    os.environ["ROS_LOG_DIR"] = str(run_dir / "ros_log")
    Path(os.environ["ROS_HOME"]).mkdir(parents=True, exist_ok=True)
    Path(os.environ["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)

    bridge = None
    current_updown_logical: float | None = None
    try:
        print(f"ROS_DOMAIN_ID={domain if domain is not None else 'unset'}", flush=True)
        if args.fixed_updown_from_joint_states:
            raw_updown_physical = read_joint_position_once(
                args.updown_joint_state_topic,
                "updown",
                args.joint_state_timeout_s,
            )
            # /joint_states 上 updown 是电机物理值；仍经合同函数转成 URDF/MoveIt 逻辑值。
            # 当前实机标定为零偏移，因此数值不变，但禁止绕过合同边界。
            args.fixed_updown = physical_to_logical_updown(raw_updown_physical)
            current_updown_logical = args.fixed_updown
            print(
                f"从 {args.updown_joint_state_topic} 读取 updown 电机物理值="
                f"{raw_updown_physical:.6f}m -> 逻辑值={args.fixed_updown:.6f}m",
                flush=True,
            )
        if args.executor_mode == "mock":
            bridge = start_execution_bridge(run_dir, args.hz, "execution_bridge.yaml")
            print("mock执行桥启动中。", flush=True)
        elif args.start_execution_bridge:
            bridge = start_execution_bridge(run_dir, args.hz, "ros2_control_bridge.yaml")
            print("实机转发桥启动中：/alfa_execution -> /dual_arm_trajectory_controller/follow_joint_trajectory", flush=True)
        else:
            print(f"实机直连模式：等待真实控制器 action {args.action_name}", flush=True)
        wait_for_action_server(args.action_name, 15.0)
        print(f"执行接口已就绪：mode={args.executor_mode}, action={args.action_name}", flush=True)

        snapshot_path = compute_snapshot(args, run_dir)
        snapshot = monitor.read_snapshot(snapshot_path)
        print(f"计算快照：{snapshot_path}", flush=True)

        import rerun as rr

        monitor.rr = rr
        helpers = load_rerun_helpers()
        rr.init(f"l6_r8_{args.executor_mode}_live_execution")
        if save_path is not None:
            rr.save(str(save_path))
            print(f"Rerun 保存模式：{save_path}", flush=True)
        elif args.connect:
            rr.connect()
            print("Rerun 已连接已有 viewer。", flush=True)
        else:
            rr.spawn()
            print("Rerun 实时窗口已打开。", flush=True)
        robot = helpers.UrdfRobot(helpers.render_current_urdf())

        rclpy.init()
        client = LiveExecutionClient(
            action_name=args.action_name,
            helpers=helpers,
            robot=robot,
            snapshot=snapshot,
            run_dir=run_dir,
            box_front_x=args.box_front_x,
            scene_y_shift=args.scene_y_shift,
            hz=args.hz,
            updown_topic=(
                args.updown_command_topic
                if args.executor_mode == "real" and args.send_updown
                else ""
            ),
            max_updown_speed_m_s=args.max_updown_speed_m_s,
            updown_acceleration_m_s2=args.updown_acceleration_m_s2,
            updown_deceleration_m_s2=args.updown_deceleration_m_s2,
        )
        try:
            client.log_static_scene()
            if args.home_start == "current":
                zero = read_current_execution_joint_map(args.joint_state_topic, args.joint_state_timeout_s)
                print("负重过渡起点：当前 /joint_states 姿态", flush=True)
            else:
                zero = {name: 0.0 for name in EXECUTION_JOINT_NAMES}
                print("负重过渡起点：全0姿态", flush=True)
            loaded = loaded_joint_map(args.loaded_preferred_pose_index)
            if args.executor_mode == "real" and args.send_updown:
                if current_updown_logical is None:
                    raw_updown_physical = read_joint_position_once(
                        args.updown_joint_state_topic,
                        "updown",
                        args.joint_state_timeout_s,
                    )
                    current_updown_logical = physical_to_logical_updown(raw_updown_physical)
                print(
                    "初始化 updown："
                    f"current={current_updown_logical:.6f}m "
                    f"target={args.loaded_updown:.6f}m",
                    flush=True,
                )
            else:
                current_updown_logical = args.loaded_updown
            command_joint_names = (
                REAL_CONTROLLER_JOINT_NAMES
                if args.executor_mode == "real"
                and not args.start_execution_bridge
                and args.real_controller_order == "right_first"
                else EXECUTION_JOINT_NAMES
            )
            apply_ethercat_signs = (
                args.executor_mode == "real"
                and not args.start_execution_bridge
                and args.real_apply_direction_signs
            )
            home_updown_duration = (
                abs(args.loaded_updown - current_updown_logical) / args.max_updown_speed_m_s
                if args.send_updown else 0.0
            )
            home_samples = [
                (0.0, zero),
                *resample_segment(
                    zero,
                    loaded,
                    start_time=0.0,
                    hz=args.hz,
                    max_joint_speed_deg_s=args.max_joint_speed_deg_s,
                    minimum_duration_s=home_updown_duration,
                ),
            ]
            home_duration = home_samples[-1][0]
            home_updown_samples = [
                (
                    time_s,
                    current_updown_logical
                    + (args.loaded_updown - current_updown_logical)
                    * min(1.0, max(0.0, time_s / home_duration)),
                )
                for time_s, _ in home_samples
            ]
            home_contexts = [
                {
                    "label": "home_to_loaded",
                    "stage": "zero_to_loaded",
                    "attached_boxes": [],
                    "static_box_obstacles": [],
                    "updown": updown,
                }
                for (_, _), (_, updown) in zip(home_samples, home_updown_samples)
            ]
            if args.executor_mode == "real":
                print("实机手臂 action 将发送 12 个手臂关节 + turn=0。", flush=True)
                if args.send_updown:
                    print(f"updown 将同步发布到 topic：{args.updown_command_topic}", flush=True)
                else:
                    print("updown 发送关闭：仅按 fixed_updown 规划，不控制升降轴。", flush=True)
                print(f"负重姿态族索引：{args.loaded_preferred_pose_index}（0 为当前实机确认方向）", flush=True)
                print(f"实机方向映射：{'开启' if apply_ethercat_signs else '关闭'}", flush=True)
                print("实机 joint_names 顺序：" + ", ".join(command_joint_names), flush=True)
                wait_for_enter_confirmation(
                    "确认真实机器人当前状态安全、人员远离。按回车后先回到初始化负重姿态"
                )
            print("开始执行：当前姿态 → 负重姿态", flush=True)
            home_start = time.monotonic()
            if not client.send_and_wait(
                make_trajectory(
                    home_samples,
                    command_joint_names,
                    apply_ethercat_signs=apply_ethercat_signs,
                ),
                home_contexts,
                "home_to_loaded",
                feedback_uses_ethercat_signs=apply_ethercat_signs,
                updown_samples=(home_updown_samples if args.send_updown else None),
            ):
                return 1
            print(f"完成执行：当前姿态 → 负重姿态，用时 {(time.monotonic() - home_start):.3f}s", flush=True)

            if args.trajectory_timing_source == "python":
                task_samples_raw = trajectory_from_snapshot(
                    snapshot,
                    args.hz,
                    args.max_joint_speed_deg_s,
                    initial=loaded,
                )
            else:
                task_samples_raw = trajectory_from_snapshot_preserve_timing(
                    snapshot,
                    initial=loaded,
                    initial_updown=args.loaded_updown,
                    hz=args.hz,
                    max_joint_speed_deg_s=args.max_joint_speed_deg_s,
                    max_updown_speed_m_s=args.max_updown_speed_m_s,
                )
            if not task_samples_raw:
                raise RuntimeError("snapshot produced empty execution trajectory")
            pre_contact_phase, contact_phase, post_contact_phase = split_task_execution_phases(
                task_samples_raw
            )
            extract_phase, place_phase, return_phase = split_post_contact_place_cycle(
                post_contact_phase,
                required=args.place_cycle_enabled,
            )

            def execute_phase(
                phase_samples_raw: list[tuple[float, dict[str, float], dict[str, Any]]],
                label: str,
                description: str,
            ) -> bool:
                phase_samples = [
                    (time_s, joint_map) for time_s, joint_map, _ in phase_samples_raw
                ]
                phase_contexts = [context for _, _, context in phase_samples_raw]
                planned_duration = phase_samples[-1][0] - phase_samples[0][0]
                print(
                    f"开始执行：{description}，轨迹点 {len(phase_samples)}，"
                    f"计划时长 {planned_duration:.3f}s，timing={args.trajectory_timing_source}",
                    flush=True,
                )
                phase_start = time.monotonic()
                ok = client.send_and_wait(
                    make_trajectory(
                        phase_samples,
                        command_joint_names,
                        apply_ethercat_signs=apply_ethercat_signs,
                    ),
                    phase_contexts,
                    label,
                    feedback_uses_ethercat_signs=apply_ethercat_signs,
                    updown_samples=(
                        phase_updown_samples(phase_samples_raw)
                        if args.send_updown else None
                    ),
                )
                print(
                    f"{'完成' if ok else '失败'}执行：{description}，"
                    f"实际用时 {(time.monotonic() - phase_start):.3f}s",
                    flush=True,
                )
                return ok

            task_start = time.monotonic()
            if not execute_phase(
                pre_contact_phase,
                "loaded_to_pre_contact",
                "初始化负重姿态 → IK 前 5cm 预接触姿态",
            ):
                return 1
            expected_pre_contact_updown = float(pre_contact_phase[-1][2].get("updown", args.fixed_updown))
            if args.executor_mode == "real":
                wait_for_enter_confirmation(
                    "已到 IK 前 5cm 预接触姿态。请等待并确认 updown 已到目标位置 "
                    f"logical={expected_pre_contact_updown:.3f}m 后再继续"
                )
                raw_updown_physical = read_joint_position_once(
                    args.updown_joint_state_topic,
                    "updown",
                    args.joint_state_timeout_s,
                )
                actual_updown_logical = physical_to_logical_updown(raw_updown_physical)
                print(
                    "回车确认时 updown："
                    f"physical={raw_updown_physical:.6f}m "
                    f"logical={actual_updown_logical:.6f}m "
                    f"target={expected_pre_contact_updown:.6f}m "
                    f"error={actual_updown_logical - expected_pre_contact_updown:+.6f}m",
                    flush=True,
                )
            if not execute_phase(
                contact_phase,
                "pre_contact_to_grasp",
                "预接触姿态 → IK 吸附点（前进 5cm）",
            ):
                return 1
            if args.executor_mode == "real":
                wait_for_enter_confirmation(
                    "机械臂已到 IK 吸附点。确认吸附条件正常后，按回车开始抽离并回到负重姿态"
                )
            if not execute_phase(
                extract_phase,
                "extract_to_loaded",
                "IK 吸附点 → 抽离 → 负重姿态",
            ):
                return 1
            if place_phase:
                if not execute_phase(
                    place_phase,
                    "loaded_to_place",
                    "负重姿态 → 放货姿态",
                ):
                    return 1
                if args.executor_mode == "real":
                    wait_for_enter_confirmation(
                        "机械臂已到放货姿态。确认箱子已经释放且环境安全后，按回车返回负重初始姿态"
                    )
                if not execute_phase(
                    return_phase,
                    "place_to_loaded",
                    "放货姿态 → 负重初始姿态",
                ):
                    return 1
            print(
                f"完成执行：L{args.left_box_id}/R{args.right_box_id} 分段任务，"
                f"总用时 {(time.monotonic() - task_start):.3f}s",
                flush=True,
            )
            if save_path is not None:
                print(f"Rerun 已保存：{save_path}", flush=True)
            print(f"运行目录：{run_dir}", flush=True)
            return 0
        finally:
            client.destroy_node()
            rclpy.shutdown()
    finally:
        terminate_process(bridge)


if __name__ == "__main__":
    raise SystemExit(main())
