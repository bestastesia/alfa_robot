#!/usr/bin/python3
from __future__ import annotations

import argparse
import importlib.util
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
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


SCRIPT_DIR = Path(__file__).resolve().parent


def find_repo_root() -> Path:
    env_root = os.environ.get("ALFA_ROBOT_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve()
    for candidate in [SCRIPT_DIR, *SCRIPT_DIR.parents]:
        if (candidate / "ros2_ws").is_dir() and (candidate / "scripts/ik_benchmark").is_dir():
            return candidate
        if candidate.name == "ros2_ws":
            return candidate.parent
    return Path.cwd().resolve()


REPO_ROOT = find_repo_root()
ROS_WS = REPO_ROOT / "ros2_ws"
DEFAULT_MOCK_OUTPUT_ROOT = REPO_ROOT / "data/ik_benchmark/live_mock_execution"
DEFAULT_REAL_OUTPUT_ROOT = REPO_ROOT / "data/ik_benchmark/live_real_execution"
EXECUTION_JOINT_NAMES = [
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
]
REAL_CONTROLLER_JOINT_NAMES = [
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "turn",
]

EXECUTION_TO_ETHERCAT_SIGN = {
    "left_joint1": 1.0,
    "left_joint2": 1.0,
    "left_joint3": -1.0,
    "left_joint4": 1.0,
    "left_joint5": -1.0,
    "left_joint6": 1.0,
    "right_joint1": 1.0,
    "right_joint2": -1.0,
    "right_joint3": 1.0,
    "right_joint4": 1.0,
    "right_joint5": 1.0,
    "right_joint6": 1.0,
    "turn": 1.0,
}


if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
import extract_stage_monitor_console as monitor  # noqa: E402
import process_lifecycle  # noqa: E402


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
    if name.startswith("left_v5_joint"):
        return "left_joint" + name.removeprefix("left_v5_joint")
    if name.startswith("right_v5_joint"):
        return "right_joint" + name.removeprefix("right_v5_joint")
    if name == "turn":
        return "turn"
    return None


def execution_to_moveit_name(name: str) -> str:
    if name.startswith("left_joint"):
        return "left_v5_joint" + name.removeprefix("left_joint")
    if name.startswith("right_joint"):
        return "right_v5_joint" + name.removeprefix("right_joint")
    return name


def execution_to_rerun_joint_map(positions: list[float], updown: float = 0.3) -> dict[str, float]:
    joint_map = {"updown": updown, "pitch": 0.0, "turn": 0.0}
    for name, value in zip(EXECUTION_JOINT_NAMES, positions):
        joint_map[execution_to_moveit_name(name)] = float(value)
    return joint_map


def ros_to_ethercat_position(name: str, value: float) -> float:
    return float(value) * EXECUTION_TO_ETHERCAT_SIGN[name]


def ethercat_to_ros_position(name: str, value: float) -> float:
    return float(value) * EXECUTION_TO_ETHERCAT_SIGN[name]


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


def resample_segment(
    start: dict[str, float],
    goal: dict[str, float],
    *,
    start_time: float,
    hz: float,
    max_joint_speed_deg_s: float,
) -> list[tuple[float, dict[str, float]]]:
    max_delta = max(abs(goal[name] - start[name]) for name in EXECUTION_JOINT_NAMES)
    duration = max(1.0 / hz, max_delta / math.radians(max_joint_speed_deg_s))
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
    [0.0, 59.04, -135.16, 0.0, -76.13, 0.0],
    [0.0, -75.0, 135.0, 0.0, 60.0, 0.0],
    [33.87, 75.82, -135.08, 0.0, -59.25, -33.87],
]
LOADED_RIGHT_POSE_FAMILY_DEG = [
    [0.0, 58.88, -134.84, 0.0, -75.96, 0.0],
    [0.0, -75.0, 135.0, 0.0, 60.0, 0.0],
    [-30.93, 74.17, -134.92, 0.0, -60.74, 30.93],
]


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


def load_rerun_helpers():
    helper_path = REPO_ROOT / "scripts/ik_benchmark/scripts/visualize_rerun.py"
    spec = importlib.util.spec_from_file_location("alfa_visualize_rerun_helpers", helper_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {helper_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


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
    ) -> None:
        super().__init__("alfa_l6_r8_live_executor")
        self.client = ActionClient(self, FollowJointTrajectory, action_name)
        self.helpers = helpers
        self.robot = robot
        self.snapshot = snapshot
        self.run_dir = run_dir
        self.box_front_x = box_front_x
        self.scene_y_shift = scene_y_shift
        self.hz = hz
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
        monitor.log_default_container(self.scene_y_shift)
        monitor.log_box_stack(
            self.box_front_x,
            int(self.snapshot.get("left_box_id", 6)),
            int(self.snapshot.get("right_box_id", 8)),
            self.scene_y_shift,
        )

    def log_positions(self, positions: list[float], context: dict[str, Any] | None = None) -> None:
        context = context or self.last_context
        with self._lock:
            self.helpers.set_sample_time(self.sample)
            monitor.log_default_container(self.scene_y_shift)
            monitor.log_box_stack(
                self.box_front_x,
                int(self.snapshot.get("left_box_id", 6)),
                int(self.snapshot.get("right_box_id", 8)),
                self.scene_y_shift,
            )
            monitor.log_static_box_obstacles(context.get("static_box_obstacles"))
            joint_map = execution_to_rerun_joint_map(positions, updown=0.3)
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


def build_planner_args(args: argparse.Namespace, run_dir: Path, snapshot_path: Path) -> SimpleNamespace:
    return SimpleNamespace(
        box_front_x=args.box_front_x,
        scene_y_shift=args.scene_y_shift,
        fixed_updown=args.fixed_updown,
        front_z_reach_lower=args.front_z_reach_lower,
        front_z_reach_upper=args.front_z_reach_upper,
        left_box_id=args.left_box_id,
        right_box_id=args.right_box_id,
        extract_workers=args.extract_workers,
        candidate_limit=args.candidate_limit,
        dedup_joint_threshold_deg=args.dedup_joint_threshold_deg,
        dedup_h_threshold=args.dedup_h_threshold,
        loaded_candidate_limit=args.loaded_candidate_limit,
        lateral_shift_distance=args.lateral_shift_distance,
        lateral_shift_step=args.lateral_shift_step,
        lateral_shift_column=args.lateral_shift_column,
        pre_lower_left_box_id=args.pre_lower_left_box_id,
        pre_lower_right_box_id=args.pre_lower_right_box_id,
        pre_lower_updown_delta=args.pre_lower_updown_delta,
        loaded_planning_time=args.loaded_planning_time,
        loaded_planning_attempts=args.loaded_planning_attempts,
        loaded_workers=args.loaded_workers,
        loaded_preferred_pose_index=args.loaded_preferred_pose_index,
        extract_kdl_timeout=args.extract_kdl_timeout,
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
        )
        print(prewarm_output, flush=True)
        if not prewarm_ok:
            raise RuntimeError(f"IK solver 预热失败：{prewarm_output}")
        print(f"planner 启动完成，IK solver 已预热：{prewarm_ms:.1f}ms", flush=True)
        print("开始计算 L6/R8：IK → 抽离 → 横向让位 → 负重规划", flush=True)
        start = time.monotonic()
        success, output, elapsed_ms = monitor.call_trigger_service(
            "/dual_arm_planner/run_extract_monitor_full_selected",
            args.service_timeout,
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
    parser.add_argument("--box-front-x", type=float, default=0.925)
    parser.add_argument("--scene-y-shift", type=float, default=-0.4)
    parser.add_argument("--fixed-updown", type=float, default=0.3)
    parser.add_argument("--front-z-reach-lower", type=float, default=0.45)
    parser.add_argument("--front-z-reach-upper", type=float, default=1.25)
    parser.add_argument("--candidate-limit", type=int, default=64)
    parser.add_argument("--extract-workers", type=int, default=16)
    parser.add_argument("--loaded-candidate-limit", type=int, default=8)
    parser.add_argument("--loaded-workers", type=int, default=8)
    parser.add_argument(
        "--loaded-preferred-pose-index",
        type=int,
        default=0,
        help="负重姿态族索引；0 是当前实机确认的安全姿态，1 是旧的 [-75,135,60] 姿态",
    )
    parser.add_argument("--loaded-planning-time", type=float, default=1.0)
    parser.add_argument("--loaded-planning-attempts", type=int, default=8)
    parser.add_argument("--lateral-shift-distance", type=float, default=0.5)
    parser.add_argument("--lateral-shift-step", type=float, default=0.01)
    parser.add_argument("--lateral-shift-column", type=int, default=2)
    parser.add_argument("--pre-lower-left-box-id", type=int, default=0)
    parser.add_argument("--pre-lower-right-box-id", type=int, default=0)
    parser.add_argument("--pre-lower-updown-delta", type=float, default=0.0)
    parser.add_argument("--extract-kdl-timeout", type=float, default=0.003)
    parser.add_argument("--dedup-joint-threshold-deg", type=float, default=1.0)
    parser.add_argument("--dedup-h-threshold", type=float, default=0.005)
    parser.add_argument("--service-timeout", type=float, default=120.0)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--max-joint-speed-deg-s", type=float, default=45.0)
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
    if (
        args.executor_mode == "real"
        and not args.start_execution_bridge
        and not args.real_apply_direction_signs
        and os.environ.get("ALFA_ALLOW_UNSAFE_DIRECTION_OVERRIDE") != "I_UNDERSTAND_DIRECTION_RISK"
    ):
        raise SystemExit(
            "禁止 real direct L6/R8 流程关闭方向映射：这会导致实机方向反。"
            "如需诊断，必须使用独立小角度脚本；若确需绕过，显式设置 "
            "ALFA_ALLOW_UNSAFE_DIRECTION_OVERRIDE=I_UNDERSTAND_DIRECTION_RISK。"
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
    try:
        print(f"ROS_DOMAIN_ID={domain if domain is not None else 'unset'}", flush=True)
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
        )
        try:
            client.log_static_scene()
            zero = {name: 0.0 for name in EXECUTION_JOINT_NAMES}
            loaded = loaded_joint_map(args.loaded_preferred_pose_index)
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
            home_samples = [
                (0.0, zero),
                *resample_segment(
                    zero,
                    loaded,
                    start_time=0.0,
                    hz=args.hz,
                    max_joint_speed_deg_s=args.max_joint_speed_deg_s,
                ),
            ]
            home_contexts = [
                {"label": "home_to_loaded", "stage": "zero_to_loaded", "attached_boxes": [], "static_box_obstacles": []}
                for _ in home_samples
            ]
            if args.executor_mode == "real":
                print("实机将发送 12 个手臂关节 + turn=0；不发送 updown。", flush=True)
                print(f"负重姿态族索引：{args.loaded_preferred_pose_index}（0 为当前实机确认方向）", flush=True)
                print(f"实机方向映射：{'开启' if apply_ethercat_signs else '关闭'}", flush=True)
                print("实机 joint_names 顺序：" + ", ".join(command_joint_names), flush=True)
                input("确认真实机器人当前接近全0起点、人员远离、可运动后按回车开始 全0→负重姿态；Ctrl+C 取消...")
            print("开始执行：全0 → 负重姿态", flush=True)
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
            ):
                return 1
            print(f"完成执行：全0 → 负重姿态，用时 {(time.monotonic() - home_start):.3f}s", flush=True)

            input("已到负重姿态。按回车开始 L6/R8 任务执行...")

            task_samples_raw = trajectory_from_snapshot(
                snapshot,
                args.hz,
                args.max_joint_speed_deg_s,
                initial=loaded,
            )
            task_samples = [(time_s, joint_map) for time_s, joint_map, _ in task_samples_raw]
            task_contexts = [context for _, _, context in task_samples_raw]
            if not task_samples:
                raise RuntimeError("snapshot produced empty execution trajectory")
            print(f"开始执行：L6/R8 任务轨迹，轨迹点 {len(task_samples)}，频率 {args.hz:.1f}Hz", flush=True)
            task_start = time.monotonic()
            if not client.send_and_wait(
                make_trajectory(
                    task_samples,
                    command_joint_names,
                    apply_ethercat_signs=apply_ethercat_signs,
                ),
                task_contexts,
                "L6_R8_task",
                feedback_uses_ethercat_signs=apply_ethercat_signs,
            ):
                return 1
            print(f"完成执行：L6/R8 任务轨迹，用时 {(time.monotonic() - task_start):.3f}s", flush=True)
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
