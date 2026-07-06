#!/usr/bin/python3
"""Interactive staged Rerun monitor for the extract pipeline.

Enter-driven flow:
1. compute and display unique IK candidates;
2. compute and display extract-success candidates;
3. compute and display loaded-plan-success candidates;
4. display final selected plan.

The next stage is computed only after pressing Enter.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

rr: Any | None = None


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import process_lifecycle  # noqa: E402


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
SYSTEM_PYTHON = Path("/usr/bin/python3")
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "data/ik_benchmark/extract_stage_monitor"


def wall_stamp() -> str:
    return datetime.now().strftime("%H:%M:%S")


def log_event(label: str, run_start: float | None = None) -> None:
    if run_start is None:
        print(f"[{wall_stamp()}] {label}", flush=True)
    else:
        print(f"[{wall_stamp()} +{time.monotonic() - run_start:.3f}s] {label}", flush=True)


def load_rerun_helpers():
    candidates = [
        REPO_ROOT / "scripts/ik_benchmark/scripts/visualize_rerun.py",
        Path.cwd() / "scripts/ik_benchmark/scripts/visualize_rerun.py",
        Path.cwd().parent / "scripts/ik_benchmark/scripts/visualize_rerun.py",
    ]
    for helper_path in candidates:
        if helper_path.exists():
            spec = importlib.util.spec_from_file_location("alfa_visualize_rerun_helpers", helper_path)
            if spec is None or spec.loader is None:
                continue
            module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = module
            spec.loader.exec_module(module)
            return module
    raise RuntimeError("cannot find scripts/ik_benchmark/scripts/visualize_rerun.py")


def bash_source_command(command: str) -> list[str]:
    return [
        "bash",
        "-lc",
        "source /opt/ros/humble/setup.bash && "
        f"source {ROS_WS}/install/setup.bash && "
        f"cd {ROS_WS} && "
        f"{command}",
    ]


def run_text(command: str, *, timeout: float | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        bash_source_command(command),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=False,
    )


def service_exists(name: str) -> bool:
    try:
        result = run_text("ros2 service list", timeout=5.0)
    except Exception:
        return False
    if result.returncode != 0:
        return False
    return name in result.stdout.splitlines()


def extract_snapshot_path_from_service_output(output: str) -> Path | None:
    match = re.search(r"snapshot=([^'\s]+)", output)
    if not match:
        return None
    return Path(match.group(1))


def wait_for_service(
    name: str,
    planner: subprocess.Popen[str] | None,
    timeout: float,
    log_path: Path | None = None,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if planner is not None and planner.poll() is not None:
            raise RuntimeError(f"planner exited before service became available, code={planner.returncode}")
        if log_path is not None and log_path.exists():
            text = log_path.read_text(errors="ignore")
            if "DualArmPlannerNode ready" in text and "run_extract_monitor_next" in text:
                return
        result = run_text("ros2 service list", timeout=5.0)
        if name in result.stdout.splitlines():
            return
        time.sleep(0.5)
    raise TimeoutError(f"service {name} not available after {timeout:.1f}s")


def terminate_process(process: subprocess.Popen[str] | None, timeout: float = 5.0) -> None:
    process_lifecycle.terminate_process_tree(process, interrupt_timeout=timeout)


def planner_monitor_service_exists() -> bool:
    return process_lifecycle.any_service_exists(service_exists)


def wait_until_planner_services_gone(timeout: float = 15.0) -> bool:
    return process_lifecycle.wait_until_services_gone(service_exists, timeout=timeout)


def cleanup_stale_planner_stack(timeout: float = 15.0) -> bool:
    process_lifecycle.request_stale_planner_shutdown()
    return wait_until_planner_services_gone(timeout)


def build_launch_command(args: argparse.Namespace, run_dir: Path, snapshot_path: Path) -> str:
    parts = [
        "ros2 launch alfa_robot_moveit_config dual_arm_planner.launch.py",
        "execute:=false",
        "start_move_group:=true",
        f"box_front_x:={args.box_front_x}",
        f"scene_y_shift:={args.scene_y_shift}",
        f"fixed_updown:={args.fixed_updown}",
        f"extract_monitor_turn:={getattr(args, 'turn_rad', 0.0)}",
        f"front_z_reach_lower:={args.front_z_reach_lower}",
        f"front_z_reach_upper:={args.front_z_reach_upper}",
        f"top_z_reach_lower:={args.top_z_reach_lower}",
        f"top_z_reach_upper:={args.top_z_reach_upper}",
        f"ik_h_candidate_count:={args.ik_h_candidate_count}",
        f"ik_seed_count:={args.ik_seed_count}",
        f"ik_candidate_timeout:={args.ik_candidate_timeout}",
        f"ik_try_target_orders:={str(args.ik_try_target_orders).lower()}",
        f"ik_use_reversed_target_order:={str(args.ik_use_reversed_target_order).lower()}",
        f"extract_demo_left_box_id:={args.left_box_id}",
        f"extract_demo_right_box_id:={args.right_box_id}",
        f"extract_monitor_top_suction:={str(args.grasp_mode == 'top_suction').lower()}",
        "extract_demo_direct_grasp_start:=true",
        "extract_benchmark_all_legal_ik:=false",
        "extract_benchmark_dual_arm:=true",
        "extract_benchmark_dual_async:=true",
        f"extract_benchmark_extract_workers:={args.extract_workers}",
        f"extract_benchmark_candidate_limit:={args.candidate_limit}",
        f"extract_step_x:={args.extract_step_x}",
        "extract_ik_dedup_enabled:=true",
        f"extract_ik_dedup_joint_threshold_deg:={args.dedup_joint_threshold_deg}",
        f"extract_ik_dedup_h_threshold:={args.dedup_h_threshold}",
        "extract_benchmark_plan_loaded_after_success:=true",
        f"extract_loaded_candidate_limit:={args.loaded_candidate_limit}",
        "extract_loaded_sort_by_pose_distance:=true",
        "extract_loaded_stop_on_first_success:=false",
        "extract_loaded_lateral_shift_enabled:=true",
        f"extract_loaded_lateral_shift_distance:={args.lateral_shift_distance}",
        f"extract_loaded_lateral_shift_step:={args.lateral_shift_step}",
        f"extract_loaded_lateral_shift_column:={args.lateral_shift_column}",
        f"extract_loaded_pre_lower_left_box_id:={args.pre_lower_left_box_id}",
        f"extract_loaded_pre_lower_right_box_id:={args.pre_lower_right_box_id}",
        f"extract_loaded_pre_lower_updown_delta:={args.pre_lower_updown_delta}",
        f"extract_loaded_target_updown:={args.fixed_updown}",
        f"extract_loaded_planning_time:={args.loaded_planning_time}",
        f"extract_loaded_planning_attempts:={args.loaded_planning_attempts}",
        "extract_loaded_use_direct_pipeline:=true",
        f"extract_loaded_parallel_workers:={args.loaded_workers}",
        f"loaded_preferred_pose_index:={getattr(args, 'loaded_preferred_pose_index', 0)}",
        "extract_use_independent_kdl:=true",
        f"extract_kdl_timeout:={args.extract_kdl_timeout}",
        f"planning_attempts:={args.loaded_planning_attempts}",
        "velocity_scale:=1.0",
        "acceleration_scale:=1.0",
        "record_trajectories:=false",
        f"record_jsonl_path:={run_dir / 'flow_unused.jsonl'}",
        f"extract_monitor_snapshot_path:={snapshot_path}",
    ]
    return " ".join(parts)


def call_trigger_service(service_name: str, timeout: float) -> tuple[bool, str, float]:
    start = time.monotonic()
    try:
        import rclpy
        from std_srvs.srv import Trigger
    except Exception:
        command = f"ros2 service call {service_name} std_srvs/srv/Trigger {{}}"
        result = run_text(command, timeout=timeout)
        elapsed = (time.monotonic() - start) * 1000.0
        output = result.stdout.strip()
        success = result.returncode == 0 and ("success=True" in output or "success: true" in output)
        return success, output, elapsed

    rclpy.init(args=None)
    node = rclpy.create_node("extract_stage_monitor_trigger_client")
    try:
        client = node.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=timeout):
            elapsed = (time.monotonic() - start) * 1000.0
            return False, f"service {service_name} not available after {timeout:.1f}s", elapsed
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        elapsed = (time.monotonic() - start) * 1000.0
        if not future.done():
            return False, f"service {service_name} call timed out after {timeout:.1f}s", elapsed
        response = future.result()
        if response is None:
            return False, "service returned no response", elapsed
        output = (
            "requester: direct rclpy Trigger request\n\n"
            f"response:\nstd_srvs.srv.Trigger_Response(success={response.success}, "
            f"message='{response.message}')"
        )
        return bool(response.success), output, elapsed
    finally:
        node.destroy_node()
        rclpy.shutdown()


def call_configure_extract_monitor_service(
    service_name: str,
    left_box_id: int,
    right_box_id: int,
    snapshot_path: Path,
    timeout: float,
) -> tuple[bool, str, float]:
    start = time.monotonic()
    try:
        import rclpy
        from alfa_robot_moveit_config.srv import ConfigureExtractMonitor
    except Exception:
        command = (
            f"ros2 service call {service_name} "
            "alfa_robot_moveit_config/srv/ConfigureExtractMonitor "
            f"\"{{left_box_id: {left_box_id}, right_box_id: {right_box_id}, "
            f"snapshot_path: '{snapshot_path}'}}\""
        )
        result = run_text(command, timeout=timeout)
        elapsed = (time.monotonic() - start) * 1000.0
        output = result.stdout.strip()
        success = result.returncode == 0 and ("success=True" in output or "success: true" in output)
        return success, output, elapsed

    rclpy.init(args=None)
    node = rclpy.create_node("extract_monitor_config_client")
    try:
        client = node.create_client(ConfigureExtractMonitor, service_name)
        if not client.wait_for_service(timeout_sec=timeout):
            elapsed = (time.monotonic() - start) * 1000.0
            return False, f"service {service_name} not available after {timeout:.1f}s", elapsed
        request = ConfigureExtractMonitor.Request()
        request.left_box_id = int(left_box_id)
        request.right_box_id = int(right_box_id)
        request.snapshot_path = str(snapshot_path)
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        elapsed = (time.monotonic() - start) * 1000.0
        if not future.done():
            return False, f"service {service_name} call timed out after {timeout:.1f}s", elapsed
        response = future.result()
        if response is None:
            return False, "service returned no response", elapsed
        output = (
            "requester: direct rclpy ConfigureExtractMonitor request\n\n"
            f"response:\nConfigureExtractMonitor_Response(success={response.success}, "
            f"message='{response.message}')"
        )
        return bool(response.success), output, elapsed
    finally:
        node.destroy_node()
        rclpy.shutdown()


class ExtractMonitorServiceClient:
    """Reusable rclpy clients for repeated extract monitor calls."""

    def __init__(
        self,
        *,
        configure_service: str,
        trigger_service: str,
        timeout: float,
        node_name: str = "extract_monitor_service_client",
    ) -> None:
        import rclpy
        from alfa_robot_moveit_config.srv import ConfigureExtractMonitor
        from std_srvs.srv import Trigger

        self._rclpy = rclpy
        self._configure_type = ConfigureExtractMonitor
        self._trigger_type = Trigger
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init(args=None)
        self.node = rclpy.create_node(node_name)
        self.configure_service = configure_service
        self.trigger_service = trigger_service
        self.configure_client = self.node.create_client(ConfigureExtractMonitor, configure_service)
        self.trigger_client = self.node.create_client(Trigger, trigger_service)
        self.wait_for_services(timeout)

    def wait_for_services(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        for client, service_name in (
            (self.configure_client, self.configure_service),
            (self.trigger_client, self.trigger_service),
        ):
            while time.monotonic() < deadline:
                if client.wait_for_service(timeout_sec=0.1):
                    break
            else:
                raise TimeoutError(f"service {service_name} not available after {timeout:.1f}s")

    def configure(
        self,
        left_box_id: int,
        right_box_id: int,
        snapshot_path: Path,
        timeout: float,
    ) -> tuple[bool, str, float]:
        start = time.monotonic()
        request = self._configure_type.Request()
        request.left_box_id = int(left_box_id)
        request.right_box_id = int(right_box_id)
        request.snapshot_path = str(snapshot_path)
        future = self.configure_client.call_async(request)
        self._rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        elapsed = (time.monotonic() - start) * 1000.0
        if not future.done():
            return False, f"ConfigureExtractMonitor timeout after {timeout:.1f}s", elapsed
        response = future.result()
        if response is None:
            return False, f"ConfigureExtractMonitor failed: {future.exception()}", elapsed
        output = (
            "requester: reusable rclpy ConfigureExtractMonitor request\n\n"
            f"response:\n{response}"
        )
        return bool(response.success), output, elapsed

    def trigger(self, timeout: float) -> tuple[bool, str, float]:
        start = time.monotonic()
        future = self.trigger_client.call_async(self._trigger_type.Request())
        self._rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        elapsed = (time.monotonic() - start) * 1000.0
        if not future.done():
            return False, f"Trigger timeout after {timeout:.1f}s", elapsed
        response = future.result()
        if response is None:
            return False, f"Trigger failed: {future.exception()}", elapsed
        output = (
            "requester: reusable rclpy Trigger request\n\n"
            f"response:\n{response}"
        )
        return bool(response.success), output, elapsed

    def close(self) -> None:
        if getattr(self, "node", None) is not None:
            self.node.destroy_node()
            self.node = None
        if self._owns_rclpy and self._rclpy.ok():
            self._rclpy.shutdown()

    def __enter__(self) -> "ExtractMonitorServiceClient":
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()


def read_snapshot(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(path)
    return json.loads(path.read_text())


def short_label(record: dict[str, Any], phase: str) -> str:
    idx = record.get("display_index", 0)
    if phase == "ik_candidates":
        return (
            f"#{idx} score={float(record.get('score', 0.0)):.2f} "
            f"h={float(record.get('h', 0.0)):.3f} seed={record.get('seed_index', 0)}"
        )
    if phase == "extract_successes":
        return (
            f"#{idx} cand={record.get('candidate_order', 0)} "
            f"roll={float(record.get('rollout_ms', 0.0)):.1f}ms "
            f"x={float(record.get('final_retreat_x', 0.0)):.2f}"
        )
    if phase == "loaded_plan_successes":
        return (
            f"#{idx} cand={record.get('candidate_order', 0)} rank={record.get('loaded_plan_rank', 0)} "
            f"plan={float(record.get('loaded_plan_ms', 0.0)):.1f}ms "
            f"dist={float(record.get('loaded_plan_trajectory_distance', 0.0)):.2f}"
        )
    return (
        f"FINAL cand={record.get('candidate_order', 0)} rank={record.get('loaded_plan_rank', 0)} "
        f"dist={float(record.get('loaded_plan_trajectory_distance', 0.0)):.2f}"
    )


def record_joint_map(record: dict[str, Any]) -> dict[str, float] | None:
    state = record.get("state", {})
    joint_map = state.get("joint_map", {})
    if not joint_map:
        return None
    return {str(name): float(value) for name, value in joint_map.items()}


def transform_point(transform: np.ndarray, point: list[float]) -> list[float]:
    homogeneous = transform @ np.array([point[0], point[1], point[2], 1.0], dtype=float)
    return [float(homogeneous[0]), float(homogeneous[1]), float(homogeneous[2])]


def matrix_to_quaternion(matrix: np.ndarray) -> list[float]:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    return [float(x), float(y), float(z), float(w)]


def all_boxes(box_x: float, y_shift: float = 0.0) -> dict[int, tuple[float, float, float]]:
    rows = [
        [(1, 0.8), (2, 0.4), (3, 0.0), (4, -0.4), (5, -0.8)],
        [(6, 0.8), (7, 0.4), (8, 0.0), (9, -0.4), (10, -0.8)],
        [(11, 0.8), (12, 0.4), (13, 0.0), (14, -0.4), (15, -0.8)],
        [(16, 0.8), (17, 0.4), (18, 0.0), (19, -0.4), (20, -0.8)],
        [(21, 0.8), (22, 0.4), (23, 0.0), (24, -0.4), (25, -0.8)],
    ]
    out: dict[int, tuple[float, float, float]] = {}
    for row_i, row in enumerate(rows):
        z = 0.2 + 0.4 * (len(rows) - 1 - row_i)
        for box_id, y in row:
            out[box_id] = (box_x, y + y_shift, z)
    return out


def log_box_stack(box_x: float, left_box_id: int, right_box_id: int, y_shift: float = 0.0) -> None:
    centers = []
    half_sizes = []
    colors = []
    labels = []
    for box_id, (x, y, z) in sorted(all_boxes(box_x, y_shift).items()):
        centers.append([x + 0.15, y, z])
        half_sizes.append([0.15, 0.2, 0.2])
        if box_id in (left_box_id, right_box_id):
            colors.append([80, 240, 100, 190])
        else:
            colors.append([255, 180, 60, 125])
        labels.append(str(box_id))
    rr.log("monitor/scene/boxes", rr.Boxes3D(centers=centers, half_sizes=half_sizes, colors=colors, labels=labels), static=True)


def log_default_container(y_shift: float = 0.0) -> None:
    thickness = 0.02
    length = 4.0
    width = 2.2
    height = 2.4
    center_x = 0.8
    center_y = y_shift
    floor_z = 0.0
    panels = [
        ([center_x, center_y + width * 0.5 + thickness * 0.5, floor_z + height * 0.5], [length, thickness, height], "left_wall"),
        ([center_x, center_y - width * 0.5 - thickness * 0.5, floor_z + height * 0.5], [length, thickness, height], "right_wall"),
        ([center_x, center_y, floor_z + height + thickness * 0.5], [length, width + 2.0 * thickness, thickness], "ceiling"),
    ]
    rr.log(
        "monitor/scene/container",
        rr.Boxes3D(
            centers=[panel[0] for panel in panels],
            half_sizes=[[value * 0.5 for value in panel[1]] for panel in panels],
            colors=[[80, 170, 255, 45] for _ in panels],
            labels=[panel[2] for panel in panels],
        ),
        static=True,
    )


def log_static_box_obstacles(config: dict[str, Any] | None) -> None:
    if not config or not config.get("enabled", False):
        rr.log("monitor/scene/static_box_obstacles", rr.Boxes3D(centers=[], half_sizes=[]))
        return
    centers = []
    half_sizes = []
    colors = []
    labels = []
    for box in config.get("boxes", []):
        center = box.get("center", [])
        size = box.get("size", [])
        if len(center) != 3 or len(size) != 3:
            continue
        centers.append([float(value) for value in center])
        half_sizes.append([float(value) * 0.5 for value in size])
        colors.append([170, 80, 255, 110])
        labels.append(str(box.get("id", "static_box_obstacle")))
    rr.log("monitor/scene/static_box_obstacles", rr.Boxes3D(centers=centers, half_sizes=half_sizes, colors=colors, labels=labels))


def joint_dict_from_stage_point(stage: dict[str, Any], point: dict[str, Any]) -> dict[str, float]:
    state_map = stage.get("start_state", {}).get("joint_map", {})
    out = {str(name): float(value) for name, value in state_map.items()}
    names = stage.get("trajectory", {}).get("joint_names", [])
    positions = point.get("positions", [])
    for name, value in zip(names, positions):
        out[str(name)] = float(value)
    return out


def start_point_from_stage(stage: dict[str, Any]) -> dict[str, Any] | None:
    state_map = stage.get("start_state", {}).get("joint_map", {})
    names = stage.get("trajectory", {}).get("joint_names", [])
    if not state_map or not names:
        return None
    try:
        positions = [float(state_map[str(name)]) for name in names]
    except KeyError:
        return None
    return {"time_from_start_sec": 0.0, "positions": positions, "velocities": [0.0 for _ in positions]}


def ensure_points_start_at_stage_start(stage: dict[str, Any], points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    start_point = start_point_from_stage(stage)
    if start_point is None or not points:
        return points if points else ([start_point] if start_point is not None else [])
    first_positions = points[0].get("positions", [])
    start_positions = start_point["positions"]
    if len(first_positions) == len(start_positions):
        if max(abs(float(first) - float(start)) for first, start in zip(first_positions, start_positions)) < 1e-6:
            return points
    return [start_point, *points]


def log_attached_boxes(robot: Any, joints: dict[str, float], attached_boxes: list[dict[str, Any]]) -> None:
    if not attached_boxes:
        rr.log("monitor/scene/attached_boxes", rr.Clear(recursive=True))
        return
    fk = robot.fk(joints)
    centers = []
    half_sizes = []
    quaternions = []
    colors = []
    labels = []
    for box in attached_boxes:
        link_name = str(box.get("link_name", ""))
        link_tf = fk.get(link_name)
        center_in_link = box.get("center_in_link", [])
        size = box.get("size", [])
        if link_tf is None or len(center_in_link) != 3 or len(size) != 3:
            continue
        centers.append(transform_point(link_tf, [float(value) for value in center_in_link]))
        half_sizes.append([float(value) * 0.5 for value in size])
        quaternions.append(matrix_to_quaternion(link_tf[:3, :3]))
        colors.append([40, 220, 90, 150])
        labels.append(str(box.get("id", "carried_box")))
    if centers:
        rr.log(
            "monitor/scene/attached_boxes",
            rr.Boxes3D(centers=centers, half_sizes=half_sizes, quaternions=quaternions, colors=colors, labels=labels),
        )


def log_robot_skeleton(robot: Any, joint_map: dict[str, float], root: str) -> None:
    transforms = robot.fk(joint_map)
    positions: dict[str, list[float]] = {
        link_name: [
            float(transform[0, 3]),
            float(transform[1, 3]),
            float(transform[2, 3]),
        ]
        for link_name, transform in transforms.items()
    }
    points = list(positions.values())
    if points:
        rr.log(
            f"{root}/robot_links",
            rr.Points3D(
                positions=points,
                radii=0.025,
                colors=[90, 180, 255],
            ),
        )

    strips = []
    for joint in robot.joints.values():
        parent = positions.get(joint.parent)
        child = positions.get(joint.child)
        if parent is None or child is None:
            continue
        strips.append([parent, child])
    if strips:
        rr.log(
            f"{root}/robot_skeleton",
            rr.LineStrips3D(
                strips=strips,
                colors=[240, 240, 240],
                radii=0.01,
            ),
        )


def log_snapshot(snapshot: dict[str, Any], helpers: Any, robot: Any, args: argparse.Namespace) -> None:
    phase = str(snapshot.get("phase", "unknown"))
    records = list(snapshot.get("records", []))[: args.max_display]
    if hasattr(rr, "Clear"):
        rr.log("monitor", rr.Clear(recursive=True))
    else:
        rr.log("monitor", rr.TextLog("clear requested; installed rerun has no Clear archetype"))

    rr.log(
        "monitor/title",
        rr.TextLog(
            f"{snapshot.get('phase_label', phase)} | 显示 {len(records)}/{len(snapshot.get('records', []))} | "
            f"阶段耗时 {float(snapshot.get('elapsed_ms', 0.0)):.1f} ms | "
            f"L{snapshot.get('left_box_id', '?')}/R{snapshot.get('right_box_id', '?')}"
        ),
    )

    if not records:
        rr.log("monitor/empty", rr.TextLog("本阶段没有可显示结果"))
        return

    cols = max(1, args.grid_cols)
    for index, record in enumerate(records):
        row = index // cols
        col = index % cols
        root = f"monitor/{phase}/slot_{index:03d}"
        rr.log(root, rr.Transform3D(translation=[col * args.spacing, -row * args.spacing, 0.0]))
        rr.log(f"{root}/label", rr.TextLog(short_label(record, phase)))
        joint_map = record_joint_map(record)
        if not joint_map:
            continue
        if args.render_mode == "mesh":
            helpers.log_robot_static_model(robot, f"{root}/robot", log_meshes=True)
            helpers.log_robot_state(robot, joint_map, f"{root}/robot")
        else:
            log_robot_skeleton(robot, joint_map, root)


def log_selected_replay(snapshot: dict[str, Any], helpers: Any, robot: Any, args: argparse.Namespace) -> int:
    replay_stages = list(snapshot.get("replay_stages", []))
    if hasattr(rr, "Clear"):
        rr.log("monitor", rr.Clear(recursive=True))
    rr.log("monitor", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    helpers.log_robot_static_model(robot, "monitor/robot", log_meshes=True)
    scene_y_shift = float(snapshot.get("scene_y_shift", args.scene_y_shift))
    log_default_container(scene_y_shift)
    log_box_stack(float(args.box_front_x), int(snapshot.get("left_box_id", args.left_box_id)), int(snapshot.get("right_box_id", args.right_box_id)), scene_y_shift)
    rr.log(
        "monitor/title",
        rr.TextLog(
            f"{snapshot.get('phase_label', '完整流程最终采用方案')} | "
            f"总计算 {float(snapshot.get('elapsed_ms', 0.0)):.1f} ms | "
            f"IK {float(snapshot.get('ik_elapsed_ms', 0.0)):.1f} ms | "
            f"抽离 {float(snapshot.get('extract_elapsed_ms', 0.0)):.1f} ms | "
            f"负重规划 {float(snapshot.get('loaded_elapsed_ms', 0.0)):.1f} ms"
        ),
    )
    if not replay_stages:
        records = snapshot.get("records", [])
        if records:
            joint_map = record_joint_map(records[0])
            if joint_map:
                helpers.set_sample_time(0)
                helpers.log_robot_state(robot, joint_map, "monitor/robot")
        rr.log("monitor/warn", rr.TextLog("没有 replay_stages，只能显示最终静态姿态"))
        return 1

    sample = 0
    for stage_index, stage in enumerate(replay_stages):
        points = ensure_points_start_at_stage_start(stage, list(stage.get("trajectory", {}).get("points", [])))
        if not points:
            continue
        selected_indices = list(range(0, len(points), max(1, args.stride)))
        if selected_indices[-1] != len(points) - 1:
            selected_indices.append(len(points) - 1)
        for point_index in selected_indices:
            helpers.set_sample_time(sample)
            log_default_container(scene_y_shift)
            log_box_stack(
                float(args.box_front_x),
                int(snapshot.get("left_box_id", args.left_box_id)),
                int(snapshot.get("right_box_id", args.right_box_id)),
                scene_y_shift,
            )
            log_static_box_obstacles(stage.get("static_box_obstacles"))
            point = points[point_index]
            joints = joint_dict_from_stage_point(stage, point)
            helpers.log_robot_state(robot, joints, "monitor/robot")
            log_attached_boxes(robot, joints, stage.get("attached_boxes", []))
            rr.log(
                "monitor/info",
                rr.TextLog(
                    f"stage {stage_index + 1}/{len(replay_stages)}: {stage.get('stage')} | "
                    f"point {point_index + 1}/{len(points)}"
                ),
            )
            sample += 1
    return sample


def stream_planner_log(log_path: Path, offset: int) -> int:
    if not log_path.exists():
        return offset
    with log_path.open(errors="ignore") as file:
        file.seek(offset)
        for line in file:
            if any(token in line for token in (
                "direct IK selected",
                "IK阶段",
                "Extract primitive",
                "Loaded pose direct",
                "Computed path is not valid",
                "Plan failed",
            )):
                print(line.rstrip())
        return file.tell()


def run_full_selected_once(
    *,
    service_name: str,
    snapshot_path: Path,
    launch_log: Path,
    service_timeout: float,
    helpers: Any | None,
    robot: Any | None,
    args: argparse.Namespace,
    log_offset: int = 0,
    run_start: float | None = None,
) -> tuple[bool, int, int]:
    log_event("计算开始：IK → 抽离 → 横向让位 → 负重规划", run_start)
    compute_start = time.monotonic()
    success, output, elapsed_ms = call_trigger_service(service_name, service_timeout)
    compute_wall_ms = (time.monotonic() - compute_start) * 1000.0
    if launch_log.exists():
        log_offset = stream_planner_log(launch_log, log_offset)
    print(output)
    print(f"服务调用墙钟耗时：{elapsed_ms:.1f} ms")
    log_event(f"计算结束：服务返回 success={success}，本地等待 {compute_wall_ms:.1f} ms", run_start)
    if not success:
        return False, 0, log_offset
    returned_snapshot_path = extract_snapshot_path_from_service_output(output)
    if returned_snapshot_path is not None and returned_snapshot_path != snapshot_path:
        raise RuntimeError(
            "服务返回的 snapshot 路径和本轮不一致，说明当前 console 连到了旧 planner。\n"
            f"本轮期望：{snapshot_path}\n"
            f"服务返回：{returned_snapshot_path}\n"
            "请清理旧 ROS 进程后重新启动。"
        )
    snapshot = read_snapshot(snapshot_path)
    print(
        f"阶段：{snapshot.get('phase_label', snapshot.get('phase'))} | "
        f"快照：{snapshot_path} | "
        f"阶段内部耗时：{float(snapshot.get('elapsed_ms', 0.0)):.1f} ms | "
        f"结果数：{len(snapshot.get('records', []))}"
    )
    sample_count = 0
    if helpers is not None and robot is not None:
        log_event("Rerun 写入开始", run_start)
        sample_count = log_selected_replay(snapshot, helpers, robot, args)
        log_event(f"Rerun 写入完成：samples={sample_count}", run_start)
    return True, sample_count, log_offset


def main() -> int:
    run_start = time.monotonic()
    parser = argparse.ArgumentParser(description="交互式抽箱流程阶段监控台")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--left-box-id", type=int, default=2)
    parser.add_argument("--right-box-id", type=int, default=3)
    parser.add_argument("--box-front-x", type=float, default=0.925)
    parser.add_argument("--scene-y-shift", type=float, default=None, help="场景相对机器人 y 偏移；机器人左移 0.4m 时通常传 -0.4")
    parser.add_argument("--box-stack-y-shift", type=float, default=None, help="兼容旧参数名；等同于 --scene-y-shift")
    parser.add_argument("--fixed-updown", type=float, default=0.3)
    parser.add_argument("--turn-deg", type=float, default=0.0)
    parser.add_argument("--grasp-mode", choices=["front", "top_suction"], default="front")
    parser.add_argument("--front-z-reach-lower", type=float, default=0.45)
    parser.add_argument("--front-z-reach-upper", type=float, default=1.25)
    parser.add_argument("--top-z-reach-lower", type=float, default=0.3)
    parser.add_argument("--top-z-reach-upper", type=float, default=0.45)
    parser.add_argument("--ik-h-candidate-count", type=int, default=16)
    parser.add_argument("--ik-seed-count", type=int, default=32)
    parser.add_argument("--ik-candidate-timeout", type=float, default=0.01)
    parser.add_argument("--ik-try-target-orders", action="store_true")
    parser.add_argument("--ik-use-reversed-target-order", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--candidate-limit", type=int, default=64)
    parser.add_argument("--extract-workers", type=int, default=16)
    parser.add_argument("--extract-step-x", type=float, default=0.03)
    parser.add_argument("--loaded-candidate-limit", type=int, default=8)
    parser.add_argument("--loaded-workers", type=int, default=8)
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
    parser.add_argument(
        "--mode",
        choices=["full-selected", "staged"],
        default="full-selected",
        help="full-selected: 回车后完整计算并只显示最终方案；staged: 每次回车推进一个内部阶段",
    )
    parser.add_argument("--no-start-planner", action="store_true", help="不启动 planner，只连接已有监控服务")
    parser.add_argument(
        "--ros-domain-id",
        default="auto",
        help="本次 ROS_DOMAIN_ID；auto 会隔离自启动测试，inherit 表示沿用当前终端。",
    )
    parser.add_argument("--connect", action="store_true", help="连接已有 Rerun viewer，而不是 spawn 新 viewer")
    parser.add_argument("--no-rerun", action="store_true", help="只在终端打印，不显示 Rerun")
    parser.add_argument("--once", action="store_true", help="不等待回车，只完整计算一次后退出")
    parser.add_argument("--save", type=Path, default=None, help="保存最终选中流程为 .rrd；设置后自动只跑一次")
    parser.add_argument(
        "--cleanup-stale-planner",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="启动自管 planner 前自动清理旧 dual_arm_planner/move_group 服务。",
    )
    parser.add_argument("--max-display", type=int, default=64)
    parser.add_argument("--grid-cols", type=int, default=8)
    parser.add_argument("--spacing", type=float, default=2.4)
    parser.add_argument("--stride", type=int, default=1, help="完整回放时每隔 N 个轨迹点记录一次")
    parser.add_argument(
        "--render-mode",
        choices=["skeleton", "mesh"],
        default="mesh",
        help="full-selected 默认 mesh 动态回放；staged 多候选模式可用 skeleton 避免爆显存",
    )
    args = parser.parse_args()
    args.turn_rad = math.radians(args.turn_deg)
    if args.scene_y_shift is None:
        args.scene_y_shift = 0.0 if args.box_stack_y_shift is None else args.box_stack_y_shift
    if args.save is not None and args.no_rerun:
        raise RuntimeError("--save 需要启用 Rerun 记录，不能和 --no-rerun 同时使用")
    if args.save is not None and args.mode != "full-selected":
        raise RuntimeError("--save 只支持 full-selected 模式")
    if args.no_start_planner and args.ros_domain_id == "auto":
        args.ros_domain_id = "inherit"
    domain = process_lifecycle.configure_ros_domain(args.ros_domain_id)
    log_event(f"ROS_DOMAIN_ID={domain if domain is not None else 'unset'}", run_start)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = args.output_root / f"L{args.left_box_id}_R{args.right_box_id}_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    snapshot_path = run_dir / "stage_snapshot.json"
    launch_log = run_dir / "planner.log"
    ros_home = run_dir / "ros_home"
    ros_log_dir = run_dir / "ros_log"
    ros_home.mkdir(parents=True, exist_ok=True)
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("ROS_HOME", str(ros_home))
    os.environ.setdefault("ROS_LOG_DIR", str(ros_log_dir))

    planner: subprocess.Popen[str] | None = None
    if not args.no_start_planner:
        if planner_monitor_service_exists():
            if not args.cleanup_stale_planner:
                raise RuntimeError(
                    "检测到已有 /dual_arm_planner 监控服务。"
                    "这通常说明上一轮 planner 没关干净；为了避免连到旧节点，本次拒绝启动。\n"
                    "可去掉 --no-cleanup-stale-planner 让脚本自动清理，"
                    "或手动关闭仍在运行的 dual_arm_planner/move_group。"
                )
            log_event("检测到旧 /dual_arm_planner 服务，尝试自动清理旧 planner/move_group", run_start)
            if not cleanup_stale_planner_stack(15.0):
                raise RuntimeError(
                    "旧 /dual_arm_planner 服务清理超时。"
                    "请检查是否有外部终端仍在运行 dual_arm_planner/move_group。"
                )
        launch_command = build_launch_command(args, run_dir, snapshot_path)
        domain_export = f"export ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}\n" if "ROS_DOMAIN_ID" in os.environ else ""
        (run_dir / "launch_command.sh").write_text(
            "#!/usr/bin/env bash\nset -e\n"
            f"{domain_export}"
            "source /opt/ros/humble/setup.bash\n"
            f"source {ROS_WS}/install/setup.bash\n"
            f"cd {ROS_WS}\n{launch_command}\n"
        )
        log_event(f"启动 planner，日志：{launch_log}", run_start)
        with launch_log.open("w") as log_file:
            planner = subprocess.Popen(
                bash_source_command(launch_command),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid,
            )

    helpers = None
    robot = None
    if not args.no_rerun:
        global rr
        import rerun as rerun_module

        rr = rerun_module
        helpers = load_rerun_helpers()
        rr.init("extract_stage_monitor")
        if args.save is not None:
            args.save.parent.mkdir(parents=True, exist_ok=True)
            rr.save(str(args.save))
        elif args.connect:
            rr.connect()
        else:
            rr.spawn()
        urdf_text = helpers.render_current_urdf()
        robot = helpers.UrdfRobot(urdf_text)

    try:
        service_name = (
            "/dual_arm_planner/run_extract_monitor_next"
            if args.mode == "staged"
            else "/dual_arm_planner/run_extract_monitor_full_selected"
        )
        log_offset = 0
        log_event(f"等待监控服务：{service_name}", run_start)
        wait_for_service(service_name, planner, args.service_timeout, launch_log)
        if planner is not None:
            log_event("预热 IK solver：配置初始抽箱任务", run_start)
            prewarm_ok, prewarm_output, prewarm_ms = call_configure_extract_monitor_service(
                "/dual_arm_planner/configure_extract_monitor",
                args.left_box_id,
                args.right_box_id,
                snapshot_path,
                args.service_timeout,
            )
            if launch_log.exists():
                log_offset = stream_planner_log(launch_log, 0)
            print(prewarm_output)
            if not prewarm_ok:
                raise RuntimeError(f"IK solver 预热失败：{prewarm_output}")
            log_event(f"监控服务已就绪，IK solver 已预热：{prewarm_ms:.1f} ms", run_start)
        else:
            log_event("监控服务已就绪", run_start)
        if args.mode == "staged":
            print("回车顺序：1 IK候选 -> 2 抽离成功 -> 3 负重规划成功 -> 4 最终方案；第5次会重新开始。Ctrl-C 退出。")
        else:
            print("回车一次：从零开始完整计算 IK→抽离→负重规划，并只显示最终采用方案。Ctrl-C 退出。")
        if args.once or args.save is not None:
            ok, sample_count, log_offset = run_full_selected_once(
                service_name=service_name,
                snapshot_path=snapshot_path,
                launch_log=launch_log,
                service_timeout=args.service_timeout,
                helpers=helpers,
                robot=robot,
                args=args,
                log_offset=log_offset,
                run_start=run_start,
            )
            if args.save is not None and ok:
                log_event(f"已保存 Rerun：{args.save} | samples={sample_count}", run_start)
            return 0 if ok else 1
        stage_index = 0
        while True:
            try:
                prompt = "\n按回车开始完整计算..." if args.mode == "full-selected" else "\n按回车开始计算下一阶段..."
                input(prompt)
            except EOFError:
                print("\n输入结束，监控台退出。")
                return 0
            stage_index += 1
            log_event("计算开始：请求监控服务", run_start)
            success, output, elapsed_ms = call_trigger_service(service_name, args.service_timeout)
            if launch_log.exists():
                log_offset = stream_planner_log(launch_log, log_offset)
            print(output)
            print(f"服务调用墙钟耗时：{elapsed_ms:.1f} ms")
            log_event(f"计算结束：服务返回 success={success}", run_start)
            if not success:
                print("阶段失败，保留当前显示；再次回车会继续请求服务。")
                continue
            if args.mode == "full-selected":
                returned_snapshot_path = extract_snapshot_path_from_service_output(output)
                if returned_snapshot_path is not None and returned_snapshot_path != snapshot_path:
                    raise RuntimeError(
                        "服务返回的 snapshot 路径和本轮不一致，说明当前 console 连到了旧 planner。\n"
                        f"本轮期望：{snapshot_path}\n"
                        f"服务返回：{returned_snapshot_path}\n"
                        "请清理旧 ROS 进程后重新启动。"
                    )
                snapshot = read_snapshot(snapshot_path)
                print(
                    f"阶段：{snapshot.get('phase_label', snapshot.get('phase'))} | "
                    f"快照：{snapshot_path} | "
                    f"阶段内部耗时：{float(snapshot.get('elapsed_ms', 0.0)):.1f} ms | "
                    f"结果数：{len(snapshot.get('records', []))}"
                )
                if helpers is not None and robot is not None:
                    log_event("Rerun 写入开始", run_start)
                    sample_count = log_selected_replay(snapshot, helpers, robot, args)
                    log_event(f"Rerun 写入完成：samples={sample_count}", run_start)
            else:
                returned_snapshot_path = extract_snapshot_path_from_service_output(output)
                if returned_snapshot_path is not None and returned_snapshot_path != snapshot_path:
                    raise RuntimeError(
                        "服务返回的 snapshot 路径和本轮不一致，说明当前 console 连到了旧 planner。\n"
                        f"本轮期望：{snapshot_path}\n"
                        f"服务返回：{returned_snapshot_path}\n"
                        "请清理旧 ROS 进程后重新启动。"
                    )
                snapshot = read_snapshot(snapshot_path)
                print(
                    f"阶段：{snapshot.get('phase_label', snapshot.get('phase'))} | "
                    f"快照：{snapshot_path} | "
                    f"阶段内部耗时：{float(snapshot.get('elapsed_ms', 0.0)):.1f} ms | "
                    f"结果数：{len(snapshot.get('records', []))}"
                )
                if helpers is not None and robot is not None:
                    helpers.set_sample_time(stage_index)
                    log_snapshot(snapshot, helpers, robot, args)
    except KeyboardInterrupt:
        print("\n监控台退出。")
        return 0
    finally:
        if planner is not None:
            log_event("关闭 planner 进程组", run_start)
            terminate_process(planner)
            if not wait_until_planner_services_gone(15.0):
                log_event("planner 服务仍未消失，追加清理旧 planner/move_group", run_start)
                cleanup_stale_planner_stack(15.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
