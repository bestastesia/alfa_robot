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
from alfa_robot_rerun import visualize_rerun as rerun_helpers

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
        if (candidate / "ros2_ws" / "src").is_dir():
            return candidate
        if candidate.name == "ros2_ws":
            return candidate.parent
    return Path.cwd().resolve()


REPO_ROOT = find_repo_root()
ROS_WS = REPO_ROOT / "ros2_ws"
SYSTEM_PYTHON = Path("/usr/bin/python3")
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "data/ik_benchmark/extract_stage_monitor"
DEFAULT_LOADED_POSE_FAMILY_DEG = "[0.0,-45.0,120.0,-75.0,0.0,0.0]"


def wall_stamp() -> str:
    return datetime.now().strftime("%H:%M:%S")


def log_event(label: str, run_start: float | None = None) -> None:
    if run_start is None:
        print(f"[{wall_stamp()}] {label}", flush=True)
    else:
        print(f"[{wall_stamp()} +{time.monotonic() - run_start:.3f}s] {label}", flush=True)


def load_rerun_helpers():
    return rerun_helpers


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
        f"world_to_base_z:={getattr(args, 'world_to_base_z', 0.202094)}",
        f"fixed_updown:={args.fixed_updown}",
        f"extract_monitor_turn:={getattr(args, 'turn_rad', 0.0)}",
        f"front_z_reach_lower:={args.front_z_reach_lower}",
        f"front_z_reach_upper:={args.front_z_reach_upper}",
        f"top_z_reach_lower:={args.top_z_reach_lower}",
        f"top_z_reach_upper:={args.top_z_reach_upper}",
        f"top_suction_x_offset:={getattr(args, 'top_suction_x_offset', 0.15)}",
        f"top_suction_z_offset:={getattr(args, 'top_suction_z_offset', 0.2)}",
        f"ik_top_position_tolerance:={getattr(args, 'ik_top_position_tolerance', 0.04)}",
        f"ik_top_orientation_tolerance_deg:={getattr(args, 'ik_top_orientation_tolerance_deg', 7.0)}",
        f"ik_h_candidate_count:={args.ik_h_candidate_count}",
        f"ik_h_lower:={getattr(args, 'ik_h_lower', 0.0)}",
        f"ik_h_upper:={getattr(args, 'ik_h_upper', 0.7)}",
        f"ik_h_step:={getattr(args, 'ik_h_step', 0.01)}",
        f"ik_full_h_range_scan:={str(getattr(args, 'ik_full_h_range_scan', True)).lower()}",
        f"ik_seed_count:={args.ik_seed_count}",
        f"ik_workers:={args.ik_workers}",
        f"ik_candidate_timeout:={args.ik_candidate_timeout}",
        f"ik_try_target_orders:={str(args.ik_try_target_orders).lower()}",
        f"ik_use_reversed_target_order:={str(args.ik_use_reversed_target_order).lower()}",
        f"optimized_ik_check_collision:={str(getattr(args, 'optimized_ik_check_collision', True)).lower()}",
        f"extract_demo_left_box_id:={args.left_box_id}",
        f"extract_demo_right_box_id:={args.right_box_id}",
        f"extract_monitor_top_suction:={str(args.grasp_mode == 'top_suction').lower()}",
        f"extract_monitor_left_top_suction:={str(getattr(args, 'left_grasp_mode', args.grasp_mode) == 'top_suction').lower()}",
        f"extract_monitor_right_top_suction:={str(getattr(args, 'right_grasp_mode', args.grasp_mode) == 'top_suction').lower()}",
        "extract_demo_direct_grasp_start:=true",
        "extract_benchmark_all_legal_ik:=false",
        "extract_benchmark_dual_arm:=true",
        "extract_benchmark_dual_async:=true",
        f"extract_benchmark_extract_workers:={args.extract_workers}",
        f"extract_benchmark_candidate_limit:={args.candidate_limit}",
        f"extract_benchmark_extract_success_quorum:={getattr(args, 'extract_success_quorum', 0)}",
        f"extract_benchmark_extract_quality_success_quorum:={getattr(args, 'extract_quality_success_quorum', 0)}",
        f"extract_benchmark_extract_quality_loaded_distance_sum:={getattr(args, 'extract_quality_loaded_distance_sum', 0.0)}",
        f"extract_step_x:={args.extract_step_x}",
        f"extract_max_joint_delta:={getattr(args, 'extract_max_joint_delta', 10.0 * math.pi / 180.0)}",
        "extract_ik_dedup_enabled:=true",
        f"extract_ik_dedup_joint_threshold_deg:={args.dedup_joint_threshold_deg}",
        f"extract_ik_dedup_h_threshold:={args.dedup_h_threshold}",
        f"extract_ik_stratified_limit_enabled:={str(getattr(args, 'extract_ik_stratified_limit_enabled', False)).lower()}",
        f"extract_ik_stratified_h_bucket:={getattr(args, 'extract_ik_stratified_h_bucket', 0.05)}",
        f"extract_ik_stratified_top_score_count:={getattr(args, 'extract_ik_stratified_top_score_count', 12)}",
        f"extract_ik_candidate_reserve_limit:={getattr(args, 'extract_ik_candidate_reserve_limit', 64)}",
        f"extract_ik_candidate_reserve_stratified:={str(getattr(args, 'extract_ik_candidate_reserve_stratified', True)).lower()}",
        f"extract_ik_candidate_reserve_interleave_stride:={getattr(args, 'extract_ik_candidate_reserve_interleave_stride', 4)}",
        f"extract_ik_loaded_distance_order_weight:={getattr(args, 'extract_ik_loaded_distance_order_weight', 0.0)}",
        f"extract_monitor_capture_raw_ik:={str(getattr(args, 'ik_only_raw', False)).lower()}",
        f"extract_monitor_build_final_replay:={str(getattr(args, 'extract_monitor_build_final_replay', True)).lower()}",
        f"extract_monitor_place_cycle_enabled:={str(getattr(args, 'place_cycle_enabled', False)).lower()}",
        f"extract_monitor_place_updown:={getattr(args, 'place_updown', 0.10)}",
        f"extract_monitor_place_transition_updown:={getattr(args, 'place_transition_updown', 0.10)}",
        f"extract_monitor_place_left_pose_deg:='{getattr(args, 'place_left_pose_deg', '[0.0,-55.0,-50.0,-60.0,0.0,0.0]')}'",
        f"extract_monitor_place_right_pose_deg:='{getattr(args, 'place_right_pose_deg', '[0.0,-55.0,-50.0,-60.0,0.0,0.0]')}'",
        f"extract_rollout_mode:={getattr(args, 'extract_rollout_mode', 'greedy')}",
        f"extract_top_updown_lift_distance:={getattr(args, 'extract_top_updown_lift_distance', 0.40)}",
        f"extract_box_pose_rrt_edge_scene_collision:={str(getattr(args, 'extract_box_pose_rrt_edge_scene_collision', True)).lower()}",
        f"extract_box_pose_rrt_max_iterations:={getattr(args, 'extract_box_pose_rrt_max_iterations', 160)}",
        f"extract_box_pose_rrt_paths_per_arm:={getattr(args, 'extract_box_pose_rrt_paths_per_arm', 8)}",
        f"extract_box_pose_rrt_path_pair_limit:={getattr(args, 'extract_box_pose_rrt_path_pair_limit', 64)}",
        f"extract_box_pose_rrt_parent_candidates:={getattr(args, 'extract_box_pose_rrt_parent_candidates', 8)}",
        f"extract_box_pose_rrt_parent_diverse_candidates:={getattr(args, 'extract_box_pose_rrt_parent_diverse_candidates', 0)}",
        f"extract_box_pose_rrt_parent_endpoint_score_weight:={getattr(args, 'extract_box_pose_rrt_parent_endpoint_score_weight', 0.05)}",
        f"extract_box_pose_rrt_parent_node_score_weight:={getattr(args, 'extract_box_pose_rrt_parent_node_score_weight', 0.0)}",
        f"extract_box_pose_rrt_parent_density_weight:={getattr(args, 'extract_box_pose_rrt_parent_density_weight', 0.0)}",
        f"extract_box_pose_rrt_max_lateral:={getattr(args, 'extract_box_pose_rrt_max_lateral', 0.0)}",
        f"extract_box_pose_rrt_step_lateral:={getattr(args, 'extract_box_pose_rrt_step_lateral', 0.02)}",
        f"extract_box_pose_rrt_front_free_motion:={str(getattr(args, 'extract_box_pose_rrt_front_free_motion', True)).lower()}",
        f"extract_box_pose_rrt_front_goal_requires_max_pitch:={str(getattr(args, 'extract_box_pose_rrt_front_goal_requires_max_pitch', False)).lower()}",
        f"extract_box_pose_rrt_best_first_fallback:={str(getattr(args, 'extract_box_pose_rrt_best_first_fallback', True)).lower()}",
        f"extract_box_pose_rrt_best_first_first:={str(getattr(args, 'extract_box_pose_rrt_best_first_first', False)).lower()}",
        f"extract_box_pose_rrt_top_best_first_first:={str(getattr(args, 'extract_box_pose_rrt_top_best_first_first', False)).lower()}",
        f"extract_box_pose_rrt_best_first_max_expansions:={getattr(args, 'extract_box_pose_rrt_best_first_max_expansions', 800)}",
        f"extract_box_pose_rrt_best_first_heuristic_weight:={getattr(args, 'extract_box_pose_rrt_best_first_heuristic_weight', 1.0)}",
        f"extract_rrt_rollout_enabled:={str(getattr(args, 'extract_rrt', False)).lower()}",
        f"extract_rrt_planning_group:={getattr(args, 'extract_rrt_planning_group', 'dual_arm')}",
        f"extract_rrt_planning_time:={getattr(args, 'extract_rrt_planning_time', 0.35)}",
        f"extract_rrt_planning_attempts:={getattr(args, 'extract_rrt_planning_attempts', 1)}",
        f"extract_rrt_endpoint_per_arm_limit:={getattr(args, 'extract_rrt_endpoint_per_arm_limit', 8)}",
        f"extract_rrt_goal_limit:={getattr(args, 'extract_rrt_goal_limit', 8)}",
        "extract_benchmark_plan_loaded_after_success:=true",
        f"extract_loaded_candidate_limit:={args.loaded_candidate_limit}",
        f"extract_loaded_sort_by_pose_distance:={str(getattr(args, 'loaded_sort_by_pose_distance', True)).lower()}",
        f"extract_loaded_stop_on_first_success:={str(getattr(args, 'loaded_stop_on_first_success', False)).lower()}",
        f"extract_loaded_lateral_shift_enabled:={str(args.lateral_shift_enabled).lower()}",
        f"extract_loaded_lateral_shift_distance:={args.lateral_shift_distance}",
        f"extract_loaded_lateral_shift_step:={args.lateral_shift_step}",
        f"extract_loaded_lateral_shift_column:={args.lateral_shift_column}",
        f"extract_loaded_pre_lower_left_box_id:={args.pre_lower_left_box_id}",
        f"extract_loaded_pre_lower_right_box_id:={args.pre_lower_right_box_id}",
        f"extract_loaded_pre_lower_updown_delta:={args.pre_lower_updown_delta}",
        f"extract_loaded_target_updown:={getattr(args, 'loaded_updown', 0.1)}",
        f"extract_loaded_preserve_lower_updown:={str(getattr(args, 'loaded_preserve_lower_updown', False)).lower()}",
        f"extract_loaded_planning_time:={args.loaded_planning_time}",
        f"extract_loaded_planning_attempts:={args.loaded_planning_attempts}",
        "extract_loaded_use_direct_pipeline:=true",
        f"extract_loaded_planning_mode:={getattr(args, 'loaded_planning_mode', 'rrt')}",
        f"extract_loaded_parallel_workers:={args.loaded_workers}",
        f"loaded_preferred_pose_index:={getattr(args, 'loaded_preferred_pose_index', 0)}",
        f"loaded_left_pose_family_deg:='{getattr(args, 'loaded_left_pose_family_deg', DEFAULT_LOADED_POSE_FAMILY_DEG)}'",
        f"loaded_right_pose_family_deg:='{getattr(args, 'loaded_right_pose_family_deg', DEFAULT_LOADED_POSE_FAMILY_DEG)}'",
        f"planning_attempts:={args.loaded_planning_attempts}",
        "velocity_scale:=1.0",
        "acceleration_scale:=1.0",
        "record_trajectories:=false",
        f"record_jsonl_path:={run_dir / 'flow_unused.jsonl'}",
        f"extract_monitor_snapshot_path:={snapshot_path}",
    ]
    loaded_planner_id = getattr(args, "loaded_planner_id", "")
    if loaded_planner_id:
        parts.append(f"extract_loaded_planner_id:={loaded_planner_id}")
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


def assign_explicit_targets(
    request: Any,
    left_target: dict[str, Any] | None,
    right_target: dict[str, Any] | None,
) -> None:
    request.use_explicit_targets = left_target is not None and right_target is not None
    if not request.use_explicit_targets:
        return
    for target_message, target in (
        (request.left_target, left_target),
        (request.right_target, right_target),
    ):
        target_message.header.frame_id = str(target.get("frame_id", "base_link"))
        position = target["position"]
        orientation = target["orientation"]
        target_message.pose.position.x = float(position[0])
        target_message.pose.position.y = float(position[1])
        target_message.pose.position.z = float(position[2])
        target_message.pose.orientation.x = float(orientation[0])
        target_message.pose.orientation.y = float(orientation[1])
        target_message.pose.orientation.z = float(orientation[2])
        target_message.pose.orientation.w = float(orientation[3])


def call_configure_extract_monitor_service(
    service_name: str,
    left_box_id: int,
    right_box_id: int,
    snapshot_path: Path,
    timeout: float,
    left_top_suction: bool = False,
    right_top_suction: bool = False,
    left_target: dict[str, Any] | None = None,
    right_target: dict[str, Any] | None = None,
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
            f"snapshot_path: '{snapshot_path}', "
            f"left_top_suction: {str(left_top_suction).lower()}, "
            f"right_top_suction: {str(right_top_suction).lower()}}}\""
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
        request.left_top_suction = bool(left_top_suction)
        request.right_top_suction = bool(right_top_suction)
        assign_explicit_targets(request, left_target, right_target)
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
        left_top_suction: bool = False,
        right_top_suction: bool = False,
        left_target: dict[str, Any] | None = None,
        right_target: dict[str, Any] | None = None,
        runtime_config: dict[str, Any] | None = None,
    ) -> tuple[bool, str, float]:
        start = time.monotonic()
        request = self._configure_type.Request()
        request.left_box_id = int(left_box_id)
        request.right_box_id = int(right_box_id)
        request.snapshot_path = str(snapshot_path)
        request.left_top_suction = bool(left_top_suction)
        request.right_top_suction = bool(right_top_suction)
        assign_explicit_targets(request, left_target, right_target)
        if runtime_config is not None:
            request.update_runtime_config = True
            request.box_front_x = float(runtime_config["box_front_x"])
            request.scene_y_shift = float(runtime_config["scene_y_shift"])
            request.extract_rollout_mode = str(runtime_config["extract_rollout_mode"])
            request.loaded_lateral_shift_enabled = bool(
                runtime_config["loaded_lateral_shift_enabled"]
            )
            request.extract_box_pose_rrt_max_iterations = int(
                runtime_config["extract_box_pose_rrt_max_iterations"]
            )
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
        [(1, 0.5), (2, 0.0), (3, -0.5)],
        [(4, 0.5), (5, 0.0), (6, -0.5)],
        [(7, 0.5), (8, 0.0), (9, -0.5)],
        [(10, 0.5), (11, 0.0), (12, -0.5)],
        [(13, 0.5), (14, 0.0), (15, -0.5)],
    ]
    out: dict[int, tuple[float, float, float]] = {}
    for row_i, row in enumerate(rows):
        z = (len(rows) - row_i - 0.5) * 0.4
        for box_id, y in row:
            out[box_id] = (box_x, y + y_shift, z)
    return out


def log_container_panels(panels: list[dict[str, Any]] | None, *, static: bool = False) -> None:
    """绘制集装箱壳碰撞几何。

    数据必须来自规划节点写入 snapshot 的 ``container_panels`` 字段——它就是
    ``MotionSceneAdapter::containerPanels()`` 实际写入 MoveIt PlanningScene 的同一份
    碰撞几何（含绕 Z 轴 ``yaw``）。可视化侧不再重新计算/硬编码任何集装箱尺寸；
    若 snapshot 没有该字段（例如旧快照），则不绘制任何集装箱，避免画出与真实
    碰撞检测不一致的伪几何。
    """
    if not panels:
        rr.log("monitor/scene/container", rr.Clear(recursive=True))
        return
    centers = []
    half_sizes = []
    rotations = []
    colors = []
    labels = []
    for panel in panels:
        center = panel.get("center", [])
        size = panel.get("size", [])
        if len(center) != 3 or len(size) != 3:
            continue
        centers.append([float(value) for value in center])
        half_sizes.append([float(value) * 0.5 for value in size])
        yaw = float(panel.get("yaw", 0.0))
        rotations.append(rr.RotationAxisAngle(axis=[0.0, 0.0, 1.0], radians=yaw))
        colors.append([80, 170, 255, 45])
        labels.append(str(panel.get("id", "container_panel")))
    rr.log(
        "monitor/scene/container",
        rr.Boxes3D(
            centers=centers,
            half_sizes=half_sizes,
            rotation_axis_angles=rotations,
            colors=colors,
            labels=labels,
        ),
        static=static,
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


def point_from_state_map(stage: dict[str, Any], state_key: str, time_from_start_sec: float) -> dict[str, Any] | None:
    state_map = stage.get(state_key, {}).get("joint_map", {})
    names = stage.get("trajectory", {}).get("joint_names", [])
    if not state_map or not names:
        return None
    try:
        positions = [float(state_map[str(name)]) for name in names]
    except KeyError:
        return None
    return {
        "time_from_start_sec": float(time_from_start_sec),
        "positions": positions,
        "velocities": [0.0 for _ in positions],
    }


def ensure_points_end_at_stage_goal(stage: dict[str, Any], points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not points:
        return points
    last_time = float(points[-1].get("time_from_start_sec", 0.0))
    goal_point = point_from_state_map(stage, "goal_state", last_time)
    if goal_point is None:
        return points
    last_positions = points[-1].get("positions", [])
    goal_positions = goal_point["positions"]
    if len(last_positions) == len(goal_positions):
        if max(abs(float(last) - float(goal)) for last, goal in zip(last_positions, goal_positions)) < 1e-6:
            return points
    return [*points, goal_point]


def densify_stage_points(points: list[dict[str, Any]], max_joint_step_rad: float = 5.0 * math.pi / 180.0) -> list[dict[str, Any]]:
    if len(points) < 2:
        return points
    out: list[dict[str, Any]] = [points[0]]
    for point in points[1:]:
        prev = out[-1]
        prev_positions = [float(value) for value in prev.get("positions", [])]
        next_positions = [float(value) for value in point.get("positions", [])]
        if len(prev_positions) != len(next_positions) or not prev_positions:
            out.append(point)
            continue
        max_delta = max(abs(next_value - prev_value) for prev_value, next_value in zip(prev_positions, next_positions))
        step_count = max(1, int(math.ceil(max_delta / max_joint_step_rad)))
        prev_time = float(prev.get("time_from_start_sec", 0.0))
        next_time = float(point.get("time_from_start_sec", prev_time))
        for step in range(1, step_count + 1):
            ratio = float(step) / float(step_count)
            positions = [
                prev_value + (next_value - prev_value) * ratio
                for prev_value, next_value in zip(prev_positions, next_positions)
            ]
            if step == step_count:
                out.append({**point, "positions": positions})
            else:
                out.append({
                    "time_from_start_sec": prev_time + (next_time - prev_time) * ratio,
                    "positions": positions,
                    "velocities": [0.0 for _ in positions],
                })
    return out


def playback_points_for_stage(stage: dict[str, Any]) -> list[dict[str, Any]]:
    points = ensure_points_start_at_stage_start(stage, list(stage.get("trajectory", {}).get("points", [])))
    points = ensure_points_end_at_stage_goal(stage, points)
    return densify_stage_points(points)


def log_attached_boxes(robot: Any, joints: dict[str, float], attached_boxes: list[dict[str, Any]]) -> None:
    if not attached_boxes:
        rr.log("monitor/scene/attached_boxes", rr.Clear(recursive=True))
        rr.log("monitor/scene/attached_box_debug", rr.Clear(recursive=True))
        return
    fk = robot.fk(joints)
    centers = []
    half_sizes = []
    quaternions = []
    colors = []
    labels = []
    tool_points = []
    box_points = []
    link_to_center_lines = []
    local_z_origins = []
    local_z_vectors = []
    debug_labels = []
    for box in attached_boxes:
        link_name = str(box.get("link_name", ""))
        link_tf = fk.get(link_name)
        center_in_link = box.get("center_in_link", [])
        size = box.get("size", [])
        if link_tf is None or len(center_in_link) != 3 or len(size) != 3:
            continue
        tool_origin = [
            float(link_tf[0, 3]),
            float(link_tf[1, 3]),
            float(link_tf[2, 3]),
        ]
        center = transform_point(link_tf, [float(value) for value in center_in_link])
        centers.append(center)
        half_sizes.append([float(value) * 0.5 for value in size])
        quaternions.append(matrix_to_quaternion(link_tf[:3, :3]))
        colors.append([40, 220, 90, 150])
        labels.append(str(box.get("id", "carried_box")))
        tool_points.append(tool_origin)
        box_points.append(center)
        link_to_center_lines.append([tool_origin, center])
        local_z_origins.append(tool_origin)
        local_z_vectors.append([float(value) * 0.18 for value in link_tf[:3, :3] @ np.array([0.0, 0.0, 1.0])])
        debug_labels.append(f"{box.get('id', 'box')}: tool0→box_center")
    if centers:
        rr.log(
            "monitor/scene/attached_boxes",
            rr.Boxes3D(centers=centers, half_sizes=half_sizes, quaternions=quaternions, colors=colors, labels=labels),
        )
        rr.log(
            "monitor/scene/attached_box_debug/tool0_points",
            rr.Points3D(positions=tool_points, colors=[80, 200, 255, 255], radii=0.025, labels=["tool0" for _ in tool_points]),
        )
        rr.log(
            "monitor/scene/attached_box_debug/box_centers",
            rr.Points3D(positions=box_points, colors=[40, 255, 80, 255], radii=0.025, labels=labels),
        )
        rr.log(
            "monitor/scene/attached_box_debug/tool_to_box_center",
            rr.LineStrips3D(strips=link_to_center_lines, colors=[255, 255, 80, 255], radii=0.008, labels=debug_labels),
        )
        rr.log(
            "monitor/scene/attached_box_debug/tool_local_plus_z",
            rr.Arrows3D(origins=local_z_origins, vectors=local_z_vectors, colors=[255, 80, 80, 255], radii=0.01),
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
    container_panels = snapshot.get("container_panels")
    log_container_panels(container_panels)
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
    previous_positions: list[float] | None = None
    previous_joint_names: list[str] | None = None
    for stage_index, stage in enumerate(replay_stages):
        points = playback_points_for_stage(stage)
        if not points:
            continue
        joint_names = list(stage.get("trajectory", {}).get("joint_names", []))
        if previous_positions is not None and previous_joint_names == joint_names:
            first_positions = [float(value) for value in points[0].get("positions", [])]
            bridge_points = densify_stage_points([
                {"time_from_start_sec": 0.0, "positions": previous_positions, "velocities": [0.0 for _ in previous_positions]},
                {"time_from_start_sec": 0.1, "positions": first_positions, "velocities": [0.0 for _ in first_positions]},
            ])
            if len(bridge_points) > 2:
                points = bridge_points[1:-1] + points
        selected_indices = list(range(0, len(points), max(1, args.stride)))
        if selected_indices[-1] != len(points) - 1:
            selected_indices.append(len(points) - 1)
        for point_index in selected_indices:
            helpers.set_sample_time(sample)
            log_container_panels(container_panels)
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
        previous_positions = [float(value) for value in points[-1].get("positions", [])]
        previous_joint_names = joint_names
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
    parser.add_argument("--left-box-id", type=int, default=1)
    parser.add_argument("--right-box-id", type=int, default=3)
    parser.add_argument("--box-front-x", type=float, default=0.925)
    parser.add_argument("--scene-y-shift", type=float, default=None, help="箱堆中心相对机器人 y 偏移；默认 0 表示机器人对准中间列")
    parser.add_argument("--box-stack-y-shift", type=float, default=None, help="兼容旧参数名；等同于 --scene-y-shift")
    parser.add_argument("--fixed-updown", type=float, default=0.3)
    parser.add_argument("--turn-deg", type=float, default=0.0)
    parser.add_argument("--grasp-mode", choices=["front", "top_suction"], default="front")
    parser.add_argument("--left-grasp-mode", choices=["front", "top_suction"], default=None)
    parser.add_argument("--right-grasp-mode", choices=["front", "top_suction"], default=None)
    parser.add_argument("--front-z-reach-lower", type=float, default=0.45)
    parser.add_argument("--front-z-reach-upper", type=float, default=1.25)
    parser.add_argument("--top-z-reach-lower", type=float, default=0.0)
    parser.add_argument("--top-z-reach-upper", type=float, default=0.6)
    parser.add_argument("--top-suction-x-offset", type=float, default=0.15)
    parser.add_argument("--top-suction-z-offset", type=float, default=0.25)
    parser.add_argument("--ik-top-position-tolerance", type=float, default=0.04)
    parser.add_argument("--ik-top-orientation-tolerance-deg", type=float, default=7.0)
    parser.add_argument("--ik-h-candidate-count", type=int, default=64)
    parser.add_argument("--ik-seed-count", type=int, default=32)
    parser.add_argument("--ik-workers", type=int, default=1)
    parser.add_argument("--ik-candidate-timeout", type=float, default=0.01)
    parser.add_argument("--ik-try-target-orders", action="store_true")
    parser.add_argument("--ik-use-reversed-target-order", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--optimized-ik-check-collision", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--candidate-limit", type=int, default=25)
    parser.add_argument("--extract-workers", type=int, default=16)
    parser.add_argument("--extract-success-quorum", type=int, default=3)
    parser.add_argument("--extract-quality-success-quorum", type=int, default=0)
    parser.add_argument("--extract-quality-loaded-distance-sum", type=float, default=0.0)
    parser.add_argument("--extract-step-x", type=float, default=0.03)
    parser.add_argument("--extract-max-joint-delta", type=float, default=10.0 * math.pi / 180.0)
    parser.add_argument(
        "--extract-rollout-mode",
        choices=["greedy", "box_pose_rrt", "moveit_rrt_legacy", "top_lift_legacy"],
        default="greedy",
        help="抽离策略；box_pose_rrt 为箱体位姿 RRT，greedy 为稳定旧策略。",
    )
    parser.add_argument("--extract-rrt", action="store_true")
    parser.add_argument(
        "--extract-box-pose-rrt-edge-scene-collision",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="箱体位姿 RRT 每条插值边同时检查机器人、附着箱和场景碰撞；关闭用于复现旧方案。",
    )
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
    parser.add_argument("--loaded-candidate-limit", type=int, default=8)
    parser.add_argument("--loaded-workers", type=int, default=8)
    parser.add_argument("--loaded-planner-id", default="")
    parser.add_argument("--loaded-planning-mode", choices=["rrt", "shortcut"], default="rrt")
    parser.add_argument("--loaded-planning-time", type=float, default=1.0)
    parser.add_argument("--loaded-planning-attempts", type=int, default=8)
    parser.add_argument("--loaded-sort-by-pose-distance", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--loaded-stop-on-first-success", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--loaded-updown", type=float, default=0.3)
    parser.add_argument(
        "--loaded-preserve-lower-updown",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="回负重姿态时仅把高于 --loaded-updown 的 updown 降到上限，低位保持不升高",
    )
    parser.add_argument("--lateral-shift-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--lateral-shift-distance", type=float, default=0.5)
    parser.add_argument("--lateral-shift-step", type=float, default=0.01)
    parser.add_argument("--lateral-shift-column", type=int, default=2)
    parser.add_argument("--pre-lower-left-box-id", type=int, default=0)
    parser.add_argument("--pre-lower-right-box-id", type=int, default=0)
    parser.add_argument("--pre-lower-updown-delta", type=float, default=0.0)
    parser.add_argument("--dedup-joint-threshold-deg", type=float, default=1.0)
    parser.add_argument("--dedup-h-threshold", type=float, default=0.005)
    parser.add_argument("--extract-ik-stratified-limit-enabled", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--extract-ik-stratified-h-bucket", type=float, default=0.05)
    parser.add_argument("--extract-ik-stratified-top-score-count", type=int, default=12)
    parser.add_argument("--extract-ik-candidate-reserve-limit", type=int, default=64)
    parser.add_argument("--extract-ik-candidate-reserve-stratified", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--extract-ik-candidate-reserve-interleave-stride", type=int, default=4)
    parser.add_argument("--extract-ik-loaded-distance-order-weight", type=float, default=0.0)
    parser.add_argument("--extract-monitor-build-final-replay", action=argparse.BooleanOptionalAction, default=True)
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
    if args.left_grasp_mode is None:
        args.left_grasp_mode = args.grasp_mode
    if args.right_grasp_mode is None:
        args.right_grasp_mode = args.grasp_mode
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
                args.left_grasp_mode == "top_suction",
                args.right_grasp_mode == "top_suction",
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
