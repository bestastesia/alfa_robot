#!/usr/bin/python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import extract_stage_monitor_console as monitor
import process_lifecycle

import numpy as np


DEFAULT_OUTPUT_ROOT = Path("/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/extract_sequence_rerun")
DEFAULT_SEQUENCE = "1,3;1,6;4,3;4,6;4,9;7,6;7,9;7,12;10,9;10,12;10,15;13,12;13,15"
DEFAULT_LOADED_POSE_FAMILY_DEG = "[0.0,-90.0,120.0,-75.0,0.0,0.0]"
DEFAULT_PRE_PLACE_POSE_DEG = "[0.0,-90.0,120.0,-75.0,0.0,0.0]"
FRONT_SUCTION_BOX_IDS = {1, 3, 4, 6}
OUTER_GRASP_TARGET_Y_M = 0.40
TASK_LAYOUT_Y_OFFSETS = {
    "centered": 0.0,
    "right_shift_0p1": 0.05,
}
FRONT_TOOL_ORIENTATION_XYZW = [0.70710678, 0.0, 0.70710678, 0.0]
TOP_TOOL_ORIENTATION_XYZW = [1.0, 0.0, 0.0, 0.0]


def point_positions_by_name(joint_names: list[str], point: dict[str, Any]) -> dict[str, float]:
    positions = point.get("positions", [])
    result: dict[str, float] = {}
    for index, name in enumerate(joint_names):
        if index < len(positions):
            result[name] = float(positions[index])
    return result


def accumulate_joint_distance(
    previous: dict[str, float] | None,
    current: dict[str, float],
) -> tuple[float, float, dict[str, float] | None]:
    if previous is None:
        return 0.0, 0.0, current
    mixed_total = 0.0
    revolute_total = 0.0
    for name, value in current.items():
        if name not in previous:
            continue
        delta = abs(value - previous[name])
        mixed_total += delta
        if name != "updown":
            revolute_total += delta
    return mixed_total, revolute_total, current


def stage_kind(stage: dict[str, Any]) -> str:
    name = str(stage.get("stage", ""))
    extra = stage.get("extra", {})
    if isinstance(extra, dict):
        kind = str(extra.get("stage_kind", ""))
        if kind:
            return kind
    if "pre_attach" in name:
        return "pre_attach"
    if "extract" in name:
        return "extract"
    if "lateral" in name:
        return "lateral_shift"
    if "loaded" in name:
        return "loaded_plan"
    return "other"


def summarize_snapshot_motion(snapshot: dict[str, Any]) -> dict[str, Any]:
    totals = {
        "motion_total_axis_mixed": 0.0,
        "motion_total_joint_rad": 0.0,
        "motion_total_joint_deg": 0.0,
        "motion_pre_attach_joint_rad": 0.0,
        "motion_extract_joint_rad": 0.0,
        "motion_lateral_joint_rad": 0.0,
        "motion_loaded_joint_rad": 0.0,
        "motion_other_joint_rad": 0.0,
        "motion_point_count": 0,
        "motion_stage_count": 0,
    }
    previous: dict[str, float] | None = None
    for stage in snapshot.get("replay_stages", []):
        if not isinstance(stage, dict):
            continue
        trajectory = stage.get("trajectory", {})
        if not isinstance(trajectory, dict):
            continue
        joint_names = [str(name) for name in trajectory.get("joint_names", [])]
        points = trajectory.get("points", [])
        if not joint_names or not isinstance(points, list):
            continue
        totals["motion_stage_count"] += 1
        kind = stage_kind(stage)
        bucket = {
            "pre_attach": "motion_pre_attach_joint_rad",
            "extract": "motion_extract_joint_rad",
            "lateral_shift": "motion_lateral_joint_rad",
            "post_extract_loaded_plan": "motion_loaded_joint_rad",
            "loaded_plan": "motion_loaded_joint_rad",
        }.get(kind, "motion_other_joint_rad")
        for point in points:
            if not isinstance(point, dict):
                continue
            current = point_positions_by_name(joint_names, point)
            if not current:
                continue
            mixed, revolute, previous = accumulate_joint_distance(previous, current)
            totals["motion_total_axis_mixed"] += mixed
            totals["motion_total_joint_rad"] += revolute
            totals[bucket] += revolute
            totals["motion_point_count"] += 1
    totals["motion_total_joint_deg"] = totals["motion_total_joint_rad"] * 180.0 / math.pi

    records = [record for record in snapshot.get("records", []) if isinstance(record, dict)]
    selected = next((record for record in records if record.get("loaded_plan_selected")), None)
    if selected is None:
        selected = next((record for record in records if record.get("success")), None)
    if selected is None and records:
        selected = records[0]
    if selected is not None:
        totals.update(
            {
                "selected_candidate_order": selected.get("candidate_order", ""),
                "selected_h": selected.get("h", ""),
                "selected_h_index": selected.get("h_index", ""),
                "selected_seed_index": selected.get("seed_index", ""),
                "selected_ik_score": selected.get("ik_score", ""),
                "selected_loaded_plan_rank": selected.get("loaded_plan_rank", ""),
                "selected_loaded_plan_trajectory_distance": selected.get("loaded_plan_trajectory_distance", ""),
                "selected_loaded_pose_distance_sum": selected.get("loaded_pose_distance_sum", ""),
                "selected_loaded_pose_max_joint_delta": selected.get("loaded_pose_max_joint_delta", ""),
            }
        )
    return totals


def parse_pair_sequence(value: str) -> list[tuple[int, int]]:
    pairs: list[tuple[int, int]] = []
    for segment in value.split(";"):
        segment = segment.strip()
        if not segment:
            continue
        sep = "," if "," in segment else "/"
        parts = [part.strip() for part in segment.split(sep)]
        if len(parts) != 2:
            raise ValueError(f"invalid pair segment: {segment}")
        pairs.append((int(parts[0]), int(parts[1])))
    if not pairs:
        raise ValueError("empty pair sequence")
    return pairs


def parse_grasp_mode_sequence(value: str, count: int, default_mode: str) -> list[str]:
    if not value.strip():
        return [default_mode] * count
    modes = [segment.strip() for segment in value.split(";") if segment.strip()]
    aliases = {
        "front": "front",
        "side": "front",
        "side_suction": "front",
        "top": "top_suction",
        "top_suction": "top_suction",
        "down": "top_suction",
    }
    normalized: list[str] = []
    for mode in modes:
        key = mode.lower()
        if key not in aliases:
            raise ValueError(f"invalid grasp mode in sequence: {mode}")
        normalized.append(aliases[key])
    if len(normalized) != count:
        raise ValueError(f"grasp mode count {len(normalized)} != pair count {count}")
    return normalized


def grasp_mode_for_box(box_id: int) -> str:
    return "front" if int(box_id) in FRONT_SUCTION_BOX_IDS else "top_suction"


def pair_vehicle_mode(left_mode: str, right_mode: str) -> str:
    return "top_suction" if "top_suction" in (left_mode, right_mode) else "front"


def promote_mixed_grasp_modes_to_top(
    left_modes: list[str],
    right_modes: list[str],
) -> tuple[list[str], list[str]]:
    if len(left_modes) != len(right_modes):
        raise ValueError("left/right grasp mode counts differ")
    promoted_left = list(left_modes)
    promoted_right = list(right_modes)
    for index, (left_mode, right_mode) in enumerate(zip(left_modes, right_modes)):
        if left_mode != right_mode:
            promoted_left[index] = "top_suction"
            promoted_right[index] = "top_suction"
    return promoted_left, promoted_right


def parse_arm_grasp_mode_sequence(value: str, pairs: list[tuple[int, int]], side: str) -> list[str]:
    if not value.strip():
        index = 0 if side == "left" else 1
        return [grasp_mode_for_box(pair[index]) for pair in pairs]
    return parse_grasp_mode_sequence(value, len(pairs), "front")


def effective_box_front_x(args: argparse.Namespace, grasp_mode: str) -> float:
    if grasp_mode == "top_suction":
        top_box_front_x = getattr(args, "top_box_front_x", None)
        if top_box_front_x is not None:
            return float(top_box_front_x)
        return float(args.box_front_x) - float(getattr(args, "top_approach_forward", 0.0))
    return float(args.box_front_x)


def extract_rollout_mode_for_pair(
    args: argparse.Namespace,
    left_box_id: int,
    right_box_id: int,
    grasp_mode: str,
) -> str:
    requested = str(args.extract_rollout_mode)
    if requested == "auto":
        return requested
    if requested == "equal_height_five_row_hybrid":
        if grasp_mode == "top_suction":
            if {int(left_box_id), int(right_box_id)} == {7, 9}:
                return "box_pose_rrt"
            return str(args.top_extract_rollout_mode)
        if {int(left_box_id), int(right_box_id)} == {1, 3}:
            return "box_pose_rrt"
        return "projected_shortcut"
    if grasp_mode == "top_suction":
        return str(args.top_extract_rollout_mode)
    return requested


def extract_search_limits_for_pair(
    args: argparse.Namespace,
    rollout_mode: str,
) -> tuple[int, float]:
    success_quorum = int(args.extract_success_quorum)
    max_joint_delta = float(args.extract_max_joint_delta)
    if str(args.extract_rollout_mode) != "equal_height_five_row_hybrid":
        return success_quorum, max_joint_delta
    if rollout_mode == "projected_shortcut":
        if success_quorum > 0:
            success_quorum = max(3, success_quorum)
        max_joint_delta = min(max_joint_delta, math.radians(10.0))
    elif success_quorum > 0:
        success_quorum = 1
    return success_quorum, max_joint_delta


def sample_l1_position_jitter(
    rng: np.random.Generator,
    max_l1_m: float,
) -> list[float]:
    if max_l1_m <= 0.0:
        return [0.0, 0.0, 0.0]
    weights = rng.exponential(1.0, size=3)
    weights /= float(np.sum(weights))
    radius = float(max_l1_m) * float(rng.random()) ** (1.0 / 3.0)
    signs = rng.choice(np.array([-1.0, 1.0]), size=3)
    return [float(value) for value in radius * weights * signs]


def safe_y_fallback_position_jitter(position_jitter: list[float]) -> list[float]:
    if len(position_jitter) != 3:
        raise ValueError("position_jitter must contain x/y/z")
    return [float(position_jitter[0]), 0.0, float(position_jitter[2])]


def explicit_grasp_target(
    args: argparse.Namespace,
    box_id: int,
    grasp_mode: str,
    position_jitter: list[float] | None = None,
) -> dict[str, Any]:
    boxes = monitor.all_boxes(float(args.box_front_x), float(args.scene_y_shift))
    if box_id not in boxes:
        raise ValueError(f"unknown box id: {box_id}")
    box_x, box_y, box_z = boxes[box_id]
    if grasp_mode == "top_suction":
        position = [
            box_x + float(args.top_suction_x_offset),
            box_y,
            box_z + float(args.top_suction_z_offset) - float(args.world_to_base_z),
        ]
        orientation = TOP_TOOL_ORIENTATION_XYZW
    else:
        position = [box_x, box_y, box_z - float(args.world_to_base_z)]
        orientation = FRONT_TOOL_ORIENTATION_XYZW
    position[1] = (
        OUTER_GRASP_TARGET_Y_M if box_id % 3 == 1 else -OUTER_GRASP_TARGET_Y_M
    ) + float(args.scene_y_shift)
    if position_jitter is not None:
        if len(position_jitter) != 3:
            raise ValueError("position_jitter must contain x/y/z")
        position = [
            value + float(delta)
            for value, delta in zip(position, position_jitter)
        ]
    return {
        "frame_id": "base_link",
        "position": position,
        "orientation": orientation,
    }


def make_pair_args(
    args: argparse.Namespace,
    left_id: int,
    right_id: int,
    grasp_mode: str | None = None,
    left_grasp_mode: str | None = None,
    right_grasp_mode: str | None = None,
    task_layout: str = "centered",
    scene_y_shift: float | None = None,
) -> SimpleNamespace:
    left_mode = left_grasp_mode or grasp_mode_for_box(left_id)
    right_mode = right_grasp_mode or grasp_mode_for_box(right_id)
    mode = grasp_mode or pair_vehicle_mode(left_mode, right_mode)
    lateral_shift_enabled = args.lateral_shift_enabled
    if args.lateral_shift_enabled_auto:
        lateral_shift_enabled = left_mode == "front" or right_mode == "front"
    loaded_preferred_pose_index = args.loaded_preferred_pose_index
    loaded_left_pose_family_deg = args.loaded_left_pose_family_deg
    loaded_right_pose_family_deg = args.loaded_right_pose_family_deg
    if args.top_loaded_left_pose_family_deg or args.top_loaded_right_pose_family_deg:
        raise ValueError(
            "顶吸不再使用独立负重姿态族；所有任务统一先经过初始位，再到 --loaded-*-pose-family-deg"
        )
    rollout_mode = extract_rollout_mode_for_pair(args, left_id, right_id, mode)
    extract_success_quorum, extract_max_joint_delta = extract_search_limits_for_pair(
        args, rollout_mode
    )
    return SimpleNamespace(
        box_front_x=effective_box_front_x(args, mode),
        top_box_front_x=effective_box_front_x(args, mode),
        top_approach_forward=0.0,
        scene_y_shift=args.scene_y_shift if scene_y_shift is None else scene_y_shift,
        task_layout=task_layout,
        container_height=args.container_height,
        world_to_base_z=args.world_to_base_z,
        fixed_updown=args.fixed_updown,
        turn_rad=math.radians(args.turn_deg),
        grasp_mode=mode,
        left_grasp_mode=left_mode,
        right_grasp_mode=right_mode,
        front_z_reach_lower=args.front_z_reach_lower,
        front_z_reach_upper=args.front_z_reach_upper,
        top_z_reach_lower=args.top_z_reach_lower,
        top_z_reach_upper=args.top_z_reach_upper,
        top_suction_x_offset=args.top_suction_x_offset,
        top_suction_z_offset=args.top_suction_z_offset,
        ik_top_position_tolerance=args.ik_top_position_tolerance,
        ik_top_orientation_tolerance_deg=args.ik_top_orientation_tolerance_deg,
        ik_h_candidate_count=args.ik_h_candidate_count,
        ik_h_lower=args.ik_h_lower,
        ik_h_upper=args.ik_h_upper,
        ik_h_step=args.ik_h_step,
        ik_full_h_range_scan=args.ik_full_h_range_scan,
        ik_seed_count=args.ik_seed_count,
        ik_workers=args.ik_workers,
        ik_candidate_timeout=args.ik_candidate_timeout,
        ik_try_target_orders=args.ik_try_target_orders,
        ik_use_reversed_target_order=args.ik_use_reversed_target_order,
        optimized_ik_check_collision=args.optimized_ik_check_collision,
        left_box_id=left_id,
        right_box_id=right_id,
        target_y_fallback_enabled=args.target_y_fallback_enabled,
        extract_workers=args.extract_workers,
        extract_success_quorum=extract_success_quorum,
        extract_quality_success_quorum=args.extract_quality_success_quorum,
        extract_quality_loaded_distance_sum=args.extract_quality_loaded_distance_sum,
        candidate_limit=args.candidate_limit,
        extract_step_x=args.extract_step_x,
        extract_max_joint_delta=extract_max_joint_delta,
        extract_rrt=args.extract_rrt,
        extract_rrt_planning_group=args.extract_rrt_planning_group,
        extract_rrt_planning_time=args.extract_rrt_planning_time,
        extract_rrt_planning_attempts=args.extract_rrt_planning_attempts,
        extract_rrt_endpoint_per_arm_limit=args.extract_rrt_endpoint_per_arm_limit,
        extract_rrt_goal_limit=args.extract_rrt_goal_limit,
        extract_rollout_mode=rollout_mode,
        extract_top_updown_lift_distance=args.extract_top_updown_lift_distance,
        extract_top_updown_retreat_distance=args.extract_top_updown_retreat_distance,
        extract_box_pose_rrt_edge_scene_collision=args.extract_box_pose_rrt_edge_scene_collision,
        extract_box_pose_rrt_max_iterations=(
            max(400, args.extract_box_pose_rrt_max_iterations)
            if {left_id, right_id} == {13, 15}
            else args.extract_box_pose_rrt_max_iterations
        ),
        extract_box_pose_rrt_paths_per_arm=args.extract_box_pose_rrt_paths_per_arm,
        extract_box_pose_rrt_path_pair_limit=args.extract_box_pose_rrt_path_pair_limit,
        extract_box_pose_rrt_max_lift=args.extract_box_pose_rrt_max_lift,
        extract_box_pose_rrt_max_pitch_deg=args.extract_box_pose_rrt_max_pitch_deg,
        extract_box_pose_rrt_parent_candidates=args.extract_box_pose_rrt_parent_candidates,
        extract_box_pose_rrt_parent_diverse_candidates=args.extract_box_pose_rrt_parent_diverse_candidates,
        extract_box_pose_rrt_parent_endpoint_score_weight=args.extract_box_pose_rrt_parent_endpoint_score_weight,
        extract_box_pose_rrt_parent_node_score_weight=args.extract_box_pose_rrt_parent_node_score_weight,
        extract_box_pose_rrt_parent_density_weight=args.extract_box_pose_rrt_parent_density_weight,
        extract_box_pose_rrt_max_lateral=args.extract_box_pose_rrt_max_lateral,
        extract_box_pose_rrt_step_lateral=args.extract_box_pose_rrt_step_lateral,
        extract_box_pose_rrt_front_free_motion=args.extract_box_pose_rrt_front_free_motion,
        extract_box_pose_rrt_front_goal_requires_max_pitch=args.extract_box_pose_rrt_front_goal_requires_max_pitch,
        extract_box_pose_rrt_front_goal_requires_horizontal_detachment=(
            args.extract_box_pose_rrt_front_goal_requires_horizontal_detachment
        ),
        extract_box_pose_rrt_best_first_fallback=args.extract_box_pose_rrt_best_first_fallback,
        extract_box_pose_rrt_best_first_first=args.extract_box_pose_rrt_best_first_first,
        extract_box_pose_rrt_top_best_first_first=args.extract_box_pose_rrt_top_best_first_first,
        extract_box_pose_rrt_top_goal_min_pitch_deg=args.extract_box_pose_rrt_top_goal_min_pitch_deg,
        extract_box_pose_rrt_best_first_max_expansions=args.extract_box_pose_rrt_best_first_max_expansions,
        extract_box_pose_rrt_best_first_heuristic_weight=args.extract_box_pose_rrt_best_first_heuristic_weight,
        dedup_joint_threshold_deg=args.dedup_joint_threshold_deg,
        dedup_h_threshold=args.dedup_h_threshold,
        extract_ik_stratified_limit_enabled=args.extract_ik_stratified_limit_enabled,
        extract_ik_stratified_h_bucket=args.extract_ik_stratified_h_bucket,
        extract_ik_stratified_top_score_count=args.extract_ik_stratified_top_score_count,
        extract_ik_candidate_reserve_limit=args.extract_ik_candidate_reserve_limit,
        extract_ik_candidate_reserve_stratified=args.extract_ik_candidate_reserve_stratified,
        extract_ik_candidate_reserve_interleave_stride=args.extract_ik_candidate_reserve_interleave_stride,
        extract_ik_loaded_distance_order_weight=args.extract_ik_loaded_distance_order_weight,
        extract_projected_joint4_positive_penalty_weight=args.extract_projected_joint4_positive_penalty_weight,
        extract_monitor_build_final_replay=args.place_cycle_enabled or not args.no_rerun,
        loaded_candidate_limit=args.loaded_candidate_limit,
        lateral_shift_enabled=lateral_shift_enabled,
        lateral_shift_distance=args.lateral_shift_distance,
        lateral_shift_step=args.lateral_shift_step,
        lateral_shift_column=args.lateral_shift_column,
        pre_lower_left_box_id=args.pre_lower_left_box_id,
        pre_lower_right_box_id=args.pre_lower_right_box_id,
        pre_lower_updown_delta=args.pre_lower_updown_delta,
        loaded_updown=args.loaded_updown,
        loaded_preserve_lower_updown=args.loaded_preserve_lower_updown,
        loaded_planner_id=args.loaded_planner_id,
        loaded_planning_mode=args.loaded_planning_mode,
        loaded_planning_time=args.loaded_planning_time,
        loaded_planning_attempts=args.loaded_planning_attempts,
        loaded_workers=args.loaded_workers,
        loaded_sort_by_pose_distance=args.loaded_sort_by_pose_distance,
        loaded_stop_on_first_success=args.loaded_stop_on_first_success,
        loaded_preferred_pose_index=loaded_preferred_pose_index,
        loaded_left_pose_family_deg=loaded_left_pose_family_deg,
        loaded_right_pose_family_deg=loaded_right_pose_family_deg,
        place_cycle_enabled=args.place_cycle_enabled,
        place_updown=args.place_updown,
        place_transition_updown=args.place_transition_updown,
        pre_place_left_pose_deg=args.pre_place_left_pose_deg,
        pre_place_right_pose_deg=args.pre_place_right_pose_deg,
        place_left_pose_deg=args.place_left_pose_deg,
        place_right_pose_deg=args.place_right_pose_deg,
        service_timeout=args.service_timeout,
        stride=args.stride,
        continue_on_failure=args.continue_on_failure,
        extract_only=args.extract_only,
        ik_only_raw=args.ik_only_raw,
        ik_scene_rejected=args.ik_scene_rejected,
        start_support_nodes=not args.external_control_stack,
        planning_joint_states_topic=args.planning_joint_states_topic,
    )


def wait_until_service_gone(timeout: float = 15.0) -> bool:
    return monitor.wait_until_planner_services_gone(timeout)


def cleanup_planner_processes() -> bool:
    process_lifecycle.request_stale_planner_shutdown()
    return wait_until_service_gone(15.0)


def display_base_transform(args: argparse.Namespace) -> np.ndarray:
    yaw = math.radians(float(getattr(args, "display_base_yaw_deg", 0.0)))
    c = math.cos(yaw)
    s = math.sin(yaw)
    transform = np.eye(4)
    transform[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    transform[:3, 3] = [float(getattr(args, "display_base_x", 0.0)), float(getattr(args, "display_base_y", 0.0)), 0.0]
    return transform


def display_scene_y_shift(args: argparse.Namespace) -> float:
    return float(args.scene_y_shift) + float(getattr(args, "display_base_y", 0.0))


def log_robot_state_display(helpers: Any, robot: Any, joints: dict[str, float], path: str, args: argparse.Namespace) -> None:
    base_tf = display_base_transform(args)
    display_joints = dict(joints)
    display_joints["turn"] = float(display_joints.get("turn", 0.0)) + math.radians(float(getattr(args, "display_turn_offset_deg", 0.0)))
    transforms = robot.fk(display_joints)
    for link_name, transform in transforms.items():
        helpers.log_transform_matrix(f"{path}/{link_name}", base_tf @ transform)


def log_attached_boxes_display(
    robot: Any,
    joints: dict[str, float],
    attached_boxes: list[dict[str, Any]],
    args: argparse.Namespace,
    collision_ids: set[str] | None = None,
) -> None:
    if not attached_boxes:
        monitor.rr.log("monitor/scene/attached_boxes", monitor.rr.Clear(recursive=True))
        return
    display_joints = dict(joints)
    display_joints["turn"] = float(display_joints.get("turn", 0.0)) + math.radians(float(getattr(args, "display_turn_offset_deg", 0.0)))
    fk = robot.fk(display_joints)
    base_tf = display_base_transform(args)
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
        world_link_tf = base_tf @ link_tf
        center = world_link_tf @ np.array([float(center_in_link[0]), float(center_in_link[1]), float(center_in_link[2]), 1.0])
        centers.append(center[:3].tolist())
        half_sizes.append([float(value) * 0.5 for value in size])
        quaternions.append(monitor.matrix_to_quaternion(world_link_tf[:3, :3]))
        box_id = str(box.get("id", "carried_box"))
        colors.append(
            [255, 30, 30, 230]
            if collision_ids and box_id in collision_ids
            else [40, 220, 90, 150]
        )
        labels.append(box_id)
    monitor.rr.log(
        "monitor/scene/attached_boxes",
        monitor.rr.Boxes3D(centers=centers, half_sizes=half_sizes, quaternions=quaternions, colors=colors, labels=labels),
    )


def log_sequence_replay(
    snapshot: dict[str, Any],
    helpers: Any,
    robot: Any,
    args: argparse.Namespace,
    task_index: int,
    pair_count: int,
    sample_start: int,
) -> int:
    replay_stages = list(snapshot.get("replay_stages", []))
    replay_source = "selected"
    if not replay_stages:
        records = list(snapshot.get("records", []))

        def failure_record_score(record: dict[str, Any]) -> tuple[int, int, int, float]:
            stages = list(record.get("replay_stages", []))
            return (
                1 if record.get("loaded_plan_attempted") else 0,
                1 if record.get("lateral_shift_success") else 0,
                len(stages),
                -float(record.get("loaded_plan_rank", 999999)),
            )

        records = [record for record in records if record.get("replay_stages")]
        if records:
            record = max(records, key=failure_record_score)
            replay_stages = list(record.get("replay_stages", []))
            replay_source = (
                "failed_candidate "
                f"rank={record.get('loaded_plan_rank')} "
                f"loaded_success={record.get('loaded_plan_success')} "
                f"reason={record.get('loaded_plan_failure_reason') or record.get('failure_reason')}"
            )
    left_id = int(snapshot.get("left_box_id", 0))
    right_id = int(snapshot.get("right_box_id", 0))
    container_panels = snapshot.get("container_panels")
    sample = sample_start
    previous_positions: list[float] | None = None
    previous_joint_names: list[str] | None = None

    for stage_index, stage in enumerate(replay_stages):
        points = monitor.playback_points_for_stage(stage)
        if not points:
            continue
        joint_names = list(stage.get("trajectory", {}).get("joint_names", []))
        if previous_positions is not None and previous_joint_names == joint_names:
            first_positions = [float(value) for value in points[0].get("positions", [])]
            bridge_points = monitor.densify_stage_points([
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
            collision_ids = (
                monitor.collision_ids_for_stage(stage)
                if point_index == selected_indices[-1]
                else set()
            )
            monitor.log_container_panels(
                container_panels,
                collision_ids=collision_ids,
            )
            monitor.log_static_box_obstacles(
                stage.get("static_box_obstacles"),
                collision_ids,
            )
            point = points[point_index]
            joints = monitor.joint_dict_from_stage_point(stage, point)
            log_robot_state_display(helpers, robot, joints, "monitor/robot", args)
            log_attached_boxes_display(
                robot,
                joints,
                stage.get("attached_boxes", []),
                args,
                collision_ids,
            )
            monitor.rr.log(
                "monitor/info",
                monitor.rr.TextLog(
                    f"任务 {task_index}/{pair_count}: L{left_id}/R{right_id} | "
                    f"{replay_source} | "
                    f"stage {stage_index + 1}/{len(replay_stages)}: {stage.get('stage')} | "
                    f"point {point_index + 1}/{len(points)}"
                ),
            )
            extra = stage.get("extra", {})
            if isinstance(extra, dict) and extra.get("collision_diagnostic"):
                collision_frame = not bool(extra.get("accepted", True))
                monitor.rr.log(
                    "monitor/diagnostics/collision_frame",
                    monitor.rr.Points3D(
                        positions=[[0.0, 0.0, 0.0]],
                        radii=[0.08],
                        colors=[[255, 30, 30, 255] if collision_frame else [255, 210, 30, 255]],
                        labels=[str(extra.get("collision_reason", "collision diagnostic"))],
                    ),
                )
            sample += 1
        previous_positions = [float(value) for value in points[-1].get("positions", [])]
        previous_joint_names = joint_names
    return sample - sample_start


def log_raw_ik_records(
    snapshot: dict[str, Any],
    helpers: Any,
    robot: Any,
    args: argparse.Namespace,
    task_index: int,
    pair_count: int,
    sample_start: int,
) -> int:
    records = [record for record in snapshot.get("records", []) if isinstance(record, dict)]
    left_id = int(snapshot.get("left_box_id", 0))
    right_id = int(snapshot.get("right_box_id", 0))
    container_panels = snapshot.get("container_panels")
    sample = sample_start
    for record_index, record in enumerate(records):
        joint_map = record.get("state", {}).get("joint_map", {})
        if not isinstance(joint_map, dict) or not joint_map:
            continue
        helpers.set_sample_time(sample)
        monitor.log_container_panels(container_panels)
        log_robot_state_display(
            helpers,
            robot,
            {str(name): float(value) for name, value in joint_map.items()},
            "monitor/robot",
            args,
        )
        monitor.rr.log("monitor/scene/attached_boxes", monitor.rr.Clear(recursive=True))
        monitor.rr.log(
            "monitor/info",
            monitor.rr.TextLog(
                f"任务 {task_index}/{pair_count}: L{left_id}/R{right_id} | "
                f"代价函数前原始合法解 {record_index + 1}/{len(records)} | "
                f"generation={record.get('generation_index', record_index)} "
                f"h={float(record.get('h', 0.0)):.4f} "
                f"h_index={record.get('h_index')} seed_index={record.get('seed_index')} | "
                "未打分、未排序、未去重、未做附着箱场景过滤"
            ),
        )
        sample += 1
    return sample - sample_start


def log_scene_rejected_ik_records(
    snapshot: dict[str, Any],
    helpers: Any,
    robot: Any,
    args: argparse.Namespace,
    task_index: int,
    pair_count: int,
    sample_start: int,
) -> int:
    records = [record for record in snapshot.get("scene_rejected_records", []) if isinstance(record, dict)]
    left_id = int(snapshot.get("left_box_id", 0))
    right_id = int(snapshot.get("right_box_id", 0))
    attached_boxes = snapshot.get("attached_boxes", [])
    container_panels = snapshot.get("container_panels")
    sample = sample_start
    for record_index, record in enumerate(records):
        joint_map = record.get("state", {}).get("joint_map", {})
        if not isinstance(joint_map, dict) or not joint_map:
            continue
        joints = {str(name): float(value) for name, value in joint_map.items()}
        reason = str(record.get("scene_rejection_reason", "ik_candidate_scene_rejected"))
        helpers.set_sample_time(sample)
        monitor.log_container_panels(container_panels)
        monitor.log_static_box_obstacles(snapshot.get("static_box_obstacles"))
        log_robot_state_display(helpers, robot, joints, "monitor/robot", args)
        log_attached_boxes_display(robot, joints, attached_boxes, args)
        monitor.rr.log(
            "monitor/diagnostics/scene_rejection",
            monitor.rr.Points3D(
                positions=[[0.0, 0.0, 0.0]],
                radii=[0.09],
                colors=[[255, 35, 35, 255]],
                labels=[reason],
            ),
        )
        monitor.rr.log(
            "monitor/info",
            monitor.rr.TextLog(
                f"任务 {task_index}/{pair_count}: L{left_id}/R{right_id} | "
                f"附着场景拒绝 IK {record_index + 1}/{len(records)} | "
                f"h={float(record.get('h', 0.0)):.4f} score={float(record.get('score', 0.0)):.4f} | "
                f"原因：{reason}"
            ),
        )
        sample += 1
    return sample - sample_start


def log_failure_marker(
    helpers: Any,
    robot: Any,
    args: argparse.Namespace,
    task_index: int,
    pair_count: int,
    left_id: int,
    right_id: int,
    sample_start: int,
    output: str,
) -> int:
    helpers.set_sample_time(sample_start)
    # 规划失败时没有 snapshot，也就没有权威的集装箱碰撞几何可画；不再画硬编码的
    # 集装箱壳/3x4 箱堆（那属于"仅为好看"的伪几何）。仅保留下方的目标点标记，
    # 目标点由 all_boxes 查表得到，与规划器 make_boxes 推导抓取目标同源。
    box_front_x = effective_box_front_x(args, getattr(args, "grasp_mode", "front"))
    boxes = monitor.all_boxes(box_front_x, args.scene_y_shift)
    target_centers = []
    target_labels = []
    for side, box_id, side_mode in (
        ("L", left_id, getattr(args, "left_grasp_mode", getattr(args, "grasp_mode", "front"))),
        ("R", right_id, getattr(args, "right_grasp_mode", getattr(args, "grasp_mode", "front"))),
    ):
        if box_id not in boxes:
            continue
        box_x, box_y, box_z = boxes[box_id]
        if side_mode == "top_suction":
            target_centers.append([
                box_x + float(args.top_suction_x_offset),
                box_y,
                box_z + float(args.top_suction_z_offset),
            ])
        else:
            target_centers.append([box_x, box_y, box_z])
        target_labels.append(f"{side}{box_id}_{side_mode}_target")
    if target_centers:
        monitor.rr.log(
            "monitor/scene/grasp_targets",
            monitor.rr.Points3D(
                positions=target_centers,
                radii=[0.04 for _ in target_centers],
                colors=[[80, 220, 255, 255] for _ in target_centers],
                labels=target_labels,
            ),
        )
    zero_joints = {name: 0.0 for name in robot.joints.keys()}
    zero_joints["updown"] = float(args.fixed_updown)
    helpers.log_robot_state(robot, zero_joints, "monitor/robot")
    monitor.rr.log(
        "monitor/info",
        monitor.rr.TextLog(
            f"任务 {task_index}/{pair_count}: L{left_id}/R{right_id} 失败，未生成可回放轨迹\n{output}"
        ),
    )
    return 1


def run_one_pair(
    args: argparse.Namespace,
    helpers: Any | None,
    robot: Any | None,
    run_root: Path,
    planner: subprocess.Popen[str],
    launch_log: Path,
    service_client: monitor.ExtractMonitorServiceClient,
    startup_ms: float,
    left_id: int,
    right_id: int,
    task_index: int,
    pair_count: int,
    sample_start: int,
) -> tuple[bool, int, dict[str, Any]]:
    run_dir = run_root / f"{task_index:02d}_L{left_id}_R{right_id}"
    run_dir.mkdir(parents=True, exist_ok=True)
    snapshot_path = run_dir / "stage_snapshot.json"
    primary_failure_snapshot_path = run_dir / "stage_snapshot_primary_failure.json"
    print(
        f"\n===== 任务 {task_index}/{pair_count}: "
        f"[{getattr(args, 'task_layout', 'centered')}] L{left_id}/R{right_id} ====="
    )
    try:
        config_ok, config_output, config_ms = service_client.configure(
            left_id,
            right_id,
            snapshot_path,
            args.service_timeout,
            args.left_grasp_mode == "top_suction",
            args.right_grasp_mode == "top_suction",
            explicit_grasp_target(
                args,
                left_id,
                args.left_grasp_mode,
                args.left_target_position_jitter,
            ),
            explicit_grasp_target(
                args,
                right_id,
                args.right_grasp_mode,
                args.right_target_position_jitter,
            ),
        )
        print(config_output)
        print(f"任务配置完成：success={config_ok} configure={config_ms:.1f}ms snapshot={snapshot_path}")
        if not config_ok:
            summary = {
                "left": left_id,
                "right": right_id,
                "success": False,
                "startup_ms": startup_ms,
                "configure_ms": config_ms,
                "service_ms": 0.0,
                "wall_ms": 0.0,
                "snapshot": str(snapshot_path),
                "left_target_dx_m": float(args.left_target_position_jitter[0]),
                "left_target_dy_m": float(args.left_target_position_jitter[1]),
                "left_target_dz_m": float(args.left_target_position_jitter[2]),
                "left_target_l1_m": float(sum(abs(value) for value in args.left_target_position_jitter)),
                "right_target_dx_m": float(args.right_target_position_jitter[0]),
                "right_target_dy_m": float(args.right_target_position_jitter[1]),
                "right_target_dz_m": float(args.right_target_position_jitter[2]),
                "right_target_l1_m": float(sum(abs(value) for value in args.right_target_position_jitter)),
                "failure_reason": config_output,
            }
            return False, 1, summary

        ik_stage_ms = 0.0
        extract_stage_ms = 0.0
        primary_success = False
        primary_failure_reason = ""
        primary_service_ms = 0.0
        fallback_attempted = False
        fallback_success = False
        fallback_configure_ms = 0.0
        fallback_service_ms = 0.0
        fallback_failure_reason = ""
        if args.extract_only:
            print("计算开始：IK → 抽离（抽离完成即结束）")
            start = time.monotonic()
            ik_success, ik_output, ik_service_ms = service_client.trigger(args.service_timeout)
            print(ik_output)
            if ik_success:
                ik_stage_ms = float(monitor.read_snapshot(snapshot_path).get("elapsed_ms", 0.0))
                success, output, extract_service_ms = service_client.trigger(args.service_timeout)
                print(output)
                if snapshot_path.exists():
                    extract_stage_ms = float(
                        monitor.read_snapshot(snapshot_path).get("elapsed_ms", 0.0)
                    )
            else:
                success = False
                output = ik_output
                extract_service_ms = 0.0
            elapsed_ms = ik_service_ms + extract_service_ms
            wall_ms = (time.monotonic() - start) * 1000.0
            print(
                f"计算结束：success={success} service={elapsed_ms:.1f}ms wall={wall_ms:.1f}ms "
                f"ik={ik_stage_ms:.1f}ms extract={extract_stage_ms:.1f}ms"
            )
        else:
            if args.ik_only_raw:
                print("计算开始：仅生成代价函数前的全部合法 IK 解")
            else:
                print("计算开始：初始位 → 预接触 → IK吸附位 → 抽离 → 负重位 → 放置位 → 回初始位")
            start = time.monotonic()
            success, output, elapsed_ms = service_client.trigger(args.service_timeout)
            primary_success = success
            primary_service_ms = elapsed_ms
            if not success:
                primary_failure_reason = output
            can_fallback_y = (
                bool(args.target_y_fallback_enabled)
                and (
                    abs(float(args.left_target_position_jitter[1])) > 1e-12
                    or abs(float(args.right_target_position_jitter[1])) > 1e-12
                )
            )
            if not success and can_fallback_y:
                fallback_attempted = True
                if snapshot_path.exists():
                    shutil.copy2(snapshot_path, primary_failure_snapshot_path)
                fallback_left_jitter = safe_y_fallback_position_jitter(
                    args.left_target_position_jitter
                )
                fallback_right_jitter = safe_y_fallback_position_jitter(
                    args.right_target_position_jitter
                )
                print(
                    "原始6D目标失败，启用安全Y降级："
                    f"left_dy={args.left_target_position_jitter[1]:+.4f}->0.0000m "
                    f"right_dy={args.right_target_position_jitter[1]:+.4f}->0.0000m；"
                    "x/z/姿态保持不变"
                )
                fallback_config_ok, fallback_config_output, fallback_configure_ms = (
                    service_client.configure(
                        left_id,
                        right_id,
                        snapshot_path,
                        args.service_timeout,
                        args.left_grasp_mode == "top_suction",
                        args.right_grasp_mode == "top_suction",
                        explicit_grasp_target(
                            args,
                            left_id,
                            args.left_grasp_mode,
                            fallback_left_jitter,
                        ),
                        explicit_grasp_target(
                            args,
                            right_id,
                            args.right_grasp_mode,
                            fallback_right_jitter,
                        ),
                    )
                )
                print(fallback_config_output)
                if fallback_config_ok:
                    fallback_success, fallback_output, fallback_service_ms = (
                        service_client.trigger(args.service_timeout)
                    )
                    success = fallback_success
                    output = fallback_output
                else:
                    success = False
                    output = fallback_config_output
                if not success:
                    fallback_failure_reason = output
                elapsed_ms = primary_service_ms + fallback_service_ms
            wall_ms = (time.monotonic() - start) * 1000.0
            print(output)
            print(
                f"计算结束：success={success} service={elapsed_ms:.1f}ms "
                f"wall={wall_ms:.1f}ms fallback={fallback_attempted}"
            )
        returned_snapshot = monitor.extract_snapshot_path_from_service_output(output)
        if returned_snapshot is not None and returned_snapshot != snapshot_path:
            raise RuntimeError(f"服务连到了旧 planner：expected={snapshot_path}, got={returned_snapshot}")
        summary: dict[str, Any] = {
            "left": left_id,
            "right": right_id,
            "success": success,
            "loaded_planning_mode": getattr(args, "loaded_planning_mode", "rrt"),
            "loaded_planner_id": getattr(args, "loaded_planner_id", ""),
            "startup_ms": startup_ms,
            "configure_ms": config_ms,
            "service_ms": elapsed_ms,
            "wall_ms": wall_ms,
            "primary_success": primary_success,
            "primary_service_ms": primary_service_ms,
            "primary_failure_reason": primary_failure_reason,
            "fallback_attempted": fallback_attempted,
            "fallback_success": fallback_success,
            "fallback_configure_ms": fallback_configure_ms,
            "fallback_service_ms": fallback_service_ms,
            "fallback_failure_reason": fallback_failure_reason,
            "primary_failure_snapshot": (
                str(primary_failure_snapshot_path) if fallback_attempted else ""
            ),
            "snapshot": str(snapshot_path),
            "left_target_dx_m": float(args.left_target_position_jitter[0]),
            "left_target_dy_m": float(args.left_target_position_jitter[1]),
            "left_target_dz_m": float(args.left_target_position_jitter[2]),
            "left_target_l1_m": float(sum(abs(value) for value in args.left_target_position_jitter)),
            "right_target_dx_m": float(args.right_target_position_jitter[0]),
            "right_target_dy_m": float(args.right_target_position_jitter[1]),
            "right_target_dz_m": float(args.right_target_position_jitter[2]),
            "right_target_l1_m": float(sum(abs(value) for value in args.right_target_position_jitter)),
        }
        snapshot: dict[str, Any] | None = None
        sample_count = 0
        if snapshot_path.exists():
            snapshot = monitor.read_snapshot(snapshot_path)
            summary.update(summarize_snapshot_motion(snapshot))
            if helpers is not None and robot is not None:
                if args.ik_scene_rejected:
                    sample_count = log_scene_rejected_ik_records(
                        snapshot, helpers, robot, args, task_index, pair_count, sample_start)
                elif args.ik_only_raw:
                    sample_count = log_raw_ik_records(snapshot, helpers, robot, args, task_index, pair_count, sample_start)
                else:
                    sample_count = log_sequence_replay(snapshot, helpers, robot, args, task_index, pair_count, sample_start)
        if not success and sample_count <= 0 and helpers is not None and robot is not None:
            sample_count = log_failure_marker(
                helpers, robot, args, task_index, pair_count, left_id, right_id, sample_start, output
            )
        if snapshot is None:
            summary.update(
                {
                    "total_ms": wall_ms,
                    "ik_ms": elapsed_ms,
                    "extract_ms": 0.0,
                    "loaded_ms": 0.0,
                    "loaded_plan_batch_wall_ms": 0.0,
                    "loaded_plan_candidate_count": 0,
                    "loaded_plan_attempted_count": 0,
                    "loaded_plan_success_count": 0,
                    "loaded_parallel_workers": 0,
                    "final_ms": 0.0,
                    "loaded_to_place_ms": 0.0,
                    "place_to_loaded_ms": 0.0,
                    "place_to_initial_ms": 0.0,
                    "samples": sample_count,
                    "failure_reason": output,
                }
            )
            return success, sample_count, summary
        place_cycle = snapshot.get("place_cycle", {})
        if not isinstance(place_cycle, dict):
            place_cycle = {}
        summary.update(
            {
                "total_ms": (
                    ik_stage_ms + extract_stage_ms
                    if args.extract_only else float(snapshot.get("elapsed_ms", 0.0))
                ),
                "ik_ms": (
                    ik_stage_ms if args.extract_only else float(snapshot.get("ik_elapsed_ms", 0.0))
                ),
                "extract_ms": (
                    extract_stage_ms if args.extract_only else float(snapshot.get("extract_elapsed_ms", 0.0))
                ),
                "loaded_ms": float(snapshot.get("loaded_elapsed_ms", 0.0)),
                "loaded_plan_batch_wall_ms": float(snapshot.get("loaded_plan_batch_wall_ms", 0.0)),
                "loaded_plan_candidate_count": int(snapshot.get("loaded_plan_candidate_count", 0)),
                "loaded_plan_attempted_count": int(snapshot.get("loaded_plan_attempted_count", 0)),
                "loaded_plan_success_count": int(snapshot.get("loaded_plan_success_count", 0)),
                "loaded_parallel_workers": int(snapshot.get("loaded_parallel_workers", 0)),
                "final_ms": float(snapshot.get("final_elapsed_ms", 0.0)),
                "loaded_to_place_ms": float(place_cycle.get("loaded_to_place_ms", 0.0)),
                "loaded_to_pre_place_ms": float(place_cycle.get("loaded_to_pre_place_ms", 0.0)),
                "pre_place_to_place_ms": float(place_cycle.get("pre_place_to_place_ms", 0.0)),
                "place_to_loaded_ms": float(place_cycle.get("place_to_loaded_ms", 0.0)),
                "place_to_initial_ms": float(
                    place_cycle.get("place_to_initial_ms", place_cycle.get("place_to_loaded_ms", 0.0))
                ),
                "samples": sample_count,
                "failure_reason": "" if success else output,
            }
        )
        return success, sample_count, summary
    finally:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description="生成 13 组等高/相邻层双臂抽箱任务连续全流程 Rerun")
    parser.add_argument(
        "--planner-server",
        action="store_true",
        help="只启动并预热 planner，保持进程常驻；任务由外部配置服务提交。",
    )
    parser.add_argument(
        "--external-control-stack",
        action="store_true",
        help="使用外部真实控制栈的 /joint_states，不启动本地 ros2_control/RSP 支持节点。",
    )
    parser.add_argument(
        "--planning-joint-states-topic",
        default="/joint_states",
        help="MoveIt内部使用的完整模型关节状态；实机模式应指向补齐pitch的只读适配话题。",
    )
    parser.add_argument("--pair-sequence", default=DEFAULT_SEQUENCE)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--save", type=Path, default=None)
    parser.add_argument("--no-rerun", action="store_true", help="不生成 Rerun，只保存 snapshot/summary/stats CSV")
    parser.add_argument("--repeat", type=int, default=1, help="重复运行整组 pair sequence 的次数")
    parser.add_argument(
        "--target-position-jitter-max",
        type=float,
        default=0.0,
        help="左右末端各自独立的三维位置随机误差上限；约束 |dx|+|dy|+|dz| 不超过该值",
    )
    parser.add_argument(
        "--target-position-jitter-seed",
        type=int,
        default=20260807,
        help="末端位置随机误差种子，保证鲁棒性测试可复现",
    )
    parser.add_argument(
        "--target-y-fallback-enabled",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="完整任务失败后仅把左右目标Y恢复到标称安全值，保留请求X/Z和姿态并完整重算",
    )
    parser.add_argument("--stats-csv", type=Path, default=None, help="统计 CSV 输出路径；默认写入 run_root/stats.csv")
    parser.add_argument("--box-front-x", type=float, default=0.90)
    parser.add_argument("--top-approach-forward", type=float, default=0.0, help="顶吸额外前移量；默认0，侧吸顶吸统一使用box-front-x")
    parser.add_argument("--top-box-front-x", type=float, default=0.70, help="顶吸专用箱墙前表面 x；默认 0.70m")
    parser.add_argument("--scene-y-shift", type=float, default=0.0)
    parser.add_argument(
        "--container-height",
        type=float,
        default=2.4,
        help="集装箱内部高度；诊断无顶板约束时可临时设为较大值",
    )
    parser.add_argument(
        "--task-layout",
        choices=["centered", "right_shift_0p1", "both"],
        default="centered",
        help="横向布局：居中（目标y=+0.40/-0.40m）、偏差版（+0.45/-0.35m），或两套连续运行。",
    )
    parser.add_argument("--world-to-base-z", type=float, default=0.202094)
    parser.add_argument("--fixed-updown", type=float, default=0.3)
    parser.add_argument("--turn-deg", type=float, default=0.0)
    parser.add_argument("--display-base-yaw-deg", type=float, default=0.0, help="仅用于 Rerun 回放显示底盘外部 yaw；规划仍使用当前 MoveIt base_link")
    parser.add_argument("--display-base-x", type=float, default=0.0, help="仅用于 Rerun 回放显示底盘外部 x 平移")
    parser.add_argument("--display-base-y", type=float, default=0.0, help="仅用于 Rerun 回放显示底盘外部 y 平移；若 scene_y_shift=-base_y，则显示为世界固定箱墙")
    parser.add_argument("--display-turn-offset-deg", type=float, default=0.0, help="仅用于 Rerun 回放显示外部底盘 yaw 后的 turn 反向补偿")
    parser.add_argument("--grasp-mode", choices=["front", "top_suction"], default="front")
    parser.add_argument("--grasp-mode-sequence", default="", help="每组任务吸附模式，例如 front;front;top_suction。留空则全部使用 --grasp-mode")
    parser.add_argument("--left-grasp-mode-sequence", default="", help="左臂逐任务吸附模式；留空按箱号自动：1/4/7 为侧吸，其余顶吸")
    parser.add_argument("--right-grasp-mode-sequence", default="", help="右臂逐任务吸附模式；留空按箱号自动：3/6/9 为侧吸，其余顶吸")
    parser.add_argument("--front-z-reach-lower", type=float, default=0.45)
    parser.add_argument("--front-z-reach-upper", type=float, default=1.25)
    parser.add_argument("--top-z-reach-lower", type=float, default=0.0)
    parser.add_argument("--top-z-reach-upper", type=float, default=0.6)
    parser.add_argument("--top-suction-x-offset", type=float, default=0.15, help="顶吸目标相对箱子前表面向箱体内部的 x 偏移")
    parser.add_argument("--top-suction-z-offset", type=float, default=0.2, help="顶吸目标相对箱子中心的 z 偏移")
    parser.add_argument("--ik-top-position-tolerance", type=float, default=0.04)
    parser.add_argument("--ik-top-orientation-tolerance-deg", type=float, default=7.0)
    parser.add_argument("--ik-h-candidate-count", type=int, default=64)
    parser.add_argument("--ik-h-lower", type=float, default=0.0)
    parser.add_argument("--ik-h-upper", type=float, default=0.7)
    parser.add_argument("--ik-h-step", type=float, default=0.01)
    parser.add_argument(
        "--ik-full-h-range-scan",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="默认在0~0.7m范围按ik-h-step逐点扫描。",
    )
    parser.add_argument("--ik-seed-count", type=int, default=32)
    parser.add_argument("--ik-workers", type=int, default=1)
    parser.add_argument("--ik-candidate-timeout", type=float, default=0.01)
    parser.add_argument("--ik-try-target-orders", action="store_true")
    parser.add_argument("--ik-use-reversed-target-order", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--optimized-ik-check-collision", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--candidate-limit", type=int, default=25)
    parser.add_argument("--extract-workers", type=int, default=16)
    parser.add_argument(
        "--extract-success-quorum",
        type=int,
        default=3,
        help="抽离阶段达到 N 个成功候选后停止分发后续 IK 候选；0 表示跑完全部候选。",
    )
    parser.add_argument("--extract-quality-success-quorum", type=int, default=0)
    parser.add_argument("--extract-quality-loaded-distance-sum", type=float, default=0.0)
    parser.add_argument("--extract-step-x", type=float, default=0.03)
    parser.add_argument("--extract-max-joint-delta", type=float, default=10.0 * math.pi / 180.0)
    parser.add_argument(
        "--extract-rollout-mode",
        choices=[
            "auto",
            "greedy",
            "projected_shortcut",
            "box_pose_rrt",
            "equal_height_five_row_hybrid",
            "moveit_rrt_legacy",
            "top_lift_legacy",
            "top_updown_lift",
            "direct_updown_lift",
        ],
        default="auto",
        help=(
            "抽离策略；projected_shortcut 将负重关节直连投影为箱体Y固定的X/Z解析IK轨迹；"
            "equal_height_five_row_hybrid 对第一排使用箱体位姿RRT、第二排使用投影Shortcut、"
            "第三排使用updown优先顶吸箱体位姿RRT、第四五排使用顶吸箱体位姿RRT"
        ),
    )
    parser.add_argument(
        "--top-extract-rollout-mode",
        choices=["box_pose_rrt", "top_lift_legacy", "top_updown_lift"],
        default="box_pose_rrt",
        help="顶吸抽离策略；默认采用有界 updown 抬升后接箱体位姿 RRT。",
    )
    parser.add_argument(
        "--extract-top-updown-lift-distance",
        type=float,
        default=0.40,
        help="top_updown_lift 模式固定抬升距离。",
    )
    parser.add_argument(
        "--extract-top-updown-retreat-distance",
        type=float,
        default=0.0,
        help="兼容参数；默认不在抬升阶段同步后抽，抬升结束后统一后抽半个箱深。",
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
    parser.add_argument("--extract-box-pose-rrt-max-lift", type=float, default=0.55)
    parser.add_argument("--extract-box-pose-rrt-max-pitch-deg", type=float, default=90.0)
    parser.add_argument("--extract-box-pose-rrt-parent-candidates", type=int, default=8)
    parser.add_argument("--extract-box-pose-rrt-parent-diverse-candidates", type=int, default=0)
    parser.add_argument("--extract-box-pose-rrt-parent-endpoint-score-weight", type=float, default=0.05)
    parser.add_argument("--extract-box-pose-rrt-parent-node-score-weight", type=float, default=0.0)
    parser.add_argument("--extract-box-pose-rrt-parent-density-weight", type=float, default=0.0)
    parser.add_argument("--extract-box-pose-rrt-max-lateral", type=float, default=0.0)
    parser.add_argument("--extract-box-pose-rrt-step-lateral", type=float, default=0.02)
    parser.add_argument("--extract-box-pose-rrt-front-free-motion", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--extract-box-pose-rrt-front-goal-requires-max-pitch", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--extract-box-pose-rrt-front-goal-requires-horizontal-detachment", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--extract-box-pose-rrt-best-first-fallback", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--extract-box-pose-rrt-best-first-first", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--extract-box-pose-rrt-top-best-first-first", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--extract-box-pose-rrt-top-goal-min-pitch-deg", type=float, default=0.0)
    parser.add_argument("--extract-box-pose-rrt-best-first-max-expansions", type=int, default=800)
    parser.add_argument("--extract-box-pose-rrt-best-first-heuristic-weight", type=float, default=1.0)
    parser.add_argument("--extract-rrt-planning-group", default="dual_arm")
    parser.add_argument("--extract-rrt-planning-time", type=float, default=0.35)
    parser.add_argument("--extract-rrt-planning-attempts", type=int, default=1)
    parser.add_argument("--extract-rrt-endpoint-per-arm-limit", type=int, default=8)
    parser.add_argument("--extract-rrt-goal-limit", type=int, default=8)
    parser.add_argument(
        "--loaded-candidate-limit",
        type=int,
        default=0,
        help="负重规划候选总上限；0表示不硬截断，仍由--loaded-workers控制每批并行度",
    )
    parser.add_argument("--loaded-workers", type=int, default=8)
    parser.add_argument("--loaded-planner-id", default="")
    parser.add_argument("--loaded-planning-mode", choices=["rrt", "shortcut"], default="shortcut")
    parser.add_argument("--loaded-planning-time", type=float, default=1.0)
    parser.add_argument("--loaded-planning-attempts", type=int, default=8)
    parser.add_argument("--loaded-sort-by-pose-distance", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--loaded-stop-on-first-success", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--loaded-updown", type=float, default=0.1)
    parser.add_argument(
        "--loaded-preserve-lower-updown",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="把 --loaded-updown 作为回负重姿态的高度上限，抽离终态低于该值时保持原高度",
    )
    parser.add_argument("--loaded-preferred-pose-index", type=int, default=0)
    parser.add_argument(
        "--loaded-left-pose-family-deg",
        default=DEFAULT_LOADED_POSE_FAMILY_DEG,
    )
    parser.add_argument(
        "--loaded-right-pose-family-deg",
        default=DEFAULT_LOADED_POSE_FAMILY_DEG,
    )
    parser.add_argument("--top-loaded-preferred-pose-index", type=int, default=0)
    parser.add_argument(
        "--top-loaded-left-pose-family-deg",
        default="",
        help="顶吸专用负重姿态族；留空则沿用 --loaded-left-pose-family-deg",
    )
    parser.add_argument(
        "--top-loaded-right-pose-family-deg",
        default="",
        help="顶吸专用负重姿态族；留空则沿用 --loaded-right-pose-family-deg",
    )
    parser.add_argument(
        "--place-cycle-enabled",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="负重后规划到放置姿态（不再绕初始位），释放箱体，再返回初始姿态。",
    )
    parser.add_argument("--place-updown", type=float, default=0.10)
    parser.add_argument("--place-transition-updown", type=float, default=0.10)
    parser.add_argument("--pre-place-left-pose-deg", default=DEFAULT_PRE_PLACE_POSE_DEG)
    parser.add_argument("--pre-place-right-pose-deg", default=DEFAULT_PRE_PLACE_POSE_DEG)
    parser.add_argument(
        "--place-left-pose-deg",
        default="[0.0,-55.0,-50.0,-60.0,0.0,0.0]",
    )
    parser.add_argument(
        "--place-right-pose-deg",
        default="[0.0,-55.0,-50.0,-60.0,0.0,0.0]",
    )
    parser.add_argument("--lateral-shift-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--lateral-shift-enabled-auto", action=argparse.BooleanOptionalAction, default=True, help="按吸附模式自动控制负重前横向让位：侧吸开启，顶吸关闭")
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
    parser.add_argument("--extract-projected-joint4-positive-penalty-weight", type=float, default=2.0)
    parser.add_argument("--service-timeout", type=float, default=120.0)
    parser.add_argument("--startup-retries", type=int, default=1, help="planner 启动超时后的重试次数")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--continue-on-failure", action="store_true")
    parser.add_argument(
        "--extract-only",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="只计算 IK 和抽离；默认运行 IK→抽离→负重→放置→回负重完整循环。",
    )
    parser.add_argument(
        "--ik-only-raw",
        action="store_true",
        help="每组只运行 IK 阶段，并展示进入代价函数前的全部原始合法解",
    )
    parser.add_argument(
        "--ik-scene-rejected",
        action="store_true",
        help="每组只运行正常 IK 阶段，并展示所有被附着场景碰撞过滤拒绝的去重候选",
    )
    parser.add_argument(
        "--ros-domain-id",
        default="auto",
        help="本次 ROS_DOMAIN_ID；auto 隔离自启动序列测试，inherit 表示沿用当前终端。",
    )
    args = parser.parse_args()
    if args.repeat < 1:
        raise ValueError("--repeat must be >= 1")
    if args.target_position_jitter_max < 0.0:
        raise ValueError("--target-position-jitter-max must be >= 0")
    jitter_rng = np.random.default_rng(args.target_position_jitter_seed)
    domain = process_lifecycle.configure_ros_domain(args.ros_domain_id)
    print(f"ROS_DOMAIN_ID={domain if domain is not None else 'unset'}")

    pairs = parse_pair_sequence(args.pair_sequence)
    left_grasp_modes = parse_arm_grasp_mode_sequence(args.left_grasp_mode_sequence, pairs, "left")
    right_grasp_modes = parse_arm_grasp_mode_sequence(args.right_grasp_mode_sequence, pairs, "right")
    left_grasp_modes, right_grasp_modes = promote_mixed_grasp_modes_to_top(
        left_grasp_modes, right_grasp_modes
    )
    if args.grasp_mode_sequence.strip():
        vehicle_modes = parse_grasp_mode_sequence(args.grasp_mode_sequence, len(pairs), args.grasp_mode)
    else:
        vehicle_modes = [
            pair_vehicle_mode(left_mode, right_mode)
            for left_mode, right_mode in zip(left_grasp_modes, right_grasp_modes)
        ]
    task_layouts = (
        list(TASK_LAYOUT_Y_OFFSETS)
        if args.task_layout == "both"
        else [args.task_layout]
    )
    total_task_count = len(pairs) * len(task_layouts) * args.repeat
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_root = args.output_root / f"sequence_{stamp}"
    run_root.mkdir(parents=True, exist_ok=True)
    save_path = args.save or (run_root / "extract_sequence_full.rrd")
    helpers = None
    robot = None
    if not args.no_rerun:
        save_path.parent.mkdir(parents=True, exist_ok=True)

        global rr
        import rerun as rr

        monitor.rr = rr
        helpers = monitor.load_rerun_helpers()
        rr.init("extract_sequence_rerun")
        rr.save(str(save_path))
        urdf_text = helpers.render_current_urdf()
        robot = helpers.UrdfRobot(urdf_text)
        rr.log("monitor", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        helpers.log_robot_static_model(robot, "monitor/robot", log_meshes=True)

    if monitor.service_exists("/dual_arm_planner/run_extract_monitor_full_selected"):
        print("检测到旧 /dual_arm_planner 服务，自动清理旧 planner/move_group...")
        if not cleanup_planner_processes():
            raise RuntimeError("旧 /dual_arm_planner 服务清理超时，请检查外部 ROS 进程")

    ros_home = run_root / "ros_home"
    ros_log_dir = run_root / "ros_log"
    ros_home.mkdir(parents=True, exist_ok=True)
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_HOME"] = str(ros_home)
    os.environ["ROS_LOG_DIR"] = str(ros_log_dir)

    sample = 0
    summaries: list[dict[str, Any]] = []
    planner: subprocess.Popen[str] | None = None
    service_client: monitor.ExtractMonitorServiceClient | None = None
    current_mode: str | None = None
    startup_ms = 0.0

    def stop_current_planner() -> None:
        nonlocal planner, service_client, current_mode
        if service_client is not None:
            service_client.close()
            service_client = None
        if planner is not None:
            monitor.terminate_process(planner)
            planner = None
        if not wait_until_service_gone():
            cleanup_planner_processes()
        current_mode = None

    def start_planner_for_mode(
        group_key: str,
        layout_name: str,
        mode: str,
        initial_args: argparse.Namespace,
        left_id: int,
        right_id: int,
        group_index: int,
    ) -> tuple[subprocess.Popen[str], monitor.ExtractMonitorServiceClient, float]:
        nonlocal current_mode
        group_label = f"{layout_name}_{mode}_{initial_args.extract_rollout_mode}"
        launch_log = run_root / f"planner_{group_index:02d}_{group_label}.log"
        initial_snapshot = run_root / f"initial_stage_snapshot_{group_index:02d}_{group_label}.json"
        launch_command = monitor.build_launch_command(initial_args, run_root, initial_snapshot)
        domain_export = f"export ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}\n" if "ROS_DOMAIN_ID" in os.environ else ""
        (run_root / f"launch_command_{group_index:02d}_{group_label}.sh").write_text(
            "#!/usr/bin/env bash\nset -e\n"
            f"{domain_export}"
            "source /opt/ros/humble/setup.bash\n"
            f"source {monitor.ROS_SETUP}\n"
            f"cd {monitor.ROS_WS}\n{launch_command}\n"
        )
        print(f"启动 planner[{group_label}]，日志：{launch_log}")
        start = time.monotonic()
        with launch_log.open("w") as log_file:
            new_planner = subprocess.Popen(
                monitor.bash_source_command(launch_command),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid,
            )
        try:
            monitor.wait_for_service(
                "/dual_arm_planner/configure_extract_monitor",
                new_planner,
                args.service_timeout,
                launch_log,
            )
            new_client = monitor.ExtractMonitorServiceClient(
                configure_service="/dual_arm_planner/configure_extract_monitor",
                trigger_service=(
                    "/dual_arm_planner/run_extract_monitor_next"
                    if args.extract_only or args.ik_only_raw or args.ik_scene_rejected
                    else "/dual_arm_planner/run_extract_monitor_full_selected"
                ),
                timeout=args.service_timeout,
            )
            prewarm_ok, prewarm_output, prewarm_ms = new_client.configure(
                left_id,
                right_id,
                initial_snapshot,
                args.service_timeout,
                initial_args.left_grasp_mode == "top_suction",
                initial_args.right_grasp_mode == "top_suction",
                explicit_grasp_target(
                    initial_args,
                    left_id,
                    initial_args.left_grasp_mode,
                    initial_args.left_target_position_jitter,
                ),
                explicit_grasp_target(
                    initial_args,
                    right_id,
                    initial_args.right_grasp_mode,
                    initial_args.right_target_position_jitter,
                ),
            )
            print(prewarm_output)
            if not prewarm_ok:
                new_client.close()
                raise RuntimeError(f"planner[{mode}] IK 预热失败：{prewarm_output}")
            elapsed = (time.monotonic() - start) * 1000.0
            print(f"planner[{group_label}] 启动完成：startup={elapsed:.1f}ms prewarm={prewarm_ms:.1f}ms")
            current_mode = group_key
            return new_planner, new_client, elapsed
        except Exception:
            monitor.terminate_process(new_planner)
            wait_until_service_gone(10.0)
            raise

    try:
        group_index = 0
        task_global_index = 0
        stop_sequence = False
        for repeat_index in range(1, args.repeat + 1):
            print(f"\n######## 重复轮次 {repeat_index}/{args.repeat} ########")
            for layout_name in task_layouts:
                layout_scene_y_shift = (
                    float(args.scene_y_shift) + TASK_LAYOUT_Y_OFFSETS[layout_name]
                )
                print(
                    f"\n---- 横向布局 {layout_name}: "
                    f"scene_y_shift={layout_scene_y_shift:+.2f}m ----"
                )
                for pair_index, ((left_id, right_id), left_mode, right_mode, mode) in enumerate(
                    zip(pairs, left_grasp_modes, right_grasp_modes, vehicle_modes),
                    start=1,
                ):
                    task_global_index += 1
                    pair_args = make_pair_args(
                        args,
                        left_id,
                        right_id,
                        mode,
                        left_mode,
                        right_mode,
                        task_layout=layout_name,
                        scene_y_shift=layout_scene_y_shift,
                    )
                    pair_args.left_target_position_jitter = sample_l1_position_jitter(
                        jitter_rng,
                        args.target_position_jitter_max,
                    )
                    pair_args.right_target_position_jitter = sample_l1_position_jitter(
                        jitter_rng,
                        args.target_position_jitter_max,
                    )
                    print(
                        "末端位置误差："
                        f"left={pair_args.left_target_position_jitter} "
                        f"L1={sum(abs(value) for value in pair_args.left_target_position_jitter):.4f}m; "
                        f"right={pair_args.right_target_position_jitter} "
                        f"L1={sum(abs(value) for value in pair_args.right_target_position_jitter):.4f}m"
                    )
                    group_key = f"{layout_name}:{mode}:{pair_args.extract_rollout_mode}"
                    group_label = f"{layout_name}_{mode}_{pair_args.extract_rollout_mode}"
                    if current_mode != group_key:
                        stop_current_planner()
                        group_index += 1
                        last_error: Exception | None = None
                        for attempt in range(args.startup_retries + 1):
                            try:
                                if attempt > 0:
                                    print(
                                        f"planner[{group_label}] 启动重试 "
                                        f"{attempt}/{args.startup_retries}"
                                    )
                                planner, service_client, startup_ms = start_planner_for_mode(
                                    group_key,
                                    layout_name,
                                    mode,
                                    pair_args,
                                    left_id,
                                    right_id,
                                    group_index,
                                )
                                last_error = None
                                break
                            except Exception as exc:
                                last_error = exc
                                cleanup_planner_processes()
                        if last_error is not None:
                            raise last_error
                    assert planner is not None
                    assert service_client is not None
                    if args.planner_server:
                        print(
                            f"PLANNER_SERVER_READY startup_ms={startup_ms:.1f} "
                            f"group={group_label}",
                            flush=True,
                        )
                        while planner.poll() is None:
                            time.sleep(0.5)
                        return int(planner.returncode or 0)
                    ok, sample_count, summary = run_one_pair(
                        pair_args,
                        helpers,
                        robot,
                        run_root,
                        planner,
                        run_root / f"planner_{group_index:02d}_{group_label}.log",
                        service_client,
                        startup_ms,
                        left_id,
                        right_id,
                        task_global_index,
                        total_task_count,
                        sample,
                    )
                    startup_ms = 0.0
                    summary["repeat"] = repeat_index
                    summary["layout"] = layout_name
                    summary["scene_y_shift"] = layout_scene_y_shift
                    summary["pair_index"] = pair_index
                    summary["grasp_mode"] = mode
                    summary["left_grasp_mode"] = left_mode
                    summary["right_grasp_mode"] = right_mode
                    summaries.append(summary)
                    sample += max(1, sample_count) + 5
                    if not ok and not args.continue_on_failure:
                        stop_sequence = True
                        break
                if stop_sequence:
                    break
            if stop_sequence:
                break
    finally:
        stop_current_planner()

    summary_path = run_root / "summary.json"
    summary_path.write_text(json.dumps(summaries, ensure_ascii=False, indent=2))
    stats_path = args.stats_csv or (run_root / "stats.csv")
    if summaries:
        fieldnames = [
            "repeat",
            "layout",
            "scene_y_shift",
            "pair_index",
            "left",
            "right",
            "success",
            "loaded_planning_mode",
            "loaded_planner_id",
            "startup_ms",
            "configure_ms",
            "service_ms",
            "wall_ms",
            "primary_success",
            "primary_service_ms",
            "fallback_attempted",
            "fallback_success",
            "fallback_configure_ms",
            "fallback_service_ms",
            "total_ms",
            "ik_ms",
            "extract_ms",
            "loaded_ms",
            "final_ms",
            "loaded_to_place_ms",
            "place_to_loaded_ms",
            "place_to_initial_ms",
            "loaded_plan_batch_wall_ms",
            "loaded_plan_candidate_count",
            "loaded_plan_attempted_count",
            "loaded_plan_success_count",
            "loaded_parallel_workers",
            "motion_total_joint_rad",
            "motion_total_joint_deg",
            "motion_total_axis_mixed",
            "motion_pre_attach_joint_rad",
            "motion_extract_joint_rad",
            "motion_lateral_joint_rad",
            "motion_loaded_joint_rad",
            "selected_loaded_plan_trajectory_distance",
            "selected_loaded_plan_rank",
            "selected_h",
            "selected_h_index",
            "selected_seed_index",
            "left_target_dx_m",
            "left_target_dy_m",
            "left_target_dz_m",
            "left_target_l1_m",
            "right_target_dx_m",
            "right_target_dy_m",
            "right_target_dz_m",
            "right_target_l1_m",
            "failure_reason",
            "primary_failure_reason",
            "fallback_failure_reason",
            "primary_failure_snapshot",
            "snapshot",
        ]
        stats_path.parent.mkdir(parents=True, exist_ok=True)
        with stats_path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            for item in summaries:
                writer.writerow(item)
    print("\n===== 序列完成 =====")
    for item in summaries:
        status = "成功" if item.get("success") else "失败"
        print(
            f"[{item.get('layout', 'centered')}] "
            f"L{item['left']}/R{item['right']}: {status} "
            f"startup={item.get('startup_ms', 0.0):.1f}ms "
            f"total={item.get('total_ms', 0.0):.1f}ms "
            f"loaded_batch={item.get('loaded_plan_batch_wall_ms', 0.0):.1f}ms "
            f"place={item.get('loaded_to_place_ms', 0.0):.1f}ms "
            f"return_initial={item.get('place_to_initial_ms', item.get('place_to_loaded_ms', 0.0)):.1f}ms "
            f"samples={item.get('samples', 0)}"
        )
    if not args.no_rerun:
        print(f"Rerun: {save_path}")
    print(f"Summary: {summary_path}")
    print(f"Stats CSV: {stats_path}")
    return 0 if all(item.get("success") for item in summaries) else 1


if __name__ == "__main__":
    raise SystemExit(main())
