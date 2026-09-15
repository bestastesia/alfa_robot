#!/usr/bin/env python3
"""Visualize independently tested V3 reachability and continuity centers."""

from __future__ import annotations

import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import re
import struct

import numpy as np

import rerun as rr
import rerun.blueprint as rrb

from alfa_robot_rerun.visualize_rerun import (
    UrdfRobot,
    log_robot_state,
    log_robot_static_model,
    package_uri_to_path,
    render_current_urdf,
)


GREEN = [45, 210, 95, 230]
RED = [235, 65, 70, 190]
BLUE = [65, 150, 245, 230]
ORANGE = [255, 150, 45, 220]
MAGENTA = [225, 70, 225, 220]
YELLOW = [245, 220, 55, 220]
GRAY = [145, 150, 160, 180]
CYAN = [60, 220, 235, 230]
DARK_RED = [150, 20, 35, 245]

SHOULDER_CENTERS = {
    "V3.0.6": {
        "left": np.array([-0.055, 0.4025, 0.25]),
        "right": np.array([-0.055, -0.4125, 0.25]),
    },
    "V3.0.7": {
        "left": np.array([0.18099999, -0.4115, 1.3228]),
        "right": np.array([0.18099999, 0.4115, 1.3228]),
    },
    "V3.0.8": {
        "left": np.array([0.181, -0.47659019, 1.34243275]),
        "right": np.array([0.181, 0.47659019, 1.34243279]),
    },
    "V3.0.9": {
        "left": np.array([0.181, -0.47659019, 1.34243275]),
        "right": np.array([0.181, 0.47659019, 1.34243279]),
    },
    "V3.1.1": {
        "left": np.array([0.180999997236, 0.468061833255, 1.37493884175]),
        "right": np.array([0.181, -0.476590193255, 1.37493884175]),
    },
}
UPPER_ARM_LENGTH = 0.506
FOREARM_LENGTH = 0.473
TOOL_LENGTH = 0.13585


JUMP_SEVERITIES = [
    (12.0, "10-12deg", YELLOW),
    (15.0, "12-15deg", ORANGE),
    (20.0, "15-20deg", RED),
    (30.0, "20-30deg", MAGENTA),
    (math.inf, "30deg-plus", DARK_RED),
]


def build_disk_path_offsets(config: dict) -> list[list[float]]:
    radius_limit = float(config["disk_radius"])
    ring_step = float(config["disk_ring_step"])
    path_step = float(config["path_step"])
    offsets = [[0.0, 0.0, 0.0]]
    previous_radius = 0.0
    while previous_radius < radius_limit - 1.0e-12:
        radius = min(radius_limit, previous_radius + ring_step)
        radial_segments = max(1, math.ceil((radius - previous_radius) / path_step))
        for segment in range(1, radial_segments + 1):
            ratio = segment / radial_segments
            radial_position = previous_radius + ratio * (radius - previous_radius)
            offsets.append([0.0, radial_position, 0.0])
        ring_segments = max(8, math.ceil(2.0 * math.pi * radius / path_step))
        for segment in range(1, ring_segments + 1):
            angle = 2.0 * math.pi * segment / ring_segments
            offsets.append([0.0, radius * math.cos(angle), radius * math.sin(angle)])
        previous_radius = radius
    return offsets


def failure_class(point: dict) -> tuple[str, list[int]]:
    reason = str(point["failure_reason"])
    if reason == "analytic_no_solution":
        if int(point["failure_step_index"]) == 0:
            return "中心立即无解析解", RED
        return "圆盘路径进入不可达区", ORANGE
    if reason == "all_candidates_exceed_joint_delta":
        return "相邻关节跳变超过10度", MAGENTA
    if "deviate_from_task_path" in reason:
        return "关节插值偏离目标路径", MAGENTA
    if "colliding" in reason:
        return "碰撞", YELLOW
    return "其他连续性失败", GRAY


def inside_bounds(point: dict, args: argparse.Namespace) -> bool:
    x, y, z = [float(value) for value in point["position"]]
    return (
        args.x_min <= x <= args.x_max
        and args.y_min <= y <= args.y_max
        and args.z_min <= z <= args.z_max
    )


def set_failure_time(index: int) -> None:
    if hasattr(rr, "set_time_sequence"):
        rr.set_time_sequence("failure_case", index)
    else:
        rr.set_time("failure_case", sequence=index)


def set_point_stage(seconds: float) -> None:
    rr.set_time("point_stage", duration=seconds)


def jump_severity(point: dict) -> tuple[str, list[int]]:
    delta = float(point.get("diagnostic_joint_delta_deg", 0.0))
    for upper_bound, label, color in JUMP_SEVERITIES:
        if delta < upper_bound:
            return label, color
    raise AssertionError("unreachable jump severity")


def extract_jump_component(
    failed_points: list[dict], config: dict, seed_position: list[float]
) -> list[dict]:
    jump_points = [
        point
        for point in failed_points
        if point["failure_reason"] == "all_candidates_exceed_joint_delta"
    ]
    if not jump_points:
        raise RuntimeError("输入数据中没有关节跳变失败点")

    steps = [
        float(config["x_start_step"]),
        float(config["lateral_step"]),
        float(config["height_step"]),
    ]
    origins = [
        float(config["x_start_min"]),
        float(config["lateral_min"]),
        float(config["height_min"]),
    ]

    def grid_key(position: list[float]) -> tuple[int, int, int]:
        return tuple(
            int(round((float(position[axis]) - origins[axis]) / steps[axis]))
            for axis in range(3)
        )

    points_by_key = {grid_key(point["position"]): point for point in jump_points}
    seed_key = grid_key(seed_position)
    if seed_key not in points_by_key:
        nearest = min(
            jump_points,
            key=lambda point: sum(
                (float(point["position"][axis]) - seed_position[axis]) ** 2
                for axis in range(3)
            ),
        )
        raise RuntimeError(
            "指定种子不是关节跳变点；最近跳变点为 "
            f"{nearest['position']}"
        )

    neighbor_offsets = [
        (1, 0, 0),
        (-1, 0, 0),
        (0, 1, 0),
        (0, -1, 0),
        (0, 0, 1),
        (0, 0, -1),
    ]
    component_keys = {seed_key}
    pending = deque([seed_key])
    while pending:
        current = pending.popleft()
        for offset in neighbor_offsets:
            neighbor = tuple(current[axis] + offset[axis] for axis in range(3))
            if neighbor in component_keys or neighbor not in points_by_key:
                continue
            component_keys.add(neighbor)
            pending.append(neighbor)
    return [points_by_key[key] for key in component_keys]


def coordinate_bounds(points: list[dict], field: str) -> str:
    positions = [point[field] for point in points]
    labels = []
    for axis, name in enumerate("XYZ"):
        values = [float(position[axis]) for position in positions]
        labels.append(f"{name}={min(values):.3f}~{max(values):.3f}m")
    return ", ".join(labels)


def find_jump_case(failed_points: list[dict], center: list[float]) -> dict:
    jump_points = [
        point
        for point in failed_points
        if point["failure_reason"] == "all_candidates_exceed_joint_delta"
    ]
    if not jump_points:
        raise RuntimeError("输入数据中没有关节跳变失败点")
    nearest = min(
        jump_points,
        key=lambda point: sum(
            (float(point["position"][axis]) - center[axis]) ** 2
            for axis in range(3)
        ),
    )
    distance = math.sqrt(
        sum(
            (float(nearest["position"][axis]) - center[axis]) ** 2
            for axis in range(3)
        )
    )
    if distance > 1.0e-6:
        raise RuntimeError(
            f"指定中心不是关节跳变点；最近跳变点为 {nearest['position']}"
        )
    return nearest


def grid_key(position: list[float], config: dict) -> tuple[int, int, int]:
    origins = [
        float(config["x_start_min"]),
        float(config["lateral_min"]),
        float(config["height_min"]),
    ]
    steps = [
        float(config["x_start_step"]),
        float(config["lateral_step"]),
        float(config["height_step"]),
    ]
    return tuple(
        int(round((float(position[axis]) - origins[axis]) / steps[axis]))
        for axis in range(3)
    )


def find_sandwiched_collision_failures(data: dict) -> list[dict]:
    config = data["config"]
    successful_keys = {
        grid_key(point["position"], config)
        for point in data["successful_start_points"]
    }
    axis_names = ["X", "Y", "Z"]
    selected: list[dict] = []
    for point in data["failed_start_points"]:
        if "colliding" not in str(point["failure_reason"]):
            continue
        if not point.get("path_joint_samples") or not point.get("diagnostic_joints"):
            continue
        key = grid_key(point["position"], config)
        axes = []
        for axis, axis_name in enumerate(axis_names):
            lower = list(key)
            upper = list(key)
            lower[axis] -= 1
            upper[axis] += 1
            if tuple(lower) in successful_keys and tuple(upper) in successful_keys:
                axes.append(axis_name)
        if axes:
            selected_point = dict(point)
            selected_point["sandwiched_axes"] = axes
            selected.append(selected_point)
    return sorted(
        selected,
        key=lambda point: tuple(float(value) for value in point["position"]),
    )


def find_exact_failed_point(failed_points: list[dict], center: list[float]) -> dict:
    if not failed_points:
        raise RuntimeError("筛选范围内没有失败点")
    nearest = min(
        failed_points,
        key=lambda point: sum(
            (float(point["position"][axis]) - center[axis]) ** 2
            for axis in range(3)
        ),
    )
    distance = math.sqrt(
        sum(
            (float(nearest["position"][axis]) - center[axis]) ** 2
            for axis in range(3)
        )
    )
    if distance > 1.0e-6:
        raise RuntimeError(
            f"指定中心不是失败点；最近失败点为 {nearest['position']}"
        )
    return nearest


def find_failure_path_case(failed_points: list[dict], center: list[float]) -> dict:
    nearest = find_exact_failed_point(failed_points, center)
    if not nearest.get("path_joint_samples") or not nearest.get("diagnostic_joints"):
        raise RuntimeError(
            "指定失败点没有完整轨迹诊断数据；请用 "
            "record_failure_paths:=true record_failure_states:=true 重新扫描"
        )
    selected = dict(nearest)
    selected["selection_description"] = "user-selected failure center"
    return selected


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cosine_roll, sine_roll = math.cos(roll), math.sin(roll)
    cosine_pitch, sine_pitch = math.cos(pitch), math.sin(pitch)
    cosine_yaw, sine_yaw = math.cos(yaw), math.sin(yaw)
    rotation_x = np.array(
        [[1.0, 0.0, 0.0], [0.0, cosine_roll, -sine_roll], [0.0, sine_roll, cosine_roll]]
    )
    rotation_y = np.array(
        [[cosine_pitch, 0.0, sine_pitch], [0.0, 1.0, 0.0], [-sine_pitch, 0.0, cosine_pitch]]
    )
    rotation_z = np.array(
        [[cosine_yaw, -sine_yaw, 0.0], [sine_yaw, cosine_yaw, 0.0], [0.0, 0.0, 1.0]]
    )
    return rotation_z @ rotation_y @ rotation_x


def reach_sphere_wireframe(center: np.ndarray, radius: float) -> list[list[list[float]]]:
    angles = np.linspace(0.0, 2.0 * math.pi, 129)
    circles = []
    for first_axis, second_axis in ((0, 1), (0, 2), (1, 2)):
        points = np.repeat(center[None, :], len(angles), axis=0)
        points[:, first_axis] += radius * np.cos(angles)
        points[:, second_axis] += radius * np.sin(angles)
        circles.append(points.tolist())
    return circles


def log_reach_limit_playback(
    robot: UrdfRobot,
    point: dict,
    region_points: list[dict],
    config: dict,
    frames_per_second: float,
) -> None:
    if point["failure_reason"] != "analytic_no_solution" or int(
        point["failure_step_index"]
    ) != 0:
        raise RuntimeError("指定中心不是起点解析不可达案例")
    side = str(config["side"])
    analytic_model = str(config.get("analytic_model", "V3.0.6"))
    shoulder = SHOULDER_CENTERS[analytic_model][side].copy()
    shoulder[0] += float(config.get("arm_mount_forward_offset", 0.0))
    target = np.asarray(point["position"], dtype=float)
    roll, pitch, yaw = [float(value) for value in config["target_orientation_rpy"]]
    target_rotation = rotation_from_rpy(roll, pitch, yaw)
    wrist = target - target_rotation @ np.array([0.0, 0.0, TOOL_LENGTH])
    shoulder_to_wrist = wrist - shoulder
    desired_distance = float(np.linalg.norm(shoulder_to_wrist))
    direction = shoulder_to_wrist / desired_distance
    maximum_distance = UPPER_ARM_LENGTH + FOREARM_LENGTH
    maximum_wrist = shoulder + maximum_distance * direction
    overreach = max(0.0, desired_distance - maximum_distance)
    frame_count = max(2, int(round(3.0 * frames_per_second)))

    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(
                    origin="/",
                    contents=["/robot/**", "/diagnostics/**"],
                    name="V3.0.6 reach-limit diagnosis",
                ),
                rrb.TextDocumentView(
                    origin="/diagnostics/current_details", name="Diagnosis"
                ),
                column_shares=[0.76, 0.24],
            ),
            collapse_panels=True,
        )
    )
    log_robot_state(robot, {}, "robot")
    rr.log(
        "diagnostics/red_region",
        rr.Points3D(
            [candidate["position"] for candidate in region_points],
            colors=[RED] * len(region_points),
            radii=0.006,
            labels=["起点无解析解"] * len(region_points),
        ),
        static=True,
    )
    rr.log(
        "diagnostics/reach_boundary",
        rr.LineStrips3D(
            reach_sphere_wireframe(shoulder, maximum_distance),
            colors=[[245, 210, 55, 150]] * 3,
            radii=0.002,
        ),
        static=True,
    )
    rr.log(
        "diagnostics/key_points",
        rr.Points3D(
            [shoulder.tolist(), maximum_wrist.tolist(), wrist.tolist(), target.tolist()],
            colors=[BLUE, YELLOW, RED, MAGENTA],
            radii=[0.022, 0.018, 0.020, 0.020],
            labels=["肩部中心", "最大可达腕点", "目标腕点", "目标工具点"],
        ),
        static=True,
    )
    rr.log(
        "diagnostics/reach_segments",
        rr.LineStrips3D(
            [
                [shoulder.tolist(), maximum_wrist.tolist()],
                [maximum_wrist.tolist(), wrist.tolist()],
                [wrist.tolist(), target.tolist()],
            ],
            colors=[GREEN, RED, MAGENTA],
            radii=[0.004, 0.006, 0.003],
        ),
        static=True,
    )

    for frame_index in range(frame_count):
        ratio = frame_index / (frame_count - 1)
        current_distance = ratio * desired_distance
        probe = shoulder + current_distance * direction
        reachable = current_distance <= maximum_distance + 1.0e-12
        rr.set_time("reach_limit_playback", duration=frame_index / frames_per_second)
        rr.log(
            "diagnostics/probe",
            rr.Points3D(
                [probe.tolist()],
                colors=[GREEN if reachable else RED],
                radii=0.018,
                labels=["腕点仍在臂长范围内" if reachable else "腕点已越过最大臂长"],
            ),
        )
        rr.log(
            "diagnostics/current_details",
            rr.TextDocument(
                "\n".join(
                    [
                        "# 前伸红区：几何不可达",
                        f"- 扫描中心 `[x,y,z]`: `{point['position']}`",
                        f"- 肩到目标腕点: `{desired_distance:.4f} m`",
                        f"- 最大肩腕距离: `{maximum_distance:.4f} m`",
                        f"- 超出臂长: `{1000.0 * overreach:.1f} mm`",
                        f"- 当前探针距离: `{current_distance:.4f} m`",
                        "- 黄色线框：理论最大肩腕球面。",
                        "- 红色线段：目标腕点超出最大臂长的部分。",
                        "- 该点在第 0 步即无解，因此没有碰撞姿态可播放。",
                    ]
                ),
                media_type=rr.MediaType.MARKDOWN,
            ),
        )
    rr.set_time("reach_limit_playback", duration=0.0)


def collision_links_from_reason(reason: str) -> list[str]:
    if "collision:" not in reason:
        return []
    pairs = reason.rsplit("collision:", 1)[1]
    links = []
    for pair in pairs.split(","):
        links.extend(part.strip() for part in pair.split("<->") if part.strip())
    return list(dict.fromkeys(links))


def stl_bounds(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = path.read_bytes()
    vertices: list[tuple[float, float, float]] = []
    if len(data) >= 84:
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        expected_size = 84 + triangle_count * 50
        if expected_size == len(data):
            for triangle_index in range(triangle_count):
                values = struct.unpack_from("<12fH", data, 84 + triangle_index * 50)
                vertices.extend(
                    (values[index], values[index + 1], values[index + 2])
                    for index in (3, 6, 9)
                )
    if not vertices:
        text = data.decode("utf-8", errors="ignore")
        vertices = [
            tuple(float(value) for value in match.groups())
            for match in re.finditer(
                r"\bvertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)",
                text,
            )
        ]
    if not vertices:
        raise RuntimeError(f"无法读取 STL 顶点：{path}")
    points = np.asarray(vertices, dtype=float)
    return points.min(axis=0), points.max(axis=0)


def link_local_bounds(robot: UrdfRobot, link_name: str) -> tuple[list[float], list[float]]:
    if link_name == "base_link":
        return [0.0, 0.0, 0.25], [0.4, 0.5, 0.5]
    link = robot.links.get(link_name)
    if link is None or not link.visuals:
        return [0.0, 0.0, 0.0], [0.08, 0.08, 0.08]
    all_corners = []
    for visual in link.visuals:
        mesh_path = package_uri_to_path(visual.path)
        if not mesh_path or not Path(mesh_path).exists():
            continue
        lower, upper = stl_bounds(Path(mesh_path))
        lower = lower * visual.scale
        upper = upper * visual.scale
        corners = np.asarray(
            [
                [x, y, z, 1.0]
                for x in (lower[0], upper[0])
                for y in (lower[1], upper[1])
                for z in (lower[2], upper[2])
            ]
        )
        all_corners.append((visual.origin @ corners.T).T[:, :3])
    if not all_corners:
        return [0.0, 0.0, 0.0], [0.08, 0.08, 0.08]
    corners = np.concatenate(all_corners, axis=0)
    lower = corners.min(axis=0)
    upper = corners.max(axis=0)
    return ((lower + upper) * 0.5).tolist(), (upper - lower).tolist()


def log_collision_link_boxes(
    robot: UrdfRobot,
    joint_positions: dict[str, float],
    link_names: list[str],
    bounds_cache: dict[str, tuple[list[float], list[float]]],
) -> None:
    transforms = robot.fk(joint_positions)
    for link_name in link_names:
        transform = transforms.get(link_name)
        if transform is None:
            continue
        center, size = bounds_cache.setdefault(
            link_name, link_local_bounds(robot, link_name)
        )
        rr.log(
            f"diagnostics/collision_links/{link_name}",
            rr.Transform3D(
                translation=transform[:3, 3].tolist(),
                mat3x3=transform[:3, :3].tolist(),
            ),
        )
        rr.log(
            f"diagnostics/collision_links/{link_name}/bounds",
            rr.Boxes3D(
                centers=[center],
                sizes=[size],
                colors=[[255, 25, 25, 175]],
                fill_mode="solid",
                labels=[link_name],
                show_labels=True,
            ),
        )


def log_sandwiched_collision_playback(
    robot: UrdfRobot,
    points: list[dict],
    side: str,
    frames_per_second: float,
    collision_hold_s: float,
) -> None:
    if not points:
        raise RuntimeError("没有找到前后相邻中心均成功、当前中心因碰撞失败的案例")
    joint_names = [f"{side}_joint{index}" for index in range(1, 8)]
    bounds_cache: dict[str, tuple[list[float], list[float]]] = {}
    frame_index = 0
    hold_frames = max(1, int(round(collision_hold_s * frames_per_second)))
    gap_frames = max(1, int(round(0.2 * frames_per_second)))
    focus = np.asarray(points[0]["position"], dtype=float)
    camera_side = 1.0 if side == "left" else -1.0

    rr.log(
        "diagnostics/all_case_centers",
        rr.Points3D(
            [point["position"] for point in points],
            colors=[[255, 145, 40, 130]] * len(points),
            radii=0.004,
            labels=[f"case {index + 1}" for index in range(len(points))],
        ),
        static=True,
    )
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(
                    origin="/",
                    contents=["/robot/**", "/diagnostics/**"],
                    name="Single-arm failure trajectory",
                    eye_controls=rrb.EyeControls3D(
                        position=(focus + np.array([1.8, 1.8 * camera_side, 1.0])).tolist(),
                        look_target=(focus + np.array([-0.15, 0.0, 0.0])).tolist(),
                        eye_up=[0.0, 0.0, 1.0],
                    ),
                ),
                rrb.TextDocumentView(
                    origin="/diagnostics/current_details", name="Current case"
                ),
                column_shares=[0.76, 0.24],
            ),
            collapse_panels=True,
        )
    )

    for case_index, point in enumerate(points):
        joint_samples = point["path_joint_samples"]
        target_samples = point["path_target_samples"]
        failure_target = point["failure_target_position"]
        collision_links = collision_links_from_reason(str(point["failure_reason"]))
        full_target_path = target_samples + [failure_target]
        for sample_index, (joints, target) in enumerate(
            zip(joint_samples, target_samples)
        ):
            rr.set_time("collision_playback", duration=frame_index / frames_per_second)
            rr.log("diagnostics/collision_links", rr.Clear(recursive=True))
            joint_positions = {
                name: float(value) for name, value in zip(joint_names, joints)
            }
            log_robot_state(robot, joint_positions, "robot")
            rr.log(
                "diagnostics/current_target",
                rr.Points3D(
                    [target], colors=[CYAN], radii=0.012, labels=["current target"]
                ),
            )
            if sample_index == 0:
                rr.log(
                    "diagnostics/current_path",
                    rr.LineStrips3D(
                        [full_target_path], colors=[[70, 190, 245, 185]], radii=0.002
                    ),
                )
                rr.log(
                    "diagnostics/current_center",
                    rr.Points3D(
                        [point["position"]],
                        colors=[BLUE],
                        radii=0.016,
                        labels=["tested center"],
                    ),
                )
                rr.log(
                    "diagnostics/current_details",
                    rr.TextDocument(
                        "\n".join(
                            [
                                f"# Case {case_index + 1}/{len(points)}",
                                f"- Center: {point['position']}",
                                f"- Selection: {point.get('selection_description', '前后相邻中心成功')}",
                                *(
                                    [f"- Successful neighbors along: {', '.join(point['sandwiched_axes'])}"]
                                    if point.get("sandwiched_axes")
                                    else []
                                ),
                                f"- Failure step: {point['failure_step_index']}/{len(full_target_path) - 1}",
                                f"- Collision links: {', '.join(collision_links)}",
                                "- Cyan: commanded Cartesian path.",
                                "- Final red boxes: links reported by MoveIt/FCL collision checking.",
                            ]
                        ),
                        media_type=rr.MediaType.MARKDOWN,
                    ),
                )
            frame_index += 1

        diagnostic_joints = point["diagnostic_joints"]
        diagnostic_positions = {
            name: float(value)
            for name, value in zip(joint_names, diagnostic_joints)
        }
        for hold_index in (0, hold_frames):
            rr.set_time(
                "collision_playback",
                duration=(frame_index + hold_index) / frames_per_second,
            )
            log_robot_state(robot, diagnostic_positions, "robot")
            rr.log("diagnostics/collision_links", rr.Clear(recursive=True))
            log_collision_link_boxes(
                robot, diagnostic_positions, collision_links, bounds_cache
            )
            rr.log(
                "diagnostics/current_target",
                rr.Points3D(
                    [failure_target],
                    colors=[RED],
                    radii=0.016,
                    labels=["collision target"],
                ),
            )
            rr.log(
                "diagnostics/current_details",
                rr.TextDocument(
                    "\n".join(
                        [
                            f"# Case {case_index + 1}/{len(points)} - collision",
                            f"- Center: {point['position']}",
                            f"- Reached: {point['reached_point_count']} samples",
                            f"- Failure step: {point['failure_step_index']}",
                            f"- Collision links: {', '.join(collision_links)}",
                            f"- Raw reason: {point['failure_reason']}",
                        ]
                    ),
                    media_type=rr.MediaType.MARKDOWN,
                ),
            )
        frame_index += hold_frames + gap_frames
    rr.set_time("collision_playback", duration=0.0)


def log_unreachable_path_playback(
    robot: UrdfRobot,
    point: dict,
    side: str,
    frames_per_second: float,
    failure_hold_s: float,
) -> None:
    joint_samples = point.get("path_joint_samples", [])
    target_samples = point.get("path_target_samples", [])
    if not joint_samples or len(joint_samples) != len(target_samples):
        raise RuntimeError("指定失败点没有可播放的合法关节轨迹")
    joint_names = [f"{side}_joint{index}" for index in range(1, 8)]
    failure_target = [float(value) for value in point["failure_target_position"]]
    complete_path = [*target_samples, failure_target]
    hold_frames = max(1, int(round(failure_hold_s * frames_per_second)))
    focus = np.asarray(target_samples[len(target_samples) // 2], dtype=float)
    camera_side = 1.0 if side == "left" else -1.0

    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(
                    origin="/",
                    contents=["/robot/**", "/diagnostics/**"],
                    name="Actual IK path to reach limit",
                    eye_controls=rrb.EyeControls3D(
                        position=(focus + np.array([1.8, 1.8 * camera_side, 1.0])).tolist(),
                        look_target=(focus + np.array([-0.15, 0.0, 0.0])).tolist(),
                        eye_up=[0.0, 0.0, 1.0],
                    ),
                ),
                rrb.TextDocumentView(
                    origin="/diagnostics/current_details", name="Current state"
                ),
                column_shares=[0.78, 0.22],
            ),
            collapse_panels=True,
        )
    )
    rr.log(
        "diagnostics/commanded_path",
        rr.LineStrips3D([complete_path], colors=[[60, 220, 235, 190]], radii=0.004),
        static=True,
    )
    rr.log(
        "diagnostics/key_targets",
        rr.Points3D(
            [target_samples[0], target_samples[-1], failure_target],
            colors=[BLUE, GREEN, RED],
            radii=[0.018, 0.018, 0.022],
            labels=["reachable start", "last valid IK target", "first unreachable target"],
        ),
        static=True,
    )

    for frame_index, (joints, target) in enumerate(zip(joint_samples, target_samples)):
        rr.set_time("ik_reach_playback", duration=frame_index / frames_per_second)
        joint_positions = {
            name: float(value) for name, value in zip(joint_names, joints)
        }
        log_robot_state(robot, joint_positions, "robot")
        rr.log(
            "diagnostics/current_target",
            rr.Points3D(
                [target], colors=[GREEN], radii=0.016, labels=["current valid target"]
            ),
        )
        rr.log(
            "diagnostics/current_details",
            rr.TextDocument(
                "\n".join(
                    [
                        "# Actual analytic-IK trajectory",
                        f"- Valid sample: {frame_index + 1}/{len(joint_samples)}",
                        f"- Current tool target: {target}",
                        f"- First unreachable target: {failure_target}",
                        "- The robot pose is the real collision-checked IK result.",
                    ]
                ),
                media_type=rr.MediaType.MARKDOWN,
            ),
        )

    last_joint_positions = {
        name: float(value) for name, value in zip(joint_names, joint_samples[-1])
    }
    first_hold_frame = len(joint_samples)
    for hold_offset in (0, hold_frames):
        rr.set_time(
            "ik_reach_playback",
            duration=(first_hold_frame + hold_offset) / frames_per_second,
        )
        log_robot_state(robot, last_joint_positions, "robot")
        rr.log(
            "diagnostics/current_target",
            rr.Points3D(
                [failure_target],
                colors=[RED],
                radii=0.024,
                labels=["analytic IK unavailable"],
            ),
        )
        rr.log(
            "diagnostics/current_details",
            rr.TextDocument(
                "\n".join(
                    [
                        "# Analytic reach boundary reached",
                        f"- Last valid target: {target_samples[-1]}",
                        f"- First unreachable target: {failure_target}",
                        f"- Failure reason: {point['failure_reason']}",
                        "- The robot is held at the final real IK pose.",
                        "- Red target has no joint solution, so no fake robot pose is shown.",
                    ]
                ),
                media_type=rr.MediaType.MARKDOWN,
            ),
        )
    rr.set_time("ik_reach_playback", duration=0.0)


def log_single_jump_case(robot: UrdfRobot, point: dict, side: str) -> None:
    joint_names = [f"{side}_joint{index}" for index in range(1, 8)]
    center = [float(value) for value in point["position"]]
    last_valid = [float(value) for value in point["last_valid_position"]]
    failure_target = [float(value) for value in point["failure_target_position"]]
    previous_joints = [float(value) for value in point["last_valid_joints"]]
    candidate_joints = [float(value) for value in point["diagnostic_joints"]]
    joint_deltas = [
        math.remainder(candidate - previous, 2.0 * math.pi)
        for previous, candidate in zip(previous_joints, candidate_joints)
    ]
    joint_deltas_deg = [math.degrees(delta) for delta in joint_deltas]
    maximum_joint = max(range(7), key=lambda index: abs(joint_deltas[index]))
    color = jump_severity(point)[1]

    rr.log(
        "jump_demo/reference/center",
        rr.Points3D([center], colors=[BLUE], radii=0.017, labels=["Test center"]),
        static=True,
    )
    rr.log(
        "jump_demo/reference/last_valid_target",
        rr.Points3D(
            [last_valid], colors=[GREEN], radii=0.020, labels=["Last valid target"]
        ),
        static=True,
    )
    rr.log(
        "jump_demo/reference/failure_target",
        rr.Points3D(
            [failure_target], colors=[color], radii=0.023, labels=["Next target"]
        ),
        static=True,
    )
    rr.log(
        "jump_demo/reference/cartesian_step",
        rr.Arrows3D(
            origins=[last_valid],
            vectors=[
                [failure_target[axis] - last_valid[axis] for axis in range(3)]
            ],
            colors=[CYAN],
            radii=0.005,
        ),
        static=True,
    )
    cartesian_distance_mm = 1000.0 * math.sqrt(
        sum(
            (failure_target[axis] - last_valid[axis]) ** 2 for axis in range(3)
        )
    )
    rr.log(
        "jump_demo/summary",
        rr.TextDocument(
            "\n".join(
                [
                    "# Single-case joint jump",
                    f"- Test center: {center}",
                    f"- Last valid target: {last_valid}",
                    f"- Next target: {failure_target}",
                    f"- Cartesian target step: {cartesian_distance_mm:.2f}mm",
                    f"- Minimum required joint jump: {point['diagnostic_joint_delta_deg']:.3f}deg",
                    f"- Largest joint change: J{maximum_joint + 1} "
                    f"({joint_deltas_deg[maximum_joint]:+.3f}deg)",
                    "- The solver rejected this as one adjacent-frame jump.",
                    "- The 2-second interpolation is slow motion for inspection, not a valid path.",
                ]
            ),
            media_type=rr.MediaType.MARKDOWN,
        ),
        static=True,
    )
    rr.log(
        "jump_demo/joint_delta_deg",
        rr.BarChart(joint_deltas_deg, abscissa=list(range(1, 8))),
        static=True,
    )

    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(
                    origin="/",
                    contents=[
                        "/robot/base_link/**",
                        "/robot/left_arm_base/**",
                        "/robot/left_joint1/**",
                        "/robot/left_joint2/**",
                        "/robot/left_joint3/**",
                        "/robot/left_joint4/**",
                        "/robot/left_joint5/**",
                        "/robot/left_joint6/**",
                        "/robot/left_joint7/**",
                        "/robot/left_tool0/**",
                        "/jump_demo/reference/**",
                    ],
                    name="Robot slow motion",
                ),
                rrb.Vertical(
                    rrb.TextDocumentView(
                        origin="/jump_demo/summary", name="Case summary"
                    ),
                    rrb.BarChartView(
                        origin="/jump_demo/joint_delta_deg", name="Joint delta (deg)"
                    ),
                    rrb.TimeSeriesView(
                        origin="/jump_demo/joint_change",
                        contents=[
                            "/jump_demo/joint_change/J5",
                            "/jump_demo/joint_change/J7",
                        ],
                        name="J5/J7 slow-motion change",
                    ),
                    row_shares=[0.34, 0.33, 0.33],
                ),
                column_shares=[0.68, 0.32],
            ),
            collapse_panels=True,
        )
    )

    frames_per_second = 30
    hold_frames = frames_per_second
    transition_frames = 2 * frames_per_second
    final_hold_frames = frames_per_second
    total_frames = hold_frames + transition_frames + final_hold_frames
    for frame in range(total_frames + 1):
        rr.set_time("jump_slow_motion", duration=frame / frames_per_second)
        if frame <= hold_frames:
            ratio = 0.0
            phase = "Last valid pose (hold)"
        elif frame <= hold_frames + transition_frames:
            raw_ratio = (frame - hold_frames) / transition_frames
            ratio = raw_ratio * raw_ratio * (3.0 - 2.0 * raw_ratio)
            phase = "Slow-motion expansion of one adjacent-frame jump"
        else:
            ratio = 1.0
            phase = "Closest candidate above 10deg (hold)"
        joints = [
            previous + ratio * delta
            for previous, delta in zip(previous_joints, joint_deltas)
        ]
        log_robot_state(
            robot,
            {name: value for name, value in zip(joint_names, joints)},
            "robot",
        )
        rr.log(
            "jump_demo/current_phase",
            rr.TextDocument(
                f"# {phase}\n- Progress: {100.0 * ratio:.1f}%",
                media_type=rr.MediaType.MARKDOWN,
            ),
        )
        for joint_index in [4, 6]:
            delta_deg = joint_deltas_deg[joint_index]
            rr.log(
                f"jump_demo/joint_change/J{joint_index + 1}",
                rr.Scalars([ratio * delta_deg]),
            )
    rr.set_time("jump_slow_motion", duration=0.0)


def log_jump_component(
    robot: UrdfRobot,
    points: list[dict],
    side: str,
    seed_position: list[float],
    include_poses: bool,
) -> None:
    severity_groups: dict[str, tuple[list[int], list[dict]]] = {}
    for point in points:
        label, color = jump_severity(point)
        severity_groups.setdefault(label, (color, []))[1].append(point)

    for label, (color, group) in severity_groups.items():
        point_labels = [
            f"center={point['position']} target={point['failure_target_position']} "
            f"step={point['failure_step_index']} min_jump="
            f"{point['diagnostic_joint_delta_deg']:.3f}deg"
            for point in group
        ]
        rr.log(
            f"jump_component/test_centers/{label}",
            rr.Points3D(
                [point["position"] for point in group],
                colors=[[120, 125, 135, 55]] * len(group),
                radii=0.003,
                labels=point_labels,
            ),
            static=True,
        )
        rr.log(
            f"jump_component/actual_failure_targets/{label}",
            rr.Points3D(
                [point["failure_target_position"] for point in group],
                colors=[color] * len(group),
                radii=0.009,
                labels=point_labels,
            ),
            static=True,
        )

    rr.log(
        "jump_component/last_valid_targets",
        rr.Points3D(
            [point["last_valid_position"] for point in points],
            colors=[GREEN] * len(points),
            radii=0.004,
        ),
        static=True,
    )
    rr.log(
        "jump_component/last_step_edges",
        rr.Arrows3D(
            origins=[point["last_valid_position"] for point in points],
            vectors=[
                [
                    float(point["failure_target_position"][axis])
                    - float(point["last_valid_position"][axis])
                    for axis in range(3)
                ]
                for point in points
            ],
            colors=[jump_severity(point)[1] for point in points],
            radii=0.0015,
        ),
        static=True,
    )
    rr.log(
        "jump_component/selected_seed",
        rr.Points3D(
            [seed_position], colors=[CYAN], radii=0.018, labels=["Selected seed"]
        ),
        static=True,
    )

    severity_counts = Counter(jump_severity(point)[0] for point in points)
    rr.log(
        "jump_component/summary",
        rr.TextDocument(
            "\n".join(
                [
                    "# J5/J7 branch-jump region",
                    f"- Connected grid points: {len(points)}",
                    f"- Test-center bounds: {coordinate_bounds(points, 'position')}",
                    "- Actual failure-target bounds: "
                    f"{coordinate_bounds(points, 'failure_target_position')}",
                    "- Point color is the minimum required maximum joint change.",
                    *[
                        f"- {label}: {severity_counts.get(label, 0)} points"
                        for _, label, _ in JUMP_SEVERITIES
                    ],
                    "- Green: last valid target.",
                    "- Colored: actual target that triggered the jump threshold.",
                ]
            ),
            media_type=rr.MediaType.MARKDOWN,
        ),
        static=True,
    )

    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(
                    origin="/jump_component",
                    contents=[
                        "/jump_component/actual_failure_targets/**",
                        "/jump_component/selected_seed",
                        "/jump_component/last_valid_targets",
                    ],
                    name="Actual jump targets",
                ),
                rrb.TextDocumentView(
                    origin="/jump_component/summary", name="Region summary"
                ),
                column_shares=[0.76, 0.24],
            ),
            collapse_panels=True,
        )
    )

    if not include_poses:
        return

    joint_names = [f"{side}_joint{index}" for index in range(1, 8)]
    ordered = sorted(
        points,
        key=lambda point: (
            -float(point["diagnostic_joint_delta_deg"]),
            *[float(value) for value in point["position"]],
        ),
    )
    for case_index, point in enumerate(ordered):
        center = [float(value) for value in point["position"]]
        last_valid = [float(value) for value in point["last_valid_position"]]
        failure_target = [float(value) for value in point["failure_target_position"]]
        previous_joints = [float(value) for value in point["last_valid_joints"]]
        candidate_joints = [float(value) for value in point["diagnostic_joints"]]
        joint_deltas = [
            math.degrees(math.remainder(candidate - previous, 2.0 * math.pi))
            for previous, candidate in zip(previous_joints, candidate_joints)
        ]
        maximum_joint = max(range(7), key=lambda index: abs(joint_deltas[index]))
        color = jump_severity(point)[1]
        detail_lines = [
            f"# 跳变案例 {case_index + 1}/{len(ordered)}",
            f"- 测试中心：{center}",
            f"- 上一合法目标：{last_valid}",
            f"- 实际失败目标：{failure_target}",
            f"- 失败步骤：{point['failure_step_index']}",
            f"- 最小必需跳变：{point['diagnostic_joint_delta_deg']:.3f}deg",
            f"- 最大变化关节：J{maximum_joint + 1} "
            f"({joint_deltas[maximum_joint]:+.3f}deg)",
            "- 各关节变化："
            + ", ".join(
                f"J{index + 1}={delta:+.2f}deg"
                for index, delta in enumerate(joint_deltas)
            ),
        ]
        for phase, joints, target, phase_label in [
            (0, previous_joints, last_valid, "上一合法姿态"),
            (1, candidate_joints, failure_target, "最接近但超过10deg的候选姿态"),
        ]:
            set_failure_time(case_index * 2 + phase)
            log_robot_state(
                robot,
                {name: value for name, value in zip(joint_names, joints)},
                "robot",
            )
            rr.log(
                "diagnostics/current/center",
                rr.Points3D([center], colors=[BLUE], radii=0.014, labels=["测试中心"]),
            )
            rr.log(
                "diagnostics/current/last_valid_target",
                rr.Points3D(
                    [last_valid], colors=[GREEN], radii=0.016, labels=["上一合法目标"]
                ),
            )
            rr.log(
                "diagnostics/current/failure_target",
                rr.Points3D(
                    [failure_target], colors=[color], radii=0.019, labels=["实际失败目标"]
                ),
            )
            rr.log(
                "diagnostics/current/failed_edge",
                rr.Arrows3D(
                    origins=[last_valid],
                    vectors=[
                        [
                            failure_target[axis] - last_valid[axis]
                            for axis in range(3)
                        ]
                    ],
                    colors=[color],
                    radii=0.004,
                ),
            )
            rr.log(
                "diagnostics/current/details",
                rr.TextDocument(
                    "\n".join(detail_lines + [f"- 当前帧：{phase_label}"]),
                    media_type=rr.MediaType.MARKDOWN,
                ),
            )
    if ordered:
        set_failure_time(0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_json")
    parser.add_argument("--save", default="")
    parser.add_argument("--spawn", action="store_true")
    parser.add_argument("--no-meshes", action="store_true")
    parser.add_argument("--failure-details", action="store_true")
    parser.add_argument("--failure-poses", action="store_true")
    parser.add_argument(
        "--sandwiched-collision-playback",
        action="store_true",
        help="串行播放前后相邻中心成功、当前中心碰撞失败的完整轨迹",
    )
    parser.add_argument(
        "--failure-path-center",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="播放指定失败中心的完整轨迹，并在最后一帧标红碰撞连杆",
    )
    parser.add_argument(
        "--reach-limit-center",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="播放指定起点无解析解中心的肩腕最大臂长诊断",
    )
    parser.add_argument(
        "--unreachable-path-center",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="播放沿真实解析IK轨迹到达首个无解点的全过程",
    )
    parser.add_argument("--playback-fps", type=float, default=20.0)
    parser.add_argument("--collision-hold-s", type=float, default=1.0)
    parser.add_argument(
        "--max-playback-cases",
        type=int,
        default=0,
        help="限制回放案例数量；0 表示全部",
    )
    parser.add_argument(
        "--jump-component-seed",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="只显示包含指定关节跳变点的六邻域连通分量",
    )
    parser.add_argument(
        "--jump-case-center",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="只播放指定跳变中心的一组慢动作姿态对比",
    )
    parser.add_argument("--x-min", type=float, default=-math.inf)
    parser.add_argument("--x-max", type=float, default=math.inf)
    parser.add_argument("--y-min", type=float, default=-math.inf)
    parser.add_argument("--y-max", type=float, default=math.inf)
    parser.add_argument("--z-min", type=float, default=-math.inf)
    parser.add_argument("--z-max", type=float, default=math.inf)
    parser.add_argument(
        "--highlight-forward-farthest",
        action="store_true",
        help="高亮 base_link +X 最远的可达点；同X时选择最接近测试臂Joint1横向高度的点",
    )
    args = parser.parse_args()

    data = json.loads(Path(args.input_json).read_text())
    config = data["config"]
    summary = data["summary"]
    successful_points = [
        point for point in data["successful_start_points"] if inside_bounds(point, args)
    ]
    failed_points = [
        point for point in data["failed_start_points"] if inside_bounds(point, args)
    ]
    jump_component_points: list[dict] = []
    jump_case: dict | None = None
    exclusive_modes = [
        bool(args.jump_component_seed),
        bool(args.jump_case_center),
        bool(args.failure_path_center),
        bool(args.reach_limit_center),
        bool(args.unreachable_path_center),
        bool(args.sandwiched_collision_playback),
    ]
    if sum(exclusive_modes) > 1:
        raise RuntimeError("跳变、指定失败轨迹和夹心碰撞回放模式不能同时使用")
    if args.jump_component_seed:
        jump_component_points = extract_jump_component(
            failed_points, config, args.jump_component_seed
        )
        successful_points = []
        failed_points = jump_component_points
    if args.jump_case_center:
        jump_case = find_jump_case(failed_points, args.jump_case_center)
        successful_points = []
        failed_points = [jump_case]
    travel = float(config["travel"])
    evaluation_mode = str(config.get("evaluation_mode", ""))
    disk_mode = evaluation_mode == "planar_disk_continuity"
    reachability_mode = evaluation_mode == "point_reachability"
    if not reachability_mode:
        reachability_mode = abs(travel) < 1.0e-9

    rr.init("v3_continuous_reachability", spawn=args.spawn)
    if args.save:
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        rr.save(args.save)

    robot = UrdfRobot(
        render_current_urdf(
            {
                "arm_mount_forward_offset": str(
                    config.get("arm_mount_forward_offset", 0.0)
                )
            }
        )
    )
    diagnostic_playback = any(
        (
            args.sandwiched_collision_playback,
            args.failure_path_center,
            args.reach_limit_center,
            args.unreachable_path_center,
            args.jump_case_center,
        )
    )
    side = str(config["side"])
    opposite_prefix = "right_" if side == "left" else "left_"
    log_robot_static_model(
        robot,
        "robot",
        log_meshes=not args.no_meshes,
        exclude_link_prefixes=(opposite_prefix,) if diagnostic_playback else (),
    )
    highlighted_forward_point: dict | None = None
    highlighted_forward_shoulder: np.ndarray | None = None
    staged_point_clouds = not args.failure_details and not args.failure_poses
    if args.highlight_forward_farthest:
        if not successful_points:
            raise RuntimeError("筛选范围内没有可高亮的可达点")
        side = str(config["side"])
        highlighted_forward_shoulder = robot.fk({})[f"{side}_joint1"][:3, 3]
        highlighted_forward_point = max(
            successful_points,
            key=lambda point: (
                float(point["position"][0]),
                -math.hypot(
                    float(point["position"][1]) - highlighted_forward_shoulder[1],
                    float(point["position"][2]) - highlighted_forward_shoulder[2],
                ),
            ),
        )

    if args.sandwiched_collision_playback:
        if args.playback_fps <= 0.0 or args.collision_hold_s < 0.0:
            raise RuntimeError("回放帧率必须为正，碰撞停留时间不能为负")
        playback_points = find_sandwiched_collision_failures(data)
        if args.max_playback_cases > 0:
            playback_points = playback_points[: args.max_playback_cases]
        log_sandwiched_collision_playback(
            robot,
            playback_points,
            str(config["side"]),
            args.playback_fps,
            args.collision_hold_s,
        )
        print(
            "Rerun sandwiched collision playback complete: "
            f"cases={len(playback_points)} save={args.save or '-'}"
        )
        return 0

    if args.failure_path_center:
        if args.playback_fps <= 0.0 or args.collision_hold_s < 0.0:
            raise RuntimeError("回放帧率必须为正，碰撞停留时间不能为负")
        playback_point = find_failure_path_case(
            failed_points, args.failure_path_center
        )
        log_sandwiched_collision_playback(
            robot,
            [playback_point],
            str(config["side"]),
            args.playback_fps,
            args.collision_hold_s,
        )
        print(
            "Rerun selected failure path playback complete: "
            f"center={playback_point['position']} save={args.save or '-'}"
        )
        return 0

    if args.reach_limit_center:
        if args.playback_fps <= 0.0:
            raise RuntimeError("回放帧率必须为正")
        reach_point = find_exact_failed_point(
            failed_points, args.reach_limit_center
        )
        log_reach_limit_playback(
            robot,
            reach_point,
            failed_points,
            config,
            args.playback_fps,
        )
        print(
            "Rerun reach-limit playback complete: "
            f"center={reach_point['position']} save={args.save or '-'}"
        )
        return 0

    if args.unreachable_path_center:
        if args.playback_fps <= 0.0 or args.collision_hold_s < 0.0:
            raise RuntimeError("回放帧率必须为正，失败停留时间不能为负")
        unreachable_point = find_exact_failed_point(
            failed_points, args.unreachable_path_center
        )
        log_unreachable_path_playback(
            robot,
            unreachable_point,
            str(config["side"]),
            args.playback_fps,
            args.collision_hold_s,
        )
        print(
            "Rerun unreachable path playback complete: "
            f"center={unreachable_point['position']} save={args.save or '-'}"
        )
        return 0

    if jump_component_points:
        log_jump_component(
            robot,
            jump_component_points,
            str(config["side"]),
            args.jump_component_seed,
            args.failure_poses,
        )
        print(
            f"Rerun jump component complete: points={len(jump_component_points)} "
            f"save={args.save or '-'}"
        )
        return 0
    if jump_case:
        log_single_jump_case(robot, jump_case, str(config["side"]))
        print(
            "Rerun single jump case complete: "
            f"center={jump_case['position']} "
            f"delta={jump_case['diagnostic_joint_delta_deg']:.3f}deg "
            f"save={args.save or '-'}"
        )
        return 0

    if successful_points and not args.failure_poses:
        if staged_point_clouds:
            set_point_stage(0.0)
            rr.log(
                "scan/stage_status",
                rr.TextDocument(
                    "# Stage 1/2\nSuccessful points only",
                    media_type=rr.MediaType.MARKDOWN,
                ),
            )
        side = str(config["side"])
        joint_names = [f"{side}_joint{index}" for index in range(1, 8)]
        displayed_robot_point = highlighted_forward_point or successful_points[0]
        first_joints = displayed_robot_point["start_joints"]
        log_robot_state(
            robot,
            {name: float(value) for name, value in zip(joint_names, first_joints)},
            "robot",
        )
        cloud_successful_points = [
            point for point in successful_points if point is not highlighted_forward_point
        ]
        comfortable_points = [
            point
            for point in cloud_successful_points
            if int(point.get("high_joint_gain_edge_count", 0)) == 0
        ]
        high_gain_points = [
            point
            for point in cloud_successful_points
            if int(point.get("high_joint_gain_edge_count", 0)) > 0
        ]
        base_entity = (
            "scan/reachable_points"
            if reachability_mode
            else "scan/disk_continuous_points"
            if disk_mode
            else "scan/continuous_points"
        )
        point_groups = (
            (("reachable", cloud_successful_points, GREEN),)
            if highlighted_forward_point is not None
            else (
                ("comfortable", comfortable_points, GREEN),
                ("high_joint_gain", high_gain_points, CYAN),
            )
        )
        for suffix, points, color in point_groups:
            if not points:
                continue
            rr.log(
                f"{base_entity}/{suffix}",
                rr.Points3D(
                    [point["position"] for point in points],
                    colors=[color] * len(points),
                    radii=0.008,
                    labels=[
                        f"start={point['point_index']} "
                        f"max_delta={point['maximum_path_joint_delta_deg']:.2f}deg "
                        f"high_gain_edges={point.get('high_joint_gain_edge_count', 0)}"
                        for point in points
                    ],
                ),
                static=not staged_point_clouds,
            )

        if highlighted_forward_point is not None and highlighted_forward_shoulder is not None:
            highlighted_position = np.asarray(
                highlighted_forward_point["position"], dtype=float
            )
            shoulder_distance = float(
                np.linalg.norm(highlighted_position - highlighted_forward_shoulder)
            )
            rr.log(
                "scan/forward_farthest/point",
                rr.Points3D(
                    [highlighted_position],
                    colors=[BLUE],
                    radii=0.022,
                    labels=[
                        f"+X最远可达点 {highlighted_position.tolist()} "
                        f"距Joint1={shoulder_distance:.3f}m"
                    ],
                ),
                static=True,
            )
            rr.log(
                "scan/forward_farthest/reach_vector",
                rr.Arrows3D(
                    origins=[highlighted_forward_shoulder],
                    vectors=[highlighted_position - highlighted_forward_shoulder],
                    colors=[BLUE],
                    radii=0.005,
                ),
                static=True,
            )

    if failed_points and not args.failure_details and not jump_component_points and not jump_case:
        if staged_point_clouds:
            set_point_stage(2.0)
            rr.log(
                "scan/reachable_points",
                rr.Clear(recursive=True),
            )
            rr.log(
                "scan/continuous_points",
                rr.Clear(recursive=True),
            )
            rr.log(
                "scan/disk_continuous_points",
                rr.Clear(recursive=True),
            )
            rr.log(
                "scan/stage_status",
                rr.TextDocument(
                    "# Stage 2/2\nFailed points only",
                    media_type=rr.MediaType.MARKDOWN,
                ),
            )
        rr.log(
            (
                "scan/unreachable_points"
                if reachability_mode
                else "scan/disk_discontinuous_points"
                if disk_mode
                else "scan/discontinuous_points"
            ),
            rr.Points3D(
                [point["position"] for point in failed_points],
                colors=[RED] * len(failed_points),
                radii=0.005,
            ),
            static=not staged_point_clouds,
        )

    if failed_points and args.failure_details and not jump_component_points and not jump_case:
        failure_groups: dict[str, tuple[list[int], list[dict]]] = {}
        for point in failed_points:
            label, color = failure_class(point)
            failure_groups.setdefault(label, (color, []))[1].append(point)
        for label, (color, points) in failure_groups.items():
            rr.log(
                f"scan/failure_details/{label}",
                rr.Points3D(
                    [point["position"] for point in points],
                    colors=[color] * len(points),
                    radii=0.007,
                    labels=[
                        f"center={point['point_index']} step={point['failure_step_index']} "
                        f"reached={point['reached_point_count']}"
                        for point in points
                    ],
                ),
                static=True,
            )

        if disk_mode:
            offsets = build_disk_path_offsets(config)
            late_failures = [
                point
                for point in failed_points
                if 0 < int(point["failure_step_index"]) < len(offsets)
            ]
            if late_failures:
                origins = [point["position"] for point in late_failures]
                targets = []
                colors = []
                labels = []
                for point in late_failures:
                    offset = offsets[int(point["failure_step_index"])]
                    target = [
                        float(point["position"][axis]) + offset[axis]
                        for axis in range(3)
                    ]
                    targets.append(target)
                    colors.append(failure_class(point)[1])
                    labels.append(
                        f"失败采样点 step={point['failure_step_index']} "
                        f"reason={failure_class(point)[0]}"
                    )
                rr.log(
                    "scan/failure_details/center_to_failure_target",
                    rr.Arrows3D(
                        origins=origins,
                        vectors=[
                            [target[axis] - origin[axis] for axis in range(3)]
                            for origin, target in zip(origins, targets)
                        ],
                        colors=colors,
                        radii=0.002,
                    ),
                    static=True,
                )
                rr.log(
                    "scan/failure_details/failure_targets",
                    rr.Points3D(
                        targets,
                        colors=colors,
                        radii=0.009,
                        labels=labels,
                    ),
                    static=True,
                )

    if failed_points and args.failure_poses and not jump_component_points and not jump_case:
        side = str(config["side"])
        joint_names = [f"{side}_joint{index}" for index in range(1, 8)]
        pose_points = [
            point
            for point in failed_points
            if point.get("diagnostic_joints") or point.get("last_valid_joints")
        ]
        for failure_index, point in enumerate(pose_points):
            set_failure_time(failure_index)
            joint_values = point.get("diagnostic_joints") or point["last_valid_joints"]
            log_robot_state(
                robot,
                {
                    name: float(value)
                    for name, value in zip(joint_names, joint_values)
                },
                "robot",
            )
            center = [float(value) for value in point["position"]]
            failure_target = [
                float(value)
                for value in point.get("failure_target_position", center)
            ]
            rr.log(
                "diagnostics/current/center",
                rr.Points3D([center], colors=[BLUE], radii=0.015, labels=["测试中心"]),
            )
            rr.log(
                "diagnostics/current/failure_target",
                rr.Points3D(
                    [failure_target],
                    colors=[failure_class(point)[1]],
                    radii=0.018,
                    labels=["失败目标点"],
                ),
            )
            if point.get("last_valid_position"):
                last_valid = [float(value) for value in point["last_valid_position"]]
                rr.log(
                    "diagnostics/current/last_valid_target",
                    rr.Points3D(
                        [last_valid], colors=[GREEN], radii=0.014, labels=["上一合法点"]
                    ),
                )
                rr.log(
                    "diagnostics/current/failed_edge",
                    rr.Arrows3D(
                        origins=[last_valid],
                        vectors=[
                            [failure_target[axis] - last_valid[axis] for axis in range(3)]
                        ],
                        colors=[failure_class(point)[1]],
                        radii=0.004,
                    ),
                )
            rr.log(
                "diagnostics/current/details",
                rr.TextDocument(
                    "\n".join(
                        [
                            f"# 失败案例 {failure_index + 1}/{len(pose_points)}",
                            f"- 中心编号：{point['point_index']}",
                            f"- 中心坐标：{center}",
                            f"- 失败步骤：{point['failure_step_index']}",
                            f"- 已连续通过：{point['reached_point_count']} 点",
                            f"- 原因：{point['failure_reason']}",
                            f"- 显示姿态：{point.get('diagnostic_pose_kind', 'last_valid_state')}",
                            f"- 候选关节跳变：{point.get('diagnostic_joint_delta_deg', 0.0):.3f}°",
                        ]
                    ),
                    media_type=rr.MediaType.MARKDOWN,
                ),
            )
        if pose_points:
            set_failure_time(0)

    if not reachability_mode and not disk_mode:
        rr.log(
            "scan/front_axis",
            rr.Arrows3D(
                origins=[[float(config["x_start_min"]), 0.0, 0.0]],
                vectors=[[travel, 0.0, 0.0]],
                colors=[BLUE],
                radii=0.006,
                        labels=[f"每个绿色起点独立沿 world +X 连续验证 {travel:.2f}m"],
            ),
            static=True,
        )

    if disk_mode:
        radius = float(config["disk_radius"])
        title = "V3 single-arm local disk continuity"
        success_label = f"Complete radius-{radius:.2f}m disks"
        color_label = (
            f"- Green: comfortable full radius-{radius:.2f}m disk; cyan: continuous "
            "with high joint gain; red: at least one disk step failed."
        )
    elif reachability_mode:
        title = "V3 single-arm point reachability"
        success_label = "Reachable points"
        color_label = "- Green: reachable; red: unreachable."
    else:
        title = "V3 single-arm continuous reachability"
        success_label = f"Complete continuous paths over {travel:.2f}m"
        color_label = (
            f"- Green: comfortable continuous {travel:.2f}m path; cyan: continuous "
            "with high joint gain; red: incomplete path."
        )
    rr.log(
        "summary",
        rr.TextDocument(
            "\n".join(
                [
                    f"# {title}",
                    f"- Arm: {config['side']}",
                    f"- Independent starts: {summary['start_point_count']}",
                    f"- Displayed: {len(successful_points)} complete / {len(failed_points)} failed",
                    f"- {success_label}: {summary['complete_count']}",
                    f"- Complete ratio: {100.0 * summary['complete_ratio']:.2f}%",
                    f"- High joint-gain marker: {config['maximum_joint_delta_deg']:.1f} deg",
                    *(
                        [
                            f"- Disk plane: {config['disk_plane']}",
                            f"- Disk radius: {config['disk_radius']:.2f}m",
                            f"- Ring step: {config['disk_ring_step']:.2f}m; "
                            f"path sample: about {config['path_step']:.2f}m",
                        ]
                        if disk_mode
                        else []
                    ),
                    f"- Analytic IK: {summary['analytic_calls']} calls / "
                    f"{summary['analytic_ms']:.2f} ms",
                    f"- Collision checks: {summary['collision_checks']} calls / "
                    f"{summary['collision_ms']:.2f} ms",
                    color_label,
                    "- Playback: 0s shows successful points; 2s shows failed points.",
                    *(
                        [
                            "- 失败诊断颜色：深红=中心立即无解，橙=圆盘外圈进入不可达区，",
                            "  紫=相邻关节跳变超过10°，黄=碰撞。箭头终点是实际失败采样点。",
                        ]
                        if args.failure_details
                        else []
                    ),
                    "- Every point is an independent tested center; path samples are hidden.",
                ]
            ),
            media_type=rr.MediaType.MARKDOWN,
        ),
        static=True,
    )
    if staged_point_clouds:
        set_point_stage(0.0)
        rr.log(
            "scan/stage_status",
            rr.TextDocument(
                "# Stage 1/2\nSuccessful points only",
                media_type=rr.MediaType.MARKDOWN,
            ),
        )
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(
                    origin="/",
                    contents=["/robot/**", "/scan/**"],
                    name="Robot and continuity points",
                ),
                rrb.TextDocumentView(origin="/summary", name="Scan summary"),
                column_shares=[0.78, 0.22],
            ),
            collapse_panels=True,
        )
    )
    print(
        f"Rerun complete: starts={summary['start_point_count']} "
        f"continuous={summary['complete_count']} save={args.save or '-'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
