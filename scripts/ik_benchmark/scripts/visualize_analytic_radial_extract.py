#!/usr/bin/env python3
"""Visualize the deterministic analytic radial-extraction prototype."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
MONITOR_DIR = SCRIPT_DIR.parents[2] / "ros2_ws" / "src" / "alfa_robot_moveit_config" / "scripts"
if str(MONITOR_DIR) not in sys.path:
    sys.path.insert(0, str(MONITOR_DIR))

import extract_stage_monitor_console as monitor  # noqa: E402
import rerun as rr  # noqa: E402
from alfa_robot_rerun import visualize_rerun as helpers  # noqa: E402


JOINT_NAMES = [
    "pitch",
    "turn",
    "updown",
    *[f"left_joint{index}" for index in range(1, 7)],
    *[f"right_joint{index}" for index in range(1, 7)],
]


def joint_map(frame: dict[str, Any], updown: float) -> dict[str, float] | None:
    left = frame.get("left_joints")
    right = frame.get("right_joints")
    if not isinstance(left, list) or not isinstance(right, list) or len(left) != 6 or len(right) != 6:
        return None
    values = [0.0, 0.0, float(frame.get("compensated_updown", updown)), *left, *right]
    return dict(zip(JOINT_NAMES, (float(value) for value in values)))


def quaternion_matrix(quaternion_xyzw: list[float]) -> np.ndarray:
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def obb_overlaps(
    center_a: np.ndarray,
    rotation_a: np.ndarray,
    half_a: np.ndarray,
    center_b: np.ndarray,
    rotation_b: np.ndarray,
    half_b: np.ndarray,
) -> bool:
    rotation = rotation_a.T @ rotation_b
    translation = rotation_a.T @ (center_b - center_a)
    absolute = np.abs(rotation) + 1e-9
    for axis in range(3):
        radius_a = half_a[axis]
        radius_b = float(np.dot(half_b, absolute[axis, :]))
        if abs(translation[axis]) > radius_a + radius_b:
            return False
    for axis in range(3):
        radius_a = float(np.dot(half_a, absolute[:, axis]))
        radius_b = half_b[axis]
        if abs(float(np.dot(translation, rotation[:, axis]))) > radius_a + radius_b:
            return False
    for axis_a in range(3):
        for axis_b in range(3):
            radius_a = (
                half_a[(axis_a + 1) % 3] * absolute[(axis_a + 2) % 3, axis_b]
                + half_a[(axis_a + 2) % 3] * absolute[(axis_a + 1) % 3, axis_b]
            )
            radius_b = (
                half_b[(axis_b + 1) % 3] * absolute[axis_a, (axis_b + 2) % 3]
                + half_b[(axis_b + 2) % 3] * absolute[axis_a, (axis_b + 1) % 3]
            )
            projection = abs(
                translation[(axis_a + 2) % 3] * rotation[(axis_a + 1) % 3, axis_b]
                - translation[(axis_a + 1) % 3] * rotation[(axis_a + 2) % 3, axis_b]
            )
            if projection > radius_a + radius_b:
                return False
    return True


def planned_attached_boxes(
    data: dict[str, Any], frame: dict[str, Any], z_offset: float = 0.0
) -> tuple[list[dict[str, Any]], list[str]]:
    boxes: list[dict[str, Any]] = []
    for side in ("left", "right"):
        specification = next(
            (
                box
                for box in data.get("attached_boxes", [])
                if str(box.get("link_name", "")).startswith(side)
            ),
            None,
        )
        if specification is None:
            continue
        tool_center = np.asarray(frame[f"{side}_target_position_world"], dtype=float) + np.array([0.0, 0.0, z_offset])
        quaternion = [float(value) for value in frame[f"{side}_target_orientation_xyzw"]]
        rotation = quaternion_matrix(quaternion)
        local_center = np.asarray(specification["center_in_link"], dtype=float)
        boxes.append(
            {
                "id": str(specification["id"]),
                "center": tool_center + rotation @ local_center,
                "rotation": rotation,
                "quaternion": quaternion,
                "half_size": np.asarray(specification["size"], dtype=float) * 0.5,
            }
        )

    obstacles: list[dict[str, Any]] = []
    for item in data.get("container_panels", []):
        yaw = float(item.get("yaw", 0.0))
        obstacles.append(
            {
                "id": str(item["id"]),
                "center": np.asarray(item["center"], dtype=float),
                "rotation": np.array(
                    [[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]],
                    dtype=float,
                ),
                "half_size": np.asarray(item["size"], dtype=float) * 0.5,
            }
        )
    for item in data.get("static_box_obstacles", {}).get("boxes", []):
        obstacles.append(
            {
                "id": str(item["id"]),
                "center": np.asarray(item["center"], dtype=float),
                "rotation": np.eye(3),
                "half_size": np.asarray(item["size"], dtype=float) * 0.5,
            }
        )

    contacts: list[str] = []
    for box in boxes:
        for obstacle in obstacles:
            if obb_overlaps(
                box["center"], box["rotation"], box["half_size"],
                obstacle["center"], obstacle["rotation"], obstacle["half_size"],
            ):
                contacts.append(f"{box['id']} <-> {obstacle['id']}")
    if len(boxes) == 2 and obb_overlaps(
        boxes[0]["center"], boxes[0]["rotation"], boxes[0]["half_size"],
        boxes[1]["center"], boxes[1]["rotation"], boxes[1]["half_size"],
    ):
        contacts.append(f"{boxes[0]['id']} <-> {boxes[1]['id']}")
    return boxes, contacts


def log_box_set(path: str, boxes: list[dict[str, Any]], contacts: list[str], color: list[int], label_prefix: str) -> None:
    if not boxes:
        return
    display_color = [255, 40, 40, color[3]] if contacts else color
    rr.log(
        path,
        rr.Boxes3D(
            centers=[box["center"].tolist() for box in boxes],
            half_sizes=[box["half_size"].tolist() for box in boxes],
            quaternions=[box["quaternion"] for box in boxes],
            colors=[display_color] * len(boxes),
            labels=[f"{label_prefix}:{box['id']}" for box in boxes],
        ),
    )


def collision_object_ids(contacts: list[str]) -> set[str]:
    return {
        name.strip()
        for contact in contacts
        for name in str(contact).split("<->")
        if name.strip()
    }


def log_collision_scene(data: dict[str, Any], highlighted_ids: set[str]) -> None:
    items: list[tuple[dict[str, Any], str]] = []
    items.extend((item, "container") for item in data.get("container_panels", []))
    items.extend(
        (item, "box_wall")
        for item in data.get("static_box_obstacles", {}).get("boxes", [])
    )
    centers: list[list[float]] = []
    half_sizes: list[list[float]] = []
    quaternions: list[list[float]] = []
    colors: list[list[int]] = []
    labels: list[str] = []
    for item, category in items:
        object_id = str(item["id"])
        yaw = float(item.get("yaw", 0.0))
        centers.append([float(value) for value in item["center"]])
        half_sizes.append([float(value) * 0.5 for value in item["size"]])
        quaternions.append([0.0, 0.0, float(np.sin(0.5 * yaw)), float(np.cos(0.5 * yaw))])
        if object_id in highlighted_ids:
            colors.append([255, 0, 0, 235])
            labels.append(f"COLLISION: {object_id}")
        elif object_id == "container_ceiling":
            colors.append([100, 170, 255, 105])
            labels.append(f"CEILING: {object_id}")
        elif category == "container":
            colors.append([80, 140, 230, 75])
            labels.append(object_id)
        elif "rear_guard" in object_id:
            colors.append([190, 90, 255, 95])
            labels.append(f"REAR GUARD: {object_id}")
        else:
            colors.append([255, 205, 40, 105])
            labels.append(f"BOX WALL: {object_id}")
    rr.log(
        "monitor/collision_scene/all_obstacles",
        rr.Boxes3D(
            centers=centers,
            half_sizes=half_sizes,
            quaternions=quaternions,
            colors=colors,
            labels=labels,
            show_labels=True,
        ),
    )
    highlighted = sorted(object_id for object_id in highlighted_ids if object_id in {str(item[0]["id"]) for item in items})
    rr.log(
        "monitor/collision_scene/highlight",
        rr.TextLog(
            "当前碰撞障碍: " + (", ".join(highlighted) if highlighted else "无")
        ),
    )


def log_planned_attached_boxes(data: dict[str, Any], frame: dict[str, Any]) -> dict[str, list[str]]:
    initial_z = min(
        float(data["frames"][0]["left_target_position_world"][2]),
        float(data["frames"][0]["right_target_position_world"][2]),
    )
    current_z = min(
        float(frame["left_target_position_world"][2]),
        float(frame["right_target_position_world"][2]),
    )
    fixed_updown = float(data["updown"])
    compensated_updown = min(0.7, max(0.0, fixed_updown + initial_z - current_z))
    z_offset = float(frame.get("updown_compensation", compensated_updown - fixed_updown))
    compensated_boxes, compensated_contacts = planned_attached_boxes(data, frame, z_offset)
    solved = bool(frame.get("solved", False))
    rr.log("monitor/diagnostic/fixed_updown_target_boxes", rr.Clear(recursive=True))
    log_box_set(
        "monitor/diagnostic/post_compensation_target_boxes",
        compensated_boxes,
        compensated_contacts,
        [70, 210, 255, 115] if solved else [255, 165, 30, 170],
        "post-comp",
    )
    return {"post_compensation": compensated_contacts}


def log_geometry(data: dict[str, Any], frame: dict[str, Any]) -> None:
    centers = [
        frame.get("left_center_world", data["left_center_world"]),
        frame.get("right_center_world", data["right_center_world"]),
    ]
    targets = [frame["left_target_position_world"], frame["right_target_position_world"]]
    collision_free = bool(frame.get("collision_free", False))
    color = [70, 220, 100, 255] if collision_free else [255, 40, 40, 255]
    rr.log(
        "monitor/geometry/joint2_work_centers",
        rr.Points3D(centers, colors=[[255, 220, 40, 255]] * 2, radii=0.025, labels=["left center", "right center"]),
    )
    rr.log(
        "monitor/geometry/radial_links",
        rr.LineStrips3D([[center, target] for center, target in zip(centers, targets)], colors=[color] * 2, radii=0.008),
    )
    rr.log(
        "monitor/geometry/target_tools",
        rr.Points3D(targets, colors=[color] * 2, radii=0.022, labels=["left target", "right target"]),
    )

    origins: list[list[float]] = []
    vectors: list[list[float]] = []
    for side in ("left", "right"):
        position = np.asarray(frame[f"{side}_target_position_world"], dtype=float)
        quaternion = np.asarray(frame[f"{side}_target_orientation_xyzw"], dtype=float)
        rotation = quaternion_matrix(quaternion.tolist())
        origins.append(position.tolist())
        vectors.append((rotation[:, 2] * 0.18).tolist())
    rr.log(
        "monitor/geometry/tool_plus_z",
        rr.Arrows3D(origins=origins, vectors=vectors, colors=[[255, 100, 40, 255]] * 2, radii=0.009),
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    parser.add_argument("--save", type=Path, required=True)
    args = parser.parse_args()

    data = json.loads(args.result.read_text())
    rr.init("analytic_radial_extract_prototype")
    rr.save(str(args.save))
    monitor.rr = rr
    robot = helpers.UrdfRobot(helpers.render_current_urdf())
    helpers.log_robot_static_model(robot, "monitor/robot", log_meshes=True, mesh_albedo_factor=[255, 255, 255, 165])

    sample = 0
    radial_frames = data.get("frames", [])
    transition = data.get("loaded_transition", {})
    orientation_transition = data.get("orientation_only_transition", {})
    resume_transition = data.get("radial_resume_transition", {})
    first_failure = int(data.get("first_failure_frame", -1))
    if transition.get("attempted") and first_failure >= 0:
        radial_frames = radial_frames[: first_failure + 1]
    if transition.get("start_phase") == "radial_checkpoint":
        selected_radial_frames = int(transition.get("start_phase_step", 0))
        if selected_radial_frames > 0:
            radial_frames = radial_frames[:selected_radial_frames]
    frames = [
        *radial_frames,
        *(
            orientation_transition.get("frames", [])[:int(
                orientation_transition.get(
                    "used_frame_count", len(orientation_transition.get("frames", []))
                )
            )]
            if orientation_transition.get("used_for_loaded_transition", False)
            else []
        ),
        *(
            resume_transition.get("frames", [])[:int(
                resume_transition.get(
                    "used_frame_count", len(resume_transition.get("frames", []))
                )
            )]
            if resume_transition.get("used_for_loaded_transition", False)
            else []
        ),
        *data.get("loaded_transition", {}).get("frames", []),
    ]
    radial_geometry_visible = True
    for frame in frames:
        helpers.set_sample_time(sample)
        positions = joint_map(frame, float(data["updown"]))
        if positions is not None:
            helpers.log_robot_state(robot, positions, "monitor/robot")
            collision_ids = {
                name.strip()
                for contact in frame.get("contacts", [])
                for name in str(contact).split("<->")
            }
            monitor.log_attached_boxes(robot, positions, data.get("attached_boxes", []), collision_ids)
        phase = frame.get("phase", "radial")
        if phase in ("radial", "orientation_only", "radial_resume"):
            log_geometry(data, frame)
            radial_geometry_visible = True
        elif radial_geometry_visible:
            rr.log("monitor/geometry", rr.Clear(recursive=True))
            radial_geometry_visible = False
        planned_contacts = log_planned_attached_boxes(data, frame)
        all_contacts = [
            *frame.get("contacts", []),
            *planned_contacts["post_compensation"],
        ]
        log_collision_scene(data, collision_object_ids(all_contacts))
        if phase == "radial":
            status = (
                f"phase=radial frame={frame['index'] + 1}/{data['frames_requested']} solved={frame.get('solved')} "
                f"collision_free={frame.get('collision_free')} radius={frame.get('left_radius', float('nan')):.4f}m "
                f"radial={frame.get('left_radial_angle_deg', float('nan')):.2f}deg "
                f"relative={frame.get('left_relative_angle_deg', float('nan')):.2f}deg "
                f"q1_margin={frame.get('left_q1_feasibility_margin_m', float('nan')) * 1000.0:.2f}mm "
                f"updown_comp={frame.get('updown_compensation', 0.0) * 1000.0:.2f}mm "
                f"joint_step={frame.get('max_joint_step_deg', float('nan')):.2f}deg "
                f"moveit_contacts={frame.get('contacts', [])} "
                f"post_comp_box_contacts={planned_contacts['post_compensation']} "
                f"failure={frame.get('solve_failure', '')}"
            )
        elif phase == "orientation_only":
            status = (
                f"phase=orientation_only frame={frame['index'] + 1}/{orientation_transition.get('steps_requested', 0)} "
                f"solved={frame.get('solved')} collision_free={frame.get('collision_free')} "
                f"frozen_radius={frame.get('left_radius', float('nan')):.4f}m "
                f"frozen_radial={frame.get('left_radial_angle_deg', float('nan')):.2f}deg "
                f"relative={frame.get('left_relative_angle_deg', float('nan')):.2f}deg "
                f"joint_step={frame.get('max_joint_step_deg', float('nan')):.2f}deg "
                f"moveit_contacts={frame.get('contacts', [])} "
                f"failure={frame.get('solve_failure', '') or orientation_transition.get('failure_reason', '')}"
            )
        elif phase == "radial_resume":
            status = (
                f"phase=radial_resume frame={frame['index'] + 1}/{resume_transition.get('steps_requested', 0)} "
                f"solved={frame.get('solved')} collision_free={frame.get('collision_free')} "
                f"radius={frame.get('left_radius', float('nan')):.4f}m "
                f"radial={frame.get('left_radial_angle_deg', float('nan')):.2f}deg "
                f"relative={frame.get('left_relative_angle_deg', float('nan')):.2f}deg "
                f"q1_margin={frame.get('left_q1_feasibility_margin_m', float('nan')) * 1000.0:.2f}mm "
                f"updown_comp={frame.get('updown_compensation', 0.0) * 1000.0:.2f}mm "
                f"joint_step={frame.get('max_joint_step_deg', float('nan')):.2f}deg "
                f"moveit_contacts={frame.get('contacts', [])} "
                f"failure={frame.get('solve_failure', '') or resume_transition.get('failure_reason', '')}"
            )
        else:
            transition = data.get("loaded_transition", {})
            status = (
                f"phase=loaded_transition frame={frame['index'] + 1}/{transition.get('point_count', 0)} "
                f"method={transition.get('method')} valid={transition.get('valid')} "
                f"updown={frame.get('compensated_updown', float('nan')):.4f}m "
                f"moveit_contacts={frame.get('contacts', [])} "
                f"post_comp_box_contacts={planned_contacts['post_compensation']} "
                f"failure={transition.get('failure_reason', '')}"
            )
        rr.log("monitor/status", rr.TextLog(status))
        sample += 1
    print(
        f"saved={args.save} samples={sample} path_valid={data.get('path_valid')} "
        f"first_failure={data.get('first_failure_frame')}:{data.get('first_failure_reason')} "
        f"loaded_valid={transition.get('valid')} loaded_method={transition.get('method')}"
    )


if __name__ == "__main__":
    main()
