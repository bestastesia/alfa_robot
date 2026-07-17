#!/usr/bin/python3
"""离线回放 stage snapshot 到 Rerun，验证集装箱几何与 MoveIt 规划场景保持一致。

与 extract_stage_monitor_console.py 的关键区别：这里的障碍几何（集装箱三块墙板、
静态箱墙）完全从 snapshot JSON 读取，不在 Python 侧重新计算任何几何——
snapshot 里的 container_panels/static_box_obstacles 字段就是
MotionSceneAdapter 实际写入 MoveIt PlanningScene 碰撞检测用的同一份数据。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from alfa_robot_rerun import visualize_rerun as helpers

rr: Any = None


def read_snapshot(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(path)
    return json.loads(path.read_text())


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
    return [x, y, z, w]


def log_container_panels(panels: list[dict[str, Any]]) -> None:
    """直接画 snapshot 里的 container_panels 字段，含 yaw；不重新计算几何。"""
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
    )


def log_static_box_obstacles(config: dict[str, Any] | None) -> None:
    """跟 extract_stage_monitor_console.py 里的同名函数保持同一套读取逻辑：
    只读 snapshot 的 static_box_obstacles 字段，不重新计算。"""
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
    rr.log(
        "monitor/scene/static_box_obstacles",
        rr.Boxes3D(centers=centers, half_sizes=half_sizes, colors=colors, labels=labels),
    )


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


def joint_dict_from_stage_point(stage: dict[str, Any], point: dict[str, Any]) -> dict[str, float]:
    state_map = stage.get("start_state", {}).get("joint_map", {})
    out = {str(name): float(value) for name, value in state_map.items()}
    names = stage.get("trajectory", {}).get("joint_names", [])
    positions = point.get("positions", [])
    for name, value in zip(names, positions):
        out[str(name)] = float(value)
    return out


def playback_points_for_stage(stage: dict[str, Any]) -> list[dict[str, Any]]:
    points = list(stage.get("trajectory", {}).get("points", []))
    if points:
        return points
    state_map = stage.get("start_state", {}).get("joint_map", {})
    names = stage.get("trajectory", {}).get("joint_names", [])
    if not state_map or not names:
        return []
    try:
        positions = [float(state_map[str(name)]) for name in names]
    except KeyError:
        return []
    return [{"time_from_start_sec": 0.0, "positions": positions}]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="回放一份 extract_monitor stage snapshot，障碍几何完全来自 JSON，不重新计算。"
    )
    parser.add_argument("snapshot", type=Path, help="stage snapshot JSON 文件路径")
    parser.add_argument("--connect", action="store_true", help="连接已运行的 rerun viewer，而不是新开一个窗口")
    parser.add_argument("--save", type=Path, default=None, help="保存为 .rrd 文件而不是打开窗口")
    parser.add_argument("--stride", type=int, default=1, help="轨迹点采样步长")
    args = parser.parse_args()

    snapshot = read_snapshot(args.snapshot)
    replay_stages = list(snapshot.get("replay_stages", []))
    container_panels = list(snapshot.get("container_panels", []))
    if not container_panels:
        print(
            "警告：snapshot 里没有 container_panels 字段，集装箱将不会显示。"
            " 请确认 dual_arm_planner_node 已包含本次改动并重新生成 snapshot。",
            file=sys.stderr,
        )
    if not replay_stages:
        print("snapshot 里没有 replay_stages，无法回放轨迹。", file=sys.stderr)
        return 1

    global rr
    import rerun as rerun_module

    rr = rerun_module
    rr.init("replay_container_pose_scene")
    if args.save is not None:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        rr.save(str(args.save))
    elif args.connect:
        rr.connect()
    else:
        rr.spawn()

    urdf_text = helpers.render_current_urdf()
    robot = helpers.UrdfRobot(urdf_text)
    helpers.log_robot_static_model(robot, "monitor/robot")

    sample = 0
    for stage_index, stage in enumerate(replay_stages):
        points = playback_points_for_stage(stage)
        if not points:
            continue
        static_box_obstacles = stage.get("static_box_obstacles")
        attached_boxes = stage.get("attached_boxes", [])
        selected_indices = list(range(0, len(points), max(1, args.stride)))
        if selected_indices[-1] != len(points) - 1:
            selected_indices.append(len(points) - 1)
        for point_index in selected_indices:
            helpers.set_sample_time(sample)
            log_container_panels(container_panels)
            log_static_box_obstacles(static_box_obstacles)
            point = points[point_index]
            joints = joint_dict_from_stage_point(stage, point)
            helpers.log_robot_state(robot, joints, "monitor/robot")
            log_attached_boxes(robot, joints, attached_boxes)
            rr.log(
                "monitor/info",
                rr.TextLog(
                    f"stage {stage_index + 1}/{len(replay_stages)}: {stage.get('stage')} | "
                    f"point {point_index + 1}/{len(points)}"
                ),
            )
            sample += 1

    print(f"回放完成：{len(replay_stages)} 个 stage，共 {sample} 帧。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
