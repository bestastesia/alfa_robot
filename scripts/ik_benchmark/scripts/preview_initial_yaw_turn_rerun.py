#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import rerun as rr

REPO_ROOT = Path(__file__).resolve().parents[3]
HELPER_PATH = REPO_ROOT / "scripts/ik_benchmark/scripts/visualize_rerun.py"


def load_helpers():
    spec = importlib.util.spec_from_file_location("alfa_visualize_rerun_helpers", HELPER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load helper: {HELPER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def yaw_matrix(yaw_rad: float) -> np.ndarray:
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    transform = np.eye(4)
    transform[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return transform


def base_transform(x: float, y: float, yaw_rad: float) -> np.ndarray:
    transform = yaw_matrix(yaw_rad)
    transform[:3, 3] = [x, y, 0.0]
    return transform


def transform_point(transform: np.ndarray, point: list[float]) -> list[float]:
    homogeneous = transform @ np.array([point[0], point[1], point[2], 1.0], dtype=float)
    return homogeneous[:3].tolist()


def log_world_axes() -> None:
    rr.log(
        "world/axes",
        rr.Arrows3D(
            origins=[[0.0, 0.0, 0.02], [0.0, 0.0, 0.02]],
            vectors=[[0.6, 0.0, 0.0], [0.0, 0.6, 0.0]],
            colors=[[255, 60, 60], [60, 255, 60]],
            labels=["world +X / 货墙方向", "world +Y"],
        ),
        static=True,
    )


def log_world_scene(helpers, box_front_x: float, scene_y_shift: float, scene_yaw_rad: float, left_box_id: int, right_box_id: int) -> None:
    scene_tf = yaw_matrix(scene_yaw_rad)
    scene_tf[:3, 3] = [0.0, scene_y_shift, 0.0]

    thickness = 0.02
    length = 4.0
    width = 2.2
    height = 2.4
    center_x = 0.8
    panels = [
        ([center_x, width * 0.5 + thickness * 0.5, height * 0.5], [length, thickness, height], "container_left_wall"),
        ([center_x, -width * 0.5 - thickness * 0.5, height * 0.5], [length, thickness, height], "container_right_wall"),
        ([center_x, 0.0, height + thickness * 0.5], [length, width + 2.0 * thickness, thickness], "container_ceiling"),
    ]
    scene_quat = helpers.matrix_to_quaternion_xyzw(scene_tf[:3, :3])
    rr.log(
        "world/scene/container",
        rr.Boxes3D(
            centers=[transform_point(scene_tf, panel[0]) for panel in panels],
            half_sizes=[[value * 0.5 for value in panel[1]] for panel in panels],
            quaternions=[scene_quat for _ in panels],
            colors=[[80, 170, 255, 45] for _ in panels],
            labels=[panel[2] for panel in panels],
        ),
        static=True,
    )

    rows = [
        [(1, 0.8), (2, 0.4), (3, 0.0), (4, -0.4), (5, -0.8)],
        [(6, 0.8), (7, 0.4), (8, 0.0), (9, -0.4), (10, -0.8)],
        [(11, 0.8), (12, 0.4), (13, 0.0), (14, -0.4), (15, -0.8)],
        [(16, 0.8), (17, 0.4), (18, 0.0), (19, -0.4), (20, -0.8)],
        [(21, 0.8), (22, 0.4), (23, 0.0), (24, -0.4), (25, -0.8)],
    ]
    centers = []
    half_sizes = []
    colors = []
    labels = []
    for row_i, row in enumerate(rows):
        z = 0.2 + 0.4 * (len(rows) - 1 - row_i)
        for box_id, y in row:
            centers.append(transform_point(scene_tf, [box_front_x + 0.15, y, z]))
            half_sizes.append([0.15, 0.2, 0.2])
            colors.append([80, 240, 100, 190] if box_id in (left_box_id, right_box_id) else [255, 180, 60, 125])
            labels.append(str(box_id))
    rr.log(
        "world/scene/boxes",
        rr.Boxes3D(
            centers=centers,
            half_sizes=half_sizes,
            quaternions=[scene_quat for _ in centers],
            colors=colors,
            labels=labels,
        ),
        static=True,
    )

    log_axes("world/scene/frame", transform_point(scene_tf, [0.0, 0.0, 0.04]), scene_yaw_rad, "scene/container frame")


def log_axes(path: str, origin: list[float], yaw_rad: float, label: str) -> None:
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    x_vec = [0.35 * c, 0.35 * s, 0.0]
    y_vec = [-0.25 * s, 0.25 * c, 0.0]
    rr.log(
        f"{path}/axes",
        rr.Arrows3D(
            origins=[origin, origin],
            vectors=[x_vec, y_vec],
            colors=[[255, 80, 80], [80, 255, 80]],
            labels=[f"{label} +X", f"{label} +Y"],
        ),
    )
    rr.log(f"{path}/label", rr.TextLog(label))


def log_robot(
    helpers,
    robot,
    root: str,
    joint_map: dict[str, float],
    base_yaw_rad: float,
    base_x: float,
    base_y: float,
    label: str,
) -> None:
    # Log every link as an absolute world transform below an identity root.
    # Do not also rotate the root entity, otherwise Rerun's hierarchy applies
    # base_yaw twice and the visual pose becomes misleading.
    rr.log(root, rr.Transform3D(translation=[0.0, 0.0, 0.0]))
    helpers.log_robot_static_model(robot, root, log_meshes=True)
    transforms = robot.fk(joint_map)
    base_tf = base_transform(base_x, base_y, base_yaw_rad)
    for link_name, transform in transforms.items():
        helpers.log_transform_matrix(f"{root}/{link_name}", base_tf @ transform)
    log_axes(f"{root}/base_frame_marker", [base_x, base_y, 0.05], base_yaw_rad, label)


def main() -> int:
    parser = argparse.ArgumentParser(description="预览 ALFA 初始姿态：车体 yaw 与 turn 反向补偿")
    parser.add_argument("--base-yaw-deg", type=float, default=90.0, help="车体相对 world 的 yaw，默认 +90°")
    parser.add_argument("--base-x", type=float, default=0.0, help="车体底盘在 world 中的 x 平移")
    parser.add_argument("--base-y", type=float, default=0.0, help="车体底盘在 world 中的 y 平移")
    parser.add_argument("--turn-deg", type=float, default=-90.0, help="turn 关节角，默认 -90°")
    parser.add_argument("--updown", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--save", type=Path, default=REPO_ROOT / "data/ik_benchmark/initial_pose_preview/base_yaw90_turn_minus90.rrd")
    parser.add_argument("--with-scene", action=argparse.BooleanOptionalAction, default=True, help="显示箱垛和集装箱")
    parser.add_argument("--box-front-x", type=float, default=0.925)
    parser.add_argument("--scene-y-shift", type=float, default=0.0, help="世界系中货物/集装箱整体 y 平移")
    parser.add_argument("--scene-yaw-deg", type=float, default=0.0, help="世界系中货物/集装箱 yaw；默认不随车旋转")
    parser.add_argument("--left-box-id", type=int, default=2)
    parser.add_argument("--right-box-id", type=int, default=3)
    parser.add_argument("--connect", action="store_true")
    parser.add_argument("--spawn", action="store_true")
    parser.add_argument("--show-reference", action="store_true", help="同时显示原始 yaw=0/turn=0 姿态作灰色对照")
    args = parser.parse_args()

    helpers = load_helpers()
    robot = helpers.UrdfRobot(helpers.render_current_urdf())

    rr.init("alfa_initial_yaw_turn_preview")
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        rr.save(str(args.save))
    elif args.connect:
        rr.connect()
    elif args.spawn:
        rr.spawn()

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    log_world_axes()
    rr.log(
        "world/info",
        rr.TextLog(
            "初始姿态预览\n"
            "坐标语义：货物/集装箱固定在 world；车体在 world 内旋转。\n"
            f"车体 base yaw = {args.base_yaw_deg:.1f}°\n"
            f"车体 base xy = ({args.base_x:.3f}, {args.base_y:.3f}) m\n"
            f"turn = {args.turn_deg:.1f}°\n"
            f"scene yaw = {args.scene_yaw_deg:.1f}°\n"
            f"pitch = {args.pitch_deg:.1f}°\n"
            f"updown = {args.updown:.3f} m\n"
            "红箭头为车体 +X，绿箭头为车体 +Y。"
        ),
        static=True,
    )

    if args.with_scene:
        log_world_scene(
            helpers,
            args.box_front_x,
            args.scene_y_shift,
            math.radians(args.scene_yaw_deg),
            args.left_box_id,
            args.right_box_id,
        )

    joint_map = {
        "pitch": math.radians(args.pitch_deg),
        "turn": math.radians(args.turn_deg),
        "updown": args.updown,
    }
    for side in ("left", "right"):
        for idx in range(1, 7):
            joint_map[f"{side}joint{idx}"] = 0.0

    helpers.set_sample_time(0)
    log_robot(
        helpers,
        robot,
        "world/robot_base_yaw90_turn_minus90",
        joint_map,
        math.radians(args.base_yaw_deg),
        args.base_x,
        args.base_y,
        f"base_yaw={args.base_yaw_deg:.0f} turn={args.turn_deg:.0f}",
    )

    if args.show_reference:
        reference_joint_map = dict(joint_map)
        reference_joint_map["turn"] = 0.0
        log_robot(
            helpers,
            robot,
            "world/reference_yaw0_turn0",
            reference_joint_map,
            0.0,
            0.0,
            0.0,
            "reference yaw=0 turn=0",
        )

    print(f"已生成 Rerun: {args.save}")
    print(f"查看: rerun {args.save}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
