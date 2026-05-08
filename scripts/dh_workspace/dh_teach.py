#!/usr/bin/env python3
"""Open a Robotics Toolbox teach() viewer for a DH config."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import yaml
import numpy as np


def load_config(path: Path) -> dict[str, Any]:
    with path.open() as config_file:
        return yaml.safe_load(config_file)


def build_robot(config: dict[str, Any]):
    try:
        import roboticstoolbox as rtb
    except Exception as exc:
        message = str(exc)
        if "_ARRAY_API" in message or "NumPy 1.x" in message or "numpy" in message.lower():
            raise SystemExit(
                "Robotics Toolbox failed to import because its compiled extension is not compatible "
                "with the current NumPy. In this conda env, downgrade NumPy and reinstall/use the toolbox:\n"
                "  python -m pip install --force-reinstall 'numpy<2'\n"
                "  python -m pip install --force-reinstall roboticstoolbox-python spatialmath-python swift-sim\n"
            ) from exc
        raise SystemExit(
            "roboticstoolbox-python failed to import. Install or repair it with:\n"
            "  python -m pip install roboticstoolbox-python spatialmath-python swift-sim\n"
            f"Original error: {exc}"
        ) from exc

    convention = config.get("convention", "standard")
    if convention != "standard":
        raise SystemExit("Robotics Toolbox teach backend currently supports standard DH configs only. Use --backend custom for this config.")

    rtb_links = []
    for link in config["links"]:
        joint_type = link.get("type", "revolute")
        if joint_type == "revolute":
            rtb_links.append(
                rtb.RevoluteDH(
                    d=float(link.get("d", 0.0)),
                    a=float(link.get("a", 0.0)),
                    alpha=float(link.get("alpha", 0.0)),
                    qlim=link.get("qlim"),
                )
            )
        elif joint_type == "prismatic":
            rtb_links.append(
                rtb.PrismaticDH(
                    theta=float(link.get("theta", 0.0)),
                    a=float(link.get("a", 0.0)),
                    alpha=float(link.get("alpha", 0.0)),
                    qlim=link.get("qlim"),
                )
            )
        else:
            raise SystemExit(f"unsupported joint type: {joint_type}")

    return rtb.DHRobot(rtb_links, name=config.get("name", "DH Robot"))


def parse_q(raw_q: list[float] | None, dof: int) -> list[float]:
    if raw_q is None:
        return [0.0] * dof
    if len(raw_q) != dof:
        raise SystemExit(f"--q expects {dof} values, got {len(raw_q)}")
    return raw_q



def joint_frames_builtin(config: dict[str, Any], q: list[float]) -> np.ndarray:
    from dh_workspace_sampler import dh_transform, urdf_origin_axis_transform

    convention = config.get("convention", "standard")
    frames = [np.eye(4)]
    transform = np.eye(4)
    for joint_value, link in zip(q, config["links"]):
        if convention == "urdf_origin_axis":
            transform = transform @ urdf_origin_axis_transform(link, float(joint_value))
            frames.append(transform.copy())
            continue

        joint_type = link.get("type", "revolute")
        theta_offset = float(link.get("theta", 0.0))
        d_offset = float(link.get("d", 0.0))
        if joint_type in {"revolute", "continuous"}:
            theta = theta_offset + float(joint_value)
            d = d_offset
        elif joint_type == "prismatic":
            theta = theta_offset
            d = d_offset + float(joint_value)
        else:
            raise SystemExit(f"unsupported joint type: {joint_type}")
        transform = transform @ dh_transform(
            theta=theta,
            d=d,
            a=float(link.get("a", 0.0)),
            alpha=float(link.get("alpha", 0.0)),
            convention=convention,
        )
        frames.append(transform.copy())
    return np.asarray(frames)


def teach_custom_matplotlib(config: dict[str, Any], q0: list[float]) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider

    links = config["links"]
    q = np.asarray(q0, dtype=float)

    fig = plt.figure(figsize=(10, 8))
    axis = fig.add_subplot(111, projection="3d")
    plt.subplots_adjust(left=0.1, bottom=0.1 + 0.055 * len(links))

    line, = axis.plot([], [], [], "o-", linewidth=3, markersize=6)
    frame_lines = []
    colors = ["r", "g", "b"]
    for _ in range(len(links) + 1):
        frame_lines.append([axis.plot([], [], [], color=color, linewidth=1.5)[0] for color in colors])

    def redraw():
        frames = joint_frames_builtin(config, q.tolist())
        points = frames[:, :3, 3]
        line.set_data(points[:, 0], points[:, 1])
        line.set_3d_properties(points[:, 2])

        span = max(np.ptp(points[:, 0]), np.ptp(points[:, 1]), np.ptp(points[:, 2]), 0.5)
        center = points.mean(axis=0)
        margin = span * 0.65
        axis.set_xlim(center[0] - margin, center[0] + margin)
        axis.set_ylim(center[1] - margin, center[1] + margin)
        axis.set_zlim(center[2] - margin, center[2] + margin)
        axis.set_xlabel("X")
        axis.set_ylabel("Y")
        axis.set_zlabel("Z")
        axis.set_title(config.get("name", "DH Robot"))

        frame_size = span * 0.08
        for frame, lines in zip(frames, frame_lines):
            origin = frame[:3, 3]
            rotation = frame[:3, :3]
            for axis_index, frame_line in enumerate(lines):
                end = origin + rotation[:, axis_index] * frame_size
                frame_line.set_data([origin[0], end[0]], [origin[1], end[1]])
                frame_line.set_3d_properties([origin[2], end[2]])
        fig.canvas.draw_idle()

    sliders = []
    for index, link in enumerate(links):
        lower, upper = link.get("qlim", [-np.pi, np.pi])
        slider_axis = fig.add_axes([0.18, 0.05 + index * 0.045, 0.72, 0.025])
        slider = Slider(
            ax=slider_axis,
            label=link.get("name", f"q{index + 1}"),
            valmin=float(lower),
            valmax=float(upper),
            valinit=float(q[index]),
        )
        slider.on_changed(lambda value, i=index: (q.__setitem__(i, value), redraw()))
        sliders.append(slider)

    redraw()
    plt.show()


def cylinder_pose_between(start: np.ndarray, end: np.ndarray):
    from spatialmath import SE3, SO3

    direction = end - start
    length = float(np.linalg.norm(direction))
    if length < 1e-9:
        return None, 0.0

    z_axis = direction / length
    helper = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(helper, z_axis))) > 0.95:
        helper = np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(helper, z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    center = (start + end) * 0.5
    return SE3.Rt(SO3(rotation, check=False), center), length


def joint_axis_world(config: dict[str, Any], link: dict[str, Any], parent_frame: np.ndarray) -> np.ndarray:
    convention = config.get("convention", "standard")
    if convention == "urdf_origin_axis":
        from dh_workspace_sampler import rpy_matrix

        origin = link.get("origin", {})
        axis = np.asarray(link.get("axis", [0.0, 0.0, 1.0]), dtype=float)
        rotation = parent_frame[:3, :3] @ rpy_matrix([float(value) for value in origin.get("rpy", [0.0, 0.0, 0.0])])
        world_axis = rotation @ axis
    else:
        world_axis = parent_frame[:3, 2]

    norm = np.linalg.norm(world_axis)
    if norm < 1e-9:
        return np.array([0.0, 0.0, 1.0])
    return world_axis / norm


def add_cylinder(env, sg, start: np.ndarray, end: np.ndarray, radius: float, color: list[float]) -> None:
    pose, length = cylinder_pose_between(start, end)
    if pose is not None:
        env.add(sg.Cylinder(radius=radius, length=length, base=pose, color=color))


def add_box_between(env, sg, start: np.ndarray, end: np.ndarray, thickness: float, color: list[float]) -> None:
    from spatialmath import SE3, SO3

    direction = end - start
    length = float(np.linalg.norm(direction))
    if length < 1e-9:
        return
    z_axis = direction / length
    helper = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(helper, z_axis))) > 0.95:
        helper = np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(helper, z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    center = (start + end) * 0.5
    env.add(sg.Cuboid(scale=[thickness, thickness, length], base=SE3.Rt(SO3(rotation, check=False), center), color=color))


def add_joint_symbol(env, sg, origin: np.ndarray, axis: np.ndarray, joint_type: str, size: float) -> None:
    from spatialmath import SE3

    if joint_type == "prismatic":
        rail_length = size * 6.0
        rail_gap = size * 1.15
        helper = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(helper, axis))) > 0.95:
            helper = np.array([0.0, 1.0, 0.0])
        side = np.cross(axis, helper)
        side = side / np.linalg.norm(side)
        for offset in (-rail_gap, rail_gap):
            start = origin - axis * rail_length * 0.5 + side * offset
            end = origin + axis * rail_length * 0.5 + side * offset
            add_box_between(env, sg, start, end, size * 0.45, [0.35, 0.35, 0.38, 1.0])
        add_box_between(env, sg, origin - axis * size * 1.2, origin + axis * size * 1.2, size * 1.35, [0.95, 0.62, 0.12, 1.0])
        return

    env.add(sg.Sphere(radius=size * 1.25, base=SE3(origin), color=[0.95, 0.35, 0.10, 1.0]))
    add_cylinder(env, sg, origin - axis * size * 2.2, origin + axis * size * 2.2, size * 0.55, [0.12, 0.12, 0.13, 1.0])
    add_cylinder(env, sg, origin - axis * size * 1.2, origin + axis * size * 1.2, size * 1.15, [0.85, 0.85, 0.88, 1.0])


def teach_swift_schematic(config: dict[str, Any], q0: list[float], block: bool) -> None:
    try:
        import swift
        import spatialgeometry as sg
        from spatialmath import SE3, SO3
    except Exception as exc:
        raise SystemExit(
            "Swift schematic backend requires swift-sim, spatialgeometry and spatialmath. Install/use it with:\n"
            "  pip install swift-sim spatialgeometry spatialmath-python 'websockets<11'\n"
            "or run in your dhviz conda env.\n"
            f"Original error: {exc}"
        ) from exc

    frames = joint_frames_builtin(config, q0)
    points = frames[:, :3, 3]
    span = max(float(np.ptp(points[:, 0])), float(np.ptp(points[:, 1])), float(np.ptp(points[:, 2])), 0.5)
    link_radius = float(config.get("visual", {}).get("link_radius", max(span * 0.025, 0.015)))
    joint_radius = float(config.get("visual", {}).get("joint_radius", link_radius * 1.8))
    frame_axis_length = float(config.get("visual", {}).get("frame_axis_length", max(span * 0.08, 0.05)))

    env = swift.Swift()
    try:
        env.launch(realtime=True, browser="default")
    except RuntimeError as exc:
        if "no running event loop" in str(exc):
            raise SystemExit(
                "Swift failed to start because your websockets package is too new for this swift-sim version.\n"
                "Fix in the active Python/conda env with:\n"
                "  pip install 'websockets<11'\n"
                "Then rerun --backend swift-schematic."
            ) from exc
        raise

    for link_index, (link, start, end) in enumerate(zip(config["links"], points[:-1], points[1:])):
        joint_type = link.get("type", "revolute")
        axis = joint_axis_world(config, link, frames[link_index])

        if np.linalg.norm(end - start) > 1e-9:
            add_cylinder(env, sg, start, end, link_radius, [0.12, 0.42, 0.86, 1.0])

        joint_origin = start
        add_joint_symbol(env, sg, joint_origin, axis, joint_type, joint_radius)

        t_bar = joint_radius * (4.0 if joint_type == "prismatic" else 3.2)
        add_cylinder(
            env,
            sg,
            joint_origin - axis * t_bar,
            joint_origin + axis * t_bar,
            link_radius * 0.55,
            [0.08, 0.08, 0.09, 1.0],
        )

        if np.linalg.norm(end - start) > 1e-9:
            link_direction = (end - start) / np.linalg.norm(end - start)
            shoulder = joint_origin + link_direction * min(np.linalg.norm(end - start) * 0.22, joint_radius * 3.0)
            add_cylinder(env, sg, joint_origin, shoulder, link_radius * 1.25, [0.10, 0.32, 0.72, 1.0])

    add_joint_symbol(env, sg, points[-1], frames[-1, :3, 2], "fixed", joint_radius * 0.8)

    for frame in frames:
        origin = frame[:3, 3]
        rotation = frame[:3, :3]
        axis_colors = ([1.0, 0.0, 0.0, 1.0], [0.0, 0.8, 0.0, 1.0], [0.0, 0.2, 1.0, 1.0])
        for axis_index, color in enumerate(axis_colors):
            axis_end = origin + rotation[:, axis_index] * frame_axis_length
            add_cylinder(env, sg, origin, axis_end, link_radius * 0.28, color)

    center = points.mean(axis=0)
    env.set_camera_pose([center[0] + span, center[1] - span * 1.8, center[2] + span * 0.9], center.tolist())
    print("Swift schematic viewer opened in browser.")
    print("This renders a DH engineering schematic: blue links, orange joints, dark joint axes, grey prismatic rails.")
    print("Re-run with --q to change pose.")

    if block:
        env.hold()
    else:
        env.step(0)


def teach_swift_cylinders(config: dict[str, Any], q0: list[float], block: bool) -> None:
    teach_swift_schematic(config, q0, block)

def main() -> None:
    default_config = Path(__file__).resolve().parent / "configs" / "example_4dof.yaml"
    parser = argparse.ArgumentParser(description="Open Robotics Toolbox teach() for a DH robot")
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--q", type=float, nargs="*", default=None, help="Initial joint values")
    parser.add_argument("--block", action="store_true", help="Block Python process while viewer is open")
    parser.add_argument(
        "--backend",
        choices=["custom", "swift-schematic", "swift-cylinder", "pyplot", "swift"],
        default="custom",
        help="custom: matplotlib sliders; swift-schematic: browser DH schematic via Swift; pyplot/swift: Robotics Toolbox teach() backends",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    q = parse_q(args.q, len(config["links"]))

    print(f"Opening teach viewer with q={q}, backend={args.backend}")
    if args.backend == "custom":
        print(f"Robot: {config.get('name', 'DH Robot')}, {len(config['links'])} joints, convention={config.get('convention', 'standard')}")
        teach_custom_matplotlib(config, q)
        return
    if args.backend in {"swift-schematic", "swift-cylinder"}:
        print(f"Robot: {config.get('name', 'DH Robot')}, {len(config['links'])} joints, convention={config.get('convention', 'standard')}")
        teach_swift_schematic(config, q, block=args.block)
        return

    robot = build_robot(config)
    print(robot)
    try:
        robot.teach(q=q, block=args.block, backend=args.backend)
    except NotImplementedError as exc:
        if args.backend == "swift" and "DHRobots" in str(exc):
            raise SystemExit(
                "Your Robotics Toolbox version does not implement Swift plotting for DHRobot. "
                "Use the project Swift schematic backend instead:\n"
                "  python dh_teach.py --backend swift-schematic\n"
            ) from exc
        raise
    except TypeError as exc:
        if args.backend == "pyplot" and "Slider" in str(exc):
            raise SystemExit(
                "PyPlot teach backend is incompatible with your matplotlib version. "
                "Use the built-in custom backend, or downgrade matplotlib:\n"
                "  python dh_teach.py --backend custom\n"
                "  python -m pip install 'matplotlib<3.8'\n"
            ) from exc
        raise


if __name__ == "__main__":
    main()
