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

def main() -> None:
    default_config = Path(__file__).resolve().parent / "configs" / "example_4dof.yaml"
    parser = argparse.ArgumentParser(description="Open Robotics Toolbox teach() for a DH robot")
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--q", type=float, nargs="*", default=None, help="Initial joint values")
    parser.add_argument("--block", action="store_true", help="Block Python process while viewer is open")
    parser.add_argument("--backend", choices=["custom", "pyplot", "swift"], default="custom", help="custom is a local matplotlib slider viewer; pyplot/swift call Robotics Toolbox teach()")
    args = parser.parse_args()

    config = load_config(args.config)
    q = parse_q(args.q, len(config["links"]))

    print(f"Opening teach viewer with q={q}, backend={args.backend}")
    if args.backend == "custom":
        print(f"Robot: {config.get('name', 'DH Robot')}, {len(config['links'])} joints, convention={config.get('convention', 'standard')}")
        teach_custom_matplotlib(config, q)
        return

    robot = build_robot(config)
    print(robot)
    try:
        robot.teach(q=q, block=args.block, backend=args.backend)
    except NotImplementedError as exc:
        if args.backend == "swift" and "DHRobots" in str(exc):
            raise SystemExit(
                "Your Robotics Toolbox version does not implement Swift plotting for DHRobot. "
                "Use the built-in custom backend instead:\n"
                "  python dh_teach.py --backend custom\n"
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
