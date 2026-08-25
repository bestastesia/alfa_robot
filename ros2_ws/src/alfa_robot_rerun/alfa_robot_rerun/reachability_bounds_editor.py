#!/usr/bin/env python3
"""Interactive Rerun preview for selecting a right-arm reachability volume.

This tool deliberately performs no IK or collision queries. It only renders the
current robot at a fixed updown position and lets the user tune the Cartesian
sampling bounds that can later be passed to ``nine_orient_reachability.py``.
"""

from __future__ import annotations

import argparse
import math
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import ttk

import numpy as np
import rerun as rr

from alfa_robot_rerun.visualize_rerun import (
    UrdfRobot,
    log_robot_state,
    log_robot_static_model,
    render_current_urdf,
)


@dataclass(frozen=True)
class ReachabilityBounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float
    step_x: float
    step_y: float
    step_z: float

    def normalized(self) -> "ReachabilityBounds":
        return ReachabilityBounds(
            min(self.min_x, self.max_x),
            max(self.min_x, self.max_x),
            min(self.min_y, self.max_y),
            max(self.min_y, self.max_y),
            min(self.min_z, self.max_z),
            max(self.min_z, self.max_z),
            max(0.001, self.step_x),
            max(0.001, self.step_y),
            max(0.001, self.step_z),
        )

    def center(self) -> np.ndarray:
        bounds = self.normalized()
        return np.array(
            [
                (bounds.min_x + bounds.max_x) * 0.5,
                (bounds.min_y + bounds.max_y) * 0.5,
                (bounds.min_z + bounds.max_z) * 0.5,
            ],
            dtype=float,
        )

    def half_size(self) -> np.ndarray:
        bounds = self.normalized()
        return np.array(
            [
                (bounds.max_x - bounds.min_x) * 0.5,
                (bounds.max_y - bounds.min_y) * 0.5,
                (bounds.max_z - bounds.min_z) * 0.5,
            ],
            dtype=float,
        )


def axis_values(lower: float, upper: float, step: float) -> np.ndarray:
    lower, upper = min(lower, upper), max(lower, upper)
    step = max(0.001, step)
    count = int(math.floor((upper - lower) / step + 1e-9)) + 1
    values = lower + np.arange(count, dtype=float) * step
    if values.size == 0:
        return np.array([lower], dtype=float)
    if upper - values[-1] > 1e-9:
        values = np.append(values, upper)
    return values


def grid_shape(bounds: ReachabilityBounds) -> tuple[int, int, int]:
    normalized = bounds.normalized()
    return (
        len(axis_values(normalized.min_x, normalized.max_x, normalized.step_x)),
        len(axis_values(normalized.min_y, normalized.max_y, normalized.step_y)),
        len(axis_values(normalized.min_z, normalized.max_z, normalized.step_z)),
    )


def preview_points(
    bounds: ReachabilityBounds,
    max_points: int = 30_000,
) -> tuple[np.ndarray, int]:
    normalized = bounds.normalized()
    axes = [
        axis_values(normalized.min_x, normalized.max_x, normalized.step_x),
        axis_values(normalized.min_y, normalized.max_y, normalized.step_y),
        axis_values(normalized.min_z, normalized.max_z, normalized.step_z),
    ]
    total = math.prod(len(axis) for axis in axes)
    stride = 1
    while math.prod(math.ceil(len(axis) / stride) + 1 for axis in axes) > max_points:
        stride += 1

    preview_axes: list[np.ndarray] = []
    for axis in axes:
        sampled = axis[::stride]
        if sampled[-1] != axis[-1]:
            sampled = np.append(sampled, axis[-1])
        preview_axes.append(sampled)

    mesh = np.meshgrid(*preview_axes, indexing="ij")
    points = np.column_stack([component.reshape(-1) for component in mesh])
    return points, total


def box_edges(bounds: ReachabilityBounds) -> list[list[list[float]]]:
    normalized = bounds.normalized()
    corners = np.array(
        [
            [x, y, z]
            for x in (normalized.min_x, normalized.max_x)
            for y in (normalized.min_y, normalized.max_y)
            for z in (normalized.min_z, normalized.max_z)
        ],
        dtype=float,
    )
    strips: list[list[list[float]]] = []
    for start in range(len(corners)):
        for dimension in range(3):
            end = start ^ (1 << (2 - dimension))
            if start < end:
                strips.append([corners[start].tolist(), corners[end].tolist()])
    return strips


def nine_orient_parameter_text(bounds: ReachabilityBounds, updown: float) -> str:
    bounds = bounds.normalized()
    return (
        "--ros-args "
        "-p side:=right -p reference_frame:=world "
        f"-p fixed_updown:={updown:.3f} "
        f"-p min_x:={bounds.min_x:.3f} -p max_x:={bounds.max_x:.3f} "
        f"-p min_y:={bounds.min_y:.3f} -p max_y:={bounds.max_y:.3f} "
        f"-p min_z:={bounds.min_z:.3f} -p max_z:={bounds.max_z:.3f} "
        f"-p step_x:={bounds.step_x:.3f} -p step_y:={bounds.step_y:.3f} "
        f"-p step_z:={bounds.step_z:.3f}"
    )


class ReachabilityBoundsEditor:
    UPDATE_DELAY_MS = 80

    def __init__(
        self,
        root: tk.Tk,
        robot: UrdfRobot,
        initial_bounds: ReachabilityBounds,
        updown: float,
        max_preview_points: int,
    ) -> None:
        self.root = root
        self.robot = robot
        self.updown = updown
        self.max_preview_points = max_preview_points
        self.pending_update: str | None = None
        self.variables = {
            name: tk.DoubleVar(value=value)
            for name, value in vars(initial_bounds).items()
        }
        self.status = tk.StringVar()
        self.command = tk.StringVar()

        self.root.title("右臂可达性点云范围编辑器")
        self.root.geometry("900x650")
        self._build_ui()
        self._log_robot()
        self.update_visualization()

    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(
            frame,
            text=(
                "右臂可达性采样范围预览（只显示范围，不计算 IK）\n"
                "坐标系：world；+X 车前方，+Y 车左侧，+Z 向上；Updown 固定 0.45m"
            ),
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(0, 8))

        controls = ttk.Frame(frame)
        controls.pack(fill=tk.BOTH, expand=True)
        rows = [
            ("min_x", "X 最小值", -0.5, 1.8, 0.01),
            ("max_x", "X 最大值", -0.5, 1.8, 0.01),
            ("min_y", "Y 最小值", -1.5, 1.0, 0.01),
            ("max_y", "Y 最大值", -1.5, 1.0, 0.01),
            ("min_z", "Z 最小值", 0.0, 2.8, 0.01),
            ("max_z", "Z 最大值", 0.0, 2.8, 0.01),
            ("step_x", "X 分辨率", 0.01, 0.25, 0.01),
            ("step_y", "Y 分辨率", 0.01, 0.25, 0.01),
            ("step_z", "Z 分辨率", 0.01, 0.25, 0.01),
        ]
        for row_index, (name, label, lower, upper, resolution) in enumerate(rows):
            ttk.Label(controls, text=label, width=12).grid(
                row=row_index, column=0, sticky=tk.W, padx=(0, 6), pady=3
            )
            scale = tk.Scale(
                controls,
                from_=lower,
                to=upper,
                resolution=resolution,
                orient=tk.HORIZONTAL,
                variable=self.variables[name],
                command=lambda _value: self.schedule_update(),
                showvalue=False,
                length=590,
            )
            scale.grid(row=row_index, column=1, sticky=tk.EW, pady=3)
            entry = ttk.Entry(controls, textvariable=self.variables[name], width=9)
            entry.grid(row=row_index, column=2, padx=(8, 0), pady=3)
            entry.bind("<Return>", lambda _event: self.update_visualization())
            entry.bind("<FocusOut>", lambda _event: self.schedule_update())
        controls.columnconfigure(1, weight=1)

        ttk.Separator(frame).pack(fill=tk.X, pady=8)
        ttk.Label(frame, textvariable=self.status).pack(anchor=tk.W)
        ttk.Entry(frame, textvariable=self.command, state="readonly").pack(
            fill=tk.X, pady=(6, 8)
        )

        buttons = ttk.Frame(frame)
        buttons.pack(fill=tk.X)
        ttk.Button(buttons, text="立即刷新", command=self.update_visualization).pack(side=tk.LEFT)
        ttk.Button(buttons, text="复制测试参数", command=self.copy_parameters).pack(
            side=tk.LEFT, padx=8
        )
        ttk.Button(buttons, text="恢复默认范围", command=self.reset_defaults).pack(side=tk.LEFT)
        ttk.Button(buttons, text="退出", command=self.root.destroy).pack(side=tk.RIGHT)

    def current_bounds(self) -> ReachabilityBounds:
        return ReachabilityBounds(
            **{name: float(variable.get()) for name, variable in self.variables.items()}
        ).normalized()

    def _log_robot(self) -> None:
        joint_positions = {"updown": self.updown}
        log_robot_static_model(self.robot, "world/robot", log_meshes=True)
        log_robot_state(self.robot, joint_positions, "world/robot")
        transforms = self.robot.fk(joint_positions)
        if "right_arm_base" in transforms:
            rr.log(
                "world/reachability/right_arm_base",
                rr.Points3D(
                    [transforms["right_arm_base"][:3, 3]],
                    colors=[[80, 160, 255]],
                    radii=0.025,
                    labels=["right_arm_base"],
                    show_labels=True,
                ),
                static=True,
            )
        if "right_tool0" in transforms:
            rr.log(
                "world/reachability/current_right_tool0",
                rr.Points3D(
                    [transforms["right_tool0"][:3, 3]],
                    colors=[[0, 255, 120]],
                    radii=0.02,
                    labels=["right_tool0 (all arm joints=0)"],
                    show_labels=True,
                ),
                static=True,
            )

    def schedule_update(self) -> None:
        if self.pending_update is not None:
            self.root.after_cancel(self.pending_update)
        self.pending_update = self.root.after(self.UPDATE_DELAY_MS, self.update_visualization)

    def update_visualization(self) -> None:
        self.pending_update = None
        try:
            bounds = self.current_bounds()
            points, total_points = preview_points(bounds, self.max_preview_points)
        except (ValueError, tk.TclError) as error:
            self.status.set(f"参数无效：{error}")
            return

        shape = grid_shape(bounds)
        rr.log(
            "world/reachability/right_arm/sample_volume",
            rr.Boxes3D(
                centers=[bounds.center()],
                half_sizes=[bounds.half_size()],
                colors=[[255, 170, 0, 45]],
                fill_mode=rr.components.FillMode.TransparentFillMajorWireframe,
                labels=["right-arm sample volume"],
                show_labels=True,
            ),
        )
        rr.log(
            "world/reachability/right_arm/bounds_edges",
            rr.LineStrips3D(
                box_edges(bounds),
                colors=[[255, 170, 0]],
                radii=0.004,
            ),
        )
        rr.log(
            "world/reachability/right_arm/sample_points",
            rr.Points3D(
                points,
                colors=[[40, 170, 255, 150]],
                radii=0.004,
            ),
        )

        parameter_text = nine_orient_parameter_text(bounds, self.updown)
        status_text = (
            f"网格={shape[0]}×{shape[1]}×{shape[2]}，实际测试点={total_points:,}，"
            f"Rerun预览点={len(points):,}；每点9朝向预计={total_points * 9:,}次IK"
        )
        self.status.set(status_text)
        self.command.set(parameter_text)
        rr.log(
            "world/reachability/right_arm/parameters",
            rr.TextDocument(
                "## 右臂可达性范围\n\n"
                f"- Updown: `{self.updown:.3f} m`\n"
                f"- X: `{bounds.min_x:.3f} .. {bounds.max_x:.3f} m`, step `{bounds.step_x:.3f}`\n"
                f"- Y: `{bounds.min_y:.3f} .. {bounds.max_y:.3f} m`, step `{bounds.step_y:.3f}`\n"
                f"- Z: `{bounds.min_z:.3f} .. {bounds.max_z:.3f} m`, step `{bounds.step_z:.3f}`\n"
                f"- Grid: `{shape[0]} × {shape[1]} × {shape[2]} = {total_points:,}` points\n"
                f"- Nine-orientation IK calls: `{total_points * 9:,}`\n\n"
                f"```text\n{parameter_text}\n```"
            )
        )

    def copy_parameters(self) -> None:
        text = self.command.get()
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.root.update()
        print(text, flush=True)
        self.status.set(f"参数已复制到剪贴板：{text}")

    def reset_defaults(self) -> None:
        defaults = default_bounds()
        for name, value in vars(defaults).items():
            self.variables[name].set(value)
        self.update_visualization()


def default_bounds() -> ReachabilityBounds:
    return ReachabilityBounds(
        min_x=0.20,
        max_x=1.20,
        min_y=-0.80,
        max_y=0.30,
        min_z=0.45,
        max_z=2.30,
        step_x=0.10,
        step_y=0.10,
        step_z=0.10,
    )


def parse_args() -> argparse.Namespace:
    defaults = default_bounds()
    parser = argparse.ArgumentParser(
        description="Preview right-arm reachability bounds with the full robot in Rerun."
    )
    for name, value in vars(defaults).items():
        parser.add_argument(f"--{name.replace('_', '-')}", type=float, default=value)
    parser.add_argument("--updown", type=float, default=0.45)
    parser.add_argument("--max-preview-points", type=int, default=30_000)
    parser.add_argument("--save", default="", help="Optionally save the interactive session to .rrd")
    parser.add_argument("--no-spawn", action="store_true", help="Do not spawn the Rerun viewer")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not 0.0 <= args.updown <= 0.7:
        raise SystemExit("--updown must be within the current URDF limit [0.0, 0.7] m")
    if args.max_preview_points < 100:
        raise SystemExit("--max-preview-points must be >= 100")

    rr.init("right_arm_reachability_bounds_editor", recording_id="right_arm_bounds")
    if args.save:
        save_path = Path(args.save).expanduser().resolve()
        save_path.parent.mkdir(parents=True, exist_ok=True)
        rr.save(str(save_path))
    if not args.no_spawn:
        rr.spawn()

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    robot = UrdfRobot(render_current_urdf())
    initial_bounds = ReachabilityBounds(
        **{name: float(getattr(args, name)) for name in vars(default_bounds())}
    )
    root = tk.Tk()
    ReachabilityBoundsEditor(
        root,
        robot,
        initial_bounds,
        float(args.updown),
        int(args.max_preview_points),
    )
    root.mainloop()


if __name__ == "__main__":
    main()
