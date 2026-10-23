#!/usr/bin/env python3
"""MOTION-312: static figure of one case — transport tool paths and carried-box rotation per arm.

For each recorded result of the case (`full/<case>-<arm>.json`), the executed transport frames are
run through forward kinematics: top and side views of the tool path, and the box rotation relative
to its orientation at the start of transport over normalised transport time (the required rotation
is the angle between the start and end orientations) together with the cumulative rotation, whose
excess over the required rotation is the extra rotation reported by the A/B summary. CPU only.
"""
import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import yourdfpy

from replay import MOBILE, pose_matrix

COLORS = ("#7f8ea3", "#e8892f", "#3fa36a", "#a352b5")


def transport(urdf, result):
    names = result["joint_names"]
    rows = [row for row, phase in zip(result["frames"], result["phases"]) if phase == "transport"]
    event = next(item for item in result["predicted_scene_events"] if item["phase"] == "attach")
    attachment = event["snapshot"]["attachments"][0]
    tool_to_box = pose_matrix(attachment["tool_to_object"])
    positions, rotations = [], []
    for row in rows:
        values = dict(zip(names, row))
        urdf.update_cfg(np.array([values.get(name, 0.) for name in urdf.actuated_joint_names]))
        tool = urdf.get_transform(attachment["parent_link"], urdf.base_link)
        positions.append(tool[:3, 3])
        rotations.append((tool @ tool_to_box)[:3, :3])
    rotations = Rotation.from_matrix(np.asarray(rotations))
    angle = np.degrees((rotations[0].inv() * rotations).magnitude())
    cumulative = np.concatenate([[0.], np.cumsum(np.degrees((rotations[:-1].inv() * rotations[1:]).magnitude()))])
    times = np.asarray(result.get("time_from_start_s") or np.arange(len(result["frames"])) * 0.025)
    times = np.asarray([t for t, phase in zip(times, result["phases"]) if phase == "transport"])
    return np.asarray(positions), angle, cumulative, (times - times[0]) / max(times[-1] - times[0], 1e-9), times[-1] - times[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, required=True, help="research/curobo_v3 of a prepared runtime (for the URDF)")
    parser.add_argument("--results-dir", type=Path, required=True)
    parser.add_argument("--case", required=True)
    parser.add_argument("--arms", default="joint_rrt,cartesian_v2_toppra")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    urdf = yourdfpy.URDF.load(yaml.safe_load((args.runtime / MOBILE).read_text())["kinematics"]["urdf_path"], load_meshes=False)
    figure, axes = plt.subplots(1, 3, figsize=(16, 4.8))
    for color, arm in zip(COLORS, args.arms.split(",")):
        result = json.loads((args.results_dir / f"{args.case}-{arm}.json").read_text())
        if not result["success"]:
            continue
        points, angle, cumulative, progress, duration = transport(urdf, result)
        label = f"{arm}（{duration:.1f} s）"
        axes[0].plot(points[:, 0], points[:, 1], color=color, label=label)
        axes[1].plot(points[:, 0], points[:, 2], color=color, label=label)
        axes[2].plot(progress, angle, color=color, label=f"{arm}：相对起点，最大 {angle.max():.1f}°，终点 {angle[-1]:.1f}°")
        axes[2].plot(progress, cumulative, color=color, linestyle="--", label=f"{arm}：累计转角 {cumulative[-1]:.1f}°")
        for axis, (i, j) in zip(axes[:2], ((0, 1), (0, 2))):
            axis.scatter(points[[0, -1], i], points[[0, -1], j], color=color, s=18)
    axes[0].set(title="工具路径（俯视）", xlabel="x (m)", ylabel="y (m)", aspect="equal")
    axes[1].set(title="工具路径（侧视）", xlabel="x (m)", ylabel="z (m)", aspect="equal")
    axes[2].set(title="箱体旋转（实线：相对起点；虚线：累计）", xlabel="transport 归一化时间", ylabel="角度 (°)")
    for axis in axes:
        axis.grid(alpha=0.3)
        axis.legend(fontsize=8)
    figure.suptitle(args.case)
    figure.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=130)
    print(args.output)


if __name__ == "__main__":
    main()
