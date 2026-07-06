#!/usr/bin/env python3
"""Generate a small ALFA dual-arm Rerun recording for browser interaction.

This is a visualization-only smoke demo: no cuRobo dependency, no collision checking.
It reuses the current ALFA URDF renderer and creates a smooth dual-arm motion.
"""
from __future__ import annotations

import argparse
import importlib.util
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
HELPER_PATH = REPO_ROOT / "scripts/ik_benchmark/scripts/visualize_rerun.py"


def load_helpers():
    spec = importlib.util.spec_from_file_location("alfa_visualize_rerun_helpers", HELPER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load helper: {HELPER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def deg(value: float) -> float:
    return math.radians(value)


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def smoothstep(t: float) -> float:
    return t * t * (3.0 - 2.0 * t)


def interpolate_pose(start: dict[str, float], end: dict[str, float], t: float) -> dict[str, float]:
    s = smoothstep(t)
    keys = set(start) | set(end)
    return {key: lerp(start.get(key, 0.0), end.get(key, 0.0), s) for key in keys}


def log_tool_trace(rr, robot_model, joint_map, sample: int):
    transforms = robot_model.fk(joint_map)
    colors = {
        "left_v5_tool0": [0, 170, 255],
        "right_v5_tool0": [255, 140, 0],
    }
    for tool_name, color in colors.items():
        tf = transforms.get(tool_name)
        if tf is None:
            continue
        rr.log(
            f"world/tool_trace/{tool_name}",
            rr.Points3D(positions=[tf[:3, 3].tolist()], colors=[color], radii=0.012),
        )
        rr.log(
            f"world/tool_axis/{tool_name}",
            rr.Arrows3D(
                origins=[tf[:3, 3].tolist()],
                vectors=[(tf[:3, :3] @ [0.0, 0.0, 0.08]).tolist()],
                colors=[color],
                radii=0.006,
            ),
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--save", default=str(REPO_ROOT / "data/curobo_web_demo/alfa_dual_arm_motion_web.rrd"))
    parser.add_argument("--samples", type=int, default=90)
    parser.add_argument("--no-meshes", action="store_true")
    args = parser.parse_args()

    helpers = load_helpers()
    rr = helpers.rr
    rr.init("alfa_dual_arm_web_demo", spawn=False)

    robot_model = helpers.UrdfRobot(helpers.render_current_urdf())
    helpers.log_robot_static_model(robot_model, "world/robot", log_meshes=not args.no_meshes)
    rr.log(
        "world/demo_note",
        rr.TextLog("ALFA dual-arm browser demo: synthetic smooth motion, not collision/planning validated"),
        static=True,
    )

    start = {
        "pitch": 0.0,
        "turn": 0.0,
        "updown": 0.30,
        "left_v5_joint1": 0.0,
        "left_v5_joint2": deg(-75),
        "left_v5_joint3": deg(135),
        "left_v5_joint4": 0.0,
        "left_v5_joint5": deg(60),
        "left_v5_joint6": 0.0,
        "right_v5_joint1": 0.0,
        "right_v5_joint2": deg(-75),
        "right_v5_joint3": deg(135),
        "right_v5_joint4": 0.0,
        "right_v5_joint5": deg(60),
        "right_v5_joint6": 0.0,
    }
    end = {
        **start,
        "left_v5_joint1": deg(10),
        "left_v5_joint2": deg(-68),
        "left_v5_joint4": deg(18),
        "left_v5_joint6": deg(25),
        "right_v5_joint1": deg(-10),
        "right_v5_joint2": deg(-68),
        "right_v5_joint4": deg(-18),
        "right_v5_joint6": deg(-25),
    }

    # Go start -> end -> start, so browser playback is visibly dynamic.
    total = max(3, args.samples)
    for sample in range(total):
        phase = sample / (total - 1)
        if phase <= 0.5:
            local_t = phase * 2.0
            joint_map = interpolate_pose(start, end, local_t)
        else:
            local_t = (phase - 0.5) * 2.0
            joint_map = interpolate_pose(end, start, local_t)
        helpers.set_sample_time(sample)
        helpers.log_robot_state(robot_model, joint_map, "world/robot")
        log_tool_trace(rr, robot_model, joint_map, sample)

    save_path = Path(args.save)
    save_path.parent.mkdir(parents=True, exist_ok=True)
    rr.save(str(save_path))
    print(f"saved: {save_path}")
    print(f"open native: rerun {save_path}")
    print(f"open web:    rerun {save_path} --web-viewer --web-viewer-port 9090")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
