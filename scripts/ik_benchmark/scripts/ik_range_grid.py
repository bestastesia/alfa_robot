#!/usr/bin/env python3
"""Run fast in-process MoveIt IK range grid and optionally visualize CSV.

This wraps the C++ `ik_range_grid` executable from alfa_robot_benchmarks.
It is intended for fixed-forward-axis reachability: the selected end-effector
axis is constrained to a fixed base-frame direction, while spin around that
axis is sampled and left free.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path


SOLVER_SHORTCUTS = {
    "kdl": "kdl_kinematics_plugin/KDLKinematicsPlugin",
    "bio_ik": "bio_ik/BioIKKinematicsPlugin",
}


def package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def project_root() -> Path:
    return Path(__file__).resolve().parents[3]


def version_model_dir(version: str) -> Path:
    return package_root() / "models" / "urdf_versions" / version


def default_urdf(version: str) -> Path | None:
    if version == "current":
        return None
    path = version_model_dir(version) / "alfa_robot.urdf"
    if not path.exists():
        raise SystemExit(f"URDF for {version} not found: {path}")
    return path


def default_srdf(version: str) -> Path:
    if version != "current":
        path = version_model_dir(version) / "alfa_robot.srdf"
        if path.exists():
            return path
    path = project_root() / "ros2_ws" / "src" / "alfa_robot_moveit_config" / "config" / "alfa_robot.srdf"
    if not path.exists():
        raise SystemExit(f"SRDF not found: {path}")
    return path


def default_tip_link(version: str, requested: str | None) -> str:
    if requested:
        return requested
    if version in {"v2", "v3"}:
        return "left_ee_link"
    return "left_tool0"


def find_executable() -> str:
    local_exe = project_root() / "install" / "alfa_robot_benchmarks" / "lib" / "alfa_robot_benchmarks" / "ik_range_grid"
    if local_exe.exists():
        return str(local_exe)

    direct = shutil.which("ik_range_grid")
    if direct:
        return direct
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "alfa_robot_benchmarks"],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        if result.returncode == 0:
            exe = Path(result.stdout.strip()) / "lib" / "alfa_robot_benchmarks" / "ik_range_grid"
            if exe.exists():
                return str(exe)
    except Exception:
        pass
    return "ros2 run alfa_robot_benchmarks ik_range_grid"


def prepend_env_path(env: dict[str, str], name: str, path: Path) -> None:
    if not path.exists():
        return
    old_value = env.get(name, "")
    path_text = str(path)
    parts = [part for part in old_value.split(":") if part]
    if path_text not in parts:
        env[name] = path_text if not old_value else f"{path_text}:{old_value}"


def build_runtime_env() -> dict[str, str]:
    env = os.environ.copy()
    prefixes = [
        project_root() / "install",
        project_root() / "ros2_ws" / "install",
        Path("/opt/ros/humble"),
    ]
    isolated_install = project_root() / "ros2_ws" / "install"
    if isolated_install.exists():
        prefixes.extend(
            sorted(
                child
                for child in isolated_install.iterdir()
                if (child / "share" / "ament_index" / "resource_index" / "packages").exists()
            )
        )
    for prefix in prefixes:
        prepend_env_path(env, "AMENT_PREFIX_PATH", prefix)
        prepend_env_path(env, "CMAKE_PREFIX_PATH", prefix)
        prepend_env_path(env, "PATH", prefix / "bin")
        prepend_env_path(env, "LD_LIBRARY_PATH", prefix / "lib")
        for site_packages in (prefix / "lib").glob("python*/site-packages"):
            prepend_env_path(env, "PYTHONPATH", site_packages)

    local_benchmark_lib = project_root() / "install" / "alfa_robot_benchmarks" / "lib"
    prepend_env_path(env, "LD_LIBRARY_PATH", local_benchmark_lib)
    return env


def add_range(command: list[str], name: str, values: list[float]) -> None:
    command.extend([f"--{name}", *(str(value) for value in values)])


def main() -> None:
    parser = argparse.ArgumentParser(description="Fast IK range grid wrapper")
    parser.add_argument("--version", choices=["current", "v2", "v3", "v4"], default="current")
    parser.add_argument("--group", default="left_arm")
    parser.add_argument("--solver", default="kdl", help="kdl/bio_ik or full plugin")
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument("--srdf", type=Path, default=None)
    parser.add_argument("--base-frame", default="left_arm_base")
    parser.add_argument("--tip-link", default=None)
    parser.add_argument("--x", type=float, nargs=3, metavar=("MIN", "MAX", "STEP"), default=[0.0, 1.2, 0.05])
    parser.add_argument("--y", type=float, nargs=3, metavar=("MIN", "MAX", "STEP"), default=[-0.6, 0.6, 0.05])
    parser.add_argument("--z", type=float, nargs=3, metavar=("MIN", "MAX", "STEP"), default=[-0.5, 0.8, 0.05])
    parser.add_argument("--timeout", type=float, default=0.02)
    parser.add_argument("--forward-axis", default="z", help="local EE axis: x/y/z or -x/-y/-z")
    parser.add_argument("--target-axis", default="x", help="base-frame target direction: x/y/z or -x/-y/-z")
    parser.add_argument("--spin-samples", type=int, default=12)
    parser.add_argument("--spin-min", type=float, default=0.0)
    parser.add_argument("--spin-max", type=float, default=6.283185307179586)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--success-only", action="store_true")
    parser.add_argument("--fix-joint6", action="store_true")
    parser.add_argument("--visualize", choices=["none", "open3d"], default="none")
    parser.add_argument("--show-failed", action="store_true")
    args = parser.parse_args()

    urdf = args.urdf if args.urdf is not None else default_urdf(args.version)
    srdf = args.srdf if args.srdf is not None else default_srdf(args.version)
    tip_link = default_tip_link(args.version, args.tip_link)
    output = args.output or Path(f"/tmp/alfa_ik_range_{args.version}_{args.group}.csv")
    if output.exists():
        output.unlink()

    exe = find_executable()
    command = exe.split() if exe.startswith("ros2 run ") else [exe]
    command.extend([
        "--group", args.group,
        "--solver", SOLVER_SHORTCUTS.get(args.solver, args.solver),
        "--srdf", str(srdf),
        "--base-frame", args.base_frame,
        "--tip-link", tip_link,
        "--timeout", str(args.timeout),
        "--forward-axis", args.forward_axis,
        "--target-axis", args.target_axis,
        "--spin-samples", str(args.spin_samples),
        "--spin-min", str(args.spin_min),
        "--spin-max", str(args.spin_max),
        "--output", str(output),
    ])
    if urdf is not None:
        command.extend(["--urdf", str(urdf)])
    if args.success_only:
        command.append("--success-only")
    if args.fix_joint6:
        command.append("--fix-joint6")
    add_range(command, "x", args.x)
    add_range(command, "y", args.y)
    add_range(command, "z", args.z)

    print("Running:")
    print(" ".join(command))
    env = build_runtime_env()
    subprocess.run(command, check=True, env=env)

    if args.visualize == "open3d":
        visualizer = project_root() / "ros2_ws" / "src" / "alfa_robot_moveit_config" / "scripts" / "ik_csv_open3d_visualizer.py"
        visualize_command = [sys.executable, str(visualizer), str(output)]
        if args.show_failed:
            visualize_command.append("--show-failed")
        subprocess.run(visualize_command, check=True, env=env)


if __name__ == "__main__":
    main()
