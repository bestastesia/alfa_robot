#!/usr/bin/python3
"""Generate and optionally load a parametric ALFA v5 URDF.

This tool is intentionally independent from the ROS2 description package. It is
for offline Pinocchio/MeshCat experiments and should not be used as the source
of the deployed robot model until mechanical and motion-control engineers agree.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml
from jinja2 import Environment, FileSystemLoader, StrictUndefined

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "configs" / "v5_baseline_stub.yaml"
DEFAULT_TEMPLATE_DIR = ROOT / "templates"
DEFAULT_OUTPUT = ROOT / "generated" / "alfa_v5_parametric.urdf"


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open() as stream:
        return yaml.safe_load(stream)


def xyz(values) -> str:
    return " ".join(f"{float(value):.9g}" for value in values)


def mirror_values(values, mirror_y: float) -> list[float]:
    return [float(value) * (mirror_y if index in (1,) else 1.0) for index, value in enumerate(values)]


def joint_xyz(joint: dict[str, Any], mirror_y: float) -> str:
    if "xyz_mirror" in joint:
        return xyz(mirror_values(joint["xyz_mirror"], mirror_y))
    return xyz(joint["xyz"])


def joint_rpy(joint: dict[str, Any], mirror_y: float) -> str:
    if "rpy_mirror" in joint:
        return xyz([float(value) * mirror_y if index in (0, 2) else float(value) for index, value in enumerate(joint["rpy_mirror"])])
    return xyz(joint.get("rpy", [0.0, 0.0, 0.0]))


def inertial_xml(link: dict[str, Any], mirror_y: float, indent: int = 0) -> str:
    space = " " * indent
    mass = float(link["mass"])
    com = mirror_values(link.get("com", [0.0, 0.0, 0.0]), mirror_y)
    inertia_diag = [max(float(value), 1e-8) for value in link.get("inertia_diag", [0.001, 0.001, 0.001])]
    return "\n".join([
        f"{space}<inertial>",
        f"{space}  <origin xyz=\"{xyz(com)}\" rpy=\"0 0 0\"/>",
        f"{space}  <mass value=\"{mass:.9g}\"/>",
        f"{space}  <inertia ixx=\"{inertia_diag[0]:.9g}\" ixy=\"0\" ixz=\"0\" iyy=\"{inertia_diag[1]:.9g}\" iyz=\"0\" izz=\"{inertia_diag[2]:.9g}\"/>",
        f"{space}</inertial>",
    ])


def render_urdf(config: dict[str, Any], template_dir: Path) -> str:
    env = Environment(
        loader=FileSystemLoader(str(template_dir)),
        undefined=StrictUndefined,
        trim_blocks=True,
        lstrip_blocks=True,
    )
    env.filters["join"] = lambda values, separator=" ": separator.join(str(value) for value in values)
    env.globals.update(
        xyz=xyz,
        joint_xyz=joint_xyz,
        joint_rpy=joint_rpy,
        inertial_xml=inertial_xml,
        range=range,
    )
    template = env.get_template("alfa_v5_parametric.urdf.j2")
    return template.render(**config)


def validate_urdf_xml(path: Path) -> None:
    tree = ET.parse(path)
    root = tree.getroot()
    joints = root.findall("joint")
    links = root.findall("link")
    if len(links) < 2 or len(joints) < 1:
        raise RuntimeError(f"URDF looks incomplete: links={len(links)} joints={len(joints)}")
    revolute = [joint for joint in joints if joint.get("type") == "revolute"]
    if len(revolute) != 12:
        raise RuntimeError(f"Expected 12 revolute joints for dual v5 arms, got {len(revolute)}")


def optional_pinocchio_check(path: Path, visualize: bool) -> int:
    if importlib.util.find_spec("pinocchio") is None:
        print("[WARN] pinocchio is not installed; skipped model load check.")
        return 0

    import pinocchio as pin

    if not hasattr(pin, "buildModelFromUrdf"):
        module_path = getattr(pin, "__file__", "unknown")
        print(
            "[WARN] imported module 'pinocchio' does not provide "
            f"buildModelFromUrdf (module={module_path}); skipped model load check."
        )
        print("[WARN] Install the robotics Pinocchio package if you want FK/MeshCat checks.")
        return 0

    model = pin.buildModelFromUrdf(str(path))
    data = model.createData()
    q = pin.neutral(model)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    print(f"[OK] Pinocchio loaded: nq={model.nq} nv={model.nv} njoints={model.njoints} nframes={model.nframes}")

    for frame_name in ("left_v5_tool0", "right_v5_tool0"):
        if not model.existFrame(frame_name):
            raise RuntimeError(f"Pinocchio frame missing: {frame_name}")
        frame_id = model.getFrameId(frame_name)
        translation = data.oMf[frame_id].translation
        print(f"[OK] {frame_name} neutral xyz = {translation.T}")

    if visualize:
        if importlib.util.find_spec("meshcat") is None:
            print("[WARN] meshcat is not installed; skipped visualization.")
            return 0
        from pinocchio.visualize import MeshcatVisualizer

        visual_model = pin.buildGeomFromUrdf(model, str(path), pin.GeometryType.VISUAL)
        collision_model = pin.buildGeomFromUrdf(model, str(path), pin.GeometryType.COLLISION)
        viewer = MeshcatVisualizer(model, collision_model, visual_model)
        viewer.initViewer(open=True)
        viewer.loadViewerModel(rootNodeName="alfa_v5_parametric")
        viewer.display(q)
        print("[OK] MeshCat viewer opened. Keep this process alive if you want to inspect it.")
        try:
            input("Press Enter to exit MeshCat demo...")
        except EOFError:
            pass
    return 0


def write_mechanical_handoff(config: dict[str, Any], path: Path) -> None:
    lines = [
        "# T-0016 给机械工程师的 baseline 校准交接",
        "",
        "此文件由 `tools/generate_model.py` 自动生成，说明参数化模型骨架目前用了哪些默认值。",
        "T-0015 机械工程师请用真实机械来源校准这些字段。",
        "",
        "## 当前骨架已固定的接口语义",
        "",
        "- 双臂前缀：`left_v5` / `right_v5`",
        "- 每臂关节：`*_joint1` ... `*_joint6`，均为 revolute",
        "- 每臂末端 frame：`*_tool0`",
        "- 默认左右镜像：右臂对 Y 方向镜像，joint/link 名称不变",
        "- 工具目录不改 ROS2 主 URDF，只生成临时 URDF",
        "",
        "## 需要机械工程师校准的字段",
        "",
        "| 字段 | 当前来源/默认 | 机械需确认 |",
        "| --- | --- | --- |",
        f"| `mount.left_xyz/right_xyz` | {config['mount']['left_xyz']} / {config['mount']['right_xyz']} | 左右安装基准点和坐标系 |",
    ]
    for joint_name, joint in config["kinematics"].items():
        if not joint_name.startswith("joint"):
            continue
        origin = joint.get("xyz", joint.get("xyz_mirror"))
        rpy = joint.get("rpy", joint.get("rpy_mirror", [0.0, 0.0, 0.0]))
        lines.append(f"| `{joint_name}` origin/rpy/axis | xyz={origin}, rpy={rpy}, axis={joint['axis']} | 法兰 offset、关节轴方向、左右镜像规则 |")
    for link_name, link in config["links"].items():
        lines.append(f"| `{link_name}` inertial | mass={link['mass']}, com={link['com']}, inertia_diag={link['inertia_diag']} | 真实质量、COM、惯量张量 |")
    lines.extend([
        "",
        "## 验证命令",
        "",
        "```bash",
        "cd /mnt/mydisk/ALFA/alfa_robot",
        "python3 simulation/pinocchio_parametric/tools/generate_model.py --check",
        "# 安装 pinocchio/meshcat 后可追加：--visualize",
        "```",
        "",
        "## 机械侧输出建议",
        "",
        "1. 标明每个参数来自当前 URDF、SolidWorks、手动测量还是估算。",
        "2. 大臂/小臂长度请对应到具体 joint origin 字段。",
        "3. T/I 电机法兰 offset 请对应到 joint2..joint6 的 origin/rpy/axis。",
        "4. 如果左右不是严格 Y 镜像，请明确指出差异并更新 `mirror_y` 规则。",
    ])
    path.write_text("\n".join(lines) + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--template-dir", type=Path, default=DEFAULT_TEMPLATE_DIR)
    parser.add_argument("--check", action="store_true", help="Validate XML and optionally load with Pinocchio if installed.")
    parser.add_argument("--visualize", action="store_true", help="Open MeshCat viewer if pinocchio and meshcat are installed.")
    parser.add_argument("--handoff", type=Path, default=ROOT / "MECHANICAL_HANDOFF.md")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_yaml(args.config)
    urdf = render_urdf(config, args.template_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(urdf)
    print(f"[OK] wrote URDF: {args.output}")
    write_mechanical_handoff(config, args.handoff)
    print(f"[OK] wrote mechanical handoff: {args.handoff}")
    if args.check or args.visualize:
        validate_urdf_xml(args.output)
        print("[OK] XML sanity check passed")
        return optional_pinocchio_check(args.output, args.visualize)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
