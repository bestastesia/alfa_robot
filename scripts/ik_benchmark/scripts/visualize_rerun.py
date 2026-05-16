#!/usr/bin/env python3
"""
IK Benchmark 3D 可视化回放 — Rerun.io

读取 ik_benchmark 输出的 JSONL 文件，利用 Rerun 时间轴逐帧回放：
  - 每个样本显示: FK 原始末端、扰动后的目标位姿、IK 求解后的实际末端
  - 双臂: 左臂/右臂分别显示
  - 拖动时间轴即可回放

用法:
  python3 scripts/ik_benchmark/scripts/visualize_rerun.py /tmp/ik_benchmark.jsonl
  python3 scripts/ik_benchmark/scripts/visualize_rerun.py result.jsonl --connect  # 连接已有 viewer
"""

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import numpy as np
import rerun as rr

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # 允许在未 source ROS 环境时通过相对路径兜底
    get_package_share_directory = None


@dataclass
class VisualAsset:
    path: str
    origin: np.ndarray = field(default_factory=lambda: np.eye(4))
    scale: np.ndarray = field(default_factory=lambda: np.ones(3))


@dataclass
class LinkModel:
    name: str
    visuals: list[VisualAsset] = field(default_factory=list)


@dataclass
class JointModel:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray


def pose_from_json(p: dict) -> tuple[np.ndarray, np.ndarray]:
    """从 JSON pose 解析出 (position, quaternion_xyzw)"""
    pos = np.array(p["position"], dtype=np.float64)
    quat = np.array(p["orientation"], dtype=np.float64)  # [x, y, z, w]
    return pos, quat


def parse_xyz(text: str | None, default: Iterable[float]) -> np.ndarray:
    if not text:
        return np.array(list(default), dtype=float)
    return np.array([float(value) for value in text.split()], dtype=float)


def rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def transform_from_origin(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if element is None:
        return transform
    xyz = parse_xyz(element.get("xyz"), [0.0, 0.0, 0.0])
    rpy = parse_xyz(element.get("rpy"), [0.0, 0.0, 0.0])
    transform[:3, :3] = rpy_to_matrix(rpy)
    transform[:3, 3] = xyz
    return transform


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        return np.eye(3)
    x, y, z = axis / norm
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=float,
    )


def joint_motion(joint: JointModel, value: float) -> np.ndarray:
    motion = np.eye(4)
    if joint.joint_type in {"revolute", "continuous"}:
        motion[:3, :3] = axis_angle_to_matrix(joint.axis, value)
    elif joint.joint_type == "prismatic":
        motion[:3, 3] = joint.axis * value
    return motion


def matrix_to_quaternion_xyzw(matrix: np.ndarray) -> list[float]:
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
    return [float(x), float(y), float(z), float(w)]


def log_transform_matrix(path: str, transform: np.ndarray, *, static: bool = False, scale: np.ndarray | None = None) -> None:
    rr.log(
        path,
        rr.Transform3D(
            translation=transform[:3, 3].tolist(),
            quaternion=matrix_to_quaternion_xyzw(transform[:3, :3]),
            scale=scale.tolist() if scale is not None else None,
        ),
        static=static,
    )


def package_uri_to_path(uri: str) -> str | None:
    if not uri.startswith("package://"):
        return uri
    package_and_path = uri[len("package://") :]
    if "/" not in package_and_path:
        return None
    package, relative_path = package_and_path.split("/", 1)
    if get_package_share_directory is not None:
        try:
            return str(Path(get_package_share_directory(package)) / relative_path)
        except Exception:
            pass

    workspace_root = find_workspace_root()
    source_guess = workspace_root / "src" / package / relative_path
    if source_guess.exists():
        return str(source_guess)
    install_guess = workspace_root / "install" / package / "share" / package / relative_path
    if install_guess.exists():
        return str(install_guess)
    return None


def find_workspace_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "src" / "alfa_robot_description").exists():
            return parent
    return Path.cwd() / "ros2_ws"


def command_with_workspace_setup(command: list[str]) -> list[str]:
    workspace_root = find_workspace_root()
    setup = workspace_root / "install" / "setup.bash"
    if setup.exists():
        quoted = " ".join(str(part).replace("'", "'\\''") for part in command)
        return ["bash", "-lc", f"source '{setup}' && {quoted}"]
    return command


class UrdfRobot:
    def __init__(self, urdf_text: str) -> None:
        self.links: dict[str, LinkModel] = {}
        self.joints: dict[str, JointModel] = {}
        self.children_by_parent: dict[str, list[JointModel]] = {}
        self.root_link = ""
        self._parse(urdf_text)

    def _parse(self, urdf_text: str) -> None:
        root = ET.fromstring(urdf_text)
        for link_element in root.findall("link"):
            name = link_element.get("name")
            if not name:
                continue
            link = LinkModel(name=name)
            for visual_element in link_element.findall("visual"):
                geometry = visual_element.find("geometry")
                mesh = geometry.find("mesh") if geometry is not None else None
                if mesh is None or not mesh.get("filename"):
                    continue
                link.visuals.append(
                    VisualAsset(
                        path=mesh.get("filename", ""),
                        origin=transform_from_origin(visual_element.find("origin")),
                        scale=parse_xyz(mesh.get("scale"), [1.0, 1.0, 1.0]),
                    )
                )
            self.links[name] = link

        child_links: set[str] = set()
        for joint_element in root.findall("joint"):
            name = joint_element.get("name")
            joint_type = joint_element.get("type", "fixed")
            parent_element = joint_element.find("parent")
            child_element = joint_element.find("child")
            if not name or parent_element is None or child_element is None:
                continue
            parent = parent_element.get("link", "")
            child = child_element.get("link", "")
            axis_element = joint_element.find("axis")
            joint = JointModel(
                name=name,
                joint_type=joint_type,
                parent=parent,
                child=child,
                origin=transform_from_origin(joint_element.find("origin")),
                axis=parse_xyz(axis_element.get("xyz") if axis_element is not None else None, [1.0, 0.0, 0.0]),
            )
            self.joints[name] = joint
            self.children_by_parent.setdefault(parent, []).append(joint)
            child_links.add(child)

        roots = [name for name in self.links if name not in child_links]
        self.root_link = roots[0] if roots else next(iter(self.links), "world")

    def fk(self, joint_positions: dict[str, float]) -> dict[str, np.ndarray]:
        transforms: dict[str, np.ndarray] = {self.root_link: np.eye(4)}
        stack = [self.root_link]
        while stack:
            parent = stack.pop()
            parent_tf = transforms[parent]
            for joint in self.children_by_parent.get(parent, []):
                value = joint_positions.get(joint.name, 0.0)
                child_tf = parent_tf @ joint.origin @ joint_motion(joint, value)
                transforms[joint.child] = child_tf
                stack.append(joint.child)
        return transforms


def render_current_urdf() -> str:
    if get_package_share_directory is not None:
        try:
            xacro_path = Path(get_package_share_directory("alfa_robot_description")) / "urdf" / "alfa_robot.urdf.xacro"
        except Exception:
            xacro_path = None
    else:
        xacro_path = None

    if xacro_path is None or not xacro_path.exists():
        workspace_root = find_workspace_root()
        xacro_path = workspace_root / "src" / "alfa_robot_description" / "urdf" / "alfa_robot.urdf.xacro"

    with tempfile.NamedTemporaryFile("w+", suffix=".urdf", delete=False) as tmp:
        tmp_path = tmp.name
    try:
        subprocess.run(
            command_with_workspace_setup(["xacro", str(xacro_path)]),
            check=True,
            stdout=open(tmp_path, "w"),
            text=True,
        )
        return Path(tmp_path).read_text()
    finally:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass


def log_robot_static_model(robot: UrdfRobot, world_path: str, *, log_meshes: bool = True) -> None:
    rr.log(world_path, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    missing_meshes = 0
    logged_meshes = 0
    for link in robot.links.values():
        link_path = f"{world_path}/{link.name}"
        for index, visual in enumerate(link.visuals):
            mesh_path = package_uri_to_path(visual.path)
            if not mesh_path or not Path(mesh_path).exists():
                missing_meshes += 1
                continue
            visual_path = f"{link_path}/visual_{index}"
            log_transform_matrix(visual_path, visual.origin, static=True, scale=visual.scale)
            if log_meshes:
                rr.log(visual_path, rr.Asset3D(path=mesh_path), static=True)
                logged_meshes += 1
    if missing_meshes:
        print(f"Warning: {missing_meshes} robot visual meshes were not found.")
    print(f"Robot model: root={robot.root_link}, links={len(robot.links)}, meshes={logged_meshes}")


def result_joint_positions(record: dict) -> dict[str, float] | None:
    result = record.get("result", {})
    names = result.get("joint_names", [])
    values = result.get("joint_values", [])
    if not names or len(names) != len(values):
        return None
    return {name: float(value) for name, value in zip(names, values)}


def log_robot_state(robot: UrdfRobot, joint_positions: dict[str, float], world_path: str) -> None:
    transforms = robot.fk(joint_positions)
    for link_name, transform in transforms.items():
        log_transform_matrix(f"{world_path}/{link_name}", transform)


def log_pose(entity_path: str, pos: np.ndarray, quat_xyzw: np.ndarray, label: str, color=None):
    """向 Rerun 记录一个位姿（箭头 + 点）"""
    # Rerun 用 [w, x, y, z] 格式
    quat_wxyz = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]

    rr.log(
        entity_path + "/arrow",
        rr.Arrows3D(
            vectors=[0, 0, 0.05],  # 小箭头表示朝向
            origins=pos,
            colors=color or [200, 200, 200],
        ),
    )
    rr.log(
        entity_path + "/point",
        rr.Points3D(
            positions=[pos],
            colors=color or [200, 200, 200],
            radii=0.008,
        ),
    )
    rr.log(
        entity_path,
        rr.TextLog(label),
    )


def log_transform(entity_path: str, pos: np.ndarray, quat_xyzw: np.ndarray):
    """记录一个 rigid transform，Rerun 用这个做坐标系显示"""
    quat_wxyz = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
    rr.log(
        entity_path,
        rr.Transform3D(
            translation=pos,
            rotation=rr.Quaternion(xyzw=quat_xyzw.tolist()),
        ),
    )


def set_sample_time(sample: int):
    """兼容 Rerun 新旧 SDK 的 sequence 时间轴 API。"""
    if hasattr(rr, "set_time_sequence"):
        rr.set_time_sequence("sample", sample)
    else:
        rr.set_time("sample", sequence=sample)


def main():
    parser = argparse.ArgumentParser(description="IK Benchmark Rerun 可视化")
    parser.add_argument("jsonl_path", type=str, help="ik_benchmark 输出的 JSONL 文件")
    parser.add_argument("--connect", action="store_true", help="连接已有 Rerun viewer")
    parser.add_argument("--save", type=str, default="", help="保存为 .rrd 文件")
    parser.add_argument("--no-robot", action="store_true", help="只显示末端点位，不加载机器人模型")
    parser.add_argument("--robot-path", default="robot", help="Rerun 中机器人模型根路径")
    parser.add_argument("--no-meshes", action="store_true", help="只记录 link 坐标变换，不加载 STL mesh")
    args = parser.parse_args()

    # 读取 JSONL
    records = []
    with open(args.jsonl_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if rec.get("header"):
                continue
            records.append(rec)

    if not records:
        print("No records found in", args.jsonl_path)
        return

    is_dual = "target_pose2" in records[0]
    print(f"Loaded {len(records)} samples, dual_arm={is_dual}")

    # 初始化 Rerun
    if args.save:
        rr.init("ik_benchmark", recording_id=Path(args.jsonl_path).stem)
        rr.save(args.save)
    elif args.connect:
        rr.init("ik_benchmark", recording_id=Path(args.jsonl_path).stem)
        rr.connect()
    else:
        rr.init("ik_benchmark", recording_id=Path(args.jsonl_path).stem)
        rr.spawn()

    robot = None
    if not args.no_robot:
        urdf_text = render_current_urdf()
        robot = UrdfRobot(urdf_text)
        log_robot_static_model(robot, args.robot_path, log_meshes=not args.no_meshes)

    # 设置时间轴
    set_sample_time(0)

    # 颜色定义
    COLOR_FK    = [0, 200, 100]    # 绿色: FK 原始位姿
    COLOR_TARGET = [255, 165, 0]   # 橙色: 扰动后目标位姿
    COLOR_ACTUAL = [50, 100, 255]  # 蓝色: IK 求解后实际位姿
    COLOR_FAIL   = [255, 50, 50]   # 红色: 求解失败

    for i, rec in enumerate(records):
        set_sample_time(i)

        success = rec["result"]["success"] if "result" in rec else rec.get("success", False)
        solver = rec.get("solver", "")
        pos_err = rec.get("result", rec).get("pos_error", 0.0)
        ori_err = rec.get("result", rec).get("ori_error", 0.0)
        solve_ms = rec.get("result", rec).get("solve_ms", 0.0)

        # 状态文字
        status = "OK" if success else "FAIL"
        info = f"{solver} | {status} | {solve_ms:.1f}ms"
        if success:
            info += f" | pos_err={pos_err:.6f}m ori_err={ori_err:.6f}rad"
        else:
            collision_pairs = rec.get("result", {}).get("collision_pairs", [])
            if collision_pairs:
                info += " | collision: " + ", ".join(collision_pairs[:5])
                if len(collision_pairs) > 5:
                    info += f" ... +{len(collision_pairs) - 5}"
        rr.log("info", rr.TextLog(info))

        if robot is not None:
            joint_positions = result_joint_positions(rec)
            if joint_positions is not None:
                log_robot_state(robot, joint_positions, args.robot_path)

        # ── 左臂 ──
        if "fk_pose" in rec:
            fk_pos, fk_quat = pose_from_json(rec["fk_pose"])
            log_pose("left/fk", fk_pos, fk_quat, "FK", COLOR_FK)

        if "target_pose" in rec:
            tgt_pos, tgt_quat = pose_from_json(rec["target_pose"])
            log_pose("left/target", tgt_pos, tgt_quat, "Target", COLOR_TARGET)

        if success and "actual_pose" in rec:
            act_pos, act_quat = pose_from_json(rec["actual_pose"])
            log_pose("left/actual", act_pos, act_quat, "Actual", COLOR_ACTUAL)
        else:
            # 求解失败: 用 target 位置标红
            if "target_pose" in rec:
                tgt_pos, tgt_quat = pose_from_json(rec["target_pose"])
                log_pose("left/actual", tgt_pos, tgt_quat, "FAILED", COLOR_FAIL)

        # ── 右臂（双臂模式）──
        if is_dual:
            if "fk_pose2" in rec:
                fk2_pos, fk2_quat = pose_from_json(rec["fk_pose2"])
                log_pose("right/fk", fk2_pos, fk2_quat, "FK", COLOR_FK)

            if "target_pose2" in rec:
                tgt2_pos, tgt2_quat = pose_from_json(rec["target_pose2"])
                log_pose("right/target", tgt2_pos, tgt2_quat, "Target", COLOR_TARGET)

            if success and "actual_pose2" in rec:
                act2_pos, act2_quat = pose_from_json(rec["actual_pose2"])
                log_pose("right/actual", act2_pos, act2_quat, "Actual", COLOR_ACTUAL)
            else:
                if "target_pose2" in rec:
                    tgt2_pos, tgt2_quat = pose_from_json(rec["target_pose2"])
                    log_pose("right/actual", tgt2_pos, tgt2_quat, "FAILED", COLOR_FAIL)

        # ── 连线: target → actual（误差可视化）──
        if success and "target_pose" in rec and "actual_pose" in rec:
            tgt_pos = np.array(rec["target_pose"]["position"])
            act_pos = np.array(rec["actual_pose"]["position"])
            rr.log(
                "left/error_line",
                rr.LineStrips3D(
                    strips=[[tgt_pos.tolist(), act_pos.tolist()]],
                    colors=[COLOR_ACTUAL],
                ),
            )
            if is_dual and "target_pose2" in rec and "actual_pose2" in rec:
                tgt2_pos = np.array(rec["target_pose2"]["position"])
                act2_pos = np.array(rec["actual_pose2"]["position"])
                rr.log(
                    "right/error_line",
                    rr.LineStrips3D(
                        strips=[[tgt2_pos.tolist(), act2_pos.tolist()]],
                        colors=[COLOR_ACTUAL],
                    ),
                )

    print(f"Done. {len(records)} samples logged to Rerun.")


if __name__ == "__main__":
    main()
