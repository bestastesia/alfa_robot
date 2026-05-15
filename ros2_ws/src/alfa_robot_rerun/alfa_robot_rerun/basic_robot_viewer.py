#!/usr/bin/env python3
"""Minimal ALFA robot state viewer for Rerun.

This is intentionally small: render the current robot URDF, load link meshes into
Rerun, subscribe to /joint_states, and update link transforms from a simple URDF
forward-kinematics pass.
"""

from __future__ import annotations

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
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import rerun as rr
except ImportError as exc:  # pragma: no cover - exercised on machines without SDK
    raise SystemExit(
        "Python package 'rerun-sdk' is required for alfa_robot_rerun. Install it with:\n"
        "  /usr/bin/python3 -m pip install --user rerun-sdk"
    ) from exc


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


def log_transform(path: str, transform: np.ndarray, *, static: bool = False, scale: np.ndarray | None = None) -> None:
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
    try:
        package_share = Path(get_package_share_directory(package))
    except Exception:
        return None
    return str(package_share / relative_path)


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
            joint = JointModel(
                name=name,
                joint_type=joint_type,
                parent=parent,
                child=child,
                origin=transform_from_origin(joint_element.find("origin")),
                axis=parse_xyz(joint_element.find("axis").get("xyz") if joint_element.find("axis") is not None else None, [1.0, 0.0, 0.0]),
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


class BasicRerunRobotViewer(Node):
    def __init__(self) -> None:
        super().__init__("alfa_rerun_basic_robot_viewer")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("application_id", "alfa_robot_basic_viewer")
        self.declare_parameter("spawn", True)
        self.declare_parameter("recording_path", "")
        self.declare_parameter("world_path", "world")
        self.declare_parameter("log_meshes", True)

        self.world_path = self.get_parameter("world_path").get_parameter_value().string_value
        app_id = self.get_parameter("application_id").get_parameter_value().string_value
        spawn = self.get_parameter("spawn").get_parameter_value().bool_value
        recording_path = self.get_parameter("recording_path").get_parameter_value().string_value

        rr.init(app_id, spawn=spawn)
        if recording_path:
            rr.save(recording_path)
            self.get_logger().info(f"Rerun recording will be saved to: {recording_path}")

        urdf_text = self._get_robot_description()
        self.robot = UrdfRobot(urdf_text)
        self.joint_positions: dict[str, float] = {}
        self.log_static_model()
        self.log_robot_state()

        topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        self.subscription = self.create_subscription(JointState, topic, self.on_joint_state, 10)
        self.get_logger().info(f"Rerun basic robot viewer ready. root={self.robot.root_link}, joint_states={topic}")

    def _get_robot_description(self) -> str:
        text = self.get_parameter("robot_description").get_parameter_value().string_value
        if text:
            return text
        self.get_logger().info("robot_description parameter is empty; rendering alfa_robot_description xacro locally.")
        xacro_path = Path(get_package_share_directory("alfa_robot_description")) / "urdf" / "alfa_robot.urdf.xacro"
        with tempfile.NamedTemporaryFile("w+", suffix=".urdf", delete=False) as tmp:
            tmp_path = tmp.name
        try:
            subprocess.run(["xacro", str(xacro_path)], check=True, stdout=open(tmp_path, "w"), text=True)
            return Path(tmp_path).read_text()
        finally:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass

    def log_static_model(self) -> None:
        rr.log(self.world_path, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        log_meshes = self.get_parameter("log_meshes").get_parameter_value().bool_value
        for link in self.robot.links.values():
            link_path = f"{self.world_path}/{link.name}"
            if not link.visuals:
                continue
            for index, visual in enumerate(link.visuals):
                mesh_path = package_uri_to_path(visual.path)
                if not mesh_path or not Path(mesh_path).exists():
                    self.get_logger().warn(f"Mesh not found for {link.name}: {visual.path}")
                    continue
                visual_path = f"{link_path}/visual_{index}"
                log_transform(visual_path, visual.origin, static=True, scale=visual.scale)
                if log_meshes:
                    rr.log(visual_path, rr.Asset3D(path=mesh_path), static=True)

    def on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = float(position)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0.0:
            rr.set_time("ros_time", timestamp=stamp)
        self.log_robot_state()

    def log_robot_state(self) -> None:
        transforms = self.robot.fk(self.joint_positions)
        for link_name, transform in transforms.items():
            log_transform(f"{self.world_path}/{link_name}", transform)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BasicRerunRobotViewer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
