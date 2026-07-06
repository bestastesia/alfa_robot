#!/usr/bin/env python3
"""Visualize IK range grid CSV results with the current robot model in Rerun.

CSV columns expected from alfa_robot_benchmarks ik_range_grid:
  x,y,z,is_success,time_ms,error_code,spin_rad,pos_error,ori_error,...joint columns
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import numpy as np
import rerun as rr

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
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


def parse_bool(value: str) -> bool:
    return str(value).strip().lower() in {"true", "1", "yes", "y"}


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


def find_workspace_root() -> Path:
    candidates: list[Path] = []
    for start in (Path(__file__).resolve(), Path.cwd().resolve()):
        candidates.append(start)
        candidates.extend(start.parents)

    for parent in candidates:
        if (parent / "src" / "alfa_robot_description").exists():
            return parent
        if (parent / "ros2_ws" / "src" / "alfa_robot_description").exists():
            return parent / "ros2_ws"

    fallback = Path.cwd().resolve()
    if fallback.name == "ros2_ws":
        return fallback
    return fallback / "ros2_ws"


def command_with_workspace_setup(command: list[str]) -> list[str]:
    workspace_root = find_workspace_root()
    setup = workspace_root / "install" / "setup.bash"
    if setup.exists():
        quoted = " ".join("'" + str(part).replace("'", "'\\''") + "'" for part in command)
        return ["bash", "-lc", f"source '{setup}' && {quoted}"]
    return command


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
        with open(tmp_path, "w") as output:
            subprocess.run(
                command_with_workspace_setup(["xacro", str(xacro_path)]),
                check=True,
                stdout=output,
                text=True,
            )
        return Path(tmp_path).read_text()
    finally:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass


def log_robot_static_model(robot: UrdfRobot, robot_path: str, *, log_meshes: bool = True) -> None:
    missing_meshes = 0
    logged_meshes = 0
    for link in robot.links.values():
        link_path = f"{robot_path}/{link.name}"
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


def log_robot_state(robot: UrdfRobot, joint_positions: dict[str, float], robot_path: str, *, static: bool = True) -> dict[str, np.ndarray]:
    transforms = robot.fk(joint_positions)
    for link_name, transform in transforms.items():
        log_transform_matrix(f"{robot_path}/{link_name}", transform, static=static)
    return transforms


def load_points(csv_path: Path, show_failed: bool) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    success_points: list[list[float]] = []
    failed_points: list[list[float]] = []
    success_times: list[float] = []

    with csv_path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        fieldnames = set(reader.fieldnames or [])
        xyz_columns = {"x", "y", "z"}
        missing_xyz = xyz_columns.difference(fieldnames)
        if missing_xyz:
            raise ValueError(f"CSV missing columns: {sorted(missing_xyz)}")

        if "is_success" in fieldnames:
            csv_kind = "range_grid"
        elif {"orient_label", "point_reachable"}.issubset(fieldnames):
            csv_kind = "nine_orient"
        else:
            raise ValueError(
                "CSV must be either ik_range_grid format with 'is_success' "
                "or nine_orient format with 'orient_label' and 'point_reachable'"
            )

        for row in reader:
            if csv_kind == "nine_orient" and row.get("orient_label") != "SUMMARY":
                continue

            point = [float(row["x"]), float(row["y"]), float(row["z"])]
            if csv_kind == "range_grid":
                is_success = parse_bool(row["is_success"])
                time_ms = float(row.get("time_ms") or 0.0)
            else:
                point_reachable = row.get("point_reachable")
                if point_reachable not in (None, ""):
                    is_success = parse_bool(point_reachable)
                else:
                    is_success = row.get("n_success") not in (None, "") and row.get("n_success") == row.get("n_total")
                time_ms = 0.0

            if is_success:
                success_points.append(point)
                success_times.append(time_ms)
            elif show_failed:
                failed_points.append(point)

    return (
        np.asarray(success_points, dtype=np.float32),
        np.asarray(failed_points, dtype=np.float32),
        np.asarray(success_times, dtype=np.float32),
    )


def downsample(points: np.ndarray, max_points: int | None) -> np.ndarray:
    if max_points is None or len(points) <= max_points:
        return points
    indices = np.linspace(0, len(points) - 1, max_points).astype(int)
    return points[indices]


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return points
    homogeneous = np.concatenate([points.astype(np.float64), np.ones((len(points), 1))], axis=1)
    return (homogeneous @ transform.T)[:, :3].astype(np.float32)


def log_axes(path: str, size: float, *, static: bool = True) -> None:
    rr.log(
        path,
        rr.Arrows3D(
            vectors=[[size, 0.0, 0.0], [0.0, size, 0.0], [0.0, 0.0, size]],
            origins=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            colors=[[255, 0, 0], [0, 220, 0], [0, 80, 255]],
        ),
        static=static,
    )


POINT_COLORS = [
    [230, 25, 75],    # red
    [60, 180, 75],    # green
    [0, 130, 200],    # blue
    [245, 130, 48],   # orange
    [145, 30, 180],   # purple
    [70, 240, 240],   # cyan
    [240, 50, 230],   # magenta
    [210, 245, 60],   # lime
]


def color_for_index(index: int) -> list[int]:
    return POINT_COLORS[index % len(POINT_COLORS)]


def safe_rerun_name(path: Path) -> str:
    return path.stem.replace("-", "_").replace(".", "_")


def edge_filter_from_name(path: Path) -> tuple[int, str] | None:
    parts = path.stem.split("_")
    if len(parts) != 2:
        return None
    axis_name, direction = parts
    axis_index = {"x": 0, "y": 1, "z": 2}.get(axis_name)
    if axis_index is None or direction not in {"max", "min"}:
        return None
    return axis_index, direction


def filter_edge_slice(points: np.ndarray, edge_filter: tuple[int, str] | None, tolerance: float) -> np.ndarray:
    if edge_filter is None or len(points) == 0:
        return points
    axis_index, direction = edge_filter
    values = points[:, axis_index]
    boundary = values.max() if direction == "max" else values.min()
    return points[np.abs(values - boundary) <= tolerance]


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize IK range grid CSV with current robot model in Rerun")
    parser.add_argument("csv", type=Path, nargs="+", help="One or more IK CSV paths")
    parser.add_argument("--show-failed", action="store_true", help="Also show failed points")
    parser.add_argument("--max-points", type=int, default=None, help="Downsample each point cloud for display")
    parser.add_argument("--csv-frame", default="left_arm_base", help="Frame of x/y/z columns in CSV")
    parser.add_argument("--robot-path", default="world/robot", help="Rerun path for the robot model")
    parser.add_argument("--no-robot", action="store_true", help="Only show point clouds")
    parser.add_argument("--no-meshes", action="store_true", help="Show robot link frames but skip STL meshes")
    parser.add_argument("--raw-csv-frame", action="store_true", help="Do not transform CSV points into world")
    parser.add_argument("--connect", action="store_true", help="Connect to an existing Rerun viewer")
    parser.add_argument("--save", type=Path, default=None, help="Save recording as .rrd")
    parser.add_argument("--edge-slice", action=argparse.BooleanOptionalAction, default=True, help="For files named x_max/x_min/y_max/y_min/z_max/z_min, show only the outermost slice")
    parser.add_argument("--edge-slice-tolerance", type=float, default=1e-9, help="Tolerance for selecting outermost edge slice")
    args = parser.parse_args()

    point_sets = []
    for csv_path in args.csv:
        success_points, failed_points, success_times = load_points(csv_path, args.show_failed)
        edge_filter = edge_filter_from_name(csv_path) if args.edge_slice else None
        success_points = filter_edge_slice(success_points, edge_filter, args.edge_slice_tolerance)
        failed_points = filter_edge_slice(failed_points, edge_filter, args.edge_slice_tolerance)
        point_sets.append({
            "path": csv_path,
            "name": safe_rerun_name(csv_path),
            "edge_filter": edge_filter,
            "success_points": downsample(success_points, args.max_points),
            "failed_points": downsample(failed_points, args.max_points),
            "success_times": success_times,
        })

    recording_id = args.csv[0].stem if len(args.csv) == 1 else "multi_ik_range"
    rr.init("alfa_ik_range_grid", recording_id=recording_id)
    if args.save is not None:
        rr.save(str(args.save))
    elif args.connect:
        rr.connect()
    else:
        rr.spawn()

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    log_axes("world/axes", 0.45)

    csv_to_world = np.eye(4)
    robot = None
    if not args.no_robot:
        urdf_text = render_current_urdf()
        robot = UrdfRobot(urdf_text)
        log_robot_static_model(robot, args.robot_path, log_meshes=not args.no_meshes)
        transforms = log_robot_state(robot, {}, args.robot_path, static=True)
        if args.csv_frame not in transforms:
            raise ValueError(f"CSV frame '{args.csv_frame}' not found in URDF links")
        csv_to_world = transforms[args.csv_frame]
        log_axes(f"{args.robot_path}/{args.csv_frame}/csv_frame_axes", 0.25)

    if args.raw_csv_frame:
        csv_to_world = np.eye(4)

    total_success = 0
    total_failed = 0
    info_lines = [
        "IK CSV visualization",
        f"CSV frame: {args.csv_frame}",
        "Robot front = +X, left = +Y, up = +Z. Points are transformed into world unless --raw-csv-frame is set.",
        "",
    ]

    print(f"CSV files: {len(point_sets)}")
    print(f"CSV frame: {args.csv_frame}")
    if not args.raw_csv_frame:
        xyz = csv_to_world[:3, 3]
        print(f"{args.csv_frame} origin in world: ({xyz[0]:+.4f}, {xyz[1]:+.4f}, {xyz[2]:+.4f})")

    for index, point_set in enumerate(point_sets):
        csv_path = point_set["path"]
        name = point_set["name"]
        color = color_for_index(index)
        success_world = transform_points(point_set["success_points"], csv_to_world)
        failed_world = transform_points(point_set["failed_points"], csv_to_world)
        total_success += len(success_world)
        total_failed += len(failed_world)

        edge_text = ""
        edge_filter = point_set["edge_filter"]
        if edge_filter is not None:
            axis_name = "xyz"[edge_filter[0]]
            edge_text = f" edge_slice={axis_name}_{edge_filter[1]}"
        print(f"[{index + 1}] {csv_path}: success={len(success_world)} failed={len(failed_world) if args.show_failed else 0} color={color}{edge_text}")
        info_lines.append(f"{name}: success={len(success_world)}, failed={len(failed_world) if args.show_failed else 0}, color={color}, {edge_text}, file={csv_path}")

        success_times = point_set["success_times"]
        if len(success_times) > 0:
            print(
                f"    success solve time ms: avg={success_times.mean():.3f}, "
                f"min={success_times.min():.3f}, max={success_times.max():.3f}"
            )

        if len(success_world) > 0:
            rr.log(
                f"world/ik_range/{name}/success",
                rr.Points3D(success_world, colors=color, radii=0.01),
            )
        if args.show_failed and len(failed_world) > 0:
            rr.log(
                f"world/ik_range/{name}/failed",
                rr.Points3D(failed_world, colors=[120, 120, 120], radii=0.004),
            )

    rr.log(
        "world/info",
        rr.TextDocument(
            "\n".join(info_lines)
            + f"\n\ntotal success: {total_success}\n"
            + f"total failed displayed: {total_failed if args.show_failed else 0}"
        ),
    )


if __name__ == "__main__":
    main()
