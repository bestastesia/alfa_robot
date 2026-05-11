#!/usr/bin/env python3
"""Add simple box obstacles to the MoveIt planning scene.

The default scene is a container-like workspace made from thin cuboids:
floor, left/right walls, rear wall, and an optional ceiling.  RViz displays
these as PlanningScene collision objects, and MoveIt plans around them.
"""

import math
from typing import Iterable, List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


BoxSpec = Tuple[str, Sequence[float], Sequence[float], float]


def _parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _parse_vector(text: str, expected_len: int, name: str) -> List[float]:
    values = [float(part.strip()) for part in text.split(",") if part.strip()]
    if len(values) != expected_len:
        raise ValueError(f"{name} expects {expected_len} comma-separated values, got {text!r}")
    return values


def _yaw_to_quaternion(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _make_pose(center_xyz: Sequence[float], yaw: float) -> Pose:
    pose = Pose()
    pose.position.x = float(center_xyz[0])
    pose.position.y = float(center_xyz[1])
    pose.position.z = float(center_xyz[2])
    qx, qy, qz, qw = _yaw_to_quaternion(float(yaw))
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def _make_box_object(
    object_id: str,
    frame_id: str,
    center_xyz: Sequence[float],
    size_xyz: Sequence[float],
    yaw: float,
    operation: int = CollisionObject.ADD,
) -> CollisionObject:
    collision_object = CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = object_id
    collision_object.operation = operation

    if operation == CollisionObject.ADD:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size_xyz[0]), float(size_xyz[1]), float(size_xyz[2])]
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(_make_pose(center_xyz, yaw))

    return collision_object


def _container_specs(
    prefix: str,
    origin_xyz: Sequence[float],
    length: float,
    width: float,
    height: float,
    wall_thickness: float,
    floor_thickness: float,
    yaw: float,
    include_ceiling: bool,
) -> List[BoxSpec]:
    ox, oy, oz = origin_xyz
    specs: List[BoxSpec] = []

    def world_center(local_xyz: Sequence[float]) -> Tuple[float, float, float]:
        lx, ly, lz = local_xyz
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            ox + cos_yaw * lx - sin_yaw * ly,
            oy + sin_yaw * lx + cos_yaw * ly,
            oz + lz,
        )

    specs.append((
        f"{prefix}_floor",
        world_center((0.0, 0.0, -0.5 * floor_thickness)),
        (length, width, floor_thickness),
        yaw,
    ))
    specs.append((
        f"{prefix}_left_wall",
        world_center((0.0, 0.5 * width + 0.5 * wall_thickness, 0.5 * height)),
        (length, wall_thickness, height),
        yaw,
    ))
    specs.append((
        f"{prefix}_right_wall",
        world_center((0.0, -0.5 * width - 0.5 * wall_thickness, 0.5 * height)),
        (length, wall_thickness, height),
        yaw,
    ))
    specs.append((
        f"{prefix}_rear_wall",
        world_center((0.5 * length + 0.5 * wall_thickness, 0.0, 0.5 * height)),
        (wall_thickness, width + 2.0 * wall_thickness, height),
        yaw,
    ))

    if include_ceiling:
        specs.append((
            f"{prefix}_ceiling",
            world_center((0.0, 0.0, height + 0.5 * wall_thickness)),
            (length, width + 2.0 * wall_thickness, wall_thickness),
            yaw,
        ))

    return specs


class ContainerSceneNode(Node):
    """Applies box obstacles to the active MoveIt planning scene."""

    def __init__(self):
        super().__init__("add_container_scene")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("mode", "add")
        self.declare_parameter("scene", "container")
        self.declare_parameter("object_prefix", "container")
        self.declare_parameter("origin_xyz", "1.4,0.0,0.0")
        self.declare_parameter("container_size_xyz", "1.6,1.2,1.2")
        self.declare_parameter("wall_thickness", 0.04)
        self.declare_parameter("floor_thickness", 0.04)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("include_ceiling", False)
        self.declare_parameter("box_id", "box_obstacle")
        self.declare_parameter("box_center_xyz", "1.0,0.0,0.4")
        self.declare_parameter("box_size_xyz", "0.4,0.4,0.8")
        self.declare_parameter("service_name", "/apply_planning_scene")
        self.declare_parameter("timeout_sec", 5.0)

        self._client = self.create_client(
            ApplyPlanningScene,
            self.get_parameter("service_name").value,
        )

    def run(self) -> bool:
        timeout_sec = float(self.get_parameter("timeout_sec").value)
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f"MoveIt service {self._client.srv_name} not available after {timeout_sec:.1f}s"
            )
            return False

        mode = str(self.get_parameter("mode").value).lower()
        if mode not in {"add", "remove", "clear"}:
            self.get_logger().error("mode must be one of: add, remove, clear")
            return False

        objects = self._build_collision_objects(mode)
        if not objects and mode != "clear":
            self.get_logger().error("No collision objects generated")
            return False

        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.world.collision_objects = objects

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.result() is None:
            self.get_logger().error("ApplyPlanningScene request timed out or failed")
            return False
        if not future.result().success:
            self.get_logger().error("MoveIt rejected the planning scene update")
            return False

        if mode == "clear":
            self.get_logger().info("Requested removal of default container scene objects")
        else:
            names = ", ".join(obj.id for obj in objects)
            self.get_logger().info(f"Planning scene update applied: mode={mode}, objects=[{names}]")
        return True

    def _build_collision_objects(self, mode: str) -> List[CollisionObject]:
        frame_id = str(self.get_parameter("frame_id").value)
        scene = str(self.get_parameter("scene").value).lower()
        operation = CollisionObject.ADD if mode == "add" else CollisionObject.REMOVE

        if mode == "clear":
            prefix = str(self.get_parameter("object_prefix").value)
            ids = [
                f"{prefix}_floor",
                f"{prefix}_left_wall",
                f"{prefix}_right_wall",
                f"{prefix}_rear_wall",
                f"{prefix}_ceiling",
                str(self.get_parameter("box_id").value),
            ]
            return [_make_box_object(object_id, frame_id, (0, 0, 0), (1, 1, 1), 0, operation) for object_id in ids]

        if scene == "box":
            return [
                _make_box_object(
                    str(self.get_parameter("box_id").value),
                    frame_id,
                    _parse_vector(str(self.get_parameter("box_center_xyz").value), 3, "box_center_xyz"),
                    _parse_vector(str(self.get_parameter("box_size_xyz").value), 3, "box_size_xyz"),
                    float(self.get_parameter("yaw").value),
                    operation,
                )
            ]

        if scene != "container":
            raise ValueError("scene must be one of: container, box")

        length, width, height = _parse_vector(
            str(self.get_parameter("container_size_xyz").value),
            3,
            "container_size_xyz",
        )
        specs = _container_specs(
            str(self.get_parameter("object_prefix").value),
            _parse_vector(str(self.get_parameter("origin_xyz").value), 3, "origin_xyz"),
            length,
            width,
            height,
            float(self.get_parameter("wall_thickness").value),
            float(self.get_parameter("floor_thickness").value),
            float(self.get_parameter("yaw").value),
            _parse_bool(self.get_parameter("include_ceiling").value),
        )
        return [
            _make_box_object(object_id, frame_id, center_xyz, size_xyz, yaw, operation)
            for object_id, center_xyz, size_xyz, yaw in specs
        ]


def main(args: Iterable[str] = None):
    rclpy.init(args=args)
    node = ContainerSceneNode()
    try:
        ok = node.run()
    except Exception as exc:  # Keep launch errors readable for parameter mistakes.
        node.get_logger().error(str(exc))
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
