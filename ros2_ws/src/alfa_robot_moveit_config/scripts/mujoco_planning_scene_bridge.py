#!/usr/bin/python3
"""Publish MuJoCo scene geometry as MoveIt PlanningScene collision objects.

第一版目标：
- 读取 `simulation/mujoco/scene.xml` 中的 collision box geoms；
- 转成 MoveIt `CollisionObject`，写入 `/apply_planning_scene`；
- 同时发布到 `/collision_object`，便于 RViz/MoveIt 监控；
- 可选添加一个 attached object，用于描述末端已经抓取的箱体。

说明：当前版本是 XML snapshot bridge，不实时读取 MuJoCo 运行时自由箱体位姿。
后续如果需要真实动态箱体，可把 MuJoCo runtime body pose 接到同一套
`build_planning_scene()` 输出路径上。
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET


PROJECT_SCENE_RELATIVE = Path("simulation/mujoco/scene.xml")
DEFAULT_FRAME_ID = "base_link"
DEFAULT_OBJECT_PREFIX = "mj_"


@dataclass(frozen=True)
class Pose3:
    xyz: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True)
class SceneBox:
    object_id: str
    xyz: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    dimensions: tuple[float, float, float]


def parse_vec(text: str | None, size: int, default: Iterable[float] | None = None) -> tuple[float, ...]:
    if text is None:
        if default is None:
            return tuple(0.0 for _ in range(size))
        return tuple(default)
    values = tuple(float(x) for x in text.split())
    if len(values) != size:
        raise ValueError(f"expected {size} values, got {len(values)} in {text!r}")
    return values


def quat_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(v / norm for v in q)  # type: ignore[return-value]


def quat_mul_raw(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_mul(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    return quat_normalize(quat_mul_raw(a, b))


def quat_conj(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_rotate(
    q: tuple[float, float, float, float],
    v: tuple[float, float, float],
) -> tuple[float, float, float]:
    qv = (0.0, v[0], v[1], v[2])
    out = quat_mul_raw(quat_mul_raw(q, qv), quat_conj(q))
    return (out[1], out[2], out[3])


def rpy_to_quat_wxyz(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return quat_normalize((
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ))


def local_quat(element: ET.Element) -> tuple[float, float, float, float]:
    if element.get("quat"):
        return quat_normalize(parse_vec(element.get("quat"), 4, (1.0, 0.0, 0.0, 0.0)))  # type: ignore[arg-type]
    if element.get("euler"):
        roll, pitch, yaw = parse_vec(element.get("euler"), 3)
        return rpy_to_quat_wxyz(roll, pitch, yaw)
    return (1.0, 0.0, 0.0, 0.0)


def compose(parent: Pose3, child: Pose3) -> Pose3:
    rotated = quat_rotate(parent.quat_wxyz, child.xyz)
    xyz = (
        parent.xyz[0] + rotated[0],
        parent.xyz[1] + rotated[1],
        parent.xyz[2] + rotated[2],
    )
    return Pose3(xyz=xyz, quat_wxyz=quat_mul(parent.quat_wxyz, child.quat_wxyz))


def sanitize_object_id(name: str, prefix: str = DEFAULT_OBJECT_PREFIX) -> str:
    clean = re.sub(r"[^A-Za-z0-9_]+", "_", name).strip("_")
    return f"{prefix}{clean}"


def should_import_geom(geom: ET.Element) -> bool:
    if geom.get("type", "sphere") != "box":
        return False
    if int(float(geom.get("contype", "1"))) == 0 and int(float(geom.get("conaffinity", "1"))) == 0:
        return False
    return True


def parse_mujoco_scene(xml_path: Path, object_prefix: str = DEFAULT_OBJECT_PREFIX) -> list[SceneBox]:
    root = ET.parse(xml_path).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError(f"{xml_path} has no <worldbody>")

    boxes: list[SceneBox] = []
    world_pose = Pose3((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))

    def walk_body(body: ET.Element, parent_pose: Pose3):
        body_pose = compose(
            parent_pose,
            Pose3(
                parse_vec(body.get("pos"), 3, (0.0, 0.0, 0.0)),  # type: ignore[arg-type]
                local_quat(body),
            ),
        )
        body_name = body.get("name", "body")

        for geom in body.findall("geom"):
            if not should_import_geom(geom):
                continue
            geom_pose = compose(
                body_pose,
                Pose3(
                    parse_vec(geom.get("pos"), 3, (0.0, 0.0, 0.0)),  # type: ignore[arg-type]
                    local_quat(geom),
                ),
            )
            half = parse_vec(geom.get("size"), 3)
            geom_name = geom.get("name") or f"{body_name}_geom"
            boxes.append(SceneBox(
                object_id=sanitize_object_id(geom_name, object_prefix),
                xyz=geom_pose.xyz,
                quat_wxyz=geom_pose.quat_wxyz,
                dimensions=(2.0 * half[0], 2.0 * half[1], 2.0 * half[2]),
            ))

        for child in body.findall("body"):
            walk_body(child, body_pose)

    for geom in worldbody.findall("geom"):
        if not should_import_geom(geom):
            continue
        geom_pose = compose(
            world_pose,
            Pose3(
                parse_vec(geom.get("pos"), 3, (0.0, 0.0, 0.0)),  # type: ignore[arg-type]
                local_quat(geom),
            ),
        )
        half = parse_vec(geom.get("size"), 3)
        boxes.append(SceneBox(
            object_id=sanitize_object_id(geom.get("name") or "world_geom", object_prefix),
            xyz=geom_pose.xyz,
            quat_wxyz=geom_pose.quat_wxyz,
            dimensions=(2.0 * half[0], 2.0 * half[1], 2.0 * half[2]),
        ))

    for body in worldbody.findall("body"):
        walk_body(body, world_pose)

    return boxes


def find_default_scene() -> Path:
    start = Path(__file__).resolve()
    for parent in (start.parent, *start.parents):
        candidate = parent / PROJECT_SCENE_RELATIVE
        if candidate.exists():
            return candidate
    return Path.cwd() / PROJECT_SCENE_RELATIVE


def dry_run(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Dry-run MuJoCo scene XML parser")
    parser.add_argument("--xml", default=str(find_default_scene()))
    parser.add_argument("--object-prefix", default=DEFAULT_OBJECT_PREFIX)
    args = parser.parse_args(argv)
    boxes = parse_mujoco_scene(Path(args.xml), args.object_prefix)
    print(f"xml: {args.xml}")
    print(f"boxes: {len(boxes)}")
    for box in boxes[:12]:
        print(f"  {box.object_id}: xyz={box.xyz}, dims={box.dimensions}")
    if len(boxes) > 12:
        print(f"  ... {len(boxes) - 12} more")
    return 0


def main() -> int:
    if "--dry-run" in sys.argv:
        argv = [arg for arg in sys.argv[1:] if arg != "--dry-run"]
        return dry_run(argv)

    if "--allow-legacy" not in sys.argv:
        print(
            "mujoco_planning_scene_bridge.py is the legacy static XML bridge and is disabled by default. "
            "Use mujoco_digital_twin.launch.py, which already starts semantic_scene_to_planning_scene.py. "
            "Running both bridges creates duplicated/stale MoveIt obstacles. "
            "Pass --allow-legacy only for old static-scene debugging.",
            file=sys.stderr,
        )
        return 2
    sys.argv = [sys.argv[0], *(arg for arg in sys.argv[1:] if arg != "--allow-legacy")]

    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene
    from shape_msgs.msg import SolidPrimitive

    def make_pose(xyz, quat_wxyz):
        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation.w = float(quat_wxyz[0])
        pose.orientation.x = float(quat_wxyz[1])
        pose.orientation.y = float(quat_wxyz[2])
        pose.orientation.z = float(quat_wxyz[3])
        return pose

    def make_box_object(box: SceneBox, frame_id: str, operation: int = CollisionObject.ADD):
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = box.object_id
        collision_object.operation = operation
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(v) for v in box.dimensions]
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(make_pose(box.xyz, box.quat_wxyz))
        return collision_object

    def as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).lower() in {"1", "true", "yes", "on"}

    def as_float(value) -> float:
        return float(value)

    def as_float_list(value, expected: int, default: tuple[float, ...]) -> tuple[float, ...]:
        if value is None:
            return default
        if isinstance(value, str):
            text = value.strip().strip("[]")
            if not text:
                return default
            parts = [p for p in re.split(r"[,\s]+", text) if p]
            values = tuple(float(p) for p in parts)
        else:
            values = tuple(float(v) for v in value)
        if len(values) != expected:
            raise ValueError(f"expected {expected} values, got {values}")
        return values

    class MujocoPlanningSceneBridge(Node):
        def __init__(self):
            super().__init__("mujoco_planning_scene_bridge")
            self.declare_parameter("xml_path", "")
            self.declare_parameter("frame_id", DEFAULT_FRAME_ID)
            self.declare_parameter("object_prefix", DEFAULT_OBJECT_PREFIX)
            self.declare_parameter("apply_service", "/apply_planning_scene")
            self.declare_parameter("collision_object_topic", "/collision_object")
            self.declare_parameter("planning_scene_topic", "/planning_scene")
            self.declare_parameter("publish_topic", True)
            self.declare_parameter("apply_on_start", True)
            self.declare_parameter("republish_period", 1.0)
            self.declare_parameter("attach_object_id", "")
            self.declare_parameter("attach_link", "")
            self.declare_parameter("attach_size", [0.20, 0.40, 0.40])
            self.declare_parameter("attach_xyz", [0.0, 0.0, 0.0])
            self.declare_parameter("attach_rpy", [0.0, 0.0, 0.0])
            self.declare_parameter("touch_links", [])

            xml_param = self.get_parameter("xml_path").value
            self.xml_path = Path(str(xml_param)) if str(xml_param) else find_default_scene()
            self.frame_id = str(self.get_parameter("frame_id").value)
            self.object_prefix = str(self.get_parameter("object_prefix").value)
            self.apply_service = str(self.get_parameter("apply_service").value)
            self.publish_topic = as_bool(self.get_parameter("publish_topic").value)
            self.apply_on_start = as_bool(self.get_parameter("apply_on_start").value)
            self.republish_period = as_float(self.get_parameter("republish_period").value)
            self.attach_object_id = str(self.get_parameter("attach_object_id").value)
            self.attach_link = str(self.get_parameter("attach_link").value)
            self.attach_size = as_float_list(self.get_parameter("attach_size").value, 3, (0.20, 0.40, 0.40))
            self.attach_xyz = as_float_list(self.get_parameter("attach_xyz").value, 3, (0.0, 0.0, 0.0))
            self.attach_rpy = as_float_list(self.get_parameter("attach_rpy").value, 3, (0.0, 0.0, 0.0))
            self.touch_links = [str(x) for x in self.get_parameter("touch_links").value]

            self.collision_pub = self.create_publisher(
                CollisionObject,
                str(self.get_parameter("collision_object_topic").value),
                10,
            )
            self.planning_scene_pub = self.create_publisher(
                PlanningScene,
                str(self.get_parameter("planning_scene_topic").value),
                10,
            )
            self.scene_client = self.create_client(ApplyPlanningScene, self.apply_service)
            self.boxes = parse_mujoco_scene(self.xml_path, self.object_prefix)
            self.get_logger().info(
                f"loaded {len(self.boxes)} MuJoCo boxes from {self.xml_path} into frame {self.frame_id}"
            )

            if self.apply_on_start:
                self.apply_scene(wait_for_service=True)
            if self.publish_topic:
                self.publish_collision_objects()
            if self.republish_period > 0.0:
                self.create_timer(self.republish_period, self.publish_collision_objects)

        def build_scene(self) -> PlanningScene:
            scene = PlanningScene()
            scene.is_diff = True
            scene.world.collision_objects = [make_box_object(box, self.frame_id) for box in self.boxes]
            if self.attach_object_id and self.attach_link:
                attached = AttachedCollisionObject()
                attached.link_name = self.attach_link
                attached.touch_links = list(self.touch_links)
                attached.object.header.frame_id = self.attach_link
                attached.object.id = self.attach_object_id
                attached.object.operation = CollisionObject.ADD
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [float(v) for v in self.attach_size]
                attached.object.primitives.append(primitive)
                attached.object.primitive_poses.append(
                    make_pose(self.attach_xyz, rpy_to_quat_wxyz(*self.attach_rpy))
                )
                scene.robot_state.attached_collision_objects.append(attached)
                scene.robot_state.is_diff = True
            return scene

        def apply_scene(self, wait_for_service: bool = False):
            if wait_for_service:
                self.get_logger().info(f"waiting for {self.apply_service} ...")
                if not self.scene_client.wait_for_service(timeout_sec=10.0):
                    self.get_logger().warn(
                        f"{self.apply_service} unavailable; publishing /collision_object only"
                    )
                    return False
            req = ApplyPlanningScene.Request()
            req.scene = self.build_scene()
            future = self.scene_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.done() and future.result() is not None and future.result().success:
                attached_note = ""
                if self.attach_object_id:
                    attached_note = f", attached={self.attach_object_id}@{self.attach_link}"
                self.get_logger().info(f"applied {len(self.boxes)} collision objects{attached_note}")
                return True
            self.get_logger().warn("ApplyPlanningScene did not confirm success")
            return False

        def publish_collision_objects(self):
            scene = self.build_scene()
            self.planning_scene_pub.publish(scene)
            for box in self.boxes:
                self.collision_pub.publish(make_box_object(box, self.frame_id))
            self.get_logger().info(
                f"published planning scene diff with {len(self.boxes)} objects"
            )

    rclpy.init()
    node = MujocoPlanningSceneBridge()
    try:
        if node.republish_period > 0.0:
            rclpy.spin(node)
        else:
            time.sleep(0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
