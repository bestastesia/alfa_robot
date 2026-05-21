from __future__ import annotations

import struct
from dataclasses import dataclass


@dataclass(frozen=True)
class SemanticBoxRule:
    object_id: str
    source: str
    dimensions: tuple[float, float, float]
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0)


CONTAINER_LENGTH = 4.0
CONTAINER_INNER_WIDTH = 2.2
CONTAINER_INNER_HEIGHT = 2.4
CONTAINER_WALL_THICKNESS = 0.09
CONTAINER_ROOF_THICKNESS = 0.07
CONTAINER_FLOOR_THICKNESS = 0.07

CONTAINER_CENTER_X_OFFSET = 2.045
CONTAINER_WALL_Y_OFFSET = CONTAINER_INNER_WIDTH * 0.5 + CONTAINER_WALL_THICKNESS * 0.5
CONTAINER_WALL_Z_OFFSET = -1.225
CONTAINER_FLOOR_Z_OFFSET = -2.46

SEMANTIC_BOX_RULES = [
    SemanticBoxRule(
        "semantic_container_floor",
        "container_front",
        (CONTAINER_LENGTH, CONTAINER_INNER_WIDTH, CONTAINER_FLOOR_THICKNESS),
        offset=(CONTAINER_CENTER_X_OFFSET, 0.0, CONTAINER_FLOOR_Z_OFFSET),
    ),
    SemanticBoxRule(
        "semantic_container_left_wall",
        "container_front",
        (CONTAINER_LENGTH, CONTAINER_WALL_THICKNESS, CONTAINER_INNER_HEIGHT),
        offset=(CONTAINER_CENTER_X_OFFSET, CONTAINER_WALL_Y_OFFSET, CONTAINER_WALL_Z_OFFSET),
    ),
    SemanticBoxRule(
        "semantic_container_right_wall",
        "container_front",
        (CONTAINER_LENGTH, CONTAINER_WALL_THICKNESS, CONTAINER_INNER_HEIGHT),
        offset=(CONTAINER_CENTER_X_OFFSET, -CONTAINER_WALL_Y_OFFSET, CONTAINER_WALL_Z_OFFSET),
    ),
    SemanticBoxRule(
        "semantic_container_roof",
        "container_front",
        (CONTAINER_LENGTH, CONTAINER_INNER_WIDTH, CONTAINER_ROOF_THICKNESS),
        offset=(CONTAINER_CENTER_X_OFFSET, 0.0, 0.01),
    ),
    SemanticBoxRule("semantic_placement_platform", "table", (0.90, 1.50, 0.07)),
]

CARGO_BOX_DIMENSIONS = (0.20, 0.40, 0.40)


def quat_xyzw_rotate(q, v):
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def collision_objects_to_points(collision_objects):
    points = []
    samples = (-0.5, 0.0, 0.5)
    for collision_object in collision_objects:
        for primitive, pose in zip(collision_object.primitives, collision_object.primitive_poses):
            if len(primitive.dimensions) < 3:
                continue
            sx, sy, sz = primitive.dimensions[:3]
            quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            center = (pose.position.x, pose.position.y, pose.position.z)
            for ix in samples:
                for iy in samples:
                    for iz in samples:
                        rotated = quat_xyzw_rotate(quat, (sx * ix, sy * iy, sz * iz))
                        points.append((center[0] + rotated[0], center[1] + rotated[1], center[2] + rotated[2]))
    return points


def make_point_cloud2(points, frame_id, node):
    from sensor_msgs.msg import PointCloud2, PointField

    msg = PointCloud2()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = b"".join(struct.pack("<fff", *point) for point in points)
    return msg


def make_box_collision_object(CollisionObject, SolidPrimitive, frame_id, object_id, dimensions, pose):
    collision_object = CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = object_id
    collision_object.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(dimensions)
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(pose)
    return collision_object


def offset_pose(source_pose, dx=0.0, dy=0.0, dz=0.0):
    from geometry_msgs.msg import Pose

    pose = Pose()
    quat = (source_pose.orientation.x, source_pose.orientation.y, source_pose.orientation.z, source_pose.orientation.w)
    rx, ry, rz = quat_xyzw_rotate(quat, (dx, dy, dz))
    pose.position.x = source_pose.position.x + rx
    pose.position.y = source_pose.position.y + ry
    pose.position.z = source_pose.position.z + rz
    pose.orientation = source_pose.orientation
    return pose


def semantic_scene_to_collision_objects(scene_msg, frame_id, CollisionObject, SolidPrimitive):
    objects = []
    front = scene_msg.container_front_pose
    for rule in SEMANTIC_BOX_RULES:
        source_pose = front if rule.source == "container_front" else scene_msg.table_pose
        pose = offset_pose(source_pose, *rule.offset)
        objects.append(make_box_collision_object(
            CollisionObject,
            SolidPrimitive,
            frame_id,
            rule.object_id,
            rule.dimensions,
            pose,
        ))

    for cargo in scene_msg.cargo:
        objects.append(make_box_collision_object(
            CollisionObject,
            SolidPrimitive,
            frame_id,
            f"semantic_{cargo.id}",
            CARGO_BOX_DIMENSIONS,
            cargo.pose,
        ))
    return objects
