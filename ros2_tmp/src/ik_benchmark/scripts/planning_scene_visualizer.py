#!/usr/bin/env python3
"""Visualize MoveIt collision objects in RViz markers and optionally Rerun."""

from __future__ import annotations

import math
from typing import Iterable

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray


class PlanningSceneVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('planning_scene_visualizer')
        self.declare_parameter('planning_scene_topic', '/planning_scene')
        self.declare_parameter('collision_object_topic', '/collision_object')
        self.declare_parameter('marker_topic', '/alfa_visualization/planning_scene_markers')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('use_rerun', False)
        self.declare_parameter('rerun_app_id', 'alfa_planning_scene')
        self.declare_parameter('rerun_connect', False)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.objects: dict[str, CollisionObject] = {}
        self.marker_pub = self.create_publisher(MarkerArray, str(self.get_parameter('marker_topic').value), 10)
        self.create_subscription(
            PlanningScene,
            str(self.get_parameter('planning_scene_topic').value),
            self._on_planning_scene,
            10,
        )
        self.create_subscription(
            CollisionObject,
            str(self.get_parameter('collision_object_topic').value),
            self._on_collision_object,
            10,
        )
        self.timer = self.create_timer(0.5, self._publish_markers)
        self.rr = None
        if bool(self.get_parameter('use_rerun').value):
            try:
                import rerun as rr  # type: ignore

                self.rr = rr
                rr.init(str(self.get_parameter('rerun_app_id').value), spawn=not bool(self.get_parameter('rerun_connect').value))
                if bool(self.get_parameter('rerun_connect').value):
                    rr.connect()
                self.get_logger().info('Rerun visualization enabled')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'Rerun unavailable, RViz marker output still active: {exc}')
        self.get_logger().info(
            f'Planning scene visualizer ready: markers={self.get_parameter("marker_topic").value}'
        )

    def _on_planning_scene(self, msg: PlanningScene) -> None:
        for obj in msg.world.collision_objects:
            self._apply_object(obj)
        self._publish_markers()

    def _on_collision_object(self, msg: CollisionObject) -> None:
        self._apply_object(msg)
        self._publish_markers()

    def _apply_object(self, obj: CollisionObject) -> None:
        if obj.operation == CollisionObject.REMOVE:
            self.objects.pop(obj.id, None)
            return
        if obj.operation in (CollisionObject.ADD, CollisionObject.APPEND, CollisionObject.MOVE, 0):
            self.objects[obj.id] = obj

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)
        marker_id = 1
        for obj_index, obj in enumerate(sorted(self.objects.values(), key=lambda item: item.id)):
            frame_id = obj.header.frame_id or self.frame_id
            color = self._color_for_index(obj_index)
            for primitive, pose in zip(obj.primitives, obj.primitive_poses):
                marker = self._primitive_marker(obj.id, marker_id, frame_id, primitive, pose, color)
                marker_array.markers.append(marker)
                marker_id += 1
                self._log_rerun_primitive(obj.id, primitive, pose, color)
            for mesh, pose in zip(obj.meshes, obj.mesh_poses):
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = obj.id
                marker.id = marker_id
                marker.action = Marker.ADD
                marker.type = Marker.TRIANGLE_LIST
                marker.pose = pose
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
                marker.scale.x = marker.scale.y = marker.scale.z = 1.0
                for triangle in mesh.triangles:
                    for vertex_index in triangle.vertex_indices:
                        marker.points.append(mesh.vertices[vertex_index])
                marker_array.markers.append(marker)
                marker_id += 1
            label = self._label_marker(obj.id, marker_id, frame_id, self._first_pose(obj), color)
            marker_array.markers.append(label)
            marker_id += 1
        self.marker_pub.publish(marker_array)

    def _primitive_marker(self, object_id: str, marker_id: int, frame_id: str,
                          primitive: SolidPrimitive, pose: Pose, color: tuple[float, float, float, float]) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = object_id
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose = pose
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        if primitive.type == SolidPrimitive.BOX:
            marker.type = Marker.CUBE
            marker.scale.x = primitive.dimensions[SolidPrimitive.BOX_X]
            marker.scale.y = primitive.dimensions[SolidPrimitive.BOX_Y]
            marker.scale.z = primitive.dimensions[SolidPrimitive.BOX_Z]
        elif primitive.type == SolidPrimitive.SPHERE:
            marker.type = Marker.SPHERE
            diameter = 2.0 * primitive.dimensions[SolidPrimitive.SPHERE_RADIUS]
            marker.scale.x = marker.scale.y = marker.scale.z = diameter
        elif primitive.type == SolidPrimitive.CYLINDER:
            marker.type = Marker.CYLINDER
            marker.scale.x = marker.scale.y = 2.0 * primitive.dimensions[SolidPrimitive.CYLINDER_RADIUS]
            marker.scale.z = primitive.dimensions[SolidPrimitive.CYLINDER_HEIGHT]
        else:
            marker.type = Marker.CUBE
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        return marker

    def _label_marker(self, object_id: str, marker_id: int, frame_id: str,
                      pose: Pose, color: tuple[float, float, float, float]) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f'{object_id}_label'
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose = pose
        marker.pose.position.z += 0.08
        marker.text = object_id
        marker.scale.z = 0.06
        marker.color.r = marker.color.g = marker.color.b = 1.0
        marker.color.a = color[3]
        return marker

    def _first_pose(self, obj: CollisionObject) -> Pose:
        if obj.primitive_poses:
            return obj.primitive_poses[0]
        if obj.mesh_poses:
            return obj.mesh_poses[0]
        return Pose()

    def _log_rerun_primitive(self, object_id: str, primitive: SolidPrimitive,
                             pose: Pose, color: tuple[float, float, float, float]) -> None:
        if self.rr is None:
            return
        try:
            import numpy as np

            translation = [pose.position.x, pose.position.y, pose.position.z]
            rgba = [int(max(0.0, min(1.0, c)) * 255) for c in color]
            if primitive.type == SolidPrimitive.BOX:
                half_sizes = [
                    0.5 * primitive.dimensions[SolidPrimitive.BOX_X],
                    0.5 * primitive.dimensions[SolidPrimitive.BOX_Y],
                    0.5 * primitive.dimensions[SolidPrimitive.BOX_Z],
                ]
                self.rr.log(f'planning_scene/{object_id}', self.rr.Boxes3D(centers=[translation], half_sizes=[half_sizes], colors=[rgba]))
            elif primitive.type == SolidPrimitive.SPHERE:
                radius = primitive.dimensions[SolidPrimitive.SPHERE_RADIUS]
                self.rr.log(f'planning_scene/{object_id}', self.rr.Points3D(np.array([translation]), radii=[radius], colors=[rgba]))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f'Rerun log failed for {object_id}: {exc}')

    def _color_for_index(self, index: int) -> tuple[float, float, float, float]:
        hue = (index * 0.61803398875) % 1.0
        r, g, b = hsv_to_rgb(hue, 0.55, 0.95)
        return r, g, b, 0.45


def hsv_to_rgb(h: float, s: float, v: float) -> tuple[float, float, float]:
    i = int(math.floor(h * 6.0))
    f = h * 6.0 - i
    p = v * (1.0 - s)
    q = v * (1.0 - f * s)
    t = v * (1.0 - (1.0 - f) * s)
    i %= 6
    if i == 0:
        return v, t, p
    if i == 1:
        return q, v, p
    if i == 2:
        return p, v, t
    if i == 3:
        return p, q, v
    if i == 4:
        return t, p, v
    return v, p, q


def main() -> None:
    rclpy.init()
    node = PlanningSceneVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
