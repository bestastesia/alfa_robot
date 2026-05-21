#!/usr/bin/python3
"""Convert compact SemanticScene messages into MoveIt PlanningScene objects."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from alfa_robot_moveit_config.msg import SemanticScene
from semantic_scene_utils import collision_objects_to_points, make_point_cloud2, semantic_scene_to_collision_objects


class SemanticSceneToPlanningScene(Node):
    def __init__(self, args):
        super().__init__("semantic_scene_to_planning_scene")
        self.args = args
        self.frame_id = args.frame_id
        self.apply_on_start = args.apply_on_start
        self.apply_sent = False
        self.last_publish_time = self.get_clock().now()
        self.last_pointcloud_time = self.get_clock().now()
        self.min_period = 1.0 / args.max_publish_rate if args.max_publish_rate > 0.0 else 0.0
        self.pointcloud_period = 1.0 / args.pointcloud_rate if args.pointcloud_rate > 0.0 else 0.0

        self.planning_scene_pub = self.create_publisher(PlanningScene, args.planning_scene_topic, 10)
        self.collision_object_pub = self.create_publisher(CollisionObject, args.collision_object_topic, 10)
        self.pointcloud_pub = None
        if args.pointcloud_topic and args.pointcloud_rate > 0.0:
            from sensor_msgs.msg import PointCloud2
            self.pointcloud_pub = self.create_publisher(PointCloud2, args.pointcloud_topic, 10)
        self.apply_scene_client = self.create_client(ApplyPlanningScene, args.apply_service)
        self.create_subscription(SemanticScene, args.semantic_scene_topic, self.on_scene, 10)
        self.get_logger().info(
            f"SemanticScene converter ready: semantic={args.semantic_scene_topic}, "
            f"planning_scene={args.planning_scene_topic}, frame_id={args.frame_id}"
        )

    def on_scene(self, semantic_scene: SemanticScene):
        now = self.get_clock().now()
        if self.min_period > 0.0 and (now - self.last_publish_time).nanoseconds * 1e-9 < self.min_period:
            return
        self.last_publish_time = now

        frame_id = semantic_scene.header.frame_id or self.frame_id
        collision_objects = semantic_scene_to_collision_objects(
            semantic_scene,
            frame_id,
            CollisionObject,
            SolidPrimitive,
        )
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_objects
        self.planning_scene_pub.publish(scene)
        for collision_object in collision_objects:
            self.collision_object_pub.publish(collision_object)

        if self.pointcloud_pub is not None and (now - self.last_pointcloud_time).nanoseconds * 1e-9 >= self.pointcloud_period:
            self.last_pointcloud_time = now
            points = collision_objects_to_points(collision_objects)
            self.pointcloud_pub.publish(make_point_cloud2(points, frame_id, self))

        if self.apply_on_start and not self.apply_sent and self.apply_scene_client.service_is_ready():
            request = ApplyPlanningScene.Request()
            request.scene = scene
            self.apply_scene_client.call_async(request)
            self.apply_sent = True


def parse_bool(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Convert SemanticScene to MoveIt PlanningScene")
    parser.add_argument("--semantic-scene-topic", default="/mujoco_semantic_scene")
    parser.add_argument("--planning-scene-topic", default="/planning_scene")
    parser.add_argument("--collision-object-topic", default="/collision_object")
    parser.add_argument("--pointcloud-topic", default="/mujoco_scene_points")
    parser.add_argument("--pointcloud-rate", type=float, default=0.0)
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--apply-service", default="/apply_planning_scene")
    parser.add_argument("--apply-on-start", default="true")
    parser.add_argument("--max-publish-rate", type=float, default=10.0)
    args, _ros_args = parser.parse_known_args(argv)
    args.apply_on_start = parse_bool(args.apply_on_start)
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    rclpy.init()
    node = SemanticSceneToPlanningScene(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
