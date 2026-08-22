#!/usr/bin/python3
"""PROTOTYPE RViz six-DoF interactive target publisher."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


class Interactive6dTarget(Node):
    def __init__(self) -> None:
        super().__init__("interactive_6d_target")
        self.declare_parameter("marker_scale", 0.28)
        self.server = InteractiveMarkerServer(self, "/realtime_6d_pose/target_marker")
        self.target_pub = self.create_publisher(
            PoseStamped, "/realtime_6d_pose/target_pose_cmd", 10
        )
        initial_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseStamped,
            "/realtime_6d_pose/initial_target_pose",
            self.on_initial_pose,
            initial_qos,
        )
        self.initialized = False
        self.get_logger().info("waiting for Motion initial 6D pose")

    @staticmethod
    def axis_quaternion(axis: str) -> Quaternion:
        half = math.sqrt(0.5)
        quaternion = Quaternion(w=half)
        if axis == "x":
            quaternion.x = half
        elif axis == "z":
            quaternion.y = half
        else:
            quaternion.z = half
        return quaternion

    def add_axis_control(self, marker: InteractiveMarker, axis: str, mode: int) -> None:
        control = InteractiveMarkerControl()
        control.name = f"{'move' if mode == InteractiveMarkerControl.MOVE_AXIS else 'rotate'}_{axis}"
        control.orientation = self.axis_quaternion(axis)
        control.interaction_mode = mode
        marker.controls.append(control)

    def on_initial_pose(self, message: PoseStamped) -> None:
        if self.initialized:
            return
        marker = InteractiveMarker()
        marker.header.frame_id = "base_link"
        marker.name = "target_6d_pose"
        marker.description = "Drag Motion 6D target"
        marker.scale = float(self.get_parameter("marker_scale").value)
        marker.pose = message.pose

        visual = Marker()
        visual.type = Marker.SPHERE
        visual.scale.x = 0.075
        visual.scale.y = 0.075
        visual.scale.z = 0.075
        visual.color.r = 1.0
        visual.color.g = 0.55
        visual.color.b = 0.05
        visual.color.a = 0.85
        visual_control = InteractiveMarkerControl()
        visual_control.name = "target_visual"
        visual_control.always_visible = True
        visual_control.markers.append(visual)
        marker.controls.append(visual_control)

        for axis in ("x", "y", "z"):
            self.add_axis_control(marker, axis, InteractiveMarkerControl.ROTATE_AXIS)
            self.add_axis_control(marker, axis, InteractiveMarkerControl.MOVE_AXIS)

        self.server.insert(marker, feedback_callback=self.on_feedback)
        self.server.applyChanges()
        self.initialized = True
        self.publish_pose(message.pose)
        self.get_logger().info(
            "6D target ready in RViz: drag arrows/rings; publishing target_pose_cmd"
        )

    def on_feedback(self, feedback) -> None:
        self.publish_pose(feedback.pose)

    def publish_pose(self, pose) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.pose = pose
        self.target_pub.publish(message)


def main() -> None:
    rclpy.init()
    node = Interactive6dTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.server.clear()
                node.server.applyChanges()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
