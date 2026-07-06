#!/usr/bin/env python3
"""Monitor RViz MoveIt interactive-marker target poses for both end effectors.

This node listens to RViz interactive marker feedback, extracts the target pose
of the left/right MoveIt end-effector markers, prints them in real time, and
republishes them as PoseStamped plus RViz text/axis markers.

It intentionally tracks RViz goal-marker feedback, not the robot's current TF.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Dict, Iterable, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarkerFeedback, Marker, MarkerArray


@dataclass(frozen=True)
class SideConfig:
    side: str
    label: str
    marker_patterns: Iterable[str]
    topic: str
    color_rgba: tuple[float, float, float, float]


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RvizDualGoalPoseMonitor(Node):
    def __init__(self):
        super().__init__("rviz_dual_goal_pose_monitor")

        self.declare_parameter(
            "feedback_topics",
            [
                "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
                "/rviz2_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
                "/robot_interaction_interactive_marker_topic/feedback",
                "/move_marker/feedback",
            ],
        )
        self.declare_parameter("left_marker_patterns", ["left", "left", "left_tool", "left_tool0"])
        self.declare_parameter("right_marker_patterns", ["right", "right", "right_tool", "right_tool0"])
        self.declare_parameter("target_frame", "")
        self.declare_parameter("log_every_update", True)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_topic", "/rviz_dual_goal_pose_monitor/markers")

        self._log_every_update = bool(self.get_parameter("log_every_update").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._last_pose_by_side: Dict[str, PoseStamped] = {}

        self._sides = {
            "left": SideConfig(
                "left",
                "LEFT target",
                list(self.get_parameter("left_marker_patterns").value),
                "/rviz_dual_goal_pose_monitor/left_target_pose",
                (0.1, 0.6, 1.0, 1.0),
            ),
            "right": SideConfig(
                "right",
                "RIGHT target",
                list(self.get_parameter("right_marker_patterns").value),
                "/rviz_dual_goal_pose_monitor/right_target_pose",
                (1.0, 0.45, 0.05, 1.0),
            ),
        }

        self._pose_pubs = {
            side: self.create_publisher(PoseStamped, cfg.topic, 10)
            for side, cfg in self._sides.items()
        }
        self._marker_pub = None
        if bool(self.get_parameter("publish_markers").value):
            self._marker_pub = self.create_publisher(
                MarkerArray, str(self.get_parameter("marker_topic").value), 10
            )

        self._subscriptions = []
        for topic in self.get_parameter("feedback_topics").value:
            topic = str(topic)
            self._subscriptions.append(
                self.create_subscription(
                    InteractiveMarkerFeedback,
                    topic,
                    self._feedback_callback,
                    50,
                )
            )
            self.get_logger().info(f"Listening for RViz goal feedback: {topic}")

        self.get_logger().info(
            "Publishing target poses: left=%s right=%s"
            % (self._sides["left"].topic, self._sides["right"].topic)
        )

    def _feedback_callback(self, msg: InteractiveMarkerFeedback) -> None:
        if msg.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
            InteractiveMarkerFeedback.MOUSE_DOWN,
        ):
            return

        side = self._classify_side(msg.marker_name)
        if side is None:
            return

        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        if self._target_frame:
            pose_msg.header.frame_id = self._target_frame
        pose_msg.pose = msg.pose

        self._last_pose_by_side[side] = pose_msg
        self._pose_pubs[side].publish(pose_msg)

        if self._marker_pub is not None:
            self._marker_pub.publish(self._make_marker_array())

        if self._log_every_update or msg.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self._log_pose(side, msg.marker_name, pose_msg)

    def _classify_side(self, marker_name: str) -> Optional[str]:
        normalized = marker_name.lower()
        for side, cfg in self._sides.items():
            for pattern in cfg.marker_patterns:
                if re.search(str(pattern).lower(), normalized):
                    return side
        return None

    def _log_pose(self, side: str, marker_name: str, pose_msg: PoseStamped) -> None:
        pose = pose_msg.pose
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self.get_logger().info(
            "%s [%s] frame=%s xyz=(%.4f, %.4f, %.4f) "
            "quat=(%.4f, %.4f, %.4f, %.4f) yaw=%.3f"
            % (
                self._sides[side].label,
                marker_name,
                pose_msg.header.frame_id,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
                yaw,
            )
        )

    def _make_marker_array(self) -> MarkerArray:
        marker_array = MarkerArray()
        marker_id = 0
        for side, pose_msg in self._last_pose_by_side.items():
            cfg = self._sides[side]
            color = cfg.color_rgba

            sphere = Marker()
            sphere.header = pose_msg.header
            sphere.ns = "rviz_goal_pose"
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose_msg.pose
            sphere.scale.x = 0.06
            sphere.scale.y = 0.06
            sphere.scale.z = 0.06
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = color
            marker_array.markers.append(sphere)

            text = Marker()
            text.header = pose_msg.header
            text.ns = "rviz_goal_pose_label"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = pose_msg.pose
            text.pose.position.z += 0.10
            text.scale.z = 0.06
            text.color.r, text.color.g, text.color.b, text.color.a = color
            p = pose_msg.pose.position
            text.text = f"{cfg.label}\n{p.x:.3f}, {p.y:.3f}, {p.z:.3f}"
            marker_array.markers.append(text)

        return marker_array


def main(args=None) -> int:
    rclpy.init(args=args)
    node = RvizDualGoalPoseMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
