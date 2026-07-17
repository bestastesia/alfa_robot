from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from robot_motion_runtime.common import RuntimeStatusPublisher


class VehiclePoseSourceNode(Node):
    """Publishes the map->world transform describing where the vehicle sits in the map frame.

    This is a stand-in for the navigation stack's localization output. `world` remains the
    fixed base_link alias defined in the URDF (world_to_base), so this node is the only source
    of truth for how the vehicle (and therefore world/base_link) is placed in map. Until the
    navigation team's real interface is settled, motion set here comes from parameters only;
    swapping in a real localization topic later only touches this node.
    """

    def __init__(self) -> None:
        super().__init__("vehicle_pose_source")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError(f"publish_rate_hz must be > 0, got {publish_rate_hz}")

        self.broadcaster = TransformBroadcaster(self)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.status = RuntimeStatusPublisher(
            self,
            "vehicle_pose_source",
            f"simulated {self.map_frame}->{self.world_frame} vehicle pose; "
            "placeholder for the navigation stack's localization output",
        )
        self.status.mark_ready(f"x={self.x:.3f} y={self.y:.3f} yaw_deg={math.degrees(self.yaw):.2f}")
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_transform)

    @property
    def x(self) -> float:
        return float(self.get_parameter("x").value)

    @property
    def y(self) -> float:
        return float(self.get_parameter("y").value)

    @property
    def yaw(self) -> float:
        return float(self.get_parameter("yaw").value)

    def _on_set_parameters(self, params):
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == "publish_rate_hz" and param.value <= 0.0:
                return SetParametersResult(successful=False, reason="publish_rate_hz must be > 0")
        return SetParametersResult(successful=True)

    def publish_transform(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.world_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        half_yaw = self.yaw / 2.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(half_yaw)
        transform.transform.rotation.w = math.cos(half_yaw)
        self.broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = VehiclePoseSourceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
