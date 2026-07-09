from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_motion_interfaces.msg import RobotMotionScene
from robot_motion_interfaces.srv import SetRobotMotionScene
from robot_motion_runtime.common import RuntimeStatusPublisher, stamp_is_zero


class MotionSceneSourceNode(Node):
    """Authoritative planning-scene source for runtime services."""

    def __init__(self) -> None:
        super().__init__("motion_scene_source")
        self.declare_parameter("output_topic", "/robot_motion/scene")
        self.declare_parameter("set_scene_service", "/robot_motion/set_scene")
        self.declare_parameter("default_source", "explicit_set_scene")
        self.declare_parameter("publish_empty_scene_on_start", True)
        self.declare_parameter("default_frame_id", "base_link")
        self.declare_parameter("default_scene_id", "empty_scene")

        self.output_topic = str(self.get_parameter("output_topic").value)
        self.set_scene_service = str(self.get_parameter("set_scene_service").value)
        self.default_source = str(self.get_parameter("default_source").value)
        self.publish_empty_scene_on_start = bool(self.get_parameter("publish_empty_scene_on_start").value)
        self.default_frame_id = str(self.get_parameter("default_frame_id").value)
        self.default_scene_id = str(self.get_parameter("default_scene_id").value)

        self.publisher = self.create_publisher(RobotMotionScene, self.output_topic, 10)
        self.service = self.create_service(SetRobotMotionScene, self.set_scene_service, self.on_set_scene)
        self.latest_scene: RobotMotionScene | None = None
        self.status = RuntimeStatusPublisher(
            self,
            self.set_scene_service,
            "authoritative RobotMotionScene source; set_scene fixes simulation/mock scene",
        )
        self.status.mark_ready("waiting for explicit scene")
        if self.publish_empty_scene_on_start:
            self.latest_scene = self.make_empty_scene()
            self.publisher.publish(self.latest_scene)
            self.status.mark_ready("published empty startup scene")
        self.get_logger().info(
            f"Motion scene source ready: output={self.output_topic}, set_scene={self.set_scene_service}"
        )

    def make_empty_scene(self) -> RobotMotionScene:
        scene = RobotMotionScene()
        scene.context.frame_id = self.default_frame_id
        scene.context.scene_id = self.default_scene_id
        scene.source = "startup_empty_scene"
        scene.authoritative = True
        scene.context.stamp = self.get_clock().now().to_msg()
        return scene

    def on_set_scene(self, request: SetRobotMotionScene.Request, response: SetRobotMotionScene.Response):
        self.status.mark_running("set_scene request")
        scene = RobotMotionScene()
        scene.context = request.context
        if stamp_is_zero(scene.context.stamp):
            scene.context.stamp = self.get_clock().now().to_msg()
        if not scene.context.frame_id:
            scene.context.frame_id = self.default_frame_id
        if not scene.context.scene_id:
            scene.context.scene_id = f"manual_scene:{int(scene.context.stamp.sec)}.{int(scene.context.stamp.nanosec)}"
        scene.source = request.source or self.default_source
        scene.authoritative = bool(request.authoritative)
        scene.scene_objects = list(request.scene_objects)
        scene.attached_collision_objects = list(request.attached_collision_objects)

        self.latest_scene = scene
        self.publisher.publish(scene)
        response.success = True
        response.message = (
            f"scene fixed: scene_id={scene.context.scene_id} "
            f"objects={len(scene.scene_objects)} attached={len(scene.attached_collision_objects)}"
        )
        response.scene = scene
        self.status.mark_done(True, response.message)
        return response


def main() -> None:
    rclpy.init()
    node = MotionSceneSourceNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
