from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from alfa_robot_rerun import visualize_rerun as rerun_helpers


class RerunJointStateViewerNode(Node):
    """Read-only live Rerun view of /joint_states."""

    def __init__(self) -> None:
        super().__init__("rerun_joint_state_viewer")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("app_id", "robot_motion_runtime_live")
        self.declare_parameter("robot_path", "world/robot")
        self.declare_parameter("spawn_viewer", True)
        self.declare_parameter("log_rate_hz", 15.0)
        self.declare_parameter("log_meshes", True)

        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.app_id = str(self.get_parameter("app_id").value)
        self.robot_path = str(self.get_parameter("robot_path").value)
        self.spawn_viewer = bool(self.get_parameter("spawn_viewer").value)
        self.log_rate_hz = float(self.get_parameter("log_rate_hz").value)
        self.log_meshes = bool(self.get_parameter("log_meshes").value)

        try:
            import rerun as rerun_module
        except ImportError as exc:
            raise RuntimeError("rerun-sdk is not installed; disable this node or install rerun-sdk") from exc

        self.rr = rerun_module
        self.robot = rerun_helpers.UrdfRobot(rerun_helpers.render_current_urdf())
        rerun_helpers.prefer_matching_rerun_cli(self.rr)
        self.rr.init(self.app_id)
        if self.spawn_viewer:
            self.rr.spawn()
        rerun_helpers.log_robot_static_model(
            self.robot,
            self.robot_path,
            log_meshes=self.log_meshes,
        )
        rerun_helpers.set_sample_time(0)
        rerun_helpers.log_robot_state(self.robot, {}, self.robot_path)

        self.latest: dict[str, float] = {}
        self.sample = 1
        self.subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.on_joint_state,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / max(self.log_rate_hz, 1.0), self.on_timer)
        self.get_logger().info(
            f"Rerun joint-state viewer ready: topic={self.joint_state_topic}, "
            f"app={self.app_id}, path={self.robot_path}, rate={self.log_rate_hz:.1f}Hz"
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.latest = {name: float(value) for name, value in zip(msg.name, msg.position)}

    def on_timer(self) -> None:
        if not self.latest:
            return
        rerun_helpers.set_sample_time(self.sample)
        rerun_helpers.log_robot_state(self.robot, self.latest, self.robot_path)
        self.rr.log(
            f"{self.robot_path}/status",
            self.rr.TextLog(f"sample={self.sample} joints={len(self.latest)}"),
        )
        self.sample += 1


def main() -> None:
    rclpy.init()
    node = RerunJointStateViewerNode()
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
