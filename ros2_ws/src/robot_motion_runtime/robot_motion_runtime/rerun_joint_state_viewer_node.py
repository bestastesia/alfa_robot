from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def find_repo_root() -> Path:
    starts = [Path.cwd().resolve(), Path(__file__).resolve()]
    for start in list(starts):
        starts.extend(start.parents)
    for candidate in starts:
        if (candidate / "scripts" / "ik_benchmark" / "scripts" / "visualize_rerun.py").exists():
            return candidate
        if (candidate.parent / "scripts" / "ik_benchmark" / "scripts" / "visualize_rerun.py").exists():
            return candidate.parent
    raise RuntimeError("cannot find alfa_robot repo root containing scripts/ik_benchmark/scripts/visualize_rerun.py")


def load_rerun_helpers() -> Any:
    helper_path = find_repo_root() / "scripts" / "ik_benchmark" / "scripts" / "visualize_rerun.py"
    spec = importlib.util.spec_from_file_location("alfa_visualize_rerun_helpers", helper_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {helper_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


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
        self.helpers = load_rerun_helpers()
        self.robot = self.helpers.UrdfRobot(self.helpers.render_current_urdf())
        self.rr.init(self.app_id)
        if self.spawn_viewer:
            self.rr.spawn()
        self.helpers.log_robot_static_model(self.robot, self.robot_path, log_meshes=self.log_meshes)

        self.latest: dict[str, float] = {}
        self.sample = 0
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
        self.helpers.set_sample_time(self.sample)
        self.helpers.log_robot_state(self.robot, self.latest, self.robot_path)
        self.rr.log(f"{self.robot_path}/status", self.rr.TextLog(f"sample={self.sample} joints={len(self.latest)}"))
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
