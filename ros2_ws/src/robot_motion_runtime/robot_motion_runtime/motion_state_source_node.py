from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState

from robot_motion_interfaces.msg import RobotMotionState
from robot_motion_interfaces.srv import SetRobotMotionState
from robot_motion_runtime.common import RuntimeStatusPublisher, stamp_is_zero


class MotionStateSourceNode(Node):
    """Authoritative motion-state source.

    Real robot mode: subscribe to /joint_states and republish RobotMotionState.
    Simulation/mock mode: call /robot_motion/set_state once to freeze the initial fact state.
    """

    def __init__(self) -> None:
        super().__init__("motion_state_source")
        self.declare_parameter("input_joint_states", "/joint_states")
        self.declare_parameter("output_state_topic", "/robot_motion/state")
        self.declare_parameter("set_state_service", "/robot_motion/set_state")
        self.declare_parameter("source", "joint_states")
        self.declare_parameter("authoritative", True)
        self.declare_parameter("subscribe_joint_states", True)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("scene_id", "")
        self.declare_parameter("state_id_prefix", "")
        self.declare_parameter("publish_period_s", 0.2)

        self.input_topic = str(self.get_parameter("input_joint_states").value)
        self.output_topic = str(self.get_parameter("output_state_topic").value)
        self.set_state_service = str(self.get_parameter("set_state_service").value)
        self.source = str(self.get_parameter("source").value)
        self.authoritative = bool(self.get_parameter("authoritative").value)
        self.subscribe_joint_states = bool(self.get_parameter("subscribe_joint_states").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.scene_id = str(self.get_parameter("scene_id").value)
        self.state_id_prefix = str(self.get_parameter("state_id_prefix").value)
        self.publish_period_s = float(self.get_parameter("publish_period_s").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(RobotMotionState, self.output_topic, qos)
        self.latest_state: Optional[RobotMotionState] = None

        self.status = RuntimeStatusPublisher(
            self,
            self.set_state_service,
            "authoritative RobotMotionState source; one set_state call can freeze mock/simulation initial state",
        )

        self.subscription = None
        if self.subscribe_joint_states:
            self.subscription = self.create_subscription(
                JointState,
                self.input_topic,
                self.on_joint_state,
                qos_profile_sensor_data,
            )
        self.service = self.create_service(
            SetRobotMotionState,
            self.set_state_service,
            self.on_set_state,
        )
        self.timer = None
        if self.publish_period_s > 0.0 and math.isfinite(self.publish_period_s):
            self.timer = self.create_timer(self.publish_period_s, self.publish_latest)

        self.status.mark_ready(
            f"publishing {self.output_topic}, subscribe_joint_states={self.subscribe_joint_states}"
        )
        self.get_logger().info(
            "Motion state source ready: "
            f"output={self.output_topic}, set_state={self.set_state_service}, "
            f"input={self.input_topic if self.subscribe_joint_states else 'disabled'}"
        )

    def make_state_id(self, stamp, source: str) -> str:
        prefix = self.state_id_prefix or source or "motion_state"
        return f"{prefix}:{stamp.sec}.{stamp.nanosec:09d}"

    def make_state(
        self,
        joint_state: JointState,
        source: str,
        authoritative: bool,
        request_id: str = "",
        frame_id: str = "",
        scene_id: str = "",
        state_id: str = "",
    ) -> RobotMotionState:
        now = self.get_clock().now().to_msg()
        stamp = now if stamp_is_zero(joint_state.header.stamp) else joint_state.header.stamp
        out = RobotMotionState()
        out.context.request_id = request_id
        out.context.stamp = stamp
        out.context.frame_id = frame_id or self.frame_id
        out.context.scene_id = scene_id or self.scene_id
        out.context.state_id = state_id or self.make_state_id(stamp, source)
        out.source = source
        out.authoritative = authoritative
        out.joint_state = joint_state
        out.joint_state.header.stamp = stamp
        return out

    def publish_state(self, state: RobotMotionState) -> None:
        self.latest_state = state
        self.publisher.publish(state)

    def on_joint_state(self, joint_state: JointState) -> None:
        self.publish_state(
            self.make_state(
                joint_state,
                source=self.source,
                authoritative=self.authoritative,
            )
        )

    def on_set_state(self, request, response):
        self.status.mark_running("set_state request")
        if not request.joint_state.name:
            response.success = False
            response.message = "joint_state.name is empty"
            self.status.mark_done(False, response.message)
            return response
        if len(request.joint_state.position) < len(request.joint_state.name):
            response.success = False
            response.message = "joint_state.position shorter than name"
            self.status.mark_done(False, response.message)
            return response

        source = request.source or "explicit_set_state"
        state = self.make_state(
            request.joint_state,
            source=source,
            authoritative=bool(request.authoritative),
            request_id=request.context.request_id,
            frame_id=request.context.frame_id,
            scene_id=request.context.scene_id,
            state_id=request.context.state_id,
        )
        self.publish_state(state)
        response.success = True
        response.message = f"state fixed: state_id={state.context.state_id} joints={len(state.joint_state.name)}"
        response.state = state
        self.status.mark_done(True, response.message)
        return response

    def publish_latest(self) -> None:
        if self.latest_state is not None:
            self.publisher.publish(self.latest_state)


def main() -> None:
    rclpy.init()
    node = MotionStateSourceNode()
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
