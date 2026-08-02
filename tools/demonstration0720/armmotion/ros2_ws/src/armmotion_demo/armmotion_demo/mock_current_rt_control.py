from __future__ import annotations

import threading
import time

import rclpy
from alfa_robot_execution_bridge.joints import RT_CONTROL_JOINT_NAMES, model_to_rt_control_position
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.msg import PlcIoState
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from .common import loaded_joint_map


class MockCurrentRtControl(Node):
    """Non-hardware mock of the currently deployed rt-control wire."""

    def __init__(self) -> None:
        super().__init__("mock_current_rt_control")
        self.declare_parameter("trajectory_action", "/dual_arm_jtc/follow_joint_trajectory")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("plc_state_topic", "/plc/io_state")
        self.declare_parameter("execution_time_scale", 0.0)
        self._callback_group = ReentrantCallbackGroup()
        model_loaded = loaded_joint_map(
            [name for name in RT_CONTROL_JOINT_NAMES if name != "updown"]
        )
        model_loaded["updown"] = 0.3
        self._positions = {
            name: model_to_rt_control_position(name, model_loaded[name])
            for name in RT_CONTROL_JOINT_NAMES
        }
        self._lock = threading.Lock()
        self._left_solenoid = False
        self._right_solenoid = False
        self._vacuum_pump = False
        self._trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("trajectory_action").value),
            execute_callback=self._execute_trajectory,
            goal_callback=self._validate_trajectory,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._callback_group,
        )
        self.create_service(
            SetBool,
            "/plc/left_solenoid",
            lambda request, response: self._set_output("left", request, response),
            callback_group=self._callback_group,
        )
        self.create_service(
            SetBool,
            "/plc/right_solenoid",
            lambda request, response: self._set_output("right", request, response),
            callback_group=self._callback_group,
        )
        self.create_service(
            SetBool,
            "/plc/vacuum_pump",
            lambda request, response: self._set_output("pump", request, response),
            callback_group=self._callback_group,
        )
        self._joint_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            10,
        )
        self._plc_pub = self.create_publisher(
            PlcIoState,
            str(self.get_parameter("plc_state_topic").value),
            10,
        )
        self.create_timer(0.02, self._publish_joint_state)
        self.create_timer(0.1, self._publish_plc_state)
        self.get_logger().warning("已启动非硬件 rt-control Mock，禁止把该节点用于实机")

    def _validate_trajectory(self, request) -> GoalResponse:
        names = list(request.trajectory.joint_names)
        if names != list(RT_CONTROL_JOINT_NAMES) or not request.trajectory.points:
            return GoalResponse.REJECT
        if any(len(point.positions) != len(names) for point in request.trajectory.points):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_trajectory(self, goal_handle):
        result = FollowJointTrajectory.Result()
        points = list(goal_handle.request.trajectory.points)
        duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
        scale = max(0.0, float(self.get_parameter("execution_time_scale").value))
        if scale > 0.0:
            deadline = time.monotonic() + duration * scale
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "mock canceled"
                    return result
                time.sleep(0.01)
        with self._lock:
            self._positions = dict(
                zip(goal_handle.request.trajectory.joint_names, points[-1].positions)
            )
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "mock trajectory accepted"
        goal_handle.succeed()
        return result

    def _set_output(self, output: str, request, response):
        with self._lock:
            if output == "left":
                self._left_solenoid = bool(request.data)
            elif output == "right":
                self._right_solenoid = bool(request.data)
            else:
                self._vacuum_pump = bool(request.data)
        response.success = True
        response.message = f"mock {output}={bool(request.data)}"
        return response

    def _publish_joint_state(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(RT_CONTROL_JOINT_NAMES)
        with self._lock:
            message.position = [self._positions[name] for name in message.name]
        self._joint_pub.publish(message)

    def _publish_plc_state(self) -> None:
        message = PlcIoState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "plc"
        message.connected = True
        message.data_fresh = True
        with self._lock:
            message.left_solenoid_on = self._left_solenoid
            message.right_solenoid_on = self._right_solenoid
            message.vacuum_pump_on = self._vacuum_pump
            message.left_vacuum_established = self._left_solenoid and self._vacuum_pump
            message.right_vacuum_established = self._right_solenoid and self._vacuum_pump
        self._plc_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockCurrentRtControl()
    executor = MultiThreadedExecutor(num_threads=4)
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
