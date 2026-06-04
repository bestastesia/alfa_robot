"""Standalone safety process for the ALFA PLC execution layer."""

from __future__ import annotations

import threading
import time
from enum import Enum

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class SafetyState(str, Enum):
    READY = 'ready'
    SOFT_STOPPED = 'soft_stopped'
    EMERGENCY_STOPPED = 'emergency_stopped'
    RESETTING = 'resetting'
    ERROR = 'error'


class PlcSafetyNode(Node):
    """Safety layer process.

    This node owns the external safety API. It does not write Modbus directly;
    PLC ownership stays in plc_bridge_node. Safety requests are forwarded to the
    execution layer services so there is only one PLC writer process.
    """

    def __init__(self) -> None:
        super().__init__('plc_safety_node')
        self.declare_parameter('execution_soft_stop_service', '/plc_soft_stop')
        self.declare_parameter('execution_emergency_stop_service', '/plc_emergency_stop')
        self.declare_parameter('execution_reset_emergency_service', '/plc_reset_emergency')
        self.declare_parameter('execution_reset_fault_service', '/plc_reset_fault')
        self.declare_parameter('execution_clear_commands_service', '/plc_clear_commands')
        self.declare_parameter('service_timeout_s', 3.0)
        self._state = SafetyState.READY
        self._last_error = ''
        self._lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()

        self._execution_clients = {
            'soft_stop': self.create_client(
                Trigger,
                str(self.get_parameter('execution_soft_stop_service').value),
                callback_group=self._callback_group,
            ),
            'emergency_stop': self.create_client(
                Trigger,
                str(self.get_parameter('execution_emergency_stop_service').value),
                callback_group=self._callback_group,
            ),
            'reset_emergency': self.create_client(
                Trigger,
                str(self.get_parameter('execution_reset_emergency_service').value),
                callback_group=self._callback_group,
            ),
            'reset_fault': self.create_client(
                Trigger,
                str(self.get_parameter('execution_reset_fault_service').value),
                callback_group=self._callback_group,
            ),
            'clear_commands': self.create_client(
                Trigger,
                str(self.get_parameter('execution_clear_commands_service').value),
                callback_group=self._callback_group,
            ),
        }

        self.state_pub = self.create_publisher(String, '/alfa_safety/state', 10)
        self.soft_stop_srv = self.create_service(
            Trigger,
            '/alfa_safety/soft_stop',
            self._on_soft_stop,
            callback_group=self._callback_group,
        )
        self.emergency_stop_srv = self.create_service(
            Trigger,
            '/alfa_safety/emergency_stop',
            self._on_emergency_stop,
            callback_group=self._callback_group,
        )
        self.reset_srv = self.create_service(
            Trigger,
            '/alfa_safety/reset',
            self._on_reset,
            callback_group=self._callback_group,
        )
        self.clear_srv = self.create_service(
            Trigger,
            '/alfa_safety/clear_commands',
            self._on_clear_commands,
            callback_group=self._callback_group,
        )
        self.timer = self.create_timer(0.2, self._publish_state, callback_group=self._callback_group)
        self.get_logger().info('PLC safety node started; forwarding safety requests to execution layer')

    def _on_soft_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success, message = self._call_execution_service('soft_stop')
        self._set_state(SafetyState.SOFT_STOPPED if success else SafetyState.ERROR, '' if success else message)
        response.success = success
        response.message = message
        return response

    def _on_emergency_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success, message = self._call_execution_service('emergency_stop')
        self._set_state(SafetyState.EMERGENCY_STOPPED if success else SafetyState.ERROR, '' if success else message)
        response.success = success
        response.message = message
        return response

    def _on_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._set_state(SafetyState.RESETTING)
        emergency_ok, emergency_message = self._call_execution_service('reset_emergency')
        fault_ok, fault_message = self._call_execution_service('reset_fault') if emergency_ok else (False, 'skipped reset_fault')
        success = emergency_ok and fault_ok
        message = f'reset_emergency: {emergency_message}; reset_fault: {fault_message}'
        self._set_state(SafetyState.READY if success else SafetyState.ERROR, '' if success else message)
        response.success = success
        response.message = message
        return response

    def _on_clear_commands(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success, message = self._call_execution_service('clear_commands')
        self._set_state(SafetyState.SOFT_STOPPED if success else SafetyState.ERROR, '' if success else message)
        response.success = success
        response.message = message
        return response

    def _call_execution_service(self, name: str) -> tuple[bool, str]:
        client = self._execution_clients[name]
        timeout_s = float(self.get_parameter('service_timeout_s').value)
        if not client.wait_for_service(timeout_sec=timeout_s):
            return False, f'execution service unavailable: {client.srv_name}'
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return False, f'timed out calling execution service: {client.srv_name}'
        result = future.result()
        if result is None:
            return False, f'execution service returned no result: {client.srv_name}'
        return bool(result.success), str(result.message)

    def _set_state(self, state: SafetyState, error: str = '') -> None:
        with self._lock:
            self._state = state
            self._last_error = error
        self._publish_state()

    def _publish_state(self) -> None:
        with self._lock:
            state = self._state
            error = self._last_error
        msg = String()
        msg.data = f'state={state.value};last_error={error}'
        self.state_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PlcSafetyNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
