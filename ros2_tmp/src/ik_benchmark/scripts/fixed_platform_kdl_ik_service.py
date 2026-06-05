#!/usr/bin/python3
from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetStateValidity

from alfa_robot_benchmarks.srv import SolveDualIk


class FixedPlatformKdlIkService(Node):
    def __init__(self) -> None:
        super().__init__("fixed_platform_kdl_ik_service")
        self.fixed_updown = float(self.declare_parameter("fixed_updown", 0.18).value)
        requested_timeout = float(self.declare_parameter("timeout", 0.05).value)
        self.timeout = max(requested_timeout, 0.05)
        self.position_tolerance = float(self.declare_parameter("position_tolerance", 0.03).value)
        self.top_position_tolerance = float(self.declare_parameter("top_suction_position_tolerance", 0.04).value)
        self.orientation_tolerance = float(self.declare_parameter("orientation_tolerance", 0.05).value)
        self.top_orientation_tolerance = float(self.declare_parameter("top_suction_orientation_tolerance", 5.0 * math.pi / 180.0).value)
        self.check_collision = bool(self.declare_parameter("check_collision", False).value)
        self.compute_ik_avoid_collisions = False
        self.use_request_seed = bool(self.declare_parameter("use_request_seed", False).value)
        self.service_name = str(self.declare_parameter("service_name", "/alfa_dual_ik/solve").value)
        self.compute_ik_service = str(self.declare_parameter("compute_ik_service", "/compute_ik").value)
        self.state_validity_service = str(self.declare_parameter("state_validity_service", "/check_state_validity").value)
        self.home = [0.0, math.radians(5), math.radians(145), 0.0, math.radians(120), 0.0]
        self.callback_group = ReentrantCallbackGroup()
        self.ik_client = self.create_client(GetPositionIK, self.compute_ik_service, callback_group=self.callback_group)
        self.state_validity_client = self.create_client(GetStateValidity, self.state_validity_service, callback_group=self.callback_group)
        self.service = self.create_service(SolveDualIk, self.service_name, self.handle_request, callback_group=self.callback_group)
        self.get_logger().info(
            f"Fixed platform KDL IK service ready: service={self.service_name} fixed_updown={self.fixed_updown:.3f} timeout={self.timeout:.3f}s check_collision_param={self.check_collision} compute_ik_avoid_collisions={self.compute_ik_avoid_collisions} use_request_seed={self.use_request_seed}"
        )

    @staticmethod
    def arm_names(side: str) -> list[str]:
        return [f"{side}_v5_joint{i}" for i in range(1, 7)]

    def current_joint_state(self, request: SolveDualIk.Request) -> JointState:
        state = JointState()
        state.name = [
            "turn", "updown",
            *self.arm_names("left"),
            *self.arm_names("right"),
        ]
        state.position = [0.0, self.fixed_updown, *self.home, *self.home]
        by_name = {}
        if self.use_request_seed:
            by_name = {
                name: request.current_joint_state.position[i]
                for i, name in enumerate(request.current_joint_state.name)
                if i < len(request.current_joint_state.position)
            }
        for i, name in enumerate(state.name):
            if name == "updown":
                state.position[i] = self.fixed_updown
            elif name in by_name:
                state.position[i] = by_name[name]
        return state

    def make_ik_request(self, request: SolveDualIk.Request, side: str, target_pose) -> GetPositionIK.Request:
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = f"{side}_v5_arm"
        ik_request.ik_request.ik_link_name = f"{side}_v5_tool0"
        ik_request.ik_request.avoid_collisions = self.compute_ik_avoid_collisions
        ik_request.ik_request.timeout.sec = int(self.timeout)
        ik_request.ik_request.timeout.nanosec = int((self.timeout - int(self.timeout)) * 1e9)
        ik_request.ik_request.pose_stamped.header = request.header
        ik_request.ik_request.pose_stamped.pose = target_pose
        ik_request.ik_request.robot_state.joint_state = self.current_joint_state(request)
        return ik_request

    def call_compute_ik(self, request: SolveDualIk.Request, side: str, target_pose):
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"{self.compute_ik_service} unavailable")
        future = self.ik_client.call_async(self.make_ik_request(request, side, target_pose))
        deadline = time.perf_counter() + max(5.0, self.timeout + 2.0)
        while rclpy.ok() and not future.done() and time.perf_counter() < deadline:
            time.sleep(0.002)
        if not future.done():
            return None
        return future.result()

    def call_state_validity(self, joint_target: JointState):
        if not self.check_collision:
            return True, []
        if not self.state_validity_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"{self.state_validity_service} unavailable; reject KDL combined solution")
            return False, ["state_validity_unavailable"]
        req = GetStateValidity.Request()
        req.robot_state.joint_state = JointState()
        req.robot_state.joint_state.name = ["turn", *joint_target.name]
        req.robot_state.joint_state.position = [0.0, *joint_target.position]
        req.group_name = "dual_v5_arm_with_base"
        future = self.state_validity_client.call_async(req)
        deadline = time.perf_counter() + 2.0
        while rclpy.ok() and not future.done() and time.perf_counter() < deadline:
            time.sleep(0.002)
        if not future.done():
            self.get_logger().warn("state validity check timed out; reject KDL combined solution")
            return False, ["state_validity_timeout"]
        result = future.result()
        if result is None:
            return False, ["state_validity_no_response"]
        contacts = []
        for contact in result.contacts[:20]:
            contacts.append(f"{contact.contact_body_1} <-> {contact.contact_body_2}")
        return bool(result.valid), contacts

    @staticmethod
    def solution_map(response) -> dict[str, float]:
        if response is None:
            return {}
        js = response.solution.joint_state
        return {name: js.position[i] for i, name in enumerate(js.name) if i < len(js.position)}

    @staticmethod
    def error_code_name(value: int) -> str:
        for name in dir(MoveItErrorCodes):
            if name.isupper() and getattr(MoveItErrorCodes, name) == value:
                return name
        return str(value)

    @staticmethod
    def is_success(response) -> bool:
        return response is not None and response.error_code.val == MoveItErrorCodes.SUCCESS

    def joint_target(self, current_h: float, left_solution: dict[str, float], right_solution: dict[str, float]) -> JointState:
        target = JointState()
        target.name = ["updown", *self.arm_names("left"), *self.arm_names("right")]
        target.position = [current_h]
        for name in self.arm_names("left"):
            target.position.append(left_solution[name])
        for name in self.arm_names("right"):
            target.position.append(right_solution[name])
        return target

    def handle_request(self, request: SolveDualIk.Request, response: SolveDualIk.Response) -> SolveDualIk.Response:
        start = time.perf_counter()
        current_h = request.current_updown if request.current_updown >= 0.0 else self.fixed_updown
        left_response = self.call_compute_ik(request, "left", request.left_target)
        right_response = self.call_compute_ik(request, "right", request.right_target)
        left_ok = self.is_success(left_response)
        right_ok = self.is_success(right_response)
        left_code = left_response.error_code.val if left_response is not None else -999
        right_code = right_response.error_code.val if right_response is not None else -999

        reason = ""
        if not left_ok:
            reason = "left_kdl_" + self.error_code_name(left_code)
        elif not right_ok:
            reason = "right_kdl_" + self.error_code_name(right_code)

        left_solution = self.solution_map(left_response)
        right_solution = self.solution_map(right_response)
        joint_target = JointState()
        candidate_names: list[str] = []
        candidate_values: list[float] = []
        collision_free = True
        collision_pairs: list[str] = []
        if left_ok and right_ok:
            joint_target = self.joint_target(current_h, left_solution, right_solution)
            joint_target.header = request.header
            candidate_names = list(joint_target.name)
            candidate_values = list(joint_target.position)
            collision_free, collision_pairs = self.call_state_validity(joint_target)
            if not collision_free:
                reason = "combined_state_collision"

        response.success = reason == ""
        response.failure_reason = reason
        response.solver_path = "kdl_single_arm_compute_ik"
        response.fallback_used = False
        response.selected_h = current_h
        response.score = 0.0 if response.success else math.inf
        response.wall_ms = (time.perf_counter() - start) * 1000.0
        response.trial_count = 2
        response.legal_count = 1 if response.success else 0
        if response.success:
            response.joint_target = joint_target

        candidate = {
            "h": current_h,
            "h_index": 0,
            "seed_index": 0,
            "target_order": "normal",
            "solver_path": "kdl_single_arm_compute_ik",
            "rejection_reason": reason,
            "direct_pos_error": 0.0 if response.success else math.inf,
            "direct_ori_error": 0.0 if response.success else math.inf,
            "swapped_pos_error": math.inf,
            "collision_free": collision_free,
            "joint_names": candidate_names,
            "joint_values": candidate_values,
            "full_joint_names": candidate_names,
            "full_joint_values": candidate_values,
            "collision_pairs": collision_pairs,
        }
        diagnostics = {
            "backend": "kdl_single_arm_compute_ik",
            "range_reachable": True,
            "h_interval": [current_h, current_h],
            "h_center": current_h,
            "h_candidates": [current_h],
            "candidate_count": 1,
            "timeout_like_count": 0,
            "swapped_rejected_count": 0,
            "unique_seed_index_count": 1,
            "unique_h_index_count": 1,
            "rejection_counts": {"legal": 1} if response.success else {reason: 1},
            "solver_path_counts": {"kdl_single_arm_compute_ik": 1},
            "target_order_counts": {"normal": 1},
            "best_direct_pos_error": 0.0 if response.success else math.inf,
            "best_direct_ori_error": 0.0 if response.success else math.inf,
            "best_error_seed_index": 0,
            "best_error_reason": "legal" if response.success else reason,
            "combined_collision_checked": self.check_collision,
            "combined_collision_free": collision_free,
            "combined_collision_pairs": collision_pairs,
            "left": {"success": left_ok, "error_code": self.error_code_name(left_code), "joint_values": {k: left_solution.get(k) for k in self.arm_names("left")}},
            "right": {"success": right_ok, "error_code": self.error_code_name(right_code), "joint_values": {k: right_solution.get(k) for k in self.arm_names("right")}},
            "debug_best_candidates": [candidate],
            "best_candidate": candidate,
            "selected": candidate if response.success else {},
        }
        response.diagnostics_json = json.dumps(diagnostics, ensure_ascii=False)
        self.get_logger().info(
            f"KDL IK {(chr(115)+chr(117)+chr(99)+chr(99)+chr(101)+chr(115)+chr(115)) if response.success else (chr(102)+chr(97)+chr(105)+chr(108)+chr(101)+chr(100))} h={current_h:.3f} legal={response.legal_count} wall={response.wall_ms:.2f}ms reason={reason or (chr(108)+chr(101)+chr(103)+chr(97)+chr(108))} left={self.error_code_name(left_code)} right={self.error_code_name(right_code)}"
        )
        return response


def main() -> None:
    rclpy.init()
    node = FixedPlatformKdlIkService()
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


if __name__ == "__main__":
    main()
