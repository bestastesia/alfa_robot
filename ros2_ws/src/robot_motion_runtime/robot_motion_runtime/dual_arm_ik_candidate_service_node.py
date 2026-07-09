from __future__ import annotations

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_motion_interfaces.srv import PlanDualArmIk, SolveArmIk
from robot_motion_runtime.common import RuntimeStatusPublisher, joint_distance, merge_joint_state


class DualArmIkCandidateServiceNode(Node):
    """Generate dual-arm candidate states from explicit left/right target poses."""

    def __init__(self) -> None:
        super().__init__("dual_arm_ik_candidate_service")
        self.declare_parameter("service_name", "/robot_motion/plan_dual_arm_ik")
        self.declare_parameter("solve_arm_ik_service", "/robot_motion/solve_arm_ik")
        self.declare_parameter("service_timeout_s", 5.0)
        self.declare_parameter("default_position_tolerance", 0.05)
        self.declare_parameter("default_orientation_tolerance", 0.1)
        self.declare_parameter("default_max_solutions_per_arm", 8)
        self.declare_parameter("default_max_combined_candidates", 64)

        self.service_name = str(self.get_parameter("service_name").value)
        self.solve_arm_ik_service = str(self.get_parameter("solve_arm_ik_service").value)
        self.service_timeout_s = float(self.get_parameter("service_timeout_s").value)
        self.default_position_tolerance = float(self.get_parameter("default_position_tolerance").value)
        self.default_orientation_tolerance = float(self.get_parameter("default_orientation_tolerance").value)
        self.default_max_solutions_per_arm = int(self.get_parameter("default_max_solutions_per_arm").value)
        self.default_max_combined_candidates = int(self.get_parameter("default_max_combined_candidates").value)

        self.callback_group = ReentrantCallbackGroup()
        self.solve_client = self.create_client(
            SolveArmIk,
            self.solve_arm_ik_service,
            callback_group=self.callback_group,
        )
        self.service = self.create_service(
            PlanDualArmIk,
            self.service_name,
            self.on_plan_dual_arm_ik,
            callback_group=self.callback_group,
        )
        self.status = RuntimeStatusPublisher(
            self,
            self.service_name,
            "PlanDualArmIk service; calls SolveArmIk for both arms and combines candidate states",
        )
        self.status.mark_ready(f"waiting for targets; solve_service={self.solve_arm_ik_service}")
        self.get_logger().info(
            f"PlanDualArmIk service ready: service={self.service_name} solve={self.solve_arm_ik_service}"
        )

    def call_solve_arm_ik(self, request: SolveArmIk.Request) -> SolveArmIk.Response:
        if not self.solve_client.wait_for_service(timeout_sec=self.service_timeout_s):
            raise TimeoutError(f"SolveArmIk service not available: {self.solve_arm_ik_service}")
        event = threading.Event()
        holder = {}
        future = self.solve_client.call_async(request)

        def on_done(done_future):
            try:
                holder["response"] = done_future.result()
            except Exception as exc:  # pragma: no cover - defensive runtime path
                holder["error"] = exc
            event.set()

        future.add_done_callback(on_done)
        if not event.wait(timeout=self.service_timeout_s):
            raise TimeoutError("SolveArmIk call timed out")
        if "error" in holder:
            raise RuntimeError(f"SolveArmIk call failed: {holder['error']}")
        return holder["response"]

    def solve_side(self, request, side: str, top_suction: bool) -> SolveArmIk.Response:
        solve_request = SolveArmIk.Request()
        solve_request.context = request.context
        solve_request.side = side
        solve_request.target = request.left_target if side == "left" else request.right_target
        solve_request.seed_state = request.seed_state
        solve_request.fixed_updown = request.fixed_updown
        solve_request.top_suction = top_suction
        solve_request.position_tolerance = (
            request.position_tolerance if request.position_tolerance >= 1e-6 else self.default_position_tolerance
        )
        solve_request.orientation_tolerance = (
            request.orientation_tolerance if request.orientation_tolerance >= 1e-6 else self.default_orientation_tolerance
        )
        solve_request.max_solutions = int(request.max_solutions_per_arm or self.default_max_solutions_per_arm)
        return self.call_solve_arm_ik(solve_request)

    @staticmethod
    def seed_or_fixed_updown(seed_state: JointState, fixed_updown: float) -> JointState:
        if seed_state.name:
            return seed_state
        seed = JointState()
        seed.name = ["updown"]
        seed.position = [fixed_updown]
        return seed

    def combine_solutions(
        self,
        seed_state: JointState,
        fixed_updown: float,
        left_solutions: list[JointState],
        right_solutions: list[JointState],
        max_candidates: int,
    ) -> list[JointState]:
        base = self.seed_or_fixed_updown(seed_state, fixed_updown)
        combined: list[JointState] = []
        for left in left_solutions:
            left_merged = merge_joint_state(base, left)
            for right in right_solutions:
                combined.append(merge_joint_state(left_merged, right))
        combined.sort(key=lambda state: joint_distance(base, state))
        return combined[:max(1, max_candidates)]

    def on_plan_dual_arm_ik(self, request, response):
        started = time.monotonic()
        self.status.mark_running(
            f"h={request.fixed_updown:.3f} left_top={request.left_top_suction} right_top={request.right_top_suction}"
        )
        try:
            left_response = self.solve_side(request, "left", request.left_top_suction)
            right_response = self.solve_side(request, "right", request.right_top_suction)
            response.left_solution_count = len(left_response.solutions)
            response.right_solution_count = len(right_response.solutions)
            if not left_response.success:
                response.success = False
                response.message = "left SolveArmIk failed: " + left_response.message
                self.status.mark_done(False, response.message)
                return response
            if not right_response.success:
                response.success = False
                response.message = "right SolveArmIk failed: " + right_response.message
                self.status.mark_done(False, response.message)
                return response

            max_candidates = int(request.max_combined_candidates or self.default_max_combined_candidates)
            response.candidate_states = self.combine_solutions(
                request.seed_state,
                request.fixed_updown,
                list(left_response.solutions),
                list(right_response.solutions),
                max_candidates,
            )
            response.success = bool(response.candidate_states)
            response.message = (
                f"combined {len(response.candidate_states)} candidates from "
                f"left={response.left_solution_count} right={response.right_solution_count} "
                f"in {(time.monotonic() - started) * 1000.0:.2f}ms"
            )
            self.status.mark_done(response.success, response.message)
            return response
        except Exception as exc:  # pragma: no cover - runtime safety path
            response.success = False
            response.message = str(exc)
            self.status.mark_done(False, response.message)
            return response


def main() -> None:
    rclpy.init()
    node = DualArmIkCandidateServiceNode()
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
