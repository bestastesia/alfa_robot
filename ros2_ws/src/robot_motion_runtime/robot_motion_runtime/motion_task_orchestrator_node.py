from __future__ import annotations

import threading
import time
from typing import Any

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_motion_interfaces.msg import MotionPlanCandidate
from robot_motion_interfaces.srv import (
    ExecuteTrajectory,
    PlanDualArmIk,
    PlanExtract,
    PlanLoaded,
    RunDualArmPoseTask,
    RunMotionTask,
)
from robot_motion_runtime.common import RuntimeStatusPublisher, concatenate_trajectories


class MotionTaskOrchestratorNode(Node):
    """Minimal service-composed task runtime.

    This node is deliberately thin: it does not own IK, extraction planning, loaded planning,
    or execution. It proves the runtime shape requested by the architecture goal:

    set RobotMotionState once -> send one task request -> services execute the chain.
    """

    def __init__(self) -> None:
        super().__init__("motion_task_orchestrator")
        self.declare_parameter("service_name", "/robot_motion/run_task")
        self.declare_parameter("pose_task_service_name", "/robot_motion/run_dual_arm_pose_task")
        self.declare_parameter("plan_dual_arm_ik_service", "/robot_motion/plan_dual_arm_ik")
        self.declare_parameter("plan_extract_service", "/robot_motion/plan_extract")
        self.declare_parameter("plan_loaded_service", "/robot_motion/plan_loaded")
        self.declare_parameter("execute_trajectory_service", "/robot_motion/execute_trajectory")
        self.declare_parameter("service_timeout_s", 10.0)

        self.service_name = str(self.get_parameter("service_name").value)
        self.pose_task_service_name = str(self.get_parameter("pose_task_service_name").value)
        self.plan_dual_arm_ik_service = str(self.get_parameter("plan_dual_arm_ik_service").value)
        self.plan_extract_service = str(self.get_parameter("plan_extract_service").value)
        self.plan_loaded_service = str(self.get_parameter("plan_loaded_service").value)
        self.execute_trajectory_service = str(self.get_parameter("execute_trajectory_service").value)
        self.service_timeout_s = float(self.get_parameter("service_timeout_s").value)

        self.callback_group = ReentrantCallbackGroup()
        self.plan_dual_arm_ik_client = self.create_client(
            PlanDualArmIk,
            self.plan_dual_arm_ik_service,
            callback_group=self.callback_group,
        )
        self.plan_extract_client = self.create_client(
            PlanExtract,
            self.plan_extract_service,
            callback_group=self.callback_group,
        )
        self.plan_loaded_client = self.create_client(
            PlanLoaded,
            self.plan_loaded_service,
            callback_group=self.callback_group,
        )
        self.execute_client = self.create_client(
            ExecuteTrajectory,
            self.execute_trajectory_service,
            callback_group=self.callback_group,
        )
        self.service = self.create_service(
            RunMotionTask,
            self.service_name,
            self.on_run_task,
            callback_group=self.callback_group,
        )
        self.pose_task_service = self.create_service(
            RunDualArmPoseTask,
            self.pose_task_service_name,
            self.on_run_dual_arm_pose_task,
            callback_group=self.callback_group,
        )
        self.status = RuntimeStatusPublisher(
            self,
            self.service_name,
            "runtime task orchestrator; composes PlanExtract -> PlanLoaded -> ExecuteTrajectory",
        )
        self.pose_status = RuntimeStatusPublisher(
            self,
            self.pose_task_service_name,
            "runtime target-pose orchestrator; composes PlanDualArmIk -> PlanExtract -> PlanLoaded -> ExecuteTrajectory",
        )
        self.status.mark_ready("waiting for RunMotionTask requests")
        self.pose_status.mark_ready("waiting for RunDualArmPoseTask requests")
        self.get_logger().info(f"Motion task orchestrator ready: {self.service_name}")
        self.get_logger().info(f"Motion pose task orchestrator ready: {self.pose_task_service_name}")

    def call_service(self, client, request: Any, label: str) -> Any:
        if not client.wait_for_service(timeout_sec=self.service_timeout_s):
            raise TimeoutError(f"{label} service not available")
        event = threading.Event()
        holder: dict[str, Any] = {}
        future = client.call_async(request)

        def done_callback(done_future):
            try:
                holder["response"] = done_future.result()
            except Exception as exc:  # pragma: no cover - defensive runtime path
                holder["error"] = exc
            event.set()

        future.add_done_callback(done_callback)
        if not event.wait(timeout=self.service_timeout_s):
            raise TimeoutError(f"{label} call timed out")
        if "error" in holder:
            raise RuntimeError(f"{label} call failed: {holder['error']}")
        return holder["response"]

    @staticmethod
    def first_success(candidates: list[MotionPlanCandidate]) -> MotionPlanCandidate | None:
        for candidate in candidates:
            if candidate.success:
                return candidate
        return candidates[0] if candidates else None

    def run_candidate_chain(
        self,
        *,
        context,
        seed_state,
        ik_candidate_states,
        attached_boxes,
        scene_objects,
        attached_collision_objects,
        loaded_goal_family,
        candidate_limit,
        planning_mode,
        execute,
        dry_run,
        velocity_scale,
        acceleration_scale,
        strategy=None,
    ) -> dict[str, Any]:
        started = time.monotonic()
        if not ik_candidate_states:
            return {
                "success": False,
                "message": "ik_candidate_states is empty",
                "extract_candidates": [],
                "loaded_candidates": [],
                "selected_extract": MotionPlanCandidate(),
                "selected_loaded": MotionPlanCandidate(),
            }

        extract_request = PlanExtract.Request()
        extract_request.context = context
        extract_request.seed_state = seed_state
        extract_request.ik_candidate_states = ik_candidate_states
        extract_request.attached_boxes = attached_boxes
        if strategy is not None:
            extract_request.strategy = strategy
        extract_request.scene_objects = scene_objects
        extract_request.attached_collision_objects = attached_collision_objects
        extract_request.candidate_limit = candidate_limit
        extract_request.dual_arm = True
        extract_response = self.call_service(
            self.plan_extract_client, extract_request, "PlanExtract"
        )
        extract_candidates = list(extract_response.candidates)
        selected_extract = self.first_success(extract_candidates)
        if not extract_response.success or selected_extract is None:
            return {
                "success": False,
                "message": f"PlanExtract failed: {extract_response.message}",
                "extract_candidates": extract_candidates,
                "loaded_candidates": [],
                "selected_extract": selected_extract or MotionPlanCandidate(),
                "selected_loaded": MotionPlanCandidate(),
            }

        loaded_request = PlanLoaded.Request()
        loaded_request.context = context
        loaded_request.extract_goal_states = [
            candidate.goal_state for candidate in extract_candidates if candidate.success
        ]
        loaded_request.attached_boxes = attached_boxes
        loaded_request.scene_objects = scene_objects
        loaded_request.attached_collision_objects = attached_collision_objects
        loaded_request.loaded_goal_family = loaded_goal_family
        loaded_request.planning_mode = planning_mode
        loaded_request.candidate_limit = candidate_limit
        loaded_request.parallel_workers = 1
        loaded_response = self.call_service(
            self.plan_loaded_client, loaded_request, "PlanLoaded"
        )
        loaded_candidates = list(loaded_response.candidates)
        selected_loaded = self.first_success(loaded_candidates)
        if not loaded_response.success or selected_loaded is None:
            return {
                "success": False,
                "message": f"PlanLoaded failed: {loaded_response.message}",
                "extract_candidates": extract_candidates,
                "loaded_candidates": loaded_candidates,
                "selected_extract": selected_extract,
                "selected_loaded": selected_loaded or MotionPlanCandidate(),
            }

        if execute:
            execute_request = ExecuteTrajectory.Request()
            execute_request.context = context
            try:
                execute_request.trajectory = concatenate_trajectories(
                    selected_extract.trajectory,
                    selected_loaded.trajectory,
                )
            except ValueError as exc:
                return {
                    "success": False,
                    "message": f"failed to compose execution trajectory: {exc}",
                    "extract_candidates": extract_candidates,
                    "loaded_candidates": loaded_candidates,
                    "selected_extract": selected_extract,
                    "selected_loaded": selected_loaded,
                }
            execute_request.dry_run = dry_run
            execute_request.velocity_scale = velocity_scale
            execute_request.acceleration_scale = acceleration_scale
            execute_response = self.call_service(
                self.execute_client, execute_request, "ExecuteTrajectory"
            )
            if not execute_response.accepted:
                return {
                    "success": False,
                    "message": f"ExecuteTrajectory rejected: {execute_response.message}",
                    "extract_candidates": extract_candidates,
                    "loaded_candidates": loaded_candidates,
                    "selected_extract": selected_extract,
                    "selected_loaded": selected_loaded,
                }

        elapsed_ms = (time.monotonic() - started) * 1000.0
        return {
            "success": True,
            "message": (
                f"task chain complete in {elapsed_ms:.2f}ms; "
                f"extract={len(extract_candidates)} loaded={len(loaded_candidates)} "
                f"execute={execute}"
            ),
            "extract_candidates": extract_candidates,
            "loaded_candidates": loaded_candidates,
            "selected_extract": selected_extract,
            "selected_loaded": selected_loaded,
        }

    def apply_chain_result(self, response, result: dict[str, Any]):
        response.success = bool(result["success"])
        response.message = str(result["message"])
        response.extract_candidates = list(result["extract_candidates"])
        response.loaded_candidates = list(result["loaded_candidates"])
        response.selected_extract = result["selected_extract"]
        response.selected_loaded = result["selected_loaded"]

    def on_run_task(self, request, response):
        self.status.mark_running(
            f"ik_candidates={len(request.ik_candidate_states)} execute={request.execute} dry_run={request.dry_run}"
        )
        try:
            result = self.run_candidate_chain(
                context=request.context,
                seed_state=request.seed_state,
                ik_candidate_states=list(request.ik_candidate_states),
                attached_boxes=list(request.attached_boxes),
                scene_objects=list(request.scene_objects),
                attached_collision_objects=list(request.attached_collision_objects),
                loaded_goal_family=list(request.loaded_goal_family),
                candidate_limit=request.candidate_limit,
                planning_mode=request.planning_mode,
                execute=request.execute,
                dry_run=request.dry_run,
                velocity_scale=request.velocity_scale,
                acceleration_scale=request.acceleration_scale,
                strategy=None,
            )
            self.apply_chain_result(response, result)
            self.status.mark_done(response.success, response.message)
            return response
        except Exception as exc:  # pragma: no cover - runtime safety path
            response.success = False
            response.message = str(exc)
            self.status.mark_done(False, response.message)
            return response

    def on_run_dual_arm_pose_task(self, request, response):
        started = time.monotonic()
        self.pose_status.mark_running(
            f"h={request.fixed_updown:.3f} execute={request.execute} dry_run={request.dry_run}"
        )
        try:
            ik_request = PlanDualArmIk.Request()
            ik_request.context = request.context
            ik_request.left_target = request.left_target
            ik_request.right_target = request.right_target
            ik_request.seed_state = request.seed_state
            ik_request.fixed_updown = request.fixed_updown
            ik_request.left_top_suction = request.left_top_suction
            ik_request.right_top_suction = request.right_top_suction
            ik_request.position_tolerance = request.position_tolerance if request.position_tolerance >= 1e-6 else 0.0
            ik_request.orientation_tolerance = (
                request.orientation_tolerance if request.orientation_tolerance >= 1e-6 else 0.0
            )
            ik_request.max_solutions_per_arm = request.max_solutions_per_arm
            ik_request.max_combined_candidates = request.candidate_limit
            ik_response = self.call_service(
                self.plan_dual_arm_ik_client, ik_request, "PlanDualArmIk"
            )
            response.ik_candidate_states = list(ik_response.candidate_states)
            if not ik_response.success:
                response.success = False
                response.message = f"PlanDualArmIk failed: {ik_response.message}"
                self.pose_status.mark_done(False, response.message)
                return response

            result = self.run_candidate_chain(
                context=request.context,
                seed_state=request.seed_state,
                ik_candidate_states=list(ik_response.candidate_states),
                attached_boxes=list(request.attached_boxes),
                scene_objects=list(request.scene_objects),
                attached_collision_objects=list(request.attached_collision_objects),
                loaded_goal_family=list(request.loaded_goal_family),
                candidate_limit=request.candidate_limit,
                planning_mode=request.planning_mode,
                execute=request.execute,
                dry_run=request.dry_run,
                velocity_scale=request.velocity_scale,
                acceleration_scale=request.acceleration_scale,
                strategy=request.strategy,
            )
            self.apply_chain_result(response, result)
            response.message = (
                f"{response.message}; ik={len(response.ik_candidate_states)} "
                f"total={(time.monotonic() - started) * 1000.0:.2f}ms"
            )
            self.pose_status.mark_done(response.success, response.message)
            return response
        except Exception as exc:  # pragma: no cover - runtime safety path
            response.success = False
            response.message = str(exc)
            self.pose_status.mark_done(False, response.message)
            return response


def main() -> None:
    rclpy.init()
    node = MotionTaskOrchestratorNode()
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
