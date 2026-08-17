from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from motion_internal_interfaces.msg import MotionPlanCandidate, RobotMotionScene, RobotMotionState
from motion_internal_interfaces.srv import CheckCollision, PlanExtract
from robot_motion_runtime.common import (
    RuntimeStatusPublisher,
    joint_distance,
    make_fixed_rate_interpolated_trajectory,
    make_interpolated_trajectory,
)


class PlanExtractServiceNode(Node):
    """Independent PlanExtract service facade.

    This node is intentionally service-shaped now: callers submit explicit IK candidate states
    and receive ranked extraction-stage candidates. The current implementation is a deterministic
    shortcut baseline, so the runtime graph can already be split and observed while deeper
    C++ extraction rollout is migrated behind the same interface.
    """

    def __init__(self) -> None:
        super().__init__("plan_extract_service")
        self.declare_parameter("service_name", "/robot_motion/plan_extract")
        self.declare_parameter("state_topic", "/robot_motion/state")
        self.declare_parameter("scene_topic", "/robot_motion/scene")
        self.declare_parameter("default_candidate_limit", 64)
        self.declare_parameter("trajectory_duration_s", 0.0)
        self.declare_parameter("trajectory_rate_hz", 10.0)
        self.declare_parameter("max_joint_step_deg", 4.5)
        self.declare_parameter("max_updown_step_m", 0.01)
        self.declare_parameter("check_collision", False)
        self.declare_parameter("collision_service_name", "/robot_motion/check_collision")
        self.declare_parameter("collision_timeout_s", 2.0)
        self.declare_parameter("collision_use_current_scene_as_base", True)
        self.declare_parameter("collision_enforce_bounds", True)

        self.service_name = str(self.get_parameter("service_name").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.scene_topic = str(self.get_parameter("scene_topic").value)
        self.default_candidate_limit = int(self.get_parameter("default_candidate_limit").value)
        self.trajectory_duration_s = float(self.get_parameter("trajectory_duration_s").value)
        self.trajectory_rate_hz = float(self.get_parameter("trajectory_rate_hz").value)
        self.max_joint_step_rad = math.radians(float(self.get_parameter("max_joint_step_deg").value))
        self.max_updown_step_m = float(self.get_parameter("max_updown_step_m").value)
        self.check_collision = bool(self.get_parameter("check_collision").value)
        self.collision_service_name = str(self.get_parameter("collision_service_name").value)
        self.collision_timeout_s = float(self.get_parameter("collision_timeout_s").value)
        self.collision_use_current_scene_as_base = bool(
            self.get_parameter("collision_use_current_scene_as_base").value
        )
        self.collision_enforce_bounds = bool(self.get_parameter("collision_enforce_bounds").value)

        self.callback_group = ReentrantCallbackGroup()
        self.latest_state: Optional[RobotMotionState] = None
        self.latest_scene: Optional[RobotMotionScene] = None
        self.state_sub = self.create_subscription(RobotMotionState, self.state_topic, self.on_state, 10)
        self.scene_sub = self.create_subscription(RobotMotionScene, self.scene_topic, self.on_scene, 10)
        self.collision_client = self.create_client(
            CheckCollision,
            self.collision_service_name,
            callback_group=self.callback_group,
        )
        self.service = self.create_service(
            PlanExtract,
            self.service_name,
            self.on_plan_extract,
            callback_group=self.callback_group,
        )
        self.status = RuntimeStatusPublisher(
            self,
            self.service_name,
            "PlanExtract service; ranks explicit IK candidate states and returns extraction-stage trajectories",
        )
        mode = "collision-aware shortcut" if self.check_collision else "shortcut baseline"
        timing_mode = (
            f"legacy_fixed_duration={self.trajectory_duration_s:.3f}s"
            if self.trajectory_duration_s > 0.0
            else f"fixed_rate={self.trajectory_rate_hz:.1f}Hz"
        )
        self.status.mark_ready(f"{mode} {timing_mode} waiting for PlanExtract requests")
        self.get_logger().info(
            f"PlanExtract service ready: {self.service_name}; {timing_mode}, "
            f"max_joint_step={math.degrees(self.max_joint_step_rad):.3f}deg"
        )

    def on_state(self, state: RobotMotionState) -> None:
        if state.authoritative:
            self.latest_state = state

    def on_scene(self, scene: RobotMotionScene) -> None:
        if scene.authoritative:
            self.latest_scene = scene

    def seed_state_for(self, request) :
        if request.seed_state.name:
            return request.seed_state
        if self.latest_state is not None:
            return self.latest_state.joint_state
        return None

    def scene_objects_for(self, request):
        if request.scene_objects or request.attached_collision_objects:
            return list(request.scene_objects), list(request.attached_collision_objects)
        if self.latest_scene is not None:
            return (
                list(self.latest_scene.scene_objects),
                list(self.latest_scene.attached_collision_objects),
            )
        return [], []

    def check_candidate_collision(self, request, start_state, trajectory) -> tuple[bool, str]:
        if not self.check_collision:
            return True, ""
        if not self.collision_client.wait_for_service(timeout_sec=self.collision_timeout_s):
            return False, f"collision_service_unavailable:{self.collision_service_name}"

        collision_request = CheckCollision.Request()
        collision_request.context = request.context
        collision_request.start_state = start_state
        collision_request.trajectory = trajectory
        collision_request.attached_boxes = request.attached_boxes
        scene_objects, attached_collision_objects = self.scene_objects_for(request)
        collision_request.scene_objects = scene_objects
        collision_request.attached_collision_objects = attached_collision_objects
        collision_request.use_current_scene_as_base = self.collision_use_current_scene_as_base
        collision_request.enforce_bounds = self.collision_enforce_bounds

        done = threading.Event()
        holder = {}
        future = self.collision_client.call_async(collision_request)

        def on_done(done_future):
            try:
                holder["response"] = done_future.result()
            except Exception as exc:  # pragma: no cover - defensive runtime path
                holder["error"] = exc
            done.set()

        future.add_done_callback(on_done)
        if not done.wait(timeout=self.collision_timeout_s):
            return False, "collision_service_timeout"
        if "error" in holder:
            return False, f"collision_service_error:{holder['error']}"
        collision_response = holder["response"]
        return bool(collision_response.valid), collision_response.reason

    def on_plan_extract(self, request, response):
        started = time.monotonic()
        self.status.mark_running(f"candidates={len(request.ik_candidate_states)}")
        seed_state = self.seed_state_for(request)
        if seed_state is None:
            response.success = False
            response.message = "missing seed_state and no authoritative RobotMotionState received"
            response.selected_index = 0
            self.status.mark_done(False, response.message)
            return response
        if not request.ik_candidate_states:
            response.success = False
            response.message = "ik_candidate_states is empty"
            response.selected_index = 0
            self.status.mark_done(False, response.message)
            return response

        limit = int(request.candidate_limit or self.default_candidate_limit)
        limit = max(1, min(limit, len(request.ik_candidate_states)))
        candidates: list[MotionPlanCandidate] = []
        for source_index, goal_state in enumerate(request.ik_candidate_states[:limit]):
            if self.trajectory_duration_s > 0.0:
                trajectory = make_interpolated_trajectory(
                    seed_state,
                    goal_state,
                    duration_s=self.trajectory_duration_s,
                    max_joint_step_rad=self.max_joint_step_rad,
                    max_linear_step_m=self.max_updown_step_m,
                )
            else:
                trajectory = make_fixed_rate_interpolated_trajectory(
                    seed_state,
                    goal_state,
                    rate_hz=self.trajectory_rate_hz,
                    max_joint_step_rad=self.max_joint_step_rad,
                    max_linear_step_m=self.max_updown_step_m,
                )
            candidate = MotionPlanCandidate()
            candidate.source_candidate_index = source_index
            collision_ok, collision_reason = self.check_candidate_collision(
                request,
                seed_state,
                trajectory,
            )
            candidate.success = collision_ok
            candidate.failure_reason = "" if collision_ok else collision_reason
            candidate.cost = joint_distance(seed_state, goal_state)
            candidate.planning_time_ms = 0.0
            candidate.trajectory_distance = candidate.cost
            candidate.start_state = seed_state
            candidate.goal_state = goal_state
            candidate.trajectory = trajectory
            candidates.append(candidate)

        candidates.sort(key=lambda item: (not item.success, item.cost, item.source_candidate_index))
        for rank, candidate in enumerate(candidates, start=1):
            candidate.rank = rank

        success_count = sum(1 for candidate in candidates if candidate.success)
        response.success = success_count > 0
        response.message = (
            f"planned {success_count}/{len(candidates)} extract candidates in "
            f"{(time.monotonic() - started) * 1000.0:.2f}ms; "
            f"mode={'collision_checked_shortcut' if self.check_collision else 'shortcut_baseline'}"
        )
        response.candidates = candidates
        response.selected_index = candidates[0].source_candidate_index if candidates else 0
        self.status.mark_done(response.success, response.message)
        return response


def main() -> None:
    rclpy.init()
    node = PlanExtractServiceNode()
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
