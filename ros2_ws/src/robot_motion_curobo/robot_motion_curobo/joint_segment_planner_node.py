"""ROS 2 service exposing a single persistent cuRobo plan_cspace planner."""

from __future__ import annotations

import math
import os
import threading

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from robot_motion_interfaces.srv import CheckCollision, PlanJointSegment

from .curobo_backend import CuroboBackend
from .planner_core import (
    AttachedPayload,
    JointSegmentPlannerCore,
    JointValues,
    SegmentRequest,
    _validate_limits,
    reorder_joint_values,
    validate_payloads,
)


def _duration(seconds: float) -> Duration:
    whole = int(math.floor(seconds))
    nanoseconds = int(round((seconds - whole) * 1_000_000_000.0))
    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000
    return Duration(sec=whole, nanosec=nanoseconds)


class JointSegmentPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("joint_segment_planner")
        self._service_name = self.declare_parameter(
            "service_name", "/robot_motion/plan_joint_segment"
        ).value
        robot_config_path = self.declare_parameter("robot_config_path", "").value
        scene_config_path = self.declare_parameter("scene_config_path", "").value
        expected_scene_id = self.declare_parameter("expected_scene_id", "").value
        self._expected_scene_id = expected_scene_id
        request_timeout_s = float(
            self.declare_parameter("request_timeout_s", 0.5).value
        )
        max_attempts = int(self.declare_parameter("max_attempts", 3).value)

        self.get_logger().info(
            "Loading persistent cuRobo planner; the service becomes available only after warmup"
        )
        self._backend = CuroboBackend(
            robot_config_path=robot_config_path,
            scene_config_path=scene_config_path,
            curobo_python_root=self.declare_parameter("curobo_python_root", "").value,
            dependency_venv=self.declare_parameter("dependency_venv", "").value,
            warmup_iterations=int(
                self.declare_parameter("warmup_iterations", 5).value
            ),
            use_cuda_graph=bool(self.declare_parameter("use_cuda_graph", True).value),
            self_collision_check=bool(
                self.declare_parameter("self_collision_check", True).value
            ),
            num_ik_seeds=int(self.declare_parameter("num_ik_seeds", 32).value),
            num_trajopt_seeds=int(
                self.declare_parameter("num_trajopt_seeds", 4).value
            ),
            trajopt_num_iters=int(
                self.declare_parameter("trajopt_num_iters", 100).value
            ),
            trajopt_inner_iters=int(
                self.declare_parameter("trajopt_inner_iters", 25).value
            ),
            trajopt_history=int(
                self.declare_parameter("trajopt_history", 27).value
            ),
            trajopt_n_knots=int(
                self.declare_parameter("trajopt_n_knots", 16).value
            ),
            trajopt_interpolation_steps=int(
                self.declare_parameter("trajopt_interpolation_steps", 4).value
            ),
            trajopt_finetune_attempts=int(
                self.declare_parameter("trajopt_finetune_attempts", 3).value
            ),
            trajopt_finetune_dt_scale=float(
                self.declare_parameter("trajopt_finetune_dt_scale", 0.75).value
            ),
            optimizer_collision_activation_distance=float(
                self.declare_parameter(
                    "optimizer_collision_activation_distance", 0.01
                ).value
            ),
            collision_cache_cuboids=int(
                self.declare_parameter("collision_cache_cuboids", 128).value
            ),
            collision_cache_meshes=int(
                self.declare_parameter("collision_cache_meshes", 8).value
            ),
            interpolation_dt=float(
                self.declare_parameter("interpolation_dt", 0.02).value
            ),
            left_payload_link=self.declare_parameter("left_payload_link", "").value,
            right_payload_link=self.declare_parameter("right_payload_link", "").value,
            payload_grid_resolution_m=float(
                self.declare_parameter("payload_grid_resolution_m", 0.1).value
            ),
            world_payload_object_template=self.declare_parameter(
                "world_payload_object_template", "cargo_box_{box_id:02d}"
            ).value,
            require_world_payload_object=bool(
                self.declare_parameter("require_world_payload_object", False).value
            ),
        )
        self._core = JointSegmentPlannerCore(
            self._backend,
            expected_scene_id=expected_scene_id,
            default_timeout_s=request_timeout_s,
            default_max_attempts=max_attempts,
        )
        self._callbacks = ReentrantCallbackGroup()
        self._backend_lock = threading.Lock()
        self._service = self.create_service(
            PlanJointSegment,
            self._service_name,
            self._handle_request,
            callback_group=self._callbacks,
        )
        collision_service_name = self.declare_parameter(
            "collision_service_name", "/robot_motion/check_curobo_collision"
        ).value
        self._collision_service = self.create_service(
            CheckCollision,
            collision_service_name,
            self._handle_collision_request,
            callback_group=self._callbacks,
        )
        self.get_logger().info(
            f"Local cuRobo service ready: {self._service_name}; "
            f"warmup_count={self._backend.warmup_count} scene_id={expected_scene_id or '(unchecked)'}"
        )

    @staticmethod
    def _payload(message) -> AttachedPayload:
        return AttachedPayload(
            object_id=message.id,
            box_id=int(message.box_id),
            side=message.side,
            link_name=message.link_name,
            center_xyz=(
                float(message.center_in_link.position.x),
                float(message.center_in_link.position.y),
                float(message.center_in_link.position.z),
            ),
            orientation_xyzw=(
                float(message.center_in_link.orientation.x),
                float(message.center_in_link.orientation.y),
                float(message.center_in_link.orientation.z),
                float(message.center_in_link.orientation.w),
            ),
            size_xyz=(float(message.size.x), float(message.size.y), float(message.size.z)),
        )

    def _handle_request(self, request, response):
        core_request = SegmentRequest(
            request_id=request.context.request_id,
            scene_id=request.context.scene_id,
            start=JointValues(
                tuple(request.start_state.name), tuple(request.start_state.position)
            ),
            goal=JointValues(
                tuple(request.goal_state.name), tuple(request.goal_state.position)
            ),
            attached_payloads=tuple(self._payload(item) for item in request.attached_boxes),
            force_graph=bool(request.force_graph),
            max_attempts=int(request.max_attempts),
            timeout_s=float(request.timeout_s),
        )
        with self._backend_lock:
            result = self._core.plan(core_request)
        response.success = result.success
        response.message = result.message
        response.planner_method = result.planner_method
        response.total_time_ms = result.total_time_ms
        response.solve_time_ms = result.solve_time_ms
        response.queue_time_ms = result.queue_time_ms
        response.endpoint_check_time_ms = result.endpoint_check_time_ms
        response.graph_time_ms = result.graph_time_ms
        response.trajopt_time_ms = result.trajopt_time_ms
        response.interpolation_time_ms = result.interpolation_time_ms
        if result.trajectory is not None:
            response.trajectory.joint_names = list(result.trajectory.joint_names)
            for index, (positions, time_s) in enumerate(
                zip(result.trajectory.positions, result.trajectory.times_s)
            ):
                point = JointTrajectoryPoint()
                point.positions = list(positions)
                point.time_from_start = _duration(time_s)
                if result.trajectory.velocities:
                    point.velocities = list(result.trajectory.velocities[index])
                if result.trajectory.accelerations:
                    point.accelerations = list(result.trajectory.accelerations[index])
                response.trajectory.points.append(point)
        if result.success:
            self.get_logger().info(
                f"request={core_request.request_id} method={result.planner_method} "
                f"queue={result.queue_time_ms:.2f}ms endpoint={result.endpoint_check_time_ms:.2f}ms "
                f"graph={result.graph_time_ms:.2f}ms trajopt={result.trajopt_time_ms:.2f}ms "
                f"interpolation={result.interpolation_time_ms:.2f}ms solve={result.solve_time_ms:.2f}ms "
                f"total={result.total_time_ms:.2f}ms points="
                f"{len(result.trajectory.positions) if result.trajectory else 0} "
                f"gpu_peak={self._backend.peak_gpu_memory_mb():.1f}MiB"
            )
        else:
            self.get_logger().warning(
                f"request={core_request.request_id} failed={result.message} "
                f"queue={result.queue_time_ms:.2f}ms total={result.total_time_ms:.2f}ms"
            )
        return response

    def _handle_collision_request(self, request, response):
        try:
            if self._expected_scene_id and request.context.scene_id != self._expected_scene_id:
                raise ValueError(
                    f"scene_id_mismatch:{request.context.scene_id}!={self._expected_scene_id}"
                )
            if request.scene_objects or request.attached_collision_objects:
                raise ValueError("curobo_collision_check_uses_loaded_static_scene_only")
            payloads = tuple(self._payload(item) for item in request.attached_boxes)
            validate_payloads(payloads)
            rows = []
            if request.trajectory.points:
                names = tuple(request.trajectory.joint_names)
                for point in request.trajectory.points:
                    row = reorder_joint_values(
                        JointValues(names, tuple(point.positions))
                    )
                    _validate_limits(row, self._backend.joint_limits)
                    rows.append(row)
            else:
                row = reorder_joint_values(
                    JointValues(
                        tuple(request.start_state.name),
                        tuple(request.start_state.position),
                    )
                )
                _validate_limits(row, self._backend.joint_limits)
                rows.append(row)
            with self._backend_lock:
                feasible = self._backend.check_positions_feasible(rows, payloads)
            response.valid = all(feasible)
            if response.valid:
                response.reason = ""
            else:
                failed_index = next(index for index, value in enumerate(feasible) if not value)
                response.reason = f"curobo_collision_or_bounds@{failed_index}"
        except Exception as exc:
            response.valid = False
            response.reason = str(exc)
        return response

    def destroy_node(self) -> bool:
        try:
            self._backend.destroy()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = JointSegmentPlannerNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        # MotionGen owns CUDA/C++ worker state whose interpreter-finalization
        # teardown can hang after SIGINT. Shut ROS down first, then leave the
        # signal path without running Python/CUDA destructors. Normal exits and
        # initialization errors still use the full cleanup below.
        executor.shutdown(timeout_sec=1.0)
        if rclpy.ok():
            rclpy.shutdown()
        os._exit(0)
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
