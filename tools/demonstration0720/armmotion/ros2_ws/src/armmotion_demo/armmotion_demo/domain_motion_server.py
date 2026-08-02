from __future__ import annotations

import os
import threading
import time
from pathlib import Path

import rclpy
from alfa_system_interfaces.msg import DomainReadiness, ErrorInfo
from alfa_task_interfaces.action import PlanAndExecutePick, PlanAndExecutePlace
from alfa_task_interfaces.msg import PickOutcome, PickTarget, PlaceOutcome, PlaceTarget
from alfa_robot_execution_bridge.joints import EXECUTION_JOINT_NAMES
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from .common import MotionSample, loaded_joint_map, retime_segment
from .domain_contract import DomainTaskSpec, task_spec_from_pick_targets
from .hardware_executor import ARM_JOINT_NAMES, HardwareExecutor
from .planner_adapter import PlannerAdapter


UNVERIFIED = 0
ATTACHED_VERIFIED = 1

READINESS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class DomainMotionServer(Node):
    """Five-domain Motion ingress backed by the validated staged planner."""

    def __init__(self) -> None:
        super().__init__("motion_domain_server")
        default_source_ws = os.environ.get("ARMMOTION_SOURCE_WS", "/motion_ws")
        default_output_root = os.environ.get("ARMMOTION_OUTPUT_ROOT", "/motion_data")
        self.declare_parameter("source_ws", default_source_ws)
        self.declare_parameter("output_root", default_output_root)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("trajectory_rate_hz", 30.0)
        self.declare_parameter("execution_speed_scale", 3.0)
        self.declare_parameter("max_joint_speed_deg_s", 10.0)
        self.declare_parameter("max_joint_acceleration_deg_s2", 60.0)
        self.declare_parameter("max_updown_speed_m_s", 0.05)
        self.declare_parameter("updown_acceleration_m_s2", 0.05)
        self.declare_parameter("planner_timeout_s", 180.0)
        self.declare_parameter("interface_timeout_s", 10.0)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("trajectory_action", "/whole_body_jtc/follow_joint_trajectory")
        self.declare_parameter("vacuum_action", "/vacuum/grip")
        self.declare_parameter("pick_action", "/motion/plan_and_execute_pick")
        self.declare_parameter("place_action", "/motion/plan_and_execute_place")
        self.declare_parameter("readiness_topic", "/motion/readiness")
        self.declare_parameter("initialize_service", "/motion/dev/initialize_loaded_pose")
        self.declare_parameter("interface_version", "five-domain-motion-dev-v0")
        self.declare_parameter("model_version", "alfa_robot_current")
        self.declare_parameter("calibration_version", "rt_control_current")
        self.declare_parameter("allow_partial_domain_test", False)

        rate_hz = float(self.get_parameter("trajectory_rate_hz").value)
        speed_scale = float(self.get_parameter("execution_speed_scale").value)
        max_joint_speed = float(self.get_parameter("max_joint_speed_deg_s").value)
        max_joint_acceleration = float(
            self.get_parameter("max_joint_acceleration_deg_s2").value
        )
        max_updown_speed = float(self.get_parameter("max_updown_speed_m_s").value)
        updown_acceleration = float(self.get_parameter("updown_acceleration_m_s2").value)
        interface_timeout = float(self.get_parameter("interface_timeout_s").value)

        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._busy = False
        self._goal_reservation = ""
        self._active_request_id = ""
        self._active_plan = None
        self._active_task: DomainTaskSpec | None = None
        self._place_attempted = False
        self._scene_unknown = False
        self._sequence_ledger: set[tuple[str, int]] = set()
        self._last_error = self._error(ErrorInfo.SUCCESS)
        self._hardware = HardwareExecutor(
            self,
            dry_run=bool(self.get_parameter("dry_run").value),
            action_name=str(self.get_parameter("trajectory_action").value),
            left_solenoid_service="/motion/internal/unused_left_solenoid",
            right_solenoid_service="/motion/internal/unused_right_solenoid",
            vacuum_pump_service="/motion/internal/unused_vacuum_pump",
            wait_timeout_s=interface_timeout,
            joint_state_topic=str(self.get_parameter("joint_state_topic").value),
            vacuum_action_name=str(self.get_parameter("vacuum_action").value),
        )
        self._hardware.verify_interfaces()
        self._planner = PlannerAdapter(
            source_ws=Path(str(self.get_parameter("source_ws").value)),
            output_root=Path(str(self.get_parameter("output_root").value)),
            rate_hz=rate_hz,
            max_joint_speed_deg_s=max_joint_speed,
            max_joint_acceleration_deg_s2=max_joint_acceleration,
            max_updown_speed_m_s=max_updown_speed,
            max_updown_acceleration_m_s2=updown_acceleration,
            speed_scale=speed_scale,
            timeout_s=float(self.get_parameter("planner_timeout_s").value),
        )
        self._retime_parameters = {
            "rate_hz": rate_hz,
            "max_joint_speed_deg_s": max_joint_speed,
            "max_joint_acceleration_deg_s2": max_joint_acceleration,
            "max_updown_speed_m_s": max_updown_speed,
            "max_updown_acceleration_m_s2": updown_acceleration,
            "speed_scale": speed_scale,
        }

        self._readiness_pub = self.create_publisher(
            DomainReadiness,
            str(self.get_parameter("readiness_topic").value),
            READINESS_QOS,
        )
        self._readiness_timer = self.create_timer(1.0, self._publish_readiness)
        self._initialize_service = self.create_service(
            Trigger,
            str(self.get_parameter("initialize_service").value),
            self._initialize_loaded_pose,
            callback_group=self._callback_group,
        )
        self._pick_server = ActionServer(
            self,
            PlanAndExecutePick,
            str(self.get_parameter("pick_action").value),
            execute_callback=self._execute_pick,
            goal_callback=self._pick_goal,
            cancel_callback=self._cancel_goal,
            callback_group=self._callback_group,
        )
        self._place_server = ActionServer(
            self,
            PlanAndExecutePlace,
            str(self.get_parameter("place_action").value),
            execute_callback=self._execute_place,
            goal_callback=self._place_goal,
            cancel_callback=self._cancel_goal,
            callback_group=self._callback_group,
        )
        self._publish_readiness()
        self.get_logger().info(
            "Motion 域服务已就绪："
            f"pick={self.get_parameter('pick_action').value} "
            f"place={self.get_parameter('place_action').value} "
            f"trajectory={self.get_parameter('trajectory_action').value} "
            f"vacuum={self.get_parameter('vacuum_action').value}"
        )

    @staticmethod
    def _error(code: int, message: str = "", origin: str = "motion") -> ErrorInfo:
        out = ErrorInfo()
        out.code = int(code)
        out.retryable = False
        out.message = str(message)
        out.origin = str(origin)
        return out

    def _publish_readiness(self) -> None:
        message = DomainReadiness()
        message.header.stamp = self.get_clock().now().to_msg()
        message.domain = "motion"
        with self._lock:
            partial_test = bool(self.get_parameter("allow_partial_domain_test").value)
            message.ready = (
                partial_test
                and not self._busy
                and not self._goal_reservation
                and self._active_plan is None
                and not self._scene_unknown
            )
            if self._scene_unknown:
                message.state = "SCENE_UNKNOWN"
            elif self._busy or self._goal_reservation:
                message.state = "BUSY"
            elif self._active_plan is not None:
                message.state = "WAITING_PLACE"
            elif partial_test:
                message.state = "DEVELOPMENT_READY"
            else:
                message.state = "INTEGRATION_BLOCKED"
            message.last_error = self._last_error
        message.interface_version = str(self.get_parameter("interface_version").value)
        message.model_version = str(self.get_parameter("model_version").value)
        message.calibration_version = str(self.get_parameter("calibration_version").value)
        self._readiness_pub.publish(message)

    def _pick_goal(self, request) -> GoalResponse:
        try:
            task = self._validate_pick_goal(request)
        except ValueError as exc:
            self.get_logger().error(f"拒绝 M-02 Goal: {exc}")
            return GoalResponse.REJECT
        sequence_key = (task.task_id, task.sequence_id)
        with self._lock:
            if (
                self._busy
                or self._goal_reservation
                or self._active_plan is not None
                or self._scene_unknown
                or sequence_key in self._sequence_ledger
            ):
                return GoalResponse.REJECT
            self._goal_reservation = "pick"
            self._sequence_ledger.add(sequence_key)
        self._publish_readiness()
        return GoalResponse.ACCEPT

    def _validate_pick_goal(self, request) -> DomainTaskSpec:
        if not bool(self.get_parameter("allow_partial_domain_test").value):
            raise ValueError("当前首版尚未接入 Gate/安全准入，未开启部分域联调许可")
        if not request.context.request_id or not request.context.task_id:
            raise ValueError("context.request_id/task_id 不能为空")
        if int(request.context.sequence_id) <= 0:
            raise ValueError("context.sequence_id 必须从1开始")
        if not request.expected_map_version:
            raise ValueError("expected_map_version 不能为空")
        if not request.motion_profile_id or not request.grip_profile_id:
            raise ValueError("motion_profile_id/grip_profile_id 不能为空")
        max_pose_age_s = float(request.max_pose_age.sec) + float(
            request.max_pose_age.nanosec
        ) * 1e-9
        if max_pose_age_s <= 0.0:
            raise ValueError("max_pose_age 必须为正数")
        now_s = self.get_clock().now().nanoseconds * 1e-9
        for target in request.targets:
            for name, stamped in (
                ("body_pose", target.refined_geometry.body_pose),
                ("suction_surface_pose", target.refined_geometry.suction_surface_pose),
            ):
                stamp_s = float(stamped.header.stamp.sec) + float(
                    stamped.header.stamp.nanosec
                ) * 1e-9
                age_s = now_s - stamp_s
                if stamp_s <= 0.0 or age_s > max_pose_age_s or age_s < -0.2:
                    raise ValueError(
                        f"box={target.box_id} {name} 时间戳不新鲜: "
                        f"age={age_s:.3f}s limit={max_pose_age_s:.3f}s"
                    )
        return task_spec_from_pick_targets(
            request.context.request_id,
            request.context.task_id,
            request.context.sequence_id,
            list(request.targets),
        )

    def _place_goal(self, request) -> GoalResponse:
        with self._lock:
            profiles_valid = bool(
                request.motion_profile_id
                and request.placement_profile_id
                and request.grip_profile_id
            )
            targets_valid = all(
                int(target.box_id) > 0
                and int(target.row) > 0
                and int(target.column) > 0
                for target in request.targets
            )
            if (
                self._busy
                or self._goal_reservation
                or self._active_plan is None
                or self._active_task is None
                or self._place_attempted
                or self._scene_unknown
                or not profiles_valid
                or not targets_valid
            ):
                return GoalResponse.REJECT
            expected = [
                (self._active_task.left_box_id, PlaceTarget.ARM_LEFT),
                (self._active_task.right_box_id, PlaceTarget.ARM_RIGHT),
            ]
            actual = [(int(target.box_id), int(target.arm_id)) for target in request.targets]
            context_matches = (
                request.context.task_id == self._active_task.task_id
                and int(request.context.sequence_id) == self._active_task.sequence_id
                and bool(request.context.request_id)
            )
            if actual != expected or not context_matches:
                return GoalResponse.REJECT
            self._place_attempted = True
            self._goal_reservation = "place"
        self._publish_readiness()
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_goal(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _set_busy(self, busy: bool, request_id: str = "") -> None:
        with self._lock:
            self._busy = bool(busy)
            self._active_request_id = request_id if busy else ""
            if not busy:
                self._goal_reservation = ""
        self._publish_readiness()

    @staticmethod
    def _feedback(action_type, stage: str, progress: float):
        feedback = action_type.Feedback()
        feedback.stage = stage
        feedback.progress_0_to_1 = float(progress)
        return feedback

    def _run_stage(self, goal_handle, stage_number: int, label: str) -> None:
        with self._lock:
            plan = self._active_plan
        if plan is None:
            raise RuntimeError("内部执行计划不存在")
        segments = plan.stages[stage_number]
        for index, segment in enumerate(segments, start=1):
            if goal_handle.is_cancel_requested:
                raise InterruptedError("动作在轨迹段边界被取消")
            self._hardware.execute_segment(segment, f"{label}/{index}")

    def _execute_pick(self, goal_handle):
        result = PlanAndExecutePick.Result()
        request = goal_handle.request
        request_id = request.context.request_id or f"pick-{time.time_ns()}"
        self._set_busy(True, request_id)
        grip_command_sent = False
        try:
            task = task_spec_from_pick_targets(
                request_id,
                request.context.task_id,
                request.context.sequence_id,
                list(request.targets),
            )
            goal_handle.publish_feedback(self._feedback(PlanAndExecutePick, "PLANNING", 0.05))
            plan = self._planner.compute(task)
            with self._lock:
                self._active_plan = plan
                self._active_task = task
            goal_handle.publish_feedback(self._feedback(PlanAndExecutePick, "APPROACH", 0.25))
            self._run_stage(goal_handle, 1, "M-02/approach_pre_contact")
            self._run_stage(goal_handle, 2, "M-02/contact")
            goal_handle.publish_feedback(self._feedback(PlanAndExecutePick, "GRIP", 0.55))
            self._hardware.set_grasp_solenoids(True)
            grip_command_sent = True
            goal_handle.publish_feedback(self._feedback(PlanAndExecutePick, "RETREAT", 0.70))
            self._run_stage(goal_handle, 3, "M-02/retreat_to_loaded")
            result.outcomes = [
                self._pick_outcome(target, True) for target in request.targets
            ]
            result.overall_verification_level = ATTACHED_VERIFIED
            result.error = self._error(ErrorInfo.SUCCESS)
            goal_handle.publish_feedback(self._feedback(PlanAndExecutePick, "COMPLETE", 1.0))
            goal_handle.succeed()
            with self._lock:
                self._last_error = self._error(ErrorInfo.SUCCESS)
                self._place_attempted = False
        except InterruptedError as exc:
            result.outcomes = [self._pick_outcome(target, False, str(exc)) for target in request.targets]
            result.error = self._error(ErrorInfo.EXECUTION_FAILED, str(exc))
            goal_handle.canceled()
            with self._lock:
                if grip_command_sent:
                    self._scene_unknown = True
                else:
                    self._active_plan = None
                    self._active_task = None
                self._last_error = result.error
        except Exception as exc:
            result.outcomes = [self._pick_outcome(target, False, str(exc)) for target in request.targets]
            result.error = self._error(ErrorInfo.EXECUTION_FAILED, str(exc))
            goal_handle.abort()
            with self._lock:
                if grip_command_sent:
                    self._scene_unknown = True
                else:
                    self._active_plan = None
                    self._active_task = None
                self._last_error = result.error
            self.get_logger().error(f"M-02 失败: {exc}")
        finally:
            self._set_busy(False)
        return result

    def _execute_place(self, goal_handle):
        result = PlanAndExecutePlace.Result()
        request = goal_handle.request
        request_id = request.context.request_id or f"place-{time.time_ns()}"
        self._set_busy(True, request_id)
        try:
            goal_handle.publish_feedback(self._feedback(PlanAndExecutePlace, "TRANSFER", 0.10))
            self._run_stage(goal_handle, 4, "M-03/transfer_to_place")
            goal_handle.publish_feedback(self._feedback(PlanAndExecutePlace, "RELEASE", 0.60))
            self._hardware.set_grasp_solenoids(False)
            goal_handle.publish_feedback(self._feedback(PlanAndExecutePlace, "WITHDRAW", 0.75))
            self._run_stage(goal_handle, 6, "M-03/withdraw_to_loaded")
            result.outcomes = [self._place_outcome(target, True) for target in request.targets]
            result.overall_verification_level = UNVERIFIED
            result.error = self._error(ErrorInfo.SUCCESS)
            goal_handle.publish_feedback(self._feedback(PlanAndExecutePlace, "COMPLETE", 1.0))
            goal_handle.succeed()
            with self._lock:
                self._active_plan = None
                self._active_task = None
                self._place_attempted = False
                self._last_error = self._error(ErrorInfo.SUCCESS)
        except InterruptedError as exc:
            result.outcomes = [self._place_outcome(target, False, str(exc)) for target in request.targets]
            result.error = self._error(ErrorInfo.EXECUTION_FAILED, str(exc))
            goal_handle.canceled()
            with self._lock:
                self._scene_unknown = True
                self._last_error = result.error
        except Exception as exc:
            result.outcomes = [self._place_outcome(target, False, str(exc)) for target in request.targets]
            result.error = self._error(ErrorInfo.EXECUTION_FAILED, str(exc))
            goal_handle.abort()
            with self._lock:
                self._scene_unknown = True
                self._last_error = result.error
            self.get_logger().error(f"M-03 失败: {exc}")
        finally:
            self._set_busy(False)
        return result

    def _pick_outcome(self, target: PickTarget, success: bool, message: str = "") -> PickOutcome:
        outcome = PickOutcome()
        outcome.box_id = target.box_id
        outcome.arm_id = target.arm_id
        outcome.pick_motion_completed = success
        outcome.grip_command_completed = success
        outcome.retreat_completed = success
        outcome.verification_level = ATTACHED_VERIFIED if success else UNVERIFIED
        outcome.error = self._error(ErrorInfo.SUCCESS if success else ErrorInfo.EXECUTION_FAILED, message)
        return outcome

    def _place_outcome(self, target: PlaceTarget, success: bool, message: str = "") -> PlaceOutcome:
        outcome = PlaceOutcome()
        outcome.box_id = target.box_id
        outcome.arm_id = target.arm_id
        outcome.place_motion_completed = success
        outcome.release_command_completed = success
        outcome.retreat_completed = success
        outcome.verification_level = UNVERIFIED
        outcome.error = self._error(ErrorInfo.SUCCESS if success else ErrorInfo.EXECUTION_FAILED, message)
        return outcome

    def _initialize_loaded_pose(self, _request, response):
        with self._lock:
            if (
                self._busy
                or self._goal_reservation
                or self._active_plan is not None
                or self._scene_unknown
            ):
                response.success = False
                response.message = "Motion 正忙或仍持有待放置任务"
                return response
            self._busy = True
        self._publish_readiness()
        try:
            target_joints = loaded_joint_map(EXECUTION_JOINT_NAMES)
            if self._hardware.dry_run:
                current = MotionSample(
                    time_s=0.0,
                    joints=dict(target_joints),
                    updown_m=0.3,
                    context={"stage": "initialization/current", "updown": 0.3},
                )
            else:
                current = self._hardware.current_sample()
            target_joints["turn"] = current.joints.get("turn", 0.0)
            target = MotionSample(
                time_s=0.1,
                joints=target_joints,
                updown_m=0.3,
                context={"stage": "initialization/loaded", "updown": 0.3},
            )
            samples = retime_segment(
                [current, target],
                list(ARM_JOINT_NAMES),
                **self._retime_parameters,
            )
            metrics = self._hardware.execute_segment(samples, "Motion 初始化负重位")
            response.success = True
            response.message = f"初始化完成 duration={metrics['duration_s']:.3f}s"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        finally:
            with self._lock:
                self._busy = False
            self._publish_readiness()
        return response

    def close(self) -> None:
        self._planner.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DomainMotionServer()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
