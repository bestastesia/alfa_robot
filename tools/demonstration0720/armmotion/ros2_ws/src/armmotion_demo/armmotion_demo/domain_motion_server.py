from __future__ import annotations

import os
import math
import threading
import time
from pathlib import Path

import rclpy
from alfa_robot_execution_bridge.joints import EXECUTION_JOINT_NAMES
from alfa_motion_interfaces.action import ExecuteMotionStage
from alfa_motion_interfaces.msg import DualArmPoseTargets, MotionErrorInfo, MotionReadiness
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from .common import (
    MotionSample,
    loaded_joint_map,
    nearest_equivalent_angle,
    retime_segment,
)
from .hardware_executor import ARM_JOINT_NAMES, HardwareExecutor
from .planner_adapter import PlannerAdapter
from .stage_contract import planning_task_from_stage_goal, validate_stage_pose_targets


READINESS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class DomainMotionServer(Node):
    """Single staged Motion ingress; vacuum ownership stays outside Motion."""

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
        self.declare_parameter("recapture_turn_target_deg", -90.0)
        self.declare_parameter("recapture_turn_tolerance_deg", 1.0)
        self.declare_parameter("recapture_preferred_updown_m", 0.3)
        self.declare_parameter("planner_timeout_s", 180.0)
        self.declare_parameter("interface_timeout_s", 10.0)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "trajectory_action",
            "/dual_arm_jtc/follow_joint_trajectory",
        )
        self.declare_parameter("stage_action", "/motion/execute_stage")
        self.declare_parameter("readiness_topic", "/motion/readiness")
        self.declare_parameter("initialize_service", "/motion/dev/initialize_loaded_pose")
        self.declare_parameter("interface_version", "autonomy-motion-action-v1")
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
        self._goal_reserved = False
        self._scene_unknown = False
        self._active_plan = None
        self._recapture_sample = None
        self._cycle_serial = 0
        self._cycle_id = ""
        self._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW
        self._last_error = self._error(MotionErrorInfo.SUCCESS)
        self._hardware = HardwareExecutor(
            self,
            dry_run=bool(self.get_parameter("dry_run").value),
            action_name=str(self.get_parameter("trajectory_action").value),
            left_solenoid_service="",
            right_solenoid_service="",
            vacuum_pump_service="",
            wait_timeout_s=interface_timeout,
            joint_state_topic=str(self.get_parameter("joint_state_topic").value),
            manage_grasp_io=False,
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
            MotionReadiness,
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
        self._stage_server = ActionServer(
            self,
            ExecuteMotionStage,
            str(self.get_parameter("stage_action").value),
            execute_callback=self._execute_stage,
            goal_callback=self._accept_stage_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._callback_group,
        )
        self._publish_readiness()
        self.get_logger().info(
            "Motion 域阶段服务已就绪："
            f"stage={self.get_parameter('stage_action').value} "
            f"trajectory={self.get_parameter('trajectory_action').value}; "
            "吸附通路由 Autonomy/RT-Control 负责"
        )

    @staticmethod
    def _error(
        code: int,
        message: str = "",
        origin: str = "motion",
    ) -> MotionErrorInfo:
        error = MotionErrorInfo()
        error.code = int(code)
        error.retryable = False
        error.message = str(message)
        error.origin = str(origin)
        return error

    def _publish_readiness(self) -> None:
        message = MotionReadiness()
        message.header.stamp = self.get_clock().now().to_msg()
        with self._lock:
            partial_test = bool(self.get_parameter("allow_partial_domain_test").value)
            message.ready = partial_test and not self._busy and not self._goal_reserved
            if self._scene_unknown:
                message.state = "SCENE_UNKNOWN"
                message.ready = False
            elif self._busy or self._goal_reserved:
                message.state = "BUSY"
            elif (
                self._active_plan is None
                and self._recapture_sample is None
                and not self._cycle_id
            ):
                message.state = "DEVELOPMENT_READY" if partial_test else "INTEGRATION_BLOCKED"
            else:
                message.state = self._stage_name(self._next_stage)
            message.last_error = self._last_error
        message.interface_version = str(self.get_parameter("interface_version").value)
        message.model_version = str(self.get_parameter("model_version").value)
        message.calibration_version = str(self.get_parameter("calibration_version").value)
        self._readiness_pub.publish(message)

    @staticmethod
    def _stage_name(stage: int) -> str:
        return {
            ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW: "WAITING_CAMERA_VIEW",
            ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP: "WAITING_PREGRASP",
            ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH: "WAITING_APPROACH",
            ExecuteMotionStage.Goal.EXECUTION_STAGE_PLACE: "WAITING_PLACE",
            ExecuteMotionStage.Goal.EXECUTION_STAGE_HOME: "WAITING_HOME",
        }.get(int(stage), "UNKNOWN_STAGE")

    def _accept_stage_goal(self, request) -> GoalResponse:
        try:
            self._validate_stage_request(request)
        except ValueError as exc:
            self.get_logger().error(f"拒绝 Motion 阶段 Goal: {exc}")
            return GoalResponse.REJECT
        with self._lock:
            if self._busy or self._goal_reserved or self._scene_unknown:
                return GoalResponse.REJECT
            stage = int(request.execution_stage)
            if stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW:
                if (
                    self._active_plan is not None
                    or self._recapture_sample is not None
                    or self._cycle_id
                ):
                    return GoalResponse.REJECT
                if int(self._next_stage) != ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW:
                    return GoalResponse.REJECT
            elif stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP:
                if (
                    self._active_plan is not None
                    or self._recapture_sample is None
                    or not self._cycle_id
                ):
                    return GoalResponse.REJECT
                if int(self._next_stage) != ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP:
                    return GoalResponse.REJECT
            else:
                if self._active_plan is None:
                    return GoalResponse.REJECT
                if stage != int(self._next_stage):
                    return GoalResponse.REJECT
            self._goal_reserved = True
        self._publish_readiness()
        return GoalResponse.ACCEPT

    def _validate_stage_request(self, request) -> None:
        if not bool(self.get_parameter("allow_partial_domain_test").value):
            raise ValueError("当前开发入口未开启部分域联调许可")
        valid_stages = {
            ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW,
            ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP,
            ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH,
            ExecuteMotionStage.Goal.EXECUTION_STAGE_PLACE,
            ExecuteMotionStage.Goal.EXECUTION_STAGE_HOME,
        }
        stage = int(request.execution_stage)
        if stage not in valid_stages:
            raise ValueError(f"不支持的 Motion 阶段: {request.execution_stage}")
        if stage in {
            ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW,
            ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP,
        }:
            validate_stage_pose_targets(request)
            if int(request.targets.left_stage) == DualArmPoseTargets.STAGE_NO_MOVE:
                raise ValueError("当前双臂流程暂不支持左臂 NO_MOVE")
            if int(request.targets.right_stage) == DualArmPoseTargets.STAGE_NO_MOVE:
                raise ValueError("当前双臂流程暂不支持右臂 NO_MOVE")

    @staticmethod
    def _feedback(state: int):
        feedback = ExecuteMotionStage.Feedback()
        feedback.motion_state = int(state)
        return feedback

    @staticmethod
    def _base_link_pose_stamped(pose) -> PoseStamped:
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose = pose
        return target

    def _run_plan_stage(self, goal_handle, plan_stage: int, label: str) -> float:
        with self._lock:
            plan = self._active_plan
        if plan is None:
            raise RuntimeError("内部执行计划不存在")
        started = time.monotonic()
        for index, segment in enumerate(plan.stages[plan_stage], start=1):
            if goal_handle.is_cancel_requested:
                raise InterruptedError("动作在轨迹段边界被取消")
            self._hardware.execute_segment(segment, f"{label}/{index}")
        return time.monotonic() - started

    def _align_turn_for_recapture(
        self,
        current: MotionSample,
    ) -> tuple[MotionSample, float]:
        target_angle = nearest_equivalent_angle(
            current.joints["turn"],
            math.radians(float(self.get_parameter("recapture_turn_target_deg").value)),
        )
        tolerance = math.radians(
            float(self.get_parameter("recapture_turn_tolerance_deg").value)
        )
        if abs(target_angle - current.joints["turn"]) <= tolerance:
            return current, 0.0
        target_joints = dict(current.joints)
        target_joints["turn"] = target_angle
        target = MotionSample(
            time_s=0.1,
            joints=target_joints,
            updown_m=current.updown_m,
            context={
                "stage": "recapture/align_turn",
                "updown": current.updown_m,
            },
        )
        samples = retime_segment(
            [current, target],
            list(EXECUTION_JOINT_NAMES),
            **self._retime_parameters,
        )
        duration = float(
            self._hardware.execute_segment(
                samples,
                "重拍前旋转 turn",
                command_turn=True,
            )["duration_s"]
        )
        self.get_logger().info(
            "重拍前 turn 对齐完成："
            f"{math.degrees(current.joints['turn']):.2f}deg -> "
            f"{math.degrees(target_angle):.2f}deg"
        )
        if self._hardware.dry_run:
            return samples[-1], duration
        return self._hardware.current_sample(), duration

    @staticmethod
    def _mask_turn_for_planning(sample: MotionSample) -> MotionSample:
        joints = dict(sample.joints)
        joints["turn"] = 0.0
        velocities = dict(sample.joint_velocities)
        velocities["turn"] = 0.0
        accelerations = dict(sample.joint_accelerations)
        accelerations["turn"] = 0.0
        return MotionSample(
            time_s=sample.time_s,
            joints=joints,
            updown_m=sample.updown_m,
            context={**sample.context, "planning_turn_masked": True},
            joint_velocities=velocities,
            updown_velocity_m_s=sample.updown_velocity_m_s,
            joint_accelerations=accelerations,
            updown_acceleration_m_s2=sample.updown_acceleration_m_s2,
        )

    def _execute_stage(self, goal_handle):
        request = goal_handle.request
        stage = int(request.execution_stage)
        result = ExecuteMotionStage.Result()
        planning_time_s = 0.0
        execution_time_s = 0.0
        with self._lock:
            self._busy = True
            self._goal_reserved = False
        self._publish_readiness()
        try:
            if stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW:
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_EXECUTING)
                )
                current = self._current_sample_for_planning()
                current, turn_execution_time_s = self._align_turn_for_recapture(current)
                execution_time_s += turn_execution_time_s
                current = self._mask_turn_for_planning(current)
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_PLANNING)
                )
                started = time.monotonic()
                samples, metrics = self._planner.plan_recapture(
                    self._base_link_pose_stamped(request.targets.left_pose),
                    self._base_link_pose_stamped(request.targets.right_pose),
                    current,
                    preferred_updown=float(
                        self.get_parameter("recapture_preferred_updown_m").value
                    ),
                )
                planning_time_s = time.monotonic() - started
                self.get_logger().info(
                    "重拍位规划完成："
                    f"updown={metrics['selected_updown']:.3f}m "
                    f"ik={metrics['ik_ms']:.2f}ms plan={metrics['planning_ms']:.2f}ms"
                )
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_EXECUTING)
                )
                execution_time_s += float(
                    self._hardware.execute_segment(samples, "重拍位")["duration_s"]
                )
                recapture_sample = (
                    samples[-1]
                    if self._hardware.dry_run
                    else self._mask_turn_for_planning(self._hardware.current_sample())
                )
                with self._lock:
                    self._cycle_serial += 1
                    self._cycle_id = f"motion-cycle-{self._cycle_serial:06d}"
                    self._recapture_sample = recapture_sample
                    self._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP
            elif stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP:
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_PLANNING)
                )
                with self._lock:
                    cycle_id = self._cycle_id
                task = planning_task_from_stage_goal(request, cycle_id)
                started = time.monotonic()
                with self._lock:
                    recapture_sample = self._recapture_sample
                if recapture_sample is None:
                    raise RuntimeError("缺少重拍阶段真实末态")
                plan = self._planner.compute(task, initial_sample=recapture_sample)
                planning_time_s = time.monotonic() - started
                with self._lock:
                    self._active_plan = plan
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_EXECUTING)
                )
                execution_time_s = self._run_plan_stage(
                    goal_handle,
                    1,
                    "预抓取",
                )
                self._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH
            elif stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH:
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_EXECUTING)
                )
                execution_time_s = self._run_plan_stage(goal_handle, 2, "靠近吸附")
                self._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_PLACE
            elif stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_PLACE:
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_EXECUTING)
                )
                execution_time_s += self._run_plan_stage(goal_handle, 3, "抽离到负重")
                execution_time_s += self._run_plan_stage(goal_handle, 4, "负重到放置")
                self._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_HOME
            else:
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_EXECUTING)
                )
                execution_time_s = self._run_plan_stage(goal_handle, 6, "返回初始位")
                with self._lock:
                    self._active_plan = None
                    self._recapture_sample = None
                    self._cycle_id = ""
                self._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW

            result.diagnostic = (
                f"{self._stage_name(stage)} complete; "
                f"planning={planning_time_s:.3f}s execution={execution_time_s:.3f}s"
            )
            goal_handle.publish_feedback(
                self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_SETTLING)
            )
            goal_handle.succeed()
            with self._lock:
                self._last_error = self._error(MotionErrorInfo.SUCCESS)
        except InterruptedError as exc:
            error = self._error(MotionErrorInfo.EXECUTION_FAILED, str(exc))
            result.diagnostic = str(exc)
            goal_handle.canceled()
            self._handle_stage_failure(stage, error)
        except Exception as exc:
            error_code = (
                MotionErrorInfo.PLANNING_FAILED
                if stage in {
                    ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW,
                    ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP,
                }
                else MotionErrorInfo.EXECUTION_FAILED
            )
            error = self._error(error_code, str(exc))
            result.diagnostic = str(exc)
            goal_handle.abort()
            self._handle_stage_failure(stage, error)
            self.get_logger().error(f"Motion 阶段失败 stage={stage}: {exc}")
        finally:
            with self._lock:
                self._busy = False
                self._goal_reserved = False
            self._publish_readiness()
        return result

    def _handle_stage_failure(self, stage: int, error: MotionErrorInfo) -> None:
        with self._lock:
            self._last_error = error
            if stage >= ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH:
                self._scene_unknown = True
            else:
                self._active_plan = None
                self._recapture_sample = None
                self._cycle_id = ""
                self._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW

    def _current_sample_for_planning(self) -> MotionSample:
        if not self._hardware.dry_run:
            return self._hardware.current_sample()
        target_joints = loaded_joint_map(EXECUTION_JOINT_NAMES)
        return MotionSample(
            time_s=0.0,
            joints=target_joints,
            updown_m=0.3,
            context={"stage": "dry_run/current", "updown": 0.3},
        )

    def _move_to_loaded_pose(self, label: str) -> float:
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
        return float(self._hardware.execute_segment(samples, label)["duration_s"])

    def _initialize_loaded_pose(self, _request, response):
        with self._lock:
            if self._busy or self._goal_reserved or self._active_plan is not None:
                response.success = False
                response.message = "Motion 正忙或仍持有任务计划"
                return response
            self._busy = True
        self._publish_readiness()
        try:
            duration = self._move_to_loaded_pose("Motion 初始化负重位")
            response.success = True
            response.message = f"初始化完成 duration={duration:.3f}s"
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
