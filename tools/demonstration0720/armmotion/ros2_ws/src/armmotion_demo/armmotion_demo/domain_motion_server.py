from __future__ import annotations

import copy
import math
import os
import threading
import time
import uuid
from pathlib import Path

import rclpy
from alfa_robot_execution_bridge.joints import EXECUTION_JOINT_NAMES, RT_CONTROL_ACTION_NAME
from geometry_msgs.msg import PoseStamped
from robot_interfaces_qos import latched
from robot_motion_interfaces.action import ExecuteMotionStage
from robot_motion_interfaces.msg import DualArmPoseTargets
from robot_system_interfaces.msg import DomainReadiness, ErrorCode, ErrorInfo
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .common import (
    MotionSample,
    loaded_joint_map,
)
from .hardware_executor import HardwareExecutor
from .planner_adapter import PlannerAdapter
from .stage_contract import (
    align_target_pair_to_average_x,
    align_target_pair_to_lower_height,
    canonicalize_stage_target_orientations,
    planning_task_from_resolved_targets,
    resolve_dual_stage_targets,
    validate_default_stage_targets,
    validate_stage_pose_targets,
)
from .turn_frame import compensate_pose_y, pose_at_zero_turn


SIDE_RECAPTURE_ARM_POSE_DEG = (0.0, -88.0, 135.0, -40.0, 0.0, 0.0)
REMOTE_CAMERA_VIEW_JOINTS_RAD = {
    "left": (
        0.000047935,
        -0.785901500,
        2.103159565,
        -1.309444351,
        0.000059923,
        0.0,
    ),
    "right": (
        -0.000407465,
        -0.785374195,
        2.102200827,
        -1.204246822,
        0.000239686,
        0.000095874,
    ),
}
ARM_CONVERGED_JOINTS_RAD = {
    "left": (
        0.645769957,
        -0.854858731,
        1.548457731,
        -0.685905129,
        0.642342472,
        -0.000023968,
    ),
    "right": (
        -0.584698350,
        -0.800761939,
        1.328523236,
        -0.516603981,
        -0.579425288,
        0.000071905,
    ),
}
INDEPENDENT_EXECUTION_STAGES = frozenset(
    {
        ExecuteMotionStage.Goal.EXECUTION_STAGE_TURN,
        ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW,
        ExecuteMotionStage.Goal.EXECUTION_STAGE_NAMED_JOINT_POSE,
    }
)


def pregrasp_entry_mode(
    *,
    active_plan,
    recapture_sample,
    cycle_id: str,
    next_stage: int,
) -> str | None:
    if (
        active_plan is None
        and recapture_sample is None
        and not cycle_id
        and int(next_stage) == ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW
    ):
        return "from_idle"
    return None


def pregrasp_planning_start_sample(
    *,
    entry_mode: str,
    current_sample: MotionSample,
    recapture_sample: MotionSample | None,
) -> MotionSample:
    if entry_mode != "from_idle":
        raise ValueError(f"无效的 PREGRASP 入口模式: {entry_mode}")
    return current_sample


def named_joint_pose_target(
    current: MotionSample,
    named_joint_pose: int,
) -> tuple[MotionSample, str]:
    definitions = {
        ExecuteMotionStage.Goal.NAMED_JOINT_POSE_REMOTE_CAMERA_VIEW: (
            REMOTE_CAMERA_VIEW_JOINTS_RAD,
            "remote_camera_view",
        ),
        ExecuteMotionStage.Goal.NAMED_JOINT_POSE_ARM_CONVERGED: (
            ARM_CONVERGED_JOINTS_RAD,
            "arm_converged",
        ),
    }
    try:
        pose_by_side, label = definitions[int(named_joint_pose)]
    except KeyError as exc:
        raise ValueError(f"不支持的命名关节姿态: {named_joint_pose}") from exc
    joints = dict(current.joints)
    for side, values in pose_by_side.items():
        for index, value in enumerate(values, start=1):
            joints[f"{side}_joint{index}"] = float(value)
    return (
        MotionSample(
            time_s=0.1,
            joints=joints,
            updown_m=float(current.updown_m),
            context={
                "stage": f"named_joint_pose/{label}",
                "named_joint_pose": int(named_joint_pose),
                "updown": float(current.updown_m),
            },
        ),
        label,
    )


def turn_stage_target(current: MotionSample, target_rad: float) -> MotionSample:
    if not math.isfinite(float(target_rad)):
        raise ValueError("Turn 目标必须为有限弧度值")
    joints = dict(current.joints)
    joints["turn"] = float(target_rad)
    return MotionSample(
        time_s=0.1,
        joints=joints,
        updown_m=float(current.updown_m),
        context={
            "stage": "turn",
            "turn_target_rad": float(target_rad),
            "updown": float(current.updown_m),
        },
    )


def cache_result_diagnostic(metrics: dict) -> str:
    cache_path = str(metrics.get("trajectory_cache_path", ""))
    if not cache_path:
        return ""
    cache_key = Path(cache_path).name.removesuffix(".json.gz")
    return (
        f"cache_key={cache_key} "
        f"nearest_success={bool(metrics.get('trajectory_cache_nearest_success_used', False))} "
        f"requested_x={float(metrics['trajectory_cache_requested_distance_m']):.3f}m "
        f"selected_x={float(metrics['trajectory_cache_distance_m']):.3f}m "
        f"requested_y_offset="
        f"{float(metrics['trajectory_cache_requested_lateral_offset_m']):+.3f}m "
        f"selected_y_offset="
        f"{float(metrics['trajectory_cache_lateral_offset_m']):+.3f}m"
    )


def fixed_side_recapture_target(
    current: MotionSample,
    targets,
    *,
    row_split_z_m: float,
    first_row_updown_m: float,
    second_row_updown_m: float,
    arm_pose_deg=SIDE_RECAPTURE_ARM_POSE_DEG,
) -> tuple[MotionSample, int, float] | None:
    if (
        int(targets.left_grasp_mode)
        != DualArmPoseTargets.GRASP_MODE_SIDE_SUCTION
        or int(targets.right_grasp_mode)
        != DualArmPoseTargets.GRASP_MODE_SIDE_SUCTION
    ):
        return None
    if len(arm_pose_deg) != 6 or not all(math.isfinite(float(value)) for value in arm_pose_deg):
        raise ValueError("侧吸重拍固定关节姿态必须包含6个有限角度")
    if not all(
        math.isfinite(float(value))
        for value in (row_split_z_m, first_row_updown_m, second_row_updown_m)
    ):
        raise ValueError("侧吸重拍分层参数必须为有限值")

    average_z_m = 0.5 * (
        float(targets.left_pose.position.z) + float(targets.right_pose.position.z)
    )
    row = 1 if average_z_m >= float(row_split_z_m) else 2
    target_updown_m = (
        float(first_row_updown_m) if row == 1 else float(second_row_updown_m)
    )
    joints = dict(current.joints)
    for side in ("left", "right"):
        for index, value_deg in enumerate(arm_pose_deg, start=1):
            joints[f"{side}_joint{index}"] = math.radians(float(value_deg))
    joints["turn"] = 0.0
    return (
        MotionSample(
            time_s=0.1,
            joints=joints,
            updown_m=target_updown_m,
            context={
                "stage": "recapture/fixed_side",
                "row": row,
                "source_average_z_m": average_z_m,
                "updown": target_updown_m,
            },
        ),
        row,
        average_z_m,
    )


class DomainMotionServer(Node):
    """Single staged Motion ingress; vacuum ownership stays outside Motion."""

    def __init__(self) -> None:
        super().__init__("motion_domain_server")
        default_source_ws = os.environ.get("ARMMOTION_SOURCE_WS", "")
        default_output_root = os.environ.get("ARMMOTION_OUTPUT_ROOT", "/motion_data")
        self.declare_parameter("source_ws", default_source_ws)
        self.declare_parameter("output_root", default_output_root)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("trajectory_rate_hz", 30.0)
        self.declare_parameter("execution_speed_scale", 3.0)
        self.declare_parameter("max_joint_speed_deg_s", 10.0)
        self.declare_parameter("max_joint_acceleration_deg_s2", 60.0)
        self.declare_parameter("max_updown_speed_m_s", 0.15)
        self.declare_parameter("updown_acceleration_m_s2", 0.05)
        self.declare_parameter("max_joint_jerk_deg_s3", 6000.0)
        self.declare_parameter("max_updown_jerk_m_s3", 5.0)
        self.declare_parameter("controller_interpolation_rate_hz", 250.0)
        self.declare_parameter("trajectory_smoothing_path_tolerance", 0.001)
        self.declare_parameter("trajectory_smoothing_resample_dt", 0.1)
        self.declare_parameter("recapture_preferred_updown_m", 0.3)
        self.declare_parameter("recapture_analytic_approach_limit_m", 0.10)
        self.declare_parameter("recapture_analytic_approach_step_m", 0.01)
        self.declare_parameter("side_recapture_fixed_joint_enabled", True)
        self.declare_parameter("side_recapture_row_split_z_m", 1.10)
        self.declare_parameter("side_recapture_first_row_updown_m", 0.40)
        self.declare_parameter("side_recapture_second_row_updown_m", 0.0)
        self.declare_parameter(
            "side_recapture_arm_pose_deg",
            list(SIDE_RECAPTURE_ARM_POSE_DEG),
        )
        self.declare_parameter("trajectory_cache_root", "")
        self.declare_parameter("turn_tf_frame", "turn")
        self.declare_parameter("turn_tf_timeout_s", 1.0)
        self.declare_parameter("turn_zero_target_y_compensation_m", 0.0)
        self.declare_parameter("planner_timeout_s", 180.0)
        self.declare_parameter("interface_timeout_s", 10.0)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_state_max_age_s", 0.2)
        self.declare_parameter("start_joint_tolerance_deg", 0.75)
        self.declare_parameter("start_updown_tolerance_m", 0.015)
        self.declare_parameter(
            "trajectory_action",
            RT_CONTROL_ACTION_NAME,
        )
        self.declare_parameter("stage_action", "/motion/execute_stage")
        self.declare_parameter("readiness_topic", "/motion/readiness")
        self.declare_parameter("initialize_service", "/motion/dev/initialize_loaded_pose")
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
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._lock = threading.RLock()
        self._busy = False
        self._goal_reserved = False
        self._scene_unknown = False
        self._active_plan = None
        self._recapture_sample = None
        self._cycle_serial = 0
        self._cycle_id = ""
        self._producer_instance_id = str(uuid.uuid4())
        self._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW
        self._last_error = self._error(ErrorCode.SUCCESS)
        self._hardware = HardwareExecutor(
            self,
            dry_run=bool(self.get_parameter("dry_run").value),
            action_name=str(self.get_parameter("trajectory_action").value),
            left_solenoid_service="",
            right_solenoid_service="",
            vacuum_pump_service="",
            wait_timeout_s=interface_timeout,
            joint_state_topic=str(self.get_parameter("joint_state_topic").value),
            start_joint_tolerance_deg=float(
                self.get_parameter("start_joint_tolerance_deg").value
            ),
            start_updown_tolerance_m=float(
                self.get_parameter("start_updown_tolerance_m").value
            ),
            manage_grasp_io=False,
        )
        self._hardware.verify_interfaces()
        self._planner = PlannerAdapter(
            source_ws=(
                Path(str(self.get_parameter("source_ws").value))
                if str(self.get_parameter("source_ws").value).strip()
                else None
            ),
            output_root=Path(str(self.get_parameter("output_root").value)),
            rate_hz=rate_hz,
            max_joint_speed_deg_s=max_joint_speed,
            max_joint_acceleration_deg_s2=max_joint_acceleration,
            max_updown_speed_m_s=max_updown_speed,
            max_updown_acceleration_m_s2=updown_acceleration,
            speed_scale=speed_scale,
            timeout_s=float(self.get_parameter("planner_timeout_s").value),
            trajectory_cache_enabled=True,
            trajectory_cache_required=True,
            trajectory_cache_fallback_on_planning_failure=False,
            trajectory_cache_root=(
                Path(str(self.get_parameter("trajectory_cache_root").value)).resolve()
                if str(self.get_parameter("trajectory_cache_root").value).strip()
                else None
            ),
            max_joint_jerk_deg_s3=float(
                self.get_parameter("max_joint_jerk_deg_s3").value
            ),
            max_updown_jerk_m_s3=float(
                self.get_parameter("max_updown_jerk_m_s3").value
            ),
            trajectory_smoothing_path_tolerance=float(
                self.get_parameter("trajectory_smoothing_path_tolerance").value
            ),
            trajectory_smoothing_resample_dt=float(
                self.get_parameter("trajectory_smoothing_resample_dt").value
            ),
            controller_interpolation_rate_hz=float(
                self.get_parameter("controller_interpolation_rate_hz").value
            ),
        )
        self.get_logger().info("Planner 启动期主动预热开始")
        planner_startup_ms = self._planner.start()
        self._readiness_pub = self.create_publisher(
            DomainReadiness,
            str(self.get_parameter("readiness_topic").value),
            latched(),
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
            f"trajectory={self.get_parameter('trajectory_action').value} "
            f"planner_startup={planner_startup_ms:.1f}ms; "
            "吸附通路由 Autonomy/RT-Control 负责"
        )

    @staticmethod
    def _error(
        code: int,
        message: str = "",
        origin: str = "motion",
        retryable: bool = False,
        severity: int | None = None,
        detail: str = "",
    ) -> ErrorInfo:
        error = ErrorInfo()
        try:
            error.code = int(code)
        except (AssertionError, TypeError):
            error.code = str(int(code))
        error.message = str(message)
        error.retryable = bool(retryable)
        if severity is not None:
            error.severity = int(severity)
        elif int(code) == ErrorCode.SUCCESS:
            error.severity = ErrorInfo.OK
        elif retryable:
            error.severity = ErrorInfo.WARN
        else:
            error.severity = ErrorInfo.FAULT
        error.source = str(origin)
        error.detail = str(detail)
        return error

    def _publish_readiness(self) -> None:
        message = DomainReadiness()
        message.header.stamp = self.get_clock().now().to_msg()
        message.domain = "motion"
        message.readiness_name = "motion_execution"
        message.map_version = ""
        message.producer_instance_id = self._producer_instance_id
        with self._lock:
            message.ready = not self._busy and not self._goal_reserved
            blockers: list[str] = []
            planner_running = self._planner.is_running()
            joint_state_fresh = self._hardware.state_is_fresh(
                float(self.get_parameter("joint_state_max_age_s").value)
            )
            if not planner_running:
                message.operational_state = "PLANNER_UNAVAILABLE"
                message.ready = False
                blockers.append("planner_unavailable")
            elif not joint_state_fresh:
                message.operational_state = "JOINT_STATES_STALE"
                message.ready = False
                blockers.append("joint_states_stale")
            elif self._scene_unknown:
                message.operational_state = "SCENE_UNKNOWN"
                message.ready = False
                blockers.append("scene_unknown")
            elif self._busy or self._goal_reserved:
                message.operational_state = "BUSY"
                blockers.append("motion_busy")
            elif (
                self._active_plan is None
                and self._recapture_sample is None
                and not self._cycle_id
            ):
                message.operational_state = "IDLE"
            else:
                message.operational_state = self._stage_name(self._next_stage)
            errors: list[ErrorInfo] = []
            if int(self._last_error.code or ErrorCode.SUCCESS) != ErrorCode.SUCCESS:
                readiness_error = copy.deepcopy(self._last_error)
                if message.ready:
                    readiness_error.severity = ErrorInfo.WARN
                errors.append(readiness_error)
            message.blockers = blockers
            message.errors = errors
            if message.ready:
                message.status = (
                    DomainReadiness.STATUS_DEGRADED
                    if errors
                    else DomainReadiness.STATUS_HEALTHY
                )
            else:
                message.status = DomainReadiness.STATUS_UNAVAILABLE
        self._readiness_pub.publish(message)

    @staticmethod
    def _stage_name(stage: int) -> str:
        return {
            ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW: "WAITING_CAMERA_VIEW",
            ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP: "WAITING_PREGRASP",
            ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH: "WAITING_APPROACH",
            ExecuteMotionStage.Goal.EXECUTION_STAGE_PLACE: "WAITING_PLACE",
            ExecuteMotionStage.Goal.EXECUTION_STAGE_HOME: "WAITING_HOME",
            ExecuteMotionStage.Goal.EXECUTION_STAGE_TURN: "TURN",
            ExecuteMotionStage.Goal.EXECUTION_STAGE_NAMED_JOINT_POSE: (
                "NAMED_JOINT_POSE"
            ),
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
            if stage in INDEPENDENT_EXECUTION_STAGES:
                if (
                    self._active_plan is not None
                    or self._recapture_sample is not None
                    or self._cycle_id
                ):
                    return GoalResponse.REJECT
            elif stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP:
                if pregrasp_entry_mode(
                    active_plan=self._active_plan,
                    recapture_sample=self._recapture_sample,
                    cycle_id=self._cycle_id,
                    next_stage=self._next_stage,
                ) is None:
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
        valid_stages = {
            ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW,
            ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP,
            ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH,
            ExecuteMotionStage.Goal.EXECUTION_STAGE_PLACE,
            ExecuteMotionStage.Goal.EXECUTION_STAGE_HOME,
            ExecuteMotionStage.Goal.EXECUTION_STAGE_TURN,
            ExecuteMotionStage.Goal.EXECUTION_STAGE_NAMED_JOINT_POSE,
        }
        stage = int(request.execution_stage)
        if stage not in valid_stages:
            raise ValueError(f"不支持的 Motion 阶段: {request.execution_stage}")
        if stage in {
            ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW,
            ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP,
        }:
            validate_stage_pose_targets(request)
        else:
            validate_default_stage_targets(request)
        turn_target_rad = float(request.turn_target_rad)
        if not math.isfinite(turn_target_rad):
            raise ValueError("turn_target_rad 必须为有限值")
        if (
            stage != ExecuteMotionStage.Goal.EXECUTION_STAGE_TURN
            and abs(turn_target_rad) > 1e-12
        ):
            raise ValueError("仅 TURN 阶段允许设置 turn_target_rad")
        named_joint_pose = int(request.named_joint_pose)
        valid_named_joint_poses = {
            ExecuteMotionStage.Goal.NAMED_JOINT_POSE_REMOTE_CAMERA_VIEW,
            ExecuteMotionStage.Goal.NAMED_JOINT_POSE_ARM_CONVERGED,
        }
        if stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_NAMED_JOINT_POSE:
            if named_joint_pose not in valid_named_joint_poses:
                raise ValueError(f"不支持的 named_joint_pose: {named_joint_pose}")
        elif named_joint_pose != ExecuteMotionStage.Goal.NAMED_JOINT_POSE_UNSPECIFIED:
            raise ValueError("仅 NAMED_JOINT_POSE 阶段允许设置 named_joint_pose")

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

    def _targets_for_zero_turn(
        self,
        request,
        current: MotionSample,
        *,
        canonicalize_grasp_orientation: bool,
        align_to_lower_height: bool,
        align_to_average_x: bool,
    ):
        current_turn = float(current.joints.get("turn", 0.0))
        corrected_request = copy.deepcopy(request)
        base_to_turn = None
        if abs(current_turn) > 1e-8:
            try:
                base_to_turn = self._tf_buffer.lookup_transform(
                    "base_link",
                    str(self.get_parameter("turn_tf_frame").value),
                    Time(),
                    timeout=Duration(
                        seconds=float(self.get_parameter("turn_tf_timeout_s").value)
                    ),
                )
            except TransformException as exc:
                raise RuntimeError(f"无法将目标换算到 Turn=0：{exc}") from exc
        y_compensation_m = float(
            self.get_parameter("turn_zero_target_y_compensation_m").value
        )
        for side in ("left", "right"):
            grasp_mode = int(
                getattr(corrected_request.targets, f"{side}_grasp_mode")
            )
            if grasp_mode == DualArmPoseTargets.GRASP_MODE_NO_MOVE:
                continue
            original_pose = getattr(corrected_request.targets, f"{side}_pose")
            corrected_pose = original_pose
            if base_to_turn is not None:
                corrected_pose = pose_at_zero_turn(
                    corrected_pose,
                    base_to_turn.transform,
                    current_turn,
                )
            corrected_pose = compensate_pose_y(corrected_pose, y_compensation_m)
            setattr(corrected_request.targets, f"{side}_pose", corrected_pose)
        orientation_deviations: dict[str, float] = {}
        if canonicalize_grasp_orientation:
            corrected_request, orientation_deviations = (
                canonicalize_stage_target_orientations(corrected_request)
            )
        targets = resolve_dual_stage_targets(corrected_request)
        original_left_z = float(targets.left_pose.position.z)
        original_right_z = float(targets.right_pose.position.z)
        if align_to_lower_height:
            targets = align_target_pair_to_lower_height(targets)
        if align_to_average_x:
            targets = align_target_pair_to_average_x(targets)
        lateral_offset_m = 0.5 * (
            float(targets.left_pose.position.y)
            + float(targets.right_pose.position.y)
        )
        self.get_logger().info(
            "目标 Pose 预处理完成："
            f"actual_turn={current_turn:.6f}rad "
            f"y_compensation={y_compensation_m:+.3f}m "
            f"canonical_orientation={canonicalize_grasp_orientation} "
            f"align_lower_height={align_to_lower_height} "
            f"align_average_x={align_to_average_x} "
            f"lateral_offset={lateral_offset_m:+.3f}m "
            f"left=({targets.left_pose.position.x:.3f},"
            f"{targets.left_pose.position.y:.3f},"
            f"{targets.left_pose.position.z:.3f}) "
            f"right=({targets.right_pose.position.x:.3f},"
            f"{targets.right_pose.position.y:.3f},"
            f"{targets.right_pose.position.z:.3f}) "
            f"input_z=L{original_left_z:.3f}/R{original_right_z:.3f} "
            f"output_z=L{targets.left_pose.position.z:.3f}/"
            f"R{targets.right_pose.position.z:.3f} "
            f"grasp_orientation_correction_deg="
            f"L{math.degrees(orientation_deviations.get('left', 0.0)):.2f}/"
            f"R{math.degrees(orientation_deviations.get('right', 0.0)):.2f}"
        )
        return targets

    def _run_plan_action(self, goal_handle, action_name: str, label: str) -> float:
        with self._lock:
            plan = self._active_plan
        if plan is None:
            raise RuntimeError("内部执行计划不存在")
        trajectory = plan.action_trajectories.get(action_name)
        if not trajectory:
            raise RuntimeError(f"内部执行计划缺少动作轨迹: {action_name}")
        if goal_handle.is_cancel_requested:
            raise InterruptedError("动作在轨迹发送前被取消")
        return float(self._hardware.execute_segment(trajectory, label)["duration_s"])

    def _plan_and_execute_joint_target(
        self,
        goal_handle,
        current: MotionSample,
        target: MotionSample,
        *,
        context_label: str,
        execution_label: str,
        hold_turn: bool,
    ) -> tuple[float, float, dict]:
        goal_handle.publish_feedback(
            self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_PLANNING)
        )
        started = time.monotonic()
        samples, smoothing_metrics = self._planner.smooth_motion_samples(
            [current, target],
            context_label=context_label,
        )
        planning_time_s = time.monotonic() - started
        goal_handle.publish_feedback(
            self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_EXECUTING)
        )
        execution_time_s = float(
            self._hardware.execute_segment(
                samples,
                execution_label,
                hold_turn=hold_turn,
            )["duration_s"]
        )
        return planning_time_s, execution_time_s, smoothing_metrics

    @staticmethod
    def _planning_sample_without_external_turn(sample: MotionSample) -> MotionSample:
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
            context={**sample.context, "external_turn_ignored": True},
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
        result_detail = ""
        with self._lock:
            self._busy = True
            self._goal_reserved = False
        self._publish_readiness()
        try:
            if stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_TURN:
                current = self._current_sample_for_planning()
                target = turn_stage_target(current, request.turn_target_rad)
                (
                    planning_time_s,
                    execution_time_s,
                    smoothing_metrics,
                ) = self._plan_and_execute_joint_target(
                    goal_handle,
                    current,
                    target,
                    context_label="turn",
                    execution_label="Turn 独立运动",
                    hold_turn=False,
                )
                result_detail = (
                    f"turn_target_rad={float(request.turn_target_rad):.6f}; "
                    f"smoothing={smoothing_metrics.get('mode', 'unknown')}"
                )
            elif stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_NAMED_JOINT_POSE:
                current = self._current_sample_for_planning()
                target, named_label = named_joint_pose_target(
                    current,
                    request.named_joint_pose,
                )
                (
                    planning_time_s,
                    execution_time_s,
                    smoothing_metrics,
                ) = self._plan_and_execute_joint_target(
                    goal_handle,
                    current,
                    target,
                    context_label=f"named_joint_pose/{named_label}",
                    execution_label=f"命名姿态 {named_label}",
                    hold_turn=True,
                )
                result_detail = (
                    f"named_joint_pose={named_label}; "
                    f"smoothing={smoothing_metrics.get('mode', 'unknown')}"
                )
            elif stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW:
                current_with_turn = self._current_sample_for_planning()
                current = self._planning_sample_without_external_turn(current_with_turn)
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_PLANNING)
                )
                started = time.monotonic()
                fixed_side = None
                if bool(self.get_parameter("side_recapture_fixed_joint_enabled").value):
                    side_recapture_arm_pose_deg = tuple(
                        float(value)
                        for value in self.get_parameter(
                            "side_recapture_arm_pose_deg"
                        ).value
                    )
                    fixed_side = fixed_side_recapture_target(
                        current,
                        resolve_dual_stage_targets(request),
                        row_split_z_m=float(
                            self.get_parameter("side_recapture_row_split_z_m").value
                        ),
                        first_row_updown_m=float(
                            self.get_parameter("side_recapture_first_row_updown_m").value
                        ),
                        second_row_updown_m=float(
                            self.get_parameter("side_recapture_second_row_updown_m").value
                        ),
                        arm_pose_deg=side_recapture_arm_pose_deg,
                    )
                if fixed_side is not None:
                    target, side_row, source_average_z_m = fixed_side
                    samples, smoothing_metrics = self._planner.smooth_motion_samples(
                        [current, target],
                        context_label="camera_view/fixed_side",
                    )
                    metrics = {
                        "selected_updown": target.updown_m,
                        "viewpose_approach_offset_m": 0.0,
                        "numeric_fallback_used": False,
                        "ik_ms": 0.0,
                        "planning_ms": 0.0,
                        "trajectory_smoothing": smoothing_metrics,
                    }
                    self.get_logger().info(
                        "侧吸重拍位使用固定关节姿态："
                        f"row={side_row} source_average_z={source_average_z_m:.3f}m "
                        f"updown={target.updown_m:.3f}m "
                        f"arm_pose_deg={list(side_recapture_arm_pose_deg)}"
                    )
                else:
                    targets = self._targets_for_zero_turn(
                        request,
                        current_with_turn,
                        canonicalize_grasp_orientation=True,
                        align_to_lower_height=False,
                        align_to_average_x=True,
                    )
                    samples, metrics = self._planner.plan_recapture_with_approach_search(
                        self._base_link_pose_stamped(targets.left_pose),
                        self._base_link_pose_stamped(targets.right_pose),
                        current,
                        preferred_updown=float(
                            self.get_parameter("recapture_preferred_updown_m").value
                        ),
                        approach_limit_m=float(
                            self.get_parameter("recapture_analytic_approach_limit_m").value
                        ),
                        approach_step_m=float(
                            self.get_parameter("recapture_analytic_approach_step_m").value
                        ),
                    )
                    self.get_logger().info(
                        "顶吸重拍位规划完成："
                        f"updown={metrics['selected_updown']:.3f}m "
                        f"x_approach={metrics['viewpose_approach_offset_m']:.3f}m "
                        f"numeric_fallback={metrics['numeric_fallback_used']} "
                        f"ik={metrics['ik_ms']:.2f}ms plan={metrics['planning_ms']:.2f}ms "
                        f"mirrored_from={targets.mirrored_from or 'none'}"
                    )
                planning_time_s = time.monotonic() - started
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_EXECUTING)
                )
                execution_time_s += float(
                    self._hardware.execute_segment(samples, "重拍位")["duration_s"]
                )
            elif stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP:
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_PLANNING)
                )
                with self._lock:
                    cycle_id = self._cycle_id
                    recapture_sample = self._recapture_sample
                    entry_mode = pregrasp_entry_mode(
                        active_plan=self._active_plan,
                        recapture_sample=recapture_sample,
                        cycle_id=cycle_id,
                        next_stage=self._next_stage,
                    )
                if entry_mode is None:
                    raise RuntimeError("PREGRASP 入口状态已失效")
                current_with_turn = self._current_sample_for_planning()
                current = self._planning_sample_without_external_turn(current_with_turn)
                with self._lock:
                    self._cycle_serial += 1
                    cycle_id = f"motion-cycle-{self._cycle_serial:06d}"
                    self._cycle_id = cycle_id
                self.get_logger().info(
                    "PREGRASP 从当前机器人真实状态建立抓取流程"
                )
                planning_start = pregrasp_planning_start_sample(
                    entry_mode=entry_mode,
                    current_sample=current,
                    recapture_sample=recapture_sample,
                )
                targets = self._targets_for_zero_turn(
                    request,
                    current_with_turn,
                    canonicalize_grasp_orientation=True,
                    align_to_lower_height=True,
                    align_to_average_x=True,
                )
                task = planning_task_from_resolved_targets(targets, cycle_id)
                started = time.monotonic()
                plan = self._planner.compute(task, initial_sample=planning_start)
                planning_time_s = time.monotonic() - started
                self.get_logger().info(
                    "任务缓存规划完成："
                    f"requested_x={plan.metrics['trajectory_cache_requested_distance_m']:.3f}m "
                    f"selected_x={plan.metrics['trajectory_cache_distance_m']:.3f}m "
                    f"requested_y_offset="
                    f"{plan.metrics['trajectory_cache_requested_lateral_offset_m']:+.3f}m "
                    f"selected_y_offset="
                    f"{plan.metrics['trajectory_cache_lateral_offset_m']:+.3f}m "
                    f"nearest_success="
                    f"{plan.metrics['trajectory_cache_nearest_success_used']} "
                    f"bridge={plan.metrics['trajectory_cache_bridge_ms']:.2f}ms "
                    f"cache={plan.metrics['trajectory_cache_path']}"
                )
                result_detail = cache_result_diagnostic(plan.metrics)
                with self._lock:
                    self._active_plan = plan
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_EXECUTING)
                )
                execution_time_s = self._run_plan_action(
                    goal_handle,
                    "pregrasp",
                    "预抓取",
                )
                self._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH
            elif stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH:
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_EXECUTING)
                )
                execution_time_s = self._run_plan_action(
                    goal_handle,
                    "approach",
                    "靠近吸附",
                )
                self._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_PLACE
            elif stage == ExecuteMotionStage.Goal.EXECUTION_STAGE_PLACE:
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_EXECUTING)
                )
                execution_time_s = self._run_plan_action(
                    goal_handle,
                    "place",
                    "抽离到放置",
                )
                self._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_HOME
            else:
                goal_handle.publish_feedback(
                    self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_EXECUTING)
                )
                execution_time_s = self._run_plan_action(
                    goal_handle,
                    "home",
                    "返回初始位",
                )
                with self._lock:
                    self._active_plan = None
                    self._recapture_sample = None
                    self._cycle_id = ""
                self._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW

            result.diagnostic = (
                f"{self._stage_name(stage)} complete; "
                f"planning={planning_time_s:.3f}s execution={execution_time_s:.3f}s"
                + (f"; {result_detail}" if result_detail else "")
            )
            goal_handle.publish_feedback(
                self._feedback(ExecuteMotionStage.Feedback.MOTION_STATE_SETTLING)
            )
            goal_handle.succeed()
            with self._lock:
                self._last_error = self._error(ErrorCode.SUCCESS)
            result.ok = True
            result.error = self._error(ErrorCode.SUCCESS)
        except InterruptedError as exc:
            error = self._error(ErrorCode.CANCELED, str(exc))
            result.ok = False
            result.error = error
            result.diagnostic = str(exc)
            goal_handle.canceled()
            self._handle_stage_failure(stage, error)
        except Exception as exc:
            error_code = (
                self._planning_error_code(str(exc))
                if stage in {
                    ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW,
                    ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP,
                    ExecuteMotionStage.Goal.EXECUTION_STAGE_TURN,
                    ExecuteMotionStage.Goal.EXECUTION_STAGE_NAMED_JOINT_POSE,
                }
                else ErrorCode.MOTION_EXECUTION_FAILED
            )
            error = self._error(
                error_code,
                str(exc),
                retryable=error_code in {
                    ErrorCode.MOTION_PLANNING_FAILED,
                    ErrorCode.MOTION_COLLISION_DETECTED,
                    ErrorCode.MOTION_IK_NO_SOLUTION,
                },
            )
            result.ok = False
            result.error = error
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

    @staticmethod
    def _planning_error_code(message: str) -> int:
        normalized = message.lower()
        if "collision" in normalized or "碰撞" in message:
            return ErrorCode.MOTION_COLLISION_DETECTED
        if "ik" in normalized or "逆解" in message or "无解" in message:
            return ErrorCode.MOTION_IK_NO_SOLUTION
        return ErrorCode.MOTION_PLANNING_FAILED

    def _handle_stage_failure(self, stage: int, error: ErrorInfo) -> None:
        with self._lock:
            self._last_error = error
            if stage in {
                ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH,
                ExecuteMotionStage.Goal.EXECUTION_STAGE_PLACE,
                ExecuteMotionStage.Goal.EXECUTION_STAGE_HOME,
            }:
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
        samples, _ = self._planner.smooth_motion_samples(
            [current, target],
            context_label="initialization/loaded",
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
