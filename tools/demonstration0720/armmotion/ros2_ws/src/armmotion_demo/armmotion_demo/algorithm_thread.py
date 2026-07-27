from __future__ import annotations

import os
import threading
import time
import traceback
from pathlib import Path
from typing import Any

import rclpy
from alfa_robot_execution_bridge.joints import EXECUTION_JOINT_NAMES
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .common import (
    STAGE_LABELS,
    MotionSample,
    decode_message,
    encode_message,
    loaded_joint_map,
    parse_task_code,
    retime_segment,
)
from .hardware_executor import ARM_JOINT_NAMES, HardwareExecutor
from .planner_adapter import PlannerAdapter


STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


class AlgorithmThread(Node):
    def __init__(self) -> None:
        super().__init__("armmotion_algorithm_thread")
        default_source_ws = os.environ.get(
            "ARMMOTION_SOURCE_WS",
            "/home/ar/demostration0720/src/armmotion/ros2_ws",
        )
        default_output_root = os.environ.get(
            "ARMMOTION_OUTPUT_ROOT",
            "/home/ar/demostration0720/src/armmotion/data",
        )
        self.declare_parameter("source_ws", default_source_ws)
        self.declare_parameter("output_root", default_output_root)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("trajectory_rate_hz", 10.0)
        self.declare_parameter("execution_speed_scale", 1.0)
        self.declare_parameter("max_joint_speed_deg_s", 10.0)
        self.declare_parameter("max_joint_acceleration_deg_s2", 60.0)
        self.declare_parameter("max_updown_speed_m_s", 0.05)
        self.declare_parameter("updown_acceleration_m_s2", 0.05)
        self.declare_parameter("updown_deceleration_m_s2", 0.05)
        self.declare_parameter("planner_timeout_s", 180.0)
        self.declare_parameter("interface_timeout_s", 10.0)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("start_joint_tolerance_deg", 5.0)
        self.declare_parameter("start_updown_tolerance_m", 0.015)
        self.declare_parameter(
            "action_name",
            "/dual_arm_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter(
            "updown_topic",
            "/canopen/updown_position_controller/commands",
        )
        self.declare_parameter("left_solenoid_service", "/plc/left_solenoid")
        self.declare_parameter("right_solenoid_service", "/plc/right_solenoid")

        rate_hz = float(self.get_parameter("trajectory_rate_hz").value)
        max_joint_speed_deg_s = float(self.get_parameter("max_joint_speed_deg_s").value)
        max_joint_acceleration_deg_s2 = float(
            self.get_parameter("max_joint_acceleration_deg_s2").value
        )
        max_updown_speed_m_s = float(self.get_parameter("max_updown_speed_m_s").value)
        execution_speed_scale = float(self.get_parameter("execution_speed_scale").value)
        self.planner = PlannerAdapter(
            source_ws=Path(str(self.get_parameter("source_ws").value)),
            output_root=Path(str(self.get_parameter("output_root").value)),
            rate_hz=rate_hz,
            max_joint_speed_deg_s=max_joint_speed_deg_s,
            max_joint_acceleration_deg_s2=max_joint_acceleration_deg_s2,
            max_updown_speed_m_s=max_updown_speed_m_s,
            speed_scale=execution_speed_scale,
            timeout_s=float(self.get_parameter("planner_timeout_s").value),
        )
        self.hardware = HardwareExecutor(
            self,
            dry_run=bool(self.get_parameter("dry_run").value),
            action_name=str(self.get_parameter("action_name").value),
            updown_topic=str(self.get_parameter("updown_topic").value),
            left_solenoid_service=str(self.get_parameter("left_solenoid_service").value),
            right_solenoid_service=str(self.get_parameter("right_solenoid_service").value),
            max_updown_speed_m_s=max_updown_speed_m_s * execution_speed_scale,
            updown_acceleration_m_s2=float(
                self.get_parameter("updown_acceleration_m_s2").value
            ),
            updown_deceleration_m_s2=float(
                self.get_parameter("updown_deceleration_m_s2").value
            ),
            wait_timeout_s=float(self.get_parameter("interface_timeout_s").value),
            joint_state_topic=str(self.get_parameter("joint_state_topic").value),
            start_joint_tolerance_deg=float(
                self.get_parameter("start_joint_tolerance_deg").value
            ),
            start_updown_tolerance_m=float(
                self.get_parameter("start_updown_tolerance_m").value
            ),
        )
        self.status_publisher = self.create_publisher(String, "/armmotion/status", STATUS_QOS)
        self.create_subscription(String, "/armmotion/task_request", self._on_task_request, 10)
        self.create_subscription(String, "/armmotion/next_stage", self._on_next_stage, 10)
        self.create_subscription(String, "/armmotion/initialize", self._on_initialize, 10)
        self._lock = threading.Lock()
        self._planning = False
        self._executing = False
        self._plan = None
        self._request_id = ""
        self._completed_stage = 0
        self.hardware.verify_interfaces()
        self._publish(
            "ready",
            dry_run=bool(self.get_parameter("dry_run").value),
            trajectory_rate_hz=rate_hz,
            max_joint_speed_deg_s=max_joint_speed_deg_s,
            max_updown_speed_m_s=max_updown_speed_m_s,
            execution_speed_scale=execution_speed_scale,
            effective_max_joint_speed_deg_s=max_joint_speed_deg_s * execution_speed_scale,
            effective_max_updown_speed_m_s=max_updown_speed_m_s * execution_speed_scale,
            planner_session_startup_ms=self.planner.startup_ms,
            planner_session_log=str(self.planner.session_log),
        )
        self.get_logger().info(
            "算法线程已就绪："
            f"trajectory={rate_hz:.1f}Hz speed_scale={execution_speed_scale:.1f}x "
            f"joint_limit={max_joint_speed_deg_s * execution_speed_scale:.1f}deg/s "
            f"updown_limit={max_updown_speed_m_s * execution_speed_scale:.3f}m/s；"
            "等待 A1..A5/B1..B5 任务"
        )

    def _on_initialize(self, message: String) -> None:
        try:
            request = decode_message(message.data)
            request_id = str(request["request_id"])
        except Exception as exc:
            self._publish("initialization_failed", reason=str(exc))
            return
        with self._lock:
            if self._planning or self._executing or self._plan is not None:
                self._publish(
                    "initialization_failed",
                    request_id=request_id,
                    reason="算法线程正忙",
                )
                return
            self._executing = True
        threading.Thread(
            target=self._initialize_loaded_pose,
            args=(request_id,),
            daemon=True,
            name=f"initialize-{request_id}",
        ).start()

    def _initialize_loaded_pose(self, request_id: str) -> None:
        started = time.monotonic()
        self._publish("initialization_started", request_id=request_id)
        self.get_logger().info("初始化开始：当前位置 -> 负重位，turn 保持当前值")
        try:
            target_joints = loaded_joint_map(EXECUTION_JOINT_NAMES)
            if self.hardware.dry_run:
                current = MotionSample(
                    time_s=0.0,
                    joints=dict(target_joints),
                    updown_m=0.3,
                    context={"stage": "initialization/current", "updown": 0.3},
                )
            else:
                current = self.hardware.current_sample()
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
                rate_hz=float(self.get_parameter("trajectory_rate_hz").value),
                max_joint_speed_deg_s=float(
                    self.get_parameter("max_joint_speed_deg_s").value
                ),
                max_updown_speed_m_s=float(
                    self.get_parameter("max_updown_speed_m_s").value
                ),
                speed_scale=float(self.get_parameter("execution_speed_scale").value),
                max_joint_acceleration_deg_s2=float(
                    self.get_parameter("max_joint_acceleration_deg_s2").value
                ),
            )
            metrics = self.hardware.execute_segment(samples, "初始化负重位")
            wall_ms = (time.monotonic() - started) * 1000.0
            with self._lock:
                self._executing = False
            self._publish(
                "initialization_complete",
                request_id=request_id,
                wall_ms=wall_ms,
                planned_duration_s=float(metrics.get("duration_s", 0.0)),
            )
            self.get_logger().info(f"初始化负重位完成：wall={wall_ms:.1f}ms")
        except Exception as exc:
            with self._lock:
                self._executing = False
            self._publish(
                "initialization_failed",
                request_id=request_id,
                reason=str(exc),
            )
            self.get_logger().error(f"初始化负重位失败: {exc}")

    def _publish(self, event: str, **fields: Any) -> None:
        message = String()
        message.data = encode_message(event, timestamp=time.time(), **fields)
        self.status_publisher.publish(message)

    def _on_task_request(self, message: String) -> None:
        try:
            request = decode_message(message.data)
            request_id = str(request["request_id"])
            task = parse_task_code(
                str(request["task_code"]),
                float(request["front_distance_m"]),
                float(request["top_distance_m"]),
            )
        except Exception as exc:
            self._publish("request_rejected", reason=str(exc))
            return
        with self._lock:
            if self._planning or self._executing or self._plan is not None:
                self._publish(
                    "request_rejected",
                    request_id=request_id,
                    task_code=task.code,
                    reason="上一任务尚未完成",
                )
                return
            self._planning = True
            self._request_id = request_id
            self._completed_stage = 0
        threading.Thread(
            target=self._compute_plan,
            args=(request_id, task),
            daemon=True,
            name=f"plan-{request_id}",
        ).start()

    def _compute_plan(self, request_id: str, task) -> None:
        started = time.monotonic()
        self._publish(
            "planning_started",
            request_id=request_id,
            task_code=task.code,
            left_box_id=task.left_box_id,
            right_box_id=task.right_box_id,
            layout=task.layout,
            grasp_family=task.grasp_family,
            extraction_mode=task.extraction_mode,
            front_distance_m=task.front_distance_m,
            top_distance_m=task.top_distance_m,
        )
        self.get_logger().info(
            f"[{task.code}] 计算开始：L{task.left_box_id}/R{task.right_box_id} "
            f"front={task.front_distance_m:.3f}m top={task.top_distance_m:.3f}m"
        )
        try:
            plan = self.planner.compute(task)
            wall_ms = (time.monotonic() - started) * 1000.0
            with self._lock:
                self._plan = plan
                self._planning = False
            metrics = dict(plan.metrics)
            self._publish(
                "planning_complete",
                request_id=request_id,
                task_code=task.code,
                wall_ms=wall_ms,
                total_ms=float(metrics.get("total_ms", 0.0)),
                ik_ms=float(metrics.get("ik_ms", 0.0)),
                extract_ms=float(metrics.get("extract_ms", 0.0)),
                loaded_ms=float(metrics.get("loaded_ms", 0.0)),
                final_ms=float(metrics.get("final_ms", 0.0)),
                snapshot=str(plan.snapshot_path),
                planner_log=str(metrics.get("planner_log", "")),
            )
            self.get_logger().info(
                f"[{task.code}] 计算完成：wall={wall_ms:.1f}ms；等待第1次回车"
            )
        except Exception as exc:
            with self._lock:
                self._planning = False
                self._plan = None
                self._request_id = ""
            self._publish(
                "failed",
                request_id=request_id,
                task_code=task.code,
                phase="planning",
                reason=str(exc),
            )
            self.get_logger().error(f"[{task.code}] 规划失败: {exc}")
            self.get_logger().debug(traceback.format_exc())

    def _on_next_stage(self, message: String) -> None:
        try:
            request = decode_message(message.data)
            request_id = str(request["request_id"])
            stage_number = int(request["stage"])
        except Exception as exc:
            self._publish("stage_rejected", reason=str(exc))
            return
        with self._lock:
            expected = self._completed_stage + 1
            if self._plan is None:
                reason = "当前没有已完成规划的任务"
            elif request_id != self._request_id:
                reason = "request_id 与当前任务不一致"
            elif self._executing:
                reason = "上一阶段仍在执行"
            elif stage_number != expected:
                reason = f"阶段顺序错误：期望 {expected}，收到 {stage_number}"
            else:
                reason = ""
                self._executing = True
        if reason:
            self._publish(
                "stage_rejected",
                request_id=request_id,
                stage=stage_number,
                reason=reason,
            )
            return
        threading.Thread(
            target=self._execute_stage,
            args=(request_id, stage_number),
            daemon=True,
            name=f"execute-{request_id}-{stage_number}",
        ).start()

    def _execute_stage(self, request_id: str, stage_number: int) -> None:
        plan = self._plan
        if plan is None:
            return
        label = STAGE_LABELS[stage_number]
        started = time.monotonic()
        self._publish(
            "stage_started",
            request_id=request_id,
            task_code=plan.task.code,
            stage=stage_number,
            label=label,
        )
        self.get_logger().info(f"[{plan.task.code}] 第{stage_number}/7步开始：{label}")
        try:
            segment_metrics = []
            for segment_index, segment in enumerate(plan.stages[stage_number], start=1):
                segment_metrics.append(
                    self.hardware.execute_segment(
                        segment,
                        f"{plan.task.code} 第{stage_number}步/{segment_index}",
                    )
                )
            if stage_number == 2:
                self.hardware.set_grasp_solenoids(True)
            elif stage_number == 6:
                self.hardware.set_grasp_solenoids(False)
            wall_ms = (time.monotonic() - started) * 1000.0
            planned_duration_s = sum(
                float(metrics.get("duration_s", 0.0)) for metrics in segment_metrics
            )
            with self._lock:
                self._completed_stage = stage_number
                self._executing = False
            self._publish(
                "stage_complete",
                request_id=request_id,
                task_code=plan.task.code,
                stage=stage_number,
                label=label,
                wall_ms=wall_ms,
                planned_duration_s=planned_duration_s,
                segment_metrics=segment_metrics,
            )
            self.get_logger().info(
                f"[{plan.task.code}] 第{stage_number}/7步完成：wall={wall_ms:.1f}ms"
            )
            if stage_number == 7:
                self._publish(
                    "task_complete",
                    request_id=request_id,
                    task_code=plan.task.code,
                    snapshot=str(plan.snapshot_path),
                )
                with self._lock:
                    self._plan = None
                    self._request_id = ""
                    self._completed_stage = 0
                self.get_logger().info(f"[{plan.task.code}] 七步任务全部完成，等待下一任务")
        except Exception as exc:
            with self._lock:
                self._executing = False
                self._plan = None
                self._request_id = ""
                self._completed_stage = 0
            self._publish(
                "failed",
                request_id=request_id,
                task_code=plan.task.code,
                phase="execution",
                stage=stage_number,
                reason=str(exc),
            )
            self.get_logger().error(
                f"[{plan.task.code}] 第{stage_number}步失败，任务已中止: {exc}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlgorithmThread()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.planner.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
