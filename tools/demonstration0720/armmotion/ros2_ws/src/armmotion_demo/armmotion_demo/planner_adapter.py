from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from alfa_robot_execution_bridge.joints import EXECUTION_JOINT_NAMES
from geometry_msgs.msg import PoseStamped

from .common import (
    ExecutionPlan,
    MotionSample,
    PoseTaskSpec,
    TaskSpec,
    planning_task_from_suction_surface_poses,
    pose6d_from_dict,
    retime_segment,
    retime_all_stages,
    split_execution_stages,
    validate_stage_contracts,
)
from .trajectory_cache import TrajectoryCache, format_cache_miss_diagnostic


PlanningTask = TaskSpec | PoseTaskSpec


def _load_script_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载 Python 模块: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class PlannerAdapter:
    def __init__(
        self,
        *,
        source_ws: Path,
        output_root: Path,
        rate_hz: float,
        max_joint_speed_deg_s: float,
        max_joint_acceleration_deg_s2: float,
        max_updown_speed_m_s: float,
        max_updown_acceleration_m_s2: float,
        speed_scale: float,
        timeout_s: float,
        trajectory_cache_enabled: bool = False,
        trajectory_cache_required: bool = False,
        trajectory_cache_fallback_on_planning_failure: bool = False,
        trajectory_cache_root: Path | None = None,
    ) -> None:
        self.source_ws = source_ws.resolve()
        self.output_root = output_root.resolve()
        self.rate_hz = float(rate_hz)
        self.max_joint_speed_deg_s = float(max_joint_speed_deg_s)
        self.max_joint_acceleration_deg_s2 = float(max_joint_acceleration_deg_s2)
        self.max_updown_speed_m_s = float(max_updown_speed_m_s)
        self.max_updown_acceleration_m_s2 = float(max_updown_acceleration_m_s2)
        self.speed_scale = float(speed_scale)
        if self.speed_scale <= 0.0:
            raise ValueError("speed_scale 必须为正数")
        self.timeout_s = float(timeout_s)
        self.trajectory_cache = TrajectoryCache(
            trajectory_cache_root,
            enabled=trajectory_cache_enabled,
        )
        self.trajectory_cache_required = bool(trajectory_cache_required)
        self.trajectory_cache_fallback_on_planning_failure = bool(
            trajectory_cache_fallback_on_planning_failure
        )
        self.scripts_dir = self.source_ws / "src/alfa_robot_moveit_config/scripts"
        self.planner_script = self.scripts_dir / "extract_sequence_rerun.py"
        self.execution_script = self.scripts_dir / "execute_l6_r8_mock_live.py"
        for path in (self.planner_script, self.execution_script):
            if not path.is_file():
                raise FileNotFoundError(path)
        if str(self.scripts_dir) not in sys.path:
            sys.path.insert(0, str(self.scripts_dir))
        self.execution_helpers = _load_script_module(
            "armmotion_execution_helpers",
            self.execution_script,
        )
        self.sequence_helpers = _load_script_module(
            "armmotion_sequence_helpers",
            self.planner_script,
        )
        self.monitor_helpers = self.sequence_helpers.monitor
        self.session_root = self.output_root / f"planner_session_{int(time.time() * 1000)}"
        self.session_root.mkdir(parents=True, exist_ok=False)
        self.session_log = self.session_root / "planner_session.log"
        self._planner_process: subprocess.Popen[str] | None = None
        self._service_client = None
        self._recapture_client = None
        self.startup_ms = 0.0

    def _ensure_session(self) -> None:
        if self._planner_process is not None and self._planner_process.poll() is None:
            return
        self.close()
        try:
            self._start_session()
        except Exception:
            self.close()
            raise

    def start(self) -> float:
        self._ensure_session()
        return self.startup_ms

    def _start_session(self) -> None:
        command = [
            sys.executable,
            str(self.planner_script),
            "--planner-server",
            "--external-control-stack",
            "--planning-joint-states-topic",
            "/motion/internal/model_joint_states",
            "--pair-sequence",
            "1,3",
            "--task-layout",
            "centered",
            "--box-front-x",
            "0.9",
            "--top-box-front-x",
            "0.7",
            "--container-height",
            "2.4",
            "--fixed-updown",
            "0.3",
            "--loaded-updown",
            "0.1",
            "--no-loaded-preserve-lower-updown",
            "--place-updown",
            "0.1",
            "--place-transition-updown",
            "0.1",
            "--output-root",
            str(self.session_root),
            "--no-rerun",
            "--service-timeout",
            "60.0",
            "--startup-retries",
            "1",
            "--ros-domain-id",
            "inherit",
        ]
        environment = os.environ.copy()
        environment["ALFA_ROBOT_ROOT"] = str(self.source_ws.parent)
        started = time.monotonic()
        log_stream = self.session_log.open("w", encoding="utf-8")
        try:
            self._planner_process = subprocess.Popen(
                command,
                cwd=self.source_ws,
                env=environment,
                stdout=log_stream,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        finally:
            log_stream.close()
        deadline = started + self.timeout_s
        ready_marker = "PLANNER_SERVER_READY"
        while time.monotonic() < deadline:
            if self._planner_process.poll() is not None:
                raise RuntimeError(
                    f"planner 长驻进程启动失败 exit={self._planner_process.returncode}; "
                    f"{self._log_tail(self.session_log)}"
                )
            if ready_marker in self.session_log.read_text(
                encoding="utf-8", errors="replace"
            ):
                break
            time.sleep(0.1)
        else:
            self.close()
            raise TimeoutError(
                f"planner 长驻进程 {self.timeout_s:.1f}s 内未就绪; "
                f"{self._log_tail(self.session_log)}"
            )
        self._service_client = self.monitor_helpers.ExtractMonitorServiceClient(
            configure_service="/dual_arm_planner/configure_extract_monitor",
            trigger_service="/dual_arm_planner/run_extract_monitor_full_selected",
            timeout=self.timeout_s,
            node_name="armmotion_persistent_planner_client",
        )
        from alfa_robot_moveit_config.srv import PlanRecapture

        self._recapture_type = PlanRecapture
        self._recapture_client = self._service_client.node.create_client(
            PlanRecapture,
            "/dual_arm_planner/plan_recapture",
        )
        if not self._recapture_client.wait_for_service(timeout_sec=self.timeout_s):
            raise TimeoutError("service /dual_arm_planner/plan_recapture not available")
        self.startup_ms = (time.monotonic() - started) * 1000.0
        print(
            f"planner 长驻会话已就绪：startup={self.startup_ms:.1f}ms "
            f"log={self.session_log}",
            flush=True,
        )

    def _target_args(self, task: PlanningTask) -> SimpleNamespace:
        scene_y_shift = getattr(task, "scene_y_shift", None)
        if scene_y_shift is None:
            scene_y_shift = self.sequence_helpers.TASK_LAYOUT_Y_OFFSETS[task.task_layout]
        return SimpleNamespace(
            box_front_x=task.effective_distance_m,
            scene_y_shift=float(scene_y_shift),
            world_to_base_z=0.202094,
            top_suction_x_offset=0.15,
            top_suction_z_offset=0.2,
        )

    def _runtime_config(self, task: PlanningTask, target_args: SimpleNamespace) -> dict[str, Any]:
        return {
            "box_front_x": task.effective_distance_m,
            "scene_y_shift": target_args.scene_y_shift,
            "extract_rollout_mode": task.extraction_mode,
            "loaded_lateral_shift_enabled": (
                task.left_grasp_mode == "front" or task.right_grasp_mode == "front"
            ),
            "extract_box_pose_rrt_max_iterations": int(
                getattr(task, "extract_box_pose_rrt_max_iterations", 400 if task.index == 5 else 160)
            ),
        }

    def _summary_from_snapshot(
        self,
        *,
        task: PlanningTask,
        snapshot: dict[str, Any],
        snapshot_path: Path,
        configure_ms: float,
        service_ms: float,
        planner_wall_ms: float,
        success: bool,
        failure_reason: str = "",
    ) -> dict[str, Any]:
        place_cycle = snapshot.get("place_cycle", {})
        if not isinstance(place_cycle, dict):
            place_cycle = {}
        summary: dict[str, Any] = {
            "left_row": getattr(task, "left_row", 0),
            "right_row": getattr(task, "right_row", 0),
            "success": success,
            "startup_ms": 0.0,
            "planner_session_startup_ms": self.startup_ms,
            "configure_ms": configure_ms,
            "service_ms": service_ms,
            "wall_ms": planner_wall_ms,
            "snapshot": str(snapshot_path),
            "total_ms": float(snapshot.get("elapsed_ms", 0.0)),
            "ik_ms": float(snapshot.get("ik_elapsed_ms", 0.0)),
            "extract_ms": float(snapshot.get("extract_elapsed_ms", 0.0)),
            "loaded_ms": float(snapshot.get("loaded_elapsed_ms", 0.0)),
            "loaded_plan_batch_wall_ms": float(
                snapshot.get("loaded_plan_batch_wall_ms", 0.0)
            ),
            "loaded_plan_candidate_count": int(
                snapshot.get("loaded_plan_candidate_count", 0)
            ),
            "loaded_plan_attempted_count": int(
                snapshot.get("loaded_plan_attempted_count", 0)
            ),
            "loaded_plan_success_count": int(
                snapshot.get("loaded_plan_success_count", 0)
            ),
            "loaded_parallel_workers": int(snapshot.get("loaded_parallel_workers", 0)),
            "final_ms": float(snapshot.get("final_elapsed_ms", 0.0)),
            "loaded_to_place_ms": float(place_cycle.get("loaded_to_place_ms", 0.0)),
            "place_to_loaded_ms": float(place_cycle.get("place_to_loaded_ms", 0.0)),
            "failure_reason": failure_reason,
        }
        summary.update(self.sequence_helpers.summarize_snapshot_motion(snapshot))
        return summary

    def plan_recapture(
        self,
        left_target: PoseStamped,
        right_target: PoseStamped,
        current: MotionSample,
        *,
        preferred_updown: float = 0.3,
        exact_target: MotionSample | None = None,
        context_stage: str = "recapture/planned",
    ) -> tuple[list[MotionSample], dict[str, Any]]:
        self._ensure_session()
        if self._recapture_client is None:
            raise RuntimeError("recapture planner client 未初始化")
        request = self._recapture_type.Request()
        request.left_target = left_target
        request.right_target = right_target
        request.preferred_updown = float(preferred_updown)
        request.use_exact_target_state = exact_target is not None
        request.start_state.name = [
            "updown",
            "left_joint1", "left_joint2", "left_joint3",
            "left_joint4", "left_joint5", "left_joint6",
            "right_joint1", "right_joint2", "right_joint3",
            "right_joint4", "right_joint5", "right_joint6",
        ]
        request.start_state.position = [
            float(current.updown_m),
            *(float(current.joints[name]) for name in request.start_state.name[1:]),
        ]
        if exact_target is not None:
            request.exact_target_state.name = list(request.start_state.name)
            request.exact_target_state.position = [
                float(exact_target.updown_m),
                *(
                    float(exact_target.joints[name])
                    for name in request.exact_target_state.name[1:]
                ),
            ]
        started = time.monotonic()
        future = self._recapture_client.call_async(request)
        self._service_client._rclpy.spin_until_future_complete(
            self._service_client.node,
            future,
            timeout_sec=self.timeout_s,
        )
        wall_ms = (time.monotonic() - started) * 1000.0
        if not future.done():
            raise TimeoutError(f"重拍位规划超时 {self.timeout_s:.1f}s")
        response = future.result()
        if response is None:
            raise RuntimeError(f"重拍位规划调用失败: {future.exception()}")
        if not response.success:
            raise RuntimeError("重拍位规划失败: " + response.message)

        joint_map = dict(current.joints)
        updown = float(current.updown_m)
        raw_samples = [
            MotionSample(
                time_s=0.0,
                joints=dict(joint_map),
                updown_m=updown,
                context={"stage": context_stage, "updown": updown},
            )
        ]
        trajectory = response.trajectory
        for point in trajectory.points:
            for index, name in enumerate(trajectory.joint_names):
                if index >= len(point.positions):
                    continue
                if name == "updown":
                    updown = float(point.positions[index])
                elif name in joint_map:
                    joint_map[name] = float(point.positions[index])
            stamp = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
            raw_samples.append(
                MotionSample(
                    time_s=max(0.001, stamp),
                    joints=dict(joint_map),
                    updown_m=updown,
                    context={"stage": context_stage, "updown": updown},
                )
            )
        if len(raw_samples) < 2:
            raise RuntimeError(
                "重拍位规划成功但轨迹为空: "
                f"response_points={len(trajectory.points)} message={response.message}"
            )
        samples = retime_segment(
            raw_samples,
            EXECUTION_JOINT_NAMES,
            rate_hz=self.rate_hz,
            max_joint_speed_deg_s=self.max_joint_speed_deg_s,
            max_joint_acceleration_deg_s2=self.max_joint_acceleration_deg_s2,
            max_updown_speed_m_s=self.max_updown_speed_m_s,
            max_updown_acceleration_m_s2=self.max_updown_acceleration_m_s2,
            speed_scale=self.speed_scale,
        )
        return samples, {
            "wall_ms": wall_ms,
            "ik_ms": float(response.ik_time_ms),
            "planning_ms": float(response.planning_time_ms),
            "selected_updown": float(response.selected_updown),
            "message": response.message,
        }

    @staticmethod
    def _cached_task(task: PlanningTask, cache_match) -> PoseTaskSpec:
        targets = cache_match.canonical_targets
        return planning_task_from_suction_surface_poses(
            task.code,
            pose6d_from_dict(targets["left"]["pose_6d"], "cache.left.pose_6d"),
            pose6d_from_dict(targets["right"]["pose_6d"], "cache.right.pose_6d"),
            str(targets["left"]["grasp_mode"]),
            str(targets["right"]["grasp_mode"]),
        )

    @staticmethod
    def _cached_pregrasp_sample(cache_match) -> MotionSample:
        state = cache_match.pregrasp_state
        joints = state.get("joints", {})
        missing = [name for name in EXECUTION_JOINT_NAMES if name not in joints]
        if missing:
            raise ValueError(f"缓存预抓取状态缺少关节: {', '.join(missing)}")
        updown = float(state["updown_m"])
        return MotionSample(
            time_s=0.0,
            joints={name: float(joints[name]) for name in EXECUTION_JOINT_NAMES},
            updown_m=updown,
            context={"stage": "cache/pregrasp", "updown": updown},
        )

    def _resolve_cache_match(
        self,
        task: PlanningTask,
        initial_sample: MotionSample | None,
    ):
        if initial_sample is None:
            match = None
            reason = "initial_sample_missing"
        else:
            match, reason = self.trajectory_cache.find_with_reason(task, initial_sample)
        if match is None and self.trajectory_cache_required:
            raise RuntimeError(
                format_cache_miss_diagnostic(task, reason, self.trajectory_cache.root)
            )
        return match

    @staticmethod
    def _base_link_target(target: dict[str, Any]) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = str(target.get("frame_id", "base_link"))
        position = target["position"]
        orientation = target["orientation"]
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = float(orientation[0])
        message.pose.orientation.y = float(orientation[1])
        message.pose.orientation.z = float(orientation[2])
        message.pose.orientation.w = float(orientation[3])
        return message

    def compute(
        self,
        task: PlanningTask,
        *,
        initial_sample: MotionSample | None = None,
    ) -> ExecutionPlan:
        if (
            self.trajectory_cache.enabled
            and not self.trajectory_cache_required
            and self.trajectory_cache_fallback_on_planning_failure
        ):
            try:
                plan = self._compute_once(
                    task,
                    initial_sample=initial_sample,
                    cache_match=None,
                )
                plan.metrics["cache_fallback_used"] = False
                return plan
            except RuntimeError as online_error:
                cache_match = self._resolve_cache_match(task, initial_sample)
                if cache_match is None:
                    raise RuntimeError(
                        f"在线规划失败且缓存未命中: {online_error}"
                    ) from online_error
                print(
                    "在线规划失败，降级使用轨迹缓存："
                    f"task={task.code} cache={cache_match.path} "
                    f"reason={online_error}",
                    flush=True,
                )
                try:
                    plan = self._compute_once(
                        task,
                        initial_sample=initial_sample,
                        cache_match=cache_match,
                    )
                except RuntimeError as cache_error:
                    raise RuntimeError(
                        "在线规划与缓存降级均失败: "
                        f"online=({online_error}); cache=({cache_error})"
                    ) from cache_error
                plan.metrics["cache_fallback_used"] = True
                plan.metrics["online_planning_failure"] = str(online_error)
                return plan

        cache_match = self._resolve_cache_match(task, initial_sample)
        plan = self._compute_once(
            task,
            initial_sample=initial_sample,
            cache_match=cache_match,
        )
        plan.metrics["cache_fallback_used"] = False
        return plan

    def _compute_once(
        self,
        task: PlanningTask,
        *,
        initial_sample: MotionSample | None,
        cache_match,
    ) -> ExecutionPlan:
        request_root = self.output_root / f"{task.code}_{time.time_ns()}"
        request_root.mkdir(parents=True, exist_ok=False)
        snapshot_path = request_root / "stage_snapshot.json"
        summary_path = request_root / "summary.json"
        cache_started = time.monotonic()
        planning_task = self._cached_task(task, cache_match) if cache_match is not None else task
        target_args = self._target_args(planning_task)
        runtime_config = self._runtime_config(planning_task, target_args)
        left_grasp_mode = planning_task.left_grasp_mode
        right_grasp_mode = planning_task.right_grasp_mode
        explicit_targets = getattr(planning_task, "explicit_targets", None)
        left_target = (
            explicit_targets["left"]
            if explicit_targets is not None
            else self.sequence_helpers.explicit_grasp_target(
                target_args, planning_task.left_box_id, left_grasp_mode
            )
        )
        right_target = (
            explicit_targets["right"]
            if explicit_targets is not None
            else self.sequence_helpers.explicit_grasp_target(
                target_args, planning_task.right_box_id, right_grasp_mode
            )
        )
        left_scene_slot_id = int(
            getattr(planning_task, "left_scene_slot_id", planning_task.left_box_id)
        )
        right_scene_slot_id = int(
            getattr(planning_task, "right_scene_slot_id", planning_task.right_box_id)
        )
        request_record = {
            "request_id": task.code,
            "left_suction_surface_pose_6d": getattr(
                task, "left_suction_surface_pose", None
            ).__dict__ if hasattr(task, "left_suction_surface_pose") else None,
            "right_suction_surface_pose_6d": getattr(
                task, "right_suction_surface_pose", None
            ).__dict__ if hasattr(task, "right_suction_surface_pose") else None,
            "left_front_face_pose_6d": getattr(
                task, "left_front_face_pose", None
            ).__dict__ if hasattr(task, "left_front_face_pose") else None,
            "right_front_face_pose_6d": getattr(
                task, "right_front_face_pose", None
            ).__dict__ if hasattr(task, "right_front_face_pose") else None,
            "derived_rows": [
                int(getattr(task, "left_row", 0)),
                int(getattr(task, "right_row", 0)),
            ],
            "left_grasp_mode": left_grasp_mode,
            "right_grasp_mode": right_grasp_mode,
            "explicit_targets": explicit_targets,
            "runtime_config": runtime_config,
            "trajectory_cache_path": str(cache_match.path) if cache_match is not None else "",
            "trajectory_cache_canonical_targets": (
                cache_match.canonical_targets if cache_match is not None else None
            ),
        }
        (request_root / "planner_request.json").write_text(
            json.dumps(request_record, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        if cache_match is not None:
            self._ensure_session()
            if self._service_client is None:
                raise RuntimeError("planner service client 未初始化")
            configure_ok, configure_output, configure_ms = self._service_client.configure(
                left_scene_slot_id,
                right_scene_slot_id,
                snapshot_path,
                self.timeout_s,
                left_grasp_mode == "top_suction",
                right_grasp_mode == "top_suction",
                left_target,
                right_target,
                runtime_config=runtime_config,
                strategy=getattr(planning_task, "strategy", None),
                start_joint_positions=initial_sample.joints,
                start_updown=initial_sample.updown_m,
            )
            if not configure_ok:
                raise RuntimeError(f"缓存场景配置失败: {configure_output}")
            pregrasp_sample = self._cached_pregrasp_sample(cache_match)
            cache_bridge, bridge_metrics = self.plan_recapture(
                self._base_link_target(left_target),
                self._base_link_target(right_target),
                initial_sample,
                preferred_updown=pregrasp_sample.updown_m,
                exact_target=pregrasp_sample,
                context_stage=(
                    "cache_bridge/selected_pre_attach_loaded_to_pre_contact"
                ),
            )
            snapshot = cache_match.snapshot
            snapshot_path.write_text(
                json.dumps(snapshot, ensure_ascii=False, separators=(",", ":")),
                encoding="utf-8",
            )
            service_ms = 0.0
            planner_wall_ms = (time.monotonic() - cache_started) * 1000.0
            success = True
            service_output = "trajectory cache hit with exact pregrasp bridge"
        else:
            self._ensure_session()
            if self._service_client is None:
                raise RuntimeError("planner service client 未初始化")
            started = time.monotonic()
            configure_ok, configure_output, configure_ms = self._service_client.configure(
                left_scene_slot_id,
                right_scene_slot_id,
                snapshot_path,
                self.timeout_s,
                left_grasp_mode == "top_suction",
                right_grasp_mode == "top_suction",
                left_target,
                right_target,
                runtime_config=runtime_config,
                strategy=getattr(task, "strategy", None),
                start_joint_positions=(initial_sample.joints if initial_sample is not None else None),
                start_updown=(initial_sample.updown_m if initial_sample is not None else None),
            )
            if not configure_ok:
                planner_wall_ms = (time.monotonic() - started) * 1000.0
                summary = {
                    "left_row": getattr(task, "left_row", 0),
                    "right_row": getattr(task, "right_row", 0),
                    "success": False,
                    "startup_ms": 0.0,
                    "planner_session_startup_ms": self.startup_ms,
                    "configure_ms": configure_ms,
                    "service_ms": 0.0,
                    "wall_ms": planner_wall_ms,
                    "snapshot": str(snapshot_path),
                    "failure_reason": configure_output,
                }
                summary_path.write_text(
                    json.dumps([summary], ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
                raise RuntimeError(f"{task.code} planner 配置失败: {configure_output}")

            success, service_output, service_ms = self._service_client.trigger(self.timeout_s)
            planner_wall_ms = (time.monotonic() - started) * 1000.0
            snapshot = (
                json.loads(snapshot_path.read_text(encoding="utf-8"))
                if snapshot_path.is_file()
                else {}
            )
        summary = self._summary_from_snapshot(
            task=planning_task,
            snapshot=snapshot,
            snapshot_path=snapshot_path,
            configure_ms=configure_ms,
            service_ms=service_ms,
            planner_wall_ms=planner_wall_ms,
            success=success,
            failure_reason="" if success else service_output,
        )
        summary_path.write_text(
            json.dumps([summary], ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        if not success:
            raise RuntimeError(f"{task.code} 规划失败: {service_output}")
        if not snapshot:
            raise RuntimeError(f"规划成功但快照不存在: {snapshot_path}")

        raw_samples = self.execution_helpers.trajectory_from_snapshot_preserve_timing(
            snapshot,
            initial=(
                dict(pregrasp_sample.joints)
                if cache_match is not None
                else dict(initial_sample.joints)
                if initial_sample is not None
                else self.execution_helpers.loaded_joint_map(0)
            ),
            initial_updown=(
                pregrasp_sample.updown_m
                if cache_match is not None
                else initial_sample.updown_m
                if initial_sample is not None
                else 0.3
            ),
            hz=self.rate_hz / self.speed_scale,
            max_joint_speed_deg_s=self.max_joint_speed_deg_s,
            max_updown_speed_m_s=self.max_updown_speed_m_s,
        )
        samples = [
            MotionSample(
                time_s=float(time_s),
                joints={name: float(joint_map[name]) for name in EXECUTION_JOINT_NAMES},
                updown_m=float(context.get("updown", 0.3)),
                context=dict(context),
            )
            for time_s, joint_map, context in raw_samples
        ]
        if cache_match is not None:
            samples = [*cache_bridge, *samples]
        stages = split_execution_stages(samples)
        validate_stage_contracts(task, stages, EXECUTION_JOINT_NAMES)
        stages = retime_all_stages(
            stages,
            EXECUTION_JOINT_NAMES,
            rate_hz=self.rate_hz,
            max_joint_speed_deg_s=self.max_joint_speed_deg_s,
            max_joint_acceleration_deg_s2=self.max_joint_acceleration_deg_s2,
            max_updown_speed_m_s=self.max_updown_speed_m_s,
            speed_scale=self.speed_scale,
            max_updown_acceleration_m_s2=self.max_updown_acceleration_m_s2,
        )
        validate_stage_contracts(task, stages, EXECUTION_JOINT_NAMES)
        metrics = {
            **summary,
            "planner_wall_ms": planner_wall_ms,
            "planner_log": str(self.session_log),
            "front_distance_m": float(getattr(task, "front_distance_m", task.effective_distance_m)),
            "top_distance_m": float(getattr(task, "top_distance_m", task.effective_distance_m)),
            "effective_distance_m": task.effective_distance_m,
            "request_id": task.code,
            "trajectory_rate_hz": self.rate_hz,
            "execution_speed_scale": self.speed_scale,
            "effective_max_joint_speed_deg_s": (
                self.max_joint_speed_deg_s * self.speed_scale
            ),
            "max_joint_acceleration_deg_s2": self.max_joint_acceleration_deg_s2,
            "effective_max_updown_speed_m_s": self.max_updown_speed_m_s,
            "trajectory_cache_hit": cache_match is not None,
            "trajectory_cache_path": str(cache_match.path) if cache_match is not None else "",
            "trajectory_cache_distance_m": (
                cache_match.distance_cm / 100.0 if cache_match is not None else 0.0
            ),
            "trajectory_cache_bridge_ms": (
                float(bridge_metrics["wall_ms"]) if cache_match is not None else 0.0
            ),
        }
        return ExecutionPlan(
            task=task,
            snapshot_path=snapshot_path,
            summary_path=summary_path,
            stages=stages,
            metrics=metrics,
        )

    def close(self) -> None:
        if self._service_client is not None:
            self._service_client.close()
            self._service_client = None
        self._recapture_client = None
        process = self._planner_process
        self._planner_process = None
        if process is None:
            return
        if process.poll() is None:
            self.monitor_helpers.terminate_process(process, timeout=10.0)
        if not self.monitor_helpers.wait_until_planner_services_gone(timeout=5.0):
            self.monitor_helpers.cleanup_stale_planner_stack(timeout=10.0)

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    @staticmethod
    def _log_tail(path: Path, line_count: int = 20) -> str:
        if not path.is_file():
            return "planner.log 不存在"
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        return " | ".join(lines[-line_count:])
