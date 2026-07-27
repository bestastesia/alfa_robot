from __future__ import annotations

import importlib.util
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from alfa_robot_execution_bridge.joints import EXECUTION_JOINT_NAMES

from .common import (
    ExecutionPlan,
    MotionSample,
    TaskSpec,
    retime_all_stages,
    split_seven_stages,
    validate_stage_contracts,
)


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
        speed_scale: float,
        timeout_s: float,
    ) -> None:
        self.source_ws = source_ws.resolve()
        self.output_root = output_root.resolve()
        self.rate_hz = float(rate_hz)
        self.max_joint_speed_deg_s = float(max_joint_speed_deg_s)
        self.max_joint_acceleration_deg_s2 = float(max_joint_acceleration_deg_s2)
        self.max_updown_speed_m_s = float(max_updown_speed_m_s)
        self.speed_scale = float(speed_scale)
        if self.speed_scale <= 0.0:
            raise ValueError("speed_scale 必须为正数")
        self.timeout_s = float(timeout_s)
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
        self.startup_ms = 0.0
        self._start_session()

    def _start_session(self) -> None:
        command = [
            sys.executable,
            str(self.planner_script),
            "--planner-server",
            "--pair-sequence",
            "1,3",
            "--task-layout",
            "centered",
            "--box-front-x",
            "0.9",
            "--top-box-front-x",
            "0.7",
            "--fixed-updown",
            "0.3",
            "--loaded-updown",
            "0.45",
            "--loaded-preserve-lower-updown",
            "--place-updown",
            "0.1",
            "--place-transition-updown",
            "0.1",
            "--output-root",
            str(self.session_root),
            "--no-rerun",
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
        self.startup_ms = (time.monotonic() - started) * 1000.0
        print(
            f"planner 长驻会话已就绪：startup={self.startup_ms:.1f}ms "
            f"log={self.session_log}",
            flush=True,
        )

    def _target_args(self, task: TaskSpec) -> SimpleNamespace:
        return SimpleNamespace(
            box_front_x=task.effective_distance_m,
            scene_y_shift=float(
                self.sequence_helpers.TASK_LAYOUT_Y_OFFSETS[task.task_layout]
            ),
            world_to_base_z=0.202094,
            top_suction_x_offset=0.15,
            top_suction_z_offset=0.2,
        )

    def _runtime_config(self, task: TaskSpec, target_args: SimpleNamespace) -> dict[str, Any]:
        return {
            "box_front_x": task.effective_distance_m,
            "scene_y_shift": target_args.scene_y_shift,
            "extract_rollout_mode": task.extraction_mode,
            "loaded_lateral_shift_enabled": task.grasp_family == "front",
            "extract_box_pose_rrt_max_iterations": 400 if task.index == 5 else 160,
        }

    def _summary_from_snapshot(
        self,
        *,
        task: TaskSpec,
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
            "left": task.left_box_id,
            "right": task.right_box_id,
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

    def compute(self, task: TaskSpec) -> ExecutionPlan:
        if self._planner_process is None or self._planner_process.poll() is not None:
            raise RuntimeError(
                f"planner 长驻进程已退出; {self._log_tail(self.session_log)}"
            )
        if self._service_client is None:
            raise RuntimeError("planner service client 未初始化")

        request_root = self.output_root / f"{task.code}_{int(time.time() * 1000)}"
        request_root.mkdir(parents=True, exist_ok=False)
        snapshot_path = request_root / "stage_snapshot.json"
        summary_path = request_root / "summary.json"
        target_args = self._target_args(task)
        runtime_config = self._runtime_config(task, target_args)
        grasp_mode = task.grasp_family
        request_record = {
            "task_code": task.code,
            "left_box_id": task.left_box_id,
            "right_box_id": task.right_box_id,
            "grasp_mode": grasp_mode,
            "runtime_config": runtime_config,
        }
        (request_root / "planner_request.json").write_text(
            json.dumps(request_record, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        started = time.monotonic()
        configure_ok, configure_output, configure_ms = self._service_client.configure(
            task.left_box_id,
            task.right_box_id,
            snapshot_path,
            self.timeout_s,
            grasp_mode == "top_suction",
            grasp_mode == "top_suction",
            self.sequence_helpers.explicit_grasp_target(
                target_args, task.left_box_id, grasp_mode
            ),
            self.sequence_helpers.explicit_grasp_target(
                target_args, task.right_box_id, grasp_mode
            ),
            runtime_config=runtime_config,
        )
        if not configure_ok:
            planner_wall_ms = (time.monotonic() - started) * 1000.0
            summary = {
                "left": task.left_box_id,
                "right": task.right_box_id,
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
            task=task,
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
            initial=self.execution_helpers.loaded_joint_map(0),
            initial_updown=0.3,
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
        stages = split_seven_stages(samples)
        validate_stage_contracts(task, stages, EXECUTION_JOINT_NAMES)
        stages = retime_all_stages(
            stages,
            EXECUTION_JOINT_NAMES,
            rate_hz=self.rate_hz,
            max_joint_speed_deg_s=self.max_joint_speed_deg_s,
            max_joint_acceleration_deg_s2=self.max_joint_acceleration_deg_s2,
            max_updown_speed_m_s=self.max_updown_speed_m_s,
            speed_scale=self.speed_scale,
        )
        validate_stage_contracts(task, stages, EXECUTION_JOINT_NAMES)
        metrics = {
            **summary,
            "planner_wall_ms": planner_wall_ms,
            "planner_log": str(self.session_log),
            "front_distance_m": task.front_distance_m,
            "top_distance_m": task.top_distance_m,
            "effective_distance_m": task.effective_distance_m,
            "task_code": task.code,
            "trajectory_rate_hz": self.rate_hz,
            "execution_speed_scale": self.speed_scale,
            "effective_max_joint_speed_deg_s": (
                self.max_joint_speed_deg_s * self.speed_scale
            ),
            "max_joint_acceleration_deg_s2": self.max_joint_acceleration_deg_s2,
            "effective_max_updown_speed_m_s": (
                self.max_updown_speed_m_s * self.speed_scale
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
        process = self._planner_process
        self._planner_process = None
        if process is None or process.poll() is not None:
            return
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)

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
