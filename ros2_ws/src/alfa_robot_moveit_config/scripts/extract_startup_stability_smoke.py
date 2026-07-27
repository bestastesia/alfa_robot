#!/usr/bin/python3
"""Smoke test for extract planner startup/restart stability.

The test intentionally starts and stops the full planner stack multiple times
in the same ROS domain.  It fails if a run connects to a stale planner, leaves
planner services behind, or produces the old joint-name pollution that used to
make MoveIt logs explode.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import extract_stage_monitor_console as monitor  # noqa: E402
import process_lifecycle  # noqa: E402


DEFAULT_OUTPUT_ROOT = monitor.REPO_ROOT / "data/ik_benchmark/startup_stability_smoke"


def parse_total_ms(output: str) -> float | None:
    match = re.search(r"total=([0-9.]+)ms", output)
    return float(match.group(1)) if match else None


def count_pattern(path: Path, pattern: str) -> int:
    if not path.exists():
        return 0
    result = subprocess.run(
        ["rg", "-c", "-F", pattern, str(path)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode != 0:
        return 0
    text = result.stdout.strip()
    try:
        return int(text)
    except ValueError:
        return 0


def format_optional_ms(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.1f}ms"


def run_once(args: argparse.Namespace, run_index: int, run_root: Path) -> dict:
    run_dir = run_root / f"round_{run_index:02d}"
    run_dir.mkdir(parents=True, exist_ok=True)
    snapshot_path = run_dir / "stage_snapshot.json"
    launch_log = run_dir / "planner.log"

    os.environ["ROS_HOME"] = str(run_dir / "ros_home")
    os.environ["ROS_LOG_DIR"] = str(run_dir / "ros_log")
    Path(os.environ["ROS_HOME"]).mkdir(parents=True, exist_ok=True)
    Path(os.environ["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)

    if monitor.planner_monitor_service_exists():
        if not monitor.cleanup_stale_planner_stack(args.cleanup_timeout):
            raise RuntimeError("旧 /dual_arm_planner 服务清理超时")

    pair_args = argparse.Namespace(
        box_front_x=args.box_front_x,
        scene_y_shift=args.scene_y_shift,
        fixed_updown=args.fixed_updown,
        loaded_updown=args.loaded_updown,
        grasp_mode=args.grasp_mode,
        front_z_reach_lower=args.front_z_reach_lower,
        front_z_reach_upper=args.front_z_reach_upper,
        top_z_reach_lower=args.top_z_reach_lower,
        top_z_reach_upper=args.top_z_reach_upper,
        top_suction_x_offset=args.top_suction_x_offset,
        top_suction_z_offset=args.top_suction_z_offset,
        ik_top_position_tolerance=args.ik_top_position_tolerance,
        ik_top_orientation_tolerance_deg=args.ik_top_orientation_tolerance_deg,
        ik_h_candidate_count=args.ik_h_candidate_count,
        ik_seed_count=args.ik_seed_count,
        ik_candidate_timeout=args.ik_candidate_timeout,
        ik_try_target_orders=args.ik_try_target_orders,
        ik_use_reversed_target_order=args.ik_use_reversed_target_order,
        optimized_ik_check_collision=args.optimized_ik_check_collision,
        left_box_id=args.left_box_id,
        right_box_id=args.right_box_id,
        extract_workers=args.extract_workers,
        candidate_limit=args.candidate_limit,
        extract_step_x=args.extract_step_x,
        dedup_joint_threshold_deg=args.dedup_joint_threshold_deg,
        dedup_h_threshold=args.dedup_h_threshold,
        loaded_candidate_limit=args.loaded_candidate_limit,
        lateral_shift_enabled=args.lateral_shift_enabled,
        lateral_shift_distance=args.lateral_shift_distance,
        lateral_shift_step=args.lateral_shift_step,
        lateral_shift_column=args.lateral_shift_column,
        pre_lower_left_box_id=args.pre_lower_left_box_id,
        pre_lower_right_box_id=args.pre_lower_right_box_id,
        pre_lower_updown_delta=args.pre_lower_updown_delta,
        loaded_planning_time=args.loaded_planning_time,
        loaded_planning_attempts=args.loaded_planning_attempts,
        loaded_workers=args.loaded_workers,
        loaded_preferred_pose_index=args.loaded_preferred_pose_index,
    )
    launch_command = monitor.build_launch_command(pair_args, run_dir, snapshot_path)
    domain_export = f"export ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}\n" if "ROS_DOMAIN_ID" in os.environ else ""
    (run_dir / "launch_command.sh").write_text(
        "#!/usr/bin/env bash\nset -e\n"
        f"{domain_export}"
        "source /opt/ros/humble/setup.bash\n"
        f"source {monitor.ROS_WS}/install/setup.bash\n"
        f"cd {monitor.ROS_WS}\n{launch_command}\n"
    )

    print(f"\n===== smoke round {run_index}/{args.rounds} =====", flush=True)
    print(f"run_dir={run_dir}", flush=True)
    started_at = time.monotonic()
    with launch_log.open("w") as log_file:
        planner = subprocess.Popen(
            monitor.bash_source_command(launch_command),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )

    service_name = "/dual_arm_planner/run_extract_monitor_full_selected"
    try:
        monitor.wait_for_service("/dual_arm_planner/configure_extract_monitor", planner, args.service_timeout, launch_log)
        prewarm_ok, prewarm_output, prewarm_ms = monitor.call_configure_extract_monitor_service(
            "/dual_arm_planner/configure_extract_monitor",
            args.left_box_id,
            args.right_box_id,
            snapshot_path,
            args.service_timeout,
        )
        print(prewarm_output, flush=True)
        if not prewarm_ok:
            raise RuntimeError(f"IK solver 预热失败：{prewarm_output}")
        ready_ms = (time.monotonic() - started_at) * 1000.0
        print(f"service ready + IK prewarm: {ready_ms:.1f} ms (prewarm call {prewarm_ms:.1f} ms)", flush=True)
        success, output, service_ms = monitor.call_trigger_service(service_name, args.service_timeout)
        total_ms = parse_total_ms(output)
        print(output, flush=True)
        print(f"service wall: {service_ms:.1f} ms", flush=True)

        returned_snapshot = monitor.extract_snapshot_path_from_service_output(output)
        if returned_snapshot is not None and returned_snapshot != snapshot_path:
            raise RuntimeError(f"连到了旧 planner：expected={snapshot_path}, got={returned_snapshot}")
        if args.require_flow_success and not success:
            raise RuntimeError(f"smoke service failed: {output}")
        if success and not snapshot_path.exists():
            raise RuntimeError(f"snapshot missing: {snapshot_path}")

        left_old = count_pattern(launch_log, "Joint 'left_joint")
        right_old = count_pattern(launch_log, "Joint 'right_joint")
        ready_count = count_pattern(launch_log, "DualArmPlannerNode ready")
        selected_count = count_pattern(launch_log, "direct IK selected")
        if left_old or right_old:
            raise RuntimeError(f"旧 joint 名称污染仍存在：left={left_old}, right={right_old}")
        if ready_count != 1:
            raise RuntimeError(f"planner ready 次数异常：{ready_count}")
        if selected_count < 1:
            raise RuntimeError("未观察到 direct IK selected 日志")

        return {
            "round": run_index,
            "run_dir": str(run_dir),
            "ready_ms": ready_ms,
            "service_ms": service_ms,
            "total_ms": total_ms,
            "flow_success": success,
            "snapshot": str(snapshot_path) if snapshot_path.exists() else "",
            "old_joint_name_count": left_old + right_old,
            "planner_ready_count": ready_count,
            "direct_ik_selected_count": selected_count,
        }
    finally:
        monitor.terminate_process(planner)
        if not monitor.wait_until_planner_services_gone(args.cleanup_timeout):
            monitor.cleanup_stale_planner_stack(args.cleanup_timeout)
        if monitor.planner_monitor_service_exists():
            raise RuntimeError("本轮结束后仍残留 /dual_arm_planner 服务")


def main() -> int:
    parser = argparse.ArgumentParser(description="连续复启抽箱 planner，验证启动/关闭稳定性")
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--ros-domain-id", default="auto")
    parser.add_argument("--left-box-id", type=int, default=6)
    parser.add_argument("--right-box-id", type=int, default=8)
    parser.add_argument("--box-front-x", type=float, default=0.925)
    parser.add_argument("--scene-y-shift", type=float, default=0.0)
    parser.add_argument("--fixed-updown", type=float, default=0.3)
    parser.add_argument("--loaded-updown", type=float, default=0.3)
    parser.add_argument("--grasp-mode", choices=["front", "top_suction"], default="front")
    parser.add_argument("--front-z-reach-lower", type=float, default=0.45)
    parser.add_argument("--front-z-reach-upper", type=float, default=1.25)
    parser.add_argument("--top-z-reach-lower", type=float, default=0.3)
    parser.add_argument("--top-z-reach-upper", type=float, default=0.45)
    parser.add_argument("--top-suction-x-offset", type=float, default=0.15)
    parser.add_argument("--top-suction-z-offset", type=float, default=0.25)
    parser.add_argument("--ik-top-position-tolerance", type=float, default=0.04)
    parser.add_argument("--ik-top-orientation-tolerance-deg", type=float, default=7.0)
    parser.add_argument("--ik-h-candidate-count", type=int, default=64)
    parser.add_argument("--ik-seed-count", type=int, default=32)
    parser.add_argument("--ik-candidate-timeout", type=float, default=0.01)
    parser.add_argument("--ik-try-target-orders", action="store_true")
    parser.add_argument("--ik-use-reversed-target-order", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--optimized-ik-check-collision", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--candidate-limit", type=int, default=64)
    parser.add_argument("--extract-workers", type=int, default=16)
    parser.add_argument("--extract-step-x", type=float, default=0.03)
    parser.add_argument("--loaded-candidate-limit", type=int, default=8)
    parser.add_argument("--loaded-workers", type=int, default=8)
    parser.add_argument("--loaded-planning-time", type=float, default=1.0)
    parser.add_argument("--loaded-planning-attempts", type=int, default=8)
    parser.add_argument("--loaded-preferred-pose-index", type=int, default=0)
    parser.add_argument("--lateral-shift-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--lateral-shift-distance", type=float, default=0.5)
    parser.add_argument("--lateral-shift-step", type=float, default=0.01)
    parser.add_argument("--lateral-shift-column", type=int, default=2)
    parser.add_argument("--pre-lower-left-box-id", type=int, default=0)
    parser.add_argument("--pre-lower-right-box-id", type=int, default=0)
    parser.add_argument("--pre-lower-updown-delta", type=float, default=0.0)
    parser.add_argument("--dedup-joint-threshold-deg", type=float, default=1.0)
    parser.add_argument("--dedup-h-threshold", type=float, default=0.005)
    parser.add_argument("--service-timeout", type=float, default=120.0)
    parser.add_argument("--cleanup-timeout", type=float, default=15.0)
    parser.add_argument(
        "--require-flow-success",
        action="store_true",
        help="默认只验证启动/隔离/清理稳定性；打开后完整流程失败也会让 smoke 失败。",
    )
    args = parser.parse_args()

    if args.rounds < 1:
        raise SystemExit("--rounds must be >= 1")

    domain = process_lifecycle.configure_ros_domain(args.ros_domain_id)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_root = args.output_root / f"restart_smoke_{stamp}"
    run_root.mkdir(parents=True, exist_ok=True)

    print("extract startup stability smoke", flush=True)
    print(f"ROS_DOMAIN_ID={domain if domain is not None else 'unset'}", flush=True)
    print(f"output={run_root}", flush=True)

    summaries = [run_once(args, index, run_root) for index in range(1, args.rounds + 1)]
    summary_path = run_root / "summary.json"
    summary_path.write_text(json.dumps(summaries, ensure_ascii=False, indent=2))

    print("\n===== smoke summary =====", flush=True)
    for item in summaries:
        print(
            f"round {item['round']}: ready={item['ready_ms']:.1f}ms "
            f"service={item['service_ms']:.1f}ms total={format_optional_ms(item['total_ms'])} "
            f"flow_success={item['flow_success']} old_joint_names={item['old_joint_name_count']}",
            flush=True,
        )
    print(f"summary={summary_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
