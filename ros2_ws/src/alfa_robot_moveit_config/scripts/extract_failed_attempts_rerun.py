#!/usr/bin/python3
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import extract_stage_monitor_console as monitor
import process_lifecycle


DEFAULT_OUTPUT_ROOT = Path("/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/extract_failed_attempts_rerun")


def pair_args(args: argparse.Namespace, run_dir: Path, snapshot_path: Path) -> SimpleNamespace:
    return SimpleNamespace(
        box_front_x=args.box_front_x,
        scene_y_shift=args.scene_y_shift,
        fixed_updown=args.fixed_updown,
        turn_rad=0.0,
        grasp_mode=args.grasp_mode,
        left_grasp_mode=args.left_grasp_mode,
        right_grasp_mode=args.right_grasp_mode,
        front_z_reach_lower=args.front_z_reach_lower,
        front_z_reach_upper=args.front_z_reach_upper,
        top_z_reach_lower=args.top_z_reach_lower,
        top_z_reach_upper=args.top_z_reach_upper,
        top_suction_x_offset=args.top_suction_x_offset,
        top_suction_z_offset=args.top_suction_z_offset,
        ik_top_position_tolerance=args.ik_top_position_tolerance,
        ik_top_orientation_tolerance_deg=args.ik_top_orientation_tolerance_deg,
        ik_h_candidate_count=args.ik_h_candidate_count,
        ik_h_lower=args.ik_h_lower,
        ik_h_upper=args.ik_h_upper,
        ik_h_step=args.ik_h_step,
        ik_full_h_range_scan=args.ik_full_h_range_scan,
        ik_seed_count=args.ik_seed_count,
        ik_workers=args.ik_workers,
        ik_candidate_timeout=args.ik_candidate_timeout,
        ik_try_target_orders=args.ik_try_target_orders,
        ik_use_reversed_target_order=args.ik_use_reversed_target_order,
        optimized_ik_check_collision=args.optimized_ik_check_collision,
        left_box_id=args.left_box_id,
        right_box_id=args.right_box_id,
        extract_workers=args.extract_workers,
        candidate_limit=args.candidate_limit,
        extract_step_x=args.extract_step_x,
        extract_rollout_mode=args.extract_rollout_mode,
        extract_box_pose_rrt_edge_scene_collision=args.extract_box_pose_rrt_edge_scene_collision,
        extract_rrt=args.extract_rrt,
        extract_rrt_planning_group=args.extract_rrt_planning_group,
        extract_rrt_planning_time=args.extract_rrt_planning_time,
        extract_rrt_planning_attempts=args.extract_rrt_planning_attempts,
        extract_rrt_endpoint_per_arm_limit=args.extract_rrt_endpoint_per_arm_limit,
        extract_rrt_goal_limit=args.extract_rrt_goal_limit,
        dedup_joint_threshold_deg=args.dedup_joint_threshold_deg,
        dedup_h_threshold=args.dedup_h_threshold,
        loaded_candidate_limit=args.loaded_candidate_limit,
        loaded_updown=args.loaded_updown,
        loaded_planner_id=args.loaded_planner_id,
        loaded_planning_mode=args.loaded_planning_mode,
        lateral_shift_distance=args.lateral_shift_distance,
        lateral_shift_step=args.lateral_shift_step,
        lateral_shift_column=args.lateral_shift_column,
        lateral_shift_enabled=args.lateral_shift_enabled,
        pre_lower_left_box_id=args.pre_lower_left_box_id,
        pre_lower_right_box_id=args.pre_lower_right_box_id,
        pre_lower_updown_delta=args.pre_lower_updown_delta,
        loaded_planning_time=args.loaded_planning_time,
        loaded_planning_attempts=args.loaded_planning_attempts,
        loaded_workers=args.loaded_workers,
        output_root=args.output_root,
        save=args.save,
        no_rerun=False,
        no_start_planner=False,
        connect=False,
        mode="staged",
        service_timeout=args.service_timeout,
        max_display=args.max_display,
        grid_cols=8,
        spacing=2.4,
        stride=args.stride,
        render_mode="mesh",
        box_stack_y_shift=None,
        once=False,
        run_dir=run_dir,
        snapshot_path=snapshot_path,
    )


def log_stage_record_replay(
    snapshot: dict[str, Any],
    record: dict[str, Any],
    helpers: Any,
    robot: Any,
    args: argparse.Namespace,
    sample: int,
    section_label: str,
    record_index: int,
    record_count: int,
) -> int:
    replay_stages = list(record.get("replay_stages", []))
    container_panels = snapshot.get("container_panels")
    candidate_order = record.get("candidate_order", "?")
    loaded_rank = record.get("loaded_plan_rank", 0)
    loaded_ok = bool(record.get("loaded_plan_success", False))
    failure = str(record.get("loaded_plan_failure_reason") or record.get("failure_reason") or "")

    if not replay_stages:
      helpers.set_sample_time(sample)
      monitor.log_container_panels(container_panels)
      joint_map = monitor.record_joint_map(record)
      if joint_map:
          helpers.log_robot_state(robot, joint_map, "monitor/robot")
      monitor.rr.log(
          "monitor/info",
          monitor.rr.TextLog(
              f"{section_label} {record_index + 1}/{record_count} | "
              f"cand={candidate_order} rank={loaded_rank} success={loaded_ok} | 无回放轨迹 | {failure}"
          ),
      )
      return 1

    used = 0
    for stage_index, stage in enumerate(replay_stages):
        points = monitor.ensure_points_start_at_stage_start(
            stage, list(stage.get("trajectory", {}).get("points", []))
        )
        if not points:
            continue
        selected_indices = list(range(0, len(points), max(1, args.stride)))
        if selected_indices[-1] != len(points) - 1:
            selected_indices.append(len(points) - 1)
        for point_index in selected_indices:
            helpers.set_sample_time(sample + used)
            monitor.log_container_panels(container_panels)
            monitor.log_static_box_obstacles(stage.get("static_box_obstacles"))
            point = points[point_index]
            joints = monitor.joint_dict_from_stage_point(stage, point)
            helpers.log_robot_state(robot, joints, "monitor/robot")
            monitor.log_attached_boxes(robot, joints, stage.get("attached_boxes", []))
            monitor.rr.log(
                "monitor/info",
                monitor.rr.TextLog(
                    f"{section_label} {record_index + 1}/{record_count} | "
                    f"cand={candidate_order} rank={loaded_rank} success={loaded_ok} | "
                    f"stage {stage_index + 1}/{len(replay_stages)} {stage.get('stage')} | "
                    f"point {point_index + 1}/{len(points)} | {failure}"
                ),
            )
            used += 1
    return max(1, used)


def log_snapshot_records(
    snapshot: dict[str, Any],
    helpers: Any,
    robot: Any,
    args: argparse.Namespace,
    sample: int,
    section_label: str,
) -> int:
    records = list(snapshot.get("records", []))[: args.max_display]
    monitor.rr.log(
        "monitor/title",
        monitor.rr.TextLog(
            f"L{args.left_box_id}/R{args.right_box_id} | {section_label} | "
            f"显示 {len(records)}/{len(snapshot.get('records', []))} | "
            f"阶段耗时 {float(snapshot.get('elapsed_ms', 0.0)):.1f}ms"
        ),
    )
    used_total = 0
    for index, record in enumerate(records):
        used = log_stage_record_replay(
            snapshot,
            record,
            helpers,
            robot,
            args,
            sample + used_total,
            section_label,
            index,
            len(records),
        )
        used_total += used + args.gap_samples
    return max(1, used_total)


def main() -> int:
    parser = argparse.ArgumentParser(description="保存失败任务的全部抽离/让位候选 Rerun")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--left-box-id", type=int, required=True)
    parser.add_argument("--right-box-id", type=int, required=True)
    parser.add_argument("--save", type=Path, default=None)
    parser.add_argument("--box-front-x", type=float, default=0.925)
    parser.add_argument("--scene-y-shift", type=float, default=0.0)
    parser.add_argument("--fixed-updown", type=float, default=0.3)
    parser.add_argument("--grasp-mode", choices=["front", "top_suction"], default="front")
    parser.add_argument("--left-grasp-mode", choices=["front", "top_suction"], default=None)
    parser.add_argument("--right-grasp-mode", choices=["front", "top_suction"], default=None)
    parser.add_argument("--front-z-reach-lower", type=float, default=0.45)
    parser.add_argument("--front-z-reach-upper", type=float, default=1.25)
    parser.add_argument("--top-z-reach-lower", type=float, default=0.0)
    parser.add_argument("--top-z-reach-upper", type=float, default=0.45)
    parser.add_argument("--top-suction-x-offset", type=float, default=0.15)
    parser.add_argument("--top-suction-z-offset", type=float, default=0.25)
    parser.add_argument("--ik-top-position-tolerance", type=float, default=0.04)
    parser.add_argument("--ik-top-orientation-tolerance-deg", type=float, default=7.0)
    parser.add_argument("--ik-h-candidate-count", type=int, default=64)
    parser.add_argument("--ik-h-lower", type=float, default=0.0)
    parser.add_argument("--ik-h-upper", type=float, default=0.7)
    parser.add_argument("--ik-h-step", type=float, default=0.01)
    parser.add_argument("--ik-full-h-range-scan", action="store_true")
    parser.add_argument("--ik-seed-count", type=int, default=32)
    parser.add_argument("--ik-workers", type=int, default=1)
    parser.add_argument("--ik-candidate-timeout", type=float, default=0.01)
    parser.add_argument("--ik-try-target-orders", action="store_true")
    parser.add_argument("--ik-use-reversed-target-order", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--optimized-ik-check-collision", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--candidate-limit", type=int, default=64)
    parser.add_argument("--extract-workers", type=int, default=16)
    parser.add_argument("--extract-step-x", type=float, default=0.03)
    parser.add_argument(
        "--extract-rollout-mode",
        choices=["greedy", "box_pose_rrt", "moveit_rrt_legacy", "top_lift_legacy"],
        default="box_pose_rrt",
    )
    parser.add_argument("--extract-box-pose-rrt-edge-scene-collision", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--extract-rrt", action="store_true")
    parser.add_argument("--extract-rrt-planning-group", default="dual_arm")
    parser.add_argument("--extract-rrt-planning-time", type=float, default=0.35)
    parser.add_argument("--extract-rrt-planning-attempts", type=int, default=1)
    parser.add_argument("--extract-rrt-endpoint-per-arm-limit", type=int, default=8)
    parser.add_argument("--extract-rrt-goal-limit", type=int, default=8)
    parser.add_argument("--loaded-candidate-limit", type=int, default=8)
    parser.add_argument("--loaded-workers", type=int, default=8)
    parser.add_argument("--loaded-updown", type=float, default=0.3)
    parser.add_argument("--loaded-planner-id", default="")
    parser.add_argument("--loaded-planning-mode", choices=["rrt", "shortcut"], default="rrt")
    parser.add_argument("--loaded-planning-time", type=float, default=1.0)
    parser.add_argument("--loaded-planning-attempts", type=int, default=8)
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
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--gap-samples", type=int, default=5)
    parser.add_argument("--max-display", type=int, default=128)
    parser.add_argument(
        "--ros-domain-id",
        default="auto",
        help="本次 ROS_DOMAIN_ID；auto 隔离自启动失败回放，inherit 表示沿用当前终端。",
    )
    args = parser.parse_args()
    if args.left_grasp_mode is None:
        args.left_grasp_mode = args.grasp_mode
    if args.right_grasp_mode is None:
        args.right_grasp_mode = args.grasp_mode
    domain = process_lifecycle.configure_ros_domain(args.ros_domain_id)
    print(f"ROS_DOMAIN_ID={domain if domain is not None else 'unset'}")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = args.output_root / f"L{args.left_box_id}_R{args.right_box_id}_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    snapshot_path = run_dir / "stage_snapshot.json"
    save_path = args.save or (run_dir / f"L{args.left_box_id}_R{args.right_box_id}_extract_shift_attempts.rrd")
    save_path.parent.mkdir(parents=True, exist_ok=True)

    ros_home = run_dir / "ros_home"
    ros_log_dir = run_dir / "ros_log"
    ros_home.mkdir(parents=True, exist_ok=True)
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_HOME"] = str(ros_home)
    os.environ["ROS_LOG_DIR"] = str(ros_log_dir)

    if monitor.planner_monitor_service_exists():
        print("检测到旧 /dual_arm_planner 服务，自动清理旧 planner/move_group...")
        if not monitor.cleanup_stale_planner_stack(15.0):
            raise RuntimeError("旧 /dual_arm_planner 服务清理超时，请检查外部 ROS 进程")

    launch_args = pair_args(args, run_dir, snapshot_path)
    launch_command = monitor.build_launch_command(launch_args, run_dir, snapshot_path)
    domain_export = f"export ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}\n" if "ROS_DOMAIN_ID" in os.environ else ""
    (run_dir / "launch_command.sh").write_text(
        "#!/usr/bin/env bash\nset -e\n"
        f"{domain_export}"
        "source /opt/ros/humble/setup.bash\n"
        f"source {monitor.ROS_WS}/install/setup.bash\n"
        f"cd {monitor.ROS_WS}\n{launch_command}\n"
    )
    launch_log = run_dir / "planner.log"
    print(f"启动 planner：L{args.left_box_id}/R{args.right_box_id}")
    print(f"日志：{launch_log}")

    with launch_log.open("w") as log_file:
        planner = subprocess.Popen(
            monitor.bash_source_command(launch_command),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )

    try:
        monitor.wait_for_service("/dual_arm_planner/run_extract_monitor_next", planner, args.service_timeout, launch_log)
        prewarm_ok, prewarm_output, prewarm_ms = monitor.call_configure_extract_monitor_service(
            "/dual_arm_planner/configure_extract_monitor",
            args.left_box_id,
            args.right_box_id,
            snapshot_path,
            args.service_timeout,
            args.left_grasp_mode == "top_suction",
            args.right_grasp_mode == "top_suction",
        )
        print(prewarm_output)
        if not prewarm_ok:
            raise RuntimeError(f"IK solver 预热失败：{prewarm_output}")
        print(f"planner 启动完成，IK solver 已预热：{prewarm_ms:.1f}ms")

        global rr
        import rerun as rr

        monitor.rr = rr
        helpers = monitor.load_rerun_helpers()
        rr.init("extract_failed_attempts_rerun")
        rr.save(str(save_path))
        robot = helpers.UrdfRobot(helpers.render_current_urdf())
        rr.log("monitor", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        helpers.log_robot_static_model(robot, "monitor/robot", log_meshes=True)

        snapshots: list[dict[str, Any]] = []
        labels = ["IK候选", "所有抽离成功结果", "所有让位/负重尝试结果"]
        print("开始计算：IK -> 抽离 -> 让位/负重尝试")
        total_start = time.monotonic()
        for index, label in enumerate(labels, start=1):
            print(f"[{index}/3] {label} 开始")
            stage_start = time.monotonic()
            success, output, elapsed_ms = monitor.call_trigger_service(
                "/dual_arm_planner/run_extract_monitor_next",
                args.service_timeout,
            )
            print(output)
            print(f"[{index}/3] {label} 完成 success={success} service={elapsed_ms:.1f}ms wall={(time.monotonic() - stage_start) * 1000.0:.1f}ms")
            if not snapshot_path.exists():
                raise RuntimeError(f"snapshot 未生成：{snapshot_path}")
            snapshot = monitor.read_snapshot(snapshot_path)
            snapshots.append(snapshot)
            if not success:
                if index == 2 and snapshot.get("records"):
                    print(f"{label} 失败，但已生成 {len(snapshot.get('records', []))} 条失败诊断记录，停止后续阶段并保存回放。")
                    break
                if index < 3:
                    raise RuntimeError(f"{label} 失败，且没有可回放诊断记录")
                break

        sample = 0
        if len(snapshots) >= 2:
            extract_label = "抽离成功候选"
            if int(snapshots[1].get("success_count", 0)) == 0:
                extract_label = "抽离失败停留状态"
            sample += log_snapshot_records(snapshots[1], helpers, robot, args, sample, extract_label)
        if len(snapshots) >= 3:
            sample += log_snapshot_records(snapshots[2], helpers, robot, args, sample + args.gap_samples, "让位/负重尝试候选")

        summary = {
            "left_box_id": args.left_box_id,
            "right_box_id": args.right_box_id,
            "save": str(save_path),
            "run_dir": str(run_dir),
            "total_wall_ms": (time.monotonic() - total_start) * 1000.0,
            "snapshots": snapshots,
        }
        summary_path = run_dir / "summary.json"
        summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2))
        print(f"Rerun: {save_path}")
        print(f"Summary: {summary_path}")
        return 0
    finally:
        monitor.terminate_process(planner)
        if not monitor.wait_until_planner_services_gone(15.0):
            monitor.cleanup_stale_planner_stack(15.0)


if __name__ == "__main__":
    raise SystemExit(main())
