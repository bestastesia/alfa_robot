#!/usr/bin/python3
from __future__ import annotations

import argparse
import math
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

import numpy as np


DEFAULT_OUTPUT_ROOT = Path("/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/extract_sequence_rerun")
DEFAULT_SEQUENCE = "1,2;6,3;7,8;11,12;16,13;17,18"


def parse_pair_sequence(value: str) -> list[tuple[int, int]]:
    pairs: list[tuple[int, int]] = []
    for segment in value.split(";"):
        segment = segment.strip()
        if not segment:
            continue
        sep = "," if "," in segment else "/"
        parts = [part.strip() for part in segment.split(sep)]
        if len(parts) != 2:
            raise ValueError(f"invalid pair segment: {segment}")
        pairs.append((int(parts[0]), int(parts[1])))
    if not pairs:
        raise ValueError("empty pair sequence")
    return pairs


def make_pair_args(args: argparse.Namespace, left_id: int, right_id: int) -> SimpleNamespace:
    return SimpleNamespace(
        box_front_x=args.box_front_x,
        scene_y_shift=args.scene_y_shift,
        fixed_updown=args.fixed_updown,
        turn_rad=math.radians(args.turn_deg),
        grasp_mode=args.grasp_mode,
        front_z_reach_lower=args.front_z_reach_lower,
        front_z_reach_upper=args.front_z_reach_upper,
        top_z_reach_lower=args.top_z_reach_lower,
        top_z_reach_upper=args.top_z_reach_upper,
        ik_h_candidate_count=args.ik_h_candidate_count,
        ik_seed_count=args.ik_seed_count,
        ik_candidate_timeout=args.ik_candidate_timeout,
        ik_try_target_orders=args.ik_try_target_orders,
        ik_use_reversed_target_order=args.ik_use_reversed_target_order,
        left_box_id=left_id,
        right_box_id=right_id,
        extract_workers=args.extract_workers,
        candidate_limit=args.candidate_limit,
        extract_step_x=args.extract_step_x,
        dedup_joint_threshold_deg=args.dedup_joint_threshold_deg,
        dedup_h_threshold=args.dedup_h_threshold,
        loaded_candidate_limit=args.loaded_candidate_limit,
        lateral_shift_distance=args.lateral_shift_distance,
        lateral_shift_step=args.lateral_shift_step,
        lateral_shift_column=args.lateral_shift_column,
        pre_lower_left_box_id=args.pre_lower_left_box_id,
        pre_lower_right_box_id=args.pre_lower_right_box_id,
        pre_lower_updown_delta=args.pre_lower_updown_delta,
        loaded_planning_time=args.loaded_planning_time,
        loaded_planning_attempts=args.loaded_planning_attempts,
        loaded_workers=args.loaded_workers,
        extract_kdl_timeout=args.extract_kdl_timeout,
    )


def wait_until_service_gone(timeout: float = 15.0) -> bool:
    return monitor.wait_until_planner_services_gone(timeout)


def cleanup_planner_processes() -> bool:
    process_lifecycle.request_stale_planner_shutdown()
    return wait_until_service_gone(15.0)


def display_base_transform(args: argparse.Namespace) -> np.ndarray:
    yaw = math.radians(float(getattr(args, "display_base_yaw_deg", 0.0)))
    c = math.cos(yaw)
    s = math.sin(yaw)
    transform = np.eye(4)
    transform[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    transform[:3, 3] = [float(getattr(args, "display_base_x", 0.0)), float(getattr(args, "display_base_y", 0.0)), 0.0]
    return transform


def display_scene_y_shift(args: argparse.Namespace) -> float:
    return float(args.scene_y_shift) + float(getattr(args, "display_base_y", 0.0))


def log_robot_state_display(helpers: Any, robot: Any, joints: dict[str, float], path: str, args: argparse.Namespace) -> None:
    base_tf = display_base_transform(args)
    display_joints = dict(joints)
    display_joints["turn"] = float(display_joints.get("turn", 0.0)) + math.radians(float(getattr(args, "display_turn_offset_deg", 0.0)))
    transforms = robot.fk(display_joints)
    for link_name, transform in transforms.items():
        helpers.log_transform_matrix(f"{path}/{link_name}", base_tf @ transform)


def log_attached_boxes_display(robot: Any, joints: dict[str, float], attached_boxes: list[dict[str, Any]], args: argparse.Namespace) -> None:
    if not attached_boxes:
        monitor.rr.log("monitor/scene/attached_boxes", monitor.rr.Clear(recursive=True))
        return
    display_joints = dict(joints)
    display_joints["turn"] = float(display_joints.get("turn", 0.0)) + math.radians(float(getattr(args, "display_turn_offset_deg", 0.0)))
    fk = robot.fk(display_joints)
    base_tf = display_base_transform(args)
    centers = []
    half_sizes = []
    quaternions = []
    colors = []
    labels = []
    for box in attached_boxes:
        link_name = str(box.get("link_name", ""))
        link_tf = fk.get(link_name)
        center_in_link = box.get("center_in_link", [])
        size = box.get("size", [])
        if link_tf is None or len(center_in_link) != 3 or len(size) != 3:
            continue
        world_link_tf = base_tf @ link_tf
        center = world_link_tf @ np.array([float(center_in_link[0]), float(center_in_link[1]), float(center_in_link[2]), 1.0])
        centers.append(center[:3].tolist())
        half_sizes.append([float(value) * 0.5 for value in size])
        quaternions.append(monitor.matrix_to_quaternion(world_link_tf[:3, :3]))
        colors.append([40, 220, 90, 150])
        labels.append(str(box.get("id", "carried_box")))
    monitor.rr.log(
        "monitor/scene/attached_boxes",
        monitor.rr.Boxes3D(centers=centers, half_sizes=half_sizes, quaternions=quaternions, colors=colors, labels=labels),
    )


def log_sequence_replay(
    snapshot: dict[str, Any],
    helpers: Any,
    robot: Any,
    args: argparse.Namespace,
    task_index: int,
    pair_count: int,
    sample_start: int,
) -> int:
    replay_stages = list(snapshot.get("replay_stages", []))
    scene_y_shift = display_scene_y_shift(args)
    left_id = int(snapshot.get("left_box_id", 0))
    right_id = int(snapshot.get("right_box_id", 0))
    sample = sample_start

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
            helpers.set_sample_time(sample)
            monitor.log_default_container(scene_y_shift)
            monitor.log_box_stack(float(args.box_front_x), left_id, right_id, scene_y_shift)
            monitor.log_static_box_obstacles(stage.get("static_box_obstacles"))
            point = points[point_index]
            joints = monitor.joint_dict_from_stage_point(stage, point)
            log_robot_state_display(helpers, robot, joints, "monitor/robot", args)
            log_attached_boxes_display(robot, joints, stage.get("attached_boxes", []), args)
            monitor.rr.log(
                "monitor/info",
                monitor.rr.TextLog(
                    f"任务 {task_index}/{pair_count}: L{left_id}/R{right_id} | "
                    f"stage {stage_index + 1}/{len(replay_stages)}: {stage.get('stage')} | "
                    f"point {point_index + 1}/{len(points)}"
                ),
            )
            sample += 1
    return sample - sample_start


def run_one_pair(
    args: argparse.Namespace,
    helpers: Any,
    robot: Any,
    run_root: Path,
    planner: subprocess.Popen[str],
    launch_log: Path,
    service_client: monitor.ExtractMonitorServiceClient,
    startup_ms: float,
    left_id: int,
    right_id: int,
    task_index: int,
    pair_count: int,
    sample_start: int,
) -> tuple[bool, int, dict[str, Any]]:
    run_dir = run_root / f"{task_index:02d}_L{left_id}_R{right_id}"
    run_dir.mkdir(parents=True, exist_ok=True)
    snapshot_path = run_dir / "stage_snapshot.json"
    print(f"\n===== 任务 {task_index}/{pair_count}: L{left_id}/R{right_id} =====")
    try:
        config_ok, config_output, config_ms = service_client.configure(
            left_id,
            right_id,
            snapshot_path,
            args.service_timeout,
        )
        print(config_output)
        print(f"任务配置完成：success={config_ok} configure={config_ms:.1f}ms snapshot={snapshot_path}")
        if not config_ok:
            summary = {
                "left": left_id,
                "right": right_id,
                "success": False,
                "startup_ms": startup_ms if task_index == 1 else 0.0,
                "configure_ms": config_ms,
                "service_ms": 0.0,
                "wall_ms": 0.0,
                "snapshot": str(snapshot_path),
                "failure_reason": config_output,
            }
            return False, 1, summary

        print("计算开始：IK → 抽离 → 横向让位 → 负重规划")
        start = time.monotonic()
        success, output, elapsed_ms = service_client.trigger(args.service_timeout)
        wall_ms = (time.monotonic() - start) * 1000.0
        print(output)
        print(f"计算结束：success={success} service={elapsed_ms:.1f}ms wall={wall_ms:.1f}ms")
        returned_snapshot = monitor.extract_snapshot_path_from_service_output(output)
        if returned_snapshot is not None and returned_snapshot != snapshot_path:
            raise RuntimeError(f"服务连到了旧 planner：expected={snapshot_path}, got={returned_snapshot}")
        summary: dict[str, Any] = {
            "left": left_id,
            "right": right_id,
            "success": success,
            "startup_ms": startup_ms if task_index == 1 else 0.0,
            "configure_ms": config_ms,
            "service_ms": elapsed_ms,
            "wall_ms": wall_ms,
            "snapshot": str(snapshot_path),
        }
        if not success:
            helpers.set_sample_time(sample_start)
            monitor.rr.log(
                "monitor/info",
                monitor.rr.TextLog(f"任务 {task_index}/{pair_count}: L{left_id}/R{right_id} 失败\n{output}"),
            )
            return False, 1, summary
        snapshot = monitor.read_snapshot(snapshot_path)
        sample_count = log_sequence_replay(snapshot, helpers, robot, args, task_index, pair_count, sample_start)
        summary.update(
            {
                "total_ms": float(snapshot.get("elapsed_ms", 0.0)),
                "ik_ms": float(snapshot.get("ik_elapsed_ms", 0.0)),
                "extract_ms": float(snapshot.get("extract_elapsed_ms", 0.0)),
                "loaded_ms": float(snapshot.get("loaded_elapsed_ms", 0.0)),
                "loaded_plan_batch_wall_ms": float(snapshot.get("loaded_plan_batch_wall_ms", 0.0)),
                "loaded_plan_candidate_count": int(snapshot.get("loaded_plan_candidate_count", 0)),
                "loaded_plan_attempted_count": int(snapshot.get("loaded_plan_attempted_count", 0)),
                "loaded_plan_success_count": int(snapshot.get("loaded_plan_success_count", 0)),
                "loaded_parallel_workers": int(snapshot.get("loaded_parallel_workers", 0)),
                "final_ms": float(snapshot.get("final_elapsed_ms", 0.0)),
                "samples": sample_count,
            }
        )
        return True, sample_count, summary
    finally:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description="生成 6 次抽箱任务连续全流程 Rerun")
    parser.add_argument("--pair-sequence", default=DEFAULT_SEQUENCE)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--save", type=Path, default=None)
    parser.add_argument("--box-front-x", type=float, default=0.925)
    parser.add_argument("--scene-y-shift", type=float, default=-0.4)
    parser.add_argument("--fixed-updown", type=float, default=0.3)
    parser.add_argument("--turn-deg", type=float, default=0.0)
    parser.add_argument("--display-base-yaw-deg", type=float, default=0.0, help="仅用于 Rerun 回放显示底盘外部 yaw；规划仍使用当前 MoveIt base_link")
    parser.add_argument("--display-base-x", type=float, default=0.0, help="仅用于 Rerun 回放显示底盘外部 x 平移")
    parser.add_argument("--display-base-y", type=float, default=0.0, help="仅用于 Rerun 回放显示底盘外部 y 平移；若 scene_y_shift=-base_y，则显示为世界固定箱墙")
    parser.add_argument("--display-turn-offset-deg", type=float, default=0.0, help="仅用于 Rerun 回放显示外部底盘 yaw 后的 turn 反向补偿")
    parser.add_argument("--grasp-mode", choices=["front", "top_suction"], default="front")
    parser.add_argument("--front-z-reach-lower", type=float, default=0.45)
    parser.add_argument("--front-z-reach-upper", type=float, default=1.25)
    parser.add_argument("--top-z-reach-lower", type=float, default=0.3)
    parser.add_argument("--top-z-reach-upper", type=float, default=0.45)
    parser.add_argument("--ik-h-candidate-count", type=int, default=16)
    parser.add_argument("--ik-seed-count", type=int, default=32)
    parser.add_argument("--ik-candidate-timeout", type=float, default=0.01)
    parser.add_argument("--ik-try-target-orders", action="store_true")
    parser.add_argument("--ik-use-reversed-target-order", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--candidate-limit", type=int, default=64)
    parser.add_argument("--extract-workers", type=int, default=16)
    parser.add_argument("--extract-step-x", type=float, default=0.03)
    parser.add_argument("--loaded-candidate-limit", type=int, default=8)
    parser.add_argument("--loaded-workers", type=int, default=8)
    parser.add_argument("--loaded-planning-time", type=float, default=1.0)
    parser.add_argument("--loaded-planning-attempts", type=int, default=8)
    parser.add_argument("--lateral-shift-distance", type=float, default=0.5)
    parser.add_argument("--lateral-shift-step", type=float, default=0.01)
    parser.add_argument("--lateral-shift-column", type=int, default=2)
    parser.add_argument("--pre-lower-left-box-id", type=int, default=0)
    parser.add_argument("--pre-lower-right-box-id", type=int, default=0)
    parser.add_argument("--pre-lower-updown-delta", type=float, default=0.0)
    parser.add_argument("--extract-kdl-timeout", type=float, default=0.003)
    parser.add_argument("--dedup-joint-threshold-deg", type=float, default=1.0)
    parser.add_argument("--dedup-h-threshold", type=float, default=0.005)
    parser.add_argument("--service-timeout", type=float, default=120.0)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--continue-on-failure", action="store_true")
    parser.add_argument(
        "--ros-domain-id",
        default="auto",
        help="本次 ROS_DOMAIN_ID；auto 隔离自启动序列测试，inherit 表示沿用当前终端。",
    )
    args = parser.parse_args()
    domain = process_lifecycle.configure_ros_domain(args.ros_domain_id)
    print(f"ROS_DOMAIN_ID={domain if domain is not None else 'unset'}")

    pairs = parse_pair_sequence(args.pair_sequence)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_root = args.output_root / f"sequence_{stamp}"
    run_root.mkdir(parents=True, exist_ok=True)
    save_path = args.save or (run_root / "extract_sequence_full.rrd")
    save_path.parent.mkdir(parents=True, exist_ok=True)

    global rr
    import rerun as rr

    monitor.rr = rr
    helpers = monitor.load_rerun_helpers()
    rr.init("extract_sequence_rerun")
    rr.save(str(save_path))
    urdf_text = helpers.render_current_urdf()
    robot = helpers.UrdfRobot(urdf_text)
    rr.log("monitor", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    helpers.log_robot_static_model(robot, "monitor/robot", log_meshes=True)

    if monitor.service_exists("/dual_arm_planner/run_extract_monitor_full_selected"):
        print("检测到旧 /dual_arm_planner 服务，自动清理旧 planner/move_group...")
        if not cleanup_planner_processes():
            raise RuntimeError("旧 /dual_arm_planner 服务清理超时，请检查外部 ROS 进程")

    ros_home = run_root / "ros_home"
    ros_log_dir = run_root / "ros_log"
    ros_home.mkdir(parents=True, exist_ok=True)
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_HOME"] = str(ros_home)
    os.environ["ROS_LOG_DIR"] = str(ros_log_dir)

    first_left, first_right = pairs[0]
    initial_args = make_pair_args(args, first_left, first_right)
    launch_log = run_root / "planner.log"
    initial_snapshot = run_root / "initial_stage_snapshot.json"
    launch_command = monitor.build_launch_command(initial_args, run_root, initial_snapshot)
    domain_export = f"export ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}\n" if "ROS_DOMAIN_ID" in os.environ else ""
    (run_root / "launch_command.sh").write_text(
        "#!/usr/bin/env bash\nset -e\n"
        f"{domain_export}"
        "source /opt/ros/humble/setup.bash\n"
        f"source {monitor.ROS_WS}/install/setup.bash\n"
        f"cd {monitor.ROS_WS}\n{launch_command}\n"
    )
    print(f"启动共享 planner，日志：{launch_log}")
    startup_start = time.monotonic()
    with launch_log.open("w") as log_file:
        planner = subprocess.Popen(
            monitor.bash_source_command(launch_command),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )

    sample = 0
    summaries: list[dict[str, Any]] = []
    service_client: monitor.ExtractMonitorServiceClient | None = None
    try:
        monitor.wait_for_service(
            "/dual_arm_planner/configure_extract_monitor",
            planner,
            args.service_timeout,
            launch_log,
        )
        service_client = monitor.ExtractMonitorServiceClient(
            configure_service="/dual_arm_planner/configure_extract_monitor",
            trigger_service="/dual_arm_planner/run_extract_monitor_full_selected",
            timeout=args.service_timeout,
        )
        prewarm_ok, prewarm_output, prewarm_ms = service_client.configure(
            first_left,
            first_right,
            initial_snapshot,
            args.service_timeout,
        )
        print(prewarm_output)
        if not prewarm_ok:
            raise RuntimeError(f"共享 planner IK 预热失败：{prewarm_output}")
        startup_ms = (time.monotonic() - startup_start) * 1000.0
        print(f"共享 planner 启动完成：startup={startup_ms:.1f}ms prewarm={prewarm_ms:.1f}ms")
        for index, (left_id, right_id) in enumerate(pairs, start=1):
            ok, sample_count, summary = run_one_pair(
                args,
                helpers,
                robot,
                run_root,
                planner,
                launch_log,
                service_client,
                startup_ms,
                left_id,
                right_id,
                index,
                len(pairs),
                sample,
            )
            summaries.append(summary)
            sample += max(1, sample_count) + 5
            if not ok and not args.continue_on_failure:
                break
    finally:
        if service_client is not None:
            service_client.close()
        monitor.terminate_process(planner)
        if not wait_until_service_gone():
            cleanup_planner_processes()

    summary_path = run_root / "summary.json"
    import json

    summary_path.write_text(json.dumps(summaries, ensure_ascii=False, indent=2))
    print("\n===== 序列完成 =====")
    for item in summaries:
        status = "成功" if item.get("success") else "失败"
        print(
            f"L{item['left']}/R{item['right']}: {status} "
            f"startup={item.get('startup_ms', 0.0):.1f}ms "
            f"total={item.get('total_ms', 0.0):.1f}ms "
            f"loaded_batch={item.get('loaded_plan_batch_wall_ms', 0.0):.1f}ms "
            f"samples={item.get('samples', 0)}"
        )
    print(f"Rerun: {save_path}")
    print(f"Summary: {summary_path}")
    return 0 if all(item.get("success") for item in summaries) else 1


if __name__ == "__main__":
    raise SystemExit(main())
