#!/usr/bin/python3
from __future__ import annotations

import argparse
import os
import signal
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
        front_z_reach_lower=args.front_z_reach_lower,
        front_z_reach_upper=args.front_z_reach_upper,
        left_box_id=left_id,
        right_box_id=right_id,
        extract_workers=args.extract_workers,
        candidate_limit=args.candidate_limit,
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


def wait_until_service_gone(timeout: float = 15.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not (
            monitor.service_exists("/dual_arm_planner/run_extract_monitor_next")
            or monitor.service_exists("/dual_arm_planner/run_extract_monitor_full_selected")
        ):
            return
        time.sleep(0.1)


def cleanup_planner_processes() -> None:
    subprocess.run(
        ["pkill", "-INT", "-f", "dual_arm_planner|move_group|ros2 launch alfa_robot_moveit_config dual_arm_planner"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    wait_until_service_gone(15.0)


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
    scene_y_shift = float(snapshot.get("scene_y_shift", args.scene_y_shift))
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
            helpers.log_robot_state(robot, joints, "monitor/robot")
            monitor.log_attached_boxes(robot, joints, stage.get("attached_boxes", []))
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
    left_id: int,
    right_id: int,
    task_index: int,
    pair_count: int,
    sample_start: int,
) -> tuple[bool, int, dict[str, Any]]:
    pair_args = make_pair_args(args, left_id, right_id)
    run_dir = run_root / f"{task_index:02d}_L{left_id}_R{right_id}"
    run_dir.mkdir(parents=True, exist_ok=True)
    snapshot_path = run_dir / "stage_snapshot.json"
    launch_log = run_dir / "planner.log"
    ros_home = run_dir / "ros_home"
    ros_log_dir = run_dir / "ros_log"
    ros_home.mkdir(parents=True, exist_ok=True)
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_HOME"] = str(ros_home)
    os.environ["ROS_LOG_DIR"] = str(ros_log_dir)

    if monitor.service_exists("/dual_arm_planner/run_extract_monitor_full_selected"):
        raise RuntimeError("检测到旧 /dual_arm_planner 服务，请先清理旧 ROS 进程")

    launch_command = monitor.build_launch_command(pair_args, run_dir, snapshot_path)
    (run_dir / "launch_command.sh").write_text(
        "#!/usr/bin/env bash\nset -e\n"
        "source /opt/ros/humble/setup.bash\n"
        f"source {monitor.ROS_WS}/install/setup.bash\n"
        f"cd {monitor.ROS_WS}\n{launch_command}\n"
    )
    print(f"\n===== 任务 {task_index}/{pair_count}: L{left_id}/R{right_id} =====")
    print(f"启动 planner，日志：{launch_log}")
    with launch_log.open("w") as log_file:
        planner = subprocess.Popen(
            monitor.bash_source_command(launch_command),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )

    try:
        service_name = "/dual_arm_planner/run_extract_monitor_full_selected"
        monitor.wait_for_service(service_name, planner, args.service_timeout, launch_log)
        print("计算开始：IK → 抽离 → 横向让位 → 负重规划")
        start = time.monotonic()
        success, output, elapsed_ms = monitor.call_trigger_service(service_name, args.service_timeout)
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
                "samples": sample_count,
            }
        )
        return True, sample_count, summary
    finally:
        monitor.terminate_process(planner)
        wait_until_service_gone()
        if monitor.service_exists("/dual_arm_planner/run_extract_monitor_full_selected"):
            cleanup_planner_processes()


def main() -> int:
    parser = argparse.ArgumentParser(description="生成 6 次抽箱任务连续全流程 Rerun")
    parser.add_argument("--pair-sequence", default=DEFAULT_SEQUENCE)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--save", type=Path, default=None)
    parser.add_argument("--box-front-x", type=float, default=0.925)
    parser.add_argument("--scene-y-shift", type=float, default=-0.4)
    parser.add_argument("--fixed-updown", type=float, default=0.3)
    parser.add_argument("--front-z-reach-lower", type=float, default=0.45)
    parser.add_argument("--front-z-reach-upper", type=float, default=1.25)
    parser.add_argument("--candidate-limit", type=int, default=64)
    parser.add_argument("--extract-workers", type=int, default=16)
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
    args = parser.parse_args()

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

    sample = 0
    summaries: list[dict[str, Any]] = []
    for index, (left_id, right_id) in enumerate(pairs, start=1):
        ok, sample_count, summary = run_one_pair(
            args, helpers, robot, run_root, left_id, right_id, index, len(pairs), sample
        )
        summaries.append(summary)
        sample += max(1, sample_count) + 5
        if not ok and not args.continue_on_failure:
            break

    summary_path = run_root / "summary.json"
    import json

    summary_path.write_text(json.dumps(summaries, ensure_ascii=False, indent=2))
    print("\n===== 序列完成 =====")
    for item in summaries:
        status = "成功" if item.get("success") else "失败"
        print(
            f"L{item['left']}/R{item['right']}: {status} "
            f"total={item.get('total_ms', 0.0):.1f}ms samples={item.get('samples', 0)}"
        )
    print(f"Rerun: {save_path}")
    print(f"Summary: {summary_path}")
    return 0 if all(item.get("success") for item in summaries) else 1


if __name__ == "__main__":
    raise SystemExit(main())
