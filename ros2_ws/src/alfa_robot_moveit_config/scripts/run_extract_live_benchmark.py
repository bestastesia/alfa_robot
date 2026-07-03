#!/usr/bin/python3
"""Run the extract benchmark repeatedly and open successful results in Rerun.

This is a small operator-facing wrapper around ``dual_arm_planner.launch.py``:

1. start the planner with the known fast extract/RRT benchmark parameters;
2. call ``/dual_arm_planner/configure_extract_monitor`` once to prewarm the
   shared IK solver pool;
3. call ``/dual_arm_planner/run_left_extract_demo`` once;
4. generate an ``.rrd`` from the JSONL if that run succeeded;
5. optionally open Rerun and wait until it exits before the next round.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Iterable

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import process_lifecycle
from extract_stage_monitor_console import call_configure_extract_monitor_service


REPO_ROOT = Path("/mnt/mydisk/ALFA/alfa_robot")
ROS_WS = REPO_ROOT / "ros2_ws"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "data/ik_benchmark/lateral_shift_after_extract/live_extract_benchmark"
VISUALIZER = ROS_WS / "src/alfa_robot_moveit_config/scripts/visualize_moveit_box_stack_flow.py"
SYSTEM_PYTHON = Path("/usr/bin/python3")


def bash_source_command(command: str) -> list[str]:
    return [
        "bash",
        "-lc",
        "source /opt/ros/humble/setup.bash && "
        f"source {ROS_WS}/install/setup.bash && "
        f"cd {ROS_WS} && "
        f"{command}",
    ]


def run_text(command: str, *, timeout: float | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        bash_source_command(command),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=False,
    )


def service_exists(name: str) -> bool:
    try:
        result = run_text("ros2 service list", timeout=5.0)
    except Exception:
        return False
    if result.returncode != 0:
        return False
    return name in result.stdout.splitlines()


def wait_until_planner_services_gone(timeout: float = 15.0) -> bool:
    return process_lifecycle.wait_until_services_gone(service_exists, timeout=timeout)


def cleanup_stale_planner_stack(timeout: float = 15.0) -> bool:
    process_lifecycle.request_stale_planner_shutdown()
    return wait_until_planner_services_gone(timeout)


def wait_for_service(name: str, planner: subprocess.Popen[str], timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if planner.poll() is not None:
            raise RuntimeError(f"planner exited before service became available, code={planner.returncode}")
        result = run_text("ros2 service list", timeout=5.0)
        if name in result.stdout.splitlines():
            return
        time.sleep(0.5)
    raise TimeoutError(f"service {name} not available after {timeout:.1f}s")


def terminate_process(process: subprocess.Popen[str] | None, timeout: float = 5.0) -> None:
    process_lifecycle.terminate_process_tree(process, interrupt_timeout=timeout)


def build_launch_args(args: argparse.Namespace, run_dir: Path) -> list[str]:
    return [
        "ros2 launch alfa_robot_moveit_config dual_arm_planner.launch.py",
        "execute:=false",
        "start_move_group:=true",
        "max_rounds:=10",
        f"box_front_x:={args.box_front_x}",
        f"fixed_updown:={args.fixed_updown}",
        "front_z_reach_lower:=0.2",
        "front_z_reach_upper:=1.3",
        f"extract_demo_pair_sequence:='{args.pair_sequence}'",
        "extract_demo_all_rows:=true",
        "extract_demo_direct_grasp_start:=true",
        "extract_benchmark_all_legal_ik:=true",
        "extract_benchmark_dual_arm:=true",
        "extract_benchmark_dual_async:=true",
        f"extract_benchmark_extract_workers:={args.extract_workers}",
        f"extract_benchmark_candidate_limit:={args.candidate_limit}",
        "extract_ik_dedup_enabled:=true",
        "extract_benchmark_plan_loaded_after_success:=true",
        f"extract_loaded_candidate_limit:={args.loaded_candidate_limit}",
        "extract_loaded_sort_by_pose_distance:=true",
        "extract_loaded_stop_on_first_success:=false",
        "extract_loaded_lateral_shift_enabled:=true",
        f"extract_loaded_lateral_shift_distance:={args.lateral_shift_distance}",
        f"extract_loaded_lateral_shift_step:={args.lateral_shift_step}",
        f"extract_loaded_target_updown:={args.fixed_updown}",
        "extract_loaded_planning_time:=1.0",
        f"extract_loaded_planning_attempts:={args.loaded_planning_attempts}",
        "extract_loaded_use_direct_pipeline:=true",
        f"extract_loaded_parallel_workers:={args.loaded_workers}",
        "extract_use_independent_kdl:=true",
        f"extract_kdl_timeout:={args.extract_kdl_timeout}",
        f"planning_attempts:={args.loaded_planning_attempts}",
        "velocity_scale:=1.0",
        "acceleration_scale:=1.0",
        f"record_trajectories:={'true' if args.record_trajectories else 'false'}",
        f"extract_benchmark_record_rollouts:={'true' if args.record_rollouts else 'false'}",
        f"record_jsonl_path:={run_dir / 'flow.jsonl'}",
        f"extract_benchmark_csv_path:={run_dir / 'pairs.csv'}",
    ]


def parse_summary(jsonl_path: Path) -> dict:
    summary: dict = {}
    if not jsonl_path.exists():
        return summary
    with jsonl_path.open() as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            if row.get("type") == "summary":
                summary = row
    return summary


def service_succeeded(text: str) -> bool:
    return "success=True" in text or "success: true" in text


def first_pair(pair_sequence: str) -> tuple[int, int]:
    token = pair_sequence.split(";", 1)[0].strip()
    if not token:
        raise ValueError("pair sequence is empty")
    parts = [part.strip() for part in token.split(",")]
    if len(parts) != 2:
        raise ValueError(f"invalid first pair: {token}")
    return int(parts[0]), int(parts[1])


def stream_new_log_lines(log_path: Path, offset: int) -> int:
    if not log_path.exists():
        return offset
    with log_path.open(errors="ignore") as file:
        file.seek(offset)
        for line in file:
            if any(token in line for token in (
                "direct IK selected",
                "dual_extract_benchmark",
                "Loaded pose direct planning pipeline ready",
                "Extract primitive IK=",
                "Computed path is not valid",
                "Wrote extract all-legal-IK timing CSV",
            )):
                print(line.rstrip())
        return file.tell()


def generate_rrd(jsonl_path: Path, rrd_path: Path, stride: int) -> None:
    command = (
        f"{SYSTEM_PYTHON} {VISUALIZER} {jsonl_path} "
        f"--save {rrd_path} --stride {stride}"
    )
    result = run_text(command, timeout=300.0)
    if result.returncode != 0:
        raise RuntimeError(result.stdout)
    print(result.stdout.rstrip())


def start_live_rerun(jsonl_path: Path, stride: int, log_path: Path) -> subprocess.Popen[str]:
    command = (
        f"{SYSTEM_PYTHON} {VISUALIZER} {jsonl_path} "
        f"--follow --stride {stride}"
    )
    log_file = log_path.open("w")
    return subprocess.Popen(
        bash_source_command(command),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
    )


def open_rerun(rrd_path: Path) -> int:
    try:
        return subprocess.call(["rerun", str(rrd_path)])
    except FileNotFoundError:
        print(f"未找到 rerun 命令，请手动打开：rerun {rrd_path}")
        return 127


def summarize_csvs(run_dir: Path) -> None:
    csv_files = sorted(run_dir.glob("pairs_L*_R*.csv"))
    if not csv_files:
        return
    print("本轮 CSV：")
    for csv_path in csv_files:
        print(f"  {csv_path.name}")


def run_one_round(args: argparse.Namespace, index: int, output_root: Path) -> bool:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = output_root / f"round_{index:02d}_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    launch_log = run_dir / "launch.log"
    service_log = run_dir / "service_call.log"

    launch_command = " ".join(build_launch_args(args, run_dir))
    domain_export = f"export ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}\n" if "ROS_DOMAIN_ID" in os.environ else ""
    (run_dir / "run_launch.sh").write_text(
        "#!/usr/bin/env bash\nset -e\n"
        f"{domain_export}"
        "source /opt/ros/humble/setup.bash\n"
        f"source {ROS_WS}/install/setup.bash\n"
        f"cd {ROS_WS}\n"
        f"{launch_command}\n"
    )

    print(f"\n========== round {index} ==========")
    print(f"输出目录：{run_dir}")
    if process_lifecycle.any_service_exists(service_exists):
        print("检测到旧 /dual_arm_planner 服务，自动清理旧 planner/move_group...")
        if not cleanup_stale_planner_stack():
            raise RuntimeError("旧 /dual_arm_planner 服务清理超时，请检查外部 ROS 进程")
    with launch_log.open("w") as log_file:
        planner = subprocess.Popen(
            bash_source_command(launch_command),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )

    log_offset = 0
    live_viewer: subprocess.Popen[str] | None = None
    try:
        wait_for_service("/dual_arm_planner/configure_extract_monitor", planner, args.service_timeout)
        prewarm_left, prewarm_right = first_pair(args.pair_sequence)
        prewarm_snapshot = run_dir / "prewarm_stage_snapshot.json"
        prewarm_ok, prewarm_output, prewarm_ms = call_configure_extract_monitor_service(
            "/dual_arm_planner/configure_extract_monitor",
            prewarm_left,
            prewarm_right,
            prewarm_snapshot,
            args.service_timeout,
        )
        print(prewarm_output)
        print(f"planner ready，IK solver 预热完成：success={prewarm_ok} prewarm={prewarm_ms:.1f}ms")
        if not prewarm_ok:
            raise RuntimeError(f"IK solver 预热失败：{prewarm_output}")
        log_offset = stream_new_log_lines(launch_log, log_offset)
        print("开始计算 run_left_extract_demo...")
        if args.live_rerun:
            live_viewer = start_live_rerun(run_dir / "flow.jsonl", args.visual_stride, run_dir / "rerun_live.log")
            print("Rerun 实时可视化已启动；计算出的 stage 会立刻推送。")

        service = subprocess.Popen(
            bash_source_command("ros2 service call /dual_arm_planner/run_left_extract_demo std_srvs/srv/Trigger {}"),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        while service.poll() is None:
            log_offset = stream_new_log_lines(launch_log, log_offset)
            time.sleep(0.5)
        service_text, _ = service.communicate(timeout=5.0)
        service_log.write_text(service_text + "\n")
        print(service_text)
        log_offset = stream_new_log_lines(launch_log, log_offset)

        success = service_succeeded(service_text)
        summary = parse_summary(run_dir / "flow.jsonl")
        if summary:
            print("summary:")
            print(json.dumps(summary, ensure_ascii=False, indent=2))
        summarize_csvs(run_dir)

        if success and args.generate_rerun:
            rrd_path = run_dir / "successful_round.rrd"
            print("计算成功，生成 Rerun...")
            generate_rrd(run_dir / "flow.jsonl", rrd_path, args.visual_stride)
            if args.open_rerun:
                print("打开 Rerun；关闭窗口后进入下一轮。")
                open_rerun(rrd_path)
        elif success:
            print("本轮成功；已按参数跳过 Rerun 生成。")
        else:
            print("本轮失败，不打开 Rerun。")
        return success
    finally:
        if live_viewer and live_viewer.poll() is None:
            live_viewer.terminate()
            try:
                live_viewer.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                live_viewer.kill()
        terminate_process(planner)
        if not wait_until_planner_services_gone():
            print("planner 服务仍未消失，追加清理旧 planner/move_group...")
            cleanup_stale_planner_stack()


def main() -> None:
    parser = argparse.ArgumentParser(description="Run live extract benchmark rounds with per-round Rerun playback.")
    parser.add_argument("--rounds", type=int, default=1, help="Number of rounds to run. Use 0 for infinite.")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--pair-sequence", default="2,3;7,4;8,9;12,13;17,14;18,19")
    parser.add_argument("--box-front-x", type=float, default=0.925)
    parser.add_argument("--fixed-updown", type=float, default=0.3)
    parser.add_argument("--candidate-limit", type=int, default=64)
    parser.add_argument("--extract-workers", type=int, default=16)
    parser.add_argument("--extract-kdl-timeout", type=float, default=0.003)
    parser.add_argument("--loaded-candidate-limit", type=int, default=8)
    parser.add_argument("--loaded-workers", type=int, default=8)
    parser.add_argument("--loaded-planning-attempts", type=int, default=1)
    parser.add_argument("--lateral-shift-distance", type=float, default=0.5)
    parser.add_argument("--lateral-shift-step", type=float, default=0.03)
    parser.add_argument("--record-trajectories", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--record-rollouts", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--live-rerun", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--generate-rerun", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--open-rerun", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--visual-stride", type=int, default=1)
    parser.add_argument("--service-timeout", type=float, default=60.0)
    parser.add_argument(
        "--ros-domain-id",
        default="auto",
        help="本次 ROS_DOMAIN_ID；auto 隔离自启动测试，inherit 表示沿用当前终端。",
    )
    args = parser.parse_args()
    domain = process_lifecycle.configure_ros_domain(args.ros_domain_id)

    if not VISUALIZER.exists():
        raise SystemExit(f"visualizer not found: {VISUALIZER}")

    args.output_root.mkdir(parents=True, exist_ok=True)
    print("live extract benchmark")
    print(f"output_root={args.output_root}")
    print(f"ROS_DOMAIN_ID={domain if domain is not None else 'unset'}")
    print("提示：默认每轮启动 planner 和实时 Rerun；stage 算出来就立刻播放，整轮结束后进入下一轮。")

    index = 1
    while args.rounds == 0 or index <= args.rounds:
        run_one_round(args, index, args.output_root)
        index += 1


if __name__ == "__main__":
    main()
