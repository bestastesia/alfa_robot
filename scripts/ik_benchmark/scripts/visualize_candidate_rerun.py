#!/usr/bin/env python3
"""Page through IK candidate robot poses in Rerun, sorted by cost/rank.

Example:
  python3 visualize_candidate_rerun.py result.jsonl --stage round_1/safe --page-size 10
  python3 visualize_candidate_rerun.py result.jsonl --stage-index 0 --offset 10 --page-size 10
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

rr: Any = None
UrdfRobot: Any = None
log_pose: Any = None
log_robot_state: Any = None
log_robot_static_model: Any = None
render_current_urdf: Any = None
set_sample_time: Any = None


def load_rerun_helpers() -> None:
    global rr, UrdfRobot, log_pose, log_robot_state, log_robot_static_model, render_current_urdf, set_sample_time
    try:
        import rerun as rerun_module
    except ModuleNotFoundError as exc:
        raise SystemExit("当前 Python 环境没有 rerun 包；请切到安装 rerun-sdk 的环境，或先执行 `python3 -m pip install rerun-sdk`。") from exc
    from alfa_robot_rerun.visualize_rerun import (
        UrdfRobot as UrdfRobotClass,
        log_pose as log_pose_fn,
        log_robot_state as log_robot_state_fn,
        log_robot_static_model as log_robot_static_model_fn,
        render_current_urdf as render_current_urdf_fn,
        set_sample_time as set_sample_time_fn,
    )
    rr = rerun_module
    UrdfRobot = UrdfRobotClass
    log_pose = log_pose_fn
    log_robot_state = log_robot_state_fn
    log_robot_static_model = log_robot_static_model_fn
    render_current_urdf = render_current_urdf_fn
    set_sample_time = set_sample_time_fn


def load_stages(path: Path) -> list[dict[str, Any]]:
    stages: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if rec.get("type") == "stage":
                stages.append(rec)
    if not stages:
        raise SystemExit(f"No stage records found: {path}")
    return stages


def stage_sort_key(stage: str) -> tuple[int, int, str]:
    order = {"safe": 0, "approach": 1, "grasp": 2, "retreat": 3, "place_safe": 4}
    parts = stage.split("/")
    round_no = 0
    if parts and parts[0].startswith("round_"):
        try:
            round_no = int(parts[0].split("_", 1)[1])
        except ValueError:
            pass
    return round_no, order.get(parts[-1], 99), stage


def candidate_sort_key(candidate: dict[str, Any]) -> tuple[int, float, float, int]:
    legal_sort = 0 if candidate.get("legal") else 1
    rank = candidate.get("rank")
    score = candidate.get("score")
    index = candidate.get("candidate_index", 10**9)
    try:
        rank_value = float(rank)
    except (TypeError, ValueError):
        rank_value = math.inf
    try:
        score_value = float(score)
    except (TypeError, ValueError):
        score_value = math.inf
    return legal_sort, rank_value, score_value, int(index) if isinstance(index, int) else 10**9


def select_stage(stages: list[dict[str, Any]], stage: str | None, stage_index: int | None) -> dict[str, Any]:
    ordered = sorted(stages, key=lambda rec: stage_sort_key(str(rec.get("stage", ""))))
    if stage:
        for rec in ordered:
            if rec.get("stage") == stage:
                return rec
        names = "\n".join(str(rec.get("stage")) for rec in ordered)
        raise SystemExit(f"Stage not found: {stage}\nAvailable stages:\n{names}")
    if stage_index is None:
        print("Available stages:")
        for i, rec in enumerate(ordered):
            print(f"  {i}: {rec.get('stage')} candidates={len(rec.get('candidates', []) or [])}")
        raise SystemExit("Please pass --stage or --stage-index")
    if stage_index < 0 or stage_index >= len(ordered):
        raise SystemExit(f"stage-index out of range: {stage_index}")
    return ordered[stage_index]


def joint_positions(candidate: dict[str, Any]) -> dict[str, float] | None:
    names = candidate.get("full_joint_names") or candidate.get("joint_names") or []
    values = candidate.get("full_joint_values") or candidate.get("joint_values") or []
    if not names or len(names) != len(values):
        return None
    return {str(name): float(value) for name, value in zip(names, values)}


def pose_from_record(record: dict[str, Any], key: str) -> tuple[np.ndarray, np.ndarray] | None:
    pose = record.get(key)
    if not isinstance(pose, dict):
        return None
    position = pose.get("position")
    orientation = pose.get("orientation")
    if not isinstance(position, list) or not isinstance(orientation, list):
        return None
    return np.array(position, dtype=float), np.array(orientation, dtype=float)


def log_stage_targets(stage_record: dict[str, Any]) -> None:
    colors = {"target_pose": [255, 165, 0], "target_pose2": [255, 120, 0]}
    for key, label in [("target_pose", "left target"), ("target_pose2", "right target")]:
        pose = pose_from_record(stage_record, key)
        if pose is None:
            continue
        pos, quat = pose
        log_pose(f"targets/{key}", pos, quat, label, colors[key])


def log_candidate_text(path: str, stage: str, candidate: dict[str, Any], rank_offset: int) -> None:
    cost = candidate.get("cost_breakdown") if isinstance(candidate.get("cost_breakdown"), dict) else {}
    lines = [
        f"stage: {stage}",
        f"page_rank: {rank_offset}",
        f"candidate_index: {candidate.get('candidate_index')}",
        f"rank: {candidate.get('rank')}",
        f"selected: {candidate.get('selected')}",
        f"legal: {candidate.get('legal')}",
        f"reason: {candidate.get('rejection_reason', '')}",
        f"path/order: {candidate.get('solver_path')} / {candidate.get('target_order')}",
        f"h: {candidate.get('h')}  h_index: {candidate.get('h_index')}  seed_index: {candidate.get('seed_index')}",
        f"score: {candidate.get('score')}  cost_total: {cost.get('total')}",
        f"j2 lever L/R/max: {candidate.get('left_joint2_lever_length')} / {candidate.get('right_joint2_lever_length')} / {candidate.get('joint2_lever_length')}",
        f"j3 lever L/R/max: {candidate.get('left_joint3_lever_length')} / {candidate.get('right_joint3_lever_length')} / {candidate.get('joint3_lever_length')}",
    ]
    rr.log(path, rr.TextLog("\n".join(lines)))


def main() -> None:
    parser = argparse.ArgumentParser(description="分页可视化某个 stage 的 IK candidate 姿态")
    parser.add_argument("jsonl", type=Path, help="updown_solver_comparison JSONL")
    parser.add_argument("--stage", default="", help="流程节点名，例如 round_1/safe")
    parser.add_argument("--stage-index", type=int, default=None, help="按排序后的 stage index 选择")
    parser.add_argument("--offset", type=int, default=0, help="从排序后第几个 candidate 开始")
    parser.add_argument("--page-size", type=int, default=10, help="每次显示多少个 candidate")
    parser.add_argument("--only-legal", action="store_true", help="只显示合法候选")
    parser.add_argument("--include-illegal", action="store_true", help="保留参数兼容；默认已显示所有候选")
    parser.add_argument("--connect", action="store_true", help="连接已有 Rerun viewer")
    parser.add_argument("--interactive", action="store_true", help="按 Enter 继续加载下一页候选，每页 page-size 个")
    parser.add_argument("--save", type=Path, default=None, help="保存为 .rrd")
    parser.add_argument("--robot-path", default="robot", help="Rerun robot 根路径")
    parser.add_argument("--no-meshes", action="store_true", help="只显示 link 坐标，不加载 mesh")
    args = parser.parse_args()

    load_rerun_helpers()
    stages = load_stages(args.jsonl)
    stage_record = select_stage(stages, args.stage or None, args.stage_index)
    candidates = [c for c in (stage_record.get("candidates", []) or []) if isinstance(c, dict)]
    if args.only_legal:
        candidates = [c for c in candidates if c.get("legal")]
    candidates = sorted(candidates, key=candidate_sort_key)
    page = candidates[args.offset : args.offset + args.page_size]
    if not page:
        raise SystemExit(f"No candidates in requested page: offset={args.offset}, page_size={args.page_size}, total={len(candidates)}")

    recording_id = f"ik_candidates_{Path(args.jsonl).stem}_{stage_record.get('stage', 'stage').replace('/', '_')}"
    if args.save:
        rr.init("ik_candidate_page", recording_id=recording_id)
        rr.save(str(args.save))
    elif args.connect:
        rr.init("ik_candidate_page", recording_id=recording_id)
        rr.connect()
    else:
        rr.init("ik_candidate_page", recording_id=recording_id)
        rr.spawn()

    robot = UrdfRobot(render_current_urdf())
    log_robot_static_model(robot, args.robot_path, log_meshes=not args.no_meshes)
    log_stage_targets(stage_record)

    def log_page(offset: int) -> int:
        page = candidates[offset : offset + args.page_size]
        for page_i, candidate in enumerate(page):
            global_i = offset + page_i
            set_sample_time(global_i)
            positions = joint_positions(candidate)
            if positions is not None:
                log_robot_state(robot, positions, args.robot_path)
            log_candidate_text("candidate_info", str(stage_record.get("stage", "")), candidate, global_i)
        next_offset = offset + len(page)
        print(f"stage={stage_record.get('stage')} total_candidates={len(candidates)} shown={offset}:{next_offset}")
        return next_offset

    if args.interactive and args.save:
        raise SystemExit("--interactive 不能和 --save 同时使用；保存 rrd 请用 --offset/--page-size 分页生成。")

    if args.interactive:
        offset = args.offset
        while offset < len(candidates):
            offset = log_page(offset)
            if offset >= len(candidates):
                print("已到最后一页。")
                break
            try:
                answer = input("按 Enter 加载下一页；输入 q 回车退出：").strip().lower()
            except EOFError:
                break
            if answer in {"q", "quit", "exit"}:
                break
    else:
        next_offset = log_page(args.offset)
        if next_offset < len(candidates):
            print("Next page:")
            print(f"  python3 {Path(__file__).resolve()} {args.jsonl} --stage {stage_record.get('stage')} --offset {next_offset} --page-size {args.page_size}")
            print("Interactive page mode:")
            print(f"  python3 {Path(__file__).resolve()} {args.jsonl} --stage {stage_record.get('stage')} --page-size {args.page_size} --interactive")


if __name__ == "__main__":
    main()
