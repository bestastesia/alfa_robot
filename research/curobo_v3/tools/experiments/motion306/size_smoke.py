"""MOTION-306 M0a GPU check: box-size scenes are built consistently and run real strict three-stage episodes.

Per size level and mode one train instance (zero docking offset) runs the frozen MOTION-311 R3 seed-0 policy
through the strict evaluator (one action per stage, no retry). Scene, contact, payload, extraction and
conveyor geometry are checked against v3_box_geometry. The legacy path replays stored MOTION-311 dev
records (same instance, planner seed and checkpoint) and must reproduce their frame hashes.
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from motion311_learning.evaluate import _load_policy, evaluate_instance, mode
from motion311_learning.stage_env import StageEnv
from v3_batched_loaded_search import loaded_robot
from v3_box_geometry import SPHERE_MODEL_NAME, extraction_distance_m, wall_center
from v3_conveyor_cycle_planner import ConveyorCyclePlanner
from v3_scene_variants import load_registry
from v3_task_scene import grasp_transform
from v3_wall_ik_benchmark import ACTIVE_JOINTS

ROOT = Path(__file__).resolve().parents[3]
MODES = ("dual-side", "dual-top", "single-right-side", "single-left-top")
LEGACY_KEYS = ("success", "status", "failure_stage", "reason", "raw_t_path_s", "third_stage_reached")


def size_items(registry, levels):
    chosen = {}
    for item in registry["instances"]:
        if item["group"] == "train" and item["variant"] == 0 and item["box_level"] in levels:
            chosen.setdefault((item["box_level"], mode(item)), item)
    return [chosen[(level, name)] for level in levels for name in MODES if (level, name) in chosen]


def tool_positions(env, row):
    env.planner.fk(np.asarray(row)[env.active_indices])
    return {side: env.planner.fk_robot.get_transform(side + "_tool0", env.planner.fk_robot.base_link)
            for side in env.boxes}


def scene_checks(env, item):
    planner, dims = env.planner, tuple(item["box_dims"])
    checks = dict(sphere_model=planner.box_fit["name"] == SPHERE_MODEL_NAME and tuple(planner.box_fit["dimensions_m"]) == dims)
    loaded = loaded_robot(planner.robot, planner.box_fit, attachments=planner.task_attachments(env.boxes))
    for side, box_id in env.boxes.items():
        box = planner.snapshot.object(f"wall_box_{box_id:02d}")
        expected = wall_center(planner.front, 0.9, box_id, dims) + (item["offset_m"][0] - planner.base_advance, item["offset_m"][1], 0.001)
        checks[side + "_box_pose_dims"] = box.dimensions_m == dims and bool(np.allclose(box.pose.position, expected, atol=1e-9))
        face = np.asarray(box.pose.position) + ((0, 0, dims[2] / 2) if planner.top_grasp else (-dims[0] / 2, 0, 0))
        contact = planner.contact_poses_for_task(env.boxes)[side + "_tool0"]["position"]  # env.targets moves on per stage
        checks[side + "_contact_on_face"] = bool(np.allclose(contact, face, atol=1e-9))
        extra = (len(loaded["kinematics"]["collision_spheres"][side + "_link7"])
                 - len(planner.robot["kinematics"]["collision_spheres"][side + "_link7"]))
        checks[side + "_payload_spheres"] = extra == planner.box_fit["actual_spheres"]
    return checks


def episode_checks(env, item):
    dims, phases, frames = tuple(item["box_dims"]), env.report["phases"], env.report["frames"]
    checks = {}
    if "attach" in phases and "extract" in phases:
        start = tool_positions(env, frames[phases.index("attach")])
        end = tool_positions(env, frames[len(phases) - 1 - phases[::-1].index("extract")])
        pulls = {side: float(start[side][0, 3] - end[side][0, 3]) for side in env.boxes}
        checks["extraction_m"] = pulls
        checks["extraction_matches"] = all(abs(value - extraction_distance_m(dims)) < 2e-3 for value in pulls.values())
    if "release" in phases:
        tools = tool_positions(env, frames[phases.index("release")])
        bottoms = {side: float((tool @ grasp_transform(side, env.planner.top_grasp, dims))[2, 3] - dims[2] / 2)
                   for side, tool in tools.items()}
        target = ConveyorCyclePlanner.conveyor_height_m + ConveyorCyclePlanner.placement_clearance_m
        checks["release_box_bottom_m"] = bottoms
        checks["release_height_matches"] = all(abs(value - target) < 3e-3 for value in bottoms.values())
    timed = env.report.get("timed_validation")
    checks["timed_validation_success"] = None if timed is None else bool(timed.get("success"))
    return checks


def compact(row):
    return {key: row.get(key) for key in ("instance", "mode", "success", "status", "failure_stage", "reason",
                                          "third_stage_reached", "raw_t_path_s", "executed_t_path_s", "wall_s")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--models", type=Path, default=Path("/home/astesia/Sevenova/motion311-fourroutes/formal-v3/r3-seed0"))
    parser.add_argument("--legacy-records", type=Path,
                        default=Path("/home/astesia/Sevenova/motion311-fourroutes/formal-v3/eval-full-training-seed0/r3/dev-s101/instances"))
    parser.add_argument("--legacy-count", type=int, default=6)
    parser.add_argument("--levels", default="")
    parser.add_argument("--seed", type=int, default=101)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    size_registry = load_registry(ROOT / "config/motion306_registry.json")
    legacy_registry = {item["instance"]: item for item in load_registry(ROOT / "config/motion311_registry.json")["instances"]}
    levels = [level for level in (args.levels.split(",") if args.levels else size_registry["size_levels"])]
    policy, checkpoint = _load_policy("r3", args.models)
    began = time.perf_counter()
    env = StageEnv(timed=True, physical=False)
    print(json.dumps(dict(event="env_ready", init_s=time.perf_counter() - began)), flush=True)
    results = dict(checkpoint=checkpoint, seed=args.seed, size=[], legacy=[])
    for item in size_items(size_registry, levels):
        row = evaluate_instance(env, item, "r3", policy, args.seed)
        entry = dict(compact(row), box_level=item["box_level"], box_dims=item["box_dims"])
        if row["status"] != "infrastructure" and getattr(env, "item", {}).get("instance") == item["instance"]:
            entry["scene"] = scene_checks(env, item)
            entry["episode"] = episode_checks(env, item)
        results["size"].append(entry)
        print(json.dumps(entry, ensure_ascii=False), flush=True)
    stored = sorted((json.loads(path.read_text()) for path in args.legacy_records.glob("*.json")),
                    key=lambda record: (not record["success"], record["instance"]))
    for record in [r for r in stored if r["success"]][:args.legacy_count // 2] + [r for r in stored if not r["success"]][:args.legacy_count - args.legacy_count // 2]:
        row = evaluate_instance(env, legacy_registry[record["instance"]], "r3", policy, args.seed)
        same = {key: row.get(key) == record.get(key) for key in LEGACY_KEYS}
        same["frames_sha256"] = row["report"].get("frames_sha256") == record["report"].get("frames_sha256")
        entry = dict(compact(row), identical=same, all_identical=all(same.values()))
        results["legacy"].append(entry)
        print(json.dumps(entry, ensure_ascii=False), flush=True)
    results["wall_s"] = time.perf_counter() - began
    (args.output / "size_smoke.json").write_text(json.dumps(results, ensure_ascii=False, indent=1) + "\n")


if __name__ == "__main__":
    main()
