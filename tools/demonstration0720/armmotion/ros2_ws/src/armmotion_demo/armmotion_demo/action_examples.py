from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path
from typing import Any

from robot_motion_runtime.dual_grasp_strategy import Pose6DValue, quaternion_xyzw

from .common import camera_view_pose_from_suction_surface
from .trajectory_cache import CACHE_MAX_DISTANCE_CM, CACHE_MIN_DISTANCE_CM, TrajectoryCache


ACTION_NAME = "/motion/execute_stage"
ACTION_TYPE = "robot_motion_interfaces/action/ExecuteMotionStage"
CAMERA_VIEW = 1
PREGRASP = 2
APPROACH = 3
PLACE = 4
HOME = 5
TOP_SUCTION = 1
SIDE_SUCTION = 2
NO_MOVE = 3
def _pose6d(value: dict[str, Any]) -> Pose6DValue:
    return Pose6DValue(
        x=float(value["x"]),
        y=float(value["y"]),
        z=float(value["z"]),
        roll=float(value["roll"]),
        pitch=float(value["pitch"]),
        yaw=float(value["yaw"]),
    )


def _pose_message(value: Pose6DValue) -> dict[str, Any]:
    orientation = quaternion_xyzw(value)
    return {
        "position": {"x": value.x, "y": value.y, "z": value.z},
        "orientation": {
            "x": orientation[0],
            "y": orientation[1],
            "z": orientation[2],
            "w": orientation[3],
        },
    }


def _recapture_pose(suction_surface: Pose6DValue) -> Pose6DValue:
    return camera_view_pose_from_suction_surface(suction_surface)


def _target_mode(grasp_mode: str) -> int:
    if grasp_mode == "front":
        return SIDE_SUCTION
    if grasp_mode == "top_suction":
        return TOP_SUCTION
    raise ValueError(f"未知吸附模式: {grasp_mode}")


def _target_pair(
    left_pose: Pose6DValue,
    right_pose: Pose6DValue,
    left_mode: int,
    right_mode: int,
) -> dict[str, Any]:
    return {
        "left_grasp_mode": left_mode,
        "left_pose": _pose_message(left_pose),
        "right_grasp_mode": right_mode,
        "right_pose": _pose_message(right_pose),
    }


def _empty_targets() -> dict[str, Any]:
    zero_pose = {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0},
    }
    return {
        "left_grasp_mode": NO_MOVE,
        "left_pose": zero_pose,
        "right_grasp_mode": NO_MOVE,
        "right_pose": zero_pose,
    }


def build_action_example(record: dict[str, Any]) -> dict[str, Any]:
    canonical = record["canonical_targets"]
    left = _pose6d(canonical["left"]["pose_6d"])
    right = _pose6d(canonical["right"]["pose_6d"])
    left_mode = _target_mode(str(canonical["left"]["grasp_mode"]))
    right_mode = _target_mode(str(canonical["right"]["grasp_mode"]))
    empty = _empty_targets()
    return {
        "cache_key": f"x_{int(record['distance_cm']):02d}cm_row_{int(record['row'])}",
        "distance_cm": int(record["distance_cm"]),
        "row": int(record["row"]),
        "action_name": ACTION_NAME,
        "action_type": ACTION_TYPE,
        "goals": [
            {
                "label": "CAMERA_VIEW",
                "goal": {
                    "execution_stage": CAMERA_VIEW,
                    "targets": _target_pair(
                        _recapture_pose(left),
                        _recapture_pose(right),
                        left_mode,
                        right_mode,
                    ),
                },
            },
            {
                "label": "PREGRASP",
                "goal": {
                    "execution_stage": PREGRASP,
                    "targets": _target_pair(left, right, left_mode, right_mode),
                },
            },
            {"label": "APPROACH", "goal": {"execution_stage": APPROACH, "targets": empty}},
            {"label": "PLACE", "goal": {"execution_stage": PLACE, "targets": empty}},
            {"label": "HOME", "goal": {"execution_stage": HOME, "targets": empty}},
        ],
    }


def load_examples(cache_root: Path | None = None) -> list[dict[str, Any]]:
    root = TrajectoryCache(cache_root).root
    examples: list[dict[str, Any]] = []
    for distance_cm in range(CACHE_MIN_DISTANCE_CM, CACHE_MAX_DISTANCE_CM + 1):
        for row in range(1, 6):
            path = root / f"x_{distance_cm:02d}cm_row_{row}.json.gz"
            if not path.is_file():
                continue
            with gzip.open(path, "rt", encoding="utf-8") as stream:
                examples.append(build_action_example(json.load(stream)))
    if not examples:
        raise FileNotFoundError(f"缓存目录中没有可用轨迹: {root}")
    return examples


def as_single_arm(example: dict[str, Any], active_arm: str) -> dict[str, Any]:
    if active_arm not in {"left", "right"}:
        raise ValueError(f"单臂必须是 left/right，当前 {active_arm}")
    result = json.loads(json.dumps(example))
    inactive_arm = "right" if active_arm == "left" else "left"
    zero = _empty_targets()
    for stage in result["goals"][:2]:
        targets = stage["goal"]["targets"]
        targets[f"{inactive_arm}_grasp_mode"] = NO_MOVE
        targets[f"{inactive_arm}_pose"] = zero[f"{inactive_arm}_pose"]
    result["single_arm"] = active_arm
    return result


def _command(goal: dict[str, Any]) -> str:
    payload = json.dumps(goal, ensure_ascii=False, separators=(",", ":"))
    return f"ros2 action send_goal {ACTION_NAME} {ACTION_TYPE} '{payload}' --feedback"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="输出可用缓存对应的真实 Motion Action Goal")
    parser.add_argument("--distance-cm", type=int)
    parser.add_argument("--row", type=int)
    parser.add_argument("--single-arm", choices=("left", "right"))
    parser.add_argument("--format", choices=("json", "commands"), default="json")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> None:
    options = parse_args()
    examples = load_examples()
    if options.distance_cm is not None:
        examples = [item for item in examples if item["distance_cm"] == options.distance_cm]
    if options.row is not None:
        examples = [item for item in examples if item["row"] == options.row]
    if options.single_arm is not None:
        examples = [as_single_arm(item, options.single_arm) for item in examples]
    if not examples:
        raise SystemExit("没有匹配的缓存 Action 样例")
    if options.format == "commands":
        lines: list[str] = []
        for item in examples:
            lines.append(f"# {item['cache_key']}")
            for stage in item["goals"]:
                lines.append(f"# {stage['label']}")
                lines.append(_command(stage["goal"]))
        output = "\n".join(lines) + "\n"
    else:
        wire_goal_groups = {
            item["cache_key"]: [stage["goal"] for stage in item["goals"]]
            for item in examples
        }
        output = json.dumps(
            {
                "schema_version": 2,
                "units": {"position": "m", "orientation": "quaternion_xyzw"},
                "diagnostic_only": {
                    "action_name": ACTION_NAME,
                    "action_type": ACTION_TYPE,
                    "group_keys": "x_XXcm_row_N仅用于定位样例，不属于Action Goal",
                },
                "wire_goal_groups": wire_goal_groups,
            },
            ensure_ascii=False,
            indent=2,
        ) + "\n"
    if options.output is None:
        print(output, end="")
    else:
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(output, encoding="utf-8")
        print(options.output)


if __name__ == "__main__":
    main()
