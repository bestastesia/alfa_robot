from __future__ import annotations

import gzip
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


CACHE_SCHEMA_VERSION = 1
CACHE_MIN_DISTANCE_CM = 70
CACHE_MAX_DISTANCE_CM = 75
CACHE_DEFAULT_SCENE_Y_SHIFT_M = 0.0
CACHE_INITIAL_TOLERANCE = 1e-6


@dataclass(frozen=True)
class TrajectoryCacheMatch:
    path: Path
    distance_cm: int
    row: int
    record: dict[str, Any]

    @property
    def snapshot(self) -> dict[str, Any]:
        return dict(self.record["snapshot"])


def upward_distance_bucket_cm(distance_m: float) -> int | None:
    if not math.isfinite(distance_m):
        return None
    distance_cm = int(math.ceil(distance_m * 100.0 - 1e-9))
    if not CACHE_MIN_DISTANCE_CM <= distance_cm <= CACHE_MAX_DISTANCE_CM:
        return None
    return distance_cm


def _task_row(task: Any) -> int | None:
    left_row = int(getattr(task, "left_row", 0))
    right_row = int(getattr(task, "right_row", 0))
    if left_row or right_row:
        return left_row if left_row == right_row and 1 <= left_row <= 5 else None
    index = int(getattr(task, "index", 0))
    return index if 1 <= index <= 5 else None


def _task_distance_m(task: Any) -> float:
    left_pose = getattr(task, "left_front_face_pose", None)
    right_pose = getattr(task, "right_front_face_pose", None)
    if left_pose is not None and right_pose is not None:
        return max(float(left_pose.x), float(right_pose.x))
    return float(task.effective_distance_m)


def _initial_state_matches(record: dict[str, Any], initial_sample: Any | None) -> bool:
    if initial_sample is None:
        return True
    expected = record.get("initial_state", {})
    expected_joints = expected.get("joints", {})
    if not isinstance(expected_joints, dict):
        return False
    if abs(float(initial_sample.updown_m) - float(expected.get("updown_m", 0.0))) > CACHE_INITIAL_TOLERANCE:
        return False
    return all(
        name in initial_sample.joints
        and abs(float(initial_sample.joints[name]) - float(value)) <= CACHE_INITIAL_TOLERANCE
        for name, value in expected_joints.items()
    )


class TrajectoryCache:
    def __init__(self, root: Path | None = None, *, enabled: bool = True) -> None:
        self.root = (
            Path(root).resolve()
            if root is not None
            else Path(__file__).resolve().parent / "trajectory_cache"
        )
        self.enabled = bool(enabled)

    def find(self, task: Any, initial_sample: Any | None = None) -> TrajectoryCacheMatch | None:
        if not self.enabled:
            return None
        row = _task_row(task)
        if row is None:
            return None
        scene_y_shift = getattr(task, "scene_y_shift", 0.0)
        if scene_y_shift is None:
            scene_y_shift = 0.0
        if abs(float(scene_y_shift) - CACHE_DEFAULT_SCENE_Y_SHIFT_M) > 1e-6:
            return None
        distance_cm = upward_distance_bucket_cm(_task_distance_m(task))
        if distance_cm is None:
            return None
        path = self.root / f"x_{distance_cm:02d}cm_row_{row}.json.gz"
        if not path.is_file():
            return None
        with gzip.open(path, "rt", encoding="utf-8") as stream:
            record = json.load(stream)
        if int(record.get("schema_version", 0)) != CACHE_SCHEMA_VERSION:
            return None
        if int(record.get("distance_cm", -1)) != distance_cm or int(record.get("row", -1)) != row:
            return None
        if not _initial_state_matches(record, initial_sample):
            return None
        snapshot = record.get("snapshot")
        if not isinstance(snapshot, dict) or not snapshot.get("replay_stages"):
            return None
        return TrajectoryCacheMatch(path, distance_cm, row, record)
