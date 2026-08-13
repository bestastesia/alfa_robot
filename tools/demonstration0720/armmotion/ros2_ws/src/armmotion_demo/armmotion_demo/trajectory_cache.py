from __future__ import annotations

import gzip
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


CACHE_SCHEMA_VERSION = 2
CACHE_MIN_DISTANCE_CM = 70
CACHE_MAX_DISTANCE_CM = 80


@dataclass(frozen=True)
class TrajectoryCacheMatch:
    path: Path
    distance_cm: int
    row: int
    record: dict[str, Any]

    @property
    def snapshot(self) -> dict[str, Any]:
        return dict(self.record["snapshot"])

    @property
    def pregrasp_state(self) -> dict[str, Any]:
        return dict(self.record["pregrasp_state"])

    @property
    def canonical_targets(self) -> dict[str, Any]:
        return dict(self.record["canonical_targets"])


def upward_distance_bucket_cm(distance_m: float) -> int | None:
    if not math.isfinite(distance_m):
        return None
    distance_cm = int(math.ceil(distance_m * 100.0 - 1e-9))
    return min(CACHE_MAX_DISTANCE_CM, max(CACHE_MIN_DISTANCE_CM, distance_cm))


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


def _pose6d_text(task: Any, side: str) -> str:
    pose = getattr(task, f"{side}_suction_surface_pose", None)
    if pose is None:
        return "unavailable"
    names = ("x", "y", "z", "roll", "pitch", "yaw")
    try:
        values = [float(getattr(pose, name)) for name in names]
    except (AttributeError, TypeError, ValueError):
        return "invalid"
    return ",".join(f"{name}={value:.6f}" for name, value in zip(names, values))


def _distance_excess(value: float) -> float:
    minimum = CACHE_MIN_DISTANCE_CM / 100.0
    maximum = CACHE_MAX_DISTANCE_CM / 100.0
    if value < minimum:
        return value - minimum
    if value > maximum:
        return value - maximum
    return 0.0


def format_cache_miss_diagnostic(task: Any, reason: str, root: Path) -> str:
    left_front = getattr(task, "left_front_face_pose", None)
    right_front = getattr(task, "right_front_face_pose", None)
    left_x = float(getattr(left_front, "x", math.nan))
    right_x = float(getattr(right_front, "x", math.nan))
    lookup_x = max(left_x, right_x)
    raw_bucket = (
        int(math.ceil(lookup_x * 100.0 - 1e-9))
        if math.isfinite(lookup_x)
        else -1
    )
    return (
        "轨迹缓存未命中，已禁止在线 IK/抽离/RRT 规划: "
        f"reason={reason}; "
        f"allowed_front_x=[{CACHE_MIN_DISTANCE_CM / 100.0:.3f},"
        f"{CACHE_MAX_DISTANCE_CM / 100.0:.3f}]m; "
        f"lookup_front_x={lookup_x:.6f}m bucket={raw_bucket}cm; "
        f"left_front_x={left_x:.6f}m left_excess={_distance_excess(left_x):+.6f}m; "
        f"right_front_x={right_x:.6f}m right_excess={_distance_excess(right_x):+.6f}m; "
        f"rows=({int(getattr(task, 'left_row', 0))},"
        f"{int(getattr(task, 'right_row', 0))}); "
        f"left_turn0_pose_6d=({_pose6d_text(task, 'left')}); "
        f"right_turn0_pose_6d=({_pose6d_text(task, 'right')}); "
        f"cache_root={root}"
    )


class TrajectoryCache:
    def __init__(self, root: Path | None = None, *, enabled: bool = True) -> None:
        self.root = (
            Path(root).resolve()
            if root is not None
            else Path(__file__).resolve().parent / "trajectory_cache_pregrasp_v2"
        )
        self.enabled = bool(enabled)

    def find_with_reason(
        self,
        task: Any,
        initial_sample: Any | None = None,
    ) -> tuple[TrajectoryCacheMatch | None, str]:
        if not self.enabled:
            return None, "cache_disabled"
        row = _task_row(task)
        if row is None:
            return None, "row_mismatch_or_out_of_range"
        distance_m = _task_distance_m(task)
        if not math.isfinite(distance_m):
            return None, "front_x_not_finite"
        distance_cm = upward_distance_bucket_cm(distance_m)
        if distance_cm is None:
            return None, "front_x_out_of_cache_range"
        path = self.root / f"x_{distance_cm:02d}cm_row_{row}.json.gz"
        if not path.is_file():
            return None, "cache_file_missing"
        with gzip.open(path, "rt", encoding="utf-8") as stream:
            record = json.load(stream)
        if int(record.get("schema_version", 0)) != CACHE_SCHEMA_VERSION:
            return None, "cache_schema_mismatch"
        if int(record.get("distance_cm", -1)) != distance_cm or int(record.get("row", -1)) != row:
            return None, "cache_metadata_mismatch"
        pregrasp_state = record.get("pregrasp_state")
        canonical_targets = record.get("canonical_targets")
        if not isinstance(pregrasp_state, dict) or not isinstance(canonical_targets, dict):
            return None, "cache_target_state_invalid"
        snapshot = record.get("snapshot")
        if not isinstance(snapshot, dict) or not snapshot.get("replay_stages"):
            return None, "cache_snapshot_invalid"
        return TrajectoryCacheMatch(path, distance_cm, row, record), "cache_hit"

    def find(self, task: Any, initial_sample: Any | None = None) -> TrajectoryCacheMatch | None:
        return self.find_with_reason(task, initial_sample)[0]
