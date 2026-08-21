from __future__ import annotations

import gzip
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any


CACHE_SCHEMA_VERSION = 3
CACHE_MIN_DISTANCE_CM = 75
CACHE_MAX_DISTANCE_CM = 82
CACHE_MIN_LATERAL_OFFSET_CM = -10
CACHE_MAX_LATERAL_OFFSET_CM = 10
CACHE_LATERAL_OFFSET_STEP_CM = 2
CACHE_NOMINAL_ARM_SPACING_CM = 82


@dataclass(frozen=True)
class TrajectoryCacheMatch:
    path: Path
    distance_cm: int
    lateral_offset_cm: int
    requested_distance_cm: int
    requested_lateral_offset_cm: int
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


def nearest_lateral_offset_bucket_cm(offset_m: float) -> int | None:
    if not math.isfinite(offset_m):
        return None
    offset_cm = int(round(offset_m * 100.0 / CACHE_LATERAL_OFFSET_STEP_CM))
    offset_cm *= CACHE_LATERAL_OFFSET_STEP_CM
    return min(
        CACHE_MAX_LATERAL_OFFSET_CM,
        max(CACHE_MIN_LATERAL_OFFSET_CM, offset_cm),
    )


def cache_filename(distance_cm: int, lateral_offset_cm: int, row: int) -> str:
    offset_token = (
        f"p{lateral_offset_cm:02d}"
        if lateral_offset_cm >= 0
        else f"m{abs(lateral_offset_cm):02d}"
    )
    return f"x_{distance_cm:02d}cm_y_{offset_token}cm_row_{row}.json.gz"


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


def _task_lateral_geometry_m(task: Any) -> tuple[float, float]:
    left_pose = getattr(task, "left_suction_surface_pose", None)
    right_pose = getattr(task, "right_suction_surface_pose", None)
    if left_pose is None or right_pose is None:
        left_pose = getattr(task, "left_front_face_pose", None)
        right_pose = getattr(task, "right_front_face_pose", None)
    if left_pose is None or right_pose is None:
        return math.nan, math.nan
    left_y = float(getattr(left_pose, "y", math.nan))
    right_y = float(getattr(right_pose, "y", math.nan))
    return 0.5 * (left_y + right_y), left_y - right_y


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
    lateral_offset_m, spacing_m = _task_lateral_geometry_m(task)
    lateral_bucket = nearest_lateral_offset_bucket_cm(lateral_offset_m)
    return (
        "轨迹缓存未命中，已禁止在线 IK/抽离/RRT 规划: "
        f"reason={reason}; "
        f"allowed_front_x=[{CACHE_MIN_DISTANCE_CM / 100.0:.3f},"
        f"{CACHE_MAX_DISTANCE_CM / 100.0:.3f}]m; "
        f"lookup_front_x={lookup_x:.6f}m bucket={raw_bucket}cm; "
        f"left_front_x={left_x:.6f}m left_excess={_distance_excess(left_x):+.6f}m; "
        f"right_front_x={right_x:.6f}m right_excess={_distance_excess(right_x):+.6f}m; "
        f"allowed_lateral_offset=[{CACHE_MIN_LATERAL_OFFSET_CM},"
        f"{CACHE_MAX_LATERAL_OFFSET_CM}]cm step={CACHE_LATERAL_OFFSET_STEP_CM}cm; "
        f"lookup_lateral_offset={lateral_offset_m:.6f}m bucket={lateral_bucket}cm; "
        f"arm_spacing={spacing_m:.6f}m nominal={CACHE_NOMINAL_ARM_SPACING_CM / 100.0:.3f}m; "
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
            else Path(__file__).resolve().parent / "trajectory_cache_pregrasp_v3"
        )
        self.enabled = bool(enabled)
        self._available = self._available_cache_entries()

    def _available_cache_entries(self) -> dict[tuple[int, int, int], Path]:
        pattern = re.compile(
            r"^x_(\d+)cm_y_([pm])(\d+)cm_row_(\d+)\.json\.gz$"
        )
        entries: dict[tuple[int, int, int], Path] = {}
        if not self.root.is_dir():
            return entries
        for path in self.root.glob("*.json.gz"):
            matched = pattern.match(path.name)
            if matched is None:
                continue
            distance_cm = int(matched.group(1))
            lateral_offset_cm = int(matched.group(3))
            if matched.group(2) == "m":
                lateral_offset_cm = -lateral_offset_cm
            row = int(matched.group(4))
            entries[(distance_cm, lateral_offset_cm, row)] = path
        return entries

    def _nearest_successful_entry(
        self,
        distance_cm: int,
        lateral_offset_cm: int,
        row: int,
    ) -> tuple[tuple[int, int, int], Path] | None:
        candidates = [
            (key, path)
            for key, path in self._available.items()
            if key[2] == row
        ]
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda item: (
                abs(item[0][0] - distance_cm)
                + abs(item[0][1] - lateral_offset_cm),
                abs(item[0][0] - distance_cm),
                abs(item[0][1] - lateral_offset_cm),
                -item[0][0],
            ),
        )

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
        lateral_offset_m, _ = _task_lateral_geometry_m(task)
        lateral_offset_cm = nearest_lateral_offset_bucket_cm(lateral_offset_m)
        if lateral_offset_cm is None:
            return None, "lateral_offset_not_finite"
        requested_distance_cm = distance_cm
        requested_lateral_offset_cm = lateral_offset_cm
        selected_key = (distance_cm, lateral_offset_cm, row)
        path = self._available.get(selected_key)
        reason = "cache_hit"
        if path is None:
            nearest = self._nearest_successful_entry(
                distance_cm,
                lateral_offset_cm,
                row,
            )
            if nearest is None:
                return None, "cache_file_missing"
            selected_key, path = nearest
            distance_cm, lateral_offset_cm, _ = selected_key
            reason = "cache_hit_nearest_success"
        with gzip.open(path, "rt", encoding="utf-8") as stream:
            record = json.load(stream)
        if int(record.get("schema_version", 0)) != CACHE_SCHEMA_VERSION:
            return None, "cache_schema_mismatch"
        if (
            int(record.get("distance_cm", -1)) != distance_cm
            or int(record.get("lateral_offset_cm", 999)) != lateral_offset_cm
            or int(record.get("row", -1)) != row
        ):
            return None, "cache_metadata_mismatch"
        pregrasp_state = record.get("pregrasp_state")
        canonical_targets = record.get("canonical_targets")
        if not isinstance(pregrasp_state, dict) or not isinstance(canonical_targets, dict):
            return None, "cache_target_state_invalid"
        snapshot = record.get("snapshot")
        if not isinstance(snapshot, dict) or not snapshot.get("replay_stages"):
            return None, "cache_snapshot_invalid"
        return TrajectoryCacheMatch(
            path,
            distance_cm,
            lateral_offset_cm,
            requested_distance_cm,
            requested_lateral_offset_cm,
            row,
            record,
        ), reason

    def find(self, task: Any, initial_sample: Any | None = None) -> TrajectoryCacheMatch | None:
        return self.find_with_reason(task, initial_sample)[0]
