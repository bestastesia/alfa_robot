import gzip
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from armmotion_demo.planner_adapter import PlannerAdapter
from armmotion_demo.trajectory_cache import (
    CACHE_LATERAL_OFFSET_STEP_CM,
    CACHE_MAX_DISTANCE_CM,
    CACHE_MAX_LATERAL_OFFSET_CM,
    CACHE_MIN_DISTANCE_CM,
    CACHE_MIN_LATERAL_OFFSET_CM,
    TrajectoryCache,
    cache_filename,
    format_cache_miss_diagnostic,
    nearest_lateral_offset_bucket_cm,
    upward_distance_bucket_cm,
)


def task(distance_m=0.725, row=2, lateral_offset_m=0.0):
    return SimpleNamespace(
        left_row=row,
        right_row=row,
        left_front_face_pose=SimpleNamespace(x=distance_m),
        right_front_face_pose=SimpleNamespace(x=distance_m),
        left_suction_surface_pose=SimpleNamespace(
            x=distance_m,
            y=0.41 + lateral_offset_m,
            z=1.6,
            roll=3.14,
            pitch=-1.57,
            yaw=0.0,
        ),
        right_suction_surface_pose=SimpleNamespace(
            x=distance_m,
            y=-0.41 + lateral_offset_m,
            z=1.6,
            roll=3.14,
            pitch=-1.57,
            yaw=0.0,
        ),
        scene_y_shift=lateral_offset_m,
    )


def test_distance_bucket_rounds_up_to_next_centimetre():
    assert upward_distance_bucket_cm(0.70) == 75
    assert upward_distance_bucket_cm(0.725) == 75
    assert upward_distance_bucket_cm(0.7500000001) == 76
    assert upward_distance_bucket_cm(0.82) == 82
    assert upward_distance_bucket_cm(0.90) == 82
    assert upward_distance_bucket_cm(0.65) == 75


def test_lateral_offset_bucket_uses_nearest_two_centimetres_and_clamps():
    assert nearest_lateral_offset_bucket_cm(-0.10) == -10
    assert nearest_lateral_offset_bucket_cm(-0.074) == -8
    assert nearest_lateral_offset_bucket_cm(0.075) == 8
    assert nearest_lateral_offset_bucket_cm(0.15) == 10


@pytest.fixture
def cache_root(tmp_path):
    for distance_cm in range(CACHE_MIN_DISTANCE_CM, CACHE_MAX_DISTANCE_CM + 1):
        for lateral_offset_cm in range(
            CACHE_MIN_LATERAL_OFFSET_CM,
            CACHE_MAX_LATERAL_OFFSET_CM + 1,
            CACHE_LATERAL_OFFSET_STEP_CM,
        ):
            for row in range(1, 6):
                record = {
                    "schema_version": 3,
                    "distance_cm": distance_cm,
                    "lateral_offset_cm": lateral_offset_cm,
                    "row": row,
                    "pregrasp_state": {"joints": {}, "updown_m": 0.3},
                    "canonical_targets": {"left": {}, "right": {}},
                    "snapshot": {"replay_stages": [{"stage": "contact"}]},
                }
                path = tmp_path / cache_filename(distance_cm, lateral_offset_cm, row)
                with gzip.open(path, "wt", encoding="utf-8") as stream:
                    json.dump(record, stream)
    return tmp_path


def test_cache_contains_complete_two_centimetre_grid(cache_root):
    cache = TrajectoryCache(cache_root)
    for distance_cm in range(CACHE_MIN_DISTANCE_CM, CACHE_MAX_DISTANCE_CM + 1):
        for lateral_offset_cm in range(
            CACHE_MIN_LATERAL_OFFSET_CM,
            CACHE_MAX_LATERAL_OFFSET_CM + 1,
            CACHE_LATERAL_OFFSET_STEP_CM,
        ):
            for row in range(1, 6):
                match = cache.find(
                    task(distance_cm / 100.0, row, lateral_offset_cm / 100.0)
                )
                assert match is not None
                assert match.distance_cm == distance_cm
                assert match.lateral_offset_cm == lateral_offset_cm
                assert match.row == row
def test_odd_lateral_offset_maps_to_nearest_two_centimetre_entry(cache_root):
    match, reason = TrajectoryCache(cache_root).find_with_reason(
        task(0.82, 3, 0.07)
    )
    assert reason == "cache_hit"
    assert match is not None
    assert match.distance_cm == 82
    assert match.lateral_offset_cm in {6, 8}


def test_cache_uses_larger_x_bucket(cache_root):
    match = TrajectoryCache(cache_root).find(task(0.755, 2))
    assert match is not None
    assert match.distance_cm == 76


def test_cache_selects_distinct_lateral_offset(cache_root):
    match = TrajectoryCache(cache_root).find(task(lateral_offset_m=0.071))
    assert match is not None
    assert match.lateral_offset_cm == 8


def test_cache_uses_nearest_successful_entry_when_exact_cell_is_missing(cache_root):
    missing = cache_root / cache_filename(78, 10, 3)
    missing.unlink()

    match, reason = TrajectoryCache(cache_root).find_with_reason(
        task(0.78, 3, 0.10)
    )

    assert reason == "cache_hit_nearest_success"
    assert match is not None
    assert match.requested_distance_cm == 78
    assert match.requested_lateral_offset_cm == 10
    assert (match.distance_cm, match.lateral_offset_cm) == (79, 10)


def test_cache_rejects_unequal_rows(cache_root):
    value = task()
    value.right_row = 3
    assert TrajectoryCache(cache_root).find(value) is None


def test_cache_does_not_require_recapture_state_to_match_cached_pregrasp(cache_root):
    initial = SimpleNamespace(updown_m=0.3, joints={"left_joint1": 1.0})
    assert TrajectoryCache(cache_root).find(task(0.75, 1), initial) is not None


def test_far_target_clamps_to_farthest_cache_entry(cache_root):
    value = task(0.725, 1)
    value.left_front_face_pose.x = 0.8262565202
    value.right_front_face_pose.x = 0.9259990334
    value.left_suction_surface_pose = SimpleNamespace(
        x=0.8262565202,
        y=0.5077833853,
        z=1.6221428853,
        roll=-3.1415926123,
        pitch=-1.5143182051,
        yaw=0.1988427342,
    )
    value.right_suction_surface_pose = SimpleNamespace(
        x=0.9259990334,
        y=-0.2949195610,
        z=1.6443653240,
        roll=0.0000002778,
        pitch=-1.5598133979,
        yaw=-2.9541003065,
    )

    cache = TrajectoryCache(cache_root)
    match, reason = cache.find_with_reason(value)

    assert reason == "cache_hit"
    assert match is not None
    assert match.distance_cm == 82


def test_required_cache_policy_rejects_before_online_planning(cache_root):
    value = task(0.726, 1)
    value.right_row = 2
    adapter = PlannerAdapter.__new__(PlannerAdapter)
    adapter.trajectory_cache = TrajectoryCache(cache_root)
    adapter.trajectory_cache_required = True

    with pytest.raises(RuntimeError, match="已禁止在线 IK/抽离/RRT"):
        adapter._resolve_cache_match(value, initial_sample=object())
