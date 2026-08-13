import gzip
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from armmotion_demo.planner_adapter import PlannerAdapter
from armmotion_demo.trajectory_cache import (
    TrajectoryCache,
    format_cache_miss_diagnostic,
    upward_distance_bucket_cm,
)


def task(distance_m=0.725, row=2, scene_y_shift=0.0):
    return SimpleNamespace(
        left_row=row,
        right_row=row,
        left_front_face_pose=SimpleNamespace(x=distance_m),
        right_front_face_pose=SimpleNamespace(x=distance_m),
        left_suction_surface_pose=SimpleNamespace(
            x=distance_m,
            y=0.4,
            z=1.6,
            roll=3.14,
            pitch=-1.57,
            yaw=0.0,
        ),
        right_suction_surface_pose=SimpleNamespace(
            x=distance_m,
            y=-0.4,
            z=1.6,
            roll=3.14,
            pitch=-1.57,
            yaw=0.0,
        ),
        scene_y_shift=scene_y_shift,
    )


def test_distance_bucket_rounds_up_to_next_centimetre():
    assert upward_distance_bucket_cm(0.70) == 70
    assert upward_distance_bucket_cm(0.725) == 73
    assert upward_distance_bucket_cm(0.7500000001) == 76
    assert upward_distance_bucket_cm(0.82) == 80
    assert upward_distance_bucket_cm(0.65) == 70


@pytest.fixture
def cache_root(tmp_path):
    for distance_cm in range(70, 81):
        for row in range(1, 6):
            record = {
                "schema_version": 2,
                "distance_cm": distance_cm,
                "row": row,
                "pregrasp_state": {"joints": {}, "updown_m": 0.3},
                "canonical_targets": {"left": {}, "right": {}},
                "snapshot": {"replay_stages": [{"stage": "contact"}]},
            }
            path = tmp_path / f"x_{distance_cm:02d}cm_row_{row}.json.gz"
            with gzip.open(path, "wt", encoding="utf-8") as stream:
                json.dump(record, stream)
    return tmp_path


def test_cache_contains_all_eleven_distances_and_five_rows(cache_root):
    cache = TrajectoryCache(cache_root)
    for distance_cm in range(70, 81):
        for row in range(1, 6):
            match = cache.find(task(distance_cm / 100.0, row))
            assert match is not None
            assert match.distance_cm == distance_cm
            assert match.row == row


def test_cache_uses_larger_x_bucket(cache_root):
    match = TrajectoryCache(cache_root).find(task(0.725, 2))
    assert match is not None
    assert match.distance_cm == 73


def test_cache_ignores_small_y_layout_offset(cache_root):
    assert TrajectoryCache(cache_root).find(task(scene_y_shift=0.01)) is not None


def test_cache_rejects_unequal_rows(cache_root):
    value = task()
    value.right_row = 3
    assert TrajectoryCache(cache_root).find(value) is None


def test_cache_does_not_require_recapture_state_to_match_cached_pregrasp(cache_root):
    initial = SimpleNamespace(updown_m=0.3, joints={"left_joint1": 1.0})
    assert TrajectoryCache(cache_root).find(task(0.70, 1), initial) is not None


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
    assert match.distance_cm == 80


def test_required_cache_policy_rejects_before_online_planning(cache_root):
    value = task(0.726, 1)
    value.right_row = 2
    adapter = PlannerAdapter.__new__(PlannerAdapter)
    adapter.trajectory_cache = TrajectoryCache(cache_root)
    adapter.trajectory_cache_required = True

    with pytest.raises(RuntimeError, match="已禁止在线 IK/抽离/RRT"):
        adapter._resolve_cache_match(value, initial_sample=object())
