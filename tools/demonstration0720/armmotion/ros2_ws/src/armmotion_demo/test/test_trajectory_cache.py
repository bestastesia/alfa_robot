from pathlib import Path
from types import SimpleNamespace

import pytest

from armmotion_demo.trajectory_cache import TrajectoryCache, upward_distance_bucket_cm


def task(distance_m=0.725, row=2, scene_y_shift=0.0):
    return SimpleNamespace(
        left_row=row,
        right_row=row,
        left_front_face_pose=SimpleNamespace(x=distance_m),
        right_front_face_pose=SimpleNamespace(x=distance_m),
        scene_y_shift=scene_y_shift,
    )


def test_distance_bucket_rounds_up_to_next_centimetre():
    assert upward_distance_bucket_cm(0.70) == 70
    assert upward_distance_bucket_cm(0.725) == 73
    assert upward_distance_bucket_cm(0.7500000001) is None


def test_default_cache_contains_all_six_distances_and_five_rows():
    cache = TrajectoryCache()
    for distance_cm in range(70, 76):
        for row in range(1, 6):
            match = cache.find(task(distance_cm / 100.0, row))
            assert match is not None
            assert match.distance_cm == distance_cm
            assert match.row == row


def test_cache_uses_larger_x_bucket():
    match = TrajectoryCache().find(task(0.725, 2))
    assert match is not None
    assert match.distance_cm == 73


def test_cache_rejects_non_default_y_layout():
    assert TrajectoryCache().find(task(scene_y_shift=0.01)) is None


def test_cache_rejects_unequal_rows():
    value = task()
    value.right_row = 3
    assert TrajectoryCache().find(value) is None


def test_cache_rejects_incompatible_initial_state():
    initial = SimpleNamespace(updown_m=0.3, joints={"left_joint1": 1.0})
    assert TrajectoryCache().find(task(0.70, 1), initial) is None
