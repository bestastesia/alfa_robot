import numpy as np

from robot_motion_curobo.collision_model_visualizer import Cuboid, Payload
from robot_motion_curobo.rejected_trajectory_dual_scene_visualizer import (
    aabb_from_payload,
    aabb_overlaps,
)


def test_payload_world_aabb_uses_rotated_corners():
    payload = Payload(
        side="left",
        box_id=16,
        link_name="left_tool0",
        grasp_mode="top_suction",
        center_xyz=(0.0, 0.0, 0.2),
        size_xyz=(0.2, 0.4, 0.4),
    )
    transform = np.eye(4)
    transform[:3, :3] = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    transform[:3, 3] = [1.0, 2.0, 3.0]

    lower, upper = aabb_from_payload(payload, transform)

    assert np.allclose(lower, [0.8, 1.9, 3.0])
    assert np.allclose(upper, [1.2, 2.1, 3.4])


def test_aabb_overlap_matches_production_inclusive_boundary():
    rear_guard = Cuboid(
        "box_wall_L16_R18_rear_guard",
        (1.0, 0.0, 0.5),
        (0.1, 2.0, 1.0),
        (1.0, 0.0, 0.0, 0.0),
    )

    assert aabb_overlaps(
        np.asarray([0.8, -0.1, 0.2]), np.asarray([0.95, 0.1, 0.8]), rear_guard
    )
    assert not aabb_overlaps(
        np.asarray([0.8, -0.1, 0.2]), np.asarray([0.949, 0.1, 0.8]), rear_guard
    )
