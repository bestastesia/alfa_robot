from robot_motion_curobo.snapshot_collision_sphere_visualizer import (
    densify_points,
    payload_aabb_collision_map,
    unrepaired_loaded_samples,
)
import numpy as np

from robot_motion_curobo.collision_model_visualizer import Cuboid, Payload


def test_densify_points_preserves_endpoints():
    points = [
        {"time_from_start_sec": 0.0, "positions": [0.0, 0.0]},
        {"time_from_start_sec": 1.0, "positions": [0.2, -0.1]},
    ]
    result = densify_points(points, max_step=0.05)
    assert result[0]["positions"] == [0.0, 0.0]
    assert result[-1]["positions"] == [0.2, -0.1]
    assert len(result) == 5


def test_unrepaired_loaded_samples_interpolates_all_target_joints():
    stage = {
        "stage": "selected_loaded_plan",
        "target_names": ["updown", "leftjoint1", "rightjoint1"],
        "start_state": {
            "joint_map": {"pitch": 0.0, "turn": 0.0, "updown": 0.5, "leftjoint1": 1.0, "rightjoint1": -1.0}
        },
        "goal_state": {
            "joint_map": {"pitch": 0.0, "turn": 0.0, "updown": 0.3, "leftjoint1": 0.0, "rightjoint1": 0.0}
        },
        "attached_boxes": [{"id": "left_box"}],
    }
    samples = unrepaired_loaded_samples({"replay_stages": [stage]}, 2)
    assert len(samples) == 3
    assert samples[1].joints["updown"] == 0.4
    assert samples[1].joints["leftjoint1"] == 0.5
    assert samples[1].joints["rightjoint1"] == -0.5
    assert samples[-1].joints["updown"] == 0.3


def test_unrepaired_payload_collision_uses_actual_box_aabb():
    payload = Payload(
        side="left",
        box_id=16,
        link_name="left_tool0",
        grasp_mode="top_suction",
        center_xyz=(0.0, 0.0, 0.2),
        size_xyz=(0.3, 0.4, 0.4),
    )
    transform = np.eye(4)
    transform[:3, 3] = [0.9, 0.0, 0.5]
    rear_guard = Cuboid(
        "box_wall_L16_R18_rear_guard",
        (1.0, 0.0, 0.7),
        (0.01, 2.2, 2.0),
        (1.0, 0.0, 0.0, 0.0),
    )

    collisions, counts = payload_aabb_collision_map(
        (payload,), {"left_tool0": transform}, (rear_guard,)
    )

    assert collisions == {"left": ("box_wall_L16_R18_rear_guard",)}
    assert counts == {"box_wall_L16_R18_rear_guard": 1}
