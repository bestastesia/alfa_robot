import math

import numpy as np

from robot_motion_curobo.collision_model_visualizer import (
    Cuboid,
    Payload,
    Sphere,
    cuboid_covering_spheres,
    sphere_intersects_cuboid,
    transform_spheres,
)


def test_sphere_intersects_axis_aligned_and_rotated_cuboid():
    box = Cuboid("box", (0.0, 0.0, 0.0), (2.0, 1.0, 1.0), (1.0, 0.0, 0.0, 0.0))
    assert sphere_intersects_cuboid(Sphere("link", (1.1, 0.0, 0.0), 0.11), box)
    assert not sphere_intersects_cuboid(Sphere("link", (1.2, 0.0, 0.0), 0.1), box)

    angle = math.pi * 0.5
    rotated = Cuboid(
        "rotated",
        (0.0, 0.0, 0.0),
        (2.0, 1.0, 1.0),
        (math.cos(angle * 0.5), 0.0, 0.0, math.sin(angle * 0.5)),
    )
    assert sphere_intersects_cuboid(Sphere("link", (0.0, 1.05, 0.0), 0.06), rotated)
    assert not sphere_intersects_cuboid(Sphere("link", (1.05, 0.0, 0.0), 0.06), rotated)


def test_payload_cover_matches_backend_grid_shape_and_radius():
    payload = Payload(
        side="left",
        box_id=11,
        link_name="left_tool0",
        grasp_mode="top_suction",
        center_xyz=(0.0, 0.0, 0.2),
        size_xyz=(0.3, 0.4, 0.4),
    )
    spheres = cuboid_covering_spheres(payload, 0.1)
    assert len(spheres) == 3 * 4 * 4
    assert all(math.isclose(sphere.radius, math.sqrt(3.0) * 0.05) for sphere in spheres)
    lower = np.min([sphere.center for sphere in spheres], axis=0)
    upper = np.max([sphere.center for sphere in spheres], axis=0)
    assert np.allclose(lower, [-0.1, -0.15, 0.05])
    assert np.allclose(upper, [0.1, 0.15, 0.35])


def test_transform_spheres_uses_link_frame():
    transform = np.eye(4)
    transform[:3, :3] = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    transform[:3, 3] = [1.0, 2.0, 3.0]
    result = transform_spheres((Sphere("tool", (1.0, 0.0, 0.0), 0.2),), {"tool": transform})
    assert np.allclose(result[0].center, [1.0, 3.0, 3.0])
    assert result[0].radius == 0.2
