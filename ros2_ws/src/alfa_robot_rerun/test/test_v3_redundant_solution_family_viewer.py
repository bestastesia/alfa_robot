import math

from alfa_robot_rerun.v3_redundant_solution_family_viewer import (
    wrapped_joint_delta_degrees,
)


def test_wrapped_joint_delta_uses_short_rotation():
    previous = (math.radians(179.0), 0.0)
    current = (math.radians(-179.0), math.radians(1.0))

    assert abs(wrapped_joint_delta_degrees(previous, current) - 2.0) < 1e-9
