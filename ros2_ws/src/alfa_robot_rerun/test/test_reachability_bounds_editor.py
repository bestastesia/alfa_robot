import numpy as np

from alfa_robot_rerun.reachability_bounds_editor import (
    ReachabilityBounds,
    axis_values,
    box_edges,
    grid_shape,
    nine_orient_parameter_text,
    preview_points,
)


def fixture_bounds() -> ReachabilityBounds:
    return ReachabilityBounds(
        min_x=0.2,
        max_x=0.4,
        min_y=-0.2,
        max_y=0.0,
        min_z=0.5,
        max_z=0.7,
        step_x=0.1,
        step_y=0.1,
        step_z=0.1,
    )


def test_axis_values_include_regular_grid_endpoints():
    assert np.allclose(axis_values(0.2, 0.4, 0.1), [0.2, 0.3, 0.4])
    assert np.allclose(axis_values(-0.09, 0.98, 0.05)[-2:], [0.96, 0.98])


def test_preview_points_reports_full_grid_and_limits_visual_points():
    bounds = ReachabilityBounds(0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.01, 0.01, 0.01)
    points, total = preview_points(bounds, max_points=1_000)

    assert total == 101**3
    assert len(points) <= 1_000
    assert np.allclose(points.min(axis=0), [0.0, 0.0, 0.0])
    assert np.allclose(points.max(axis=0), [1.0, 1.0, 1.0])


def test_grid_shape_edges_and_test_parameter_text_match_bounds():
    bounds = fixture_bounds()

    assert grid_shape(bounds) == (3, 3, 3)
    assert len(box_edges(bounds)) == 12
    text = nine_orient_parameter_text(bounds, updown=0.45)
    assert "-p side:=right" in text
    assert "-p fixed_updown:=0.450" in text
    assert "-p min_y:=-0.200" in text
    assert "-p step_z:=0.100" in text
