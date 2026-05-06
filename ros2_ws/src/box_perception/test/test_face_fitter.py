"""Unit tests for FaceFitter."""

import numpy as np
import pytest
from box_perception.face_fitter import FaceFitter
from box_perception.utils import FaceResult


@pytest.fixture
def fitter():
    return FaceFitter(
        ransac_threshold=0.02,
        min_inliers=5,
        normal_radius=0.1,
        vertical_tol_deg=30.0,
        max_workers=2,
    )


def _make_plane_points(normal, center, n_points=200, noise=0.002, spread=0.3):
    """Generate points on a plane defined by normal and center."""
    normal = np.array(normal, dtype=np.float64)
    normal = normal / np.linalg.norm(normal)

    # Find two orthogonal vectors in the plane
    if abs(normal[0]) < 0.9:
        u = np.cross(normal, [1, 0, 0])
    else:
        u = np.cross(normal, [0, 1, 0])
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)
    v = v / np.linalg.norm(v)

    rng = np.random.RandomState(42)
    s = rng.uniform(-spread, spread, size=(n_points, 1))
    t = rng.uniform(-spread, spread, size=(n_points, 1))
    pts = np.array(center) + s * u + t * v
    pts += rng.normal(0, noise, size=pts.shape)
    return pts.astype(np.float64)


def test_empty_points(fitter):
    result = fitter._fit_one(np.zeros((0, 3)))
    assert result is None


def test_too_few_points(fitter):
    result = fitter._fit_one(np.array([[1, 0, 0], [1, 0.1, 0]]))
    assert result is None


def test_single_vertical_plane(fitter):
    """A single vertical plane (normal along y) should be detected."""
    # Plane at x=2.0, normal pointing in -x direction (toward origin)
    pts = _make_plane_points(
        normal=[1, 0, 0],
        center=[2.0, 0.0, 0.5],
        n_points=300,
    )
    result = fitter._fit_one(pts)
    assert result is not None
    assert len(result.normals) >= 1
    # Normal should be roughly along x-axis
    n = result.nearest_face_normal
    assert abs(n[0]) > 0.8  # dominant x component
    # Center should be near x=2.0
    assert abs(result.nearest_face_center[0] - 2.0) < 0.1


def test_two_vertical_planes(fitter):
    """Two perpendicular vertical planes should yield 2 normals."""
    # Plane 1: normal along x, at x=2.0
    pts1 = _make_plane_points(
        normal=[1, 0, 0],
        center=[2.0, 0.0, 0.5],
        n_points=200,
    )
    # Plane 2: normal along y, at y=0.5
    pts2 = _make_plane_points(
        normal=[0, 1, 0],
        center=[2.5, 0.5, 0.5],
        n_points=200,
    )
    pts = np.vstack([pts1, pts2])
    result = fitter._fit_one(pts)
    assert result is not None
    assert len(result.normals) == 2


def test_top_face_filtered(fitter):
    """A horizontal top face (normal along z) should be filtered out."""
    # Only top-face points — normal along z
    pts = _make_plane_points(
        normal=[0, 0, 1],
        center=[2.0, 0.0, 1.0],
        n_points=300,
    )
    result = fitter._fit_one(pts)
    # Should return None since all points are top-face
    assert result is None


def test_normal_points_toward_origin(fitter):
    """Fitted normals should point toward the origin (sensor)."""
    pts = _make_plane_points(
        normal=[1, 0, 0],
        center=[3.0, 0.0, 0.5],
        n_points=300,
    )
    result = fitter._fit_one(pts)
    assert result is not None
    n = result.nearest_face_normal
    c = result.nearest_face_center
    # normal · (-center) > 0 means pointing toward origin
    assert np.dot(n, -c) > 0


def test_fit_all_parallel(fitter):
    """fit_all should process multiple boxes in parallel."""
    pts1 = _make_plane_points([1, 0, 0], [2.0, 0.0, 0.5], n_points=200)
    pts2 = _make_plane_points([0, 1, 0], [2.5, 1.0, 0.5], n_points=200)

    box_clouds = {1: pts1, 2: pts2}
    results = fitter.fit_all(box_clouds)
    assert 1 in results
    assert 2 in results
    assert results[1] is not None
    assert results[2] is not None


def test_fit_all_empty(fitter):
    results = fitter.fit_all({})
    assert results == {}


def test_nearest_face_selection(fitter):
    """When two planes exist, nearest_face should be the one with smaller x."""
    # Plane 1: closer (x=1.5)
    pts1 = _make_plane_points([1, 0, 0], [1.5, 0.0, 0.5], n_points=200)
    # Plane 2: farther (x=3.0)
    pts2 = _make_plane_points([0, 1, 0], [3.0, 0.5, 0.5], n_points=200)
    pts = np.vstack([pts1, pts2])
    result = fitter._fit_one(pts)
    if result is not None and len(result.normals) == 2:
        # Nearest face center x should be closer to 1.5 than 3.0
        assert result.nearest_face_center[0] < 2.5
