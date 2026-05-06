"""Unit tests for FrustumFilter."""

import numpy as np
import pytest
from box_perception.frustum_filter import FrustumFilter


@pytest.fixture
def filt():
    return FrustumFilter(
        fx=386.84, fy=386.42, cx=320.85, cy=240.23,
        image_width=640, image_height=480, cam_x_offset=0.05,
    )


def test_empty_input(filt):
    pts = np.zeros((0, 3), dtype=np.float32)
    out = filt.filter(pts)
    assert out.shape == (0, 3)


def test_behind_camera_rejected(filt):
    """Points with x <= cam_x_offset should be rejected."""
    pts = np.array([
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.04, 0.0, 0.0],
    ], dtype=np.float32)
    out = filt.filter(pts)
    assert out.shape[0] == 0


def test_center_point_kept(filt):
    """A point on the optical axis should project to image center and be kept."""
    # x=2.0, y=0, z=0 -> xn=0, yn=0 -> u=cx, v=cy (inside image)
    pts = np.array([[2.0, 0.0, 0.0]], dtype=np.float32)
    out = filt.filter(pts)
    assert out.shape[0] == 1
    np.testing.assert_array_almost_equal(out[0], [2.0, 0.0, 0.0])


def test_far_off_axis_rejected(filt):
    """A point far to the side should project outside the image."""
    # x=1.0, y=10.0 -> xn = -10/0.95 ~ -10.5 -> u = 386.84*(-10.5)+320.85 << 0
    pts = np.array([[1.0, 10.0, 0.0]], dtype=np.float32)
    out = filt.filter(pts)
    assert out.shape[0] == 0


def test_mixed_points(filt):
    """Mix of valid and invalid points."""
    pts = np.array([
        [2.0, 0.0, 0.0],     # center, should pass
        [-1.0, 0.0, 0.0],    # behind, rejected
        [1.0, 0.0, 0.0],     # on axis, should pass
        [1.0, 10.0, 10.0],   # far off, rejected
    ], dtype=np.float32)
    out = filt.filter(pts)
    assert out.shape[0] == 2
    np.testing.assert_array_almost_equal(out[0], [2.0, 0.0, 0.0])
    np.testing.assert_array_almost_equal(out[1], [1.0, 0.0, 0.0])


def test_project_consistency(filt):
    """project() output should match the filter's internal projection."""
    pts = np.array([[2.0, 0.0, 0.0], [1.5, -0.1, -0.05]], dtype=np.float32)
    uv = filt.project(pts)
    assert uv.shape == (2, 2)
    # First point: xn=0, yn=0 -> u=cx, v=cy
    np.testing.assert_almost_equal(uv[0, 0], filt.cx, decimal=1)
    np.testing.assert_almost_equal(uv[0, 1], filt.cy, decimal=1)


def test_vectorized_no_loop(filt):
    """Large input should work efficiently (vectorized)."""
    rng = np.random.RandomState(42)
    pts = rng.uniform(0.1, 5.0, size=(100000, 3)).astype(np.float32)
    pts[:, 1] *= 0.5  # keep y moderate
    pts[:, 2] *= 0.5
    out = filt.filter(pts)
    assert out.shape[0] > 0
    assert out.shape[1] == 3
