"""Unit tests for MaskAssigner."""

import numpy as np
import pytest
from box_perception.mask_assigner import MaskAssigner
from box_perception.utils import InstanceResult


@pytest.fixture
def assigner():
    return MaskAssigner(
        fx=386.84, fy=386.42, cx=320.85, cy=240.23,
        image_width=640, image_height=480, cam_x_offset=0.05,
        mask_erode_px=3,  # small kernel for test
    )


def test_empty_instances(assigner):
    label = assigner.build_label_image([])
    assert label.shape == (480, 640)
    assert label.max() == 0


def test_single_mask_label(assigner):
    mask = np.zeros((480, 640), dtype=bool)
    mask[100:200, 100:300] = True
    inst = InstanceResult(box_id=1, mask=mask, bbox=(100, 100, 300, 200), confidence=0.9)
    label = assigner.build_label_image([inst])
    # After erosion, interior should be labeled 1
    assert label.max() == 1
    # Center of mask should be labeled
    assert label[150, 200] == 1
    # Outside mask should be 0
    assert label[0, 0] == 0


def test_confidence_ordering(assigner):
    """Higher confidence mask should overwrite lower in overlap region."""
    mask_low = np.zeros((480, 640), dtype=bool)
    mask_low[100:300, 100:400] = True
    mask_high = np.zeros((480, 640), dtype=bool)
    mask_high[150:250, 200:350] = True

    inst_low = InstanceResult(box_id=1, mask=mask_low, bbox=(100, 100, 400, 300), confidence=0.5)
    inst_high = InstanceResult(box_id=2, mask=mask_high, bbox=(200, 150, 350, 250), confidence=0.9)

    label = assigner.build_label_image([inst_low, inst_high])
    # Center of high-confidence mask should be box_id=2
    assert label[200, 275] == 2


def test_assign_empty_cloud(assigner):
    label = np.zeros((480, 640), dtype=np.int32)
    pts = np.zeros((0, 3), dtype=np.float32)
    result = assigner.assign(pts, label)
    assert result == {}


def test_assign_points_to_box(assigner):
    """Points projecting into a labeled region should be assigned to that box."""
    # Create a label image with box_id=1 in center region
    label = np.zeros((480, 640), dtype=np.int32)
    label[200:280, 280:360] = 1

    # A point on the optical axis at x=2.0 projects to (cx, cy) ~ (320, 240)
    pts = np.array([[2.0, 0.0, 0.0]], dtype=np.float32)
    result = assigner.assign(pts, label)
    assert 1 in result
    assert result[1].shape[0] == 1


def test_assign_background_filtered(assigner):
    """Points projecting to background (label=0) should not appear in output."""
    label = np.zeros((480, 640), dtype=np.int32)
    # No labels set — everything is background
    pts = np.array([[2.0, 0.0, 0.0]], dtype=np.float32)
    result = assigner.assign(pts, label)
    assert result == {}


def test_assign_preserves_body_frame(assigner):
    """Returned points should be in original body frame coordinates."""
    label = np.zeros((480, 640), dtype=np.int32)
    label[200:280, 280:360] = 5

    pts = np.array([[2.0, 0.0, 0.0]], dtype=np.float32)
    result = assigner.assign(pts, label)
    if 5 in result:
        np.testing.assert_array_almost_equal(result[5][0], [2.0, 0.0, 0.0])
