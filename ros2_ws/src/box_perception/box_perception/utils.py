"""Utility helpers for box_perception."""

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import RegionOfInterest


@dataclass
class InstanceResult:
    """Single YOLO instance segmentation result."""
    box_id: int
    mask: np.ndarray          # (H, W) bool
    bbox: tuple               # (x1, y1, x2, y2) int
    confidence: float


@dataclass
class FaceResult:
    """Plane fitting result for one box."""
    normals: List[np.ndarray] = field(default_factory=list)          # up to 2 vertical face normals, each (3,)
    inlier_counts: List[int] = field(default_factory=list)
    nearest_face_center: np.ndarray = field(default_factory=lambda: np.zeros(3))  # (3,)
    nearest_face_normal: np.ndarray = field(default_factory=lambda: np.zeros(3))  # (3,)


def to_point_msg(arr: np.ndarray) -> Point:
    """Convert (3,) array to geometry_msgs/Point."""
    p = Point()
    p.x, p.y, p.z = float(arr[0]), float(arr[1]), float(arr[2])
    return p


def to_vector3_msg(arr: np.ndarray) -> Vector3:
    """Convert (3,) array to geometry_msgs/Vector3."""
    v = Vector3()
    v.x, v.y, v.z = float(arr[0]), float(arr[1]), float(arr[2])
    return v


def to_roi_msg(bbox: tuple) -> RegionOfInterest:
    """Convert (x1, y1, x2, y2) to sensor_msgs/RegionOfInterest."""
    roi = RegionOfInterest()
    roi.x_offset = int(bbox[0])
    roi.y_offset = int(bbox[1])
    roi.width = int(bbox[2] - bbox[0])
    roi.height = int(bbox[3] - bbox[1])
    roi.do_rectify = False
    return roi
