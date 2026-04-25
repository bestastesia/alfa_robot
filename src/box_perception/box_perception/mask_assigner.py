"""Mask-to-pointcloud assignment via label_image lookup.

Builds a label image from YOLO instance masks, then assigns each FOV-filtered
LiDAR point to a box_id by projecting into pixel space and looking up the label.
All operations are numpy-vectorized.
"""

import cv2
import numpy as np
from typing import Dict, List

from box_perception.utils import InstanceResult


class MaskAssigner:
    """Assign LiDAR points to box instances using projected label image lookup."""

    def __init__(
        self,
        fx: float = 386.8415832519531,
        fy: float = 386.4196472167969,
        cx: float = 320.8486328125,
        cy: float = 240.2317657470703,
        image_width: int = 640,
        image_height: int = 480,
        cam_x_offset: float = 0.05,
        mask_erode_px: int = 8,
    ):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.w = image_width
        self.h = image_height
        self.cam_x_offset = cam_x_offset
        self.mask_erode_px = mask_erode_px

    def build_label_image(self, instances: List[InstanceResult]) -> np.ndarray:
        """Build a (H, W) int32 label image from instance masks.

        Masks are written in ascending confidence order so higher-confidence
        instances overwrite lower ones in overlapping regions. Each mask is
        eroded before writing to shrink overlap boundaries.

        Uses the actual mask dimensions from YOLO output, not the configured
        image size, so it works regardless of camera resolution.

        Args:
            instances: list of InstanceResult from segmentor.

        Returns:
            (H, W) int32 array. 0 = background, >0 = box_id.
        """
        if not instances:
            return np.zeros((self.h, self.w), dtype=np.int32)

        # Use actual mask dimensions from YOLO output
        h_actual, w_actual = instances[0].mask.shape[:2]
        label_image = np.zeros((h_actual, w_actual), dtype=np.int32)

        # Sort by confidence ascending — high confidence written last (overwrites)
        sorted_inst = sorted(instances, key=lambda inst: inst.confidence)

        kernel = np.ones((self.mask_erode_px, self.mask_erode_px), dtype=np.uint8)

        for inst in sorted_inst:
            mask_u8 = inst.mask.astype(np.uint8) * 255
            eroded = cv2.erode(mask_u8, kernel, iterations=1)
            label_image[eroded > 0] = inst.box_id

        return label_image

    def assign(
        self,
        points: np.ndarray,
        label_image: np.ndarray,
    ) -> Dict[int, np.ndarray]:
        """Assign FOV-filtered points to box instances via label_image lookup.

        Points are projected to pixel coordinates using the body-frame axis
        swap convention, then each point's box_id is read from label_image.
        Projection is scaled to match the actual label_image dimensions.

        Args:
            points: (M, 3) float array in body/lidar frame, already FOV-filtered.
            label_image: (H, W) int32 from build_label_image.

        Returns:
            Dict mapping box_id -> (K, 3) point cloud subset in body frame.
        """
        if points.shape[0] == 0:
            return {}

        lh, lw = label_image.shape[:2]
        scale_x = lw / self.w
        scale_y = lh / self.h

        x = points[:, 0]

        # Must be in front of camera
        valid = x > self.cam_x_offset
        pts_valid = points[valid]
        denom = pts_valid[:, 0] - self.cam_x_offset

        # Body frame -> pixel coords, scaled to actual image size
        xn = -pts_valid[:, 1] / denom
        yn = -pts_valid[:, 2] / denom
        u = (self.fx * xn + self.cx) * scale_x
        v = (self.fy * yn + self.cy) * scale_y

        u_int = np.round(u).astype(np.int32)
        v_int = np.round(v).astype(np.int32)

        # Bounds check against actual label_image size
        in_bounds = (u_int >= 0) & (u_int < lw) & (v_int >= 0) & (v_int < lh)
        pts_bounded = pts_valid[in_bounds]
        u_bounded = u_int[in_bounds]
        v_bounded = v_int[in_bounds]

        # Vectorized label lookup
        labels = label_image[v_bounded, u_bounded]

        # Filter background (label == 0)
        fg_mask = labels > 0
        pts_fg = pts_bounded[fg_mask]
        labels_fg = labels[fg_mask]

        # Group by box_id
        result: Dict[int, np.ndarray] = {}
        unique_ids = np.unique(labels_fg)
        for bid in unique_ids:
            result[int(bid)] = pts_fg[labels_fg == bid]

        return result
