"""FOV frustum filter: keep only LiDAR points that project into the camera image.

All operations are numpy-vectorized — no Python for-loops over points.
"""

import numpy as np


class FrustumFilter:
    """Filter a body-frame point cloud to the camera field of view."""

    def __init__(
        self,
        fx: float = 386.8415832519531,
        fy: float = 386.4196472167969,
        cx: float = 320.8486328125,
        cy: float = 240.2317657470703,
        image_width: int = 640,
        image_height: int = 480,
        cam_x_offset: float = 0.05,
    ):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.w = image_width
        self.h = image_height
        self.cam_x_offset = cam_x_offset

    def filter(self, points: np.ndarray) -> np.ndarray:
        """Filter points to camera FOV.

        Args:
            points: (N, 3) float array in body/lidar frame [x, y, z].

        Returns:
            (M, 3) subset of points that project within image bounds.
        """
        if points.shape[0] == 0:
            return points

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        # Only keep points in front of camera
        depth_mask = x > self.cam_x_offset

        # Avoid division by zero — apply depth mask first
        safe_x = x[depth_mask]
        safe_y = y[depth_mask]
        safe_z = z[depth_mask]
        denom = safe_x - self.cam_x_offset

        # Body frame -> normalized image coords (from reference code)
        xn = -safe_y / denom
        yn = -safe_z / denom

        # Project to pixel coords
        u = self.fx * xn + self.cx
        v = self.fy * yn + self.cy

        # Bounds check
        in_fov = (u >= 0) & (u < self.w) & (v >= 0) & (v < self.h)

        return points[depth_mask][in_fov]

    def project(self, points: np.ndarray) -> np.ndarray:
        """Project body-frame points to pixel coordinates (u, v).

        Args:
            points: (N, 3) float array, must have x > cam_x_offset.

        Returns:
            (N, 2) float array of (u, v) pixel coordinates.
        """
        denom = points[:, 0] - self.cam_x_offset
        xn = -points[:, 1] / denom
        yn = -points[:, 2] / denom
        u = self.fx * xn + self.cx
        v = self.fy * yn + self.cy
        return np.column_stack((u, v))
