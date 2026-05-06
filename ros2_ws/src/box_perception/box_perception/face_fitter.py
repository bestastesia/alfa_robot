"""Parallel face fitting: normal estimation, K-means clustering, RANSAC plane fitting.

Each box's point cloud is processed independently in a thread pool.
Open3D's C++ backend releases the GIL, enabling true parallelism.
"""

from concurrent.futures import ThreadPoolExecutor
from typing import Dict, Optional

import numpy as np
import open3d as o3d
from scipy.cluster.vq import kmeans2

from box_perception.utils import FaceResult


class FaceFitter:
    """Fit vertical face planes for each detected box."""

    def __init__(
        self,
        ransac_threshold: float = 0.015,
        min_inliers: int = 10,
        normal_radius: float = 0.05,
        normal_knn: int = 20,
        vertical_tol_deg: float = 30.0,
        max_workers: int = 4,
    ):
        self.ransac_threshold = ransac_threshold
        self.min_inliers = min_inliers
        self.normal_radius = normal_radius
        self.normal_knn = normal_knn
        self.vertical_tol = np.sin(np.radians(vertical_tol_deg))
        self.max_workers = max_workers

    def fit_all(
        self,
        box_clouds: Dict[int, np.ndarray],
    ) -> Dict[int, Optional[FaceResult]]:
        """Run _fit_one in parallel for each box_id.

        Args:
            box_clouds: Dict mapping box_id -> (K, 3) body-frame points.

        Returns:
            Dict mapping box_id -> FaceResult or None.
        """
        results: Dict[int, Optional[FaceResult]] = {}
        if not box_clouds:
            return results

        with ThreadPoolExecutor(max_workers=self.max_workers) as pool:
            futures = {
                pool.submit(self._fit_one, pts): bid
                for bid, pts in box_clouds.items()
            }
            for future in futures:
                bid = futures[future]
                try:
                    results[bid] = future.result()
                except Exception:
                    results[bid] = None

        return results

    def _fit_one(self, points: np.ndarray) -> Optional[FaceResult]:
        """Process a single box's point cloud.

        Steps:
            1. Estimate normals with Open3D
            2. Filter out top-face points (|normal_z| > vertical_tol)
            3. K-means(k=2) on remaining normals
            4. RANSAC plane fit per cluster
            5. Select nearest face by min face-center x (depth)
            6. Unify normal direction toward origin

        Args:
            points: (K, 3) body-frame point cloud for one box.

        Returns:
            FaceResult or None if insufficient points.
        """
        if points.shape[0] < self.min_inliers:
            return None

        # Step 1: Estimate normals (KNN for robustness with sparse clouds)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamKNN(knn=self.normal_knn)
        )
        normals = np.asarray(pcd.normals)

        # Step 2: Filter top-face points — keep only vertical-face points
        # Vertical face: |normal_z| < sin(vertical_tol_deg)
        vertical_mask = np.abs(normals[:, 2]) < self.vertical_tol
        vert_points = points[vertical_mask]
        vert_normals = normals[vertical_mask]

        if vert_points.shape[0] < self.min_inliers:
            return None

        # Unify normal direction before clustering: flip normals to point toward origin
        # dot(normal, -point_center) > 0 means pointing toward origin
        for i in range(vert_normals.shape[0]):
            if np.dot(vert_normals[i], -vert_points[i]) < 0:
                vert_normals[i] = -vert_normals[i]

        # Step 3: K-means(k=2) on vertical normals
        k = min(2, vert_points.shape[0])
        try:
            centroids, cluster_labels = kmeans2(vert_normals, k, minit='points')
        except Exception:
            return None

        # Step 4: RANSAC plane fit per cluster
        face_normals = []
        face_centers = []
        face_inliers = []

        for ci in range(k):
            cluster_mask = cluster_labels == ci
            cluster_pts = vert_points[cluster_mask]

            if cluster_pts.shape[0] < self.min_inliers:
                continue

            cluster_pcd = o3d.geometry.PointCloud()
            cluster_pcd.points = o3d.utility.Vector3dVector(cluster_pts)

            try:
                plane_model, inlier_idx = cluster_pcd.segment_plane(
                    distance_threshold=self.ransac_threshold,
                    ransac_n=3,
                    num_iterations=1000,
                )
            except Exception:
                continue

            if len(inlier_idx) < self.min_inliers:
                continue

            normal = np.array(plane_model[:3])
            inlier_pts = cluster_pts[inlier_idx]
            center = inlier_pts.mean(axis=0)

            # Step 6: Unify normal direction — should point toward origin
            if np.dot(normal, -center) < 0:
                normal = -normal

            # Normalize
            norm_len = np.linalg.norm(normal)
            if norm_len > 1e-6:
                normal = normal / norm_len

            face_normals.append(normal)
            face_centers.append(center)
            face_inliers.append(len(inlier_idx))

        if not face_normals:
            return None

        # Step 5: Nearest face = smallest x (depth in body frame)
        centers_x = np.array([c[0] for c in face_centers])
        nearest_idx = int(np.argmin(centers_x))

        return FaceResult(
            normals=face_normals,
            inlier_counts=face_inliers,
            nearest_face_center=face_centers[nearest_idx],
            nearest_face_normal=face_normals[nearest_idx],
        )
