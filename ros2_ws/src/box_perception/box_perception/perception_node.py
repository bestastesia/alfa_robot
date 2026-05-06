#!/usr/bin/env python3
"""Main ROS2 perception node: orchestrates the full pipeline.

Pipeline: time sync -> FOV filter -> YOLO segment -> mask assign -> face fit -> publish
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from box_perception.frustum_filter import FrustumFilter
from box_perception.segmentor import Segmentor
from box_perception.mask_assigner import MaskAssigner
from box_perception.face_fitter import FaceFitter
from box_perception.utils import (
    to_point_msg, to_vector3_msg, to_roi_msg, FaceResult,
)

# Import custom messages from the separate msgs package
from box_perception_msgs.msg import BoxResult, BoxPerceptionResult


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Convert PointCloud2 to (N,3) float32 array. Handles both structured and flat layouts."""
    try:
        from sensor_msgs_py import point_cloud2 as pc2
        points_gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.array(list(points_gen), dtype=np.float32)
        if pts.ndim == 2 and pts.shape[1] >= 3:
            return pts[:, :3]
        return pts.reshape(-1, 3) if pts.size > 0 else np.zeros((0, 3), dtype=np.float32)
    except Exception:
        pass
    # Fallback: raw byte parsing
    import struct
    point_step = msg.point_step
    data = np.frombuffer(msg.data, dtype=np.uint8)
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32)
    # Find xyz field offsets
    offsets = {}
    for f in msg.fields:
        if f.name in ('x', 'y', 'z'):
            offsets[f.name] = f.offset
    pts = np.zeros((n, 3), dtype=np.float32)
    for i in range(n):
        base = i * point_step
        for j, name in enumerate(('x', 'y', 'z')):
            off = offsets[name]
            pts[i, j] = struct.unpack_from('f', data, base + off)[0]
    mask = ~np.isnan(pts).any(axis=1)
    return pts[mask]


class PerceptionNode(Node):
    """Box perception main node."""

    def __init__(self):
        super().__init__('box_perception_node')

        # Declare all parameters with defaults
        self.declare_parameter('cloud_topic', '/cloud_registered_body')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('fx', 386.8415832519531)
        self.declare_parameter('fy', 386.4196472167969)
        self.declare_parameter('cx', 320.8486328125)
        self.declare_parameter('cy', 240.2317657470703)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('cam_x_offset', 0.05)
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('mask_erode_px', 8)
        self.declare_parameter('ransac_threshold', 0.015)
        self.declare_parameter('min_inliers', 10)
        self.declare_parameter('normal_radius', 0.05)
        self.declare_parameter('normal_knn', 20)
        self.declare_parameter('vertical_tol_deg', 30.0)
        self.declare_parameter('max_workers', 4)
        self.declare_parameter('debug_publish', True)
        self.declare_parameter('accumulate_frames', 5)

        # Read parameters
        p = lambda name: self.get_parameter(name).value
        fx = p('fx')
        fy = p('fy')
        cx = p('cx')
        cy = p('cy')
        w = p('image_width')
        h = p('image_height')
        cam_x_offset = p('cam_x_offset')

        # Resolve model_path: if relative, resolve from package share directory
        model_path = p('model_path')
        if model_path and not os.path.isabs(model_path):
            pkg_share = get_package_share_directory('box_perception')
            model_path = os.path.join(pkg_share, model_path)

        # Initialize pipeline components
        self.frustum = FrustumFilter(
            fx=fx, fy=fy, cx=cx, cy=cy,
            image_width=w, image_height=h,
            cam_x_offset=cam_x_offset,
        )
        self.segmentor = Segmentor(
            model_path=model_path,
            conf_threshold=p('conf_threshold'),
            image_height=h,
            image_width=w,
        )
        self.assigner = MaskAssigner(
            fx=fx, fy=fy, cx=cx, cy=cy,
            image_width=w, image_height=h,
            cam_x_offset=cam_x_offset,
            mask_erode_px=p('mask_erode_px'),
        )
        self.fitter = FaceFitter(
            ransac_threshold=p('ransac_threshold'),
            min_inliers=p('min_inliers'),
            normal_radius=p('normal_radius'),
            normal_knn=p('normal_knn'),
            vertical_tol_deg=p('vertical_tol_deg'),
            max_workers=p('max_workers'),
        )

        self.bridge = CvBridge()
        self.frame_id = 0
        self.debug_publish = p('debug_publish')
        self.accumulate_frames = p('accumulate_frames')
        self.cloud_buffer = []  # buffer for accumulating point clouds
        self.latest_image = None  # cache latest image

        # Publishers
        self.pub_result = self.create_publisher(
            BoxPerceptionResult, '/box_perception/result', 10)
        self.pub_debug_img = self.create_publisher(
            Image, '/box_perception/debug/image', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/box_perception/debug/markers', 10)
        self.pub_odom1 = self.create_publisher(
            Odometry, '/target_odometry1', 10)
        self.pub_odom2 = self.create_publisher(
            Odometry, '/target_odometry2', 10)

        # Subscribers: cache image, trigger pipeline on pointcloud
        cloud_topic = p('cloud_topic')
        image_topic = p('image_topic')
        self.create_subscription(Image, image_topic, self._image_cb, 10)
        self.create_subscription(PointCloud2, cloud_topic, self._cloud_cb, 10)

        self.get_logger().info(
            f'PerceptionNode started. cloud={cloud_topic} image={image_topic}')

    def _image_cb(self, msg: Image):
        """Cache the latest image."""
        self.latest_image = msg

    def _cloud_cb(self, pc_msg: PointCloud2):
        """Sliding window: keep latest N clouds, trigger pipeline every frame once full."""
        if self.latest_image is None:
            self.get_logger().warn('No image received yet, skipping', throttle_duration_sec=2.0)
            return

        cloud = pointcloud2_to_xyz(pc_msg)
        self.cloud_buffer.append(cloud)

        if len(self.cloud_buffer) > self.accumulate_frames:
            self.cloud_buffer.pop(0)

        if len(self.cloud_buffer) < self.accumulate_frames:
            return

        merged_cloud = np.vstack(self.cloud_buffer)
        self.callback(self.latest_image, merged_cloud)

    def callback(self, img_msg: Image, cloud: np.ndarray):
        """Main pipeline callback triggered by accumulated clouds + latest image."""
        try:
            rgb = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return

        # ② FOV filter
        cloud_fov = self.frustum.filter(cloud)

        # ③ YOLO segmentation
        instances = self.segmentor.infer(rgb)

        # ④ Mask -> point cloud assignment
        label_image = self.assigner.build_label_image(instances)
        box_clouds = self.assigner.assign(cloud_fov, label_image)

        # ⑤ Parallel face fitting
        face_results = self.fitter.fit_all(box_clouds)

        # ⑥ Pack and publish
        result_msg = BoxPerceptionResult()
        result_msg.header = img_msg.header
        result_msg.frame_id = self.frame_id
        self.frame_id += 1

        for inst in instances:
            face = face_results.get(inst.box_id)
            if face is None:
                continue

            box = BoxResult()
            box.header = img_msg.header
            box.box_id = inst.box_id
            box.confidence = float(inst.confidence)
            box.nearest_face_center = to_point_msg(face.nearest_face_center)
            box.nearest_face_normal = to_vector3_msg(face.nearest_face_normal)
            box.bbox = to_roi_msg(inst.bbox)

            if len(face.normals) >= 1:
                box.face_normal_0 = to_vector3_msg(face.normals[0])
                box.face_inlier_count_0 = face.inlier_counts[0]
            if len(face.normals) >= 2:
                box.face_normal_1 = to_vector3_msg(face.normals[1])
                box.face_inlier_count_1 = face.inlier_counts[1]

            result_msg.boxes.append(box)

        self.pub_result.publish(result_msg)

        # Publish Odometry for the nearest 2 boxes (sorted by distance to origin)
        self._publish_odometry(result_msg)

        # Debug visualization
        if self.debug_publish:
            self._publish_debug(rgb, instances, face_results, img_msg.header)

    def _publish_odometry(self, result_msg: BoxPerceptionResult):
        """Publish Odometry for the nearest 2 boxes sorted by distance."""
        if not result_msg.boxes:
            return

        # Sort boxes by Euclidean distance of nearest_face_center to origin
        def _dist(box: BoxResult) -> float:
            c = box.nearest_face_center
            return c.x * c.x + c.y * c.y + c.z * c.z

        sorted_boxes = sorted(result_msg.boxes, key=_dist)

        pubs = [self.pub_odom1, self.pub_odom2]
        for i, pub in enumerate(pubs):
            if i >= len(sorted_boxes):
                break
            box = sorted_boxes[i]
            odom = Odometry()
            odom.header.stamp = result_msg.header.stamp
            odom.header.frame_id = 'body'
            odom.child_frame_id = f'box_{box.box_id}'

            # Position = nearest face center
            odom.pose.pose.position.x = float(box.nearest_face_center.x)
            odom.pose.pose.position.y = float(box.nearest_face_center.y)
            odom.pose.pose.position.z = float(box.nearest_face_center.z)

            # Orientation from face normal: align x-axis with normal
            n = np.array([box.nearest_face_normal.x,
                          box.nearest_face_normal.y,
                          box.nearest_face_normal.z])
            norm = np.linalg.norm(n)
            if norm > 1e-6:
                n = n / norm
                # Rotation from [1,0,0] to n using quaternion
                # q = [cos(theta/2), sin(theta/2)*axis]
                ref = np.array([1.0, 0.0, 0.0])
                dot = np.clip(np.dot(ref, n), -1.0, 1.0)
                if dot < -0.9999:
                    # ~180 deg: pick orthogonal axis
                    q = [0.0, 0.0, 1.0, 0.0]  # x,y,z,w
                elif dot > 0.9999:
                    q = [0.0, 0.0, 0.0, 1.0]
                else:
                    axis = np.cross(ref, n)
                    axis = axis / np.linalg.norm(axis)
                    half_angle = np.arccos(dot) / 2.0
                    s = np.sin(half_angle)
                    q = [axis[0]*s, axis[1]*s, axis[2]*s, np.cos(half_angle)]
            else:
                q = [0.0, 0.0, 0.0, 1.0]

            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]

            pub.publish(odom)

    def _publish_debug(self, rgb, instances, face_results, header):
        """Publish debug image and RViz markers."""
        import cv2

        debug_img = rgb.copy()
        for inst in instances:
            x1, y1, x2, y2 = inst.bbox
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(debug_img, f'id={inst.box_id} {inst.confidence:.2f}',
                        (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = header
            self.pub_debug_img.publish(debug_msg)
        except Exception:
            pass

        # RViz markers for face normals
        marker_array = MarkerArray()
        marker_id = 0
        for bid, face in face_results.items():
            if face is None:
                continue
            center = face.nearest_face_center
            normal = face.nearest_face_normal

            marker = Marker()
            marker.header = header
            marker.ns = 'face_normals'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            from geometry_msgs.msg import Point as GPoint
            start = GPoint(x=float(center[0]), y=float(center[1]), z=float(center[2]))
            end = GPoint(
                x=float(center[0] + normal[0] * 0.15),
                y=float(center[1] + normal[1] * 0.15),
                z=float(center[2] + normal[2] * 0.15),
            )
            marker.points = [start, end]
            marker.scale.x = 0.01
            marker.scale.y = 0.02
            marker.scale.z = 0.0
            marker.color.r = 1.0
            marker.color.a = 1.0
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 500000000

            marker_array.markers.append(marker)

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
