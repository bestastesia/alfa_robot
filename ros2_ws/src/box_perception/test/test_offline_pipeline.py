#!/usr/bin/env python3
"""Offline pipeline test: read one frame from rosbag, run full pipeline, print results.

Usage:
    python3 test/test_offline_pipeline.py
"""

import sys
import os
import numpy as np

# Add package to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

BAG_PATH = os.path.join(
    os.path.dirname(__file__), '..',
    'bags/RGB_lidar/synced_data_record_2026-03-10-01-24-09_ros2'
)
MODEL_PATH = os.path.join(os.path.dirname(__file__), '..', 'models/box_seg.pt')


def read_one_frame(bag_path):
    """Read one image + pointcloud frame from a ROS2 bag using rosbags."""
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import get_typestore, Stores

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    rgb_msg = None
    pcd_msg = None

    with Reader(bag_path) as reader:
        for conn, timestamp, rawdata in reader.messages():
            if conn.topic == '/rgb' and rgb_msg is None:
                rgb_msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            elif conn.topic == '/pcd_for_cam' and pcd_msg is None:
                pcd_msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            if rgb_msg is not None and pcd_msg is not None:
                break

    return rgb_msg, pcd_msg


def image_msg_to_cv2(msg):
    """Convert a sensor_msgs/Image to numpy BGR array."""
    import cv2
    h, w = msg.height, msg.width
    encoding = msg.encoding

    if encoding in ('rgb8', 'RGB8'):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif encoding in ('bgr8', 'BGR8'):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    elif encoding in ('mono8',):
        gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    elif encoding in ('8UC3',):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    else:
        # Try raw reshape
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, -1)
        if img.shape[2] == 3:
            return img
        return img[:, :, :3]


def pointcloud2_to_xyz(msg):
    """Convert PointCloud2 message to (N, 3) float32 array."""
    point_step = msg.point_step
    data = bytes(msg.data)
    n = msg.width * msg.height

    # Find field offsets
    offsets = {}
    for f in msg.fields:
        if f.name in ('x', 'y', 'z'):
            offsets[f.name] = f.offset

    if not all(k in offsets for k in ('x', 'y', 'z')):
        raise ValueError(f"PointCloud2 missing xyz fields. Available: {[f.name for f in msg.fields]}")

    raw = np.frombuffer(data, dtype=np.uint8)
    if len(raw) < n * point_step:
        n = len(raw) // point_step

    pts = np.zeros((n, 3), dtype=np.float32)
    for j, name in enumerate(('x', 'y', 'z')):
        off = offsets[name]
        # View as float32 at each point's field offset
        pts[:, j] = np.array([
            np.frombuffer(raw[i * point_step + off: i * point_step + off + 4], dtype=np.float32)[0]
            for i in range(n)
        ])

    mask = ~np.isnan(pts).any(axis=1)
    return pts[mask]


def main():
    print("=" * 60)
    print("Offline Pipeline Test")
    print("=" * 60)

    # --- Step 0: Read bag ---
    print("\n[0] Reading one frame from rosbag...")
    try:
        rgb_msg, pcd_msg = read_one_frame(BAG_PATH)
    except ImportError as e:
        print(f"  ERROR: 'rosbags' not installed ({e}). Install with: pip install rosbags")
        sys.exit(1)
    except Exception as e:
        print(f"  ERROR reading bag: {type(e).__name__}: {e}")
        import traceback; traceback.print_exc()
        sys.exit(1)

    if rgb_msg is None or pcd_msg is None:
        print("  ERROR: Could not read rgb or pcd from bag.")
        sys.exit(1)

    rgb = image_msg_to_cv2(rgb_msg)
    cloud = pointcloud2_to_xyz(pcd_msg)
    print(f"  Image: {rgb.shape}, dtype={rgb.dtype}")
    print(f"  Cloud: {cloud.shape}, dtype={cloud.dtype}")
    print(f"  Cloud x range: [{cloud[:,0].min():.3f}, {cloud[:,0].max():.3f}]")
    print(f"  Cloud y range: [{cloud[:,1].min():.3f}, {cloud[:,1].max():.3f}]")
    print(f"  Cloud z range: [{cloud[:,2].min():.3f}, {cloud[:,2].max():.3f}]")

    # --- Step 1: FOV filter ---
    print("\n[1] FOV frustum filter...")
    from box_perception.frustum_filter import FrustumFilter
    frustum = FrustumFilter()
    cloud_fov = frustum.filter(cloud)
    print(f"  Before: {cloud.shape[0]} pts -> After: {cloud_fov.shape[0]} pts")

    # --- Step 2: YOLO segmentation ---
    print("\n[2] YOLO segmentation...")
    from box_perception.segmentor import Segmentor
    seg = Segmentor(model_path=MODEL_PATH, conf_threshold=0.5,
                    image_height=rgb.shape[0], image_width=rgb.shape[1])
    if seg.model is not None:
        print("  Real YOLO model loaded OK")
    else:
        print("  WARNING: Using stub (model not loaded)")
    instances = seg.infer(rgb)
    print(f"  Detected {len(instances)} instance(s)")
    for inst in instances:
        print(f"    box_id={inst.box_id}, conf={inst.confidence:.3f}, "
              f"bbox={inst.bbox}, mask_pixels={inst.mask.sum()}")

    # --- Step 3: Mask assignment ---
    print("\n[3] Mask -> point cloud assignment...")
    from box_perception.mask_assigner import MaskAssigner
    assigner = MaskAssigner()
    label_image = assigner.build_label_image(instances)
    box_clouds = assigner.assign(cloud_fov, label_image)
    print(f"  Assigned to {len(box_clouds)} box(es)")
    for bid, pts in box_clouds.items():
        print(f"    box_id={bid}: {pts.shape[0]} points")

    # --- Step 4: Face fitting ---
    print("\n[4] Parallel face fitting...")
    from box_perception.face_fitter import FaceFitter
    fitter = FaceFitter(max_workers=4)
    face_results = fitter.fit_all(box_clouds)

    any_result = False
    for bid, face in face_results.items():
        if face is None:
            print(f"    box_id={bid}: fit failed (None)")
            continue
        any_result = True
        print(f"    box_id={bid}:")
        print(f"      normals: {len(face.normals)}")
        for i, n in enumerate(face.normals):
            print(f"        face_{i}: normal=[{n[0]:.4f}, {n[1]:.4f}, {n[2]:.4f}], "
                  f"inliers={face.inlier_counts[i]}")
        c = face.nearest_face_center
        n = face.nearest_face_normal
        print(f"      nearest_face_center: [{c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}]")
        print(f"      nearest_face_normal: [{n[0]:.4f}, {n[1]:.4f}, {n[2]:.4f}]")

    # --- Summary ---
    print("\n" + "=" * 60)
    if any_result:
        print("PASS: Pipeline produced face results.")
    elif len(instances) > 0 and len(box_clouds) > 0:
        print("WARN: Had detections and points but face fitting returned None for all.")
    elif len(instances) > 0:
        print("WARN: Had detections but no points assigned to any box.")
    else:
        print("WARN: No detections from YOLO.")
    print("=" * 60)

    # --- Save debug image ---
    import cv2
    debug_img = rgb.copy()
    for inst in instances:
        x1, y1, x2, y2 = inst.bbox
        cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(debug_img, f'id={inst.box_id} {inst.confidence:.2f}',
                    (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # Draw mask overlay
        overlay = debug_img.copy()
        overlay[inst.mask] = (0, 128, 255)
        debug_img = cv2.addWeighted(debug_img, 0.7, overlay, 0.3, 0)

    out_path = os.path.join(os.path.dirname(__file__), '..', 'debug_output.png')
    cv2.imwrite(out_path, debug_img)
    print(f"\nDebug image saved to: {out_path}")


if __name__ == '__main__':
    main()
