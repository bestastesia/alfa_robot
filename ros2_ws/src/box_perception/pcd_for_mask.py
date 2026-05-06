#!/usr/bin/env python3
"""
ROS1 node: crop LiDAR point cloud to the D455 camera FOV and publish it.

Subscribes to: `/livox_lidar` (sensor_msgs/PointCloud2)
Publishes to: `/lidar_for_mask` (sensor_msgs/PointCloud2)

Assumptions / defaults:
- Image size: 1280x720 (params `~img_width` / `~img_height`)
- The LiDAR frame is already aligned with the camera viewing direction:
  - `x` forward
  - `y` left
  - `z` up

Projection uses `x` as depth, `-y / x` as image-right, and `-z / x` as
image-down, then applies the provided intrinsics and radial/tangential
distortion.
"""

from typing import List, Tuple

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

class LidarCropNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/cloud_registered_body")
        self.output_topic = rospy.get_param("~output_topic", "/lidar_for_mask")
        self.img_width = int(rospy.get_param("~img_width", 640))
        self.img_height = int(rospy.get_param("~img_height", 480))

        self.fx = float(rospy.get_param("~fx", 386.8415832519531))
        self.fy = float(rospy.get_param("~fy", 386.4196472167969))
        self.cx = float(rospy.get_param("~cx", 320.8486328125))
        self.cy = float(rospy.get_param("~cy", 240.2317657470703))
        self.k1 = float(rospy.get_param("~k1", -0.05452472344040871))
        self.k2 = float(rospy.get_param("~k2", 0.06167476251721382))
        self.p1 = float(rospy.get_param("~p1", 0.00026341964257881045))
        self.p2 = float(rospy.get_param("~p2", 0.0012736229691654444))
        self.k3 = float(rospy.get_param("~k3", -0.020062286406755447))

        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self.callback,
            queue_size=1,
        )

        rospy.loginfo(
            "Listening to '%s', publishing cropped cloud to '%s'",
            self.input_topic,
            self.output_topic,
        )

    def callback(self, msg: PointCloud2) -> None:
        kept_points: List[Tuple[float, float, float]] = []
        points_iter = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        for x, y, z in points_iter:
            if x <= 0.0:
                continue

            # LiDAR/body frame to image plane:
            #   depth = x, right = -y, down = -z
            xn = -y / (x-0.05)
            yn = -z / (x-0.05)

            # r2 = xn * xn + yn * yn
            # radial = 1.0 + self.k1 * r2 + self.k2 * (r2 ** 2) + self.k3 * (r2 ** 3)
            # x_dist = xn * radial + 2.0 * self.p1 * xn * yn + self.p2 * (r2 + 2.0 * xn * xn)
            # y_dist = yn * radial + self.p1 * (r2 + 2.0 * yn * yn) + 2.0 * self.p2 * xn * yn

            # u = self.fx * x_dist + self.cx
            # v = self.fy * y_dist + self.cy

            u = self.fx * xn + self.cx
            v = self.fy * yn + self.cy

            if 0.0 <= u < self.img_width and 0.0 <= v < self.img_height:
                kept_points.append((x, y, z))

        new_header = Header()
        new_header.stamp = msg.header.stamp
        new_header.frame_id = msg.header.frame_id

        cloud_out = pc2.create_cloud_xyz32(new_header, kept_points)
        self.pub.publish(cloud_out)


def main() -> None:
    rospy.init_node("pcd_for_mask", anonymous=False)
    LidarCropNode()
    rospy.spin()


if __name__ == "__main__":
    main()
