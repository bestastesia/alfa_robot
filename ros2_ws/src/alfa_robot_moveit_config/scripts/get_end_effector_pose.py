#!/usr/bin/env python3
"""
读取双臂末端位姿的简单脚本

方法：
1. 使用 TF（最直接）
2. 使用 GetPlanningScene 服务获取机器人状态

使用方法：
  python3 get_end_effector_pose.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPlanningScene
from sensor_msgs.msg import JointState
import tf2_ros
import math


class EndEffectorPoseReader(Node):
    """末端位姿读取节点"""

    def __init__(self):
        super().__init__('ee_pose_reader')

        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # GetPlanningScene 服务客户端
        self.scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')

        # 关节状态订阅
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10
        )

        self.get_logger().info("末端位姿读取器已启动")

    def _joint_state_cb(self, msg):
        self.joint_state = msg

    def get_pose_via_tf(self, link_name: str, base_frame: str = "base_link") -> Pose:
        """通过 TF 获取末端位姿"""
        try:
            transform = self.tf_buffer.lookup_transform(
                base_frame, link_name,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=2.0)
            )
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f"TF 获取 {link_name} 失败: {e}")
            return None

    def get_robot_state_from_moveit(self):
        """从 MoveIt 获取当前机器人状态"""
        if not self.scene_client.service_is_ready():
            self.get_logger().error("GetPlanningScene 服务不可用")
            return None

        request = GetPlanningScene.Request()
        request.components.components = 2  # ROBOT_STATE

        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if not future.done():
            self.get_logger().error("服务调用超时")
            return None

        result = future.result()
        if result is None:
            return None

        return result.scene.robot_state

    def quaternion_to_rpy(self, pose: Pose) -> tuple:
        """四元数转 RPY（度）"""
        q = pose.orientation
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    def print_pose(self, name: str, pose: Pose):
        """打印位姿信息"""
        if pose is None:
            self.get_logger().info(f"{name}: 位姿获取失败")
            return

        rpy = self.quaternion_to_rpy(pose)
        self.get_logger().info(f"{name}:")
        self.get_logger().info(f"  位置 (m): x={pose.position.x:.4f}, y={pose.position.y:.4f}, z={pose.position.z:.4f}")
        self.get_logger().info(f"  姿态 (°): R={rpy[0]:.2f}, P={rpy[1]:.2f}, Y={rpy[2]:.2f}")
        self.get_logger().info(f"  四元数: w={pose.orientation.w:.4f}, x={pose.orientation.x:.4f}, y={pose.orientation.y:.4f}, z={pose.orientation.z:.4f}")

    def run(self):
        """读取并打印末端位姿"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("读取双臂末端位姿")
        self.get_logger().info("=" * 60)

        # 等待关节状态
        import time
        start = time.time()
        while self.joint_state is None and (time.time() - start) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.joint_state:
            self.get_logger().info(f"已接收到 {len(self.joint_state.name)} 个关节的状态")
            self.get_logger().info(f"关节名称: {self.joint_state.name}")

        # 方法 1: 通过 TF 获取末端位姿
        self.get_logger().info("\n--- 方法 1: TF ---")

        # 尝试不同可能的末端链接名称
        leftjoint6s = ["leftjoint5", "leftjoint6", "leftjoint6"]
        rightjoint6s = ["rightjoint4", "rightjoint6"]

        left_pose = None
        left_link_used = None
        for link in leftjoint6s:
            left_pose = self.get_pose_via_tf(link)
            if left_pose:
                left_link_used = link
                break

        right_pose = None
        right_link_used = None
        for link in rightjoint6s:
            right_pose = self.get_pose_via_tf(link)
            if right_pose:
                right_link_used = link
                break

        self.print_pose(f"左臂末端 ({left_link_used or '未知'})", left_pose)
        self.print_pose(f"右臂末端 ({right_link_used or '未知'})", right_pose)

        # 方法 2: 从 MoveIt 获取机器人状态
        self.get_logger().info("\n--- 方法 2: MoveIt GetPlanningScene ---")
        robot_state = self.get_robot_state_from_moveit()

        if robot_state:
            js = robot_state.joint_state
            self.get_logger().info(f"MoveIt 机器人状态关节: {js.name}")
            self.get_logger().info(f"关节位置: {[round(v, 4) for v in js.position]}")
        else:
            self.get_logger().warn("MoveIt 机器人状态获取失败")

        self.get_logger().info("=" * 60)


def main():
    rclpy.init()

    try:
        node = EndEffectorPoseReader()
        node.run()
    except Exception as e:
        print(f"错误: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()