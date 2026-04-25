#!/usr/bin/env python3
"""
ROS2 节点：自动抓取协调器
  1. 订阅 /box_perception/result，接收第一条有效识别结果
  2. 将 nearest_face_center + nearest_face_normal 转换为 base 系下的位置 + 四元数
  3. 通过 subprocess 启动 MoveIt 实机运动 launch
"""

import subprocess
import signal
import numpy as np

import rclpy
from rclpy.node import Node
from box_perception_msgs.msg import BoxPerceptionResult


# ====== 坐标转换逻辑（移植自 trans.py）======

def quaternion_from_matrix(matrix):
    R = np.array(matrix, dtype=np.float64)[:3, :3]
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return np.array([qx, qy, qz, qw])


def radar_to_base_pose(radar_pos, radar_normal, T_radar_to_base):
    R_radar_to_base = T_radar_to_base[:3, :3]

    P_radar_hom = np.array([radar_pos[0], radar_pos[1], radar_pos[2], 1.0])
    P_base_hom = np.dot(T_radar_to_base, P_radar_hom)
    target_pos = P_base_hom[:3]

    n_radar = np.array(radar_normal)
    n_base = np.dot(R_radar_to_base, n_radar)
    n_norm = np.linalg.norm(n_base)
    if n_norm < 1e-6:
        raise ValueError("法向量模长接近0，请检查输入！")
    x_axis = n_base / n_norm

    z_ref = np.array([0.0, 0.0, 1.0])
    z_temp = np.cross(x_axis, z_ref)
    z_temp_norm = np.linalg.norm(z_temp)

    if z_temp_norm < 1e-6:
        y_ref = np.array([0.0, 1.0, 0.0])
        y_axis = y_ref
        z_axis = np.cross(x_axis, y_axis)
    else:
        z_axis = z_temp / z_temp_norm
        y_axis = np.cross(z_axis, x_axis)

    R_base_to_ee = np.column_stack([x_axis, y_axis, z_axis])
    quat_xyzw = quaternion_from_matrix(R_base_to_ee)
    return target_pos, quat_xyzw


# 雷达到 base_link 的变换矩阵（手眼标定结果）
T_RADAR_TO_BASE = np.array([
    [1, 0, 0, 0.217],
    [0, 1, 0, 0.0],
    [0, 0, 1, 0.817],
    [0, 0, 0, 1],
])


class AutoGraspNode(Node):
    def __init__(self):
        super().__init__('auto_grasp_node')
        self._received = False
        self._moveit_proc = None

        self.sub = self.create_subscription(
            BoxPerceptionResult,
            '/box_perception/result',
            self._on_perception,
            10,
        )
        self.get_logger().info('等待视觉识别结果...')

    def _on_perception(self, msg: BoxPerceptionResult):
        if self._received:
            return

        if len(msg.boxes) == 0:
            self.get_logger().info('收到消息但无检测到箱体，继续等待...')
            return

        self._received = True
        # 取消订阅，不再接收后续消息
        self.destroy_subscription(self.sub)

        box = msg.boxes[0]
        center = [box.nearest_face_center.x, box.nearest_face_center.y, box.nearest_face_center.z]
        normal = [-1.0*box.nearest_face_normal.x, -1.0*box.nearest_face_normal.y, -1.0*box.nearest_face_normal.z]
        self.get_logger().info(
            f'收到识别结果:\n'
            f'  center: {center}\n'
            f'  normal: {normal}'
        )

        # 坐标转换
        try:
            pos, quat = radar_to_base_pose(center, normal, T_RADAR_TO_BASE)
        except ValueError as e:
            self.get_logger().error(f'坐标转换失败: {e}')
            return

        self.get_logger().info(
            f'目标位姿 (base系):\n'
            f'  位置:   x={pos[0]:.6f}, y={pos[1]:.6f}, z={pos[2]:.6f}\n'
            f'  四元数: x={quat[0]:.6f}, y={quat[1]:.6f}, z={quat[2]:.6f}, w={quat[3]:.6f}'
        )

        # 启动 MoveIt 实机运动
        self._launch_moveit(pos, quat)

    def _launch_moveit(self, pos, quat):
        launch_cmd = [
            'ros2', 'launch', 'alfa_robot_bringup', 'moveit_real_hardware_test.launch.py',
            f'target_x:={pos[0]:.8f}',
            f'target_y:={pos[1]:.8f}',
            f'target_z:={pos[2]:.8f}',
            f'target_qx:={quat[0]:.8f}',
            f'target_qy:={quat[1]:.8f}',
            f'target_qz:={quat[2]:.8f}',
            f'target_qw:={quat[3]:.8f}',
            'auto_run_test:=true',
        ]
        self.get_logger().info(f'启动 MoveIt:\n  {" ".join(launch_cmd)}')
        self._moveit_proc = subprocess.Popen(launch_cmd)

    def destroy_node(self):
        if self._moveit_proc and self._moveit_proc.poll() is None:
            self.get_logger().info('正在终止 MoveIt 进程...')
            self._moveit_proc.send_signal(signal.SIGINT)
            self._moveit_proc.wait()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutoGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
