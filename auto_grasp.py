#!/usr/bin/env python3
"""
自动抓取流程脚本：
  1. 启动视觉感知管线 (start_pipeline.sh)
  2. 订阅 /box_perception/result，接收第一条检测结果
  3. 将 nearest_face_center + nearest_face_normal 转换为 base 系下的位置 + 四元数
  4. 启动 MoveIt 实机运动到目标位姿
"""

import subprocess
import sys
import os
import signal
import time
import numpy as np

import rclpy
from rclpy.node import Node
from box_perception_msgs.msg import BoxPerceptionResult

# ====== 从 trans.py 移植的转换逻辑 ======

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

    # 位置转换
    P_radar_hom = np.array([radar_pos[0], radar_pos[1], radar_pos[2], 1.0])
    P_base_hom = np.dot(T_radar_to_base, P_radar_hom)
    target_pos = P_base_hom[:3]

    # 法向量转换
    n_radar = np.array(radar_normal)
    n_base = np.dot(R_radar_to_base, n_radar)
    n_norm = np.linalg.norm(n_base)
    if n_norm < 1e-6:
        raise ValueError("法向量模长接近0，请检查输入！")
    x_axis = n_base / n_norm

    # 构建旋转矩阵（y 轴尽量朝天）
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
    [0, 0, 1, 0.727],
    [0, 0, 0, 1],
])


class PerceptionListener(Node):
    """订阅 /box_perception/result，只接收第一条有效结果后退出"""

    def __init__(self):
        super().__init__('auto_grasp_listener')
        self.result = None
        self.sub = self.create_subscription(
            BoxPerceptionResult,
            '/box_perception/result',
            self.callback,
            10,
        )
        self.get_logger().info('等待视觉识别结果...')

    def callback(self, msg: BoxPerceptionResult):
        if self.result is not None:
            return  # 已收到，忽略后续消息

        if len(msg.boxes) == 0:
            self.get_logger().info('收到消息但无检测到箱体，继续等待...')
            return

        box = msg.boxes[0]
        self.result = {
            'center': [box.nearest_face_center.x,
                       box.nearest_face_center.y,
                       box.nearest_face_center.z],
            'normal': [box.nearest_face_normal.x,
                       box.nearest_face_normal.y,
                       box.nearest_face_normal.z],
        }
        self.get_logger().info(
            f'收到识别结果:\n'
            f'  center: {self.result["center"]}\n'
            f'  normal: {self.result["normal"]}'
        )


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # ========== 第 1 步: 启动视觉管线 ==========
    print('=' * 60)
    print('  [1/3] 启动视觉感知管线')
    print('=' * 60)
    pipeline_script = os.path.join(script_dir, 'start_pipeline.sh')
    pipeline_proc = subprocess.Popen(
        ['bash', pipeline_script],
        cwd=script_dir,
        preexec_fn=os.setsid,  # 创建新进程组，方便后续整组终止
    )

    # 等待感知节点完全启动
    print('等待感知管线启动 (20秒)...')
    time.sleep(20)

    # ========== 第 2 步: 接收第一条识别结果 ==========
    print('=' * 60)
    print('  [2/3] 等待接收视觉识别结果')
    print('=' * 60)

    rclpy.init()
    listener = PerceptionListener()

    try:
        while rclpy.ok() and listener.result is None:
            rclpy.spin_once(listener, timeout_sec=1.0)
    except KeyboardInterrupt:
        print('\n用户中断，正在清理...')
        listener.destroy_node()
        rclpy.shutdown()
        os.killpg(os.getpgid(pipeline_proc.pid), signal.SIGINT)
        sys.exit(1)

    listener.destroy_node()
    rclpy.shutdown()

    if listener.result is None:
        print('未收到识别结果，退出。')
        os.killpg(os.getpgid(pipeline_proc.pid), signal.SIGINT)
        sys.exit(1)

    # ========== 坐标转换 ==========
    radar_pos = listener.result['center']
    radar_normal = listener.result['normal']

    pos, quat = radar_to_base_pose(radar_pos, radar_normal, T_RADAR_TO_BASE)
    print('=' * 60)
    print('  坐标转换结果')
    print('=' * 60)
    print(f'  位置 (x, y, z):     {pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}')
    print(f'  四元数 (x, y, z, w): {quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}')

    # 关闭视觉管线（不再需要）
    print('\n关闭视觉感知管线...')
    os.killpg(os.getpgid(pipeline_proc.pid), signal.SIGINT)
    pipeline_proc.wait()
    time.sleep(2)

    # ========== 第 3 步: 启动 MoveIt 实机运动 ==========
    print('=' * 60)
    print('  [3/3] 启动 MoveIt 实机运动')
    print('=' * 60)
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
    print(f'执行命令:\n  {" ".join(launch_cmd)}\n')

    try:
        moveit_proc = subprocess.Popen(launch_cmd, cwd=script_dir)
        moveit_proc.wait()
    except KeyboardInterrupt:
        print('\n用户中断，正在终止 MoveIt...')
        moveit_proc.send_signal(signal.SIGINT)
        moveit_proc.wait()

    print('\n流程完成。')


if __name__ == '__main__':
    main()
