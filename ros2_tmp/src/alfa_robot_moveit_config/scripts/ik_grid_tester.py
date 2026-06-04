#!/usr/bin/env python3
"""
IK 数据采集脚本 - 用于定量对比 KDL 和 TRAC-IK 求解器

功能：
1. 在指定 3D 空间范围内生成点阵
2. 遍历每个点调用 IK 服务
3. 记录成功率和解算耗时
4. 保存为 CSV 文件

使用方法：
1. 配置好 TRAC-IK 求解器
2. 运行此脚本保存为 tracik_result.csv
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import csv
import time
import math
from datetime import datetime
import numpy as np


class IKGridTester(Node):
    def __init__(self):
        super().__init__('ik_grid_tester')

        # IK 服务客户端
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        self.get_logger().info("等待 IK 服务...")
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 IK 服务...')

        self.get_logger().info("IK 服务已就绪")

        # 关节状态订阅（用于获取当前状态）
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 等待获取关节状态
        self.get_logger().info("等待关节状态...")
        timeout = 5.0
        start = time.time()
        while self.current_joint_state is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

    def joint_state_callback(self, msg):
        """存储当前关节状态"""
        self.current_joint_state = msg

    def get_home_joint_state(self):
        """
        获取 Home 零位关节状态
        用于作为 IK 求解的初始种子（Seed）
        """
        if self.current_joint_state is None:
            self.get_logger().warn("未收到关节状态，使用默认零位")
            return None

        # 复制当前关节状态，然后设置为零位
        home_state = JointState()
        home_state.header = self.current_joint_state.header
        home_state.name = list(self.current_joint_state.name)

        # 所有关节设为零位
        home_state.position = [0.0] * len(self.current_joint_state.name)
        home_state.velocity = []
        home_state.effort = []

        return home_state

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        欧拉角转四元数
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def test_ik_at_point(self, x, y, z, group_name, ee_link,
                         orientation, timeout_sec=0.01, seed_state=None):
        """
        在指定点测试 IK 求解

        Args:
            x, y, z: 目标位置
            group_name: 规划组名称
            ee_link: 末端执行器链接名
            orientation: 目标姿态（四元数）
            timeout_sec: IK 超时时间（秒）
            seed_state: 初始关节状态种子

        Returns:
            (success: bool, time_ms: float, error_code: int)
        """
        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z
        request.ik_request.pose_stamped.pose.orientation = orientation
        request.ik_request.ik_link_name = ee_link
        request.ik_request.timeout.sec = int(timeout_sec)
        request.ik_request.timeout.nanosec = int((timeout_sec % 1) * 1e9)

        # 设置初始种子状态（Home 零位）
        if seed_state is not None:
            request.ik_request.robot_state.joint_state = seed_state

        # 关闭碰撞检测（通过设置空的碰撞检测）
        request.ik_request.robot_state.is_diff = True

        # 记录开始时间
        start_time = time.perf_counter()

        # 调用 IK 服务（异步）
        future = self.ik_client.call_async(request)

        # 使用带超时的等待
        # 注意：这里设置的超时要比 IK 内部超时稍长
        service_timeout = timeout_sec + 0.05  # 额外 50ms 用于通信

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)

            if future.done():
                break

            elapsed = time.perf_counter() - start_time
            if elapsed > service_timeout:
                # 服务调用超时
                end_time = time.perf_counter()
                elapsed_ms = (end_time - start_time) * 1000
                return False, elapsed_ms, -2  # -2 表示服务超时

        # 记录结束时间
        end_time = time.perf_counter()
        elapsed_ms = (end_time - start_time) * 1000

        if not future.done() or future.result() is None:
            return False, elapsed_ms, -1

        response = future.result()
        success = (response.error_code.val == 1)

        return success, elapsed_ms, response.error_code.val

    def generate_adaptive_values(self, range_vals, step_inner, step_outer):
        """
        生成自适应步长的坐标值
        中间 80% 区域用粗步长，边缘 20% 区域用细步长

        Args:
            range_vals: (min, max) 范围
            step_inner: 内部区域步长
            step_outer: 边缘区域步长

        Returns:
            坐标值数组
        """
        min_val, max_val = range_vals
        total_range = max_val - min_val

        # 计算 80% 中间区域的边界
        inner_min = min_val + total_range * 0.1
        inner_max = min_val + total_range * 0.9

        # 生成三个区域的值
        # 左边缘区域 (细步长)
        left_values = np.arange(min_val, inner_min + step_outer/2, step_outer)

        # 中间区域 (粗步长)
        middle_values = np.arange(inner_min + step_inner/2, inner_max + step_inner/2, step_inner)

        # 右边缘区域 (细步长)
        right_values = np.arange(inner_max + step_outer/2, max_val + step_outer/2, step_outer)

        # 合并并去重排序
        all_values = np.concatenate([left_values, middle_values, right_values])
        all_values = np.unique(np.round(all_values, 4))

        # 确保在范围内
        all_values = all_values[(all_values >= min_val) & (all_values <= max_val)]

        return np.sort(all_values)

    def run_grid_test(self, config):
        """
        执行网格测试

        Args:
            config: 测试配置字典

        Returns:
            results: 测试结果列表
        """
        results = []

        # 解析配置
        x_range = config['x_range']
        y_range = config['y_range']
        z_range = config['z_range']
        step_inner = config['step_inner']  # 内部区域步长
        step_outer = config['step_outer']  # 边缘区域步长
        group_name = config['group_name']
        ee_link = config['ee_link']
        orientation = config['orientation']
        timeout_sec = config['timeout_sec']

        # 生成自适应步长的坐标值
        x_values = self.generate_adaptive_values(x_range, step_inner, step_outer)
        y_values = self.generate_adaptive_values(y_range, step_inner, step_outer)
        z_values = self.generate_adaptive_values(z_range, step_inner, step_outer)

        total_points = len(x_values) * len(y_values) * len(z_values)

        self.get_logger().info(f"测试范围: X[{x_range[0]}, {x_range[1]}], "
                               f"Y[{y_range[0]}, {y_range[1]}], "
                               f"Z[{z_range[0]}, {z_range[1]}]")
        self.get_logger().info(f"内部步长: {step_inner}m, 边缘步长: {step_outer}m")
        self.get_logger().info(f"X点数: {len(x_values)}, Y点数: {len(y_values)}, Z点数: {len(z_values)}")
        self.get_logger().info(f"总测试点数: {total_points}")
        self.get_logger().info(f"规划组: {group_name}")
        self.get_logger().info(f"IK 超时: {timeout_sec}s")

        # 获取 Home 零位作为种子
        seed_state = self.get_home_joint_state()

        # 进度计数
        current_point = 0
        success_count = 0

        start_time = time.time()

        for x in x_values:
            for y in y_values:
                for z in z_values:
                    current_point += 1

                    # 测试 IK
                    success, time_ms, error_code = self.test_ik_at_point(
                        x, y, z, group_name, ee_link,
                        orientation, timeout_sec, seed_state
                    )

                    if success:
                        success_count += 1

                    # 记录结果
                    results.append({
                        'x': round(x, 4),
                        'y': round(y, 4),
                        'z': round(z, 4),
                        'is_success': success,
                        'time_ms': round(time_ms, 3),
                        'error_code': error_code
                    })

                    # 每 100 个点打印一次进度
                    if current_point % 100 == 0 or current_point == total_points:
                        elapsed = time.time() - start_time
                        rate = success_count / current_point * 100
                        eta = elapsed / current_point * (total_points - current_point)
                        self.get_logger().info(
                            f"进度: {current_point}/{total_points} "
                            f"({current_point/total_points*100:.1f}%) | "
                            f"成功: {success_count} ({rate:.1f}%) | "
                            f"耗时: {elapsed:.1f}s | "
                            f"预计剩余: {eta:.1f}s"
                        )

        total_time = time.time() - start_time
        final_rate = success_count / total_points * 100

        self.get_logger().info("=" * 60)
        self.get_logger().info("测试完成!")
        self.get_logger().info(f"总点数: {total_points}")
        self.get_logger().info(f"成功点数: {success_count}")
        self.get_logger().info(f"成功率: {final_rate:.2f}%")
        self.get_logger().info(f"总耗时: {total_time:.2f}s")
        self.get_logger().info(f"平均耗时: {total_time/total_points*1000:.2f}ms/点")
        self.get_logger().info("=" * 60)

        return results

    def save_results_to_csv(self, results, filename):
        """
        保存结果到 CSV 文件
        """
        fieldnames = ['x', 'y', 'z', 'is_success', 'time_ms', 'error_code']

        with open(filename, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for result in results:
                writer.writerow(result)

        self.get_logger().info(f"结果已保存到: {filename}")


def main():
    rclpy.init()

    node = IKGridTester()

    try:
        # ==================== 测试配置 ====================
        # 可根据需要修改以下参数

        config = {
            # 测试空间范围 (单位: 米)
            'x_range': (-0.3, 2.0),      # X 轴范围
            'y_range': (-0.3, 0.8),         # Y 轴范围
            'z_range': (0.5, 2.5),         # Z 轴范围

            # 自适应步长配置
            'step_inner': 0.05,           # 中间 80% 区域步长 (粗)
            'step_outer': 0.05,          # 边缘 20% 区域步长 (细)

            # 规划组配置 (二选一)
            'group_name': 'left_arm_with_base',  # 或 'right_arm_with_base'
            'ee_link': 'left_ee_link',           # 或 'right_ee_link'

            # 末端姿态 (朝下的姿态)
            # roll=π/2, pitch=0, yaw=0 表示末端朝下
            'orientation': node.euler_to_quaternion(math.pi/2, 0, 0),

            # IK 超时时间 (秒)
            'timeout_sec': 0.1,
        }

        # ==================== 用户确认 ====================
        print("\n" + "=" * 60)
        print("IK 求解器可达率测试")
        print("=" * 60)
        print(f"规划组: {config['group_name']}")
        print(f"末端链接: {config['ee_link']}")
        print(f"测试空间: X[{config['x_range'][0]}, {config['x_range'][1]}], "
              f"Y[{config['y_range'][0]}, {config['y_range'][1]}], "
              f"Z[{config['z_range'][0]}, {config['z_range'][1]}]")
        print(f"内部步长: {config['step_inner']}m, 边缘步长: {config['step_outer']}m")
        print(f"IK 超时: {config['timeout_sec']}s")
        print(f"末端姿态: 朝下 (roll=90°)")
        print("=" * 60)

        # 估算总点数 (粗略)
        print("=" * 60)

        confirm = input("\n确认开始测试? (y/n): ").strip().lower()
        if confirm != 'y':
            print("已取消测试")
            return

        # 生成输出文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"ik_result_{timestamp}.csv"
        output_filename = input(f"输出文件名 (回车使用默认: {default_filename}): ").strip()
        if not output_filename:
            output_filename = default_filename
        if not output_filename.endswith('.csv'):
            output_filename += '.csv'

        # ==================== 执行测试 ====================
        print("\n开始测试...")
        results = node.run_grid_test(config)

        # ==================== 保存结果 ====================
        node.save_results_to_csv(results, output_filename)

        # 打印统计摘要
        success_count = sum(1 for r in results if r['is_success'])
        success_times = [r['time_ms'] for r in results if r['is_success']]

        print("\n" + "=" * 60)
        print("统计摘要:")
        print(f"  总测试点数: {len(results)}")
        print(f"  成功点数: {success_count}")
        print(f"  失败点数: {len(results) - success_count}")
        print(f"  成功率: {success_count/len(results)*100:.2f}%")

        if success_times:
            print(f"  成功点平均耗时: {sum(success_times)/len(success_times):.2f}ms")
            print(f"  成功点最小耗时: {min(success_times):.2f}ms")
            print(f"  成功点最大耗时: {max(success_times):.2f}ms")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n用户中断测试")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
