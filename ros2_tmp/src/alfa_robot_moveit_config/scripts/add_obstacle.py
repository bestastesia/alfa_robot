#!/usr/bin/env python3
"""
RViz 障碍物建模工具 — 向 MoveIt 规划场景添加/移除碰撞物体

通过终端交互式添加长方体障碍物到 MoveIt 规划场景，
让规划器在运动规划时自动避障。

用法:
  ros2 launch alfa_robot_moveit_config demo.launch.py
  python3 add_obstacle.py

交互菜单:
  a - 添加长方体 (输入位置、尺寸)
  c - 添加圆柱体
  r - 移除指定物体
  l - 列出所有已添加的碰撞物体
  x - 清除所有碰撞物体
  q - 退出
"""

import sys
import time

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Header

import math


FRAME_ID = "base_link"


def make_pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0) -> Pose:
    """从 xyz + RPY 创建 Pose。"""
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z

    # RPY → 四元数
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)

    p.orientation.w = cr * cp * cy + sr * sp * sy
    p.orientation.x = sr * cp * cy - cr * sp * sy
    p.orientation.y = cr * sp * cy + sr * cp * sy
    p.orientation.z = cr * cp * sy - sr * sp * cy

    return p


def _input_float(prompt, default=0.0):
    v = input(f"  {prompt} (默认 {default}): ").strip()
    return float(v) if v else default


class ObstacleBuilder(Node):

    def __init__(self):
        super().__init__('add_obstacle')

        self.scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')

        self.get_logger().info("等待 /apply_planning_scene 服务 …")
        self.scene_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("服务就绪")

        self.objects: list[str] = []

    def _apply(self, collision_object: CollisionObject) -> bool:
        """向规划场景添加或移除碰撞物体。"""
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision_object)

        req = ApplyPlanningScene.Request()
        req.scene = scene

        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done() and future.result() is not None:
            return future.result().success
        return False

    def add_box(self, name: str, x: float, y: float, z: float,
                sx: float, sy: float, sz: float,
                roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> bool:
        """添加长方体碰撞物体。"""
        co = CollisionObject()
        co.header.frame_id = FRAME_ID
        co.id = name
        co.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [sx, sy, sz]
        co.primitives.append(box)
        co.primitive_poses.append(make_pose(x, y, z, roll, pitch, yaw))

        ok = self._apply(co)
        if ok:
            self.objects.append(name)
            self.get_logger().info(
                f"添加长方体 '{name}' @ ({x:.3f},{y:.3f},{z:.3f}) "
                f"尺寸=({sx:.3f},{sy:.3f},{sz:.3f})")
        else:
            self.get_logger().error(f"添加 '{name}' 失败")
        return ok

    def add_cylinder(self, name: str, x: float, y: float, z: float,
                     radius: float, height: float,
                     roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> bool:
        """添加圆柱体碰撞物体。"""
        co = CollisionObject()
        co.header.frame_id = FRAME_ID
        co.id = name
        co.operation = CollisionObject.ADD

        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [radius, height]
        co.primitives.append(cyl)
        co.primitive_poses.append(make_pose(x, y, z, roll, pitch, yaw))

        ok = self._apply(co)
        if ok:
            self.objects.append(name)
            self.get_logger().info(
                f"添加圆柱体 '{name}' @ ({x:.3f},{y:.3f},{z:.3f}) "
                f"r={radius:.3f} h={height:.3f}")
        else:
            self.get_logger().error(f"添加 '{name}' 失败")
        return ok

    def remove(self, name: str) -> bool:
        """移除碰撞物体。"""
        co = CollisionObject()
        co.header.frame_id = FRAME_ID
        co.id = name
        co.operation = CollisionObject.REMOVE

        ok = self._apply(co)
        if ok:
            if name in self.objects:
                self.objects.remove(name)
            self.get_logger().info(f"移除 '{name}'")
        else:
            self.get_logger().error(f"移除 '{name}' 失败")
        return ok

    def clear_all(self):
        """移除所有已添加的碰撞物体。"""
        for name in list(self.objects):
            self.remove(name)


def main():
    rclpy.init()
    builder = ObstacleBuilder()

    obj_counter = 0

    print("\n" + "=" * 60)
    print("MoveIt 障碍物建模工具")
    print("=" * 60)
    print("  a - 添加长方体")
    print("  c - 添加圆柱体")
    print("  r - 移除指定物体")
    print("  l - 列出已添加物体")
    print("  x - 清除所有物体")
    print("  q - 退出")
    print("=" * 60)
    print("坐标系: base_link, 单位: 米, 角度: 度")

    try:
        while True:
            cmd = input("\n> ").strip().lower()

            if cmd == 'q':
                break

            elif cmd == 'a':
                obj_counter += 1
                name = input(f"  名称 (默认 box_{obj_counter}): ").strip()
                if not name:
                    name = f"box_{obj_counter}"

                print("  位置 (base_link 坐标系, 米):")
                x = _input_float("x", 0.5)
                y = _input_float("y", 0.0)
                z = _input_float("z", 0.3)
                print("  尺寸 (米):")
                sx = _input_float("长(x)", 0.3)
                sy = _input_float("宽(y)", 0.3)
                sz = _input_float("高(z)", 0.3)
                print("  朝向 (度, 回车跳过为0):")
                roll = math.radians(_input_float("roll", 0.0))
                pitch = math.radians(_input_float("pitch", 0.0))
                yaw = math.radians(_input_float("yaw", 0.0))

                builder.add_box(name, x, y, z, sx, sy, sz, roll, pitch, yaw)

            elif cmd == 'c':
                obj_counter += 1
                name = input(f"  名称 (默认 cyl_{obj_counter}): ").strip()
                if not name:
                    name = f"cyl_{obj_counter}"

                print("  位置 (base_link 坐标系, 米):")
                x = _input_float("x", 0.5)
                y = _input_float("y", 0.0)
                z = _input_float("z", 0.3)
                radius = _input_float("半径", 0.05)
                height = _input_float("高度", 0.3)
                print("  朝向 (度, 回车跳过为0):")
                roll = math.radians(_input_float("roll", 0.0))
                pitch = math.radians(_input_float("pitch", 0.0))
                yaw = math.radians(_input_float("yaw", 0.0))

                builder.add_cylinder(name, x, y, z, radius, height, roll, pitch, yaw)

            elif cmd == 'r':
                name = input("  物体名称: ").strip()
                if name:
                    builder.remove(name)

            elif cmd == 'l':
                if not builder.objects:
                    print("\n  无碰撞物体")
                else:
                    print(f"\n  已添加 {len(builder.objects)} 个物体:")
                    for name in builder.objects:
                        print(f"    - {name}")

            elif cmd == 'x':
                if builder.objects:
                    builder.clear_all()
                    print("  已清除所有碰撞物体")
                else:
                    print("  无物体可清除")

            else:
                print("未知命令，可用: a / c / r / l / x / q")

    except KeyboardInterrupt:
        print("\n用户中断")

    finally:
        builder.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()