#!/usr/bin/env python3
"""
双末端位姿规划脚本

使用 dual_arm_with_base 规划组（bio_ik 求解器），同时指定：
  - 左臂末端 (left_tool0) 目标位姿
  - 右臂末端 (right_tool0) 目标位姿

bio_ik 会协调 updown 关节和双臂关节，找到满足两个末端约束的全身解。

使用方法：
  ros2 launch alfa_robot_moveit_config demo.launch.py
  python3 dual_arm_pose_planner.py
"""

import copy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints,
    PositionConstraint, OrientationConstraint,
    RobotState, MoveItErrorCodes,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import tf2_ros
import time


LEFT_TIP = "left_tool0"
RIGHT_TIP = "right_tool0"
PLANNING_GROUP = "dual_arm_with_base"
BASE_FRAME = "base_link"

# 所有属于 dual_arm_with_base 的关节（按 SRDF 顺序）
DUAL_ARM_JOINTS = [
    "updown",
    "leftjoint1", "leftjoint2", "leftjoint3",
    "leftjoint4", "leftjoint5", "leftjoint6",
    "rightjoint1", "rightjoint2", "rightjoint3",
    "rightjoint4", "rightjoint5", "rightjoint6",
]


def make_pose_constraints(link_name: str, pose: Pose,
                          pos_tol: float = 0.005,
                          ori_tol: float = 0.05) -> tuple:
    """为指定 link 构造一对位置+姿态约束。"""
    pos = PositionConstraint()
    pos.header.frame_id = BASE_FRAME
    pos.link_name = link_name
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [pos_tol]
    pos.constraint_region.primitives.append(sphere)
    region_pose = Pose()
    region_pose.position = pose.position
    region_pose.orientation.w = 1.0
    pos.constraint_region.primitive_poses.append(region_pose)
    pos.weight = 1.0

    ori = OrientationConstraint()
    ori.header.frame_id = BASE_FRAME
    ori.link_name = link_name
    ori.orientation = pose.orientation
    ori.absolute_x_axis_tolerance = ori_tol
    ori.absolute_y_axis_tolerance = ori_tol
    ori.absolute_z_axis_tolerance = ori_tol
    ori.weight = 1.0

    return pos, ori


class DualArmPlanner(Node):

    def __init__(self):
        super().__init__('dual_arm_planner')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.current_joint_state: JointState | None = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self.get_logger().info("等待 MoveIt 规划服务…")
        deadline = time.time() + 15.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.plan_client.service_is_ready():
                break
        if not self.plan_client.service_is_ready():
            raise RuntimeError("/plan_kinematic_path 服务不可用，请先启动 MoveIt")
        self.get_logger().info("MoveIt 就绪")

        # 等待第一帧关节状态
        deadline = time.time() + 5.0
        while self.current_joint_state is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_joint_state is None:
            self.get_logger().warn("未收到 /joint_states，起始状态将使用默认值")

    def _js_cb(self, msg: JointState):
        self.current_joint_state = msg

    # ------------------------------------------------------------------ #
    #  TF 工具：获取末端当前位姿
    # ------------------------------------------------------------------ #

    def get_ee_pose(self, link: str) -> Pose | None:
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, link, rclpy.time.Time(),
                rclpy.duration.Duration(seconds=2.0))
            p = Pose()
            p.position.x = t.transform.translation.x
            p.position.y = t.transform.translation.y
            p.position.z = t.transform.translation.z
            p.orientation = t.transform.rotation
            return p
        except Exception as e:
            self.get_logger().warn(f"TF {BASE_FRAME}->{link} 失败: {e}")
            return None

    def print_ee_poses(self):
        lp = self.get_ee_pose(LEFT_TIP)
        rp = self.get_ee_pose(RIGHT_TIP)
        if lp:
            self.get_logger().info(
                f"左臂 ({LEFT_TIP}): "
                f"pos=({lp.position.x:.3f}, {lp.position.y:.3f}, {lp.position.z:.3f})")
        if rp:
            self.get_logger().info(
                f"右臂 ({RIGHT_TIP}): "
                f"pos=({rp.position.x:.3f}, {rp.position.y:.3f}, {rp.position.z:.3f})")

    # ------------------------------------------------------------------ #
    #  核心：双末端位姿规划
    # ------------------------------------------------------------------ #

    def plan_dual_pose(
        self,
        left_pose: Pose,
        right_pose: Pose,
        pos_tol: float = 0.005,
        ori_tol: float = 0.05,
        timeout: float = 10.0,
        attempts: int = 20,
    ):
        """
        同时规划左右臂到目标位姿。

        bio_ik 会协调 updown + 双臂关节，找到满足两个末端约束的整体解。

        返回 RobotTrajectory，失败返回 None。
        """
        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = PLANNING_GROUP
        mpr.allowed_planning_time = timeout
        mpr.num_planning_attempts = attempts
        mpr.max_velocity_scaling_factor = 0.3
        mpr.max_acceleration_scaling_factor = 0.2

        # 起始状态
        if self.current_joint_state:
            mpr.start_state.joint_state = self.current_joint_state

        # 目标约束：两个末端各一套位置+姿态约束，放入同一个 Constraints
        goal = Constraints()
        goal.name = "dual_ee_goal"

        lpos, lori = make_pose_constraints(LEFT_TIP, left_pose, pos_tol, ori_tol)
        rpos, rori = make_pose_constraints(RIGHT_TIP, right_pose, pos_tol, ori_tol)

        goal.position_constraints.extend([lpos, rpos])
        goal.orientation_constraints.extend([lori, rori])

        mpr.goal_constraints.append(goal)
        req.motion_plan_request = mpr

        self.get_logger().info(
            f"双末端规划中… 左臂目标=({left_pose.position.x:.3f},"
            f"{left_pose.position.y:.3f},{left_pose.position.z:.3f}) "
            f"右臂目标=({right_pose.position.x:.3f},"
            f"{right_pose.position.y:.3f},{right_pose.position.z:.3f})")

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout + 5.0)

        if not future.done() or future.result() is None:
            self.get_logger().error("规划超时或服务失败")
            return None

        resp = future.result().motion_plan_response
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"规划失败，错误码: {resp.error_code.val}")
            return None

        traj = resp.trajectory
        self.get_logger().info(
            f"规划成功！轨迹点数: {len(traj.joint_trajectory.points)}，"
            f"关节: {traj.joint_trajectory.joint_names}")
        return traj

    # ------------------------------------------------------------------ #
    #  执行轨迹
    # ------------------------------------------------------------------ #

    def execute(self, trajectory) -> bool:
        """通过 ExecuteTrajectory action 执行轨迹。"""
        if not self.exec_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/execute_trajectory action 不可用")
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info("执行轨迹中…")
        send_future = self.exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()
        if not gh or not gh.accepted:
            self.get_logger().error("执行请求被拒绝")
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result()
        if result and result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("执行成功")
            return True
        self.get_logger().error(
            f"执行失败: {result.result.error_code.val if result else 'timeout'}")
        return False


# ------------------------------------------------------------------ #
#  交互式主循环
# ------------------------------------------------------------------ #

def main():
    rclpy.init()
    planner = DualArmPlanner()

    try:
        rclpy.spin_once(planner, timeout_sec=0.5)
        planner.print_ee_poses()

        print("\n" + "=" * 60)
        print("双末端位姿规划器 (dual_arm_with_base + bio_ik)")
        print("同时指定左右臂目标位姿，bio_ik 协调 updown 关节")
        print("=" * 60)

        while True:
            print("\n选择操作:")
            print("  1. 规划双臂绝对目标位姿")
            print("  2. 规划双臂相对偏移（在当前位姿基础上增量）")
            print("  3. 只移动左臂（右臂位姿固定为当前值）")
            print("  4. 只移动右臂（左臂位姿固定为当前值）")
            print("  5. 打印当前末端位姿")
            print("  0. 退出")

            choice = input("\n请输入选项: ").strip()

            if choice == "0":
                break

            elif choice == "5":
                rclpy.spin_once(planner, timeout_sec=0.3)
                planner.print_ee_poses()

            elif choice in ("1", "2", "3", "4"):
                rclpy.spin_once(planner, timeout_sec=0.3)
                cur_l = planner.get_ee_pose(LEFT_TIP)
                cur_r = planner.get_ee_pose(RIGHT_TIP)
                if cur_l is None or cur_r is None:
                    print("无法获取当前末端位姿，请稍后重试")
                    continue

                left_target = copy.deepcopy(cur_l)
                right_target = copy.deepcopy(cur_r)

                if choice == "1":
                    print("\n输入左臂目标位置 (base_link 坐标系，回车保持当前值):")
                    left_target = _input_pose(cur_l, "左臂")
                    print("\n输入右臂目标位置 (base_link 坐标系，回车保持当前值):")
                    right_target = _input_pose(cur_r, "右臂")

                elif choice == "2":
                    print("\n输入左臂位移增量 (m):")
                    left_target = _input_delta_pose(cur_l, "左臂")
                    print("\n输入右臂位移增量 (m):")
                    right_target = _input_delta_pose(cur_r, "右臂")

                elif choice == "3":
                    print("\n输入左臂目标位置 (右臂固定当前位姿):")
                    left_target = _input_pose(cur_l, "左臂")
                    # right_target 保持 cur_r（已是 deepcopy）

                elif choice == "4":
                    print("\n输入右臂目标位置 (左臂固定当前位姿):")
                    right_target = _input_pose(cur_r, "右臂")
                    # left_target 保持 cur_l

                traj = planner.plan_dual_pose(left_target, right_target)
                if traj:
                    ans = input("\n执行轨迹? (y/n): ").strip().lower()
                    if ans == 'y':
                        planner.execute(traj)
                        rclpy.spin_once(planner, timeout_sec=0.3)
                        planner.print_ee_poses()

            else:
                print("无效选项")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        planner.destroy_node()
        rclpy.shutdown()


def _input_pose(current: Pose, label: str) -> Pose:
    """交互式输入目标位姿，空输入保持当前值。"""
    p = copy.deepcopy(current)

    def _get(prompt, default):
        v = input(f"  {prompt} (当前 {default:.4f}，回车跳过): ").strip()
        return float(v) if v else default

    p.position.x = _get(f"{label} x", current.position.x)
    p.position.y = _get(f"{label} y", current.position.y)
    p.position.z = _get(f"{label} z", current.position.z)

    print(f"  姿态四元数 (回车保持当前):")
    p.orientation.x = _get("  ox", current.orientation.x)
    p.orientation.y = _get("  oy", current.orientation.y)
    p.orientation.z = _get("  oz", current.orientation.z)
    p.orientation.w = _get("  ow", current.orientation.w)
    return p


def _input_delta_pose(current: Pose, label: str) -> Pose:
    """交互式输入位移增量，空输入为 0。"""
    p = copy.deepcopy(current)

    def _get_delta(prompt):
        v = input(f"  {prompt} 增量 (m，回车=0): ").strip()
        return float(v) if v else 0.0

    p.position.x += _get_delta(f"{label} dx")
    p.position.y += _get_delta(f"{label} dy")
    p.position.z += _get_delta(f"{label} dz")
    return p


if __name__ == "__main__":
    main()
