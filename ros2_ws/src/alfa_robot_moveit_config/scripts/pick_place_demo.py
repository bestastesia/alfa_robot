#!/usr/bin/env python3
"""
四次连续抓取-放置演示

流程（每轮）:
  安全位 → 接近(x-0.2) → 笛卡尔前移0.2m → 附着盒子 → 笛卡尔后退0.2m
  → 安全位 → turn=90° → 放置 → 脱除盒子 → 放置安全位 → turn=0°

用法:
  ros2 launch alfa_robot_moveit_config demo.launch.py
  python3 pick_place_demo.py            # 完整4轮
  python3 pick_place_demo.py -n 1       # 只跑第1轮
  python3 pick_place_demo.py --ik-timeout 10
"""

import math
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene
from moveit_msgs.msg import (
    PositionIKRequest, RobotState, MoveItErrorCodes,
    PlanningScene, CollisionObject, AttachedCollisionObject,
)
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

# ── 常量 ──────────────────────────────────────────────────────

LEFT_TIP = "left_tool0"
RIGHT_TIP = "right_tool0"
BASE_FRAME = "base_link"

DUAL_ARM_GROUP = "dual_arm_with_base"

TORSO_JOINTS = ["pitch", "turn"]
ARM_JOINTS = [
    "updown",
    "leftjoint1", "leftjoint2", "leftjoint3",
    "leftjoint4", "leftjoint5", "leftjoint6",
    "rightjoint1", "rightjoint2", "rightjoint3",
    "rightjoint4", "rightjoint5", "rightjoint6",
]
ALL_JOINTS = TORSO_JOINTS + ARM_JOINTS

LEFT_TOUCH = ["leftjoint6", "left_tool0"]
RIGHT_TOUCH = ["rightjoint6", "right_tool0"]

LEFT_ATTACH_LINK = LEFT_TIP  # left_tool0
RIGHT_ATTACH_LINK = RIGHT_TIP  # right_tool0

BOX_DIMS = [0.2, 0.2, 0.2]

# ── 抓取/放置位姿定义 ─────────────────────────────────────────

GRASP_ORIENT = (0.0, 0.7071, 0.0, 0.7071)
PLACE_ORIENT = (-0.5, 0.5, 0.5, 0.5)

# 4次抓取点: (左x,y,z), (右x,y,z)
PICK_POINTS = [
    # ((0.6,  0.7, 1.5), (0.6, -0.7, 1.5)),
    ((0.6,  0.5, 1.5), (0.6, -0.5, 1.5)),
    ((0.6,  0.3, 1.5), (0.6, -0.3, 1.5)),
    ((0.6,  0, 1.5), (0.6, 0, 1.1)),
]

APPROACH_OFFSET = 0.1  # 沿 x 方向前移/后退 0.2m


def make_pose(x, y, z, qx, qy, qz, qw) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.x = float(qx)
    p.orientation.y = float(qy)
    p.orientation.z = float(qz)
    p.orientation.w = float(qw)
    return p


# ── 主类 ──────────────────────────────────────────────────────

class PickPlaceDemo(Node):

    def __init__(self, ik_timeout=5.0, step=False):
        super().__init__('pick_place_demo')
        self.ik_timeout = ik_timeout
        self.step = step

        self.current_js: dict = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # IK 服务
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.get_logger().info("等待 IK 服务 …")
        self.ik_client.wait_for_service(timeout_sec=10.0)

        # 控制器
        self.controllers: dict[str, ActionClient] = {}
        self.controller_joints: dict[str, list] = {}
        candidates = {
            "torso_controller": {
                "ns": "/torso_controller/follow_joint_trajectory",
                "joints": TORSO_JOINTS,
            },
            "dual_arm_controller": {
                "ns": "/dual_arm_controller/follow_joint_trajectory",
                "joints": ARM_JOINTS,
            },
        }
        for name, info in candidates.items():
            client = ActionClient(self, FollowJointTrajectory, info["ns"])
            if client.wait_for_server(timeout_sec=3.0):
                self.controllers[name] = client
                self.controller_joints[name] = info["joints"]
                self.get_logger().info(f"  {name}: 就绪")
            else:
                self.get_logger().warn(f"  {name}: 不可用")
                client.destroy()

        # 规划场景服务
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.scene_client.wait_for_service(timeout_sec=5.0)

        # 等关节状态到达
        deadline = time.time() + 3.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.current_js) >= 10:
                break

        self.get_logger().info("就绪")

    def _js_cb(self, msg: JointState):
        for name, val in zip(msg.name, msg.position):
            self.current_js[name] = val

    def wait_step(self, label: str = ""):
        """交互模式下按 Enter 继续。"""
        if self.step:
            input(f"  >> 按Enter继续: {label} ...")

    # ── IK ─────────────────────────────────────────────────────

    def solve_dual_ik(self, left_pose: Pose, right_pose: Pose,
                       timeout: float = None) -> dict | None:
        timeout = timeout or self.ik_timeout
        req = GetPositionIK.Request()
        req.ik_request.group_name = DUAL_ARM_GROUP
        req.ik_request.ik_link_names = [LEFT_TIP, RIGHT_TIP]
        req.ik_request.pose_stamped_vector = []
        for pose in [left_pose, right_pose]:
            ps = PoseStamped()
            ps.header.frame_id = BASE_FRAME
            ps.pose = pose
            req.ik_request.pose_stamped_vector.append(ps)
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout - int(timeout)) * 1e9)
        req.ik_request.robot_state = RobotState()
        for j in ALL_JOINTS:
            if j in self.current_js:
                req.ik_request.robot_state.joint_state.name.append(j)
                req.ik_request.robot_state.joint_state.position.append(self.current_js[j])

        self.get_logger().info(
            f"IK: L=({left_pose.position.x:.2f},{left_pose.position.y:.2f},{left_pose.position.z:.2f}) "
            f"R=({right_pose.position.x:.2f},{right_pose.position.y:.2f},{right_pose.position.z:.2f})")

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout + 2.0)

        if not future.done() or future.result() is None:
            self.get_logger().error("IK 服务调用失败/超时")
            return None

        res = future.result()
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"IK 失败: code={res.error_code.val}")
            return None

        solution = {}
        js = res.solution.joint_state
        for name, val in zip(js.name, js.position):
            solution[name] = val

        # 归一化：对所有臂关节选择离 seed 最近的等效角度
        for j in ARM_JOINTS + ["turn"]:
            if j in solution and j in self.current_js:
                sol = solution[j]
                seed = self.current_js[j]
                # 选择最近的等效角度 (相差 2π 的整数倍)
                while sol - seed > math.pi:
                    sol -= 2 * math.pi
                while sol - seed < -math.pi:
                    sol += 2 * math.pi
                solution[j] = sol

        self.get_logger().info(f"IK 成功: {len(solution)} 关节")
        return solution

    # ── 轨迹执行 ──────────────────────────────────────────────

    def _send_trajectory(self, joint_names, positions, client,
                         duration_sec=3.0) -> bool:
        current = [self.current_js.get(n, 0.0) for n in joint_names]

        start = JointTrajectoryPoint()
        start.positions = current

        end = JointTrajectoryPoint()
        end.positions = list(positions)
        end.time_from_start.sec = int(duration_sec)
        end.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.points.append(start)
        traj.points.append(end)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info(f"执行: {len(joint_names)} 关节, {duration_sec:.1f}s")
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

        if not send_future.done() or send_future.result() is None:
            self.get_logger().error("Goal 发送失败")
            return False
        gh = send_future.result()
        if not gh.accepted:
            self.get_logger().error("Goal 被拒绝")
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future,
                                         timeout_sec=duration_sec + 5.0)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error("执行超时")
            return False

        r = result_future.result()
        if r.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("执行成功")
            return True
        self.get_logger().error(f"执行失败: code={r.result.error_code}")
        return False

    def _send_interpolated(self, joint_names, start_pos, end_pos,
                            client, n_steps=10, step_sec=0.08) -> bool:
        points = []
        for i in range(n_steps + 1):
            t = i / n_steps
            pos = [s + t * (e - s) for s, e in zip(start_pos, end_pos)]
            pt = JointTrajectoryPoint()
            pt.positions = pos
            sec = i * step_sec
            pt.time_from_start.sec = int(sec)
            pt.time_from_start.nanosec = int((sec - int(sec)) * 1e9)
            points.append(pt)

        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.points = points

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        total_sec = n_steps * step_sec
        self.get_logger().info(f"插值: {n_steps}步, {total_sec:.1f}s")
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

        if not send_future.done() or send_future.result() is None:
            self.get_logger().error("插值 Goal 发送失败")
            return False
        gh = send_future.result()
        if not gh.accepted:
            self.get_logger().error("插值 Goal 被拒绝")
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future,
                                         timeout_sec=total_sec + 5.0)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error("插值执行超时")
            return False
        r = result_future.result()
        if r.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("插值执行成功")
            return True
        self.get_logger().error(f"插值执行失败: code={r.result.error_code}")
        return False

    # ── 高层执行 ──────────────────────────────────────────────

    def execute_joints(self, targets: dict, duration_sec=1.5) -> bool:
        ok = True
        covered = set()
        for ctrl_name, ctrl_joints in self.controller_joints.items():
            ctrl_targets = {n: targets[n] for n in ctrl_joints if n in targets}
            if not ctrl_targets:
                continue
            covered.update(ctrl_targets.keys())
            names = list(ctrl_targets.keys())
            positions = list(ctrl_targets.values())
            if not self._send_trajectory(names, positions,
                                         self.controllers[ctrl_name], duration_sec):
                ok = False
        uncovered = set(targets.keys()) - covered
        if uncovered:
            self.get_logger().warn(f"无控制器: {sorted(uncovered)}")
        return ok

    def execute_linear(self, start_targets: dict, end_targets: dict,
                        n_steps=10, step_sec=0.08) -> bool:
        ok = True
        covered = set()
        for ctrl_name, ctrl_joints in self.controller_joints.items():
            ctrl_start = {n: start_targets[n] for n in ctrl_joints if n in start_targets}
            ctrl_end = {n: end_targets[n] for n in ctrl_joints if n in end_targets}
            if not ctrl_start or not ctrl_end:
                continue
            covered.update(ctrl_start.keys())
            names = list(ctrl_start.keys())
            sp = [ctrl_start[n] for n in names]
            ep = [ctrl_end[n] for n in names]
            if not self._send_interpolated(names, sp, ep,
                                           self.controllers[ctrl_name],
                                           n_steps, step_sec):
                ok = False
        return ok

    def move_dual_arms(self, left_pose: Pose, right_pose: Pose,
                        duration_sec=3.0) -> bool:
        sol = self.solve_dual_ik(left_pose, right_pose)
        if sol is None:
            self.get_logger().error("IK 求解失败")
            return False
        ok = self.execute_joints(sol, duration_sec)
        if ok:
            self.sync_scene()
        return ok

    def move_dual_arms_linear(self, left_start: Pose, right_start: Pose,
                               left_end: Pose, right_end: Pose,
                               n_steps=10, step_sec=0.15) -> bool:
        sol_start = self.solve_dual_ik(left_start, right_start)
        sol_end = self.solve_dual_ik(left_end, right_end)
        if sol_start is None or sol_end is None:
            self.get_logger().error("直线运动 IK 失败")
            return False
        ok = self.execute_linear(sol_start, sol_end, n_steps, step_sec)
        if ok:
            self.sync_scene()
        return ok

    def move_turn(self, turn_rad: float, duration_sec=1.5) -> bool:
        if "torso_controller" not in self.controllers:
            self.get_logger().warn("torso_controller 不可用，跳过 turn")
            return True
        names = ["pitch", "turn"]
        positions = [self.current_js.get("pitch", 0.0), turn_rad]
        ok = self._send_trajectory(names, positions,
                                    self.controllers["torso_controller"],
                                    duration_sec)
        if ok:
            self.sync_scene()
        return ok

    # ── Scene 同步 ────────────────────────────────────────────

    def sync_scene(self):
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state = RobotState()
        scene_msg.robot_state.is_diff = True  # 关键: 只更新 joint_state, 不覆盖 attached objects
        scene_msg.robot_state.joint_state.name = list(self.current_js.keys())
        scene_msg.robot_state.joint_state.position = list(self.current_js.values())

        req = ApplyPlanningScene.Request()
        req.scene = scene_msg
        self.scene_client.call_async(req)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        pub = self.create_publisher(PlanningScene, '/planning_scene', qos)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        pub.publish(scene_msg)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_publisher(pub)

    # ── 附着/脱除 ─────────────────────────────────────────────

    def _apply_scene(self, scene: PlanningScene) -> bool:
        # 1. 调用 service (更新 move_group 内部 PlanningScene)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        service_ok = (future.done() and future.result() is not None
                      and future.result().success)
        if not service_ok:
            self.get_logger().error("ApplyPlanningScene 服务调用失败")

        # 2. 同时发布到 /planning_scene topic (让 RViz 立即更新)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        pub = self.create_publisher(PlanningScene, '/planning_scene', qos)
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.05)
        pub.publish(scene)
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.05)
        self.destroy_publisher(pub)

        return service_ok

    def _build_attach_scene(self, object_id: str, link_name: str,
                            touch_links: list = None) -> PlanningScene:
        """构建附着盒子的 scene diff（不发送）。

        注意：AttachedCollisionObject 会自动把 object 添加到 world，
        同时标记为 attached 到指定 link。
        """
        co = CollisionObject()
        co.header.frame_id = link_name
        co.id = object_id
        co.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = BOX_DIMS
        co.primitives.append(box)

        # 盒子在夹爪前方 10cm (tool0 的 +z 方向)
        pose = Pose()
        pose.position.z = 0.10
        pose.orientation.w = 1.0
        co.primitive_poses.append(pose)

        aco = AttachedCollisionObject()
        aco.object = co
        aco.link_name = link_name
        aco.touch_links = touch_links or [link_name]
        aco.weight = 0.1

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        return scene

    def detach_and_remove(self, object_id: str, link_name: str) -> bool:
        aco = AttachedCollisionObject()
        aco.object = CollisionObject()
        aco.object.id = object_id
        aco.object.operation = CollisionObject.REMOVE
        aco.link_name = link_name

        scene1 = PlanningScene()
        scene1.is_diff = True
        scene1.robot_state.is_diff = True
        scene1.robot_state.attached_collision_objects.append(aco)
        self._apply_scene(scene1)

        co = CollisionObject()
        co.header.frame_id = BASE_FRAME
        co.id = object_id
        co.operation = CollisionObject.REMOVE

        scene2 = PlanningScene()
        scene2.is_diff = True
        scene2.world.collision_objects.append(co)

        ok = self._apply_scene(scene2)
        if ok:
            self.get_logger().info(f"脱除 '{object_id}'")
            self.sync_scene()
        return ok

    # ── 单轮流程 ──────────────────────────────────────────────

    def run_one_round(self, idx: int,
                       left_pos: tuple, right_pos: tuple) -> bool:
        tag = f"[第{idx+1}轮]"
        lx, ly, lz = left_pos
        rx, ry, rz = right_pos

        g = GRASP_ORIENT
        p = PLACE_ORIENT

        # ── 抓取阶段 ──
        safe_l = make_pose(0.3,  0.3, lz, *g)
        safe_r = make_pose(0.3, -0.3, rz, *g)
        app_l = make_pose(lx - APPROACH_OFFSET, ly, lz, *g)
        app_r = make_pose(rx - APPROACH_OFFSET, ry, rz, *g)
        grasp_l = make_pose(lx, ly, lz, *g)
        grasp_r = make_pose(rx, ry, rz, *g)

        # 1. 安全位
        self.get_logger().info(f"{tag} → 安全位")
        self.wait_step("安全位")
        if not self.move_dual_arms(safe_l, safe_r, 2.5):
            return False
        time.sleep(0.3)

        # 2. 接近位
        self.get_logger().info(f"{tag} → 接近位 (x-0.2)")
        self.wait_step("接近位")
        if not self.move_dual_arms(app_l, app_r, 2.0):
            return False
        time.sleep(0.2)

        # 3. 前移0.2m
        self.get_logger().info(f"{tag} → 前移抓取")
        self.wait_step("前移抓取")
        if not self.move_dual_arms_linear(app_l, app_r, grasp_l, grasp_r,
                                           n_steps=10, step_sec=0.08):
            return False
        time.sleep(0.1)

        # 4. 附着盒子（合并到一个 scene diff）
        self.get_logger().info(f"{tag} → 附着盒子")
        self.wait_step("附着盒子")
        scene_l = self._build_attach_scene(f"box_L{idx}", LEFT_ATTACH_LINK, LEFT_TOUCH)
        scene_r = self._build_attach_scene(f"box_R{idx}", RIGHT_ATTACH_LINK, RIGHT_TOUCH)
        # 合并两个 attach 到一个 diff
        scene_l.robot_state.attached_collision_objects.extend(
            scene_r.robot_state.attached_collision_objects)
        ok = self._apply_scene(scene_l)
        if ok:
            self.get_logger().info(f"附着 box_L{idx}→{LEFT_ATTACH_LINK}, box_R{idx}→{RIGHT_ATTACH_LINK}")
        else:
            self.get_logger().error("附着失败")
        self.sync_scene()
        time.sleep(0.3)

        # 5. 后退0.2m
        self.get_logger().info(f"{tag} → 后退")
        self.wait_step("后退")
        if not self.move_dual_arms_linear(grasp_l, grasp_r, app_l, app_r,
                                           n_steps=10, step_sec=0.08):
            return False
        time.sleep(0.1)

        # 6. 安全位
        self.get_logger().info(f"{tag} → 安全位")
        self.wait_step("安全位(抓取后)")
        if not self.move_dual_arms(safe_l, safe_r, 1.5):
            return False
        time.sleep(0.1)

        # ── 放置阶段 ──
        # 7. turn 90°
        self.get_logger().info(f"{tag} → turn 90°")
        self.wait_step("turn 90°")
        if not self.move_turn(math.pi / 2, 1.5):
            return False
        time.sleep(0.2)

        # 8. 放置位
        place_l = make_pose(-0.2, 0.6, 0.6, *p)
        place_r = make_pose(0.2, 0.6, 0.6, *p)
        self.get_logger().info(f"{tag} → 放置位")
        self.wait_step("放置位")
        if not self.move_dual_arms(place_l, place_r, 1.5):
            return False
        time.sleep(0.1)

        # 9. 脱除盒子
        self.get_logger().info(f"{tag} → 脱除盒子")
        self.wait_step("脱除盒子")

        # 先 detach
        scene1 = PlanningScene()
        scene1.is_diff = True
        scene1.robot_state.is_diff = True
        for oid, lnk in [(f"box_L{idx}", LEFT_ATTACH_LINK), (f"box_R{idx}", RIGHT_ATTACH_LINK)]:
            aco = AttachedCollisionObject()
            aco.object = CollisionObject()
            aco.object.id = oid
            aco.object.operation = CollisionObject.REMOVE
            aco.link_name = lnk
            scene1.robot_state.attached_collision_objects.append(aco)
        self._apply_scene(scene1)

        # 再从 world 移除
        scene2 = PlanningScene()
        scene2.is_diff = True
        for oid in [f"box_L{idx}", f"box_R{idx}"]:
            co = CollisionObject()
            co.header.frame_id = BASE_FRAME
            co.id = oid
            co.operation = CollisionObject.REMOVE
            scene2.world.collision_objects.append(co)
        self._apply_scene(scene2)
        self.get_logger().info(f"脱除 box_L{idx}, box_R{idx}")
        self.sync_scene()
        time.sleep(0.2)

        # 10. 放置安全位
        psafe_l = make_pose(-0.3, 0.3, 0.6, *p)
        psafe_r = make_pose(0.3,  0.3, 0.6, *p)
        self.get_logger().info(f"{tag} → 放置安全位")
        self.wait_step("放置安全位")
        if not self.move_dual_arms(psafe_l, psafe_r, 1.5):
            return False
        time.sleep(0.1)

        # 11. turn 回 0°
        self.get_logger().info(f"{tag} → turn 回 0°")
        self.wait_step("turn 回 0°")
        if not self.move_turn(0.0, 1.5):
            return False
        time.sleep(0.2)

        return True


# ── 主入口 ────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="四次连续抓取-放置演示")
    parser.add_argument("-n", "--rounds", type=int, default=4,
                        help="执行轮数 (默认4)")
    parser.add_argument("--ik-timeout", type=float, default=5.0,
                        help="IK 求解超时/秒 (默认5)")
    parser.add_argument("--step", action="store_true",
                        help="交互模式: 每步按Enter继续")
    parser.add_argument("--start", type=int, default=0,
                        help="从第几轮开始 (0-based, 默认0)")
    args = parser.parse_args()

    n = min(args.rounds, len(PICK_POINTS) - args.start)

    print("\n" + "=" * 60)
    print("四次连续抓取-放置演示")
    print("=" * 60)
    for i in range(args.start, args.start + n):
        lp, rp = PICK_POINTS[i]
        print(f"  第{i+1}轮: 左({lp[0]},{lp[1]},{lp[2]}) 右({rp[0]},{rp[1]},{rp[2]})")
    print(f"  IK 超时: {args.ik_timeout}s")
    print("=" * 60)

    rclpy.init()
    demo = PickPlaceDemo(ik_timeout=args.ik_timeout, step=args.step)

    try:
        for i in range(args.start, args.start + n):
            lp, rp = PICK_POINTS[i]
            demo.get_logger().info(f"\n{'='*50}\n  开始第 {i+1} 轮\n{'='*50}")

            ok = demo.run_one_round(i, lp, rp)
            if not ok:
                demo.get_logger().error(f"第 {i+1} 轮失败，终止")
                break

            demo.get_logger().info(f"第 {i+1} 轮完成!")

        demo.get_logger().info("全部完成!")
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()