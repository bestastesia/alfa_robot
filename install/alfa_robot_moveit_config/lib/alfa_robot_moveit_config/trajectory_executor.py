#!/usr/bin/env python3

import sys
import threading
import time
from typing import Dict, List, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryExecutor(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_executor")

        self.declare_parameter("left_arm_controller", "left_arm_controller")
        self.declare_parameter("right_arm_controller", "right_arm_controller")
        self.declare_parameter("torso_controller", "torso_group_controller")
        self.declare_parameter("execute_timeout_sec", 60.0)

        left_ctrl = self.get_parameter("left_arm_controller").get_parameter_value().string_value
        right_ctrl = self.get_parameter("right_arm_controller").get_parameter_value().string_value
        torso_ctrl = self.get_parameter("torso_controller").get_parameter_value().string_value

        self._action_clients: Dict[str, ActionClient] = {
            "left_arm": ActionClient(self, FollowJointTrajectory, f"{left_ctrl}/follow_joint_trajectory"),
            "right_arm": ActionClient(self, FollowJointTrajectory, f"{right_ctrl}/follow_joint_trajectory"),
            "torso": ActionClient(self, FollowJointTrajectory, f"{torso_ctrl}/follow_joint_trajectory"),
        }

        self._controller_joints: Dict[str, List[str]] = {
            "left_arm": [
                "leftarmbase", "leftjoint1", "leftjoint2",
                "leftjoint3", "leftjoint4"
            ],
            "right_arm": [
                "rightarmbase", "rightjoint1", "rightjoint2",
                "rightjoint3", "rightjoint4"
            ],
            "torso": ["turn", "updown"],
        }

        self._lock = threading.Lock()
        self._current_goal_handle = None
        self._execution_complete = False
        self._execution_result = None

    def wait_for_controllers(self, timeout_sec: float = 10.0) -> bool:
        all_ready = True
        for group_name, client in self._action_clients.items():
            if not client.wait_for_server(timeout_sec=timeout_sec):
                self.get_logger().error(f"Controller {group_name} not available")
                all_ready = False
        return all_ready

    def _split_trajectory_by_group(self, trajectory: JointTrajectory) -> Dict[str, JointTrajectory]:
        grouped: Dict[str, JointTrajectory] = {}

        for group_name, joint_names in self._controller_joints.items():
            traj_indices = []
            for joint_name in joint_names:
                if joint_name in trajectory.joint_names:
                    traj_indices.append(trajectory.joint_names.index(joint_name))

            if not traj_indices:
                continue

            new_traj = JointTrajectory()
            new_traj.header = trajectory.header
            new_traj.joint_names = [trajectory.joint_names[i] for i in traj_indices]

            for point in trajectory.points:
                new_point = JointTrajectoryPoint()
                new_point.time_from_start = point.time_from_start
                new_point.positions = [point.positions[i] for i in traj_indices]
                if point.velocities:
                    new_point.velocities = [point.velocities[i] for i in traj_indices]
                if point.accelerations:
                    new_point.accelerations = [point.accelerations[i] for i in traj_indices]
                if point.effort:
                    new_point.effort = [point.effort[i] for i in traj_indices]
                new_traj.points.append(new_point)

            grouped[group_name] = new_traj

        return grouped

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error("Trajectory execution rejected by controller")
                with self._lock:
                    self._execution_complete = True
                    self._execution_result = False
                return

            self.get_logger().info("Trajectory execution accepted")
            self._current_goal_handle = goal_handle

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._result_callback)

        except Exception as e:
            self.get_logger().error(f"Goal response callback error: {e}")
            with self._lock:
                self._execution_complete = True
                self._execution_result = False

    def _result_callback(self, future):
        try:
            result = future.result().result
            if result is None:
                self.get_logger().error("Trajectory execution result is None")
                with self._lock:
                    self._execution_complete = True
                    self._execution_result = False
                return

            success = result.error_code == 0
            if success:
                self.get_logger().info("Trajectory executed successfully")
            else:
                self.get_logger().error(
                    f"Trajectory execution failed with error code: {result.error_code}"
                )

            with self._lock:
                self._execution_complete = True
                self._execution_result = success

        except Exception as e:
            self.get_logger().error(f"Result callback error: {e}")
            with self._lock:
                self._execution_complete = True
                self._execution_result = False

    def execute_trajectory_group(self, group_name: str,
                                 trajectory: JointTrajectory,
                                 wait: bool = True) -> bool:
        if group_name not in self._action_clients:
            self.get_logger().error(f"Unknown group: {group_name}")
            return False

        client = self._action_clients[group_name]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_tolerance = []
        goal.goal_time_tolerance = rclpy.duration.Duration(seconds=2.0).to_msg()

        with self._lock:
            self._execution_complete = False
            self._execution_result = None

        self.get_logger().info(
            f"Sending trajectory to {group_name}: "
            f"{len(trajectory.points)} points, joints={trajectory.joint_names}"
        )

        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_callback)

        if wait:
            timeout = self.get_parameter("execute_timeout_sec").get_parameter_value().double_value
            start_time = time.time()

            while not self._execution_complete:
                if time.time() - start_time > timeout:
                    self.get_logger().warning(f"Execution timed out after {timeout}s")
                    if self._current_goal_handle:
                        self._current_goal_handle.cancel_goal_async()
                    return False

                rclpy.spin_once(self, timeout_sec=0.1)

            with self._lock:
                return self._execution_result if self._execution_result is not None else False

        return True

    def execute_full_trajectory(self, trajectory: JointTrajectory,
                                simultaneous: bool = True) -> bool:
        grouped = self._split_trajectory_by_group(trajectory)

        if not grouped:
            self.get_logger().error("No valid joint groups found in trajectory")
            return False

        self.get_logger().info(
            f"Splitting trajectory into {len(grouped)} groups: {list(grouped.keys())}"
        )

        if simultaneous:
            results = {}
            threads = []

            for group_name, group_traj in grouped.items():
                def execute_group(gn=group_name, gt=group_traj):
                    results[gn] = self.execute_trajectory_group(gn, gt, wait=True)

                thread = threading.Thread(target=execute_group)
                thread.start()
                threads.append(thread)

            for thread in threads:
                thread.join(timeout=self.get_parameter("execute_timeout_sec").get_parameter_value().double_value)

            all_success = all(results.values())
            if all_success:
                self.get_logger().info("All groups executed successfully")
            else:
                failed = [gn for gn, success in results.items() if not success]
                self.get_logger().error(f"Failed groups: {failed}")

            return all_success

        else:
            for group_name, group_traj in grouped.items():
                if not self.execute_trajectory_group(group_name, group_traj, wait=True):
                    self.get_logger().error(f"Failed to execute {group_name} trajectory")
                    return False
            return True

    @staticmethod
    def create_trajectory_from_points(joint_names: List[str],
                                      waypoints: List[List[float]],
                                      time_between_points: float = 1.0) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        current_time = rclpy.duration.Duration()
        for i, positions in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = (current_time + rclpy.duration.Duration(
                seconds=time_between_points * (i + 1))).to_msg()
            trajectory.points.append(point)

        return trajectory


def main() -> None:
    rclpy.init(args=sys.argv)

    executor = TrajectoryExecutor()

    print("\n" + "="*60)
    print("TRAJECTORY EXECUTOR NODE")
    print("="*60)
    print("This node executes planned trajectories on the robot hardware.")
    print("It interfaces with ros2_control controllers to move motors.")
    print("="*60 + "\n")

    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
