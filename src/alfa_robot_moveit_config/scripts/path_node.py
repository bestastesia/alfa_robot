#!/usr/bin/env python3

import argparse
import math
import sys
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from path_planning_node import PathPlanningNode, normalize_quaternion
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from trajectory_executor import TrajectoryExecutor


class PathNode:
    def __init__(self, planning_group: str = "left_arm",
                 left_pose: Optional[List[float]] = None,
                 right_pose: Optional[List[float]] = None,
                 plan_only: bool = False) -> None:

        rclpy.init(args=sys.argv)

        self._planning_node = PathPlanningNode()
        self._executor_node = TrajectoryExecutor()
        self._user_plan_only = plan_only

        normalized_group = "dual_arm" if planning_group == "dual_arms" else planning_group

        # The planning node should always request plan-only when this wrapper is
        # responsible for sending trajectories to ros2_control. Otherwise MoveIt
        # would execute once and the external executor would send the same
        # trajectory again.
        self._planning_node.set_parameters([
            Parameter("planning_group", Parameter.Type.STRING, normalized_group),
            Parameter("plan_only", Parameter.Type.BOOL, True),
        ])

        if left_pose and len(left_pose) == 7:
            self._planning_node.set_parameters([
                Parameter("target_x_left", Parameter.Type.DOUBLE, left_pose[0]),
                Parameter("target_y_left", Parameter.Type.DOUBLE, left_pose[1]),
                Parameter("target_z_left", Parameter.Type.DOUBLE, left_pose[2]),
                Parameter("target_qx_left", Parameter.Type.DOUBLE, left_pose[3]),
                Parameter("target_qy_left", Parameter.Type.DOUBLE, left_pose[4]),
                Parameter("target_qz_left", Parameter.Type.DOUBLE, left_pose[5]),
                Parameter("target_qw_left", Parameter.Type.DOUBLE, left_pose[6]),
            ])

        if right_pose and len(right_pose) == 7:
            self._planning_node.set_parameters([
                Parameter("target_x_right", Parameter.Type.DOUBLE, right_pose[0]),
                Parameter("target_y_right", Parameter.Type.DOUBLE, right_pose[1]),
                Parameter("target_z_right", Parameter.Type.DOUBLE, right_pose[2]),
                Parameter("target_qx_right", Parameter.Type.DOUBLE, right_pose[3]),
                Parameter("target_qy_right", Parameter.Type.DOUBLE, right_pose[4]),
                Parameter("target_qz_right", Parameter.Type.DOUBLE, right_pose[5]),
                Parameter("target_qw_right", Parameter.Type.DOUBLE, right_pose[6]),
            ])

        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._planning_node)
        self._executor.add_node(self._executor_node)

    def run(self) -> int:
        print("\n" + "="*60)
        print("ALFA ROBOT PATH PLANNING AND EXECUTION NODE")
        print("="*60)

        planning_group = self._planning_node.get_parameter(
            "planning_group"
        ).get_parameter_value().string_value
        plan_only = self._planning_node.get_parameter(
            "plan_only"
        ).get_parameter_value().bool_value

        print(f"Planning group: {planning_group}")
        print(f"Plan only mode: {self._user_plan_only}")
        print("="*60 + "\n")

        if not self._executor_node.wait_for_controllers(timeout_sec=15.0):
            print("ERROR: Controllers not available")
            return 1

        exit_code = self._planning_node.run()

        if exit_code == 0 and not self._user_plan_only:
            trajectory = self._planning_node.get_planned_trajectory()

            if trajectory is not None:
                print("\n" + "-"*60)
                print("EXECUTING TRAJECTORY ON HARDWARE")
                print("-"*60 + "\n")

                success = self._executor_node.execute_full_trajectory(trajectory)

                if success:
                    print("\n" + "="*60)
                    print("TRAJECTORY EXECUTION COMPLETED SUCCESSFULLY")
                    print("="*60)
                else:
                    print("\n" + "="*60)
                    print("TRAJECTORY EXECUTION FAILED")
                    print("="*60)
                    exit_code = 6
            else:
                print("WARNING: No trajectory available for execution")

        elif exit_code == 0 and self._user_plan_only:
            trajectory = self._planning_node.get_planned_trajectory()
            if trajectory is not None:
                print("\n" + "="*60)
                print("PLANNING COMPLETED - TRAJECTORY SAVED (plan_only mode)")
                print(f"Use the trajectory data to control motors manually")
                print("="*60)

        return exit_code

    def shutdown(self) -> None:
        self._executor_node.destroy_node()
        self._planning_node.destroy_node()
        self._executor.shutdown()
        rclpy.shutdown()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Alfa Robot Path Planning and Execution Node"
    )

    parser.add_argument(
        "--planning-group", "-g",
        type=str,
        default="left_arm",
        choices=["left_arm", "right_arm", "dual_arm", "dual_arms"],
        help="Planning group name (default: left_arm)"
    )

    parser.add_argument(
        "--pose-left",
        type=float,
        nargs=7,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="Target pose for left arm (x y z qx qy qz qw)"
    )

    parser.add_argument(
        "--pose-right",
        type=float,
        nargs=7,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="Target pose for right arm (x y z qx qy qz qw)"
    )

    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="Only plan without executing"
    )

    parser.add_argument(
        "--velocity-scale",
        type=float,
        default=0.2,
        help="Max velocity scaling factor (default: 0.2)"
    )

    parser.add_argument(
        "--acceleration-scale",
        type=float,
        default=0.2,
        help="Max acceleration scaling factor (default: 0.2)"
    )

    args, _ = parser.parse_known_args()            
    return args  


def main() -> None:
    args = parse_args()

    path_node = PathNode(
        planning_group=args.planning_group,
        left_pose=args.pose_left,
        right_pose=args.pose_right,
        plan_only=args.plan_only,
    )

    if args.velocity_scale or args.acceleration_scale:
        params = []
        if args.velocity_scale:
            params.append(Parameter("max_velocity_scaling_factor", Parameter.Type.DOUBLE, args.velocity_scale))
        if args.acceleration_scale:
            params.append(Parameter("max_acceleration_scaling_factor", Parameter.Type.DOUBLE, args.acceleration_scale))
        path_node._planning_node.set_parameters(params)

    try:
        exit_code = path_node.run()
    except Exception as e:
        print(f"\nFATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        exit_code = 99
    finally:
        path_node.shutdown()

    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
