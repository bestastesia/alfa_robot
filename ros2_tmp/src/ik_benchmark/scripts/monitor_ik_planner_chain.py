#!/usr/bin/python3
from __future__ import annotations

import json
import math
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


WATCH_JOINTS = [
    "updown",
    "left_v5_joint1", "left_v5_joint2", "left_v5_joint3", "left_v5_joint4", "left_v5_joint5", "left_v5_joint6",
    "right_v5_joint1", "right_v5_joint2", "right_v5_joint3", "right_v5_joint4", "right_v5_joint5", "right_v5_joint6",
]


class IkPlannerChainMonitor(Node):
    def __init__(self) -> None:
        super().__init__("ik_planner_chain_monitor")
        self.latest: dict[str, dict[str, Any]] = {}
        topics = [
            "/alfa_debug/orchestrator_ik_result",
            "/alfa_debug/planner_received_joint_target",
            "/alfa_debug/planner_output_trajectory",
            "/alfa_debug/orchestrator_plc_trajectory",
        ]
        for topic in topics:
            self.create_subscription(String, topic, lambda msg, t=topic: self.on_msg(t, msg), 10)
        self.get_logger().info("Monitoring IK -> planner -> PLC trajectory debug topics")

    def on_msg(self, topic: str, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"{topic}: invalid JSON")
            return
        task_id = str(data.get("task_id", ""))
        msg_type = str(data.get("type", topic.rsplit("/", 1)[-1]))
        key = f"{task_id}:{msg_type}"
        self.latest[key] = data
        print(f"\n=== {msg_type} task={task_id} from {topic} ===")
        self.print_summary(data)
        self.compare_if_ready(task_id)

    def print_summary(self, data: dict[str, Any]) -> None:
        if data.get("type") == "ik_result_to_planner":
            jt = data.get("joint_target", {})
            self.print_joint_list("IK selected", jt.get("names", []), jt.get("positions_deg", []))
            print(f"IK success={data.get('success')} wall_ms={data.get('wall_ms')} reason={data.get('failure_reason')}")
        elif data.get("type") == "planner_received_joint_target":
            self.print_joint_list("Planner request", data.get("request_names", []), data.get("request_positions_deg", []))
            self.print_joint_map("Planner start", data.get("start_state_deg", {}))
            self.print_joint_map("Planner target", data.get("target_map_deg", {}))
        elif data.get("type") in ("planner_output_trajectory", "trajectory_to_plc"):
            names = data.get("joint_names", [])
            print(f"trajectory points={data.get('point_count')}")
            self.print_joint_list("Trajectory first", names, data.get("first_positions_deg", []))
            self.print_joint_list("Trajectory last", names, data.get("last_positions_deg", []))

    def print_joint_list(self, label: str, names: list[Any], values: list[Any]) -> None:
        pairs = {str(name): float(values[i]) for i, name in enumerate(names) if i < len(values)}
        text = ", ".join(f"{name}={pairs[name]:.2f}" for name in WATCH_JOINTS if name in pairs)
        print(f"{label}: {text}")

    def print_joint_map(self, label: str, values: dict[str, Any]) -> None:
        text = ", ".join(
            f"{name}={float(values[name]):.2f}" for name in WATCH_JOINTS if name in values
        )
        print(f"{label}: {text}")

    def compare_if_ready(self, task_id: str) -> None:
        ik = self.latest.get(f"{task_id}:ik_result_to_planner")
        received = self.latest.get(f"{task_id}:planner_received_joint_target")
        planner_traj = self.latest.get(f"{task_id}:planner_output_trajectory")
        plc_traj = self.latest.get(f"{task_id}:trajectory_to_plc")
        if ik and received:
            ik_map = self.as_map(ik.get("joint_target", {}).get("names", []), ik.get("joint_target", {}).get("positions_deg", []))
            req_map = self.as_map(received.get("request_names", []), received.get("request_positions_deg", []))
            self.print_delta("IK -> planner request", ik_map, req_map)
        if received and planner_traj:
            target_map = {str(k): float(v) for k, v in received.get("target_map_deg", {}).items()}
            last_map = self.as_map(planner_traj.get("joint_names", []), planner_traj.get("last_positions_deg", []))
            self.print_delta("planner target -> planner trajectory last", target_map, last_map)
        if planner_traj and plc_traj:
            planner_last = self.as_map(planner_traj.get("joint_names", []), planner_traj.get("last_positions_deg", []))
            plc_last = self.as_map(plc_traj.get("joint_names", []), plc_traj.get("last_positions_deg", []))
            self.print_delta("planner trajectory last -> PLC trajectory last", planner_last, plc_last)

    def as_map(self, names: list[Any], values: list[Any]) -> dict[str, float]:
        return {str(name): float(values[i]) for i, name in enumerate(names) if i < len(values)}

    def print_delta(self, label: str, left: dict[str, float], right: dict[str, float]) -> None:
        deltas = []
        max_name = ""
        max_delta = -1.0
        for name in WATCH_JOINTS:
            if name not in left or name not in right:
                continue
            delta = abs(left[name] - right[name])
            deltas.append(f"{name}={delta:.3f}")
            if delta > max_delta:
                max_delta = delta
                max_name = name
        if deltas:
            print(f"--- compare {label}: max {max_name}={max_delta:.3f} deg")
            print(", ".join(deltas))


def main() -> None:
    rclpy.init()
    node = IkPlannerChainMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
