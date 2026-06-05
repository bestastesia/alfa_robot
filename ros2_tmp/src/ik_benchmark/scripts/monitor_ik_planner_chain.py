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
        if msg_type == "ik_result_to_planner":
            for stale_type in ("planner_received_joint_target", "planner_output_trajectory", "trajectory_to_plc"):
                self.latest.pop(f"{task_id}:{stale_type}", None)
        key = f"{task_id}:{msg_type}"
        self.latest[key] = data
        print(f"\n=== {msg_type} task={task_id} from {topic} ===")
        self.print_summary(data)
        self.compare_if_ready(task_id)

    def print_summary(self, data: dict[str, Any]) -> None:
        if data.get("type") == "ik_result_to_planner":
            jt = data.get("joint_target", {})
            self.print_joint_list("IK selected", jt.get("names", []), jt.get("positions_deg", []), jt.get("positions_rad", []))
            print(f"IK success={data.get('success')} wall_ms={data.get('wall_ms')} reason={data.get('failure_reason')}")
        elif data.get("type") == "planner_received_joint_target":
            self.print_joint_list("Planner request", data.get("request_names", []), data.get("request_positions_deg", []), data.get("request_positions_rad", []))
            self.print_joint_map("Planner start", data.get("start_state_deg", {}), data.get("start_state_rad", {}))
            self.print_joint_map("Planner target", data.get("target_map_deg", {}), data.get("target_map_rad", {}))
        elif data.get("type") in ("planner_output_trajectory", "trajectory_to_plc"):
            names = data.get("joint_names", [])
            print(f"trajectory points={data.get('point_count')}")
            self.print_joint_list("Trajectory first", names, data.get("first_positions_deg", []), data.get("first_positions_rad", []))
            self.print_joint_list("Trajectory last", names, data.get("last_positions_deg", []), data.get("last_positions_rad", []))

    def display_value(self, name: str, value_deg: float | None, value_rad: float | None) -> float:
        if name == "updown":
            if value_rad is not None:
                return value_rad
            if value_deg is not None:
                return value_deg * math.pi / 180.0
            return 0.0
        if value_deg is not None:
            return value_deg
        if value_rad is not None:
            return value_rad * 180.0 / math.pi
        return 0.0

    def format_joint_value(self, name: str, value: float) -> str:
        if name == "updown":
            return f"{name}={value:.3f}m"
        return f"{name}={value:.2f}deg"

    def print_joint_list(self, label: str, names: list[Any], values_deg: list[Any], values_rad: list[Any] | None = None) -> None:
        values_rad = values_rad or []
        pairs: dict[str, float] = {}
        for i, raw_name in enumerate(names):
            name = str(raw_name)
            deg = float(values_deg[i]) if i < len(values_deg) else None
            rad = float(values_rad[i]) if i < len(values_rad) else None
            pairs[name] = self.display_value(name, deg, rad)
        text = ", ".join(self.format_joint_value(name, pairs[name]) for name in WATCH_JOINTS if name in pairs)
        print(f"{label}: {text}")

    def print_joint_map(self, label: str, values_deg: dict[str, Any], values_rad: dict[str, Any] | None = None) -> None:
        values_rad = values_rad or {}
        pairs: dict[str, float] = {}
        for name in WATCH_JOINTS:
            if name not in values_deg and name not in values_rad:
                continue
            deg = float(values_deg[name]) if name in values_deg else None
            rad = float(values_rad[name]) if name in values_rad else None
            pairs[name] = self.display_value(name, deg, rad)
        text = ", ".join(self.format_joint_value(name, pairs[name]) for name in WATCH_JOINTS if name in pairs)
        print(f"{label}: {text}")

    def compare_if_ready(self, task_id: str) -> None:
        ik = self.latest.get(f"{task_id}:ik_result_to_planner")
        received = self.latest.get(f"{task_id}:planner_received_joint_target")
        planner_traj = self.latest.get(f"{task_id}:planner_output_trajectory")
        plc_traj = self.latest.get(f"{task_id}:trajectory_to_plc")
        if ik and received:
            jt = ik.get("joint_target", {})
            ik_map = self.as_display_map(jt.get("names", []), jt.get("positions_deg", []), jt.get("positions_rad", []))
            req_map = self.as_display_map(received.get("request_names", []), received.get("request_positions_deg", []), received.get("request_positions_rad", []))
            self.print_delta("IK -> planner request", ik_map, req_map)
        if received and planner_traj:
            target_map = self.as_display_map_from_dict(received.get("target_map_deg", {}), received.get("target_map_rad", {}))
            last_map = self.as_display_map(planner_traj.get("joint_names", []), planner_traj.get("last_positions_deg", []), planner_traj.get("last_positions_rad", []))
            self.print_delta("planner target -> planner trajectory last", target_map, last_map)
        if planner_traj and plc_traj:
            planner_last = self.as_display_map(planner_traj.get("joint_names", []), planner_traj.get("last_positions_deg", []), planner_traj.get("last_positions_rad", []))
            plc_last = self.as_display_map(plc_traj.get("joint_names", []), plc_traj.get("last_positions_deg", []), plc_traj.get("last_positions_rad", []))
            self.print_delta("planner trajectory last -> PLC trajectory last", planner_last, plc_last)

    def as_display_map(self, names: list[Any], values_deg: list[Any], values_rad: list[Any] | None = None) -> dict[str, float]:
        values_rad = values_rad or []
        result: dict[str, float] = {}
        for i, raw_name in enumerate(names):
            name = str(raw_name)
            deg = float(values_deg[i]) if i < len(values_deg) else None
            rad = float(values_rad[i]) if i < len(values_rad) else None
            result[name] = self.display_value(name, deg, rad)
        return result

    def as_display_map_from_dict(self, values_deg: dict[str, Any], values_rad: dict[str, Any] | None = None) -> dict[str, float]:
        values_rad = values_rad or {}
        result: dict[str, float] = {}
        for name in set(values_deg.keys()) | set(values_rad.keys()):
            deg = float(values_deg[name]) if name in values_deg else None
            rad = float(values_rad[name]) if name in values_rad else None
            result[str(name)] = self.display_value(str(name), deg, rad)
        return result

    def print_delta(self, label: str, left: dict[str, float], right: dict[str, float]) -> None:
        deltas = []
        max_name = ""
        max_delta = -1.0
        for name in WATCH_JOINTS:
            if name not in left or name not in right:
                continue
            delta = abs(left[name] - right[name])
            unit = "m" if name == "updown" else "deg"
            precision = 4 if name == "updown" else 3
            deltas.append(f"{name}={delta:.{precision}f}{unit}")
            if delta > max_delta:
                max_delta = delta
                max_name = name
        if deltas:
            unit = "m" if max_name == "updown" else "deg"
            precision = 4 if max_name == "updown" else 3
            print(f"--- compare {label}: max {max_name}={max_delta:.{precision}f} {unit}")
            print(", ".join(deltas))

def main() -> None:
    rclpy.init()
    node = IkPlannerChainMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
