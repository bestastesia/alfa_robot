from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from robot_interfaces_qos import latched
from robot_system_interfaces.msg import DomainReadiness


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float, default=4.0)
    args = parser.parse_args()

    rclpy.init(args=None)
    node = Node("motion_container_healthcheck")
    received = False
    healthy = False

    def on_readiness(message: DomainReadiness) -> None:
        nonlocal received, healthy
        if message.domain == "motion":
            received = True
            critical = {
                "planner_unavailable",
                "joint_states_stale",
            }
            healthy = not critical.intersection(message.blockers)

    subscription = node.create_subscription(
        DomainReadiness,
        "/motion/readiness",
        on_readiness,
        latched(),
    )
    deadline = time.monotonic() + max(0.1, float(args.timeout_s))
    try:
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    if received and healthy:
        return 0
    reason = "not received" if not received else "reported a critical blocker"
    print(f"motion healthcheck failed: /motion/readiness {reason}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
