#!/usr/bin/env python3
"""Wait for a sustained, rejection-free real rolling HOLD before tracking."""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class TrackReadyGate(Node):
    def __init__(self, *, soak_seconds: float, min_accepted: int, timeout_seconds: float) -> None:
        super().__init__("realtime_6d_pose_track_ready_gate")
        self.soak_seconds = soak_seconds
        self.min_accepted = min_accepted
        self.deadline = time.monotonic() + timeout_seconds
        self.last_message_at: float | None = None
        self.soak_started_at: float | None = None
        self.first_accepted: int | None = None
        self.last_accepted = 0
        self.last_rejected_count = 0
        self.ready = False
        self.failure: str | None = None
        self.create_subscription(
            String,
            "/realtime_6d_pose/rt_status",
            self.on_status,
            qos_profile_sensor_data,
        )

    def on_status(self, message: String) -> None:
        now = time.monotonic()
        self.last_message_at = now
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError) as error:
            self.failure = f"invalid rt_status JSON: {error}"
            return

        session_state = str(status.get("session_state", ""))
        stop_reason = str(status.get("stop_reason", "NONE"))
        last_reject = str(status.get("last_reject", "NONE"))
        rejected_count = int(status.get("local_reject_count", 0))
        accepted = int(status.get("last_accepted_sequence", 0))
        actual_stable = bool(status.get("actual_pose_stable", False))
        view_status = str(status.get("status", ""))

        if (
            bool(status.get("watchdog_hold", False))
            or session_state in {"HOLDING", "STOPPING", "TERMINATED"}
            or stop_reason != "NONE"
            or view_status.startswith("real_error:")
        ):
            self.failure = (
                f"unsafe RT state: session={session_state} stop={stop_reason} "
                f"status={view_status}"
            )
            return

        if rejected_count < self.last_rejected_count:
            self.failure = (
                f"reject count regressed: {self.last_rejected_count} -> {rejected_count}"
            )
            return
        rejection_observed = rejected_count > self.last_rejected_count
        self.last_rejected_count = rejected_count

        if session_state != "RUNNING" or accepted < 1 or not actual_stable:
            self.soak_started_at = None
            self.first_accepted = None
            self.last_accepted = accepted
            return

        if accepted < self.last_accepted:
            self.failure = (
                f"accepted sequence regressed: {self.last_accepted} -> {accepted}"
            )
            return

        # A cumulative reject is not by itself fatal.  Restart the soak and
        # require a fresh rejection-free window; RT HOLD/watchdog states above
        # still fail immediately.
        if rejection_observed or last_reject != "NONE":
            self.soak_started_at = now
            self.first_accepted = accepted
            self.last_accepted = accepted
            return

        if self.soak_started_at is None:
            self.soak_started_at = now
            self.first_accepted = accepted
        self.last_accepted = accepted

        assert self.first_accepted is not None
        accepted_during_soak = accepted - self.first_accepted
        if (
            now - self.soak_started_at >= self.soak_seconds
            and accepted_during_soak >= self.min_accepted
        ):
            self.ready = True
            print(
                "TRACK_READY: sustained current-pose HOLD; "
                f"accepted {accepted_during_soak} updates in "
                f"{now - self.soak_started_at:.2f}s",
                flush=True,
            )

    def poll_failure(self) -> None:
        now = time.monotonic()
        if now >= self.deadline:
            self.failure = "timed out waiting for a sustained rolling HOLD"
        elif self.last_message_at is not None and now - self.last_message_at > 0.5:
            self.failure = "rt_status stream became stale for more than 500 ms"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--soak-seconds", type=float, default=3.0)
    parser.add_argument("--min-accepted", type=int, default=60)
    parser.add_argument("--timeout-seconds", type=float, default=20.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.soak_seconds <= 0.0 or args.min_accepted < 1 or args.timeout_seconds <= 0.0:
        raise ValueError("soak, accepted count and timeout must be positive")
    rclpy.init()
    node = TrackReadyGate(
        soak_seconds=args.soak_seconds,
        min_accepted=args.min_accepted,
        timeout_seconds=args.timeout_seconds,
    )
    try:
        while rclpy.ok() and not node.ready and node.failure is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.poll_failure()
        if node.ready:
            return 0
        print(f"TRACK_NOT_READY: {node.failure or 'ROS shutdown'}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
