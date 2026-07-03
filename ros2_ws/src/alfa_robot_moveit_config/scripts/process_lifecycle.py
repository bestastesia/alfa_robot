#!/usr/bin/python3
"""Small helpers for reliable ROS launch subprocess cleanup.

Most operator scripts start ``ros2 launch`` via ``bash -lc`` with
``preexec_fn=os.setsid``.  In that mode the launch process owns a whole process
group containing ``move_group``, ``dual_arm_planner_node`` and support nodes.
Killing only the parent shell is not enough and leaves stale ROS services behind.
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from collections.abc import Callable, Iterable
from typing import TextIO

PLANNER_SERVICE_NAMES = (
    "/dual_arm_planner/run_extract_monitor_next",
    "/dual_arm_planner/run_extract_monitor_full_selected",
    "/dual_arm_planner/configure_extract_monitor",
    "/dual_arm_planner/run_left_extract_demo",
    "/dual_arm_planner/run_box_stack_flow",
    "/dual_arm_planner/plan_and_execute",
)

# Intentionally narrow: only planner stack processes, not ros2_control controllers.
STALE_PLANNER_PATTERN = (
    "dual_arm_planner_node|move_group|"
    "ros2 launch alfa_robot_moveit_config dual_arm_planner.launch.py|"
    "ros2 launch alfa_robot_moveit_config dual_arm_planner"
)


def configure_ros_domain(domain_id: str | int | None, *, auto_base: int = 180, auto_span: int = 32) -> str | None:
    """Configure ``ROS_DOMAIN_ID`` for self-contained benchmark scripts.

    ``auto`` gives every top-level script process a stable private-ish domain,
    which prevents stale services/joint_states from previous manual runs from
    leaking into the new test.  Use ``inherit`` when intentionally connecting to
    an already running ROS graph.
    """
    if domain_id is None:
        return os.environ.get("ROS_DOMAIN_ID")
    text = str(domain_id).strip().lower()
    if text in ("", "inherit", "none"):
        return os.environ.get("ROS_DOMAIN_ID")
    if text == "auto":
        text = str(auto_base + (os.getpid() % max(1, auto_span)))
    try:
        numeric_domain = int(text)
    except ValueError as exc:
        raise ValueError(f"ROS_DOMAIN_ID must be an integer, auto, or inherit: {domain_id!r}") from exc
    if numeric_domain < 0 or numeric_domain > 232:
        raise ValueError(
            f"ROS_DOMAIN_ID={numeric_domain} is outside the FastDDS-safe range 0..232. "
            "Use auto or a smaller explicit value."
        )
    os.environ["ROS_DOMAIN_ID"] = text
    return text


def _wait(process: subprocess.Popen[str], timeout: float) -> bool:
    try:
        process.wait(timeout=max(0.0, timeout))
        return True
    except subprocess.TimeoutExpired:
        return False


def _process_group_id(process: subprocess.Popen[str]) -> int | None:
    try:
        return os.getpgid(process.pid)
    except ProcessLookupError:
        return None


def _process_group_alive(pgid: int | None) -> bool:
    if pgid is None:
        return False
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _wait_process_group_gone(pgid: int | None, timeout: float) -> bool:
    return wait_until(lambda: not _process_group_alive(pgid), timeout=timeout, interval=0.05)


def _signal_process_group_id(pgid: int | None, sig: signal.Signals) -> bool:
    if pgid is None:
        return False
    try:
        os.killpg(pgid, sig)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return False


def _signal_single_process(process: subprocess.Popen[str], sig: signal.Signals) -> bool:
    try:
        process.send_signal(sig)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return False


def terminate_process_tree(
    process: subprocess.Popen[str] | None,
    *,
    interrupt_timeout: float = 5.0,
    terminate_timeout: float = 2.0,
    kill_timeout: float = 2.0,
) -> None:
    """Terminate a subprocess and its process group if possible.

    Safe to call multiple times.  The function first sends SIGINT, then SIGTERM,
    then SIGKILL.  It prefers the process group created by ``os.setsid`` and
    falls back to the single process for callers that did not create a group.
    It waits for the whole process group, not just the launch parent, because
    ``ros2 launch`` may leave children behind after the parent exits.
    """
    if process is None:
        return
    pgid = _process_group_id(process)
    if process.poll() is not None and pgid is None:
        return

    if not _signal_process_group_id(pgid, signal.SIGINT):
        _signal_single_process(process, signal.SIGINT)
    _wait(process, interrupt_timeout)
    if _wait_process_group_gone(pgid, 0.2):
        return

    if not _signal_process_group_id(pgid, signal.SIGTERM):
        _signal_single_process(process, signal.SIGTERM)
    _wait(process, terminate_timeout)
    if _wait_process_group_gone(pgid, terminate_timeout):
        return

    if not _signal_process_group_id(pgid, signal.SIGKILL):
        _signal_single_process(process, signal.SIGKILL)
    _wait(process, kill_timeout)
    _wait_process_group_gone(pgid, kill_timeout)


def wait_until(
    predicate: Callable[[], bool],
    *,
    timeout: float,
    interval: float = 0.1,
) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


def wait_until_services_gone(
    service_exists: Callable[[str], bool],
    service_names: Iterable[str] = PLANNER_SERVICE_NAMES,
    *,
    timeout: float = 15.0,
    interval: float = 0.1,
) -> bool:
    names = tuple(service_names)
    return wait_until(
        lambda: not any(service_exists(name) for name in names),
        timeout=timeout,
        interval=interval,
    )


def any_service_exists(service_exists: Callable[[str], bool], service_names: Iterable[str] = PLANNER_SERVICE_NAMES) -> bool:
    return any(service_exists(name) for name in service_names)


def request_stale_planner_shutdown(pattern: str = STALE_PLANNER_PATTERN, *, stdout: TextIO | int | None = subprocess.DEVNULL) -> None:
    """Ask stale planner/move_group processes to exit with SIGINT.

    This intentionally does not target controller nodes.  It is for one-shot
    benchmark/monitor scripts that own their planner stack.
    """
    subprocess.run(
        ["pkill", "-INT", "-f", pattern],
        stdout=stdout,
        stderr=subprocess.DEVNULL,
        check=False,
    )
