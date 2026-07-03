#!/usr/bin/env python3
from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import process_lifecycle  # noqa: E402


def list_group_processes(pgid: int, token: str) -> list[str]:
    result = subprocess.run(
        ["ps", "-eo", "pid=,pgid=,args="],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    matches: list[str] = []
    for line in result.stdout.splitlines():
        parts = line.strip().split(maxsplit=2)
        if len(parts) < 3:
            continue
        try:
            line_pgid = int(parts[1])
        except ValueError:
            continue
        if line_pgid == pgid and token in parts[2]:
            matches.append(line)
    return matches


def main() -> int:
    token = f"alfa_lifecycle_test_{os.getpid()}"
    process = subprocess.Popen(
        [
            "bash",
            "-lc",
            f"python3 -c 'import time; time.sleep(60)' {token} & wait",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
        preexec_fn=os.setsid,
    )
    pgid = os.getpgid(process.pid)

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and not list_group_processes(pgid, token):
        time.sleep(0.05)
    before = list_group_processes(pgid, token)
    assert before, "child process did not start"

    process_lifecycle.terminate_process_tree(process, interrupt_timeout=1.0, terminate_timeout=1.0, kill_timeout=1.0)
    assert process.poll() is not None, "parent process is still alive"

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if not list_group_processes(pgid, token):
            break
        time.sleep(0.05)
    after = list_group_processes(pgid, token)
    assert not after, f"child process survived process-group cleanup: {after}"

    process_lifecycle.terminate_process_tree(process)

    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    try:
        os.environ.pop("ROS_DOMAIN_ID", None)
        assert process_lifecycle.configure_ros_domain("inherit") is None
        assert process_lifecycle.configure_ros_domain("42") == "42"
        assert os.environ["ROS_DOMAIN_ID"] == "42"
        assert process_lifecycle.configure_ros_domain("auto", auto_base=200, auto_span=1) == "200"
        assert os.environ["ROS_DOMAIN_ID"] == "200"
        try:
            process_lifecycle.configure_ros_domain("233")
            raise AssertionError("invalid ROS_DOMAIN_ID was accepted")
        except ValueError:
            pass
    finally:
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
