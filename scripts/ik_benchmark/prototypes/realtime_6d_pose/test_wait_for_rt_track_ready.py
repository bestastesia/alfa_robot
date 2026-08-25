#!/usr/bin/python3
"""Regression checks for the supervised rolling-HOLD readiness gate."""

from __future__ import annotations

import json

from std_msgs.msg import String

import wait_for_rt_track_ready as gate_module
from wait_for_rt_track_ready import TrackReadyGate


def _gate() -> TrackReadyGate:
    gate = object.__new__(TrackReadyGate)
    gate.soak_seconds = 3.0
    gate.min_accepted = 60
    gate.deadline = 100.0
    gate.last_message_at = None
    gate.soak_started_at = None
    gate.first_accepted = None
    gate.last_accepted = 0
    gate.last_rejected_count = 0
    gate.ready = False
    gate.failure = None
    return gate


def _status(**overrides: object) -> String:
    payload: dict[str, object] = {
        "session_state": "RUNNING",
        "stop_reason": "NONE",
        "last_reject": "NONE",
        "local_reject_count": 0,
        "last_accepted_sequence": 1,
        "actual_pose_stable": True,
        "status": "real_batch_published_waiting_ack",
        "watchdog_hold": False,
    }
    payload.update(overrides)
    return String(data=json.dumps(payload))


def test_transient_reject_restarts_soak_instead_of_failing(monkeypatch) -> None:
    gate = _gate()
    now = [1.0]
    monkeypatch.setattr(gate_module.time, "monotonic", lambda: now[0])

    gate.on_status(_status(last_accepted_sequence=100))
    now[0] = 2.0
    gate.on_status(
        _status(
            last_accepted_sequence=120,
            last_reject="SESSION_NOT_ACCEPTING",
            local_reject_count=1,
        )
    )

    assert gate.failure is None
    assert gate.soak_started_at == 2.0
    assert gate.first_accepted == 120

    now[0] = 5.1
    gate.on_status(
        _status(
            last_accepted_sequence=181,
            local_reject_count=1,
        )
    )

    assert gate.ready
    assert gate.failure is None


def test_watchdog_holding_is_still_fatal(monkeypatch) -> None:
    gate = _gate()
    monkeypatch.setattr(gate_module.time, "monotonic", lambda: 1.0)

    gate.on_status(
        _status(
            session_state="HOLDING",
            stop_reason="UPDATE_TIMEOUT",
            watchdog_hold=True,
        )
    )

    assert gate.failure is not None
    assert "unsafe RT state" in gate.failure
