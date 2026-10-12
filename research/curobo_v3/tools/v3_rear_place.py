"""Reuse the sequential baseline's rear-place/release stage from any attached state."""
from types import SimpleNamespace
import time

import numpy as np

from curobo_core.contracts import ACTIVE_JOINTS
from curobo_core.scene import SceneStore
from curobo_core.sequential import _SequentialTask, sequential_request


def place_at_rear(planner, snapshot, q, side, seed):
    request = SimpleNamespace(
        mode="sequential_unload", tasks=(("left", -1), ("right", -2)),
        targets=sequential_request(planner).targets, seed=seed,
        support_elbow_rise_m=0., cartesian_geometry_tolerance_m=.0001,
        snapshot=snapshot,
    )
    started = time.perf_counter()
    task = _SequentialTask(planner, request, lambda _message: None)
    task.store = SceneStore(snapshot)
    task.q = np.asarray(q, dtype=float).copy()
    active = [0] + list(range(1 + ("left", "right").index(side) * 7,
                                   8 + ("left", "right").index(side) * 7))
    task.place(side, active)
    task.close_stage_timing()
    return {
        "frames": task.report["frames"], "phases": task.report["phases"],
        "attachments_by_frame": task.report["attachments_by_frame"],
        "events": task.report["predicted_scene_events"],
        "final_q": task.q.tolist(), "final_snapshot": task.store.snapshot().to_dict(),
        "attempts": task.report["attempts"], "stage_audits": task.report["stage_audits"],
        "measured_wall_ms": (time.perf_counter()-started)*1000,
        "stage_timing_ms": task.report["stage_timing_ms"],
    }
