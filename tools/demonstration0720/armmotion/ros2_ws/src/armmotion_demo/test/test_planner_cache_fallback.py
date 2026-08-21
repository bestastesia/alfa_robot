from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped

from armmotion_demo.planner_adapter import PlannerAdapter


class RunningProcess:
    @staticmethod
    def poll():
        return None


def adapter_with_attempts(attempts, cache_match):
    adapter = PlannerAdapter.__new__(PlannerAdapter)
    adapter.trajectory_cache = SimpleNamespace(enabled=True)
    adapter.trajectory_cache_required = False
    adapter.trajectory_cache_fallback_on_planning_failure = True
    adapter._resolve_cache_match = lambda task, initial_sample: cache_match

    def compute_once(task, *, initial_sample, cache_match):
        attempts.append(cache_match)
        if cache_match is None:
            raise RuntimeError("online failed")
        return SimpleNamespace(metrics={})

    adapter._compute_once = compute_once
    return adapter


def test_start_prewarms_planner_once_and_reuses_session():
    adapter = PlannerAdapter.__new__(PlannerAdapter)
    adapter._planner_process = None
    adapter._service_client = None
    adapter._recapture_client = None
    adapter.startup_ms = 8421.0
    starts = []

    def start_session():
        starts.append(True)
        adapter._planner_process = RunningProcess()

    adapter._start_session = start_session

    assert adapter.start() == pytest.approx(8421.0)
    assert adapter.start() == pytest.approx(8421.0)
    assert starts == [True]


def test_online_planning_failure_falls_back_to_cache():
    attempts = []
    cache_match = SimpleNamespace(path="cache.json.gz")
    adapter = adapter_with_attempts(attempts, cache_match)

    plan = adapter.compute(SimpleNamespace(code="task"), initial_sample=object())

    assert attempts == [None, cache_match]
    assert plan.metrics["cache_fallback_used"] is True
    assert plan.metrics["online_planning_failure"] == "online failed"


def test_online_planning_failure_without_cache_remains_failure():
    attempts = []
    adapter = adapter_with_attempts(attempts, None)

    with pytest.raises(RuntimeError, match="在线规划失败且缓存未命中"):
        adapter.compute(SimpleNamespace(code="task"), initial_sample=object())

    assert attempts == [None]


def viewpose(x: float) -> PoseStamped:
    target = PoseStamped()
    target.header.frame_id = "base_link"
    target.pose.position.x = x
    target.pose.orientation.w = 1.0
    return target


def test_recapture_approach_search_moves_toward_box_until_analytic_success():
    adapter = PlannerAdapter.__new__(PlannerAdapter)
    attempts = []

    def plan_recapture(left, right, current, **options):
        attempts.append((left.pose.position.x, options["allow_numeric_fallback"]))
        if len(attempts) < 4:
            raise RuntimeError("analytic no solution")
        return ["sample"], {"selected_updown": 0.3}

    adapter.plan_recapture = plan_recapture
    samples, metrics = adapter.plan_recapture_with_approach_search(
        viewpose(0.40),
        viewpose(0.40),
        object(),
    )

    assert samples == ["sample"]
    assert attempts == [
        (pytest.approx(0.40), False),
        (pytest.approx(0.41), False),
        (pytest.approx(0.42), False),
        (pytest.approx(0.43), False),
    ]
    assert metrics["viewpose_approach_offset_m"] == pytest.approx(0.03)
    assert metrics["numeric_fallback_used"] is False


def test_recapture_approach_search_uses_original_viewpose_for_numeric_fallback():
    adapter = PlannerAdapter.__new__(PlannerAdapter)
    attempts = []

    def plan_recapture(left, right, current, **options):
        attempts.append((left.pose.position.x, options["allow_numeric_fallback"]))
        if not options["allow_numeric_fallback"]:
            raise RuntimeError("analytic no solution")
        return ["numeric_sample"], {"selected_updown": 0.3}

    adapter.plan_recapture = plan_recapture
    samples, metrics = adapter.plan_recapture_with_approach_search(
        viewpose(0.40),
        viewpose(0.40),
        object(),
    )

    assert samples == ["numeric_sample"]
    assert len(attempts) == 12
    assert attempts[-1] == (pytest.approx(0.40), True)
    assert metrics["numeric_fallback_used"] is True
