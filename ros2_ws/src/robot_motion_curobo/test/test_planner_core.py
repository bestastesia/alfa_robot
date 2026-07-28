from __future__ import annotations

import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import pytest

from robot_motion_curobo.curobo_backend import (
    CuroboBackend,
    canonical_joint_indices,
    cuboid_covering_spheres,
    load_robot_config,
)
from robot_motion_curobo.planner_core import (
    AUTHORITY_JOINT_NAMES,
    AttachedPayload,
    BackendResult,
    BackendTrajectory,
    JointSegmentPlannerCore,
    JointValues,
    SegmentRequest,
    reorder_joint_values,
)


def limits():
    return {
        name: ((0.0, 1.0) if name == "updown" else (-math.pi, math.pi))
        for name in AUTHORITY_JOINT_NAMES
    }


def values(offset=0.0, *, reversed_order=False):
    names = tuple(reversed(AUTHORITY_JOINT_NAMES)) if reversed_order else AUTHORITY_JOINT_NAMES
    mapping = {
        name: (0.4 + offset if name == "updown" else 0.01 * index + offset)
        for index, name in enumerate(AUTHORITY_JOINT_NAMES)
    }
    return JointValues(names, tuple(mapping[name] for name in names))


def payload(side):
    return AttachedPayload(
        object_id=f"carried_{side}_box_{1 if side == 'left' else 3}",
        box_id=1 if side == "left" else 3,
        side=side,
        link_name=f"{side}_tool0",
        center_xyz=(0.0, 0.0, 0.15),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        size_xyz=(0.4, 0.4, 0.3),
    )


class FakeBackend:
    joint_names = AUTHORITY_JOINT_NAMES
    joint_limits = limits()

    def __init__(self, *, fail=False, empty=False, delay=0.0):
        self.fail = fail
        self.empty = empty
        self.delay = delay
        self.calls = 0
        self.warmup_count = 1
        self.initialization_count = 1
        self.max_active = 0
        self.active = 0
        self.guard = threading.Lock()
        self.last_payloads = ()

    def plan_cspace(
        self,
        start_positions,
        goal_positions,
        attached_payloads,
        **kwargs,
    ):
        del kwargs
        with self.guard:
            self.active += 1
            self.max_active = max(self.max_active, self.active)
        try:
            self.calls += 1
            self.last_payloads = tuple(attached_payloads)
            if self.delay:
                time.sleep(self.delay)
            if self.fail:
                return BackendResult(False, "fake_plan_failed", solve_time_ms=2.0)
            if self.empty:
                return BackendResult(
                    True,
                    "ok",
                    trajectory=BackendTrajectory(AUTHORITY_JOINT_NAMES, (), ()),
                    solve_time_ms=2.0,
                )
            names = tuple(reversed(AUTHORITY_JOINT_NAMES))
            index = {name: i for i, name in enumerate(AUTHORITY_JOINT_NAMES)}

            def reverse(row):
                return tuple(row[index[name]] for name in names)

            middle = tuple((a + b) * 0.5 for a, b in zip(start_positions, goal_positions))
            trajectory = BackendTrajectory(
                joint_names=names,
                positions=(reverse(start_positions), reverse(middle), reverse(goal_positions)),
                times_s=(0.0, 0.02, 0.04),
                velocities=(tuple(0.0 for _ in names),) * 3,
                accelerations=(tuple(0.0 for _ in names),) * 3,
            )
            return BackendResult(
                True,
                "ok",
                planner_method="fake_curobo_graph",
                trajectory=trajectory,
                solve_time_ms=2.0,
            )
        finally:
            with self.guard:
                self.active -= 1


def request(**overrides):
    fields = dict(
        request_id="test",
        scene_id="scene-A",
        start=values(reversed_order=True),
        goal=values(0.01),
        attached_payloads=(payload("left"), payload("right")),
        force_graph=True,
        max_attempts=3,
        timeout_s=0.5,
    )
    fields.update(overrides)
    return SegmentRequest(**fields)


def test_joint_state_is_reordered_by_name():
    ordered = reorder_joint_values(values(reversed_order=True))
    assert ordered[0] == pytest.approx(0.4)
    assert ordered[-1] == pytest.approx(0.12)


@pytest.mark.parametrize(
    "joint_values,marker",
    [
        (JointValues(AUTHORITY_JOINT_NAMES[:-1], (0.0,) * 12), "contract_mismatch"),
        (
            JointValues(AUTHORITY_JOINT_NAMES + (AUTHORITY_JOINT_NAMES[-1],), (0.0,) * 14),
            "duplicate_joint",
        ),
        (
            JointValues(AUTHORITY_JOINT_NAMES[:-1] + ("extra",), (0.0,) * 13),
            "contract_mismatch",
        ),
        (
            JointValues(AUTHORITY_JOINT_NAMES, (float("nan"),) + (0.0,) * 12),
            "non_finite",
        ),
    ],
)
def test_invalid_joint_states_are_rejected(joint_values, marker):
    with pytest.raises(ValueError, match=marker):
        reorder_joint_values(joint_values)


def test_success_reorders_trajectory_and_forwards_both_payloads():
    backend = FakeBackend()
    core = JointSegmentPlannerCore(backend, expected_scene_id="scene-A")
    result = core.plan(request())
    assert result.success, result.message
    assert result.trajectory.joint_names == AUTHORITY_JOINT_NAMES
    assert result.trajectory.times_s == (0.0, 0.02, 0.04)
    assert result.trajectory.positions[0] == reorder_joint_values(request().start)
    assert {item.side for item in backend.last_payloads} == {"left", "right"}
    assert backend.warmup_count == 1
    assert backend.initialization_count == 1


def test_scene_mismatch_and_backend_failures_are_explicit():
    core = JointSegmentPlannerCore(FakeBackend(), expected_scene_id="scene-A")
    assert "scene_id_mismatch" in core.plan(request(scene_id="wrong")).message

    failed = JointSegmentPlannerCore(FakeBackend(fail=True)).plan(request())
    assert not failed.success
    assert failed.message == "fake_plan_failed"

    empty = JointSegmentPlannerCore(FakeBackend(empty=True)).plan(request())
    assert not empty.success
    assert "trajectory_too_short" in empty.message


def test_non_monotonic_and_endpoint_mismatch_are_rejected():
    backend = FakeBackend()
    original = backend.plan_cspace

    def invalid(*args, **kwargs):
        result = original(*args, **kwargs)
        trajectory = result.trajectory
        return BackendResult(
            True,
            "ok",
            trajectory=BackendTrajectory(
                trajectory.joint_names,
                trajectory.positions,
                (0.0, 0.0, 0.04),
            ),
        )

    backend.plan_cspace = invalid
    result = JointSegmentPlannerCore(backend).plan(request())
    assert not result.success
    assert "non_monotonic_time" in result.message


def test_concurrent_requests_are_serialized_and_planner_is_not_reinitialized():
    backend = FakeBackend(delay=0.03)
    core = JointSegmentPlannerCore(backend)
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(lambda _: core.plan(request(timeout_s=1.0)), range(2)))
    assert all(item.success for item in results)
    assert backend.max_active == 1
    assert backend.calls == 2
    assert backend.warmup_count == 1
    assert backend.initialization_count == 1


def test_request_timeout_is_reported_after_synchronous_backend_returns():
    result = JointSegmentPlannerCore(FakeBackend(delay=0.03)).plan(
        request(timeout_s=0.01)
    )
    assert not result.success
    assert "exceeded_timeout" in result.message


def test_payload_sphere_cover_is_deterministic_and_uses_pose():
    spheres = cuboid_covering_spheres(payload("left"), 0.1)
    assert len(spheres) == 48
    assert all(item[3] > 0.0 for item in spheres)
    mean_z = sum(item[2] for item in spheres) / len(spheres)
    assert mean_z == pytest.approx(0.15)


def test_external_robot_config_paths_resolve_relative_to_yaml(tmp_path):
    (tmp_path / "alfa_robot.urdf").write_text("<robot name='alfa'/>")
    (tmp_path / "collision.yml").write_text("collision_spheres: {}\n")
    config_path = tmp_path / "robot.yml"
    config_path.write_text(
        "kinematics:\n"
        "  asset_root_path: .\n"
        "  urdf_path: alfa_robot.urdf\n"
        "  collision_spheres: collision.yml\n"
    )
    config = load_robot_config(config_path)
    kinematics = config["kinematics"]
    assert kinematics["asset_root_path"] == str(tmp_path.resolve())
    assert kinematics["urdf_path"] == str((tmp_path / "alfa_robot.urdf").resolve())
    assert kinematics["collision_spheres"] == str((tmp_path / "collision.yml").resolve())


def test_locked_joints_are_dropped_from_interpolated_trajectory_by_name():
    source = AUTHORITY_JOINT_NAMES + ("pitch", "turn")
    assert canonical_joint_indices(source) == tuple(range(13))
    reordered = ("pitch",) + tuple(reversed(AUTHORITY_JOINT_NAMES)) + ("turn",)
    indices = canonical_joint_indices(reordered)
    assert tuple(reordered[index] for index in indices) == AUTHORITY_JOINT_NAMES


def test_verified_legacy_curobo_joint_names_map_to_current_contract():
    legacy = tuple(
        name.replace("left_", "left").replace("right_", "right")
        for name in AUTHORITY_JOINT_NAMES
    )
    assert canonical_joint_indices(legacy) == tuple(range(13))


def test_curobo_endpoint_precheck_returns_explicit_boundary_failure():
    class PlannerMustNotRun:
        def plan_cspace(self, **kwargs):
            del kwargs
            raise AssertionError("planner must not run for an infeasible endpoint")

    def run(feasibility):
        backend = object.__new__(CuroboBackend)
        backend._set_payload_geometry = lambda _: None
        backend._world_payload_names = lambda _: ()
        backend._set_world_payloads_enabled = lambda _names, _enabled: ()
        backend._make_joint_state = tuple
        backend._torch = SimpleNamespace(
            cuda=SimpleNamespace(synchronize=lambda: None)
        )
        backend._check_positions_feasible_loaded = lambda _: feasibility
        backend._planner = PlannerMustNotRun()
        return backend.plan_cspace(
            (0.0,) * 13,
            (0.1,) * 13,
            (),
            scene_id="scene-A",
            force_graph=True,
            max_attempts=3,
            timeout_s=0.5,
        )

    start_failure = run((False, True))
    assert not start_failure.success
    assert start_failure.message == "curobo_start_state_in_collision_or_bounds"

    goal_failure = run((True, False))
    assert not goal_failure.success
    assert goal_failure.message == "curobo_goal_state_in_collision_or_bounds"


def test_curobo_interpolated_collision_is_rejected_before_service_success():
    trajectory = BackendTrajectory(
        AUTHORITY_JOINT_NAMES,
        ((0.0,) * 13, (0.05,) * 13, (0.1,) * 13),
        (0.0, 0.02, 0.04),
    )

    class FakeResult:
        def get_interpolated_plan(self):
            return object()

    class FakeTrajoptSolver:
        config = SimpleNamespace(num_seeds=1)

        def solve_cspace(self, *args, **kwargs):
            del args, kwargs
            return FakeResult()

    backend = object.__new__(CuroboBackend)
    backend._set_payload_geometry = lambda _: None
    backend._world_payload_names = lambda _: ()
    backend._set_world_payloads_enabled = lambda _names, _enabled: ()
    backend._make_joint_state = lambda positions: tuple(positions)
    backend._torch = SimpleNamespace(cuda=SimpleNamespace(synchronize=lambda: None))
    backend._check_positions_feasible_loaded = lambda rows: (
        (True, True) if len(rows) == 2 else (True, False, True)
    )
    backend._planner = SimpleNamespace(trajopt_solver=FakeTrajoptSolver())
    backend._result_success = lambda _result: True
    backend._trajectory = lambda _plan: trajectory
    backend._trajopt_finetune_attempts = 0
    backend._trajopt_finetune_dt_scale = 0.75

    result = backend.plan_cspace(
        (0.0,) * 13,
        (0.1,) * 13,
        (),
        scene_id="scene-A",
        force_graph=False,
        max_attempts=1,
        timeout_s=0.5,
    )

    assert not result.success
    assert result.message == "curobo_interpolated_trajectory_collision@1"
