"""CPU-only contracts for one-slice-ahead, fixed-cadence segment replanning.

The caller executes the frozen prefix while a worker computes the entire suffix.
This module never sends hardware commands or treats a timeout as a physical stop.
All timestamps below belong to ONE monotonic clock domain, in seconds.
"""
from dataclasses import dataclass, replace
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .scene import Pose, SceneObject, SceneSnapshot, digest, finite_tuple
from .trajectory import TimedTrajectory


@dataclass(frozen=True)
class SliceSchedule:
    duration_s: float
    compute_bound_s: float
    link_bound_s: float
    slices: int | None = None

    def __post_init__(self):
        if (not all(math.isfinite(x) for x in
                    (self.duration_s, self.compute_bound_s, self.link_bound_s)) or
                self.duration_s <= 0 or self.compute_bound_s <= 0 or self.link_bound_s < 0):
            raise ValueError("positive t1/t2 and nonnegative t3 required")
        maximum = math.floor(self.duration_s / (self.compute_bound_s + self.link_bound_s))
        count = maximum if self.slices is None else self.slices
        if type(count) is not int or not 1 <= count <= maximum:
            raise ValueError("no feasible slicing: require integer 1 <= n <= t1/(t2+t3)")
        object.__setattr__(self, "slices", count)

    @property
    def period_s(self):
        return self.duration_s / self.slices


@dataclass(frozen=True)
class SceneObservation:
    snapshot: SceneSnapshot
    observed_at: float
    received_at: float
    valid_until: float
    velocities: tuple = ()  # (object_id, linear_velocity_in_map_m_s); others explicitly static
    position_error_m: float = 0.0
    velocity_error_m_s: float = 0.0
    source: str = "synthetic"

    def __post_init__(self):
        finite_tuple((self.observed_at, self.received_at, self.valid_until,
                      self.position_error_m, self.velocity_error_m_s))
        if (not self.observed_at <= self.received_at <= self.valid_until or
                min(self.position_error_m, self.velocity_error_m_s) < 0 or not self.source):
            raise ValueError("invalid observation clock/validity/uncertainty/source")
        if type(self.snapshot.revision) is not int or self.snapshot.revision < 0:
            raise ValueError("scene revision must be a nonnegative integer")
        if self.snapshot.frame_id != "map":
            raise ValueError("dynamic inputs must already be transformed into map")
        velocities = tuple((name, finite_tuple(value, 3)) for name, value in self.velocities)
        names = [name for name, _ in velocities]
        if len(set(names)) != len(names) or not set(names) <= {o.object_id for o in self.snapshot.objects}:
            raise ValueError("motion must reference unique observed cuboids")
        object.__setattr__(self, "velocities", velocities)

    @property
    def identity(self):
        return digest((self.snapshot.geometry_key, self.snapshot.revision, self.observed_at,
                       self.valid_until, self.velocities, self.position_error_m,
                       self.velocity_error_m_s, self.source))

    def require_fresh(self, now, max_age_s):
        if (not math.isfinite(now) or not math.isfinite(max_age_s) or max_age_s <= 0 or
                not self.received_at <= now <= self.valid_until or now - self.observed_at > max_age_s):
            raise ValueError("scene is stale or belongs to a different clock")

    def swept_snapshot(self, start, end):
        """Conservative constant-velocity translation envelope; no rotation prediction.

        ponytail: a whole-window AABB may reject feasible timed crossings; use
        time-indexed collision optimization if measured rejection rates justify it.
        The producer must bound its prediction error; unknown motion is NOT static.
        """
        if not self.observed_at <= start <= end <= self.valid_until:
            raise ValueError("trajectory exceeds the obstacle prediction horizon")
        velocities = dict(self.velocities)
        objects = []
        for obj in self.snapshot.objects:
            if obj.object_id not in velocities:
                objects.append(obj)
                continue
            velocity = np.asarray(velocities.get(obj.object_id, (0., 0., 0.)))
            p0 = np.asarray(obj.pose.position) + velocity * (start - self.observed_at)
            p1 = np.asarray(obj.pose.position) + velocity * (end - self.observed_at)
            rotation = Rotation.from_quat(np.roll(obj.pose.quaternion_wxyz, -1)).as_matrix()
            extent = np.abs(rotation) @ (np.asarray(obj.dimensions_m) / 2)
            error = self.position_error_m + self.velocity_error_m_s * (end - self.observed_at)
            dimensions = 2 * (extent + error) + np.abs(p1 - p0)
            objects.append(SceneObject(obj.object_id, tuple(dimensions), Pose(tuple((p0 + p1) / 2))))
        return replace(self.snapshot, objects=tuple(objects))


    def covered_by(self, envelope, start, end):
        """Can the previously checked fixed world conservatively cover this update?

        Full snapshots may explicitly remove obstacles. New/enlarged obstacles,
        changed payload/model/policy/meshes, or predictions outside the old volume
        require a fresh GPU check; an ID match alone is never sufficient.
        """
        candidate = self.swept_snapshot(start, end)
        if (candidate.model_id != envelope.model_id or candidate.frame_id != envelope.frame_id or
                candidate.policy != envelope.policy or candidate.attachments != envelope.attachments or
                candidate.meshes != envelope.meshes or candidate.state.base_pose != envelope.state.base_pose):
            return False
        old = {obj.object_id: obj for obj in envelope.objects}
        for obj in candidate.objects:
            if obj.object_id not in old:
                return False
            before = old[obj.object_id]
            r0 = Rotation.from_quat(np.roll(before.pose.quaternion_wxyz, -1)).as_matrix()
            r1 = Rotation.from_quat(np.roll(obj.pose.quaternion_wxyz, -1)).as_matrix()
            offset = r0.T @ (np.asarray(obj.pose.position) - before.pose.position)
            extent = np.abs(r0.T @ r1) @ (np.asarray(obj.dimensions_m) / 2)
            if np.any(np.abs(offset) + extent > np.asarray(before.dimensions_m) / 2 + 1e-9):
                return False
        return True


@dataclass(frozen=True)
class ExecutionFeedback:
    trajectory_id: str
    joint_names: tuple
    sampled_at: float
    state: dict


@dataclass(frozen=True)
class ReplanTicket:
    sequence: int
    trajectory_id: str
    observation_id: str
    tick_at: float
    splice_at: float
    compute_deadline: float
    initial_state: dict
    remaining: dict


class RollingSegment:
    """Double buffer: only install at the promised boundary, never on solve completion."""

    def __init__(self, trajectory, schedule, started_at, *, max_scene_age_s,
                 max_feedback_age_s=.1, position_tolerance=1e-4, velocity_tolerance=1e-3):
        self.active = TimedTrajectory(trajectory)
        if abs(self.active.duration - schedule.duration_s) > 1e-8:
            raise ValueError("t1 must be the initial optimized segment duration")
        if not all(math.isfinite(x) and x > 0 for x in
                   (max_scene_age_s, max_feedback_age_s, position_tolerance, velocity_tolerance)):
            raise ValueError("positive freshness and tracking tolerances required")
        if not math.isfinite(started_at):
            raise ValueError("finite monotonic start required")
        self.schedule, self.started_at, self.active_at = schedule, started_at, started_at
        self.max_scene_age_s = max_scene_age_s
        self.max_feedback_age_s = max_feedback_age_s
        self.position_tolerance, self.velocity_tolerance = position_tolerance, velocity_tolerance
        self.goal = self.active.sample(self.active.duration)
        self.trajectory_id = digest(trajectory)
        self.sequence = 0
        self.pending = None
        self.staged = None
        self.failed = None
        self.events = []
        self.last_observation = None

    def _require_running(self):
        if self.failed:
            raise ValueError(f"segment requires safety intervention: {self.failed}")

    def fail(self, reason, now):
        self.events.append({"event": "safety_required", "at": now, "reason": reason,
                            "cancelled_sequence": self.pending.sequence if self.pending else None})
        self.failed, self.pending, self.staged = reason, None, None

    def state(self, now):
        return self.active.sample(now - self.active_at)

    def _check_feedback(self, feedback, expected):
        for key, tolerance in (("positions", self.position_tolerance),
                               ("velocities", self.velocity_tolerance)):
            values = np.asarray(feedback[key], dtype=float)
            if (values.shape != expected[key].shape or not np.isfinite(values).all() or
                    np.max(np.abs(values - expected[key])) > tolerance):
                raise ValueError(f"execution {key} disagree with committed trajectory")

    def _check_execution(self, feedback, now, expected):
        if (feedback.trajectory_id != self.trajectory_id or
                tuple(feedback.joint_names) != self.active.joint_names):
            raise ValueError("execution feedback references a different trajectory or joint order")
        if (not math.isfinite(feedback.sampled_at) or
                not 0 <= now - feedback.sampled_at <= self.max_feedback_age_s):
            raise ValueError("execution feedback is stale or from a different clock")
        # Compare at the feedback sampling time; an old but otherwise fresh state
        # must never be compared with a future nominal position.
        self._check_feedback(feedback.state, expected)

    def begin(self, observation, feedback, now):
        self._require_running()
        if self.pending is not None:
            raise ValueError("one in-flight replan only")
        observation.require_fresh(now, self.max_scene_age_s)
        previous = self.last_observation
        if previous is not None:
            if (observation.snapshot.model_id != previous.snapshot.model_id or
                    observation.snapshot.revision < previous.snapshot.revision or
                    observation.observed_at < previous.observed_at or observation.received_at < previous.received_at):
                raise ValueError("out-of-order scene or changed model")
            if observation.snapshot.revision == previous.snapshot.revision and (
                    observation.snapshot.geometry_key != previous.snapshot.geometry_key or
                    observation.velocities != previous.velocities or
                    (observation.position_error_m, observation.velocity_error_m_s) !=
                    (previous.position_error_m, previous.velocity_error_m_s)):
                raise ValueError("scene facts changed without a new revision")
        self.last_observation = observation
        tick_at = self.started_at + self.sequence * self.schedule.period_s
        if now < tick_at or now >= tick_at + self.schedule.period_s:
            raise ValueError("missed scheduling tick")
        self._check_execution(feedback, now, self.state(feedback.sampled_at))
        splice_at = tick_at + self.schedule.period_s
        # The final short remainder is executed to its original endpoint, not stretched.
        if splice_at >= self.active_at + self.active.duration - 1e-9:
            return None
        remaining = self.active.suffix(splice_at - self.active_at)
        self.pending = ReplanTicket(
            self.sequence, self.trajectory_id, observation.identity, tick_at, splice_at,
            min(now + self.schedule.compute_bound_s, splice_at - self.schedule.link_bound_s),
            self.state(splice_at), remaining)
        self.events.append({"event": "requested", "at": now, "sequence": self.sequence,
                            "trajectory_id": self.trajectory_id, "scene_id": observation.identity,
                            "scene_revision": observation.snapshot.revision,
                            "observed_at": observation.observed_at, "splice_at": splice_at})
        return self.pending

    def stage(self, ticket, trajectory, observation, ready_at, received_at, *, previous_suffix_valid=False):
        self._require_running()
        if ticket is not self.pending or ticket.trajectory_id != self.trajectory_id:
            raise ValueError("cancelled or obsolete planning result")
        observation.require_fresh(received_at, self.max_scene_age_s)
        if observation.identity != ticket.observation_id:
            raise ValueError("candidate was not validated against the latest observation")
        if (not ticket.tick_at <= ready_at <= ticket.compute_deadline or
                not ready_at <= received_at <= ticket.splice_at or
                received_at - ready_at > self.schedule.link_bound_s):
            raise ValueError("computation or communication deadline miss")
        candidate = TimedTrajectory(trajectory)
        if previous_suffix_valid and candidate.duration > ticket.remaining["time_from_start_s"][-1] + 1e-6:
            raise ValueError("replacement postpones an already safe arrival")
        if candidate.joint_names != self.active.joint_names:
            raise ValueError("joint order changed")
        self._check_feedback(candidate.sample(0), ticket.initial_state)
        if not np.allclose(candidate.sample(0)["accelerations"], ticket.initial_state["accelerations"],
                           atol=1e-3, rtol=0):
            raise ValueError("acceleration discontinuity at splice")
        self._check_feedback(candidate.sample(candidate.duration), self.goal)
        if not np.allclose(candidate.sample(candidate.duration)["accelerations"],
                           self.goal["accelerations"], atol=1e-3, rtol=0):
            raise ValueError("terminal acceleration changed")
        if ticket.splice_at + candidate.duration > observation.valid_until:
            raise ValueError("candidate outlives the prediction")
        self.staged = candidate
        self.events.append({"event": "staged", "at": received_at, "sequence": ticket.sequence,
                            "safe_ready_at": ready_at,
                            "available_latency_s": ready_at - observation.observed_at,
                            "accepted_latency_s": received_at - observation.observed_at})

    def activate(self, feedback, observation, now):
        self._require_running()
        ticket = self.pending
        if ticket is None or self.staged is None:
            raise ValueError("no validated replacement at boundary")
        observation.require_fresh(now, self.max_scene_age_s)
        if observation.identity != ticket.observation_id:
            raise ValueError("scene changed after candidate validation")
        if abs(now - ticket.splice_at) > 1e-8:
            raise ValueError("activation must use the agreed controller boundary")
        if abs(feedback.sampled_at - now) > 1e-8:
            raise ValueError("boundary activation requires feedback at the splice")
        self._check_execution(feedback, now, ticket.initial_state)
        self.active, self.active_at = self.staged, ticket.splice_at
        self.trajectory_id = digest(self.active.suffix(0))
        self.events.append({"event": "activated", "at": now, "sequence": ticket.sequence,
                            "trajectory_id": self.trajectory_id,
                            "active_latency_s": now - observation.observed_at})
        self.sequence += 1
        self.pending = self.staged = None
