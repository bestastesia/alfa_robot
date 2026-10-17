"""No GPU/ROS: scheduling, asynchronous acceptance and continuous splice checks."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from curobo_core.rolling import ExecutionFeedback, RollingSegment, SceneObservation, SliceSchedule
from curobo_core.scene import Pose, RobotState, SceneObject, SceneSnapshot
from curobo_core.trajectory import TimedTrajectory, joint_state_trajectory
from test_core_trajectory import _Tensor
from types import SimpleNamespace


def trajectory(duration=4.):
    # Constant nonzero velocity: arbitrary cutting must not turn it into a stop.
    t = np.array([0., duration / 2, duration])
    return dict(joint_names=["q"], time_from_start_s=t.tolist(),
                positions=t[:, None].tolist(), velocities=np.ones((3, 1)).tolist(),
                accelerations=np.zeros((3, 1)).tolist(), jerks=np.zeros((3, 1)).tolist())


def observation(stamp=10., revision=0):
    scene = SceneSnapshot("model", RobotState(("q",), (0.,), 1),
                          (SceneObject("obstacle", (1., 1., 1.), Pose((5., 0., 0.))),), revision=revision)
    return SceneObservation(scene, stamp, stamp, 30., (("obstacle", (1., 0., 0.)),))


def feedback(segment, now):
    return ExecutionFeedback(segment.trajectory_id, segment.active.joint_names, now, segment.state(now))


class RollingTest(unittest.TestCase):
    def segment(self):
        return RollingSegment(trajectory(), SliceSchedule(4., .6, .2), 10., max_scene_age_s=2.)

    def test_cadence_budget_and_rejections(self):
        schedule = SliceSchedule(4., .6, .2)
        self.assertEqual(schedule.slices, 5)
        self.assertGreaterEqual(schedule.period_s, .6 + .2)
        for args in ((.1, .6, .2), (4, .6, .2, 6), (4, .6, .2, True),
                     (4, 0, .2), (math.nan, .6, .2), (4, .6, -.1)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                SliceSchedule(*args)

    def test_executes_prefix_then_installs_whole_suffix_with_nonzero_velocity(self):
        segment = self.segment()
        obs = observation()
        ticket = segment.begin(obs, feedback(segment, 10), 10.)
        self.assertAlmostEqual(ticket.splice_at, 10.8)
        np.testing.assert_allclose(ticket.initial_state["positions"], [.8])
        np.testing.assert_allclose(ticket.initial_state["velocities"], [1.])
        np.testing.assert_allclose(ticket.remaining["positions"][-1], [4.])
        original_id = segment.trajectory_id
        segment.stage(ticket, ticket.remaining, obs, 10.5, 10.7)
        self.assertEqual(segment.trajectory_id, original_id)  # no early switch!
        segment.activate(feedback(segment, 10.8), obs, 10.8)
        self.assertNotEqual(segment.trajectory_id, original_id)
        obs2 = observation(10.8, 1)
        next_ticket = segment.begin(obs2, feedback(segment, 10.8), 10.8)
        self.assertAlmostEqual(next_ticket.splice_at, 11.6)
        self.assertAlmostEqual(next_ticket.initial_state["positions"][0], 1.6)
        self.assertEqual(next_ticket.remaining["positions"][-1], [4.])

    def test_late_changed_cancelled_and_discontinuous_results_cannot_install(self):
        for kind in ("compute", "link", "scene", "velocity", "goal", "cancel"):
            with self.subTest(kind=kind):
                segment = self.segment(); obs = observation()
                ticket = segment.begin(obs, feedback(segment, 10), 10.)
                path = dict(ticket.remaining)
                ready, received = 10.5, 10.7
                if kind == "compute": ready = 10.65
                if kind == "link": received = 10.75
                if kind == "scene": obs = observation(10., 1)
                if kind == "velocity": path["velocities"] = [[0.]] * len(path["positions"])
                if kind == "goal": path["positions"] = [q[:] for q in path["positions"]]; path["positions"][-1] = [5.]
                if kind == "cancel": segment.fail("timeout", 10.6)
                with self.assertRaises(ValueError):
                    segment.stage(ticket, path, obs, ready, received)
                self.assertEqual(segment.trajectory_id, ticket.trajectory_id)

    def test_safe_arrival_cannot_be_pushed_back_by_every_replan(self):
        segment = self.segment(); obs = observation()
        ticket = segment.begin(obs, feedback(segment, 10), 10.)
        # Same endpoint and derivatives, but another second added to arrival.
        longer = dict(ticket.remaining)
        longer["time_from_start_s"] = [t * 1.5 for t in longer["time_from_start_s"]]
        with self.assertRaisesRegex(ValueError, "postpones"):
            segment.stage(ticket, longer, obs, 10.4, 10.5, previous_suffix_valid=True)
        segment.stage(ticket, ticket.remaining, obs, 10.4, 10.5, previous_suffix_valid=True)
        segment.activate(feedback(segment, 10.8), obs, 10.8)
        self.assertAlmostEqual(segment.active_at + segment.active.duration, 14.)

    def test_changed_scene_and_tracking_error_at_boundary_reject_staged_result(self):
        for changed_scene in (False, True):
            segment = self.segment(); obs = observation()
            ticket = segment.begin(obs, feedback(segment, 10), 10.)
            segment.stage(ticket, ticket.remaining, obs, 10.4, 10.5)
            actual = feedback(segment, 10.8)
            if changed_scene: obs = observation(10., 1)
            else: actual.state["positions"] += .1
            with self.assertRaises(ValueError):
                segment.activate(actual, obs, 10.8)
            self.assertEqual(segment.trajectory_id, ticket.trajectory_id)

    def test_predicts_from_observation_to_future_start_not_receipt_time(self):
        obs = observation()
        swept = obs.swept_snapshot(11., 13.).objects[0]
        np.testing.assert_allclose(swept.pose.position, [7., 0., 0.])
        np.testing.assert_allclose(swept.dimensions_m, [3., 1., 1.])
        with self.assertRaises(ValueError): obs.swept_snapshot(11., 31.)
        with self.assertRaises(ValueError): obs.require_fresh(13., 2.)

    def test_interpolant_preserves_derivatives_and_rejects_between_knot_overshoot(self):
        curve = TimedTrajectory(trajectory())
        values = curve.sample(.7)
        np.testing.assert_allclose(values["positions"], [.7])
        np.testing.assert_allclose(values["velocities"], [1.])
        np.testing.assert_allclose(TimedTrajectory(curve.suffix(.7)).sample(.4)["positions"], [1.1])
        curve.validate_limits([-1], [5], [1.1], [1], [1])
        doc = dict(joint_names=["q"], time_from_start_s=[0., 1.], positions=[[0.], [0.]],
                   velocities=[[4.], [-4.]], accelerations=[[0.], [0.]], jerks=[[0.], [0.]])
        with self.assertRaises(ValueError):
            TimedTrajectory(doc).validate_limits([-1], [.5], [5], [100], [1000])
        with self.assertRaises(ValueError): curve.sample(4.1)

    def test_float32_limit_roundoff_is_not_a_physical_limit_relaxation(self):
        bound = 1.8325957145940461
        rounded = float(np.nextafter(np.float32(bound), np.float32(np.inf)))
        doc = dict(joint_names=["q"], time_from_start_s=[0., 1.], positions=[[rounded], [rounded]],
                   velocities=[[0.], [0.]], accelerations=[[0.], [0.]], jerks=[[0.], [0.]])
        TimedTrajectory(doc).validate_limits([-bound], [bound], [1], [1], [1])
        doc["positions"] = [[bound + 1e-4], [bound + 1e-4]]
        with self.assertRaises(ValueError):
            TimedTrajectory(doc).validate_limits([-bound], [bound], [1], [1], [1])

    def test_final_remainder_does_not_schedule_beyond_goal(self):
        segment = RollingSegment(trajectory(.8), SliceSchedule(.8, .6, .2),
                                 10., max_scene_age_s=2.)
        self.assertIsNone(segment.begin(observation(), feedback(segment, 10.), 10.))

    def test_partial_worker_reads_and_blocked_writes_do_not_block_deadline_thread(self):
        import multiprocessing
        import os
        import queue
        import struct
        import threading
        import time
        from v3_rolling_replan import exchange_worker
        for mode in ("partial_read", "blocked_write", "complete"):
            with self.subTest(mode=mode):
                connection, peer = multiprocessing.Pipe()
                commands, replies = queue.Queue(1), queue.Queue(8)
                stop = threading.Event()
                thread = threading.Thread(target=exchange_worker, args=(connection, commands, replies, stop), daemon=True)
                thread.start()
                try:
                    peer.send({"ready": True})
                    self.assertEqual(replies.get(timeout=1), {"ready": True})
                    if mode == "blocked_write":
                        commands.put_nowait("x" * 5_000_000)  # peer never drains the socket
                    else:
                        commands.put_nowait("request")
                        self.assertEqual(peer.recv(), "request")
                        if mode == "partial_read":
                            # Multiprocessing's message length arrives, body never does.
                            os.write(peer.fileno(), struct.pack("!i", 1024))
                        else:
                            peer.send({"progress": {"phase": "validation"}})
                            peer.send({"success": True, "ready_at": 10.59})
                            self.assertIn("progress", replies.get(timeout=1))
                            self.assertTrue(replies.get(timeout=1)["success"])
                    before = time.perf_counter()
                    with self.assertRaises(queue.Empty):
                        replies.get(timeout=.03)
                    self.assertLess(time.perf_counter()-before, .5)
                finally:
                    stop.set()
                    peer.close()
                    thread.join(1)
                    self.assertFalse(thread.is_alive())
                    connection.close()

    def test_slow_admission_cannot_backdate_a_replacement_at_the_splice(self):
        for accepted_at in (10.81, 11., float("nan")):
            segment = self.segment(); obs = observation()
            ticket = segment.begin(obs, feedback(segment, 10.), 10.)
            with self.subTest(accepted_at=accepted_at), self.assertRaises(ValueError):
                segment.stage(ticket, ticket.remaining, obs, 10.59, 10.70,
                              admission_clock=lambda: accepted_at)
            self.assertIsNone(segment.staged)
            self.assertFalse(any(event["event"] == "staged" for event in segment.events))
        segment = self.segment(); obs = observation()
        ticket = segment.begin(obs, feedback(segment, 10.), 10.)
        segment.stage(ticket, ticket.remaining, obs, 10.59, 10.70, admission_clock=lambda: 10.75)
        self.assertEqual(segment.events[-1]["at"], 10.75)
        self.assertEqual(segment.events[-1]["accepted_latency_s"], .75)

    def test_reply_can_arrive_in_t3_after_a_valid_t2_computation(self):
        segment = self.segment(); obs = observation()
        ticket = segment.begin(obs, feedback(segment, 10.), 10.)
        self.assertAlmostEqual(ticket.compute_deadline, 10.6)
        segment.stage(ticket, ticket.remaining, obs, 10.59, 10.78)
        segment.activate(feedback(segment, 10.8), obs, 10.8)
        self.assertEqual(segment.sequence, 1)

    def test_terminal_safety_evidence_requires_exact_tail_and_fresh_scene(self):
        import copy
        from dataclasses import asdict, replace
        from v3_verify_rolling import terminal_evidence
        old = observation()
        curve = TimedTrajectory(trajectory(1.6))
        tail = curve.suffix(.8)
        end = 10.8 + TimedTrajectory(tail).duration
        scene = replace(old.snapshot, revision=1, objects=(replace(
            old.snapshot.objects[0], pose=Pose((5.8, 0., 0.))),))
        current = replace(old, snapshot=scene, observed_at=10.8, received_at=10.8)
        observed = {**asdict(current), "identity": current.identity}
        certificate = dict(snapshot=old.swept_snapshot(10.8, end).to_dict(), valid_from=10.8, valid_until=end)
        report = dict(started_at=10., initial_trajectory=trajectory(1.6),
                      schedule=dict(duration_s=1.6, slices=2, compute_bound_s=.6, link_bound_s=.2),
                      trajectories=[dict(active_at=10.8, trajectory=tail)], observations=[observed],
                      events=[dict(event="activated", sequence=0),
                              dict(event="terminal_certificate_reused", at=10.8, scene_id=current.identity)],
                      solves=[dict(success=True, sequence=0, trajectory=tail, certificate=certificate)])
        checked, world = terminal_evidence(report)
        self.assertEqual(checked, TimedTrajectory(tail).suffix(0))
        self.assertEqual(world.objects[0].object_id, "obstacle")
        for change in ("missing_event", "missing_certificate", "changed_obstacle", "skipped_tick"):
            bad = copy.deepcopy(report)
            if change == "missing_event": bad["events"].pop()
            if change == "missing_certificate": bad["solves"][0].pop("certificate")
            if change == "changed_obstacle":
                changed = replace(current, velocities=(("obstacle", (3., 0., 0.)),))
                bad["observations"] = [{**asdict(changed), "identity": changed.identity}]
                bad["events"][-1]["scene_id"] = changed.identity
            if change == "skipped_tick": bad["trajectories"][0]["trajectory"] = trajectory(1.)
            with self.subTest(change=change), self.assertRaises(ValueError):
                terminal_evidence(bad)
        report["events"][-1] = dict(event="terminal_validated", at=10.95, safe_ready_at=10.9, scene_id=current.identity)
        report["solves"].append(dict(success=True, terminal=True, observation_id=current.identity, ready_at=10.9,
                                      trajectory=checked, certificate=dict(snapshot=world.to_dict(),
                                                                          valid_from=10.8, valid_until=end)))
        terminal_evidence(report)
        report["solves"][-1]["trajectory"]["positions"][-1][0] += .1
        with self.assertRaisesRegex(ValueError, "executed interval"):
            terminal_evidence(report)

    def test_observation_wire_identity_is_checked(self):
        from dataclasses import asdict
        obs = observation()
        document = {**asdict(obs), "identity": obs.identity}
        self.assertEqual(SceneObservation.from_dict(document), obs)
        document["valid_until"] += 1.
        with self.assertRaisesRegex(ValueError, "identity"):
            SceneObservation.from_dict(document)

    def test_matrix_statistics_keep_failures_and_pool_raw_latency(self):
        from v3_rolling_matrix import summarize_reports
        first = dict(stage="approach", scenario="moving", success=True, trigger_count=2, splice_count=2,
                     solves=[dict(compute_wall_s=.1), dict(compute_wall_s=.3)], events=[])
        failed = dict(stage="approach", scenario="moving", success=False, trigger_count=1, splice_count=0,
                      deadline_miss_count=1, censored_compute_count=1, solves=[], events=[])
        other_seed = dict(stage="approach", scenario="moving", success=True, trigger_count=1, splice_count=1,
                          solves=[dict(compute_wall_s=.9)], events=[])
        summary = summarize_reports(iter([first, failed, other_seed]))["approach/moving/none"]
        self.assertEqual(summary["runs"], 3)
        self.assertEqual(summary["rejected"], 1)
        self.assertEqual(summary["completion_rate"], 2/3)
        self.assertEqual(summary["censored_computes"], 1)
        self.assertEqual(summary["latencies"]["compute_wall_s"]["count"], 3)
        self.assertAlmostEqual(summary["latencies"]["compute_wall_s"]["median"], .3)
        self.assertAlmostEqual(summary["latencies"]["compute_wall_s"]["p95"], .84)

    def test_online_scene_delegates_to_mentor_phase_gate(self):
        import ast
        import types
        source = Path(__file__).resolve().parents[1] / "tools/curobo_core/rolling_curobo.py"
        method = next(n for n in ast.walk(ast.parse(source.read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == "scene")
        namespace = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
        calls = []
        gate = object()
        backend = types.SimpleNamespace(
            set_snapshot=lambda snapshot, **kw: calls.append((snapshot, kw)),
            online_validity=lambda boxes, payload, phase: (calls.append((boxes, payload, phase)) or gate))
        adapter = types.SimpleNamespace(backend=backend, boxes={"left": 24}, payload=False, phase="approach")
        snapshot = observation().snapshot
        self.assertIs(namespace["scene"](adapter, snapshot), gate)
        self.assertEqual(calls[0][1], dict(allow_attached=True, require_locked_state=True))
        self.assertEqual(calls[1], ({"left": 24}, False, "approach"))


    def test_worker_rebuilds_envelope_for_actual_candidate_duration(self):
        import ast
        import types
        # Exercise the actual worker solve method without a CUDA dependency.
        source = Path(__file__).resolve().parents[1] / "tools/curobo_core/rolling_curobo.py"
        method = next(n for n in ast.walk(ast.parse(source.read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == "solve")
        namespace = dict(np=np, math=math, TimedTrajectory=TimedTrajectory,
                         time=types.SimpleNamespace(perf_counter=lambda: 10.1))
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
        for candidate_duration in (3., 5.):
            obs = observation()
            remaining = trajectory(4.)
            ticket = types.SimpleNamespace(sequence=0, splice_at=11., remaining=remaining,
                                           initial_state={}, compute_deadline=12.)
            envelopes = []
            def scene(snapshot):
                envelopes.append(snapshot)
                return object()
            def validate(document, validity):
                if document is remaining:
                    raise ValueError("old suffix blocked")
                return {"test": True}
            backend = types.SimpleNamespace(optimize_rrt_path=lambda *a, **kw: (
                None, {"accepted": True, "trajectory": trajectory(candidate_duration)}))
            adapter = types.SimpleNamespace(scene=scene, validate=validate, backend=backend, boxes={}, payload=False,
                                            check_endpoints=lambda *a: [True, True])
            result = namespace["solve"](adapter, ticket, obs, trajectory(1.), max_duration_s=19.)
            self.assertTrue(result["success"], result.get("error"))
            self.assertEqual(result["certificate"]["valid_until"], 11.+candidate_duration)
            self.assertEqual(envelopes[1].objects[0].dimensions_m[0], 5.)
            self.assertEqual(envelopes[-1].objects[0].dimensions_m[0], 1.+candidate_duration)
            self.assertLess(result["certificate"]["valid_until"], obs.valid_until)

    def test_colliding_fixed_endpoint_is_rejected_before_any_solver_call(self):
        import ast
        import types
        source = Path(__file__).resolve().parents[1] / "tools/curobo_core/rolling_curobo.py"
        method = next(n for n in ast.walk(ast.parse(source.read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == "solve")
        namespace = dict(np=np, math=math, TimedTrajectory=TimedTrajectory,
                         time=types.SimpleNamespace(perf_counter=lambda: 10.1))
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
        for mask, code in (([False, True], "SPLICE_ENDPOINT_COLLISION"),
                           ([True, False], "GOAL_ENDPOINT_COLLISION")):
            # backend has no solve method: an accidental solve would fail this test.
            adapter = types.SimpleNamespace(scene=lambda s: object(), validate=lambda *a: {},
                                            backend=object(), boxes={}, payload=False,
                                            check_endpoints=lambda *a: mask)
            ticket = types.SimpleNamespace(sequence=0, splice_at=11., remaining=trajectory(4.),
                                           initial_state={}, compute_deadline=12.)
            result = namespace["solve"](adapter, ticket, observation(), trajectory(1.), max_duration_s=19.)
            self.assertFalse(result["success"])
            self.assertEqual(result["failure_code"], code)
            self.assertEqual(result["endpoint_validity"], mask)
            self.assertEqual(result["endpoint_check_interval"], [11., 15.])
            self.assertNotIn("attempts", result)
            self.assertNotIn("trajectory", result)

    def test_unsafe_committed_prefix_never_attempts_to_repair_the_later_suffix(self):
        import ast
        import types
        source = Path(__file__).resolve().parents[1] / "tools/curobo_core/rolling_curobo.py"
        method = next(n for n in ast.walk(ast.parse(source.read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == "solve")
        namespace = dict(np=np, math=math, TimedTrajectory=TimedTrajectory,
                         time=types.SimpleNamespace(perf_counter=lambda: 10.1))
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
        def reject_prefix(*args):
            raise ValueError("collision before the splice")
        adapter = types.SimpleNamespace(scene=lambda s: object(), validate=reject_prefix,
                                        backend=object(), boxes={}, payload=False)
        ticket = types.SimpleNamespace(sequence=0, splice_at=11., remaining=trajectory(4.),
                                       initial_state={}, compute_deadline=12.)
        result = namespace["solve"](adapter, ticket, observation(), trajectory(1.), max_duration_s=19.)
        self.assertFalse(result["success"])
        self.assertEqual(result["failure_code"], "UNSAFE_COMMITTED_PREFIX")
        self.assertEqual(result["required_action"], "independent_protective_stop")
        self.assertNotIn("trajectory", result)
        self.assertNotIn("attempts", result)

    def test_failed_optional_optimization_reuses_rechecked_safe_suffix_without_search(self):
        import ast
        import types
        source = Path(__file__).resolve().parents[1] / "tools/curobo_core/rolling_curobo.py"
        method = next(n for n in ast.walk(ast.parse(source.read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == "solve")
        namespace = dict(np=np, math=math, TimedTrajectory=TimedTrajectory,
                         time=types.SimpleNamespace(perf_counter=lambda: 10.1))
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
        calls = []
        def optimize(*args, **kwargs):
            calls.append(kwargs)
            return None, {"accepted": False}
        backend = types.SimpleNamespace(optimize_rrt_path=optimize)
        adapter = types.SimpleNamespace(scene=lambda s: object(), validate=lambda *args: {"valid": True},
                                        backend=backend, boxes={}, payload=False, check_endpoints=lambda *a: [True, True])
        ticket = types.SimpleNamespace(sequence=0, splice_at=11., remaining=trajectory(4.),
                                       initial_state={}, compute_deadline=12.)
        result = namespace["solve"](adapter, ticket, observation(), trajectory(1.), max_duration_s=19.)
        self.assertTrue(result["success"])
        self.assertFalse(result["optimization_success"])
        self.assertEqual(result["strategy"], "revalidated_suffix")
        self.assertEqual(result["trajectory"], ticket.remaining)
        self.assertEqual(len(calls), 1)

    def test_long_prediction_ttl_does_not_mean_long_robot_occupancy(self):
        obs = observation()
        actual = obs.swept_snapshot(11., 13.)
        entire_ttl = obs.swept_snapshot(11., obs.valid_until)
        self.assertEqual(actual.objects[0].dimensions_m, (3., 1., 1.))
        self.assertGreater(entire_ttl.objects[0].dimensions_m[0], 10.)
        # A shorter certificate cannot validate a longer replacement.
        self.assertFalse(obs.covered_by(actual, 11., 14.))

    def test_live_world_update_cannot_change_phase_base_payload_or_safety_policy(self):
        from dataclasses import replace
        from curobo_core.scene import AttachedObject
        for change in ("base", "payload", "policy"):
            segment = self.segment(); first = observation()
            ticket = segment.begin(first, feedback(segment, 10.), 10.)
            segment.stage(ticket, ticket.remaining, first, 10.4, 10.5)
            segment.activate(feedback(segment, 10.8), first, 10.8)
            next_obs = observation(10.8, 1)
            snapshot = next_obs.snapshot
            if change == "base":
                snapshot = replace(snapshot, state=replace(snapshot.state, base_pose=Pose((.1, 0., 0.))))
            if change == "payload":
                snapshot = replace(snapshot, attachments=(AttachedObject('payload', (1., 1., 1.), 'left_tool0', Pose()),))
            if change == "policy":
                snapshot = replace(snapshot, policy=replace(snapshot.policy, max_box_tilt_deg=180.))
            next_obs = replace(next_obs, snapshot=snapshot)
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, "changed during the segment"):
                segment.begin(next_obs, feedback(segment, 10.8), 10.8)
            self.assertIsNone(segment.pending)

    def test_out_of_order_or_silently_mutated_scene_is_rejected(self):
        from dataclasses import replace
        for kind in ("revision", "timestamp", "geometry", "prediction"):
            segment = self.segment(); obs = observation(10., 2)
            ticket = segment.begin(obs, feedback(segment, 10.), 10.)
            segment.stage(ticket, ticket.remaining, obs, 10.4, 10.5)
            segment.activate(feedback(segment, 10.8), obs, 10.8)
            next_obs = observation(10.8, 2)
            if kind == "revision": next_obs = observation(10.8, 1)
            if kind == "timestamp": next_obs = replace(next_obs, observed_at=9.9)
            if kind == "geometry":
                next_obs = replace(next_obs, snapshot=replace(next_obs.snapshot, objects=()), velocities=())
                # Remove the corresponding velocity atomically with the object.
            if kind == "prediction": next_obs = replace(next_obs, velocities=(("obstacle", (2., 0., 0.)),))
            with self.subTest(kind=kind), self.assertRaisesRegex(ValueError, "out-of-order|without a new revision"):
                segment.begin(next_obs, feedback(segment, 10.8), 10.8)

    def test_online_input_cannot_ignore_motion_of_model_locked_joints(self):
        import ast
        from dataclasses import replace
        source = Path(__file__).resolve().parents[1] / "tools/curobo_core/rolling_backend.py"
        method = next(n for n in ast.walk(ast.parse(source.read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == "set_snapshot")
        namespace = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
        backend = SimpleNamespace(model_id="model", robot={"kinematics": {
            "lock_joints": {"head_joint": 0., "head_pitch_joint": 0.}}})
        snapshot = observation().snapshot
        namespace["set_snapshot"](backend, snapshot)  # legacy static omission remains compatible
        with self.assertRaisesRegex(ValueError, "locked joints"):
            namespace["set_snapshot"](backend, snapshot, require_locked_state=True)
        complete = replace(snapshot, state=RobotState(("q", "head_joint", "head_pitch_joint"), (0., 0., 0.), 1))
        namespace["set_snapshot"](backend, complete, require_locked_state=True)
        self.assertIs(backend.snapshot, complete)
        moved = replace(complete, state=complete.state.with_positions({"head_joint": .1}))
        for strict in (False, True):
            with self.subTest(strict=strict), self.assertRaisesRegex(ValueError, "locked joints"):
                namespace["set_snapshot"](backend, moved, require_locked_state=strict)
        self.assertIs(backend.snapshot, complete)

    def test_static_optimizer_cache_key_stays_compatible(self):
        # Exercise the production cache-key block without requiring CUDA.
        import ast
        from dataclasses import asdict
        from curobo_core.scene import AttachedObject, digest
        source = Path(__file__).resolve().parents[1] / "tools/curobo_core/rolling_backend.py"
        tree = ast.parse(source.read_text())
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                      and n.name == "trajectory_optimizer")
        statements = []
        for statement in method.body:
            if isinstance(statement, ast.If) and isinstance(statement.test, ast.Compare):
                break  # stop before solver construction
            statements.append(statement)
        obj = SimpleNamespace(prepare_task=lambda boxes: None, suction_mode="side", contact_offsets={},
                              box_fit={"dimensions": [1, 1, 1]}, snapshot=SimpleNamespace(attachments=()))
        context = dict(self=obj, boxes={"left": 24}, payload=True, digest=digest, asdict=asdict)
        code = compile(ast.Module(body=statements, type_ignores=[]), str(source), "exec")
        exec(code, context)
        legacy = ("loaded", ("left",), "side", digest({}), digest(obj.box_fit))
        self.assertEqual(context["key"], legacy)
        obj.snapshot.attachments = (AttachedObject("wall_box_24", (1, 1, 1), "left_tool0", Pose()),)
        exec(code, context)
        self.assertEqual(context["key"][:-1], legacy)
        self.assertEqual(context["key"][-1], digest([asdict(obj.snapshot.attachments[0])]))

    def test_prefix_envelope_does_not_include_obstacles_arriving_after_splice(self):
        obs = observation()
        prefix = obs.swept_snapshot(10., 11.).objects[0]
        suffix = obs.swept_snapshot(11., 15.).objects[0]
        # The obstacle moves away: checking a 1s prefix against the 5s world
        # incorrectly adds future occupied space to that committed prefix.
        self.assertEqual(prefix.pose.position, (5.5, 0., 0.))
        self.assertEqual(prefix.dimensions_m, (2., 1., 1.))
        self.assertEqual(suffix.pose.position, (8., 0., 0.))
        self.assertEqual(suffix.dimensions_m, (5., 1., 1.))

    def test_updated_prediction_must_fit_the_previously_certified_volume(self):
        from dataclasses import replace
        obs = observation()
        envelope = obs.swept_snapshot(10., 15.)
        later_object = replace(obs.snapshot.objects[0], pose=Pose((7., 0., 0.)))
        later = replace(obs, observed_at=12., received_at=12.,
                        snapshot=replace(obs.snapshot, objects=(later_object,), revision=1))
        self.assertTrue(later.covered_by(envelope, 12., 14.))
        outside = replace(later, snapshot=replace(later.snapshot, objects=(
            replace(later_object, pose=Pose((7., 2., 0.))),)))
        self.assertFalse(outside.covered_by(envelope, 12., 14.))
        self.assertFalse(later.covered_by(replace(envelope, model_id="different"), 12., 14.))

    def test_evidence_audit_rejects_tampered_execution_and_deadline(self):
        import copy
        import tempfile
        from v3_verify_rolling import audit
        segment = self.segment(); obs = observation()
        ticket = segment.begin(obs, feedback(segment, 10.), 10.)
        segment.stage(ticket, ticket.remaining, obs, 10.4, 10.5)
        segment.activate(feedback(segment, 10.8), obs, 10.8)
        report = dict(schedule=dict(duration_s=4., compute_bound_s=.6, link_bound_s=.2, slices=5),
                      success=True, hardware_commands=False, runtime_source_sha256={"test": "hash"},
                      initial_cycle_sha256="hash", initial_trajectory=trajectory(), started_at=10.,
                      completed_at=14., trajectories=[dict(active_at=10.8, trajectory=ticket.remaining)],
                      events=segment.events, observations=[dict(identity=obs.identity, observed_at=10.)],
                      splice_count=1, solves=[], samples=[dict(at=11., **{k: v.tolist() for k, v in segment.state(11.).items()})])
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "robot.urdf").write_text('<robot name="test"><joint name="q"><limit lower="-10" upper="10" velocity="2"/></joint></robot>')
            config = root / "robot.yml"
            config.write_text(f'kinematics:\n  urdf_path: {root / "robot.urdf"}\n  cspace:\n    joint_names: [q]\n    max_acceleration: [1]\n    max_jerk: [1]\n')
            self.assertTrue(audit(report, config)["verified"])
            report["schema_version"] = 2
            report["final_feedback"] = dict(
                trajectory_id=segment.trajectory_id, joint_names=["q"], sampled_at=14., received_at=14.,
                state={k: v.tolist() for k, v in segment.state(14.).items()})
            self.assertTrue(audit(report, config)["verified"])
            bad = copy.deepcopy(report)
            bad["final_feedback"]["state"]["positions"] = [3.9]
            with self.assertRaisesRegex(ValueError, "did not reach"):
                audit(bad, config)
            bad = copy.deepcopy(report)
            next(e for e in bad["events"] if e["event"] == "requested")["sequence"] = 2
            with self.assertRaisesRegex(ValueError, "replanning request"):
                audit(bad, config)
            bad = copy.deepcopy(report); bad["samples"][0]["positions"][0] += .1
            with self.assertRaisesRegex(ValueError, "sample"):
                audit(bad, config)
            bad = copy.deepcopy(report)
            next(e for e in bad["events"] if e["event"] == "staged")["safe_ready_at"] = 10.7
            with self.assertRaisesRegex(ValueError, "deadline"):
                audit(bad, config)

    def test_explicit_moving_obstacle_parameters_are_recorded_without_changing_defaults(self):
        from v3_rolling_replan import observe
        snapshot = observation().snapshot
        state = {"positions": [0.] * 15}
        default = observe(snapshot, [1., 2., 3.], "moving", 0, 10., 0., 5., state)
        explicit = observe(snapshot, [1., 2., 3.], "moving", 0, 10., 0., 5., state, [0., -.025, 0.])
        self.assertEqual(default.velocities[-1], ("rolling_obstacle", (0., .025, 0.)))
        self.assertEqual(explicit.velocities[-1], ("rolling_obstacle", (0., -.025, 0.)))
        self.assertAlmostEqual(default.snapshot.objects[-1].pose.position[1], 1.925)
        self.assertAlmostEqual(explicit.snapshot.objects[-1].pose.position[1], 2.075)
        self.assertNotEqual(default.identity, explicit.identity)
        with self.assertRaises(ValueError):
            observe(snapshot, [float("nan"), 0., 0.], "moving", 0, 10., 0., 5., state)

    def test_queue_ipc_sdk_and_wall_latency_have_separate_measured_scopes(self):
        from v3_rolling_replan import record_reply_timing
        from v3_rolling_matrix import summarize_reports
        result = dict(started_at=10.2, ready_at=10.7,
                      attempts=[dict(solve_time_ms=120.), dict(solve_time_ms=80.)])
        record_reply_timing(result, 10., 10.8)
        for key, value in dict(queue_ipc_s=.2, compute_wall_s=.5, request_to_ready_s=.7, result_ipc_s=.1).items():
            self.assertAlmostEqual(result[key], value)
        self.assertEqual(result["sdk_solve_times_s"], [.12, .08])
        report = dict(stage="approach", scenario="clear", success=True, solves=[result], events=[])
        summary = summarize_reports([report])["approach/clear/none"]["latencies"]
        self.assertAlmostEqual(summary["request_to_ready_s"]["max"], .7)
        self.assertEqual(summary["sdk_solve_s"]["count"], 2)
        self.assertAlmostEqual(summary["queue_ipc_s"]["median"], .2)
        with self.assertRaisesRegex(ValueError, "clock domain"):
            record_reply_timing(dict(started_at=9., ready_at=10.7), 10., 10.8)

    def test_slice_near_existing_knot_does_not_create_picosecond_quintic(self):
        times = np.linspace(0., 2., 81)
        document = dict(joint_names=["q"], time_from_start_s=times.tolist(),
                        positions=(times**2)[:, None].tolist(), velocities=(2*times)[:, None].tolist(),
                        accelerations=np.full((81, 1), 2.).tolist(), jerks=np.zeros((81, 1)).tolist())
        original = TimedTrajectory(document)
        for offset in (-2e-12, 0., 2e-12):
            start, end = .5+offset, 1.5-offset
            sliced = TimedTrajectory(original.window(start, end))
            self.assertGreater(np.diff(sliced.times).min(), .024)
            sliced.validate_limits([-1.], [5.], [5.], [3.], [1.])
            np.testing.assert_allclose(sliced.sample(.3)["positions"], original.sample(start+.3)["positions"], atol=1e-12)
            np.testing.assert_allclose(sliced.sample(0)["velocities"], original.sample(start)["velocities"], atol=1e-12)

    def test_old_matching_feedback_inside_trajectory_is_rejected_for_age(self):
        segment = self.segment()
        actual = feedback(segment, 10.)
        with self.assertRaisesRegex(ValueError, "execution feedback is stale"):
            segment.begin(observation(10.2), actual, 10.2)

    def test_feedback_identity_and_freshness_are_required(self):
        from dataclasses import replace
        for key, value in (("trajectory_id", "wrong"),
                           ("joint_names", ("different",))):
            segment = self.segment()
            actual = replace(feedback(segment, 10.), **{key: value})
            with self.subTest(key=key), self.assertRaises(ValueError):
                segment.begin(observation(), actual, 10.)

    def test_online_serialization_does_not_compress_holds(self):
        state = SimpleNamespace(joint_names=["q"], position=_Tensor([[0.], [0.], [1.]]),
                                velocity=_Tensor([[0.], [0.], [0.]]),
                                acceleration=_Tensor([[0.], [0.], [0.]]),
                                jerk=_Tensor([[0.], [0.], [0.]]), dt=_Tensor([.1]))
        doc = joint_state_trajectory(state, ["q"], preserve_timing=True)
        self.assertEqual(doc["time_from_start_s"], [0., .1, .2])
        self.assertEqual(len(joint_state_trajectory(state, ["q"])["positions"]), 2)


if __name__ == "__main__":
    unittest.main()
