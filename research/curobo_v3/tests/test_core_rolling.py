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

    def test_static_optimizer_cache_key_stays_compatible(self):
        # Exercise the production cache-key block without requiring CUDA.
        import ast
        from dataclasses import asdict
        from curobo_core.scene import AttachedObject, digest
        source = Path(__file__).resolve().parents[1] / "tools/curobo_core/backend.py"
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
            bad = copy.deepcopy(report); bad["samples"][0]["positions"][0] += .1
            with self.assertRaisesRegex(ValueError, "sample"):
                audit(bad, config)
            bad = copy.deepcopy(report)
            next(e for e in bad["events"] if e["event"] == "staged")["safe_ready_at"] = 10.7
            with self.assertRaisesRegex(ValueError, "deadline"):
                audit(bad, config)

    def test_feedback_identity_and_freshness_are_required(self):
        from dataclasses import replace
        for key, value in (("trajectory_id", "wrong"), ("sampled_at", 7.),
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
