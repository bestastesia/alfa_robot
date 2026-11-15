"""CPU checks of the mirrored-frame column plan: the reflection maps and the request it plans."""
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from curobo_core.contracts import ACTIVE_JOINTS
from curobo_core.door_mirror import (base_pose_of, mirror_box, mirror_joints, mirror_object_id, mirror_pose,
                                     mirror_result, mirror_row, mirror_side, mirror_snapshot, mirrored_request)
from curobo_core.door_recenter import door_pair, door_request
from curobo_core.scene import CollisionPolicy, Pose, RobotState, SceneObject, SceneSnapshot


def reflected_point(pose, point):
    """Matrix of a Pose applied to a point (the adapter's convention)."""
    w, x, y, z = pose.quaternion_wxyz
    rotation = np.asarray([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                           [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                           [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return rotation@np.asarray(point)+np.asarray(pose.position)


class MirrorMapTests(unittest.TestCase):
    def test_side_and_box_are_involutions(self):
        for side in ('left', 'right', 'left_tool0', 'rack_support_transfer', 'release'):
            self.assertEqual(mirror_side(mirror_side(side)), side)
        self.assertEqual(mirror_side('left_contact'), 'right_contact')
        for index in range(25):
            self.assertEqual(mirror_box(mirror_box(index)), index)
            self.assertEqual(mirror_box(index) % 5, 4-index % 5)
        self.assertEqual(mirror_object_id('wall_box_20'), 'wall_box_24')
        self.assertEqual(mirror_object_id('conveyor_left'), 'conveyor_left')

    def test_pose_reflection_matches_point_mirroring(self):
        quaternion = np.asarray([math.cos(.4), .1, math.sin(.3), -.2])
        pose = Pose((.3, -.2, .45), tuple(quaternion/np.linalg.norm(quaternion)))
        point = np.asarray([.11, -.07, .05])
        mirrored = mirror_pose(pose)
        expected = reflected_point(mirrored, (point[0], -point[1], point[2]))
        actual = reflected_point(pose, point)
        actual[1] *= -1
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        self.assertEqual(mirror_pose(mirrored), pose)

    def test_rows_and_joints_are_involutions(self):
        row = np.arange(18, dtype=float)*.01+.5
        np.testing.assert_array_equal(mirror_row(mirror_row(row)), row)
        mirrored = mirror_row(row)
        self.assertEqual(mirrored[0], row[0])
        self.assertEqual(mirrored[1], -row[1])
        self.assertEqual(mirrored[2], -row[2])
        self.assertEqual(mirrored[3], row[3])
        np.testing.assert_array_equal(mirrored[4:11], -row[11:18])
        np.testing.assert_array_equal(mirrored[11:18], -row[4:11])
        np.testing.assert_array_equal(mirror_joints(mirror_joints(row[3:])), row[3:])
        base = base_pose_of(mirrored)
        self.assertAlmostEqual(base.position[0], row[0])
        self.assertAlmostEqual(base.position[1], -row[1])
        self.assertAlmostEqual(2*math.atan2(base.quaternion_wxyz[3], base.quaternion_wxyz[0]), -row[2])


def fake_planner(home=None):
    positions = np.zeros(len(ACTIVE_JOINTS)) if home is None else np.asarray(home, dtype=float)
    state = RobotState(tuple(ACTIVE_JOINTS), tuple(positions), 0)
    snapshot = SceneSnapshot('mirror-test', state, (), policy=CollisionPolicy(False, False, max_box_tilt_deg=89.))
    return SimpleNamespace(front=1.2, home=dict(zip(ACTIVE_JOINTS, positions)), snapshot=snapshot,
                           conveyor_center_x_m=.9)


class MirroredRequestTests(unittest.TestCase):
    def test_column_zero_request_mirrors_to_column_four(self):
        planner = fake_planner()
        actual = door_request(planner, 11, 0, set(door_pair(4)))
        virtual = mirrored_request(actual)
        expected = door_request(planner, 11, 4, set(door_pair(0)))
        self.assertEqual(virtual.column, 4)
        self.assertEqual((virtual.lower_box, virtual.upper_box), (19, 24))
        self.assertEqual(virtual.tasks, expected.tasks)
        self.assertEqual(virtual.removed_box_ids, expected.removed_box_ids)
        self.assertEqual(virtual.snapshot.state, expected.snapshot.state)
        self.assertEqual({item.object_id: item for item in virtual.snapshot.objects},
                         {item.object_id: item for item in expected.snapshot.objects})
        # Reflecting a mesh reverses its triangles (a different diagonal split); the world vertices are the
        # other side's.
        def world_vertices(mesh):
            from curobo_core.adapter import pose_matrix
            return frozenset(tuple(np.round((pose_matrix(mesh.pose)@np.asarray([*vertex, 1.]))[:3], 9))
                             for vertex in mesh.vertices_m)
        self.assertEqual({mesh.object_id: world_vertices(mesh) for mesh in virtual.snapshot.meshes},
                         {mesh.object_id: world_vertices(mesh) for mesh in expected.snapshot.meshes})

    def test_mirrored_snapshot_round_trips(self):
        snapshot = door_request(fake_planner(), 5, 3, set(door_pair(1))).snapshot
        round_trip = mirror_snapshot(mirror_snapshot(snapshot))
        self.assertEqual(round_trip.to_dict(), snapshot.to_dict())

    def test_attached_scenes_cannot_be_mirrored(self):
        from curobo_core.scene import AttachedObject
        snapshot = door_request(fake_planner(), 5, 0, ()).snapshot
        attached = AttachedObject('wall_box_15', (.3, .4, .4), 'left_tool0', Pose())
        with self.assertRaises(ValueError):
            mirror_snapshot(SceneSnapshot(snapshot.model_id, snapshot.state, snapshot.objects, (attached,),
                                          meshes=snapshot.meshes))


class MirrorResultTests(unittest.TestCase):
    def source_report(self):
        return dict(
            success=True, joint_names=list(ACTIVE_JOINTS), task={'left': 19, 'right': 24},
            frames=[[0., -.2, math.radians(30.), *([0.]*15)], [0.1, -.21, math.radians(29.), *([0.1]*15)]],
            phases=['right_attach', 'release'], attachments_by_frame=[[], []], payload=[False, False],
            predicted_scene_events=[dict(phase='right_attach', object_id='wall_box_24', frame_index=0,
                                         world_pose={'position': [1., -.4, .3], 'quaternion_wxyz': [1., 0., 0., 0.]}),
                                    dict(phase='release', object_id='wall_box_19', frame_index=1,
                                         world_pose={'position': [1.1, -.425, .2], 'quaternion_wxyz': [1., 0., 0., 0.]},
                                         snapshot={'dropped': True}),
                                    dict(phase='release', object_id='wall_box_24', frame_index=1,
                                         world_pose={'position': [1.1, .425, .2], 'quaternion_wxyz': [1., 0., 0., 0.]},
                                         snapshot={'dropped': True})],
            velocities=[[0.]*18, [.1]*18], accelerations=[[0.]*18, [.2]*18], jerks=[[0.]*18, [.3]*18],
            time_from_start_s=[0., 1.], time_parameterized=True,
            timed_segments=[dict(name='release', frame_start=0, frame_end=1,
                                 positions=[[0.]*18, [.1]*18], velocities=[[0.]*18, [.1]*18],
                                 accelerations=[[0.]*18, [.2]*18], jerks=[[0.]*18, [.3]*18])])

    def request(self):
        objects = tuple(SceneObject(f'wall_box_{value:02d}', (.3, .4, .4), Pose((1.2, (value % 5-2)*.4, .2)))
                        for value in (15, 20, 19, 24))
        snapshot = SceneSnapshot('mirror-test', RobotState(tuple(ACTIVE_JOINTS), (0.,)*15, 0), objects,
                                 policy=CollisionPolicy(False, False, max_box_tilt_deg=89.))
        return SimpleNamespace(snapshot=snapshot, column=0, lower_box=15, upper_box=20,
                               tasks=(('left', 15), ('right', 20)))

    def test_result_mirrors_roles_events_and_rows(self):
        source = self.source_report()
        result = mirror_result(source, None, self.request())
        # Virtual right arm on box 24 -> actual left arm on box 20; virtual left on 19 -> actual right on 15.
        self.assertEqual(result['task'], {'left': 20, 'right': 15})
        self.assertEqual(result['phases'], ['left_attach', 'release'])
        self.assertEqual(result['column'], 0)
        self.assertNotIn('snapshot', result['predicted_scene_events'][1])
        released = {event['object_id']: event for event in result['predicted_scene_events'] if event['phase'] == 'release'}
        self.assertEqual(set(released), {'wall_box_15', 'wall_box_20'})
        # Virtual right on box 24 at +y places it; reflected, box 20 is placed at -y (the actual left arm's side).
        self.assertAlmostEqual(released['wall_box_20']['world_pose']['position'][1], -.425)
        self.assertAlmostEqual(result['frames'][1][1], .21)
        self.assertAlmostEqual(result['frames'][1][2], -math.radians(29.))
        self.assertAlmostEqual(result['velocities'][1][1], -.1)
        self.assertEqual(result['time_from_start_s'], [0., 1.])
        self.assertEqual(result['final_snapshot']['attachments'], ())
        self.assertNotIn('wall_box_20', [item['object_id'] for item in result['final_snapshot']['objects']])

    def test_a_round_that_releases_another_set_is_rejected(self):
        source = self.source_report()
        source['predicted_scene_events'][1]['object_id'] = 'wall_box_21'
        with self.assertRaises(ValueError):
            mirror_result(source, None, self.request())


if __name__ == '__main__':
    unittest.main()
