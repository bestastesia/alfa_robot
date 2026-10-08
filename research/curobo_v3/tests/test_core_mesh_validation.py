from pathlib import Path
import sys
import tempfile
import unittest
from importlib.util import find_spec

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from curobo_core.scene import Pose, RobotState, SceneObject, SceneSnapshot


@unittest.skipUnless(all(find_spec(name) is not None for name in ("trimesh", "yourdfpy", "fcl")),
                     "requires mesh dependencies; checked in the dedicated CI step")
class MeshValidationTest(unittest.TestCase):
    def test_mesh_intersection_clearance_and_named_state_validation(self):
        import trimesh
        from curobo_core.mesh_validation import MeshStateChecker

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mesh = root/'box.stl'
            trimesh.creation.box(extents=(.2, .2, .2)).export(mesh)
            urdf = root/'robot.urdf'
            urdf.write_text(f'''<robot name="mesh-test"><link name="root"/>
<link name="first"><collision><geometry><mesh filename="{mesh}"/></geometry></collision></link>
<link name="second"><collision><geometry><mesh filename="{mesh}"/></geometry></collision></link>
<joint name="fixed" type="fixed"><parent link="root"/><child link="first"/><origin xyz="0 0 1"/></joint>
<joint name="slide" type="prismatic"><parent link="root"/><child link="second"/><origin xyz="0 0 1"/><axis xyz="1 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint></robot>''')
            config = {'urdf_path': str(urdf), 'self_collision_ignore': {}, 'lock_joints': {}}
            snapshot = SceneSnapshot('test', RobotState.from_mapping({'slide': .3}, 1), ())
            checker = MeshStateChecker(config, snapshot, {}, ())
            self.assertIsNone(checker.check({'slide': .3}, attached=False, payload_enabled=False))
            hit = checker.check({'slide': .1}, attached=False, payload_enabled=False)
            self.assertEqual(set(hit['pair']), {'robot:first:0', 'robot:second:0'})
            self.assertIn('joint_limit_violation', checker.check({'slide': 2.}, attached=False, payload_enabled=False))
            with self.assertRaises(ValueError):
                checker.check({'slide': float('nan')}, attached=False, payload_enabled=False)
            obstacle = SceneObject('obstacle', (.2, .2, .2), Pose((.3, 0, 1)))
            world = MeshStateChecker(config, SceneSnapshot('test', snapshot.state, (obstacle,)), {}, ())
            hit = world.check({'slide': .3}, attached=False, payload_enabled=False)
            self.assertIn('world:obstacle', hit['pair'])


if __name__ == '__main__':
    unittest.main()
