import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from v3_stage_playback import StagePlayback, TcpTrajectoryPreview


class Node:
    def __init__(self, name, **properties):
        self.name = name
        self.visible = properties.get('visible', True)
        self.properties = properties
        self.removed = False

    def remove(self):
        self.removed = True


class Scene:
    def __init__(self):
        self.nodes = []

    def add_frame(self, name, **properties):
        node = Node(name, **properties)
        self.nodes.append(node)
        return node

    add_line_segments = add_frame
    add_label = add_frame


class StagePlaybackTests(unittest.TestCase):
    def setUp(self):
        self.result = dict(frames=[[index] for index in range(7)], phases=['home', 'approach', 'approach', 'attach', 'transport', 'release', 'return_home'], payload=[False, False, False, True, True, False, False], joint_names=['joint'])

    def test_stage_includes_true_start_and_never_next_phase(self):
        playback = StagePlayback()
        transport = playback.select(self.result, 'transport')
        self.assertEqual(transport['_source_indices'], [3, 4])
        self.assertEqual(transport['frames'], [[3], [4]])
        self.assertEqual(transport['payload'], [True, True])
        returning = playback.select(self.result, 'return_home')
        self.assertEqual(returning['frames'], [[5], [6]])
        self.assertEqual(returning['payload'], [False, False])
        self.assertIs(transport, playback.select(self.result, 'transport'))
        self.assertEqual(len(playback.select(self.result)['frames']), 7)
        self.assertEqual(self.result['phases'][3], 'attach')

    def test_missing_stage_has_no_frames_and_new_result_invalidates_cache(self):
        playback = StagePlayback()
        self.assertEqual(playback.select(self.result, 'conveyor_place')['frames'], [])
        previous = playback.select(self.result, 'transport')
        updated = dict(self.result, frames=[[index + 10] for index in range(7)])
        current = playback.select(updated, 'transport')
        self.assertIsNot(previous, current)
        self.assertEqual(current['frames'], [[13], [14]])

    def test_diagnostics_override_selected_stage_and_keep_local_collision_index(self):
        self.result['conveyor_edge_diagnostic'] = dict(frames=[[42], [43]], first_bad_index=1)
        view = StagePlayback().select(self.result, 'approach', 'collision')
        self.assertEqual(view['frames'], [[42], [43]])
        self.assertEqual(view['_source_indices'], [None, None])
        self.assertEqual(view['conveyor_edge_diagnostic']['first_bad_index'], 1)

    def test_world_trace_breaks_navigation_jump_and_options_preserve_data(self):
        scene = Scene()
        preview = TcpTrajectoryPreview.__new__(TcpTrajectoryPreview)
        preview.server = type('Server', (), {'scene': scene})()
        preview.source = None
        preview.current = None
        preview.pose_cache = {}
        preview.nodes = []
        preview.metrics = {}
        preview.poses = lambda result: {side: np.repeat(np.eye(4)[None], 4, axis=0) for side in ('left', 'right')}
        result = dict(frames=[[0., 0.], [0., .1], [1., .1], [1., .2]], joint_names=['base_x', 'joint'])
        poses = preview.poses(result)
        for values in poses.values():
            values[:, 0, 3] = [0., .1, 1.1, 1.2]
        preview.poses = lambda result: poses
        preview.update(result, axis_count=3)
        paths = [node for node in preview.nodes if node.name.endswith('/path')]
        self.assertEqual(len(paths), 2)
        for node in paths:
            self.assertEqual(node.properties['points'].shape, (2, 2, 3))
        self.assertAlmostEqual(preview.metrics['left']['length_m'], .2)
        previous_nodes = list(preview.nodes)
        preview.update(result, show_paths=False, show_axes=True, axis_count=3)
        self.assertTrue(all(not node.visible for node in paths))
        self.assertTrue(all(node.visible for node in preview.nodes if '/axes/' in node.name))
        preview.update(None)
        self.assertTrue(all(node.removed for node in previous_nodes))
        self.assertEqual(preview.metrics, {})

    def test_wrist_trace_uses_rotated_tool_offset_and_breaks_navigation_jump(self):
        scene=Scene()
        preview=TcpTrajectoryPreview.__new__(TcpTrajectoryPreview)
        preview.server=type('Server',(),{'scene':scene})()
        preview.source=None
        preview.current=None
        preview.pose_cache={}
        preview.nodes=[]
        preview.metrics={}
        poses={side:np.repeat(np.eye(4)[None],4,axis=0) for side in ('left','right')}
        for matrices in poses.values():
            matrices[:,:3,:3]=np.array([[0,0,1],[0,1,0],[-1,0,0]])
            matrices[:,0,3]=[0,.1,1.1,1.2]
        preview.poses=lambda result:poses
        geometry={'arms':{side:{'center_tool':[0,0,-.2502]} for side in poses}}
        result=dict(frames=[[0],[0],[1],[1]],joint_names=['base_x'],wrist_guidance_settings={'geometry':geometry})
        preview.update(result)
        wrist_paths=[node for node in preview.nodes if node.name.endswith('/wrist/path')]
        self.assertEqual(len(wrist_paths),2)
        for node in wrist_paths:
            points=node.properties['points']
            self.assertEqual(points.shape,(2,2,3))
            self.assertAlmostEqual(points[0,0,0],-.2502)
            self.assertAlmostEqual(points[-1,-1,0],1.2-.2502)
        preview.update(result,show_paths=False)
        self.assertTrue(all(not node.visible for node in wrist_paths))

    def test_extraction_teleport_is_not_drawn_as_a_continuous_tcp_edge(self):
        scene=Scene()
        preview=TcpTrajectoryPreview.__new__(TcpTrajectoryPreview)
        preview.server=type('Server',(),{'scene':scene})()
        preview.source=None;preview.current=None;preview.pose_cache={};preview.nodes=[];preview.metrics={}
        poses={side:np.repeat(np.eye(4)[None],4,axis=0) for side in ('left','right')}
        for values in poses.values():values[:,0,3]=[0,.1,1.1,1.2]
        preview.poses=lambda result:poses
        result=dict(frames=[[0],[.1],[1.1],[1.2]],joint_names=['joint'],phases=['home','attach','extraction_teleport','transport'])
        preview.update(result)
        paths=[node for node in preview.nodes if node.name.endswith('/path')]
        assert len(paths)==2
        assert all(node.properties['points'].shape==(2,2,3) for node in paths)
        assert np.isclose(preview.metrics['left']['length_m'],.2)


if __name__ == '__main__':
    unittest.main()
