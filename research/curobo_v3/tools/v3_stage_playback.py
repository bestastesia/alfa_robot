import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy


PLAYBACK_SCOPES = {
    '全流程': None,
    'RRT 1 · Home → 3cm预接触位': 'approach',
    'RRT 2 · 抽离终点 → 运输位': 'transport',
    'RRT 3 · 运输位 → 龙头车放箱': 'conveyor_place',
    'RRT 4 · 释放后 → Home': 'return_home',
}


class StagePlayback:
    def __init__(self):
        self.source = None
        self.views = {}

    def select(self, result, phase=None, diagnostic=None):
        if result is None:
            return None
        if self.source is not result:
            self.source = result
            self.views.clear()
        key = (phase, diagnostic)
        if key in self.views:
            return self.views[key]
        view = dict(result)
        indices = list(range(len(result.get('frames', []))))
        if diagnostic == 'collision' and result.get('conveyor_edge_diagnostic'):
            edge = result['conveyor_edge_diagnostic']
            view.update(frames=edge['frames'], phases=['conveyor_place'] * len(edge['frames']), payload=[True] * len(edge['frames']))
            indices = [None] * len(edge['frames'])
        elif diagnostic == 'extraction' and result.get('extraction_diagnostic'):
            view.update(result['extraction_diagnostic'])
            indices = [None] * len(view['frames'])
        elif phase is not None:
            indices = [index for index, label in enumerate(result.get('phases', [])) if label == phase]
            if indices:
                if indices != list(range(indices[0], indices[-1] + 1)):
                    raise ValueError('Playback stage must form one continuous frame interval')
                if indices[0] > 0:
                    indices.insert(0, indices[0] - 1)
            for field in ('frames', 'phases', 'payload'):
                view[field] = [result[field][index] for index in indices]
        view['_source_indices'] = indices
        view['_playback_source'] = result
        view['_view_phase'] = phase if diagnostic is None else None
        view['_diagnostic_view'] = diagnostic
        self.views[key] = view
        return view


class TcpTrajectoryPreview:
    def __init__(self, server, urdf_path, home):
        self.server = server
        self.model = yourdfpy.URDF.load(urdf_path, load_meshes=False)
        self.home = dict(home)
        self.current = None
        self.pose_cache = {}
        self.source = None
        self.nodes = []
        self.root = server.scene.add_frame('/motion_preview', show_axes=False)
        self.metrics = {}

    def poses(self, result):
        names = result['joint_names']
        poses = {side: [] for side in ('left', 'right')}
        for row in result['frames']:
            mapping = {**self.home, **dict(zip(names, row))}
            self.model.update_cfg(np.asarray([mapping.get(name, 0.) for name in self.model.actuated_joint_names]))
            base = np.eye(4)
            base[:3, :3] = Rotation.from_euler('z', mapping.get('base_yaw', 0.)).as_matrix()
            base[:2, 3] = (mapping.get('base_x', 0.), mapping.get('base_y', 0.))
            for side in poses:
                poses[side].append(base @ self.model.get_transform(side + '_tool0', self.model.base_link))
        return {side: np.asarray(values) for side, values in poses.items()}

    def update(self, result, show_paths=True, show_axes=True, axis_count=10):
        source = result.get('_playback_source', result) if result else None
        if source is not self.source:
            self.source = source
            self.pose_cache.clear()
        if result is not self.current or axis_count != getattr(self, 'axis_count', None):
            self.current = result
            self.axis_count = axis_count
            for node in self.nodes:
                node.remove()
            self.nodes.clear()
            self.metrics = {}
            if result and result.get('frames'):
                source = result.get('_source_indices', [])
                cache_key = (id(result['frames']), tuple(source))
                if cache_key not in self.pose_cache:
                    if len(self.pose_cache) >= 6:
                        self.pose_cache.clear()
                    self.pose_cache[cache_key] = self.poses(result)
                matrices = self.pose_cache[cache_key]
                names = result['joint_names']
                base_indices = [names.index(name) for name in ('base_x', 'base_y', 'base_yaw') if name in names]
                rows = np.asarray(result['frames'])
                continuous = np.ones(max(0, len(rows) - 1), dtype=bool)
                if 'phases' in result:
                    continuous &= np.asarray(result['phases'][1:])!='extraction_teleport'
                if base_indices:
                    continuous &= np.max(np.abs(np.diff(rows[:, base_indices], axis=0)), axis=1) < 1e-8
                for side, color in [('left', (40, 190, 255)), ('right', (255, 155, 50))]:
                    poses = matrices[side]
                    positions = poses[:, :3, 3]
                    segments = np.stack((positions[:-1], positions[1:]), axis=1)[continuous]
                    if len(segments):
                        self.nodes.append(self.server.scene.add_line_segments('/motion_preview/' + side + '/path', points=segments, colors=color, thickness=3., thickness_units='screen'))
                    wrist_geometry=result.get('wrist_guidance_settings',{}).get('geometry',{}).get('arms',{}).get(side)
                    if wrist_geometry:
                        centers=positions+np.einsum('bij,j->bi',poses[:,:3,:3],np.asarray(wrist_geometry['center_tool']))
                        wrist_segments=np.stack((centers[:-1],centers[1:]),axis=1)[continuous]
                        if len(wrist_segments):
                            wrist_color=(190,100,230) if side=='left' else (90,205,160)
                            self.nodes.append(self.server.scene.add_line_segments('/motion_preview/'+side+'/wrist/path',points=wrist_segments,colors=wrist_color,thickness=2.,thickness_units='screen'))
                        for label,index in [('起点',0),('终点',len(centers)-1)]:
                            self.nodes.append(self.server.scene.add_label(f'/motion_preview/{side}/wrist/{label}',text=f"{'左' if side=='left' else '右'}球腕交点 · {label}",position=centers[index],font_screen_scale=.7))
                    samples = np.unique(np.linspace(0, len(poses) - 1, min(axis_count, len(poses)), dtype=int))
                    for index in samples:
                        self.nodes.append(self.server.scene.add_frame(f'/motion_preview/{side}/axes/{index}', position=positions[index], wxyz=np.roll(Rotation.from_matrix(poses[index, :3, :3]).as_quat(), 1), axes_length=.075, axes_radius=.002))
                    for label, index in [('起点', 0), ('终点', len(poses) - 1)]:
                        self.nodes.append(self.server.scene.add_label(f'/motion_preview/{side}/{label}', text=f"{'左' if side == 'left' else '右'}TCP · {label}", position=positions[index], font_screen_scale=.8))
                    lengths = np.linalg.norm(np.diff(positions, axis=0), axis=1)
                    rotations = (Rotation.from_matrix(poses[:-1, :3, :3]).inv() * Rotation.from_matrix(poses[1:, :3, :3])).magnitude() if len(poses) > 1 else np.array([])
                    self.metrics[side] = dict(length_m=float(lengths[continuous].sum()), rotation_deg=float(np.rad2deg(rotations[continuous]).sum()))
        for node in self.nodes:
            node.visible = show_paths if node.name.endswith(('/path', '/起点', '/终点')) else show_axes
