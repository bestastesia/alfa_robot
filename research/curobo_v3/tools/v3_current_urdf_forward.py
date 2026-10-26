import hashlib
import copy
import json
import itertools
import time
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy
import torch

from curobo_core.contracts import PlannerAssets
from curobo_core.adapter import matrix_pose
from curobo_core.scene import AttachedObject, digest
from v3_contact_roll_policy import aligned_grasp_transform, aligned_tool_pose
from v3_forward_precontact_cycle import ForwardPrecontactPlanner
from v3_batched_loaded_search import GpuValidity
from v3_prepare_rezero_profiles import convert_row
from v3_rezero_analytic import RezeroAnalytic
from v3_wall_ik_benchmark import ACTIVE_JOINTS
from v3_zero_model_candidate import COORDINATE_REVISION, ROOT, TOOL_OFFSET_M


MODEL = ROOT / 'generated/zero_model_candidate_20261018'
ATLAS_FOLDER = ROOT / 'generated/forward_precontact_current_urdf/atlases'
TRANSPORT_HEIGHT_STEP_M = .005


def transport_heights(mode, configured, seeds):
    if mode == 'top':
        return np.unique(seeds[:, 0])
    configured = np.asarray(configured, dtype=float)
    count = int(np.ceil((configured.max() - configured.min()) / TRANSPORT_HEIGHT_STEP_M))
    return np.unique(np.round(np.r_[configured, np.linspace(configured.min(), configured.max(), count + 1)], 8))


def current_assets():
    return replace(PlannerAssets(), robot_config=MODEL / 'fixed.yaml',
                   mobile_robot_config=MODEL / 'mobile.yaml',
                   urdf=MODEL / 'robot_rezero_candidate.urdf', named_poses=MODEL / 'named_poses.yaml')


def verify_current_model(assets):
    import yaml

    manifest = json.loads((MODEL / 'manifest.json').read_text())
    if manifest['coordinate_revision'] != COORDINATE_REVISION:
        raise ValueError('当前零位版本与解析适配器不一致')
    for config_path in (assets.robot_config, assets.mobile_robot_config):
        config = yaml.safe_load(config_path.read_text())
        tree = ET.parse(config['kinematics']['urdf_path']).getroot()
        if tree.get('alfa_coordinate_revision') != COORDINATE_REVISION:
            raise ValueError(f'配置引用了旧坐标URDF: {config_path}')
        for side in ('left', 'right'):
            origin = tree.find(f"joint[@name='{side}_tool0_fixed']/origin")
            if not np.allclose(np.fromstring(origin.get('xyz'), sep=' '), [0., 0., TOOL_OFFSET_M], atol=1e-9):
                raise ValueError('当前规划必须使用J7+Z191mm TCP')
    if Path(assets.urdf).resolve() != Path(yaml.safe_load(assets.robot_config.read_text())['kinematics']['urdf_path']).resolve():
        raise ValueError('前端URDF与固定根规划URDF不一致')


def rebuild_transport_atlas(assets, mode, targets):
    sources = dict(side=ROOT / 'generated/selected_placement_20261013/all_solutions.npz',
                   top=ROOT / 'generated/adaptive_raised_cycle_20261016/top_raised20.npz')
    source = sources[mode]
    policy_path = ROOT / 'config/placement_task_policy.json'
    identity = digest(dict(coordinate_revision=COORDINATE_REVISION, targets=targets,
        files={str(path): hashlib.sha256(path.read_bytes()).hexdigest()
               for path in (source, policy_path, assets.urdf, assets.robot_config,
                            Path(__file__), ROOT / 'tools/v3_rezero_analytic.py')}))
    ATLAS_FOLDER.mkdir(parents=True, exist_ok=True)
    path = ATLAS_FOLDER / f'{mode}_{identity[:16]}.npz'
    if path.is_file():
        with np.load(path, allow_pickle=False) as stored:
            if str(stored['identity']) == identity:
                return path
    with np.load(source, allow_pickle=False) as stored:
        seeds = convert_row(list(ACTIVE_JOINTS), stored['joints']).astype(np.float64)
    seeds[:, [7, 14]] = (seeds[:, [7, 14]] + 2 * np.pi) % (2 * np.pi) - np.pi
    solver = RezeroAnalytic()
    model = yourdfpy.URDF.load(assets.urdf, load_meshes=False)
    rebuilt = []
    maximum_error = 0.
    policy = json.loads(policy_path.read_text())
    heights = transport_heights(mode, policy['side']['placement_updown_grid_m'], seeds)
    for height in heights:
        seed = seeds[np.argmin(np.abs(seeds[:, 0] - height))].copy()
        seed[0] = height
        model.update_cfg(dict(zip(ACTIVE_JOINTS, seed)))
        inverse = np.linalg.inv(model.get_transform('arm_carriage', model.base_link))
        options = []
        for arm, side in enumerate(('left', 'right')):
            offset = 1 + 7 * arm
            pose = targets[side + '_tool0']
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_quat(np.roll(pose['quaternion'], -1)).as_matrix()
            matrix[:3, 3] = pose['position']
            solutions = [solver.solve(arm, inverse @ matrix, swivel, seed[offset:offset + 7])
                         for swivel in np.deg2rad(np.arange(-180, 180, 10))]
            rows = np.unique(np.round(np.concatenate(solutions), 8), axis=0)
            options.append(rows)
            for solution in rows:
                model.update_cfg(dict(zip((f'{side}_joint{index}' for index in range(1, 8)), solution)))
                actual = model.get_transform(side + '_tool0', model.base_link)
                error = np.linalg.norm(actual[:3, 3] - pose['position'])
                angle = Rotation.from_matrix(actual[:3, :3]).inv() * Rotation.from_quat(np.roll(pose['quaternion'], -1))
                if error > .002 or angle.magnitude() > np.deg2rad(1):
                    raise ValueError('重建运输库未通过独立URDF FK验收')
                maximum_error = max(maximum_error, float(error))
        rebuilt.extend(np.r_[height, left, right] for left, right in itertools.product(*options))
    if not rebuilt:
        raise ValueError(f'{mode}运输库按新TCP重新求解后为空，停止，不沿用旧库')
    joints = np.unique(np.round(rebuilt, 7), axis=0).astype(np.float32)
    np.savez_compressed(path, joints=joints, identity=identity, coordinate_revision=COORDINATE_REVISION,
                        maximum_position_error_m=maximum_error, source_rows=len(seeds),
                        updown_grid_m=heights, redundancy_step_deg=10.,
                        collision_validation='runtime full robot, payload, ground and envelope filtering')
    return path


class CurrentUrdfForwardPlanner(ForwardPrecontactPlanner):
    analytic_bridge_type = RezeroAnalytic
    navigation_export_enabled = False
    result_directory = ROOT / 'generated/forward_precontact_current_urdf/live'

    def __init__(self, args=None, reference_source=None):
        assets = args or current_assets()
        verify_current_model(assets)
        super().__init__(assets, reference_source=reference_source)
        self.analytic = RezeroAnalytic()
        self.target_values = convert_row(list(ACTIVE_JOINTS), self.target_values)
        self.transport_atlas_paths = {}

    def configure_task(self, boxes):
        result = super().configure_task(boxes)
        self.target_poses = {frame: aligned_tool_pose(pose) for frame, pose in self.target_poses.items()}
        mode = 'top' if self.top_grasp else 'side'
        if mode not in self.transport_atlas_paths:
            policy = json.loads((ROOT / 'config/placement_task_policy.json').read_text())[mode]
            targets = {side + '_tool0': aligned_tool_pose(dict(
                position=[policy['position']['x'], policy['position']['y_' + side],
                          self.top_transport_tcp_z_m if self.top_grasp else policy['position']['z']],
                quaternion=policy['quaternion_wxyz'])) for side in ('left', 'right')}
            self.transport_atlas_paths[mode] = rebuild_transport_atlas(self.args, mode, targets)
        return result

    def contact_poses_for_task(self, boxes):
        return {frame: aligned_tool_pose(pose) for frame, pose in super().contact_poses_for_task(boxes).items()}

    def warmup_resources(self):
        if getattr(self, 'warmup_report', {}).get('complete'):
            return self.warmup_report
        began = time.perf_counter()
        original_task = dict(self.selected_task)
        original_snapshot = self.snapshot
        original_targets = copy.deepcopy(self.target_poses)
        original_flags = (self.top_grasp, self.base_advance, self.planner_kind)
        events = []
        try:
            for boxes in ({'left': 22, 'right': 17}, {'left': 3, 'right': 2}):
                self.configure_task(boxes)
                for payload, mobile in ((False, False), (True, False), (False, True)):
                    checker = self.checker(boxes, payload=payload, mobile=mobile)
                    names = checker.kinematics.joint_names
                    values = torch.tensor([[self.home.get(name, 0.) for name in names]], device='cuda', dtype=torch.float32)
                    GpuValidity(checker, check_ground=True, joint_names=names).mask(values)
            self.configure_task(original_task)
            checker = self.checker(original_task)
            start = np.asarray(self.home_values).copy()
            poses = self._tool_poses(checker, torch.tensor(start, device='cuda', dtype=torch.float32))
            case = dict(task=original_task, contact_joints=start.tolist(), payload=False,
                        tcp_start_local=poses, translation_local={side: [0., 0., 0.] for side in poses})
            self.cartesian_engine.prepare(case)
            for count in (16, 128):
                samples = {side: np.repeat(np.asarray(matrix)[None], count, axis=0) for side, matrix in poses.items()}
                references = np.repeat(start[None], count, axis=0)
                rows, indices, stats = self.cartesian_engine.lm_prior(samples, references)
                if len(rows) != count * 16 or not bool(torch.isfinite(rows).all()):
                    raise RuntimeError('LM预热未完成有效16/64批形状计算')
                events.append(dict(point_count=count, batch_points=min(64, count), seeds_per_point=16,
                                   returned_rows=len(rows), **{key: stats[key] for key in ('lm_ms', 'soft_refine_ms', 'solver_setup_ms')}))
            self.endpoint_ik_cache[('lm', True)] = self.cartesian_engine.solvers[64]
            torch.cuda.synchronize()
            self.warmup_report = dict(complete=True, events=events, coordinate_revision=COORDINATE_REVISION,
                wall_ms=(time.perf_counter() - began) * 1000,
                peak_gpu_allocated_mib=torch.cuda.max_memory_allocated() / 2**20,
                caches_planning_results=False, shared_long_solver=True)
            return self.warmup_report
        finally:
            self.configure_task(original_task)
            self.snapshot = original_snapshot
            self.target_poses = original_targets
            self.top_grasp, self.base_advance, self.planner_kind = original_flags

    def task_attachments(self, boxes):
        return tuple(AttachedObject(f'wall_box_{box_id:02d}', tuple(self.box_fit['dimensions_m']),
                     side + '_tool0', matrix_pose(aligned_grasp_transform(side, self.top_grasp)),
                     (side + '_link7', side + '_tool0')) for side, box_id in boxes.items())

    def full_cycle(self, boxes, progress=lambda text: None):
        with ExitStack() as stack:
            for module in ('v3_task_cycle_planner', 'v3_conveyor_cycle_planner', 'v3_box_rotation_cost'):
                stack.enter_context(patch(module + '.grasp_transform', aligned_grasp_transform))
            result = super().full_cycle(boxes, progress)
        result['coordinate_revision'] = COORDINATE_REVISION
        result['tool_offset_m'] = TOOL_OFFSET_M
        result['transport_atlas_paths'] = {mode: str(path) for mode, path in self.transport_atlas_paths.items()}
        result['real_robot_execution'] = False
        result['warmup'] = getattr(self, 'warmup_report', dict(complete=False))
        return result
