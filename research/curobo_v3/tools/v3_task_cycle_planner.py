import copy
from dataclasses import replace
import math
import time

import numpy as np
import torch

from curobo_core.planner import FullCyclePlanner, CycleBlocked
from curobo_core.fixtures import tasks
from curobo_core.scene import Pose, SceneObject, AttachedObject
from curobo_core.adapter import matrix_pose, pose_matrix
from v3_wall_ik_benchmark import ACTIVE_JOINTS, make_scene
from v3_task_scene import row_obstacles, grasp_transform
from v3_batched_loaded_search import GpuValidity
from v3_contact_validity import TargetContactValidity
from v3_batched_loaded_search import densify
from scipy.spatial.transform import Rotation
from v3_placement_target import placement_tool_poses
from v3_box_geometry import box_sphere_model


class TaskCyclePlanner(FullCyclePlanner):
    def __init__(self, args):
        super().__init__(args)
        self.task_options = tasks() + tasks([1, 0])
        self.task_map = dict(self.task_options)
        self.top_grasp = False
        self.base_advance = 0.0
        self.original_snapshot = self.snapshot
        self.original_targets = copy.deepcopy(self.target_poses)
        self.legacy_box_fit = self.box_fit
        self.box_dims = None  # None: legacy frozen 40-sphere model; MOTION-306 instances set (depth, width, height)
        self.planner_kind = 'informed_connect'

    def configure_task(self, boxes):
        if boxes not in self.task_map.values():
            raise ValueError('Only the original 60 pairs and 34 fourth/fifth-row pairs are supported')
        self.selected_task = boxes
        self.top_grasp = all(box_id // 5 <= 1 for box_id in boxes.values())
        self.base_advance = 0.3 if self.top_grasp else 0.0
        self.box_fit = self.legacy_box_fit if self.box_dims is None else box_sphere_model(self.box_dims)
        dims = tuple(self.box_fit['dimensions_m'])
        centers, obstacles = row_obstacles(self.front, boxes, dims)
        original_scene = make_scene(self.front, 0.9, set(), dims)
        objects = []
        for item in original_scene.cuboid:
            if item.name.startswith('wall_box_'):
                continue
            position = np.asarray(item.pose[:3], dtype=float)
            position[0] -= self.base_advance
            objects.append(SceneObject(item.name, tuple(item.dims), Pose(tuple(position), tuple(item.pose[3:]))))
        for side, box_id in boxes.items():
            position = centers[side].copy()
            position[0] -= self.base_advance
            objects.append(SceneObject(f'wall_box_{box_id:02d}', tuple(self.box_fit['dimensions_m']), Pose(tuple(position))))
        for index, item in enumerate(obstacles):
            position = np.asarray(item['center'], dtype=float)
            position[0] -= self.base_advance
            objects.append(SceneObject(f'task_fill_{index}', tuple(item['dimensions']), Pose(tuple(position))))
        self.snapshot = replace(self.original_snapshot, objects=tuple(objects), revision=self.original_snapshot.revision + 1)
        self.target_poses = placement_tool_poses()
        self.planner_kind = 'informed_connect'
        return centers, obstacles

    def contact_poses_for_task(self, boxes):
        if not self.top_grasp:
            return super().contact_poses_for_task(boxes)
        poses = {}
        for side, box_id in boxes.items():
            item = self.snapshot.object(f'wall_box_{box_id:02d}')
            position = np.asarray(item.pose.position).copy()
            position[2] += item.dimensions_m[2] / 2
            poses[side + '_tool0'] = {'position': position, 'quaternion': [0.0, 1.0, 0.0, 0.0]}
        return poses

    def task_attachments(self, boxes):
        return tuple(AttachedObject(f'wall_box_{box_id:02d}', tuple(self.box_fit['dimensions_m']),
                    side + '_tool0', matrix_pose(grasp_transform(side, self.top_grasp, self.box_fit['dimensions_m'])),
                    (side + '_link7', side + '_tool0')) for side, box_id in boxes.items())

    def target_validity(self, checker, allow_contact=False):
        targets = {}
        contacts = self.contact_poses_for_task(self.selected_task)
        indices = {}
        config = checker.kinematics.config.kinematics_config
        for side, box_id in self.selected_task.items():
            item = self.snapshot.object(f'wall_box_{box_id:02d}')
            targets[side] = dict(center=item.pose.position, dimensions=item.dimensions_m,
                                 contact=contacts[side + '_tool0']['position'])
            sphere_ids = config.get_sphere_index_from_link_name(side + '_link7').cpu().tolist()
            indices[side] = []
            for sphere_id, sphere in zip(sphere_ids, self.robot['kinematics']['collision_spheres'][side + '_link7']):
                center = np.asarray(sphere['center']) - [0,0,0.151]
                if center[2] > -0.05 and center[2] + sphere['radius'] <= 0.005 and np.linalg.norm(center[:2]) + sphere['radius'] <= 0.16:
                    indices[side].append(sphere_id)
        return TargetContactValidity(checker, targets, indices, allow_contact)

    def plan_contact_approach(self, contact, checker, validity, seed):
        started = time.perf_counter()
        self.fk(contact)
        carriage = np.linalg.inv(self.fk_robot.get_transform('arm_carriage',self.fk_robot.base_link))
        poses = {side:self.fk_robot.get_transform(side + '_tool0',self.fk_robot.base_link).copy() for side in ['left','right']}
        swivels = [self.analytic.swivel(index,contact[1+7*index:8+7*index]) for index in range(2)]
        rows = [np.asarray(contact,dtype=float)]
        for distance in np.linspace(0.005,0.03,6):
            next_row = rows[-1].copy()
            for index,side in enumerate(['left','right']):
                target = poses[side].copy()
                target[:3,3] -= target[:3,2]*distance
                offset = 1 + index*7
                solutions = self.analytic.solve(index,carriage@target,swivels[index],rows[-1][offset:offset+7])
                if not len(solutions):
                    return None,dict(failure=f'{side}预抓取反向直线IK无解@{distance:.3f}m')
                next_row[offset:offset+7] = solutions[np.argmin(np.sum((solutions-rows[-1][offset:offset+7])**2,axis=1))]
            rows.append(next_row)
        straight = np.asarray(densify(np.asarray(rows[::-1])))
        for values in straight:
            self.fk(values)
            for side in ['left','right']:
                actual = self.fk_robot.get_transform(side + '_tool0',self.fk_robot.base_link)
                error = actual[:3,3]-poses[side][:3,3]
                axial = np.dot(error,poses[side][:3,2])
                lateral = np.linalg.norm(error-axial*poses[side][:3,2])
                angle = Rotation.from_matrix(actual[:3,:3].T@poses[side][:3,:3]).magnitude()
                if lateral > 0.002 or axial > 0.001 or axial < -0.031 or angle > math.radians(1):
                    return None,dict(failure='短接近直线/朝向验收失败',lateral_error_mm=float(lateral*1000),axial_m=float(axial),orientation_error_deg=float(math.degrees(angle)))
        if not bool(validity.mask(torch.tensor(straight,device='cuda',dtype=torch.float32)).all().item()):
            return None,dict(failure='短接近机器人/其他环境碰撞（目标箱子已排除）',
                             contact_sphere_indices={},
                             diagnostic_frames=straight.tolist())
        strict = self.target_validity(checker)
        approach, stats = self.joint_plan(self.home_values,straight[0],checker,strict,seed)
        if approach is None:
            return None,stats
        self.contact_approach_split = len(approach)
        stats.update(pregrasp_clearance_m=0.03,target_collision_during_final_approach=False,
                     target_collision_during_pregrasp=True,
                     contact_pipeline_ms=(time.perf_counter()-started)*1000)
        return np.concatenate([approach,straight[1:]],axis=0),stats

    def full_cycle(self, boxes, progress=lambda text: None):
        started = time.perf_counter()
        _, obstacles = self.configure_task(boxes)
        advance_frames = []
        advance_ms = 0.0
        names = self.mobile_robot['kinematics']['cspace']['joint_names']
        if self.top_grasp:
            progress('检查底盘向前0.3m的完整边')
            step_started = time.perf_counter()
            rows = np.array([[self.home.get(name, 0.0) for name in names]] * 61, dtype=np.float32)
            rows[:, names.index('base_x')] = np.linspace(-0.3, 0.0, 61)
            checker = self.collision_checker(boxes, mobile=True)
            validity = self.target_validity(checker)
            valid = validity.mask(torch.tensor(rows, device='cuda'))
            torch.cuda.synchronize()
            advance_ms = (time.perf_counter() - step_started) * 1000
            if not bool(valid.all().item()):
                first = int((~valid).nonzero()[0].item())
                rows[:, names.index('base_x')] += self.base_advance
                return dict(success=False, task=boxes, joint_names=names, frames=rows[:first + 1].tolist(),
                            phases=['base_advance_diagnostic'] * (first + 1), payload=[False] * (first + 1),
                            blocked_stage='底盘前进', blocked_details=f'碰撞或越限@frame{first+1}',
                            total_ms=(time.perf_counter()-started)*1000, top_grasp=True, base_advance_m=0.3,
                            generated_obstacles=obstacles, tool_to_box={side:grasp_transform(side, True, self.box_fit['dimensions_m']).tolist() for side in ['left','right']})
            advance_frames = rows
        result = super().full_cycle(boxes, progress)
        rejected = next((attempt for attempt in result.get('attempts',[]) if attempt.get('approach',{}).get('diagnostic_frames')),None)
        if not result['success'] and rejected:
            frames = []
            for values in rejected['approach']['diagnostic_frames']:
                mapping = dict(zip(ACTIVE_JOINTS,values))
                mapping['base_x'] = self.base_advance
                frames.append([float(mapping.get(name,0)) for name in names])
            result['extraction_diagnostic'] = dict(candidate=rejected['candidate'],frames=frames,
                phases=['contact_approach']*len(frames),payload=[False]*len(frames),
                reason=rejected['approach']['failure'],audit={'contact_sphere_indices':rejected['approach']['contact_sphere_indices']})
        local_frames = np.asarray(result['frames'], dtype=float)
        if len(local_frames):
            local_frames[:, names.index('base_x')] += self.base_advance
        if len(advance_frames):
            advance_frames[:, names.index('base_x')] += self.base_advance
            result['frames'] = advance_frames[:-1].tolist() + local_frames.tolist()
            result['phases'] = ['base_advance'] * (len(advance_frames)-1) + result.get('phases', [])
            result['payload'] = [False] * (len(advance_frames)-1) + result.get('payload', [])
        else:
            result['frames'] = local_frames.tolist()
        result.update(top_grasp=self.top_grasp, base_advance_m=self.base_advance, generated_obstacles=obstacles,
                      tool_to_box={side:grasp_transform(side, self.top_grasp, self.box_fit['dimensions_m']).tolist() for side in ['left','right']},
                      total_ms=(time.perf_counter()-started)*1000,
                      coordinate_note='Internal snapshots and predicted events use the advanced-base local frame; output frames and generated obstacles use original map frame')
        result.setdefault('timing_ms', {})['base_advance_check'] = advance_ms
        for event in result.get('predicted_scene_events', []):
            event['frame_index'] += max(0, len(advance_frames)-1)
        if result['success']:
            rows = np.asarray(result['frames'])
            final = dict(zip(names, rows[-1]))
            result['final_base_x_m'] = final['base_x']
            result['final_home_error'] = max(abs(final[name] - self.home[name]) for name in ACTIVE_JOINTS)
            result['max_adjacent_joint_step_deg'] = float(np.rad2deg(np.abs(np.diff(rows[:, [names.index(name) for name in ACTIVE_JOINTS[1:]]], axis=0)).max()))
        self.last_partial = result
        return result
