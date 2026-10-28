"""Strict three-step environment using the existing analytic/collision/RRT/time-parameterization components.

No full_cycle call, multi-root selection, candidate retry, or learned future lookup occurs here. A fixed action
is validated once, and a task failure terminates that episode. Actor observations contain current candidates only.
GPU execution must be launched under the global flock; parallel actors must not share this mutable planner.
"""
import copy
import gc
import math
import time
from types import SimpleNamespace

import numpy as np
import torch
from scipy.spatial.transform import Rotation

from curobo_core.contracts import PlannerAssets
from curobo_core.planner import CycleBlocked, FullCyclePlanner
from v3_batched_loaded_search import GpuValidity, batched_rrt_connect_multi_goal, densify
from v3_box_geometry import box_dims, extraction_steps
from v3_box_rotation_cost import BoxRotationCost
from v3_conveyor_cycle_planner import ConveyorCyclePlanner
from v3_scene_variants import DockingOffsetMixin
from v3_single_arm_planner import ActiveRotationCost
from v3_task_cycle_planner import TaskCyclePlanner
from v3_timed_single_arm_planner import TimedSingleArmPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS
from motion311_learning.contracts import Observation, StepResult, reward
from motion311_learning.features import PHYSICS_DIM, PhysicsFeatures, state_features
from motion311_learning.redundancy import RedundancyDecoder


class EpisodeFailure(Exception):
    def __init__(self, reason):
        self.reason = reason
        super().__init__(reason)


def matrix(pose):
    result = np.eye(4)
    result[:3, 3] = pose['position']
    quaternion = np.asarray(pose['quaternion'], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    result[:3, :3] = Rotation.from_quat(np.roll(quaternion, -1)).as_matrix()
    return result


def map_frames(report, base_advance):
    """Timing expects original-map base_x, while planning/suffix use advanced-base local coordinates."""
    result = dict(report)
    frames = np.asarray(report['frames'], dtype=float).copy()
    if len(frames):
        frames[:, report['joint_names'].index('base_x')] += base_advance
    result['frames'] = frames.tolist()
    return result


def validate_checkpoint(state, names, timed):
    if state['boxes'] != state['item']['task'] or state['report']['task'] != state['boxes']:
        raise ValueError('checkpoint task differs from scene')
    if list(state['report']['joint_names']) != list(names):
        raise ValueError('checkpoint joint order differs')
    if state['stage'] not in range(3):
        raise ValueError('checkpoint stage outside episode')
    if timed and (len(state['timed_blocks']) != state['stage'] or
                  any(not block.get('timed_validation', {}).get('success') for block in state['timed_blocks'])):
        raise ValueError('checkpoint prefix has not passed timing checks')
    if not state['report']['frames']:
        raise ValueError('checkpoint lacks physical state')
    indices = [names.index(name) for name in ACTIVE_JOINTS]
    endpoint = np.asarray(state['report']['frames'][-1])[indices]
    if not np.allclose(endpoint, state['previous'], atol=1e-7, rtol=0.):
        raise ValueError('checkpoint endpoint differs from previous state')


class _StagePlanner(DockingOffsetMixin, TimedSingleArmPlanner):
    """Only override the conveyor action boundary; use composition in StageEnv for episode state."""
    suffix_action = None
    suffix_capture = None
    seed_offset = 0

    def conveyor_ik_solver(self, boxes):
        return None

    def plan_conveyor_target(self, start, targets, checker, validity, solver, report, append):
        if self.suffix_capture is not None:
            self.suffix_capture.update(targets={s: m.copy() for s, m in targets.items()})
            raise _Captured()
        if self.suffix_action is None:
            raise RuntimeError("suffix requires exactly one selected place")
        return super().plan_conveyor_target(start, targets, checker, validity, solver, report, append)

    def dual_place_candidates(self, start, targets, validity, solver, report):
        row = torch.tensor(self.suffix_action[None], device='cuda', dtype=torch.float32)
        report['conveyor_goal_ik'] = dict(mode='four-route single action', collision_valid=int(validity.mask(row).sum()))
        return row[validity.mask(row)].contiguous()

    def single_place_candidates(self, targets, start, checker):
        row = torch.tensor(self.suffix_action[None], device='cuda', dtype=torch.float32)
        return self.suffix_action[None].astype(np.float32) if bool(GpuValidity(checker, check_ground=True).mask(row).all()) else np.empty((0, 15), dtype=np.float32)


class _Captured(Exception):
    pass


def configure_planner(planner, item, boxes):
    """Scene parameters of one registry instance (docking offset, optional MOTION-306 box dims), then the task."""
    planner.scene_offset = tuple(item['offset_m'])
    planner.box_dims = None if item.get('box_dims') is None else box_dims(item['box_dims'])
    return planner.configure_task(boxes)


class StageEnv:
    caps = (64, 512, 64)

    def __init__(self, timed=True, physical=False, transport_budget_s=2., seed=311):
        self.planner = _StagePlanner(PlannerAssets(), .025)
        self.decoder = RedundancyDecoder(self.planner)
        self.physical = PhysicsFeatures(self.planner)
        self.include_physics = physical
        self.timed = timed
        self.transport_budget_s = float(transport_budget_s)
        self.seed = int(seed)
        self.velocity = self.planner._timed_limits[0][[self.planner._timed_joint_names.index(n) for n in ACTIVE_JOINTS]]
        self.names = list(self.planner._timed_joint_names)
        self.active_indices = [self.names.index(n) for n in ACTIVE_JOINTS]
        self.stage = 0
        self.done = True

    def reset(self, item, seed=None, continuous=False):
        self.item = copy.deepcopy(item)
        self.boxes = {s: int(b) for s, b in item['task'].items()}
        self.seed = self.seed if seed is None else int(seed)
        self.continuous = bool(continuous)
        p = self.planner
        configure_planner(p, item, self.boxes)
        if p.top_grasp:
            rows = np.array([[p.home.get(name, 0.) for name in self.names]] * 61, dtype=np.float32)
            rows[:, self.names.index('base_x')] = np.linspace(-.3, 0., 61)
            mobile_checker = p.collision_checker(self.boxes, mobile=True)
            if not bool(p.target_validity(mobile_checker).mask(torch.tensor(rows, device='cuda')).all()):
                self.done = True
                raise EpisodeFailure('scene_base_advance_collision')
        self.previous = np.asarray(p.home_values, dtype=np.float64).copy()
        self.home = self.previous.copy()
        self.stage, self.done, self.cost_s = 0, False, 0.
        self.report = dict(success=False, task=self.boxes, joint_names=self.names, frames=[], phases=[], payload=[], attempts=[])
        self.timed_blocks = []
        self.append([self.home], 'home', False)
        self.carry_targets = {s: matrix(p.target_poses[s + '_tool0']) for s in self.boxes}
        self.targets = {s: matrix(p.contact_poses_for_task(self.boxes)[s + '_tool0']) for s in self.boxes}
        self.box_centers = {s: np.asarray(p.snapshot.object(f'wall_box_{b:02d}').pose.position) for s, b in self.boxes.items()}
        self.empty = p.checker(self.boxes)
        self.loaded = p.checker(self.boxes, payload=True)
        self.empty_valid = GpuValidity(self.empty, check_ground=True)
        self.loaded_valid = GpuValidity(self.loaded, check_ground=True)
        if not bool(self.empty_valid.mask(torch.tensor(self.home[None], device='cuda', dtype=torch.float32)).all()):
            self.done = True
            raise EpisodeFailure('scene_home_collision')
        return self.observe()

    def append(self, rows, phase, payload):
        for row in rows:
            if len(row) == 15:
                values = np.zeros(len(self.names))
                values[self.active_indices] = row
            else:
                values = np.asarray(row, dtype=float).copy()
            # Keep phase endpoints: the timed planner needs the start of each phase even when duplicated.
            self.report['frames'].append(values.tolist())
            self.report['phases'].append(phase)
            self.report['payload'].append(bool(payload))

    def observe(self):
        if self.done:
            raise RuntimeError("episode has terminated")
        if self.stage == 1:
            self.targets = self.carry_targets
        elif self.stage == 2:
            # A current-stage target is deterministic from the already chosen carry state.
            self.targets = self.planner.conveyor_place_targets(self.previous)[0]
        began = time.perf_counter()
        if self.continuous:
            rows, actions = np.empty((0, 15)), np.empty((0, 3))
        else:
            rows, actions = self.candidate_pool()
        self.generation_ms = (time.perf_counter() - began) * 1000
        feature_began = time.perf_counter()
        physical = self.physical.candidates(rows, self.previous, self.targets, self.boxes) if self.include_physics else np.zeros((len(rows), PHYSICS_DIM))
        state = state_features(self.stage, self.previous, self.home, self.targets, self.carry_targets, self.box_centers,
                               self.boxes, self.planner.top_grasp, self.planner.scene_offset, self.cost_s)
        targets = np.array([self.targets.get(s, np.eye(4)) for s in ('left', 'right')])
        self.observation = Observation(self.stage, state, rows, actions, physical, targets)
        self.feature_ms = (time.perf_counter() - feature_began) * 1000
        return self.observation

    def candidate_pool(self):
        """Current-stage analytic candidate rows and their normalized redundancy actions."""
        return self.decoder.candidate_set(self.targets, self.previous, self.caps[self.stage], self.stage)

    def _path_cost(self, rows):
        if rows is None or len(rows) < 2:
            return 0.
        return float(np.max(np.abs(np.diff(np.asarray(rows), axis=0)) / self.velocity, axis=1).sum())

    def _check_pose(self, row):
        p = self.planner
        p.fk(row)
        for side, target in self.targets.items():
            actual = p.fk_robot.get_transform(side + '_tool0', p.fk_robot.base_link)
            angle = Rotation.from_matrix(actual[:3, :3].T @ target[:3, :3]).magnitude()
            if np.linalg.norm(actual[:3, 3] - target[:3, 3]) > .002 or angle > math.radians(1):
                raise EpisodeFailure('action_pose_error')

    def contact_checks(self, row):
        """Deterministic contact gates without search: collision, extraction and the single-arm pre-approach.
        Returns (extracted, backwards); backwards is None for dual-arm tasks."""
        p = self.planner
        if not bool(self.empty_valid.mask(torch.tensor(row[None], device='cuda', dtype=torch.float32)).all()):
            raise EpisodeFailure('contact_collision')
        if len(self.boxes) == 2:
            extracted, detail = FullCyclePlanner.analytic_extract(p, row, self.empty_valid, self.loaded_valid)
            if extracted is None:
                raise EpisodeFailure('extraction_invalid:' + str(detail))
            return extracted, None
        extracted, detail = p.cartesian_single(row, extraction_steps(p.box_fit['dimensions_m']))
        if extracted is None:
            raise EpisodeFailure('extraction_invalid:' + str(detail))
        if not bool(self.empty_valid.mask(torch.tensor(extracted, device='cuda', dtype=torch.float32)).all()) or not bool(self.loaded_valid.mask(torch.tensor(extracted[-1:], device='cuda', dtype=torch.float32)).all()):
            raise EpisodeFailure('extraction_collision')
        backwards, detail = p.cartesian_single(row, np.linspace(.005, .03, 6), True)
        if backwards is None or not bool(self.empty_valid.mask(torch.tensor(backwards, device='cuda', dtype=torch.float32)).all()):
            raise EpisodeFailure('preapproach_invalid')
        return extracted, backwards

    def contact_collision_free(self, rows):
        """Batched form of contact_checks' first gate (empty-scene collision) for a whole pool."""
        rows = torch.tensor(np.asarray(rows, dtype=np.float32).reshape(-1, 15), device='cuda')
        return self.empty_valid.mask(rows).cpu().numpy() if len(rows) else np.zeros(0, dtype=bool)

    def carry_checks(self, rows):
        """Deterministic carry gates for a batch of goals: payload collision and the single-arm 1.05 m envelope.
        Returns a reason per row, None when the row passes."""
        rows = torch.tensor(np.asarray(rows, dtype=np.float32).reshape(-1, 15), device='cuda')
        if not len(rows):
            return []
        valid = self.loaded_valid.mask(rows).cpu().numpy()
        reasons = [None if ok else 'carry_collision' for ok in valid]
        if len(self.boxes) == 1:
            radius = self.planner.full_envelope_radius(self.loaded, rows).cpu().numpy()
            reasons = [reason or ('carry_envelope' if value > 1.05 else None) for reason, value in zip(reasons, radius)]
        return reasons

    def _contact(self, row):
        p = self.planner
        extracted, backwards = self.contact_checks(row)
        if len(self.boxes) == 2:
            approach, stats = TaskCyclePlanner.plan_contact_approach(p, row, self.empty, self.empty_valid, 20261003)
            if approach is None:
                raise EpisodeFailure('approach_budget_or_validation')
            split = p.contact_approach_split
            self.append(approach[:split], 'approach', False)
            self.append(approach[max(0, split - 1):], 'contact_approach', False)
            cost = self._path_cost(approach) + self._path_cost(extracted)
        else:
            approach, stats = p.single_path(p.home_values, [backwards[-1]], self.empty, strict=p.target_validity(self.empty))
            if approach is None:
                raise EpisodeFailure('approach_budget_or_validation')
            self.append(approach, 'approach', False)
            self.append(backwards[::-1], 'contact_approach', False)
            cost = self._path_cost(approach) + self._path_cost(backwards) + self._path_cost(extracted)
        self.append([row], 'attach', True)
        self.append(extracted, 'extract', True)
        return np.asarray(extracted[-1], dtype=float), cost

    def _carry(self, row):
        p = self.planner
        reason = self.carry_checks(row[None])[0]
        if reason is not None:
            raise EpisodeFailure(reason)
        if len(self.boxes) == 1:
            path, stats = p.single_path(self.previous, [row], self.loaded, rotation=True, budget=self.transport_budget_s)
        else:
            start = torch.tensor(self.previous, device='cuda', dtype=torch.float32)
            goals = torch.tensor(row[None], device='cuda', dtype=torch.float32)
            valid = GpuValidity(self.loaded, check_ground=True)
            valid.box_rotation_cost = BoxRotationCost(self.loaded, ACTIVE_JOINTS, start, goals[0], p.top_grasp)
            valid.collision_weights = torch.tensor([5.] + [1.] * 14, device='cuda')
            lower, upper = self.loaded.kinematics.get_joint_limits().position
            path, stats = batched_rrt_connect_multi_goal(start, goals, lower + 1e-5, upper - 1e-5, valid,
                                                        self.transport_budget_s, 621648851 + self.seed, stop_at_first_valid=True)
            if path is not None:
                path = np.asarray(densify(path))
                if not bool(valid.mask(torch.tensor(path, device='cuda', dtype=torch.float32)).all()):
                    raise EpisodeFailure('transport_dense_validation')
        if path is None:
            raise EpisodeFailure('transport_budget_exhausted')
        self.append(path, 'transport', True)
        return np.asarray(path[-1], dtype=float), self._path_cost(path)

    def _place(self, row):
        p = self.planner
        p.suffix_action = row
        mark = len(self.report['frames'])
        try:
            ConveyorCyclePlanner.conveyor_suffix(p, self.boxes, self.report, self.append, lambda text: None)
        except CycleBlocked as error:
            raise EpisodeFailure(error.stage) from error
        finally:
            p.suffix_action = None
        frames = np.asarray(self.report['frames'][max(0, mark - 1):])[:, self.active_indices]
        phases = self.report['phases'][max(0, mark - 1):]
        cost = sum(self._path_cost(frames[i - 1:i + 1]) for i in range(1, len(frames)) if phases[i] in ('conveyor_place', 'return_home'))
        return self.home.copy(), cost

    def _validate_timed_stage(self, start_index):
        """Time/check just this action's newly appended phases before exposing the next state.
        Past prefixes cannot secretly fail only after a later action; saved timed blocks are assembled at the end."""
        block = dict(task=self.boxes, joint_names=self.names,
                     **{key: copy.deepcopy(self.report[key][start_index:]) for key in ('frames', 'phases', 'payload')})
        block = map_frames(block, self.planner.base_advance)
        self.planner.time_parameterize_result(block)
        self.timed_blocks.append(block)

    def _assemble_timing(self):
        from curobo_core.trajectory import assemble_timed_cycle, validate_timed_cycle
        frames, phases, payload, segments = [], [], [], []
        for block in self.timed_blocks:
            offset = len(frames)
            for segment in block['timed_segments']:
                item = dict(segment, frame_start=segment['frame_start'] + offset, frame_end=segment['frame_end'] + offset)
                segments.append(item)
            frames.extend(block['frames']); phases.extend(block['phases']); payload.extend(block['payload'])
        self.report.update(frames=frames, phases=phases, payload=payload, timed_segments=segments)
        self.report.update(assemble_timed_cycle(len(frames), segments))
        self.report['timed_validation'] = validate_timed_cycle(phases, self.report['time_from_start_s'], self.report['velocities'],
                self.report['accelerations'], self.report['jerks'], *self.planner._timed_limits, segments)
        if not self.report['timed_validation']['success']:
            raise RuntimeError('timed trajectory validation failed at assembly')

    def step(self, action):
        if self.done:
            raise RuntimeError('reset required')
        began = time.perf_counter()
        stage, before = self.stage, self.cost_s
        info = dict(stage=stage, seed=self.seed, generation_ms=self.generation_ms, feature_ms=self.feature_ms, decode_ms=0., success=False)
        try:
            if self.continuous:
                params = np.asarray(action, dtype=float)
                decode_began = time.perf_counter()
                try:
                    row = self.decoder.decode(params, self.targets, self.previous)
                finally:
                    info['decode_ms'] = (time.perf_counter() - decode_began) * 1000
                if row is None:
                    raise EpisodeFailure('analytic_no_solution')
            else:
                index = int(action)
                if index != action or index < 0 or index >= len(self.observation.candidates):
                    raise ValueError('candidate index outside observation')
                row = self.observation.candidates[index].copy()
                params = self.observation.actions[index].copy()
            info.update(action_q=row.tolist(), action_params=params.tolist())
            self._check_pose(row)
            from v3_search_recorder import SearchRecorder
            stage_start = 0 if stage == 0 else len(self.report['frames'])
            with SearchRecorder(self.seed):
                current, cost = (self._contact, self._carry, self._place)[stage](row)
            self.previous = current
            self.cost_s += cost
            if self.timed:
                self._validate_timed_stage(stage_start)
            if stage == 2:
                self.report['success'] = False
                if self.timed:
                    self._assemble_timing()
                self.report['success'] = True
                self.done = True
                timed_rows = np.asarray(self.report['frames'])[:, self.active_indices]
                timed_phases = self.report['phases']
                executed_bound = sum(self._path_cost(timed_rows[i - 1:i + 1]) for i in range(1, len(timed_rows))
                                     if timed_phases[i] in ('approach', 'contact_approach', 'extract', 'transport', 'conveyor_place', 'return_home'))
                info.update(success=True, class_='success', stage_cost_s=cost, cumulative_cost_s=self.cost_s,
                            reward_cost_kind='raw_planned_path_t_lb', executed_t_path_s=executed_bound,
                            motion_duration_s=self.report.get('motion_duration_s'))
                result = StepResult(None, reward(before, self.cost_s, True), True, False, info)
            else:
                self.stage += 1
                observation = self.observe()
                if not self.continuous and not len(observation.candidates):
                    self.done = True
                    info.update(class_='empty_next_candidates', cumulative_cost_s=self.cost_s)
                    result = StepResult(None, reward(before, self.cost_s, False), True, False, info)
                else:
                    info.update(class_='stage_success', stage_cost_s=cost, cumulative_cost_s=self.cost_s)
                    result = StepResult(observation, reward(before, self.cost_s), False, False, info)
        except EpisodeFailure as error:
            self.done = True
            info.update(class_='task_failure', reason=error.reason, cumulative_cost_s=self.cost_s)
            result = StepResult(None, reward(before, self.cost_s, False), True, False, info)
        except torch.OutOfMemoryError as error:
            self.done = True
            gc.collect(); torch.cuda.empty_cache()
            info.update(class_='infrastructure', reason='cuda_oom', error=str(error)[:200])
            result = StepResult(None, 0., False, True, info)
        except RuntimeError as error:
            if 'timed samples failed' in str(error) or 'timed trajectory validation failed' in str(error):
                self.done = True
                info.update(class_='task_failure', reason='timing_validation', error=str(error)[:200], cumulative_cost_s=self.cost_s)
                result = StepResult(None, reward(before, self.cost_s, False), True, False, info)
            else:
                raise
        info['wall_ms'] = (time.perf_counter() - began) * 1000
        return result

    def checkpoint(self):
        return copy.deepcopy(dict(item=self.item, boxes=self.boxes, seed=self.seed, continuous=self.continuous, previous=self.previous,
                                  home=self.home, stage=self.stage, done=self.done, cost_s=self.cost_s, report=self.report, timed_blocks=self.timed_blocks,
                                  carry_targets=self.carry_targets, targets=self.targets, box_centers=self.box_centers))

    def restore(self, state):
        validate_checkpoint(state, self.names, self.timed)
        for name, value in copy.deepcopy(state).items():
            setattr(self, name, value)
        configure_planner(self.planner, self.item, self.boxes)
        self.empty = self.planner.checker(self.boxes)
        self.loaded = self.planner.checker(self.boxes, payload=True)
        self.empty_valid = GpuValidity(self.empty, check_ground=True)
        self.loaded_valid = GpuValidity(self.loaded, check_ground=True)
        return self.observe()
