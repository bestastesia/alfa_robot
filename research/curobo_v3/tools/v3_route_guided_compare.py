import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from curobo.types import JointState
from curobo_core.contracts import PlannerAssets
from curobo_core.adapter import to_curobo_scene
from curobo_core.scene import Pose, SceneObject
from v3_adaptive_raised_cycle_planner import AdaptiveRaisedCyclePlanner
from v3_batched_loaded_search import GpuValidity, densify
from v3_multiroot_search import batched_rrt_connect_multi_goal
from v3_tcp_navigation import peak_normal_acceleration
from v3_wall_ik_benchmark import ACTIVE_JOINTS


class RouteGuidance:
    def __init__(self, rows, point_ids, start, goals, weights):
        self.rows = torch.as_tensor(rows, device='cuda', dtype=torch.float32)
        unused_indices, inverse, counts = np.unique(point_ids, return_inverse=True, return_counts=True)
        self.probabilities = torch.as_tensor(1. / counts[inverse], device='cuda', dtype=torch.float32)
        self.start, self.goals, self.weights = start, goals, weights
        self.injected = 0

    def __call__(self, samples, generator, lower, upper, best_cost=None):
        indices = torch.multinomial(self.probabilities, len(samples), replacement=True, generator=generator)
        prior = self.rows[indices].clone()
        selector = torch.rand((len(samples),), device='cuda', generator=generator)
        noise = torch.randn(prior.shape, device='cuda', generator=generator) * .10
        noise[:, 0] *= .02 / .10
        prior = torch.where((selector > .65)[:, None], prior + noise, prior)
        prior = torch.clamp(prior, lower, upper)
        use = selector < .9
        if best_cost is not None:
            remaining = torch.cdist(prior * self.weights, self.goals * self.weights).min(dim=1).values
            use &= torch.linalg.vector_norm((prior - self.start) * self.weights, dim=1) + remaining <= best_cost
        self.injected += int(use.sum().item())
        return torch.where(use[:, None], prior, samples)


class TcpRouteCost:
    def __init__(self, checker, targets, weights):
        self.checker, self.weights = checker, weights
        self.positions = torch.as_tensor(np.stack([targets[side][:, :3, 3] for side in ['left', 'right']], axis=1), device='cuda', dtype=torch.float32)
        quaternions = np.stack([np.roll(Rotation.from_matrix(targets[side][:, :3, :3]).as_quat(), 1, axis=1) for side in ['left', 'right']], axis=1)
        self.quaternions = torch.as_tensor(quaternions, device='cuda', dtype=torch.float32)

    def describe(self, rows):
        state = self.checker.kinematics.compute_kinematics(JointState.from_position(rows[None], joint_names=ACTIVE_JOINTS))
        poses = state.tool_poses.to_dict()
        positions = torch.stack([poses[side+'_tool0'].position.reshape(-1, 3) for side in ['left', 'right']], dim=1)
        quaternions = torch.stack([poses[side+'_tool0'].quaternion.reshape(-1, 4) for side in ['left', 'right']], dim=1)
        position_error = torch.linalg.vector_norm(positions[:, None] - self.positions[None], dim=-1).mean(dim=-1)
        cosine = (quaternions[:, None] * self.quaternions[None]).sum(-1).abs().clamp(0, 1)
        angle = 2 * torch.acos(cosine)
        score = position_error + .10 * angle.mean(-1)
        indices = score.argmin(dim=1)
        selected = torch.arange(len(rows), device='cuda')
        return position_error[selected, indices], angle.mean(-1)[selected, indices], indices.float() / (len(self.positions)-1), positions, quaternions

    def edge_cost(self, starts, ends):
        rows = torch.stack([starts, (starts+ends)*.5, ends], dim=1).reshape(-1, starts.shape[-1])
        position, angle, progress, unused_positions, unused_quaternions = self.describe(rows)
        progress = progress.reshape(-1, 3)
        length = torch.linalg.vector_norm((ends-starts)*self.weights, dim=-1)
        return length * (3 * position.reshape(-1, 3).mean(-1) + .5 * angle.reshape(-1, 3).mean(-1)) + .8 * torch.relu(progress[:, :-1] - progress[:, 1:]).sum(-1)


def retime_tracks(tracks, limits, base_dt=1/60):
    scale = 1.
    raw = {}
    for side, track in tracks.items():
        positions = np.asarray(track['positions'])
        times = np.arange(len(positions))*base_dt
        velocities = np.diff(positions,axis=0)/base_dt
        accelerations = np.diff(velocities,axis=0)/base_dt
        orientations = Rotation.from_quat(np.roll(track['quaternions_wxyz'],-1,axis=1))
        angular_velocity = orientations[:-1].apply((orientations[:-1].inv()*orientations[1:]).as_rotvec())/base_dt
        angular_acceleration = np.diff(angular_velocity,axis=0)/base_dt
        metrics = dict(peak_speed_m_s=float(np.linalg.norm(velocities,axis=1).max()),peak_acceleration_m_s2=float(np.linalg.norm(accelerations,axis=1).max()) if len(accelerations) else 0.,peak_normal_acceleration_m_s2=peak_normal_acceleration(positions,times),peak_angular_speed_deg_s=float(np.rad2deg(np.linalg.norm(angular_velocity,axis=1).max())),peak_angular_acceleration_deg_s2=float(np.rad2deg(np.linalg.norm(angular_acceleration,axis=1).max())) if len(angular_acceleration) else 0.)
        raw[side] = metrics
        scale = max(scale,metrics['peak_speed_m_s']/limits['speed_m_s'],math.sqrt(metrics['peak_acceleration_m_s2']/limits['acceleration_m_s2']),math.sqrt(metrics['peak_normal_acceleration_m_s2']/limits['normal_acceleration_m_s2']),metrics['peak_angular_speed_deg_s']/45.,math.sqrt(metrics['peak_angular_acceleration_deg_s2']/90.))
    if scale > 1:
        scale *= 1.01
    actual = {side:{name:value/(scale**2 if 'acceleration' in name else scale) for name,value in metrics.items()} for side,metrics in raw.items()}
    for side,track in tracks.items():
        measured = peak_normal_acceleration(track['positions'],np.arange(len(track['positions']))*base_dt*scale)
        if measured > limits['normal_acceleration_m_s2']*1.0001:
            raise RuntimeError('Final TCP normal acceleration verification failed')
        actual[side]['peak_normal_acceleration_m_s2'] = measured
    return dict(time_scale=scale,frame_dt_s=base_dt*scale,raw_metrics=raw,actual_metrics=actual,limits=limits,timing_contract='Uniform time stretching of unchanged collision-validated joint frames; sampled TCP v/a/angular/normal limits, no torque or full dynamics verification')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--route', type=Path, required=True)
    parser.add_argument('--maps', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--budget', type=float, default=2.)
    parser.add_argument('--seed', type=int, default=20261021)
    parser.add_argument('--stop-at-first-valid', action='store_true')
    parser.add_argument('--methods', nargs='+', choices=['analytic','full','lm'], default=['analytic','full','lm'])
    parser.add_argument('--only-selected-maps', action='store_true')
    parser.add_argument('--goal-bank', type=Path)
    parser.add_argument('--fixed-goal-result', type=Path)
    parser.add_argument('--fixed-goal-method', choices=['analytic','full','lm'], default='full')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    document = json.loads(args.route.read_text())
    planner = AdaptiveRaisedCyclePlanner(PlannerAssets())
    planner.configure_task(document['snapshot']['task'])
    planner.snapshot = replace(planner.snapshot, objects=tuple(SceneObject(item['name'], tuple(item['dimensions']), Pose(tuple(item['position']), tuple(item['quaternion_wxyz']))) for item in document['snapshot']['objects'] if item['name'] != 'navigation_ground'), meshes=())
    checker = planner.checker(planner.selected_task)
    full_scene = to_curobo_scene(planner.snapshot,())
    checker.update_world(full_scene)
    scene_names = [item.name for item in full_scene.cuboid]
    for box in planner.selected_task.values():
        if f'wall_box_{box:02d}' not in scene_names:
            raise ValueError('Guided approach experiment requires every target box in the collision scene')
    maps = {}
    map_recheck_ms = {}
    for method in (args.methods if args.only_selected_maps else ['analytic', 'full', 'lm']):
        with np.load(args.maps / method / 'trial_0.npz') as archive:
            maps[method] = {name: archive[name] for name in archive.files}
        began = time.perf_counter()
        values = maps[method]
        collision = GpuValidity(checker,check_ground=True).mask(torch.as_tensor(values['joints'],device='cuda',dtype=torch.float32)).cpu().numpy()
        values['legal'] &= collision
        map_recheck_ms[method] = (time.perf_counter()-began)*1000
    targets = {side: next(iter(maps.values()))[side+'_targets'] for side in ['left', 'right']}
    for values in maps.values():
        for side, matrices in targets.items():
            if not np.array_equal(values[side+'_targets'], matrices):
                raise ValueError('All three methods must use identical target Poses')
    goal_rows = [values['joints'][values['legal'] & (values['point_indices'] == 127)] for values in maps.values()]
    goals_np = np.unique(np.round(np.concatenate(goal_rows), 5), axis=0)
    if args.goal_bank is not None:
        goals_np = np.load(args.goal_bank).astype(np.float32).reshape(-1, len(ACTIVE_JOINTS))
    if args.fixed_goal_result is not None:
        selected = json.loads(args.fixed_goal_result.read_text())['methods'][args.fixed_goal_method]
        if not selected['success']:
            raise ValueError('Fixed-goal source must contain a validated successful trajectory')
        goals_np = np.asarray(selected['trajectory'][-1], dtype=np.float32)[None]
    if len(goals_np):
        terminal_mask = GpuValidity(checker,check_ground=True).mask(torch.as_tensor(goals_np,device='cuda',dtype=torch.float32)).cpu().numpy()
        goals_np = goals_np[terminal_mask]
    results = dict(route=document, seed=args.seed, search_budget_s=args.budget, joint_names=ACTIVE_JOINTS, methods={}, guidance_contract='All retained legal IK nodes; 65% exact prior + 25% local noise + 10% original sampling; common endpoint union; soft TCP route position/orientation/backtracking cost; no mandatory waypoint chain; all edges collision checked')
    if args.fixed_goal_result is not None:
        results['fixed_goal_source'] = dict(result=str(args.fixed_goal_result),method=args.fixed_goal_method)
    results['scene_object_names'] = scene_names
    results['collision_scope'] = 'complete exported scene including target boxes; ground separately checked with base support exemption'
    for method in args.methods:
        values = maps[method]
        began = time.perf_counter()
        validity = GpuValidity(checker, check_ground=True)
        validity.weights[0] = math.radians(15) / .1
        start = torch.as_tensor(planner.home_values, device='cuda', dtype=torch.float32)
        metadata = json.loads((args.maps / method / 'report.json').read_text())
        entry = dict(method=method, map_ms=metadata['trials'][0]['total_online_ms'], legal_nodes=int(values['legal'].sum()), covered_points=int(np.unique(values['point_indices'][values['legal']]).size), common_goal_count=len(goals_np), success=False, trajectory=[], tracks={})
        entry['prior_complete_scene_recheck_ms'] = map_recheck_ms[method]
        if not len(goals_np):
            entry['failure'] = 'No legal IK at the final reference Pose; cannot search an executable route to that target'
        elif not entry['legal_nodes']:
            entry['failure'] = 'No legal guidance configurations'
        elif not bool(validity.mask(start[None]).item()):
            entry['failure'] = 'Home is in collision in the frozen scene'
        else:
            goals = torch.as_tensor(goals_np, device='cuda', dtype=torch.float32)
            goal_mask = validity.mask(goals)
            goals = goals[goal_mask]
            guidance = RouteGuidance(values['joints'][values['legal']], values['point_indices'][values['legal']], start, goals, validity.weights)
            cost = TcpRouteCost(checker, targets, validity.weights)
            validity.post_sampling_transform = guidance
            validity.box_rotation_cost = cost
            lower, upper = checker.kinematics.get_joint_limits().position
            path, search = batched_rrt_connect_multi_goal(start, goals, lower, upper, validity, args.budget, args.seed, informed_sampling=True, stop_at_first_valid=args.stop_at_first_valid)
            entry['search'] = search
            entry['guided_samples'] = guidance.injected
            if path is None:
                entry['failure'] = 'No finely validated joint path within the search budget'
            else:
                dense = np.asarray(densify(path, validity.weights.cpu().tolist()))
                dense_tensor = torch.as_tensor(dense, device='cuda', dtype=torch.float32)
                if not bool(validity.mask(dense_tensor).all().item()):
                    raise RuntimeError('Final collision verification failed')
                metrics = [cost.describe(chunk) for chunk in dense_tensor.split(512)]
                positions = torch.cat([metric[3] for metric in metrics]).cpu().numpy()
                quaternions = torch.cat([metric[4] for metric in metrics]).cpu().numpy()
                for arm, side in enumerate(['left', 'right']):
                    position_error = np.linalg.norm(positions[-1, arm] - targets[side][-1, :3, 3])
                    orientation_error = (Rotation.from_matrix(targets[side][-1, :3, :3]).inv()*Rotation.from_quat(np.roll(quaternions[-1, arm], -1))).magnitude()
                    if position_error > .002 or orientation_error > math.radians(1):
                        raise RuntimeError('Final TCP Pose verification failed')
                    orientation = Rotation.from_quat(np.roll(quaternions[:, arm], -1, axis=1))
                    rotation_deg = np.rad2deg((orientation[:-1].inv()*orientation[1:]).magnitude()).sum()
                    entry['tracks'][side] = dict(positions=positions[:, arm].tolist(), quaternions_wxyz=quaternions[:, arm].tolist(), length_m=float(np.linalg.norm(np.diff(positions[:, arm], axis=0), axis=1).sum()), total_rotation_deg=float(rotation_deg))
                entry.update(success=True, trajectory=dense.tolist(), frame_dt_s=1/60, frames=len(dense), weighted_joint_length=float(np.linalg.norm(np.diff(dense, axis=0)*validity.weights.cpu().numpy(), axis=1).sum()), mean_route_position_error_m=float(torch.cat([metric[0] for metric in metrics]).mean().item()), mean_route_angle_error_deg=float(torch.cat([metric[1] for metric in metrics]).mean().item()*180/math.pi))
                limits = document.get('limits',{})
                if limits.get('normal_acceleration_m_s2') is not None:
                    entry['retiming'] = retime_tracks(entry['tracks'],limits)
                    entry['frame_dt_s'] = entry['retiming']['frame_dt_s']
                entry['execution_duration_s'] = (len(dense)-1)*entry['frame_dt_s']
        entry['search_and_validation_ms'] = (time.perf_counter()-began)*1000
        results['methods'][method] = entry
        (args.output / 'comparison.json').write_text(json.dumps(results, ensure_ascii=False))
        print(method, entry.get('failure', 'success'), entry['search_and_validation_ms'], flush=True)
    print(args.output / 'comparison.json', flush=True)


if __name__ == '__main__':
    main()
