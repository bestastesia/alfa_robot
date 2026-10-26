import argparse
import copy
import ctypes
import heapq
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.types import GoalToolPose, JointState, Pose
from curobo_core.contracts import PlannerAssets
from v3_batched_loaded_search import GpuValidity, densify
from v3_cartesian_cases import OUTPUT, WEIGHTS, line_samples
from v3_endpoint_guided_ik import biased_seeds, polish
from v3_multiroot_search import batched_rrt_connect_multi_goal
from v3_route_guided_compare import RouteGuidance, TcpRouteCost
from v3_serial_endpoint_demo import TeleportPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS


METHODS = ('fixed_analytic', 'grid_analytic', 'parameter_rrt', 'lm_informed_connect')


def path_accepted(metrics):
    return all(metrics[key] for key in ('collision_passed', 'endpoint_payload_collision_passed', 'anchor_joint_passed')) and metrics['endpoint_error_mm'] <= 2.


def synchronized_time():
    torch.cuda.synchronize()
    return time.perf_counter()


class CartesianComparison:
    def __init__(self, payload_policy='legacy', planner=None):
        self.planner = planner if planner is not None else TeleportPlanner(PlannerAssets())
        self.payload_policy = payload_policy
        self.solvers = {}
        if hasattr(self.planner.analytic, 'solve_grid'):
            self.library = None
            return
        self.library = ctypes.CDLL(str(OUTPUT / 'libcartesian_grid.so'))
        array = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
        integers = np.ctypeslib.ndpointer(dtype=np.int32, flags='C_CONTIGUOUS')
        self.library.cartesian_grid.argtypes = [ctypes.c_int, array, ctypes.c_int, array, ctypes.c_int, array, array, integers]
        self.library.cartesian_grid.restype = None

    def prepare(self, case):
        self.case = case
        self.planner.configure_task(case['task'])
        self.start = np.asarray(case['contact_joints'])
        self.empty = self.planner.checker(case['task'])
        self.loaded = self.planner.checker(case['task'], payload=True) if case['payload'] else self.empty
        self.checker = self.loaded if case['payload'] and self.payload_policy == 'loaded' else self.empty
        self.validity = GpuValidity(self.checker, check_ground=True)
        self.validity.weights[0] = WEIGHTS[0]
        self.validity.search_base_margin_m = .002
        self.active = tuple(case['task'])
        self.initial_poses = self.planner._tool_poses(self.checker, torch.tensor(self.start, device='cuda', dtype=torch.float32))
        self.lower, self.upper = [item.cpu().numpy() for item in self.checker.kinematics.get_joint_limits().position]

    def targets(self, fraction):
        output = {}
        for side in self.active:
            matrix = np.asarray(self.case['tcp_start_local'][side]).copy()
            matrix[:3, 3] += np.asarray(self.case['translation_local'][side]) * fraction
            output[side] = matrix
        return output

    def poses(self, rows):
        output = {side: [] for side in ('left', 'right')}
        for offset in range(0, len(rows), 512):
            values = torch.as_tensor(np.asarray(rows[offset:offset + 512]), device='cuda', dtype=torch.float32)
            state = self.checker.kinematics.compute_kinematics(JointState.from_position(values[None], joint_names=ACTIVE_JOINTS))
            for side in output:
                pose = state.tool_poses.to_dict()[side + '_tool0']
                output[side].append((pose.position.reshape(-1, 3).detach().cpu().numpy(), pose.quaternion.reshape(-1, 4).detach().cpu().numpy()))
        return {side: (np.concatenate([item[0] for item in batches]), np.concatenate([item[1] for item in batches])) for side, batches in output.items()}

    def line_metrics(self, rows):
        tracks = self.poses(rows)
        lateral, angular, outside, backwards = 0., 0., 0., 0.
        fractions = []
        serialized = {}
        for side, (positions, quaternions) in tracks.items():
            rotation = Rotation.from_quat(np.roll(quaternions, -1, axis=1))
            serialized[side] = dict(positions=positions.tolist(), quaternions_wxyz=quaternions.tolist(),
                cumulative_rotation_deg=float(np.rad2deg((rotation[:-1].inv() * rotation[1:]).magnitude()).sum()))
            if side not in self.active:
                continue
            start = np.asarray(self.case['tcp_start_local'][side])
            delta = np.asarray(self.case['translation_local'][side])
            length = np.linalg.norm(delta)
            progress = (positions - start[:3, 3]) @ delta / length ** 2
            fractions.append(progress)
            lateral = max(lateral, float(np.linalg.norm(positions - start[:3, 3] - progress[:, None] * delta, axis=1).max()))
            angular = max(angular, float((Rotation.from_matrix(start[:3, :3]).inv() * rotation).magnitude().max()))
            outside = max(outside, float(np.maximum(-progress, progress - 1).max() * length))
            backwards = max(backwards, float(np.maximum(-np.diff(progress), 0).max(initial=0) * length))
        synchronization = max((float(np.ptp(np.stack(fractions), axis=0).max()) * self.case['stage_distance_m']), 0.)
        ends = self.targets(1.)
        endpoint = max(float(np.linalg.norm(tracks[side][0][-1] - ends[side][:3, 3])) for side in self.active)
        return dict(max_line_error_mm=lateral * 1000, max_orientation_error_deg=float(np.rad2deg(angular)),
            max_outside_mm=max(0., outside * 1000), max_backward_mm=backwards * 1000,
            max_dual_progress_difference_mm=synchronization * 1000, endpoint_error_mm=endpoint * 1000,
            straight_passed=lateral <= .002 and angular <= np.deg2rad(1) and outside <= .002 and backwards <= .002 and synchronization <= .002 and endpoint <= .002,
            tcp_tracks=serialized)

    def collision_mask(self, rows, margin=0.):
        values = torch.as_tensor(np.asarray(rows), device='cuda', dtype=torch.float32)
        return self.validity.evaluate(values, base_margin_m=margin)[0].cpu().numpy()

    def audit(self, path):
        dense = np.asarray(densify(np.asarray(path)))
        mask = self.collision_mask(dense)
        metrics = self.line_metrics(dense)
        endpoint_valid = bool(mask[-1])
        if self.case['payload']:
            endpoint_valid = endpoint_valid and bool(GpuValidity(self.loaded, check_ground=True).mask(torch.tensor(dense[-1:], device='cuda', dtype=torch.float32))[0])
        anchor_valid = bool(np.max(np.abs(dense[0] - self.start)) <= 1e-5)
        metrics.update(collision_passed=bool(mask.all()), endpoint_payload_collision_passed=endpoint_valid,
            anchor_joint_passed=anchor_valid, invalid_frames=np.flatnonzero(~mask).tolist(), frame_count=len(dense),
            max_keyframe_joint_step_deg=float(np.rad2deg(np.abs(np.diff(path, axis=0)[:, 1:])).max(initial=0)),
            weighted_joint_length=float(np.linalg.norm(np.diff(dense, axis=0) * WEIGHTS, axis=1).sum()),
            updown_travel_m=float(np.abs(np.diff(dense[:, 0])).sum()),
            joint_travel_deg=np.rad2deg(np.abs(np.diff(dense[:, 1:], axis=0)).sum(axis=0)).tolist())
        passed = path_accepted(metrics)
        return passed, dense, metrics

    def lift(self, parameter, previous):
        fraction, height = parameter[:2]
        row = np.asarray(previous).copy()
        row[0] = height
        self.planner.fk(row)
        inverse = np.linalg.inv(self.planner.fk_robot.get_transform('arm_carriage', self.planner.fk_robot.base_link))
        for side in self.active:
            arm = 0 if side == 'left' else 1
            offset = 1 + arm * 7
            solutions = self.planner.analytic.solve(arm, inverse @ self.targets(fraction)[side], parameter[2 + arm], row[offset:offset + 7])
            if not len(solutions):
                return None
            row[offset:offset + 7] = solutions[np.argmin(np.linalg.norm(solutions - row[offset:offset + 7], axis=1))]
        return row

    def fixed(self):
        rows = [self.start.copy()]
        for index, fraction in enumerate(np.linspace(0, 1, self.case['sample_count'])[1:], 1):
            parameters = [fraction, self.start[0], self.case['fixed_swivels_rad']['left'], self.case['fixed_swivels_rad']['right']]
            row = self.lift(parameters, rows[-1])
            if row is None:
                return None, dict(failure='固定Updown/冗余角解析无解', failed_sample=index, fraction=float(fraction), partial_frames=np.asarray(rows).tolist())
            rows.append(row)
        return np.asarray(rows), {}

    def grids(self, fractions):
        heights = np.unique(np.r_[np.arange(-1., .0001, .05), self.start[0]])
        inverses = []
        for height in heights:
            row = self.start.copy()
            row[0] = height
            self.planner.fk(row)
            inverses.append(np.linalg.inv(self.planner.fk_robot.get_transform('arm_carriage', self.planner.fk_robot.base_link)))
        output = [[{} for height in heights] for fraction in fractions]
        for side in self.active:
            arm = 0 if side == 'left' else 1
            offset = 1 + arm * 7
            angles = np.unique(np.r_[np.deg2rad(np.arange(-180, 180, 10)), self.case['fixed_swivels_rad'][side]])
            targets = np.array([self.targets(fraction)[side] for fraction in fractions])
            matrices = np.ascontiguousarray(np.einsum('hij,pjk->phik', np.asarray(inverses), targets).reshape(-1, 4, 4))
            values = np.zeros((len(matrices), len(angles), 16, 7), dtype=np.float64)
            counts = np.zeros((len(matrices), len(angles)), dtype=np.int32)
            if hasattr(self.planner.analytic, 'solve_grid'):
                values, counts = self.planner.analytic.solve_grid(arm, matrices, angles, self.start[offset:offset + 7])
            else:
                self.library.cartesian_grid(arm, matrices, len(matrices), np.ascontiguousarray(angles), len(angles),
                    np.ascontiguousarray(self.start[offset:offset + 7]), values, counts)
            for point in range(len(fractions)):
                for height_index in range(len(heights)):
                    flat = point * len(heights) + height_index
                    valid = np.arange(16)[None] < counts[flat, :, None]
                    candidates = np.unique(np.round(values[flat][valid], 8), axis=0)
                    output[point][height_index][side] = candidates
        return heights, output

    def ordered_candidates(self, heights, grids, reference, cap=128):
        sorted_grids = []
        queue = []
        for height, grid in zip(heights, grids):
            options = []
            costs = []
            for arm, side in enumerate(('left', 'right')):
                offset = 1 + arm * 7
                values = grid.get(side, reference[None, offset:offset + 7])
                cost = np.sum((values - reference[offset:offset + 7]) ** 2, axis=1)
                order = np.argsort(cost, kind='stable')
                options.append(values[order])
                costs.append(cost[order])
            if not len(options[0]) or not len(options[1]):
                continue
            lift_cost = float(((height - reference[0]) * WEIGHTS[0]) ** 2)
            grid_index = len(sorted_grids)
            sorted_grids.append((height, options, costs, lift_cost))
            heapq.heappush(queue, (lift_cost + costs[0][0] + costs[1][0], grid_index, 0, 0))
        visited = set()
        rows = []
        while queue and len(rows) < cap:
            cost, grid_index, left, right = heapq.heappop(queue)
            if (grid_index, left, right) in visited:
                continue
            visited.add((grid_index, left, right))
            height, options, costs, lift_cost = sorted_grids[grid_index]
            rows.append(np.r_[height, options[0][left], options[1][right]])
            for next_left, next_right in ((left + 1, right), (left, right + 1)):
                if next_left < len(options[0]) and next_right < len(options[1]):
                    heapq.heappush(queue, (lift_cost + costs[0][next_left] + costs[1][next_right], grid_index, next_left, next_right))
        return np.asarray(rows).reshape(-1, 15)

    def grid(self):
        began = synchronized_time()
        fractions = np.linspace(0, 1, self.case['sample_count'])
        heights, grids = self.grids(fractions[1:])
        enumeration_ms = (synchronized_time() - began) * 1000
        beam = [(0., [self.start.copy()])]
        checked = 0
        for point, grid in enumerate(grids, 1):
            next_beam = []
            for cost, path in beam:
                candidates = self.ordered_candidates(heights, grid, path[-1], cap=128)
                if not len(candidates):
                    continue
                mask = self.collision_mask(candidates)
                for row in candidates[mask][:16]:
                    edge = np.asarray(densify(np.array([path[-1], row])))
                    checked += 1
                    if not self.collision_mask(edge).all():
                        continue
                    next_beam.append((cost + float(np.linalg.norm((row - path[-1]) * WEIGHTS)), path + [row]))
                    if len(next_beam) >= 8:
                        break
            if not next_beam:
                return None, dict(failure='当前枚举束预算内没有通过碰撞检查的连续连接', failed_sample=point,
                    fraction=float(fractions[point]), partial_frames=np.asarray(beam[0][1]).tolist(), enumeration_ms=enumeration_ms,
                    checked_edges=checked, beam_width=4, candidates_per_parent=128, edge_candidates_per_parent=16)
            beam = sorted(next_beam, key=lambda item: item[0])[:4]
        return np.asarray(beam[0][1]), dict(enumeration_ms=enumeration_ms, checked_edges=checked, beam_width=4,
            candidates_per_parent=128, edge_candidates_per_parent=16, updown_step_m=.05, swivel_step_deg=10,
            fixed_anchor_height_and_swivels_included=True)

    def endpoint_bank(self):
        heights, grids = self.grids([1.])
        rows = self.ordered_candidates(heights, grids[0], self.start, cap=1024)
        if not len(rows):
            return rows
        valid = self.collision_mask(rows, margin=self.case.get('endpoint_base_margin_m', 0.))
        if self.case['payload']:
            valid &= GpuValidity(self.loaded, check_ground=True).mask(torch.tensor(rows, device='cuda', dtype=torch.float32)).cpu().numpy()
        return rows[valid][:self.case.get('endpoint_return_limit', 16)]

    def parameter_rrt(self):
        goals = self.endpoint_bank()
        if not len(goals):
            return None, dict(failure='参数空间搜索没有合法终点种子')
        goal = goals[0]
        target_parameter = np.array([1., goal[0], *[self.planner.analytic.swivel(arm, goal[1 + arm * 7:8 + arm * 7]) for arm in range(2)]])
        root = np.array([0., self.start[0], self.case['fixed_swivels_rad']['left'], self.case['fixed_swivels_rad']['right']])
        parameters = [root]
        joints = [self.start.copy()]
        parents = [-1]
        edges = [self.start[None]]
        generator = np.random.default_rng(20261024)
        began = synchronized_time()
        attempts = 0
        while time.perf_counter() - began < 2.:
            progress = generator.random()
            if generator.random() < .7:
                sample = root * (1 - progress) + target_parameter * progress
                sample += generator.normal(size=4) * [0., .08, .5, .5]
                if generator.random() < .4:
                    sample[0] = 1.
            else:
                sample = np.r_[generator.random(), generator.uniform(-1., 0.), generator.uniform(-np.pi, np.pi, 2)]
            sample = np.clip(sample, [0., -1., -np.pi, -np.pi], [1., 0., np.pi, np.pi])
            for arm, side in enumerate(('left', 'right')):
                if side not in self.active:
                    sample[2 + arm] = root[2 + arm]
            distance = np.sum(((np.asarray(parameters) - sample) * [3., 10., 1., 1.]) ** 2, axis=1)
            possible = np.where(np.asarray(parameters)[:, 0] <= sample[0] + 1e-8)[0]
            if not len(possible):
                continue
            parent = int(possible[np.argmin(distance[possible])])
            start = parameters[parent]
            scale = min(1., 1 / np.max(np.abs(sample - start) / [.12, .04, np.deg2rad(20), np.deg2rad(20)]))
            end = start + (sample - start) * scale
            count = max(1, int(np.ceil(np.max(np.abs(end - start) / [.025, .01, np.deg2rad(5), np.deg2rad(5)]))))
            edge = [joints[parent]]
            for fraction in np.linspace(0, 1, count + 1)[1:]:
                row = self.lift(start + (end - start) * fraction, edge[-1])
                if row is None:
                    edge = None
                    break
                edge.append(row)
            attempts += 1
            if edge is None:
                continue
            dense = np.asarray(densify(np.asarray(edge)))
            if not self.collision_mask(dense, margin=.002).all():
                continue
            parameters.append(end)
            joints.append(edge[-1])
            parents.append(parent)
            edges.append(np.asarray(edge))
            if end[0] >= 1 - 1e-8:
                node = len(parents) - 1
                selected = []
                while node >= 0:
                    selected.append(edges[node])
                    node = parents[node]
                return np.concatenate(selected[::-1]), dict(search_ms=(synchronized_time() - began) * 1000, nodes=len(parameters), attempts=attempts,
                    state_space='progress,updown,left_swivel,right_swivel; active analytical branches lifted continuously', search_base_margin_m=.002)
        best = int(np.argmax(np.asarray(parameters)[:, 0]))
        node = best
        selected = []
        while node >= 0:
            selected.append(edges[node])
            node = parents[node]
        return None, dict(failure='参数空间RRT在2秒搜索预算内未到达终点', search_ms=(synchronized_time() - began) * 1000,
            nodes=len(parameters), attempts=attempts, furthest_progress=float(parameters[best][0]), partial_frames=np.concatenate(selected[::-1]).tolist())

    def lm_prior(self, samples, references):
        count = len(references)
        batch = min(64, count)
        began = synchronized_time()
        setup = 0.
        if batch not in self.solvers:
            config = InverseKinematicsCfg.create(robot=copy.deepcopy(self.planner.robot), scene_model=None,
                self_collision_check=False, num_seeds=16, seed_solver_num_seeds=16, max_batch_size=batch,
                position_tolerance=.002, orientation_tolerance=np.deg2rad(1), use_cuda_graph=True,
                override_iters_for_multi_link_ik=160, random_seed=20261021)
            config.exit_early = True
            config.exit_early_batch_success_threshold = 0.
            self.solvers[batch] = [InverseKinematics(config), None]
            setup = (synchronized_time() - began) * 1000
        solver, buffer = self.solvers[batch]
        seeds = biased_seeds(references, self.lower, self.upper)
        rows = []
        began = synchronized_time()
        for offset in range(0, count, batch):
            goals = GoalToolPose.from_poses({side + '_tool0': Pose(
                position=torch.tensor(matrix[offset:offset + batch, :3, 3], device='cuda', dtype=torch.float32),
                quaternion=torch.tensor(np.roll(Rotation.from_matrix(matrix[offset:offset + batch, :3, :3]).as_quat(), 1, axis=1), device='cuda', dtype=torch.float32))
                for side, matrix in samples.items()}, ordered_tool_frames=['left_tool0', 'right_tool0'])
            if buffer is None:
                buffer = copy.deepcopy(goals)
            else:
                buffer.position.copy_(goals.position)
                buffer.quaternion.copy_(goals.quaternion)
            state = JointState.from_position(torch.tensor(references[offset:offset + batch], device='cuda', dtype=torch.float32), joint_names=ACTIVE_JOINTS)
            with torch.enable_grad():
                result = solver.solve_pose(buffer, current_state=state, seed_config=torch.tensor(seeds[offset:offset + batch], device='cuda'), return_seeds=16)
            indices = [result.js_solution.joint_names.index(name) for name in ACTIVE_JOINTS]
            rows.append(result.js_solution.position.reshape(-1, len(result.js_solution.joint_names))[:, indices].detach().clone())
        self.solvers[batch][1] = buffer
        rows = torch.cat(rows)
        ids = np.repeat(np.arange(count), 16)
        lm_ms = (synchronized_time() - began) * 1000
        began = synchronized_time()
        rows = polish(self.checker, rows, ids, samples, references)
        return rows, ids, dict(lm_ms=lm_ms, soft_refine_ms=(synchronized_time() - began) * 1000, solver_setup_ms=setup,
            point_count=count, batch_points=batch, seeds_per_point=16)

    def lm_search(self):
        if 'fixed_goal_joints' in self.case:
            goals = np.asarray(self.case['fixed_goal_joints'], dtype=np.float32).reshape(1, 15)
            goals = goals[self.collision_mask(goals)]
        else:
            goals = self.endpoint_bank()
        if not len(goals):
            return None, dict(failure='LM引导搜索没有合法终点种子')
        target = goals[0]
        count = self.case['sample_count']
        progress = np.linspace(0., 1., count)
        references = (1 - progress[:, None]) * self.start + progress[:, None] * target
        samples = line_samples(self.case)
        for side in ('left', 'right'):
            if side not in samples:
                samples[side] = np.array([self.planner._tool_poses(self.checker, torch.tensor(row, device='cuda', dtype=torch.float32))[side] for row in references])
        rows, ids, stats = self.lm_prior(samples, references)
        began = synchronized_time()
        legal = self.validity.mask(rows)
        state = self.checker.kinematics.compute_kinematics(JointState.from_position(rows[None], joint_names=ACTIVE_JOINTS))
        for side in self.active:
            pose = state.tool_poses.to_dict()[side + '_tool0']
            position = torch.tensor(samples[side][ids, :3, 3], device='cuda', dtype=torch.float32)
            quaternion = torch.tensor(np.roll(Rotation.from_matrix(samples[side][ids, :3, :3]).as_quat(), 1, axis=1), device='cuda', dtype=torch.float32)
            legal &= torch.linalg.vector_norm(pose.position.reshape(-1, 3) - position, dim=-1) <= .002
            legal &= (pose.quaternion.reshape(-1, 4) * quaternion).sum(-1).abs() >= np.cos(np.deg2rad(1) / 2)
        selected_ids = ids[legal.cpu().numpy()]
        rows = rows[legal]
        stats.update(filter_ms=(synchronized_time() - began) * 1000, legal_nodes=len(rows), covered_points=len(np.unique(selected_ids)))
        if not len(rows):
            return None, dict(failure='直线采样没有合法LM先验', **stats)
        start = torch.tensor(self.start, device='cuda', dtype=torch.float32)
        endpoints = torch.tensor(target[None], device='cuda', dtype=torch.float32)
        self.validity.post_sampling_transform = RouteGuidance(rows.cpu().numpy(), selected_ids, start, endpoints, self.validity.weights)
        self.validity.box_rotation_cost = TcpRouteCost(self.checker, samples, self.validity.weights)
        path, search = batched_rrt_connect_multi_goal(start, endpoints, torch.tensor(self.lower, device='cuda'), torch.tensor(self.upper, device='cuda'),
            self.validity, 2., 20261024, informed_sampling=True, stop_at_first_valid=True)
        self.validity.post_sampling_transform = None
        self.validity.box_rotation_cost = None
        stats.update(search=search, search_ms=search['search_ms'], endpoint_joints=target.tolist(),
            cartesian_contract='Soft Cartesian prior/cost retained; intermediate line/orientation deviations are observations, not rejection conditions.')
        if path is None:
            stats['failure'] = 'LM引导Informed Connect未找到通过碰撞细验的路径'
        return path, stats

    def run(self, case, method):
        setup_began = synchronized_time()
        self.prepare(case)
        setup_ms = (synchronized_time() - setup_began) * 1000
        began = synchronized_time()
        functions = dict(fixed_analytic=self.fixed, grid_analytic=self.grid, parameter_rrt=self.parameter_rrt, lm_informed_connect=self.lm_search)
        path, details = functions[method]()
        planning_ms = (synchronized_time() - began) * 1000
        validation_ms = 0.
        metrics = {}
        dense = []
        success = False
        if path is not None:
            began = synchronized_time()
            success, dense, metrics = self.audit(np.asarray(path))
            validation_ms = (synchronized_time() - began) * 1000
            if not success:
                details['failure'] = '路线已生成，但碰撞或原起终点条件未通过'
        return dict(method=method, case_id=case['case_id'], success=success, connected=path is not None,
            setup_ms=setup_ms, planning_ms=planning_ms, validation_ms=validation_ms,
            online_ms=planning_ms + validation_ms, inclusive_ms=setup_ms + planning_ms + validation_ms,
            metrics=metrics, details=details, frames=np.asarray(dense).tolist(),
            payload_policy=self.payload_policy, fine_collision_margin_m=0., search_base_margin_m=.002,
            acceptance_policy='collision_and_original_endpoints; intermediate line/orientation metrics are observation only',
            measured_scope='One method invocation; reused resources, new trajectory solve. Solver setup inside planning is separately recorded; no full-cycle motion execution.')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--methods', nargs='+', choices=METHODS, default=list(METHODS))
    parser.add_argument('--cases', nargs='*')
    parser.add_argument('--stages', nargs='+', default=['precontact_reverse', 'extraction'])
    parser.add_argument('--payload-policy', choices=['legacy', 'loaded'], default='legacy')
    parser.add_argument('--output', type=Path, default=OUTPUT / 'results')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    document = json.loads((OUTPUT / 'cases.json').read_text())
    cases = [case for case in document['cases'] if case['stage'] in args.stages and (not args.cases or case['case_id'] in args.cases)]
    comparison = CartesianComparison(args.payload_policy)
    for index, case in enumerate(cases):
        case['stage_distance_m'] = .03 if case['stage'] == 'precontact_reverse' else .35
        for method in args.methods:
            path = args.output / (case['case_id'] + '__' + method + '.json')
            if path.exists():
                continue
            print('CASE', index + 1, len(cases), case['case_id'], case['label'], method, flush=True)
            result = comparison.run(case, method)
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps(result, ensure_ascii=False))
            temporary.replace(path)
            print('RESULT', result['success'], result['connected'], round(result['online_ms'], 2), result['details'].get('failure'), flush=True)


if __name__ == '__main__':
    main()
