import copy
import gc
from dataclasses import replace
import math
import time

import numpy as np
import torch
from scipy.spatial.transform import Rotation, Slerp
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.types import JointState

from curobo_core.adapter import matrix_pose
from curobo_core.backend import goal
from curobo_core.planner import CycleBlocked
from curobo_core.resource_stats import bounded_get, record_build
from curobo_core.conveyor import conveyor_pair, DEFAULT_CONVEYOR_MESH
from curobo_core.scene import Pose, digest
from v3_batched_loaded_search import GpuValidity, densify, loaded_robot
from v3_mixed_placement_planner import AnalyticContacts
from v3_task_scene import grasp_transform
from v3_wall_ik_benchmark import ACTIVE_JOINTS


class ConveyorCyclePlanner(AnalyticContacts):
    conveyor_height_m = .732
    placement_clearance_m = .25
    conveyor_push_m = .8
    conveyor_center_x_m = -2.3
    conveyor_head_x_m = 1.20
    conveyor_marker_radius_m = .35
    conveyor_marker_x_m = conveyor_head_x_m - conveyor_marker_radius_m

    def __init__(self, args):
        super().__init__(args)
        pair = conveyor_pair(pose=Pose((self.conveyor_center_x_m, 0., 0.)), gap_m=.05)
        self.original_snapshot = replace(self.original_snapshot, meshes=self.original_snapshot.meshes + pair.world_parts())
        self.snapshot = self.original_snapshot

    def configure_task(self, boxes):
        result = super().configure_task(boxes)
        meshes = tuple(replace(item, pose=Pose((item.pose.position[0] - self.base_advance,
                                                item.pose.position[1], item.pose.position[2]), item.pose.quaternion_wxyz))
                       for item in self.original_snapshot.meshes)
        self.snapshot = replace(self.snapshot, meshes=meshes)
        return result

    def transport_and_turn(self, boxes, transport, stats):
        result = super().transport_and_turn(boxes, transport, stats)
        result['frames'] = result['frames'][:result['transport_frames']]
        return result

    def full_cycle(self, boxes, progress=lambda text: None):
        result = super().full_cycle(boxes, progress)
        for group in ('placed_boxes', 'conveyor_box_targets'):
            for item in result.get(group, {}).values():
                item['position'] = list(item['position'])
                item['position'][0] += self.base_advance
        if result.get('placement_base_pose'):
            result['placement_base_pose']['position'][0] += self.base_advance
        if 'base_advance' in result.get('phases', []):
            indices = [index for index, phase in enumerate(result['phases']) if phase == 'base_advance']
            keep = [indices[0], indices[-1]] + list(range(indices[-1] + 1, len(result['frames'])))
            for key in ('frames', 'phases', 'payload'):
                result[key] = [result[key][index] for index in keep]
            result['phases'][0] = 'home'
            result['phases'][1] = 'base_pick_reposition'
            for event in result.get('predicted_scene_events', []):
                event['frame_index'] = result['phases'].index(event['phase'])
        result['base_transition_policy'] = 'teleport at navigation arrival; no base motion trajectory'
        result['turn_collision_validation'] = 'no turn trajectories; only navigation arrival states are planned'
        return result

    def payload_transform(self, side):
        return grasp_transform(side, self.top_grasp)

    def placement_box_seed(self, side, tool):
        return tool @ self.payload_transform(side)

    def placement_validity(self, checker, payload):
        return GpuValidity(checker, check_ground=True)

    def shared_parking_pose(self, required_bases):
        parking = np.mean(required_bases, axis=0)
        residual = float(np.max(np.linalg.norm(np.asarray(required_bases)-parking, axis=1)))
        if residual > .002:
            raise CycleBlocked('反算放置底盘位', '左右手不能共享同一停车位')
        return parking, residual

    def conveyor_reference_targets(self):
        targets = {}
        for side in ('left', 'right'):
            lateral = -.425 if side == 'left' else .425
            targets[side] = {'position': [self.conveyor_center_x_m + self.conveyor_marker_x_m, lateral,
                                         self.conveyor_height_m + self.box_fit['dimensions_m'][2] * .5 + self.placement_clearance_m],
                             'table_height_m': self.conveyor_height_m,
                             'conveyor_position': [self.conveyor_center_x_m, -.495 if side == 'left' else .495, 0.]}
        return targets

    def tool_matrices(self, values):
        self.fk(values)
        return {side: self.fk_robot.get_transform(side + '_tool0', self.fk_robot.base_link).copy()
                for side in ('left', 'right')}

    def conveyor_ik_solver(self,boxes):
        robot=loaded_robot(self.robot,self.box_fit,attachments=self.task_attachments(boxes))
        key=(self.snapshot.geometry_key,tuple(sorted(boxes.items())),digest(robot))
        def build():
            build_started=time.perf_counter()
            config=InverseKinematicsCfg.create(robot=copy.deepcopy(robot),scene_model=self.scene(boxes),
                collision_cache={'cuboid':max(40,len(self.snapshot.objects)),'mesh':len(self.snapshot.meshes)},
                num_seeds=16,self_collision_check=True,use_cuda_graph=False,
                position_tolerance=.002,orientation_tolerance=math.radians(1),
                override_iters_for_multi_link_ik=160,optimizer_collision_activation_distance=.005)
            solver=InverseKinematics(config);record_build(self,'conveyor_ik',build_started)
            return solver
        cache=getattr(self,'_conveyor_solver_cache',{})
        solver=bounded_get(cache,key,build,getattr(self,'conveyor_solver_cache_limit',8))
        self._conveyor_solver_cache=cache
        return solver

    def plan_conveyor_target(self, start, targets, checker, validity, solver, report, append):
        timing=report.setdefault('conveyor_pipeline_timing_ms',{})
        began=time.perf_counter()
        poses = {side + '_tool0': {'position': matrix[:3, 3].tolist(),
                                  'quaternion': np.roll(Rotation.from_matrix(matrix[:3, :3]).as_quat(), 1).tolist()}
                 for side, matrix in targets.items()}
        current = JointState.from_position(torch.tensor(np.asarray(start)[None], device='cuda', dtype=torch.float32), joint_names=ACTIVE_JOINTS)
        with torch.enable_grad():
            solved = solver.solve_pose(goal(poses), current_state=current, return_seeds=16)
        torch.cuda.synchronize();timing['placement_goal_ik']=(time.perf_counter()-began)*1000
        filter_started=time.perf_counter()
        rows = solved.js_solution.position.reshape(-1, len(solved.js_solution.joint_names))
        rows = rows[:, [solved.js_solution.joint_names.index(name) for name in ACTIVE_JOINTS]].contiguous()
        numerical_success = solved.success.reshape(-1).bool()
        candidates = rows[numerical_success].contiguous()
        valid = validity.mask(candidates) if len(candidates) else torch.zeros(0, device='cuda', dtype=torch.bool)
        candidates = candidates[valid].contiguous()
        report['conveyor_goal_ik'] = {'returned': len(rows), 'solver_success': int(numerical_success.sum()),
                                     'collision_valid': len(candidates), 'mode': 'cuRobo numerical IK, updown plus 14 arm joints'}
        report['conveyor_place_mode'] = 'fixed final box pose; multi-goal RRT; no Cartesian straight-line requirement'
        if not len(candidates):
            raise CycleBlocked('龙头车放置目标IK', f"目标IK没有合法终点：{report['conveyor_goal_ik']}", report)
        start_tensor = torch.tensor(start, device='cuda', dtype=torch.float32)
        if not bool(validity.mask(start_tensor[None]).all()):
            raise CycleBlocked('龙头车放置起点', '导航到位后的携箱起点碰撞', report)
        distances = torch.linalg.vector_norm((candidates - start_tensor) * validity.weights, dim=-1)
        candidates = candidates[torch.argsort(distances)].contiguous()
        torch.cuda.synchronize();timing['placement_candidate_filter']=(time.perf_counter()-filter_started)*1000
        lower, upper = checker.kinematics.get_joint_limits().position
        search_started=time.perf_counter()
        path, stats = self.search_path(start_tensor, candidates, lower + 1e-5, upper - 1e-5,
                                       validity, 2.0, 20261013, stop_at_first_valid=True)
        torch.cuda.synchronize();timing['placement_rrt']=(time.perf_counter()-search_started)*1000
        report['conveyor_search_stats'] = stats
        if path is None:
            raise CycleBlocked('龙头车放置RRT', '2秒预算内未找到无碰撞路径', report)
        validation_started=time.perf_counter()
        dense = np.asarray(densify(path))
        if not bool(validity.mask(torch.tensor(dense, device='cuda', dtype=torch.float32)).all()):
            raise CycleBlocked('龙头车放置RRT细验', '密化后的路径未通过碰撞检查', report)
        torch.cuda.synchronize();timing['placement_dense_validation']=(time.perf_counter()-validation_started)*1000
        names = report['joint_names']
        mobile = np.repeat(np.asarray(report['frames'][-1])[None], len(dense), axis=0)
        mobile[:, [names.index(name) for name in ACTIVE_JOINTS]] = dense
        append(mobile, 'conveyor_place', True)
        return dense[-1]

    def cartesian_segment(self, start, targets, checker, validity, solver, report, append, phase, payload):
        initial = self.tool_matrices(start)
        rotations = {side: Slerp([0, 1], Rotation.from_matrix(np.stack([initial[side][:3, :3], targets[side][:3, :3]])))
                     for side in initial}
        count = max(2, max(math.ceil(np.linalg.norm(targets[side][:3, 3] - initial[side][:3, 3]) / .01)
                           for side in initial), max(math.ceil((Rotation.from_matrix(initial[side][:3, :3]).inv() *
                           Rotation.from_matrix(targets[side][:3, :3])).magnitude() / math.radians(2)) for side in initial))
        previous = np.asarray(start, dtype=np.float32)
        names = report['joint_names']
        endpoint = np.asarray(report['frames'][-1])
        for fraction in np.linspace(0, 1, count + 1)[1:]:
            poses = {side + '_tool0': {'position': ((1 - fraction) * initial[side][:3, 3] + fraction * targets[side][:3, 3]).tolist(),
                                     'quaternion': np.roll(rotations[side](fraction).as_quat(), 1).tolist()}
                     for side in initial}
            current = JointState.from_position(torch.tensor(previous[None], device='cuda'), joint_names=ACTIVE_JOINTS)
            with torch.enable_grad():
                solved = solver.solve_pose(goal(poses), current_state=current, return_seeds=32)
            values = solved.js_solution.position.reshape(-1, len(solved.js_solution.joint_names))
            values = values[:, [solved.js_solution.joint_names.index(name) for name in ACTIVE_JOINTS]].contiguous()
            mask = solved.success.reshape(-1).bool() & validity.mask(values)
            candidates = values[mask]
            if not len(candidates):
                raise CycleBlocked(phase, f'笛卡尔进度{fraction:.3f}无合法IK', report)
            weights = torch.tensor([math.radians(15) / .1] + [1.] * 14, device='cuda')
            distance = torch.linalg.vector_norm((candidates - torch.tensor(previous, device='cuda')) * weights, dim=-1)
            next_row = candidates[int(distance.argmin())].cpu().numpy()
            dense = np.asarray(densify(np.stack([previous, next_row])))
            edge_valid = validity.mask(torch.tensor(dense, device='cuda', dtype=torch.float32))
            if not bool(edge_valid.all()):
                from v3_collision_edge_diagnostic import record_collision_edge
                report['conveyor_edge_diagnostic'] = record_collision_edge(self, checker, validity, dense,
                                                                          edge_valid, endpoint, names, fraction)
                raise CycleBlocked(phase, f'笛卡尔进度{fraction:.3f}插值边碰撞', report)
            mobile = np.repeat(endpoint[None], len(dense), axis=0)
            mobile[:, [names.index(name) for name in ACTIVE_JOINTS]] = dense
            append(mobile, phase, payload)
            previous = next_row
        return previous

    def conveyor_suffix(self, boxes, report, append, progress):
        began = time.perf_counter()
        original_snapshot = self.snapshot
        names = report['joint_names']
        endpoint = np.asarray(report['frames'][-1])
        pick_endpoint = endpoint.copy()
        yaw = math.pi
        start = endpoint[[names.index(name) for name in ACTIVE_JOINTS]]
        reference_targets = self.conveyor_reference_targets()
        initial_tools = self.tool_matrices(start)
        required_bases = []
        rotation = Rotation.from_euler('z', yaw).as_matrix()
        for side, tool in initial_tools.items():
            box = self.placement_box_seed(side, tool)
            box[0, 3] += self.conveyor_push_m
            goal_position = np.asarray(reference_targets[side]['position']).copy()
            goal_position[0] -= self.base_advance
            required_bases.append((goal_position - rotation @ box[:3, 3])[:2])
        parking, parking_residual = self.shared_parking_pose(required_bases)
        report['placement_base_inverse'] = dict(required_bases_m=np.asarray(required_bases).tolist(),
                                                selected_m=parking.tolist(),
                                                maximum_residual_m=parking_residual)
        endpoint[[names.index('base_x'), names.index('base_y')]] = parking
        endpoint[names.index('base_yaw')] = yaw
        append([endpoint], 'base_reposition', True)
        base = np.eye(4)
        base[:3, :3] = rotation
        base[:2, 3] = endpoint[[names.index('base_x'), names.index('base_y')]]
        state = original_snapshot.state.with_positions(dict(zip(names, endpoint)), matrix_pose(base))
        self.snapshot = replace(original_snapshot, state=state)
        report['conveyor_collision_validation'] = 'enabled: frozen twin conveyor STL meshes; no navigation path validation'
        report['conveyor_collision_mesh'] = str(DEFAULT_CONVEYOR_MESH)
        report['placed_boxes'] = {}
        report['conveyor_box_targets'] = {}
        report['release_policy'] = 'remove released boxes from scene and visualization'
        report['conveyor_push_m'] = self.conveyor_push_m
        report['placement_base_pose'] = {'position': [*parking.tolist(), 0.], 'yaw_rad': yaw}
        try:
            progress('导航到位瞬移完成；底盘冻结，求固定放箱目标IK并用RRT连接，启用双龙头车碰撞')
            timing=report.setdefault('conveyor_pipeline_timing_ms',{})
            setup_started=time.perf_counter()
            loaded_checker = self.checker(boxes, payload=True)
            loaded_validity = self.placement_validity(loaded_checker, True)
            solver = self.conveyor_ik_solver(boxes)
            torch.cuda.synchronize();timing['loaded_checker_and_solver_setup']=(time.perf_counter()-setup_started)*1000
            targets_started=time.perf_counter()
            targets = {}
            for side, tool in self.tool_matrices(start).items():
                box = self.placement_box_seed(side, tool)
                box[0, 3] += self.conveyor_push_m
                box[:3, :3] = Rotation.from_euler('z', math.atan2(box[1, 0], box[0, 0])).as_matrix()
                box[2, 3] = self.conveyor_height_m + self.box_fit['dimensions_m'][2] * .5 + self.placement_clearance_m
                targets[side] = box @ np.linalg.inv(self.payload_transform(side))
                report['conveyor_box_targets'][side] = {'position': (base @ box)[:3, 3].tolist(),
                    'quaternion_wxyz': matrix_pose(base @ box).quaternion_wxyz,
                    'table_height_m': self.conveyor_height_m, 'box_height_m': self.box_fit['dimensions_m'][2],
                    'conveyor_position': reference_targets[side]['conveyor_position']}
            timing['placement_target_build']=(time.perf_counter()-targets_started)*1000
            placed = self.plan_conveyor_target(start, targets, loaded_checker, loaded_validity, solver,
                                                report, append)
            cleanup_started=time.perf_counter()
            del loaded_checker, loaded_validity
            # Keep allocator/checker resources warm for time-parameterization and the next pair.
            timing['loaded_cleanup']=(time.perf_counter()-cleanup_started)*1000
            for side, tool in self.tool_matrices(placed).items():
                pose = matrix_pose(base @ tool @ self.payload_transform(side))
                report['placed_boxes'][side] = {'object_id': f"wall_box_{boxes[side]:02d}",
                    'position': pose.position, 'quaternion_wxyz': pose.quaternion_wxyz,
                    'dimensions_m': self.box_fit['dimensions_m']}
            append([report['frames'][-1]], 'release', False)
            progress('释放后箱子移出场景，直接用RRT回Home，底盘保持冻结')
            home_setup_started=time.perf_counter()
            empty_checker = self.checker(boxes)
            empty_validity = self.placement_validity(empty_checker, False)
            timing['empty_checker_setup']=(time.perf_counter()-home_setup_started)*1000
            home_search_started=time.perf_counter()
            path, stats = self.joint_plan(placed, self.home_values, empty_checker, empty_validity,
                                          20261014, stop_at_first_valid=True)
            torch.cuda.synchronize();timing['home_rrt_and_validation']=(time.perf_counter()-home_search_started)*1000
            report['conveyor_home_search_stats'] = stats
            if path is None:
                raise CycleBlocked('保持转身姿态回Home', f'2秒内无路径：{stats}', report)
            mobile = np.repeat(np.asarray(report['frames'][-1])[None], len(path), axis=0)
            mobile[:, [names.index(name) for name in ACTIVE_JOINTS]] = path
            append(mobile, 'return_home', False)
            progress('双臂已回Home，等待导航完成后瞬移回抓取底盘位')
            returned = np.asarray(report['frames'][-1]).copy()
            for name in ('base_x', 'base_y', 'base_yaw'):
                returned[names.index(name)] = pick_endpoint[names.index(name)]
            append([returned], 'base_return', False)
            report['conveyor_suffix_ms'] = (time.perf_counter() - began) * 1000
        finally:
            self.snapshot = original_snapshot
