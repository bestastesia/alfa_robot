import copy
from contextlib import contextmanager
import gc
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch
import yaml

from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.trajectory_optimizer import TrajectoryOptimizer, TrajectoryOptimizerCfg
from curobo.types import GoalToolPose, JointState, Pose

from v3_batched_loaded_search import (
    GpuValidity, batched_rrt_multi_goal, densify, loaded_robot,
    batched_rrt_connect_multi_goal, batched_prm_multi_goal,
)
from v3_search_helpers import audit_state
from v3_wall_ik_benchmark import (
    ACTIVE_JOINTS, canonical_side_suction_quaternion_wxyz,
    canonical_side_tool_to_box, chassis_front_x, make_scene,
)
from .fixtures import WallLayout, tasks
from .adapter import from_curobo_scene, task_attachments, to_curobo_scene, task_contact_positions, suction_quaternion, pose_matrix
from .scene import RobotState, SceneSnapshot, digest
from .trajectory import joint_state_trajectory, resample_path
from .distance_metric import joint_distance_weights
from .cache import resource_key

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TARGET_POSES = WORKSPACE_ROOT / 'generated/current_carry_target_6d.json'


@contextmanager
def preserve_torch_rng():
    """Keep optional TrajOpt calls from perturbing later IK randomness."""
    cpu_state = torch.random.get_rng_state()
    cuda_state = torch.cuda.get_rng_state_all()
    try:
        yield
    finally:
        torch.random.set_rng_state(cpu_state)
        torch.cuda.set_rng_state_all(cuda_state)


def goal(poses):
    frames = [frame for frame in ('left_tool0', 'right_tool0') if frame in poses]
    return GoalToolPose.from_poses({
        frame: Pose(
            position=torch.tensor(poses[frame]['position'], device='cuda', dtype=torch.float32).reshape(1, 3),
            quaternion=torch.tensor(poses[frame]['quaternion'], device='cuda', dtype=torch.float32).reshape(1, 4),
        ) for frame in frames
    }, ordered_tool_frames=frames)


def pose_dict(pose):
    return {
        'position': pose.position.reshape(-1, 3)[0].detach().cpu().numpy(),
        'quaternion': pose.quaternion.reshape(-1, 4)[0].detach().cpu().numpy(),
    }


class CuroboBackend:
    def __init__(self, args):
        self.args = args
        self.robot = yaml.safe_load(args.robot_config.read_text())
        self.mobile_robot = yaml.safe_load(args.mobile_robot_config.read_text())
        for robot in (self.robot, self.mobile_robot):
            cspace = robot.get("robot_cfg", robot)["kinematics"]["cspace"]
            cspace["cspace_distance_weight"] = joint_distance_weights(
                cspace["joint_names"], cspace.get("cspace_distance_weight"))
        self.box_fit = json.loads(args.box_fit.read_text())
        self.home = yaml.safe_load(args.named_poses.read_text())["named_poses"]["home"]
        self.front = chassis_front_x(args.urdf, self.home)
        self.home_values = np.asarray(
            [self.home[name] for name in ACTIVE_JOINTS], dtype=np.float32
        )
        self.target_values = np.zeros(15, dtype=np.float32)
        self.target_values[0] = 0.0
        self.target_values[1:] = np.deg2rad([
            130, -105, 10, 90, -90, -40, 0,
            -130, 105, -10, -90, 90, 40, 0,
        ])
        self.wall_layout = getattr(args, "wall_layout", WallLayout())
        if not isinstance(self.wall_layout, WallLayout):
            raise TypeError("wall_layout must be a WallLayout")
        self.task_options = tasks(self.wall_layout)
        if not self.task_options:
            raise ValueError("wall layout contains no task boxes")
        self.task_map = dict(self.task_options)
        self.selected_task = self.task_options[0][1]
        self.random_start = None
        self.result = None
        self.busy = False
        self.random_counter = 0
        self.planner_kind = "informed_rrt"
        self.search_seed = None
        self.suction_mode = "side"
        self.contact_offsets = {}
        self.model_id = self._model_identity()
        self.snapshot = SceneSnapshot(
            self.model_id, RobotState.from_mapping(self.home, time.time_ns()),
            from_curobo_scene(make_scene(self.front, self.wall_layout.distance_m, set(range(25)))) +
            self.wall_layout.objects(self.front),
        )
        self.target_poses = json.loads(Path(getattr(args, "target_poses", DEFAULT_TARGET_POSES)).read_text())
        self.default_target_poses = copy.deepcopy(self.target_poses)
        self._cached_task = None
        self._cached_solver = None
        self._cached_checkers = {}
        self._cached_trajectory_optimizers = {}

    def motion_weights(self, boxes):
        weights = np.ones(len(ACTIVE_JOINTS), dtype=np.float32)
        weights[0] = 5.
        if len(boxes) == 1:
            idle = next(side for side in ("left", "right") if side not in boxes)
            for index, name in enumerate(ACTIVE_JOINTS):
                if name.startswith(idle + "_joint"):
                    weights[index] = 3.
        return weights

    def motion_bounds(self, lower, upper, boxes):
        """Task-space search restriction, not a change to physical joint limits."""
        lower, upper = lower.clone(), upper.clone()
        if len(boxes) == 1:
            for name in ("left_joint1", "right_joint1"):
                index = ACTIVE_JOINTS.index(name)
                lower[index] = max(float(lower[index]), float(self.home_values[index]) - math.pi)
                upper[index] = min(float(upper[index]), float(self.home_values[index]) + math.pi)
        return lower, upper

    def search_seed_for(self, offset=0):
        base = 20261003 if self.search_seed is None else self.search_seed
        return (base + offset) % 2**32

    def search_path(self, start, goals, lower, upper, validity, budget, seed, apply_shortcut=False):
        if self.planner_kind in ("rrtconnect", "informed_connect"):
            return batched_rrt_connect_multi_goal(
                start, goals, lower, upper, validity, budget, seed,
                informed_sampling=self.planner_kind == "informed_connect",
                stop_at_first_valid=self.planner_kind == "informed_connect",
            )
        if self.planner_kind == "bitstar":
            from v3_gpu_bitstar import batched_bitstar_multi_goal

            return batched_bitstar_multi_goal(start, goals, lower, upper, validity, budget, seed)
        if self.planner_kind == "prm":
            return batched_prm_multi_goal(start, goals, lower, upper, validity, budget, seed)
        return batched_rrt_multi_goal(
            start, goals, lower, upper, validity, budget, seed,
            apply_shortcut=apply_shortcut, informed_sampling=self.planner_kind == "informed_rrt",
        )

    def prepare_task(self, boxes):
        key = resource_key(self.snapshot, boxes, self.box_fit, self.suction_mode)
        if self.contact_offsets:
            key = (*key, digest(self.contact_offsets))
        if key != self._cached_task:
            self._cached_solver = None
            self._cached_checkers.clear()
            if not hasattr(self, "_cached_trajectory_optimizers"):
                self._cached_trajectory_optimizers = {}
            self._cached_task = key
            gc.collect()

    def _model_identity(self):
        import hashlib

        return digest({"urdf": hashlib.sha256(self.args.urdf.read_bytes()).hexdigest(),
                       "fixed": self.robot, "mobile": self.mobile_robot})

    def set_snapshot(self, snapshot):
        if snapshot.model_id != self.model_id:
            raise ValueError("snapshot model does not match configured assets")
        if snapshot.attachments:
            raise ValueError("full cycle must start empty; scene store supports independent attachment updates")
        if snapshot.policy.ground_support_link != "base_link":
            raise ValueError("current checker only exempts base_link support")
        self.snapshot = snapshot

    def ik_solver(self, boxes, all_tools=False):
        self.prepare_task(boxes)
        frames = [side + "_tool0" for side in ("left", "right") if all_tools or side in boxes]
        if self._cached_solver is None or frames != self._cached_solver_frames:
            self._cached_solver_frames = frames
            robot = copy.deepcopy(self.robot)
            robot.get("robot_cfg", robot)["kinematics"]["tool_frames"] = frames
            self._cached_solver = InverseKinematics(InverseKinematicsCfg.create(
                robot=robot, scene_model=self.scene(boxes),
                collision_cache={"cuboid": max(40, len(self.snapshot.objects)),
                                 "mesh": len(self.snapshot.meshes)}, num_seeds=512,
                self_collision_check=True, use_cuda_graph=False,
                position_tolerance=0.002, orientation_tolerance=math.radians(1),
                override_iters_for_multi_link_ik=500,
                optimizer_collision_activation_distance=0.005,
            ))
        return self._cached_solver

    def task_attachments(self, boxes):
        return task_attachments(self.snapshot, boxes, self.suction_mode, self.contact_offsets)

    def attachments(self, boxes):
        return self.task_attachments(boxes)

    def collision_checker(self, boxes, payload=False, mobile=False):
        self.prepare_task(boxes)
        key = (payload, mobile)
        if key not in self._cached_checkers:
            robot = self.mobile_robot if mobile else self.robot
            configured = loaded_robot(robot, self.box_fit, active_sides=tuple(boxes), attachments=self.attachments(boxes)) if payload else copy.deepcopy(robot)
            self._cached_checkers[key] = RobotCollisionChecker(
                RobotCollisionCheckerCfg.load_from_config(
                    robot_config=configured, scene_model=self.scene(boxes, mobile=mobile),
                    n_cuboids=max(40, len(self.snapshot.objects)), n_meshes=len(self.snapshot.meshes),
                    collision_activation_distance=0.0,
                ))
            self._cached_checkers[key].task_tool_to_box = {
                item.parent_link.removesuffix("_tool0"): pose_matrix(item.tool_to_object)
                for item in self.attachments(boxes)}
        return self._cached_checkers[key]

    def scene(self, boxes, mobile=False):
        return to_curobo_scene(self.snapshot,
                               (f"wall_box_{box_id:02d}" for box_id in boxes.values()), mobile)

    def trajectory_optimizer(self, boxes, payload=False):
        """Return a cached kinematic TrajOpt solver; dynamics stay disabled until calibrated."""
        self.prepare_task(boxes)
        key = (("loaded", tuple(sorted(boxes)), self.suction_mode,
                digest(self.contact_offsets), digest(self.box_fit)) if payload else ("empty",))
        if key not in self._cached_trajectory_optimizers:
            robot = loaded_robot(
                self.robot, self.box_fit, active_sides=tuple(boxes),
                attachments=self.attachments(boxes),
            ) if payload else copy.deepcopy(self.robot)
            robot.get("robot_cfg", robot)["load_dynamics"] = False
            with preserve_torch_rng():
                config = TrajectoryOptimizerCfg.create(
                    robot=robot,
                    scene_model=self.scene(boxes),
                    collision_cache={"cuboid": max(40, len(self.snapshot.objects)),
                                     "mesh": len(self.snapshot.meshes)},
                    num_seeds=1,
                    self_collision_check=True,
                    use_cuda_graph=True,
                    optimizer_collision_activation_distance=0.0,
                    interpolation_dt=float(getattr(self.args, "trajopt_interpolation_dt", 0.025)),
                    interpolation_buffer_size=1000,
                )
                self._cached_trajectory_optimizers[key] = TrajectoryOptimizer(config)
        else:
            self._cached_trajectory_optimizers[key].scene_collision_checker.load_collision_model(
                self.scene(boxes))
        return self._cached_trajectory_optimizers[key]

    def optimize_rrt_path(self, path, validity, boxes, payload=False):
        """Optimize one 15-DoF RRT path and retain the original path on failure."""
        optimizer = self.trajectory_optimizer(boxes, payload=payload)
        seed = resample_path(path, optimizer.action_horizon, validity.weights.detach().cpu().numpy())
        seed = torch.tensor(seed, device="cuda", dtype=torch.float32).reshape(
            1, 1, optimizer.action_horizon, len(ACTIVE_JOINTS))
        state = lambda row: JointState.from_position(
            torch.tensor(np.asarray(row).reshape(1, -1), device="cuda", dtype=torch.float32),
            joint_names=ACTIVE_JOINTS,
        )
        torch.cuda.synchronize()
        started = time.perf_counter()
        with preserve_torch_rng():
            result = optimizer.solve_cspace(
                goal_state=state(path[-1]), current_state=state(path[0]), seed_traj=seed,
                num_seeds=1, return_seeds=1, finetune_attempts=2,
            )
        torch.cuda.synchronize()
        wall_ms = (time.perf_counter() - started) * 1000.0
        telemetry = {
            "attempted": True,
            "accepted": False,
            "wall_ms": wall_ms,
            "solve_time_ms": float(result.solve_time) * 1000.0,
            "source_waypoints": len(path),
            "seed_waypoints": optimizer.action_horizon,
            "payload_collision_model": bool(payload),
            "dynamics_enabled": False,
            "torque_constraints_enabled": False,
        }
        solver_success = bool(result.success.reshape(-1)[0].item())
        constraint_residual = max(
            (float(value.detach().max().cpu())
             for metrics in (result.metrics, result.interpolated_metrics) if metrics is not None
             for collection in (metrics.costs_and_constraints.constraints,
                                metrics.costs_and_constraints.hybrid_costs_constraints)
             for value in collection.values),
            default=0.0,
        )
        telemetry["solver_success"] = solver_success
        telemetry["maximum_constraint_residual"] = constraint_residual
        if not solver_success and constraint_residual > 1e-6:
            telemetry["fallback_reason"] = "cuRobo TrajOpt constraints did not converge"
            return None, telemetry
        trajectory = result.get_interpolated_plan()
        if trajectory is None:
            telemetry["fallback_reason"] = "cuRobo TrajOpt returned no interpolated trajectory"
            return None, telemetry
        timed = joint_state_trajectory(trajectory, ACTIVE_JOINTS)
        positions = np.asarray(timed["positions"], dtype=np.float32)
        if (not np.allclose(positions[0], path[0], atol=1e-5, rtol=0) or
                not np.allclose(positions[-1], path[-1], atol=1e-5, rtol=0)):
            telemetry["fallback_reason"] = "optimized trajectory changed an endpoint"
            return None, telemetry
        positions[0] = np.asarray(path[0], dtype=np.float32)
        positions[-1] = np.asarray(path[-1], dtype=np.float32)
        timed["positions"] = positions.tolist()
        values = torch.tensor(positions, device="cuda", dtype=torch.float32)
        node_mask = validity.mask(values)
        edge_mask = (torch.ones(0, dtype=torch.bool, device="cuda") if len(values) < 2 else
                     validity.edges(values[:-1], values[1:], resolution=math.radians(0.5)))
        invalid_nodes = torch.nonzero(~node_mask, as_tuple=False).reshape(-1).cpu().tolist()
        invalid_edges = torch.nonzero(~edge_mask, as_tuple=False).reshape(-1).cpu().tolist()
        telemetry["project_invalid_nodes"] = invalid_nodes[:20]
        telemetry["project_invalid_edges"] = invalid_edges[:20]
        if invalid_nodes or invalid_edges:
            telemetry["fallback_reason"] = "project collision/stability gate rejected optimized trajectory"
            return None, telemetry
        telemetry.update(accepted=True, output_frames=len(positions), trajectory=timed)
        return positions, telemetry

    def random_extract_start(self, boxes):
        started_all = time.perf_counter()
        scene = self.scene(boxes)
        solver = InverseKinematics(InverseKinematicsCfg.create(
            robot=copy.deepcopy(self.robot), scene_model=scene,
            collision_cache={"cuboid": max(40, len(self.snapshot.objects)),
                             "mesh": len(self.snapshot.meshes)}, num_seeds=512,
            self_collision_check=True, use_cuda_graph=False,
            position_tolerance=0.002, orientation_tolerance=math.radians(1.0),
            override_iters_for_multi_link_ik=500,
            optimizer_collision_activation_distance=0.005,
        ))
        home_state = JointState.from_position(
            torch.tensor(self.home_values, device="cuda").reshape(1, -1),
            joint_names=ACTIVE_JOINTS, weights=self.motion_weights(boxes).tolist(),
        )
        contact_poses = {}
        for side, position in task_contact_positions(self.snapshot, boxes, 1e-6, self.suction_mode, self.contact_offsets).items():
            contact_poses[f"{side}_tool0"] = {
                "position": position,
                "quaternion": suction_quaternion(side, self.suction_mode),
            }
        contact = solver.solve_pose(goal(contact_poses), home_state, return_seeds=32)
        names = list(contact.js_solution.joint_names)
        values = contact.js_solution.position.reshape(-1, len(names))
        success = torch.nonzero(contact.success.reshape(-1).bool(), as_tuple=False).reshape(-1)
        if not len(success):
            raise RuntimeError("接触IK无解")
        indices = torch.tensor([names.index(name) for name in ACTIVE_JOINTS], device="cuda")
        contact_candidates = values[success][:, indices]
        generator = torch.Generator(device="cuda")
        generator.manual_seed(20261003 + self.random_counter)
        order = torch.randperm(len(contact_candidates), generator=generator, device="cuda")
        contact_candidates = contact_candidates[order]
        loaded = loaded_robot(self.robot, self.box_fit, active_sides=tuple(boxes),
                              attachments=self.attachments(boxes))
        checker = RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(
            robot_config=copy.deepcopy(loaded), scene_model=scene,
            n_cuboids=max(40, len(self.snapshot.objects)), n_meshes=len(self.snapshot.meshes),
            collision_activation_distance=0.0,
        ))
        validity = GpuValidity(
            checker, active_sides=tuple(boxes), attachments=self.attachments(boxes), max_box_tilt_deg=self.snapshot.policy.max_box_tilt_deg,
            stability_weight=10.0, check_ground=True, ground_z=self.snapshot.policy.ground_z_m,
            joint_names=ACTIVE_JOINTS,
        )
        candidates = []
        for contact_values in contact_candidates:
            contact_state = JointState.from_position(
                contact_values.reshape(1, -1), joint_names=ACTIVE_JOINTS
            )
            actual = solver.compute_kinematics(contact_state).tool_poses.to_dict()
            extract_poses = {frame: pose_dict(actual[frame]) for frame in contact_poses}
            for frame in extract_poses:
                if frame.removesuffix("_tool0") not in boxes:
                    continue
                extract_poses[frame]["position"] = extract_poses[frame]["position"].copy()
                axis = 2 if self.suction_mode == "top" else 0
                extract_poses[frame]["position"][axis] += 0.35 if self.suction_mode == "top" else -0.35
            extracted = solver.solve_pose(
                goal(extract_poses), contact_state, return_seeds=16
            )
            extracted_names = list(extracted.js_solution.joint_names)
            extracted_values = extracted.js_solution.position.reshape(-1, len(extracted_names))
            extracted_success = torch.nonzero(
                extracted.success.reshape(-1).bool(), as_tuple=False
            ).reshape(-1)
            if not len(extracted_success):
                continue
            extracted_indices = torch.tensor(
                [extracted_names.index(name) for name in ACTIVE_JOINTS], device="cuda"
            )
            candidates.extend(
                extracted_values[extracted_success][:, extracted_indices]
            )
        if not candidates:
            raise RuntimeError("抽离终点IK无解")
        candidate_tensor = torch.stack(candidates)
        valid = validity.mask(candidate_tensor)
        valid_indices = torch.nonzero(valid, as_tuple=False).reshape(-1)
        if not len(valid_indices):
            raise RuntimeError("抽离终点全部碰撞")
        selected = candidate_tensor[valid_indices[0]].detach().cpu().numpy()
        self.random_counter += 1
        del solver, checker, validity
        gc.collect()
        torch.cuda.empty_cache()
        return {
            "values": selected.tolist(),
            "generation_ms": (time.perf_counter() - started_all) * 1000.0,
            "candidate_count": int(len(valid_indices)),
        }

    def plan(self, boxes, start_values):
        started_all = time.perf_counter()
        timings = {}
        scene = self.scene(boxes)
        checker = self.collision_checker(boxes, payload=True)
        validity = GpuValidity(
            checker, active_sides=tuple(boxes), attachments=self.attachments(boxes), max_box_tilt_deg=self.snapshot.policy.max_box_tilt_deg,
            stability_weight=10.0, check_ground=True, ground_z=self.snapshot.policy.ground_z_m,
            joint_names=ACTIVE_JOINTS,
        )
        start = torch.tensor(start_values, device="cuda", dtype=torch.float32)
        # The empty arm must also reach its specified carry pose before the chassis turns.
        target_solver = self.ik_solver(boxes, all_tools=True)
        target_poses = self.target_poses
        torch.cuda.synchronize()
        timings["setup"] = (time.perf_counter() - started_all) * 1000
        started_ik = time.perf_counter()
        target_result = target_solver.solve_pose(
            goal(target_poses), JointState.from_position(
                start.reshape(1, -1), joint_names=ACTIVE_JOINTS
            ), return_seeds=64,
        )
        torch.cuda.synchronize()
        target_ik_ms = (time.perf_counter() - started_ik) * 1000.0
        timings["target_ik"] = target_ik_ms
        started_filter = time.perf_counter()
        names = list(target_result.js_solution.joint_names)
        values = target_result.js_solution.position.reshape(-1, len(names))
        successful = torch.nonzero(
            target_result.success.reshape(-1).bool(), as_tuple=False
        ).reshape(-1)
        indices = torch.tensor([names.index(name) for name in ACTIVE_JOINTS], device="cuda")
        goals = values[successful][:, indices]
        if not len(goals):
            raise RuntimeError("目标IK成功0个")
        full_valid, stability = validity.evaluate(goals)
        if not bool(full_valid.any().item()):
            collision_only = GpuValidity(
                checker,
                active_sides=(),
                check_ground=True,
                ground_z=self.snapshot.policy.ground_z_m,
                joint_names=ACTIVE_JOINTS,
            )
            collision_valid = collision_only.mask(goals)
            stability_valid = stability <= 1.0 - validity.minimum_box_up_z
            raise RuntimeError(
                f"目标IK成功{len(goals)}个；碰撞通过{int(collision_valid.sum().item())}个；"
                f"箱体稳定性通过{int(stability_valid.sum().item())}个；联合合法0个"
            )
        goals = goals[full_valid]
        bounds = checker.kinematics.get_joint_limits().position
        lower, upper = self.motion_bounds(bounds[0] + 1e-5, bounds[1] - 1e-5, boxes)
        if len(boxes) == 1:
            goals = goals[((goals >= lower) & (goals <= upper)).all(dim=1)]
            if not len(goals):
                raise RuntimeError("搬运目标没有满足肩关节半圈运动约束的IK候选")
        if self.suction_mode == "top" or self.contact_offsets:
            # A static carry IK is insufficient: reject elbow/box poses that hit the room during yaw.
            mobile_names = list(self.mobile_robot["kinematics"]["cspace"]["joint_names"])
            mobile_checker = self.collision_checker(boxes, payload=True, mobile=True)
            turn_validity = GpuValidity(
                mobile_checker, active_sides=tuple(boxes),
                attachments=self.attachments(boxes),
                max_box_tilt_deg=self.snapshot.policy.max_box_tilt_deg, check_ground=True,
                ground_z=self.snapshot.policy.ground_z_m, joint_names=mobile_names,
                weights=[2., 2., 1.] + self.motion_weights(boxes).tolist())
            turning = torch.zeros((len(goals), 361, len(mobile_names)), device=goals.device, dtype=goals.dtype)
            for j, name in enumerate(ACTIVE_JOINTS):
                turning[:, :, mobile_names.index(name)] = goals[:, j, None]
            turning[:, :, mobile_names.index("base_yaw")] = torch.linspace(0., math.pi, 361, device=goals.device)
            safe = turn_validity.mask(turning.reshape(-1, len(mobile_names))).reshape(len(goals), 361).all(dim=1)
            goals = goals[safe]
            if not len(goals):
                raise RuntimeError("当前抓取搬运目标无可安全转身的IK候选")
        metric = validity.weights
        goals = goals[torch.argsort(torch.sum(((goals - start) * metric) ** 2, dim=1))[:16]]
        torch.cuda.synchronize()
        timings["target_filter"] = (time.perf_counter() - started_filter) * 1000
        started_search = time.perf_counter()
        search_seed = self.search_seed_for(self.random_counter if self.search_seed is None else 0)
        path, stats = self.search_path(
            start, goals,
            lower, upper, validity, 2.0, search_seed,
            apply_shortcut=len(boxes) == 1,
        )
        stats["search_seed"] = search_seed
        search_ms = (time.perf_counter() - started_search) * 1000.0
        timings["rrt"] = search_ms
        started_assembly = time.perf_counter()
        if path is None:
            raise RuntimeError("2秒内未找到搬运路径")
        transport = None
        if getattr(self.args, "trajopt_rrt", False):
            try:
                transport, trajectory_optimization = self.optimize_rrt_path(
                    path, validity, boxes, payload=True)
            except (RuntimeError, ValueError) as error:
                trajectory_optimization = {
                    "attempted": True, "accepted": False, "payload_collision_model": True,
                    "dynamics_enabled": False, "torque_constraints_enabled": False,
                    "fallback_reason": f"{type(error).__name__}: {error}",
                }
            stats["trajectory_optimization"] = trajectory_optimization
        if transport is None:
            transport = densify(path, [5.0] + [1.0] * 14)
            if getattr(self.args, "trajopt_rrt", False):
                stats["trajectory_fallback_waypoints"] = np.asarray(path).tolist()
        mobile_names = list(
            self.mobile_robot.get("robot_cfg", self.mobile_robot)["kinematics"]["cspace"]["joint_names"]
        )
        full_frames = []
        for values15 in transport:
            mapping = dict(zip(ACTIVE_JOINTS, values15))
            full_frames.append([mapping.get(name, 0.0) for name in mobile_names])
        endpoint = full_frames[-1]
        yaw_index = mobile_names.index("base_yaw")
        for step in range(1, 361):
            frame = endpoint.copy()
            frame[yaw_index] = math.pi * step / 360.0
            full_frames.append(frame)
        timings["frame_assembly"] = (time.perf_counter() - started_assembly) * 1000
        started_mobile_setup = time.perf_counter()
        mobile_checker = self.collision_checker(boxes, payload=True, mobile=True)
        mobile_validity = GpuValidity(
            mobile_checker, active_sides=tuple(boxes), attachments=self.attachments(boxes), max_box_tilt_deg=self.snapshot.policy.max_box_tilt_deg,
            stability_weight=10.0, check_ground=True, ground_z=self.snapshot.policy.ground_z_m,
            joint_names=mobile_names,
            weights=[2.0, 2.0, 1.0] + self.motion_weights(boxes).tolist(),
        )
        torch.cuda.synchronize()
        timings["mobile_checker_setup"] = (time.perf_counter() - started_mobile_setup) * 1000
        started_validation = time.perf_counter()
        combined_valid, combined_stability = mobile_validity.evaluate(torch.tensor(
            full_frames, device="cuda", dtype=torch.float32
        ))
        invalid = torch.nonzero(~combined_valid, as_tuple=False).reshape(-1)
        torch.cuda.synchronize()
        timings["transport_and_turn_validation"] = (time.perf_counter() - started_validation) * 1000
        failure = None
        if len(invalid):
            index = int(invalid[0].item())
            stage = "搬运" if index < len(transport) else "Yaw180"
            failure_state = JointState.from_position(
                torch.tensor([full_frames[index]], device="cuda", dtype=torch.float32),
                joint_names=mobile_names,
            )
            failure = {
                "frame_index": index,
                "stage": stage,
                "yaw_deg": math.degrees(full_frames[index][yaw_index]),
                "audit": audit_state(mobile_checker.kinematics, failure_state, scene),
            }
            failure_fk = mobile_checker.kinematics.compute_kinematics(failure_state)
            failure["spheres"] = failure_fk.robot_spheres.detach().cpu().reshape(-1, 4).tolist()
        result = {
            "success": failure is None,
            "failure": failure,
            "valid_frames": combined_valid.cpu().tolist(),
            "joint_names": mobile_names,
            "frames": full_frames,
            "transport_frames": len(transport),
            "yaw_frames": 360,
            "target_ik_ms": target_ik_ms,
            "search_ms": search_ms,
            "total_ms": (time.perf_counter() - started_all) * 1000.0,
            "rrt_stats": stats,
            "max_box_tilt_deg": math.degrees(math.acos(max(
                -1.0, min(1.0, 1.0 - float(combined_stability.max().item()))
            ))),
        }
        started_cleanup = time.perf_counter()
        del target_solver, checker, validity, mobile_checker, mobile_validity
        torch.cuda.synchronize()
        timings["cleanup"] = (time.perf_counter() - started_cleanup) * 1000
        result["timing_ms"] = timings
        return result
