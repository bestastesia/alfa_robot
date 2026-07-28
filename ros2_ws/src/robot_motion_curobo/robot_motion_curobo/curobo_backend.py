"""Persistent cuRobo MotionPlanner adapter used by the ROS service node."""

from __future__ import annotations

import importlib
import math
import os
import sys
import time
from copy import deepcopy
from pathlib import Path
from typing import Sequence

import yaml

from .planner_core import (
    AUTHORITY_JOINT_NAMES,
    AttachedPayload,
    BackendResult,
    BackendTrajectory,
)


LEGACY_CUROBO_JOINT_NAMES = (
    "updown",
    "leftjoint1",
    "leftjoint2",
    "leftjoint3",
    "leftjoint4",
    "leftjoint5",
    "leftjoint6",
    "rightjoint1",
    "rightjoint2",
    "rightjoint3",
    "rightjoint4",
    "rightjoint5",
    "rightjoint6",
)
_LEGACY_TO_AUTHORITY = dict(zip(LEGACY_CUROBO_JOINT_NAMES, AUTHORITY_JOINT_NAMES))


def authority_joint_name(name: str) -> str:
    """Normalize a legacy cuRobo model joint name to the current ROS contract."""
    return _LEGACY_TO_AUTHORITY.get(name, name)


def configure_python_paths(curobo_root: str, dependency_venv: str) -> None:
    if dependency_venv:
        venv = Path(dependency_venv).expanduser().resolve()
        version = f"python{sys.version_info.major}.{sys.version_info.minor}"
        for candidate in (
            venv / "lib" / version / "site-packages",
            venv / "local" / "lib" / version / "dist-packages",
        ):
            if candidate.exists() and str(candidate) not in sys.path:
                sys.path.insert(0, str(candidate))
    if curobo_root:
        root = Path(curobo_root).expanduser().resolve()
        if not (root / "curobo" / "__init__.py").exists():
            raise FileNotFoundError(f"curobo_python_root_invalid:{root}")
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))

    # The project's portable cuRobo checkout keeps the public compatibility
    # shims under curobo._src. Register them without modifying that checkout.
    package = importlib.import_module("curobo")
    for public_name, internal_name in (
        ("curobo.runtime", "curobo._src.runtime"),
        ("curobo.logging", "curobo._src.util.logging"),
    ):
        try:
            module = importlib.import_module(internal_name)
        except ModuleNotFoundError:
            continue
        sys.modules.setdefault(public_name, module)
        setattr(package, public_name.rsplit(".", 1)[-1], module)


def load_robot_config(robot_path: Path) -> dict:
    """Load an external robot YAML and make all referenced assets portable."""
    document = yaml.safe_load(robot_path.read_text())
    if not isinstance(document, dict):
        raise ValueError(f"robot_config_invalid:{robot_path}")
    robot_cfg = document.get("robot_cfg", document)
    if not isinstance(robot_cfg, dict) or not isinstance(robot_cfg.get("kinematics"), dict):
        raise ValueError(f"robot_config_missing_kinematics:{robot_path}")
    kinematics = robot_cfg["kinematics"]
    asset_root = Path(str(kinematics.get("asset_root_path", "")))
    if not asset_root.is_absolute():
        asset_root = (robot_path.parent / asset_root).resolve()
    urdf_value = kinematics.get("urdf_path")
    if not isinstance(urdf_value, str) or not urdf_value:
        raise ValueError(f"robot_config_missing_urdf_path:{robot_path}")
    urdf_path = Path(urdf_value)
    if not urdf_path.is_absolute():
        urdf_path = (asset_root / urdf_path).resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(f"robot_urdf_missing:{urdf_path}")
    kinematics["asset_root_path"] = str(asset_root)
    kinematics["urdf_path"] = str(urdf_path)
    collision_spheres = kinematics.get("collision_spheres")
    if isinstance(collision_spheres, str):
        sphere_path = Path(collision_spheres)
        if not sphere_path.is_absolute():
            sphere_path = (robot_path.parent / sphere_path).resolve()
        kinematics["collision_spheres"] = str(sphere_path)
    return document


def canonical_joint_indices(
    source_names: Sequence[str], target_names: Sequence[str] = AUTHORITY_JOINT_NAMES
) -> tuple[int, ...]:
    normalized = tuple(authority_joint_name(name) for name in source_names)
    if len(set(normalized)) != len(normalized):
        raise ValueError("curobo_trajectory_duplicate_joint_names")
    index = {name: position for position, name in enumerate(normalized)}
    missing = [name for name in target_names if name not in index]
    if missing:
        raise ValueError(f"curobo_trajectory_missing_joints:{missing}")
    return tuple(index[name] for name in target_names)


def _rotate_point_xyzw(
    point: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    x, y, z, w = quaternion
    px, py, pz = point
    # Quaternion-vector rotation expanded to avoid a NumPy dependency.
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    return (
        px + w * tx + (y * tz - z * ty),
        py + w * ty + (z * tx - x * tz),
        pz + w * tz + (x * ty - y * tx),
    )


def cuboid_covering_spheres(
    payload: AttachedPayload,
    nominal_cell_size: float,
) -> tuple[tuple[float, float, float, float], ...]:
    """Return a conservative volume cover expressed in the tool/payload link."""
    if not math.isfinite(nominal_cell_size) or nominal_cell_size <= 0.0:
        raise ValueError("payload_grid_resolution_m must be positive")
    counts = tuple(max(1, int(math.ceil(size / nominal_cell_size))) for size in payload.size_xyz)
    cell = tuple(size / count for size, count in zip(payload.size_xyz, counts))
    radius = 0.5 * math.sqrt(sum(value * value for value in cell))
    spheres: list[tuple[float, float, float, float]] = []
    for ix in range(counts[0]):
        for iy in range(counts[1]):
            for iz in range(counts[2]):
                local = (
                    -0.5 * payload.size_xyz[0] + (ix + 0.5) * cell[0],
                    -0.5 * payload.size_xyz[1] + (iy + 0.5) * cell[1],
                    -0.5 * payload.size_xyz[2] + (iz + 0.5) * cell[2],
                )
                rotated = _rotate_point_xyzw(local, payload.orientation_xyzw)
                spheres.append(
                    (
                        payload.center_xyz[0] + rotated[0],
                        payload.center_xyz[1] + rotated[1],
                        payload.center_xyz[2] + rotated[2],
                        radius,
                    )
                )
    return tuple(spheres)


class CuroboBackend:
    """One prewarmed MotionPlanner instance; callers serialize access."""

    def __init__(
        self,
        *,
        robot_config_path: str,
        scene_config_path: str,
        curobo_python_root: str = "",
        dependency_venv: str = "",
        warmup_iterations: int = 5,
        use_cuda_graph: bool = True,
        self_collision_check: bool = True,
        num_ik_seeds: int = 32,
        num_trajopt_seeds: int = 4,
        trajopt_num_iters: int = 100,
        trajopt_inner_iters: int = 25,
        trajopt_history: int = 27,
        trajopt_n_knots: int = 16,
        trajopt_interpolation_steps: int = 4,
        trajopt_finetune_attempts: int = 3,
        trajopt_finetune_dt_scale: float = 0.75,
        optimizer_collision_activation_distance: float = 0.01,
        collision_cache_cuboids: int = 128,
        collision_cache_meshes: int = 8,
        interpolation_dt: float = 0.02,
        left_payload_link: str = "",
        right_payload_link: str = "",
        payload_grid_resolution_m: float = 0.1,
        world_payload_object_template: str = "cargo_box_{box_id:02d}",
        require_world_payload_object: bool = False,
    ) -> None:
        robot_path = Path(robot_config_path).expanduser().resolve()
        scene_path = Path(scene_config_path).expanduser().resolve()
        if not robot_config_path or not robot_path.is_file():
            raise FileNotFoundError(f"robot_config_path_missing:{robot_path}")
        if not scene_config_path or not scene_path.is_file():
            raise FileNotFoundError(f"scene_config_path_missing:{scene_path}")
        if not left_payload_link or not right_payload_link:
            raise ValueError(
                "left_payload_link and right_payload_link must name preallocated cuRobo payload links"
            )

        configure_python_paths(curobo_python_root, dependency_venv)
        import torch
        from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
        from curobo._src.util.config_io import join_path, load_yaml
        from curobo.content import get_task_configs_path

        if not torch.cuda.is_available():
            raise RuntimeError("CUDA unavailable; local-cuRobo service cannot become ready")
        self._torch = torch
        self._interpolation_dt = interpolation_dt
        self._payload_links = {
            "left": left_payload_link,
            "right": right_payload_link,
        }
        self._payload_grid_resolution_m = payload_grid_resolution_m
        self._world_payload_object_template = world_payload_object_template
        self._require_world_payload_object = require_world_payload_object
        self._trajopt_finetune_attempts = max(0, trajopt_finetune_attempts)
        self._trajopt_finetune_dt_scale = float(trajopt_finetune_dt_scale)

        task_root = get_task_configs_path()
        trajopt_optimizer = deepcopy(
            load_yaml(join_path(task_root, "trajopt/lbfgs_bspline_trajopt.yml"))
        )
        trajopt_optimizer["optimizer"]["num_iters"] = max(1, trajopt_num_iters)
        trajopt_optimizer["optimizer"]["inner_iters"] = max(1, trajopt_inner_iters)
        trajopt_optimizer["optimizer"]["history"] = max(1, trajopt_history)
        trajopt_transition = deepcopy(
            load_yaml(join_path(task_root, "trajopt/transition_bspline_trajopt.yml"))
        )
        trajopt_transition["transition_model_cfg"]["n_knots"] = max(4, trajopt_n_knots)
        trajopt_transition["transition_model_cfg"]["interpolation_steps"] = max(
            1, trajopt_interpolation_steps
        )

        planner_cfg = MotionPlannerCfg.create(
            robot=load_robot_config(robot_path),
            scene_model=str(scene_path),
            trajopt_optimizer_configs=[trajopt_optimizer],
            trajopt_transition_model=trajopt_transition,
            collision_cache={
                "cuboid": max(1, collision_cache_cuboids),
                "mesh": max(1, collision_cache_meshes),
            },
            self_collision_check=self_collision_check,
            use_cuda_graph=use_cuda_graph,
            num_ik_seeds=max(1, num_ik_seeds),
            num_trajopt_seeds=max(1, num_trajopt_seeds),
            optimizer_collision_activation_distance=max(
                0.0, optimizer_collision_activation_distance
            ),
        )
        self._planner = MotionPlanner(planner_cfg)
        self._planner_joint_names = tuple(self._planner.joint_names)
        normalized_planner_names = tuple(
            authority_joint_name(name) for name in self._planner_joint_names
        )
        if normalized_planner_names != AUTHORITY_JOINT_NAMES:
            raise RuntimeError(
                "curobo robot config is not the current 13-DoF ALFA contract: "
                f"{self._planner_joint_names}"
            )
        self.joint_names = AUTHORITY_JOINT_NAMES
        limits = self._planner.kinematics.get_joint_limits().position.detach().cpu()
        self.joint_limits = {
            name: (float(limits[0, index]), float(limits[1, index]))
            for index, name in enumerate(self.joint_names)
        }
        self._kinematics_params = self._collect_kinematics_params()
        for side, link in self._payload_links.items():
            for params in self._kinematics_params:
                try:
                    capacity = int(params.get_sphere_index_from_link_name(link).shape[0])
                except Exception as exc:
                    raise RuntimeError(
                        f"curobo payload link missing for {side}:{link}"
                    ) from exc
                if capacity <= 0:
                    raise RuntimeError(f"curobo payload link has no sphere slots:{link}")

        self._torch.cuda.synchronize()
        warmed = self._planner.warmup(
            enable_graph=True,
            num_warmup_iterations=max(1, warmup_iterations),
        )
        self._torch.cuda.synchronize()
        if warmed is False:
            raise RuntimeError("curobo_warmup_failed")
        self.warmup_count = 1

    def _collect_kinematics_params(self) -> tuple[object, ...]:
        candidates: list[object] = []
        for solver in (self._planner.ik_solver, self._planner.trajopt_solver):
            candidates.append(solver.kinematics.config.kinematics_config)
            for rollout in solver.core.get_all_rollout_instances():
                candidates.append(rollout.transition_model.robot_model.config.kinematics_config)
        if self._planner.graph_planner is not None:
            for rollout in (
                self._planner.graph_planner.feasibility_rollout,
                self._planner.graph_planner.auxiliary_rollout,
            ):
                candidates.append(rollout.transition_model.robot_model.config.kinematics_config)
        unique: list[object] = []
        seen: set[int] = set()
        for value in candidates:
            if id(value) not in seen:
                seen.add(id(value))
                unique.append(value)
        return tuple(unique)

    def _make_joint_state(self, positions: Sequence[float]):
        from curobo.types import JointState

        tensor = self._torch.as_tensor(
            positions,
            device=self._planner.default_joint_state.position.device,
            dtype=self._planner.default_joint_state.position.dtype,
        ).view(1, -1)
        return JointState.from_position(tensor, joint_names=list(self._planner_joint_names))

    def _check_positions_feasible_loaded(
        self, position_rows: Sequence[Sequence[float]]
    ) -> tuple[bool, ...]:
        if self._planner.graph_planner is None:
            raise RuntimeError("curobo_graph_planner_unavailable_for_feasibility_check")
        if not position_rows:
            return ()
        samples = self._torch.as_tensor(
            position_rows,
            device=self._planner.default_joint_state.position.device,
            dtype=self._planner.default_joint_state.position.dtype,
        ).view(-1, len(self._planner_joint_names))
        self._torch.cuda.synchronize()
        mask = self._planner.graph_planner.check_samples_feasibility(samples)
        self._torch.cuda.synchronize()
        return tuple(bool(value) for value in mask.reshape(-1).detach().cpu().tolist())

    def _set_payload_geometry(self, payloads: Sequence[AttachedPayload]) -> None:
        by_side = {payload.side: payload for payload in payloads}
        updates: list[tuple[object, object, object, object]] = []
        for side, link in self._payload_links.items():
            spheres = (
                cuboid_covering_spheres(by_side[side], self._payload_grid_resolution_m)
                if side in by_side
                else None
            )
            for params in self._kinematics_params:
                indices = params.get_sphere_index_from_link_name(link)
                capacity = int(indices.shape[0])
                if spheres is not None and len(spheres) > capacity:
                    raise ValueError(
                        f"payload sphere capacity exceeded:{link} capacity={capacity} needed={len(spheres)}"
                    )
                previous = params.link_spheres[:, indices, :].clone()
                if spheres is None:
                    if params.reference_link_spheres is not None:
                        replacement = params.reference_link_spheres[:, indices, :].clone()
                    else:
                        replacement = self._torch.full_like(previous, -100.0)
                        replacement[:, :, :3] = 0.0
                else:
                    one = self._torch.full(
                        (capacity, 4),
                        -100.0,
                        device=params.link_spheres.device,
                        dtype=params.link_spheres.dtype,
                    )
                    one[:, :3] = 0.0
                    one[: len(spheres)] = self._torch.as_tensor(
                        spheres,
                        device=params.link_spheres.device,
                        dtype=params.link_spheres.dtype,
                    )
                    replacement = one.unsqueeze(0).expand_as(previous).clone()
                updates.append((params, indices, replacement, previous))

        applied: list[tuple[object, object, object, object]] = []
        try:
            for update in updates:
                params, indices, replacement, _ = update
                params.link_spheres[:, indices, :] = replacement
                applied.append(update)
        except Exception:
            for params, indices, _, previous in reversed(applied):
                params.link_spheres[:, indices, :] = previous
            raise
        if self._planner.graph_planner is not None:
            self._planner.graph_planner.reset_buffer()

    def _world_payload_names(self, payloads: Sequence[AttachedPayload]) -> tuple[str, ...]:
        return tuple(
            self._world_payload_object_template.format(
                box_id=payload.box_id,
                side=payload.side,
                id=payload.object_id,
            )
            for payload in payloads
            if self._world_payload_object_template
        )

    def _set_world_payloads_enabled(self, names: Sequence[str], enabled: bool) -> tuple[str, ...]:
        changed: list[str] = []
        for name in names:
            exists = self._planner.scene_collision_checker.check_obstacle_exists(name, env_idx=0)
            if not exists:
                if self._require_world_payload_object:
                    raise RuntimeError(f"payload world obstacle missing:{name}")
                continue
            self._planner.scene_collision_checker.enable_obstacle(
                name, enable=enabled, env_idx=0
            )
            changed.append(name)
        if changed and self._planner.graph_planner is not None:
            self._planner.graph_planner.reset_buffer()
        return tuple(changed)

    @staticmethod
    def _result_success(result) -> bool:
        return bool(
            result is not None
            and result.success is not None
            and bool(result.success.any().item())
        )

    def _trajectory(self, plan) -> BackendTrajectory:
        position = plan.position.detach().cpu()
        while position.ndim > 2:
            position = position[0]
        if position.ndim == 1:
            position = position.view(1, -1)
        source_names = tuple(plan.joint_names or self._planner_joint_names)
        indices = canonical_joint_indices(source_names, self.joint_names)
        if position.shape[-1] != len(source_names):
            raise ValueError(
                "curobo_trajectory_position_joint_size_mismatch:"
                f"{position.shape[-1]}!={len(source_names)}"
            )
        position = position[:, list(indices)]
        rows = tuple(tuple(float(value) for value in row.tolist()) for row in position)
        if len(rows) == 1:
            rows = (rows[0], rows[0])
        dt = self._interpolation_dt
        if plan.dt is not None:
            dt = float(plan.dt.detach().flatten()[0].cpu().item())

        def optional_rows(name: str) -> tuple[tuple[float, ...], ...]:
            value = getattr(plan, name, None)
            if value is None:
                return ()
            tensor = value.detach().cpu()
            while tensor.ndim > 2:
                tensor = tensor[0]
            if tensor.ndim == 1:
                tensor = tensor.view(1, -1)
            if tensor.shape[-1] != len(source_names):
                return ()
            tensor = tensor[:, list(indices)]
            result = tuple(tuple(float(item) for item in row.tolist()) for row in tensor)
            return result if len(result) == len(rows) else ()

        return BackendTrajectory(
            joint_names=tuple(self.joint_names),
            positions=rows,
            times_s=tuple(index * dt for index in range(len(rows))),
            velocities=optional_rows("velocity"),
            accelerations=optional_rows("acceleration"),
        )

    def plan_cspace(
        self,
        start_positions: Sequence[float],
        goal_positions: Sequence[float],
        attached_payloads: Sequence[AttachedPayload],
        *,
        scene_id: str,
        force_graph: bool,
        max_attempts: int,
        timeout_s: float,
    ) -> BackendResult:
        del scene_id  # Strict semantic matching is enforced by JointSegmentPlannerCore.
        started = time.perf_counter()
        self._set_payload_geometry(attached_payloads)
        world_names = self._world_payload_names(attached_payloads)
        disabled = self._set_world_payloads_enabled(world_names, False)
        try:
            start = self._make_joint_state(start_positions)
            goal = self._make_joint_state(goal_positions)
            self._torch.cuda.synchronize()
            solve_started = time.perf_counter()
            endpoint_started = time.perf_counter()
            endpoint_feasible = self._check_positions_feasible_loaded(
                (start_positions, goal_positions)
            )
            endpoint_check_ms = (time.perf_counter() - endpoint_started) * 1000.0
            if not endpoint_feasible[0] or not endpoint_feasible[1]:
                solve_time_ms = (time.perf_counter() - solve_started) * 1000.0
                return BackendResult(
                    False,
                    (
                        "curobo_start_state_in_collision_or_bounds"
                        if not endpoint_feasible[0]
                        else "curobo_goal_state_in_collision_or_bounds"
                    ),
                    solve_time_ms=solve_time_ms,
                    endpoint_check_time_ms=endpoint_check_ms,
                )
            result = None
            used_graph = False
            graph_wall_ms = 0.0
            trajopt_wall_ms = 0.0
            attempts_used = 0
            num_seeds = self._planner.trajopt_solver.config.num_seeds
            for current_attempt in range(max(1, max_attempts)):
                attempts_used = current_attempt + 1
                seed_trajectory = None
                use_graph = force_graph or current_attempt > 0
                if use_graph:
                    graph_started = time.perf_counter()
                    goal_configs = goal.position.view(1, 1, -1).repeat(
                        1, num_seeds, 1
                    )
                    seed_trajectory = self._planner._get_graph_seed_trajectories(
                        start, goal_configs
                    )
                    self._torch.cuda.synchronize()
                    graph_wall_ms += (time.perf_counter() - graph_started) * 1000.0
                    used_graph = True
                    if seed_trajectory is None:
                        continue
                trajopt_started = time.perf_counter()
                result = self._planner.trajopt_solver.solve_cspace(
                    goal,
                    start,
                    seed_traj=seed_trajectory,
                    finetune_attempts=self._trajopt_finetune_attempts,
                    finetune_dt_scale=self._trajopt_finetune_dt_scale,
                )
                self._torch.cuda.synchronize()
                trajopt_wall_ms += (time.perf_counter() - trajopt_started) * 1000.0
                if self._result_success(result):
                    break
            self._torch.cuda.synchronize()
            solve_time_ms = (time.perf_counter() - solve_started) * 1000.0
            total_s = time.perf_counter() - started
            if total_s > timeout_s:
                return BackendResult(
                    False,
                    "curobo_plan_cspace_exceeded_timeout",
                    solve_time_ms=solve_time_ms,
                    endpoint_check_time_ms=endpoint_check_ms,
                    graph_time_ms=graph_wall_ms,
                    trajopt_time_ms=trajopt_wall_ms,
                )
            if not self._result_success(result):
                return BackendResult(
                    False,
                    "curobo_plan_cspace_failed",
                    solve_time_ms=solve_time_ms,
                    endpoint_check_time_ms=endpoint_check_ms,
                    graph_time_ms=graph_wall_ms,
                    trajopt_time_ms=trajopt_wall_ms,
                )
            interpolation_started = time.perf_counter()
            interpolated = result.get_interpolated_plan()
            self._torch.cuda.synchronize()
            if interpolated is None:
                return BackendResult(
                    False,
                    "curobo_returned_no_interpolated_trajectory",
                    solve_time_ms=solve_time_ms,
                    endpoint_check_time_ms=endpoint_check_ms,
                    graph_time_ms=graph_wall_ms,
                    trajopt_time_ms=trajopt_wall_ms,
                )
            trajectory = self._trajectory(interpolated)
            trajectory_feasible = self._check_positions_feasible_loaded(
                trajectory.positions
            )
            first_collision = next(
                (
                    index
                    for index, feasible in enumerate(trajectory_feasible)
                    if not feasible
                ),
                None,
            )
            interpolation_ms = (time.perf_counter() - interpolation_started) * 1000.0
            if first_collision is not None:
                return BackendResult(
                    False,
                    f"curobo_interpolated_trajectory_collision@{first_collision}",
                    solve_time_ms=solve_time_ms,
                    endpoint_check_time_ms=endpoint_check_ms,
                    graph_time_ms=graph_wall_ms,
                    trajopt_time_ms=trajopt_wall_ms,
                    interpolation_time_ms=interpolation_ms,
                )
            return BackendResult(
                True,
                (
                    f"ok; attempts={attempts_used}; graph_ms={graph_wall_ms:.3f}; "
                    f"trajopt_ms={trajopt_wall_ms:.3f}"
                ),
                planner_method=(
                    "curobo_graph_trajopt" if used_graph else "curobo_direct_trajopt"
                ),
                trajectory=trajectory,
                solve_time_ms=solve_time_ms,
                endpoint_check_time_ms=endpoint_check_ms,
                graph_time_ms=graph_wall_ms,
                trajopt_time_ms=trajopt_wall_ms,
                interpolation_time_ms=interpolation_ms,
            )
        finally:
            self._set_world_payloads_enabled(disabled, True)

    def check_positions_feasible(
        self,
        position_rows: Sequence[Sequence[float]],
        attached_payloads: Sequence[AttachedPayload],
    ) -> tuple[bool, ...]:
        """Check joint samples with the planner's graph feasibility rollout."""
        self._set_payload_geometry(attached_payloads)
        world_names = self._world_payload_names(attached_payloads)
        disabled = self._set_world_payloads_enabled(world_names, False)
        try:
            return self._check_positions_feasible_loaded(position_rows)
        finally:
            self._set_world_payloads_enabled(disabled, True)

    def payload_sphere_world_bounds(
        self,
        positions: Sequence[float],
        attached_payloads: Sequence[AttachedPayload],
    ) -> dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]]:
        """Return the world AABB of each active payload sphere cover for diagnostics."""
        self._set_payload_geometry(attached_payloads)
        state = self._planner.compute_kinematics(self._make_joint_state(positions))
        spheres = state.robot_spheres.detach()[0]
        params = self._planner.kinematics.config.kinematics_config
        result = {}
        for side, link in self._payload_links.items():
            indices = params.get_sphere_index_from_link_name(link)
            selected = spheres[0, indices, :]
            selected = selected[selected[:, 3] > 0.0]
            if selected.numel() == 0:
                continue
            lower = (selected[:, :3] - selected[:, 3:4]).min(dim=0).values
            upper = (selected[:, :3] + selected[:, 3:4]).max(dim=0).values
            result[side] = (
                tuple(float(value) for value in lower.cpu().tolist()),
                tuple(float(value) for value in upper.cpu().tolist()),
            )
        return result

    def peak_gpu_memory_mb(self) -> float:
        return float(self._torch.cuda.max_memory_allocated()) / (1024.0 * 1024.0)

    def destroy(self) -> None:
        self._planner.destroy()
