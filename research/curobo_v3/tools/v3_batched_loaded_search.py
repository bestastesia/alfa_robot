#!/usr/bin/env python3

import argparse
import copy
import heapq
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import torch
import yaml
from curobo_core.distance_metric import joint_distance_weights

from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import JointState

from v3_search_helpers import graph_config, validate_payload_path
from v3_wall_ik_benchmark import (  # noqa: E402
    ACTIVE_JOINTS, canonical_side_tool_to_box, chassis_front_x, make_scene, transform,
)


def loaded_robot(robot, box_fit, active_sides=("left", "right"), attachments=None):
    if attachments is not None:
        active_sides = tuple(side for side in active_sides if any(item.parent_link == side + '_tool0' for item in attachments))
    loaded = copy.deepcopy(robot)
    kin = loaded.get("robot_cfg", loaded)["kinematics"]
    urdf_root = ET.parse(kin["urdf_path"]).getroot()
    for side in ("left", "right"):
        if side not in active_sides:
            continue
        tool_joint = next(
            joint for joint in urdf_root.findall("joint")
            if joint.find("child").attrib["link"] == f"{side}_tool0"
        )
        if (tool_joint.attrib["type"] != "fixed"
                or tool_joint.find("parent").attrib["link"] != f"{side}_link7"):
            raise ValueError(f"{side}_tool0 must be fixed to {side}_link7")
        origin = tool_joint.find("origin")
        link_to_tool = transform(
            origin.attrib.get("xyz") if origin is not None else None,
            origin.attrib.get("rpy") if origin is not None else None,
        )
        tool_to_box = canonical_side_tool_to_box(side)
        if attachments is not None:
            from curobo_core.adapter import pose_matrix

            attachment = next(item for item in attachments if item.parent_link == side + "_tool0")
            if not np.allclose(attachment.dimensions_m, box_fit["dimensions_m"], atol=1e-12):
                raise ValueError("payload dimensions do not match sphere model")
            tool_to_box = pose_matrix(attachment.tool_to_object)
        link_to_box = link_to_tool @ tool_to_box
        for center, radius in zip(box_fit["centers"], box_fit["radii"]):
            point = link_to_box @ np.array([*center, 1.0])
            kin["collision_spheres"][f"{side}_link7"].append({
                "center": point[:3].tolist(), "radius": radius,
            })
    return loaded


class GpuValidity:
    def __init__(self, checker, active_sides=(), max_box_tilt_deg=30.0,
                 stability_weight=10.0, check_ground=False, ground_z=0.0,
                 joint_names=None, weights=None):
        self.checker = checker
        self.states_checked = 0
        self.active_sides = tuple(active_sides)
        self.minimum_box_up_z = math.cos(math.radians(max_box_tilt_deg))
        self.stability_weight = stability_weight
        self.check_ground = check_ground
        self.ground_z = ground_z
        self.joint_names = list(joint_names or ACTIVE_JOINTS)
        self.weights = torch.tensor(
            joint_distance_weights(self.joint_names, weights),
            device="cuda", dtype=torch.float32,
        )
        self.ground_exempt_sphere_indices = set()
        if check_ground:
            kinematics_config = checker.kinematics.config.kinematics_config
            self.ground_exempt_sphere_indices = set(
                kinematics_config.get_sphere_index_from_link_name("base_link").cpu().tolist()
            )

    def _stability_cost(self, state, horizon):
        if not self.active_sides:
            return torch.zeros(horizon, device=state.robot_spheres.device)
        tool_poses = state.tool_poses.to_dict()
        up_z_values = []
        for side in self.active_sides:
            quaternion = tool_poses[f"{side}_tool0"].quaternion.reshape(-1, 4)
            local_up = torch.tensor(
                getattr(self.checker, 'task_tool_to_box', {}).get(side, canonical_side_tool_to_box(side))[:3, 2],
                device=quaternion.device, dtype=quaternion.dtype,
            ).expand(len(quaternion), -1)
            vector = quaternion[:, 1:]
            cross = 2.0 * torch.linalg.cross(vector, local_up)
            world_up = local_up + quaternion[:, :1] * cross + torch.linalg.cross(vector, cross)
            up_z_values.append(world_up[:, 2])
        worst_up_z = torch.stack(up_z_values).min(dim=0).values
        return 1.0 - torch.clamp(worst_up_z, -1.0, 1.0)

    def evaluate(self, values, base_margin_m=0.0):
        valid_output = []
        stability_output = []
        for start in range(0, len(values), 4096):
            chunk = values[start:start + 4096]
            horizon = chunk.shape[0]
            q = chunk.unsqueeze(0)
            self.checker.setup_batch_tensors(1, horizon)
            state = self.checker.kinematics.compute_kinematics(
                JointState.from_position(q, joint_names=self.joint_names)
            )
            spheres = state.robot_spheres.view(1, horizon, -1, 4)
            self.checker.collision_constraint.update_num_spheres(
                spheres.shape[2], batch_size=1, horizon=horizon
            )
            collision_cost = (
                self.checker.get_self_collision(spheres).reshape(1, horizon, -1).sum(-1)
                + self.checker.collision_constraint.forward(state).reshape(1, horizon, -1).sum(-1)
                + self.checker.get_bound(q).reshape(1, horizon, -1).sum(-1)
            ).reshape(-1)
            base_mesh=getattr(self.checker,'base_mesh_layer',None)
            if base_mesh is not None:
                base_cost=base_mesh.cost(state,hard=True,margin_m=base_margin_m) if base_margin_m else base_mesh.cost(state,hard=True)
                collision_cost=collision_cost+base_cost.reshape(1,horizon,-1).sum(-1).reshape(-1)
            if self.check_ground:
                ground_indices = [
                    index for index in range(spheres.shape[2])
                    if index not in self.ground_exempt_sphere_indices
                ]
                ground_spheres = spheres[0, :, ground_indices, :]
                ground_collision = (
                    (ground_spheres[..., 3] > 0.0)
                    & (ground_spheres[..., 2] - ground_spheres[..., 3] < self.ground_z)
                ).any(dim=1)
                collision_cost = collision_cost + ground_collision.to(collision_cost.dtype)
            stability_cost = self._stability_cost(state, horizon)
            tilt_valid = stability_cost <= 1.0 - self.minimum_box_up_z
            valid_output.append((collision_cost == 0) & tilt_valid)
            stability_output.append(stability_cost)
            self.states_checked += horizon
        return torch.cat(valid_output), torch.cat(stability_output)

    def mask(self, values):
        return self.evaluate(values)[0]

    def scene_margin_mask(self, values, margin_m):
        if margin_m < 0:
            raise ValueError('scene margin must be nonnegative')
        if margin_m == 0:
            return torch.ones(len(values), device=values.device, dtype=torch.bool)
        output = []
        for offset in range(0, len(values), 4096):
            chunk = values[offset:offset + 4096]
            state = self.checker.kinematics.compute_kinematics(
                JointState.from_position(chunk[None], joint_names=self.joint_names))
            spheres = state.robot_spheres.reshape(1, len(chunk), -1, 4)
            inflated = spheres.clone()
            inflated[..., 3] = torch.where(spheres[..., 3] > 0, spheres[..., 3] + margin_m, spheres[..., 3])
            cost = self.checker.get_collision_constraint(inflated).reshape(len(chunk), -1).sum(-1)
            output.append(cost == 0)
        return torch.cat(output)

    def edges(self, starts, goals, resolution=0.05, return_stability=False):
        resolution = min(resolution, getattr(self, 'search_edge_resolution', resolution))
        collision_weights = getattr(self, 'collision_weights', self.weights).to(starts.device)
        margin_m = getattr(self, 'search_scene_margin_m', 0.)

        def check(edge_starts, edge_goals, step, scene_margin_m):
            weighted = torch.abs((edge_goals - edge_starts) * collision_weights)
            steps = torch.clamp(torch.ceil(weighted.max(dim=1).values / step).long(), min=1)
            horizon = int(steps.max().item()) + 1
            fractions = torch.arange(horizon, device=starts.device).reshape(1, -1)
            fractions = torch.minimum(fractions / steps.reshape(-1, 1), torch.ones_like(fractions))
            points = edge_starts[:, None, :] + fractions[:, :, None] * (edge_goals - edge_starts)[:, None, :]
            base_margin_m = getattr(self, 'search_base_margin_m', 0.0)
            if base_margin_m:
                valid, stability = self.evaluate(points.reshape(-1, starts.shape[1]), base_margin_m=base_margin_m)
            else:
                valid, stability = self.evaluate(points.reshape(-1, starts.shape[1]))
            if scene_margin_m > 0:
                valid = valid & self.scene_margin_mask(points.reshape(-1, starts.shape[1]), scene_margin_m)
            valid = valid.reshape(len(edge_starts), horizon)
            stability = stability.reshape(len(edge_starts), horizon)
            actual = torch.arange(horizon, device=starts.device).reshape(1, -1) <= steps.reshape(-1, 1)
            return valid.all(dim=1), (stability * actual).sum(dim=1) / (steps + 1)

        edge_valid, mean_stability = check(starts, goals, resolution, margin_m)
        fine_resolution = math.radians(.5)
        if margin_m > 0 and resolution > fine_resolution and edge_valid.any():
            accepted = torch.nonzero(edge_valid, as_tuple=False).reshape(-1)
            fine_valid, _ = check(starts[accepted], goals[accepted], fine_resolution, margin_m)
            edge_valid[accepted] = fine_valid
        return (edge_valid, mean_stability) if return_stability else edge_valid


def densify(path, weights=None, maximum_weighted_step=math.radians(0.5)):
    weights = np.asarray(weights or ([5.0] + [1.0] * (len(path[0]) - 1)))
    if len(path[0]) in (15, 18):
        updown_index = 0 if len(path[0]) == 15 else 3
        weights = weights.copy()
        weights[updown_index] = max(5.0, weights[updown_index])
    dense = [path[0].tolist()]
    for target in path[1:]:
        start = np.asarray(dense[-1])
        target = np.asarray(target)
        count = max(1, int(math.ceil(
            np.max(np.abs((target - start) * weights)) / maximum_weighted_step
        )))
        dense.extend(((1.0 - alpha) * start + alpha * target).tolist()
                     for alpha in np.linspace(1.0 / count, 1.0, count))
    return dense


def batched_rrt(start, goal, lower, upper, validity, budget_s, seed):
    generator = torch.Generator(device="cuda")
    generator.manual_seed(seed)
    weights = validity.weights
    dof = start.shape[-1]
    tree = start.reshape(1, -1).clone()
    parents = [-1]
    started = time.perf_counter()
    iterations = 0
    accepted_total = 0
    while time.perf_counter() - started < budget_s and len(tree) < 30000:
        batch = 1024
        uniform = lower + torch.rand((batch, dof), device="cuda", generator=generator) * (upper - lower)
        progress = torch.rand((batch, 1), device="cuda", generator=generator)
        corridor = start + progress * (goal - start)
        noise = torch.randn((batch, dof), device="cuda", generator=generator)
        noise *= 0.45
        for joint, scale in (("base_x", 0.12), ("base_y", 0.12),
                             ("base_yaw", 0.25), ("updown", 0.08)):
            if joint in validity.joint_names:
                noise[:, validity.joint_names.index(joint)] *= scale / 0.45
        samples = torch.where(
            (torch.rand((batch, 1), device="cuda", generator=generator) < 0.65),
            corridor + noise, uniform,
        )
        samples = torch.clamp(samples, lower, upper)
        nearest_chunks = []
        for offset in range(0, len(tree), 4096):
            nodes = tree[offset:offset + 4096]
            distance = torch.sum(((samples[:, None, :] - nodes[None, :, :]) * weights) ** 2, dim=2)
            values, indices = distance.min(dim=1)
            nearest_chunks.append((values, indices + offset))
        nearest_distance = torch.stack([item[0] for item in nearest_chunks])
        selected_chunk = nearest_distance.argmin(dim=0)
        nearest_indices = torch.stack([item[1] for item in nearest_chunks])[selected_chunk, torch.arange(batch, device="cuda")]
        nearest = tree[nearest_indices]
        delta = samples - nearest
        scale = torch.clamp(0.28 / torch.max(torch.abs(delta * weights), dim=1).values, max=1.0)
        steered = nearest + delta * scale[:, None]
        edge_valid = validity.edges(nearest, steered)
        if edge_valid.any():
            accepted = steered[edge_valid]
            accepted_parents = nearest_indices[edge_valid].detach().cpu().tolist()
            first_new = len(tree)
            tree = torch.cat((tree, accepted), dim=0)
            parents.extend(accepted_parents)
            accepted_total += len(accepted)
            goal_distance = torch.max(torch.abs((accepted - goal) * weights), dim=1).values
            candidate_local = torch.argsort(goal_distance)[:min(64, len(accepted))]
            goal_valid = validity.edges(accepted[candidate_local], goal.repeat(len(candidate_local), 1))
            if goal_valid.any():
                winner = int(candidate_local[torch.nonzero(goal_valid, as_tuple=False)[0, 0]].item())
                parent = first_new + winner
                tree = torch.cat((tree, goal.reshape(1, -1)), dim=0)
                parents.append(parent)
                path_indices = []
                current = len(tree) - 1
                while current >= 0:
                    path_indices.append(current)
                    current = parents[current]
                path_indices.reverse()
                return tree[path_indices].detach().cpu().numpy(), {
                    "iterations": iterations + 1,
                    "tree_nodes": len(tree),
                    "accepted_nodes": accepted_total,
                }
        iterations += 1
    return None, {"iterations": iterations, "tree_nodes": len(tree), "accepted_nodes": accepted_total}


def batched_rrt_multi_goal(start, goals, lower, upper, validity, budget_s, seed,
                           apply_shortcut=True, informed_sampling=False):
    generator = torch.Generator(device="cuda")
    generator.manual_seed(seed)
    weights = validity.weights
    dof = start.shape[-1]
    tree = start.reshape(1, -1).clone()
    parents = [-1]
    tree_cost = torch.zeros(1, device="cuda")
    search_edge_resolution = 0.05
    started = time.perf_counter()
    best_connections = []
    first_solution_ms = None
    solutions_found = 0
    direct_starts = start.reshape(1, -1).repeat(len(goals), 1)
    direct_valid, direct_stability = validity.edges(
        direct_starts, goals, resolution=search_edge_resolution,
        return_stability=True,
    )
    if direct_valid.any():
        direct_indices = torch.nonzero(direct_valid, as_tuple=False).reshape(-1)
        direct_length = torch.linalg.vector_norm(
            (goals[direct_indices] - start) * weights, dim=1
        )
        direct_cost = direct_length * (
            1.0 + validity.stability_weight * direct_stability[direct_indices]
        )
        for index, cost in zip(direct_indices.detach().cpu().tolist(),
                               direct_cost.detach().cpu().tolist()):
            best_connections.append({"parent": 0, "goal": index, "cost": cost})
        first_solution_ms = (time.perf_counter() - started) * 1000.0
        solutions_found += int(direct_valid.sum().item())
    iterations = 0
    accepted_total = 0
    while time.perf_counter() - started < budget_s and len(tree) < 60000:
        batch = 2048
        selected_goals = goals[torch.randint(
            len(goals), (batch,), device="cuda", generator=generator
        )]
        uniform = lower + torch.rand((batch, dof), device="cuda", generator=generator) * (upper - lower)
        progress = torch.rand((batch, 1), device="cuda", generator=generator)
        corridor = start + progress * (selected_goals - start)
        noise = torch.randn((batch, dof), device="cuda", generator=generator) * 0.45
        for joint, scale in (("base_x", 0.12), ("base_y", 0.12),
                             ("base_yaw", 0.25), ("updown", 0.08)):
            if joint in validity.joint_names:
                noise[:, validity.joint_names.index(joint)] *= scale / 0.45
        guided = corridor + noise
        if informed_sampling and best_connections:
            best = best_connections[0]
            informed_goal = goals[best["goal"]]
            weighted_start = start * weights
            weighted_goal = informed_goal * weights
            center = 0.5 * (weighted_start + weighted_goal)
            direction = weighted_goal - weighted_start
            minimum_cost = torch.linalg.vector_norm(direction)
            maximum_cost = max(best["cost"], float(minimum_cost.item()) + 1e-5)
            axes = torch.full(
                (dof,),
                0.5 * math.sqrt(max(0.0, maximum_cost ** 2 - float(minimum_cost.item()) ** 2)),
                device="cuda",
            )
            axes[0] = 0.5 * maximum_cost
            unit = torch.randn((batch, dof), device="cuda", generator=generator)
            unit /= torch.linalg.vector_norm(unit, dim=1, keepdim=True)
            radius = torch.rand((batch, 1), device="cuda", generator=generator) ** (1.0 / dof)
            unit *= radius
            basis = torch.zeros(dof, device="cuda")
            basis[0] = 1.0
            target_axis = direction / minimum_cost
            householder_vector = basis - target_axis
            if torch.linalg.vector_norm(householder_vector) > 1e-6:
                householder_vector /= torch.linalg.vector_norm(householder_vector)
                rotation = torch.eye(dof, device="cuda") - 2.0 * torch.outer(
                    householder_vector, householder_vector
                )
            else:
                rotation = torch.eye(dof, device="cuda")
            guided = (unit * axes) @ rotation.T + center
            guided = guided / weights
        samples = torch.where(
            (torch.rand((batch, 1), device="cuda", generator=generator) < 0.7),
            guided, uniform,
        )
        samples = torch.clamp(samples, lower, upper)
        nearest_chunks = []
        for offset in range(0, len(tree), 4096):
            nodes = tree[offset:offset + 4096]
            distance = torch.sum(((samples[:, None, :] - nodes[None, :, :]) * weights) ** 2, dim=2)
            values, indices = distance.min(dim=1)
            nearest_chunks.append((values, indices + offset))
        nearest_distance = torch.stack([item[0] for item in nearest_chunks])
        selected_chunk = nearest_distance.argmin(dim=0)
        nearest_indices = torch.stack([item[1] for item in nearest_chunks])[
            selected_chunk, torch.arange(batch, device="cuda")
        ]
        nearest = tree[nearest_indices]
        delta = samples - nearest
        scale = torch.clamp(
            0.28 / torch.max(torch.abs(delta * weights), dim=1).values, max=1.0
        )
        steered = nearest + delta * scale[:, None]
        edge_valid, edge_stability = validity.edges(
            nearest, steered, resolution=search_edge_resolution,
            return_stability=True,
        )
        if edge_valid.any():
            accepted = steered[edge_valid]
            accepted_parents = nearest_indices[edge_valid].detach().cpu().tolist()
            first_new = len(tree)
            tree = torch.cat((tree, accepted), dim=0)
            parents.extend(accepted_parents)
            accepted_length = torch.linalg.vector_norm(
                (accepted - nearest[edge_valid]) * weights, dim=1
            )
            accepted_cost = tree_cost[nearest_indices[edge_valid]] + accepted_length * (
                1.0 + validity.stability_weight * edge_stability[edge_valid]
            )
            tree_cost = torch.cat((tree_cost, accepted_cost), dim=0)
            accepted_total += len(accepted)
            goal_distance = torch.max(
                torch.abs((accepted[:, None, :] - goals[None, :, :]) * weights), dim=2
            ).values
            pair_count = min(256, goal_distance.numel())
            flat_pairs = torch.argsort(goal_distance.reshape(-1))[:pair_count]
            node_indices = torch.div(flat_pairs, len(goals), rounding_mode="floor")
            goal_indices = flat_pairs % len(goals)
            goal_valid, goal_stability = validity.edges(
                accepted[node_indices], goals[goal_indices],
                resolution=search_edge_resolution, return_stability=True,
            )
            if goal_valid.any():
                valid_pairs = torch.nonzero(goal_valid, as_tuple=False).reshape(-1)
                valid_nodes = node_indices[valid_pairs]
                valid_goals = goal_indices[valid_pairs]
                connection_length = torch.linalg.vector_norm(
                    (goals[valid_goals] - accepted[valid_nodes]) * weights, dim=1
                )
                connection_cost = connection_length * (
                    1.0 + validity.stability_weight * goal_stability[valid_pairs]
                )
                total_cost = accepted_cost[valid_nodes] + connection_cost
                keep_count = min(16, len(total_cost))
                keep_pairs = torch.argsort(total_cost)[:keep_count]
                for keep_pair in keep_pairs.detach().cpu().tolist():
                    best_connections.append({
                        "parent": first_new + int(valid_nodes[keep_pair].item()),
                        "goal": int(valid_goals[keep_pair].item()),
                        "cost": float(total_cost[keep_pair].item()),
                    })
                best_connections = sorted(
                    best_connections, key=lambda item: item["cost"]
                )[:64]
                solutions_found += len(valid_pairs)
                if first_solution_ms is None:
                    first_solution_ms = (time.perf_counter() - started) * 1000.0
        iterations += 1
    search_ms = (time.perf_counter() - started) * 1000.0
    if not best_connections:
        return None, {
            "iterations": iterations, "tree_nodes": len(tree),
            "accepted_nodes": accepted_total, "goal_count": len(goals),
            "selected_goal": None, "first_solution_ms": None,
            "search_ms": search_ms, "solutions_found": 0,
        }
    shortcut_started = time.perf_counter()
    selected_connection = None
    selected_raw_path = None
    shortcut_path = None
    fine_seed_valid = False
    validated_candidates = 0
    for connection in best_connections:
        path_indices = []
        current = connection["parent"]
        while current >= 0:
            path_indices.append(current)
            current = parents[current]
        path_indices.reverse()
        raw_path = torch.cat((
            tree[path_indices], goals[connection["goal"]].reshape(1, -1)
        ), dim=0)
        shortcut = [raw_path[0]]
        source_index = 0
        valid_candidate = True
        while source_index < len(raw_path) - 1:
            if apply_shortcut:
                candidates = raw_path[source_index + 1:]
            else:
                candidates = raw_path[source_index + 1:source_index + 2]
            starts = raw_path[source_index].reshape(1, -1).repeat(len(candidates), 1)
            connectable = validity.edges(
                starts, candidates, resolution=math.radians(0.5)
            )
            valid_indices = torch.nonzero(connectable, as_tuple=False).reshape(-1)
            if not len(valid_indices):
                valid_candidate = False
                break
            source_index += int(valid_indices[-1].item()) + 1
            shortcut.append(raw_path[source_index])
        validated_candidates += 1
        if not valid_candidate:
            continue
        candidate_path = torch.stack(shortcut)
        dense_candidate = densify(
            candidate_path.detach().cpu().numpy(), validity.weights.detach().cpu().tolist()
        )
        if not bool(validity.mask(torch.tensor(
            dense_candidate, device="cuda", dtype=torch.float32
        )).all().item()):
            continue
        selected_connection = connection
        selected_raw_path = raw_path
        shortcut_path = candidate_path
        fine_seed_valid = True
        break
    if selected_connection is None:
        if not apply_shortcut and best_connections:
            selected_connection = best_connections[0]
            path_indices = []
            current = selected_connection["parent"]
            while current >= 0:
                path_indices.append(current)
                current = parents[current]
            path_indices.reverse()
            selected_raw_path = torch.cat((
                tree[path_indices], goals[selected_connection["goal"]].reshape(1, -1)
            ), dim=0)
            shortcut_path = selected_raw_path
        else:
            return None, {
                "iterations": iterations, "tree_nodes": len(tree),
                "accepted_nodes": accepted_total, "goal_count": len(goals),
                "selected_goal": None, "first_solution_ms": first_solution_ms,
                "search_ms": search_ms, "solutions_found": solutions_found,
                "fine_candidates_checked": validated_candidates,
            }
    return shortcut_path.detach().cpu().numpy(), {
        "iterations": iterations,
        "tree_nodes": len(tree),
        "accepted_nodes": accepted_total,
        "goal_count": len(goals),
        "selected_goal": selected_connection["goal"],
        "first_solution_ms": first_solution_ms,
        "search_ms": search_ms,
        "solutions_found": solutions_found,
        "best_weighted_length": selected_connection["cost"],
        "raw_waypoints": len(selected_raw_path),
        "shortcut_waypoints": len(shortcut_path),
        "shortcut_applied": apply_shortcut,
        "shortcut_ms": (time.perf_counter() - shortcut_started) * 1000.0,
        "fine_candidates_checked": validated_candidates,
        "fine_seed_valid": fine_seed_valid,
        "informed_sampling": informed_sampling,
    }


def batched_rrt_connect_multi_goal(start, goals, lower, upper, validity, budget_s, seed,
                                  informed_sampling=False, stop_at_first_valid=False, refinement_s=0.0):
    generator = torch.Generator(device="cuda")
    generator.manual_seed(seed)
    weights = validity.weights
    dof = start.shape[-1]
    start_tree = start.reshape(1, -1).clone()
    start_parents = [-1]
    start_cost = torch.zeros(1, device="cuda")
    goal_tree = goals.clone()
    goal_parents = [-1] * len(goals)
    goal_cost = torch.zeros(len(goals), device="cuda")
    goal_roots = list(range(len(goals)))
    best_connections = []
    first_solution_ms = None
    solutions_found = 0
    accepted_total = 0
    started = time.perf_counter()
    first_verified_ms=None
    best_valid_path=None
    best_valid_connection=None
    rotation_cost=getattr(validity,'box_rotation_cost',None)

    def nearest(tree, samples):
        best_values, best_indices = [], []
        for offset in range(0, len(tree), 4096):
            nodes = tree[offset:offset + 4096]
            distance = torch.sum(
                ((samples[:, None, :] - nodes[None, :, :]) * weights) ** 2, dim=2
            )
            values, indices = distance.min(dim=1)
            best_values.append(values)
            best_indices.append(indices + offset)
        values = torch.stack(best_values)
        selected = values.argmin(dim=0)
        return torch.stack(best_indices)[selected, torch.arange(len(samples), device="cuda")]

    iterations = 0
    fine_checked = set()
    fine_rejections = 0
    while time.perf_counter() - started < budget_s:
        if first_verified_ms is not None and (time.perf_counter()-started)*1000 >= first_verified_ms+refinement_s*1000:
            break
        batch = 1024
        goal_indices = torch.randint(len(goals), (batch,), device="cuda", generator=generator)
        progress = torch.rand((batch, 1), device="cuda", generator=generator)
        corridor = start + progress * (goals[goal_indices] - start)
        noise = torch.randn((batch, dof), device="cuda", generator=generator) * 0.45
        for joint, scale in (("base_x", 0.12), ("base_y", 0.12),
                             ("base_yaw", 0.25), ("updown", 0.08)):
            if joint in validity.joint_names:
                noise[:, validity.joint_names.index(joint)] *= scale / 0.45
        uniform = lower + torch.rand((batch, dof), device="cuda", generator=generator) * (upper - lower)
        samples = torch.where(
            torch.rand((batch, 1), device="cuda", generator=generator) < 0.7,
            corridor + noise, uniform,
        )
        samples = torch.clamp(samples, lower, upper)
        if informed_sampling and best_connections:
            from v3_gpu_bitstar import informed_samples

            samples = informed_samples(start, goals, weights, lower, upper, batch,
                                       best_connections[0]["cost"], generator)
        expand_start = iterations % 2 == 0
        tree = start_tree if expand_start else goal_tree
        tree_parents = start_parents if expand_start else goal_parents
        tree_cost = start_cost if expand_start else goal_cost
        nearest_indices = nearest(tree, samples)
        nearest_values = tree[nearest_indices]
        delta = samples - nearest_values
        scale = torch.clamp(
            0.28 / torch.max(torch.abs(delta * weights), dim=1).values, max=1.0
        )
        steered = nearest_values + delta * scale[:, None]
        edge_valid, edge_stability = validity.edges(
            nearest_values, steered, return_stability=True
        )
        if not edge_valid.any():
            iterations += 1
            continue
        accepted = steered[edge_valid]
        parent_indices = nearest_indices[edge_valid]
        accepted_length = torch.linalg.vector_norm(
            (accepted - nearest_values[edge_valid]) * weights, dim=1
        )
        accepted_cost = tree_cost[parent_indices] + accepted_length * (
            1.0 + validity.stability_weight * edge_stability[edge_valid]
        )
        if rotation_cost is not None:
            edge_start=nearest_values[edge_valid] if expand_start else accepted
            edge_end=accepted if expand_start else nearest_values[edge_valid]
            accepted_cost=accepted_cost+rotation_cost.edge_cost(edge_start,edge_end)
        first_new = len(tree)
        tree = torch.cat((tree, accepted), dim=0)
        tree_parents.extend(parent_indices.detach().cpu().tolist())
        tree_cost = torch.cat((tree_cost, accepted_cost), dim=0)
        accepted_total += len(accepted)
        if expand_start:
            start_tree, start_cost = tree, tree_cost
        else:
            inherited_roots = [goal_roots[index] for index in parent_indices.detach().cpu().tolist()]
            goal_roots.extend(inherited_roots)
            goal_tree, goal_cost = tree, tree_cost
        other_tree = goal_tree if expand_start else start_tree
        other_indices = nearest(other_tree, accepted)
        other_values = other_tree[other_indices]
        pair_distance = torch.linalg.vector_norm((accepted - other_values) * weights, dim=1)
        candidate_indices = torch.argsort(pair_distance)[:min(256, len(accepted))]
        connect_valid, connect_stability = validity.edges(
            accepted[candidate_indices], other_values[candidate_indices], return_stability=True
        )
        if connect_valid.any():
            valid_local = torch.nonzero(connect_valid, as_tuple=False).reshape(-1)
            accepted_indices = candidate_indices[valid_local]
            opposite_indices = other_indices[candidate_indices][valid_local]
            connect_length = pair_distance[candidate_indices][valid_local]
            connect_cost = connect_length * (
                1.0 + validity.stability_weight * connect_stability[valid_local]
            )
            if rotation_cost is not None:
                edge_start=accepted[accepted_indices] if expand_start else other_values[candidate_indices][valid_local]
                edge_end=other_values[candidate_indices][valid_local] if expand_start else accepted[accepted_indices]
                connect_cost=connect_cost+rotation_cost.edge_cost(edge_start,edge_end)
            if expand_start:
                total_cost = accepted_cost[accepted_indices] + goal_cost[opposite_indices] + connect_cost
            else:
                total_cost = accepted_cost[accepted_indices] + start_cost[opposite_indices] + connect_cost
            keep = torch.argsort(total_cost)[:min(16, len(total_cost))]
            for item in keep.detach().cpu().tolist():
                if expand_start:
                    start_node = first_new + int(accepted_indices[item].item())
                    goal_node = int(opposite_indices[item].item())
                else:
                    start_node = int(opposite_indices[item].item())
                    goal_node = first_new + int(accepted_indices[item].item())
                best_connections.append({
                    "start_node": start_node,
                    "goal_node": goal_node,
                    "goal": goal_roots[goal_node],
                    "cost": float(total_cost[item].item()),
                })
            best_connections = sorted(best_connections, key=lambda item: item["cost"])[:64]
            solutions_found += len(valid_local)
            if first_solution_ms is None:
                first_solution_ms = (time.perf_counter() - started) * 1000.0
        iterations += 1
        observer = getattr(validity, 'search_observer', None)
        if (observer is not None or stop_at_first_valid) and best_connections:
            for connection in best_connections:
                left_indices = []
                node = connection['start_node']
                while node >= 0:
                    left_indices.append(node)
                    node = start_parents[node]
                right_indices = []
                node = connection['goal_node']
                while node >= 0:
                    right_indices.append(node)
                    node = goal_parents[node]
                candidate_path = torch.cat((start_tree[left_indices[::-1]], goal_tree[right_indices]), dim=0)
                if stop_at_first_valid:
                    identity = (connection['start_node'], connection['goal_node'])
                    if identity in fine_checked:
                        continue
                    fine_checked.add(identity)
                    dense = densify(candidate_path.detach().cpu().numpy(), getattr(validity, 'collision_weights', weights).detach().cpu().tolist())
                    dense_tensor = torch.tensor(dense, device='cuda', dtype=torch.float32)
                    quality = getattr(validity, 'path_quality_callback', None)
                    if (bool(validity.mask(dense_tensor).all().item()) and
                            (quality is None or quality(dense_tensor))):
                        if first_verified_ms is None:first_verified_ms=(time.perf_counter()-started)*1000
                        if best_valid_connection is None or connection['cost']<best_valid_connection['cost']:
                            best_valid_path=candidate_path.detach().clone();best_valid_connection=dict(connection)
                        if refinement_s>0:
                            break
                        return candidate_path.detach().cpu().numpy(), dict(
                            informed_sampling=informed_sampling, stop_at_first_valid=True,
                            iterations=iterations, start_nodes=len(start_tree), goal_nodes=len(goal_tree),
                            tree_nodes=len(start_tree)+len(goal_tree), accepted_nodes=accepted_total,
                            goal_count=len(goals), selected_goal=connection['goal'],
                            first_solution_ms=first_solution_ms,
                            first_verified_solution_ms=(time.perf_counter()-started)*1000,
                            search_ms=(time.perf_counter()-started)*1000, solutions_found=solutions_found,
                            best_weighted_length=connection['cost'], raw_waypoints=len(candidate_path),
                            shortcut_applied=False, fine_candidates_checked=len(fine_checked),
                            fine_rejections=fine_rejections)
                    fine_rejections += 1
                if observer is not None and observer(candidate_path, connection['cost'], started):
                    break
    selected_path = None
    selected = None
    checked = 0
    if best_valid_path is not None:
        selected_path=best_valid_path;selected=best_valid_connection
    for connection in best_connections:
        if selected is not None and connection['cost']>=selected['cost']:break
        left_indices = []
        current = connection["start_node"]
        while current >= 0:
            left_indices.append(current)
            current = start_parents[current]
        left_indices.reverse()
        right_indices = []
        current = connection["goal_node"]
        while current >= 0:
            right_indices.append(current)
            current = goal_parents[current]
        path = torch.cat((start_tree[left_indices], goal_tree[right_indices]), dim=0)
        dense_path = densify(path.detach().cpu().numpy(), getattr(validity, 'collision_weights', weights).detach().cpu().tolist())
        checked += 1
        dense_tensor = torch.tensor(dense_path, device="cuda", dtype=torch.float32)
        quality = getattr(validity, 'path_quality_callback', None)
        if (bool(validity.mask(dense_tensor).all().item()) and
                (quality is None or quality(dense_tensor))):
            selected_path = path
            selected = connection
            break
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    if selected_path is None:
        return None, {
            "informed_sampling": informed_sampling,
            "iterations": iterations, "start_nodes": len(start_tree),
            "goal_nodes": len(goal_tree), "accepted_nodes": accepted_total,
            "goal_count": len(goals), "selected_goal": None,
            "first_solution_ms": first_solution_ms, "search_ms": elapsed_ms,
            "solutions_found": solutions_found, "fine_candidates_checked": checked,
        }
    return selected_path.detach().cpu().numpy(), {
        "informed_sampling": informed_sampling,
        "iterations": iterations, "start_nodes": len(start_tree),
        "goal_nodes": len(goal_tree), "tree_nodes": len(start_tree) + len(goal_tree),
        "accepted_nodes": accepted_total, "goal_count": len(goals),
        "selected_goal": selected["goal"], "first_solution_ms": first_solution_ms,
        "search_ms": elapsed_ms, "solutions_found": solutions_found,
        "best_weighted_length": selected["cost"],
        "raw_waypoints": len(selected_path), "shortcut_applied": False,
        "fine_candidates_checked": checked + len(fine_checked),
        "first_verified_solution_ms": first_verified_ms,
        "fine_rejections": fine_rejections,
        "total_search_cap_ms": budget_s*1000,
        "refinement_requested_ms":refinement_s*1000,
        "box_rotation_cost_enabled":rotation_cost is not None,
    }


def batched_prm_multi_goal(start, goals, lower, upper, validity, budget_s, seed,
                           max_nodes=6000, neighbors=12):
    generator = torch.Generator(device="cuda")
    generator.manual_seed(seed)
    weights = validity.weights
    dof = start.shape[-1]
    started = time.perf_counter()
    samples = []
    sampled_states = 0
    while sum(len(chunk) for chunk in samples) < max_nodes - len(goals) - 1:
        batch = 4096
        goal_indices = torch.randint(
            len(goals), (batch,), device="cuda", generator=generator
        )
        progress = torch.rand((batch, 1), device="cuda", generator=generator)
        corridor = start + progress * (goals[goal_indices] - start)
        noise = torch.randn((batch, dof), device="cuda", generator=generator) * 0.65
        for joint, scale in (("base_x", 0.18), ("base_y", 0.18),
                             ("base_yaw", 0.35), ("updown", 0.12)):
            if joint in validity.joint_names:
                noise[:, validity.joint_names.index(joint)] *= scale / 0.65
        uniform = lower + torch.rand((batch, dof), device="cuda", generator=generator) * (upper - lower)
        values = torch.where(
            torch.rand((batch, 1), device="cuda", generator=generator) < 0.7,
            corridor + noise, uniform,
        )
        values = torch.clamp(values, lower, upper)
        valid = validity.mask(values)
        samples.append(values[valid])
        sampled_states += batch
        if time.perf_counter() - started >= budget_s * 0.35:
            break
    retained = torch.cat(samples, dim=0)[:max_nodes - len(goals) - 1]
    nodes = torch.cat((start.reshape(1, -1), goals, retained), dim=0)
    terminal_count = len(goals) + 1
    edge_pairs = set()
    for offset in range(0, len(nodes), 256):
        chunk = nodes[offset:offset + 256]
        distance = torch.sum(
            ((chunk[:, None, :] - nodes[None, :, :]) * weights) ** 2, dim=2
        )
        local = torch.arange(len(chunk), device="cuda")
        distance[local, local + offset] = torch.inf
        nearest = torch.topk(distance, k=min(neighbors, len(nodes) - 1), largest=False).indices
        for local_index, row in enumerate(nearest.detach().cpu().tolist()):
            source = offset + local_index
            for target in row:
                edge_pairs.add((min(source, target), max(source, target)))
    pairs = list(edge_pairs)
    adjacency = [[] for _ in range(len(nodes))]
    checked_edges = 0
    valid_edges = 0
    for offset in range(0, len(pairs), 512):
        if time.perf_counter() - started >= budget_s:
            break
        chunk = pairs[offset:offset + 512]
        source_indices = torch.tensor([pair[0] for pair in chunk], device="cuda")
        target_indices = torch.tensor([pair[1] for pair in chunk], device="cuda")
        edge_valid, stability = validity.edges(
            nodes[source_indices], nodes[target_indices], return_stability=True
        )
        checked_edges += len(chunk)
        edge_length = torch.linalg.vector_norm(
            (nodes[target_indices] - nodes[source_indices]) * weights, dim=1
        )
        edge_cost = edge_length * (1.0 + validity.stability_weight * stability)
        for index in torch.nonzero(edge_valid, as_tuple=False).reshape(-1).detach().cpu().tolist():
            source, target = chunk[index]
            cost = float(edge_cost[index].item())
            adjacency[source].append((target, cost))
            adjacency[target].append((source, cost))
            valid_edges += 1
    distances = [math.inf] * len(nodes)
    parents = [-1] * len(nodes)
    distances[0] = 0.0
    queue = [(0.0, 0)]
    selected_goal = None
    while queue:
        cost, node = heapq.heappop(queue)
        if cost != distances[node]:
            continue
        if 1 <= node < terminal_count:
            selected_goal = node - 1
            break
        for neighbor, edge_cost in adjacency[node]:
            candidate = cost + edge_cost
            if candidate < distances[neighbor]:
                distances[neighbor] = candidate
                parents[neighbor] = node
                heapq.heappush(queue, (candidate, neighbor))
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    if selected_goal is None:
        return None, {
            "roadmap_nodes": len(nodes), "sampled_states": sampled_states,
            "candidate_edges": len(pairs), "checked_edges": checked_edges,
            "valid_edges": valid_edges, "selected_goal": None,
            "search_ms": elapsed_ms,
        }
    current = selected_goal + 1
    path_indices = []
    while current >= 0:
        path_indices.append(current)
        current = parents[current]
    path_indices.reverse()
    return nodes[path_indices].detach().cpu().numpy(), {
        "roadmap_nodes": len(nodes), "sampled_states": sampled_states,
        "candidate_edges": len(pairs), "checked_edges": checked_edges,
        "valid_edges": valid_edges, "selected_goal": selected_goal,
        "search_ms": elapsed_ms, "weighted_length": distances[selected_goal + 1],
        "raw_waypoints": len(path_indices), "shortcut_applied": False,
    }


def run_prm(robot, scene, start, goal, budget_s):
    class Args:
        max_nodes = 50000
        steer_buffer_size = 20000
        cspace_resolution = 0.05
        feasibility_buffer_size = 4096
        nodes_per_iteration = 1024
        graph_iterations = 20
        neighbors = 20
        exploration_radius = 1.5
        sampler_buffer_size = 8192
        connect_terminal_nodes_with_nearest = True
        use_default_position_heuristic = True
        seed = 20260930
    cfg = MotionPlannerCfg.create(
        robot=copy.deepcopy(robot), scene_model=scene, collision_cache={"cuboid": 40},
        num_ik_seeds=4, num_trajopt_seeds=2, self_collision_check=True,
        use_cuda_graph=True, optimizer_collision_activation_distance=0.0,
        graph_planner_config=graph_config(Args),
    )
    planner = MotionPlanner(cfg)
    planner.graph_planner.find_path(
        start.reshape(1, -1), goal.reshape(1, -1),
        interpolate_waypoints=False, validate_interpolated_trajectory=False,
    )
    planner.graph_planner.reset_buffer()
    started = time.perf_counter()
    result = planner.graph_planner.find_path(
        start.reshape(1, -1), goal.reshape(1, -1),
        interpolate_waypoints=False, validate_interpolated_trajectory=False,
    )
    torch.cuda.synchronize()
    elapsed = time.perf_counter() - started
    if elapsed > budget_s or not bool(result.success.any().item()):
        return None, {"wall_s": elapsed, "roadmap_nodes": planner.graph_planner.n_nodes}
    path = result.plan_waypoints[0].detach().cpu().numpy()
    return path, {"wall_s": elapsed, "roadmap_nodes": planner.graph_planner.n_nodes}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--planner", choices=["rrt", "prm"], required=True)
    parser.add_argument("--robot-config", type=Path, required=True)
    parser.add_argument("--box-fit", type=Path, required=True)
    parser.add_argument("--extract-result", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--budget", type=float, default=20.0)
    args = parser.parse_args()

    robot = yaml.safe_load(args.robot_config.read_text())
    box_fit = json.loads(args.box_fit.read_text())
    loaded = loaded_robot(robot, box_fit)
    extract = json.loads(args.extract_result.read_text())
    start_np = np.asarray(extract["extract_joints"], dtype=np.float32)
    goal_np = np.zeros(15, dtype=np.float32)
    goal_np[0] = -0.3
    goal_np[1:] = np.deg2rad([
        140, -100, -180, 20, -90, -20, 0,
        -140, 100, 180, -20, 90, 20, 0,
    ])
    front = chassis_front_x(args.urdf, {})
    scene = make_scene(front, 0.9, {20, 24})
    scene.cuboid = [item for item in scene.cuboid if item.name != "ground"]
    checker = RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(
        robot_config=copy.deepcopy(loaded), scene_model=scene,
        n_cuboids=40, n_meshes=0, collision_activation_distance=0.0,
    ))
    validity = GpuValidity(checker)
    start = torch.tensor(start_np, device="cuda")
    goal = torch.tensor(goal_np, device="cuda")
    limits = checker.kinematics.get_joint_limits().position
    lower, upper = limits[0] + 1e-5, limits[1] - 1e-5
    endpoint_valid = validity.mask(torch.stack((start, goal))).detach().cpu().tolist()
    report = {
        "planner": args.planner,
        "budget_s": args.budget,
        "description_commit": "6bb184b",
        "endpoint_valid": endpoint_valid,
        "success": False,
    }
    if not all(endpoint_valid):
        report["failure"] = "invalid_endpoint"
    else:
        started = time.perf_counter()
        if args.planner == "rrt":
            path, stats = batched_rrt(start, goal, lower, upper, validity, args.budget, 20260930)
        else:
            path, stats = run_prm(loaded, scene, start, goal, args.budget)
        report["search_wall_s"] = time.perf_counter() - started
        report["search_stats"] = stats
        if path is None:
            report["failure"] = "no_path_within_budget"
        else:
            dense = densify(path)
            dense_tensor = torch.tensor(dense, device="cuda", dtype=torch.float32)
            sphere_valid = bool(validity.mask(dense_tensor).all().item())
            tool_to_box = {
                "left": canonical_side_tool_to_box("left"),
                "right": canonical_side_tool_to_box("right"),
            }
            exact_valid, exact_reason = validate_payload_path(
                checker.kinematics, dense, tool_to_box, front, 0.9
            )
            report.update({
                "waypoints": len(path), "dense_frames": len(dense),
                "sphere_valid": sphere_valid,
                "exact_payload_valid": exact_valid,
                "exact_payload_reason": exact_reason,
                "success": sphere_valid and exact_valid,
                "frames": dense,
            })
    report["gpu_states_checked"] = validity.states_checked
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    summary = {key: value for key, value in report.items() if key != "frames"}
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
