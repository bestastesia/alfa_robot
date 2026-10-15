import time
import torch
from v3_batched_loaded_search import densify


def batched_rrt_connect_multi_goal(start, goals, lower, upper, validity, budget_s, seed,
                                  informed_sampling=False, stop_at_first_valid=False, refinement_s=0.0):
    generator = torch.Generator(device="cuda")
    generator.manual_seed(seed)
    weights = validity.weights
    dof = start.shape[-1]
    roots=start.reshape(-1,dof).clone()
    start_tree = roots.clone()
    start_parents = [-1]*len(roots)
    start_cost = torch.zeros(len(roots), device="cuda")
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
        root_indices=torch.randint(len(roots),(batch,),device='cuda',generator=generator)
        corridor = roots[root_indices] + progress * (goals[goal_indices] - roots[root_indices])
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
        sample_transform = getattr(validity, 'sample_transform', None)
        if sample_transform is not None:
            samples = sample_transform(samples, generator, lower, upper)
        if informed_sampling and best_connections and len(roots)==1:
            from v3_gpu_bitstar import informed_samples

            samples = informed_samples(roots[0], goals, weights, lower, upper, batch,
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
                    dense_tensor=torch.tensor(dense,device='cuda',dtype=torch.float32)
                    quality=getattr(validity,'path_quality_callback',None)
                    if bool(validity.mask(dense_tensor).all().item()) and (quality is None or quality(dense_tensor)):
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
        dense_tensor=torch.tensor(dense_path,device='cuda',dtype=torch.float32)
        quality=getattr(validity,'path_quality_callback',None)
        if bool(validity.mask(dense_tensor).all().item()) and (quality is None or quality(dense_tensor)):
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
