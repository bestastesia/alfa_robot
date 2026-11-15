#!/usr/bin/env python3
"""Shadow-check the latency rewrites of door search helpers against their 405b4d5 versions.

One real five-column sequence runs with each rewritten helper evaluated twice: the 405b4d5
reference (copied verbatim below) and the current implementation. Outputs and recorded side
effects must be identical:
- DoorExtraction.analytic_extract: speculative batched segment validation vs stepwise;
- DoorTask.latest_compact_candidates: score-ordered lazy gating vs gating every row;
- DoorTask.exact_pose_mask: batched FK with a URDF band vs per-row URDF pose_error.

yourdfpy FK depends on the previous configuration: trimesh keeps an edge matrix whose
replacement is within 1e-8 (EnforcedForest.add_edge), so a pose read can lag the exact one
by ~1e-8 per joint. Both extraction versions therefore start from the same yourdfpy graph
state, and the line/orientation metrics, which the current version reads through exact
batched FK, may differ from the reference's per-row reads by at most METRIC_TOLERANCE.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import torch
from curobo.types import JointState
from scipy.spatial.transform import Rotation

from curobo_core.adapter import matrix_pose
from curobo_core.contracts import ACTIVE_JOINTS
from curobo_core.door_extract import DoorExtraction
from curobo_core.door_recenter import DoorPlanner, DoorTask, ROOT
from curobo_core.door_sequential import pose_error
from v3_batched_loaded_search import densify
from v3_door_five_columns import acquire_gpu, load_assets, run_sequence


def reference_analytic_extract(self, contact, empty_validity, loaded_validity, *,
                     sides=("left", "right"), direction=(-1., 0., 0.), distance_m=.35,
                     lift_end=None, hold_sides=(), swivel_continuation=False):
    """Parameterization of f044bf1's fixed-swivel, 1cm Cartesian bridge.

    No reverse construction, per-point angle search, or globally forced
    1mm discretization. The original 35cm/two-arm defaults are unchanged.
    """
    self.last_cartesian_failure = None
    direction = np.asarray(direction, dtype=float)
    moving_sides = tuple(sides)
    sides = moving_sides+tuple(hold_sides)
    if (not moving_sides or len(set(sides)) != len(sides)
            or any(side not in ("left", "right") for side in sides)
            or direction.shape != (3,) or not np.isfinite(direction).all()
            or abs(np.linalg.norm(direction)-1.) > 1e-9
            or not np.isfinite(distance_m) or distance_m <= 0
            or (lift_end is not None and not np.isfinite(lift_end))
            or (swivel_continuation and len(moving_sides) != 1)):
        raise ValueError("invalid Cartesian sides, direction, distance or lift")
    self.fk(contact)
    world_to_carriage = np.linalg.inv(self.fk_robot.get_transform("arm_carriage", self.fk_robot.base_link))
    targets = {side: self.fk_robot.get_transform(side+"_tool0", self.fk_robot.base_link).copy() for side in sides}
    swivels = [self.analytic.swivel(side, contact[1+side*7:8+side*7]) for side in range(2)]
    rows = [np.asarray(contact, dtype=np.float64)]
    self.last_extract_frames=np.asarray(rows)
    continuation_trials = 0
    path_swivels = list(swivels)
    continuation_validity = None
    if swivel_continuation:
        if callable(loaded_validity):
            loaded_validity = loaded_validity()
        continuation_validity = (empty_validity if self.snapshot.policy.defer_payload_until_extract_end
                                 else loaded_validity)
        if callable(continuation_validity):
            continuation_validity = continuation_validity()
    distances = np.linspace(.01, .35, 35) if distance_m == .35 else np.linspace(
        0., distance_m, math.ceil(distance_m/.01)+1)[1:]
    for distance in distances:
        next_row = rows[-1].copy()
        if lift_end is not None:
            next_row[0] = contact[0]+distance/distance_m*(lift_end-contact[0])
            self.fk(next_row)
            world_to_carriage = np.linalg.inv(self.fk_robot.get_transform("arm_carriage", self.fk_robot.base_link))
        for side in sides:
            side_index = ("left", "right").index(side)
            target = targets[side].copy()
            if side in moving_sides:
                target[:3, 3] += direction*distance
            offset = 1+side_index*7
            seed = rows[-1][offset:offset+7]
            swivel = path_swivels[side_index]
            solutions = self.analytic.solve(side_index, world_to_carriage @ target, swivel, seed)
            if len(solutions) == 0 and swivel_continuation and side in moving_sides:
                for delta in (.005, -.005, .01, -.01, .02, -.02, .04, -.04,
                              .08, -.08, .16, -.16, .32, -.32):
                    continuation_trials += 1
                    trial = swivel+delta
                    solutions = self.analytic.solve(side_index, world_to_carriage @ target, trial, seed)
                    if len(solutions):
                        swivel = trial
                        path_swivels[side_index] = trial
                        break
            if len(solutions) == 0:
                self.last_extract_frames=np.asarray(densify(np.asarray(rows)))
                self.last_cartesian_failure = {"joint_positions": rows[-1].tolist(),
                    "requested_tool_matrix": target.tolist(), "distance_m": float(distance), "reason": "fixed_swivel_ik_no_solution"}
                return None, f"{side}固定冗余角解析无解@{distance*100:.1f}cm"
            nearest = np.argmin(np.sum((solutions-seed)**2, axis=1))
            next_row[offset:offset+7] = solutions[nearest]
        if swivel_continuation:
            segment = np.asarray(densify(np.asarray((rows[-1], next_row))))
            if not bool(continuation_validity.mask(torch.tensor(
                    segment, device="cuda", dtype=torch.float32)).all().item()):
                side = moving_sides[0]
                side_index = ("left", "right").index(side)
                offset = 1+side_index*7
                seed = rows[-1][offset:offset+7]
                target = targets[side].copy()
                target[:3, 3] += direction*distance
                alternatives = []
                trial_swivels = []
                base_swivel = path_swivels[side_index]
                for delta in (.005, -.005, .01, -.01, .02, -.02, .04, -.04,
                              .08, -.08, .16, -.16, .32, -.32):
                    continuation_trials += 1
                    trial = base_swivel+delta
                    trial_solutions = self.analytic.solve(
                        side_index, world_to_carriage @ target, trial, seed)
                    if len(trial_solutions):
                        candidate = next_row.copy()
                        nearest = np.argmin(np.sum((trial_solutions-seed)**2, axis=1))
                        candidate[offset:offset+7] = trial_solutions[nearest]
                        alternatives.append(np.asarray(densify(np.asarray((rows[-1], candidate)))))
                        trial_swivels.append((trial, candidate))
                selected = None
                if alternatives:
                    sizes = [len(path) for path in alternatives]
                    good = continuation_validity.mask(torch.tensor(
                        np.concatenate(alternatives), device="cuda", dtype=torch.float32)).cpu().numpy()
                    begin = 0
                    for size, candidate in zip(sizes, trial_swivels):
                        if good[begin:begin+size].all():
                            selected = candidate
                            break
                        begin += size
                if selected is None:
                    self.last_extract_frames=np.asarray(densify(np.asarray(rows)))
                    self.last_cartesian_failure = {"joint_positions": rows[-1].tolist(),
                        "requested_tool_matrix": target.tolist(), "distance_m": float(distance),
                        "reason": "bounded_swivel_continuation_exhausted"}
                    return None, f"{side}局部冗余角续解失败@{distance*100:.1f}cm"
                path_swivels[side_index], next_row = selected
        rows.append(next_row)
    dense = np.asarray(densify(np.asarray(rows)))
    if callable(loaded_validity):
        loaded_validity = loaded_validity()
    extraction_validity = empty_validity if self.snapshot.policy.defer_payload_until_extract_end else loaded_validity
    if callable(extraction_validity):
        extraction_validity = extraction_validity()
    valid = extraction_validity.mask(torch.tensor(dense, device="cuda", dtype=torch.float32))
    if not bool(valid.all().item()):
        index = int((~valid).nonzero()[0].item())
        self.last_extract_frames=dense[:max(1,index)]
        self.last_cartesian_failure = {"joint_positions": dense[index].tolist(), "segment_frame": index, "reason": "path_validation"}
        return None, f"笛卡尔路径碰撞/稳定性失败@frame{index}"
    if not bool(loaded_validity.mask(torch.tensor(dense[-1:], device="cuda", dtype=torch.float32)).item()):
        self.last_extract_frames=dense[:-1]
        return None, "笛卡尔终点携箱校验失败"
    line_error = orientation_error = 0.
    for values in dense:
        self.fk(values)
        for side in sides:
            actual = self.fk_robot.get_transform(side+"_tool0", self.fk_robot.base_link)
            delta = actual[:3, 3]-targets[side][:3, 3]
            deviation = delta-direction*np.dot(delta, direction) if side in moving_sides else delta
            line_error = max(line_error, float(np.linalg.norm(deviation)))
            orientation_error = max(orientation_error, float(Rotation.from_matrix(actual[:3, :3].T @ targets[side][:3, :3]).magnitude()))
    if line_error > .002 or orientation_error > math.radians(1.):
        return None, f"笛卡尔插值超出基线2mm/1deg门限: {line_error*1000:.3f}mm"
    self.last_extract_frames=dense
    return dense, {"strategy": "f044_fixed_swivel", "cartesian_step_m": .01,
                   "fixed_updown_m": float(contact[0]) if lift_end is None else None,
                   "start_updown_m": float(contact[0]), "end_updown_m": lift_end,
                   "fixed_psi_rad": swivels if not swivel_continuation else None,
                   "start_psi_rad": swivels,
                   "end_psi_rad": path_swivels if swivel_continuation else None,
                   "swivel_continuation_trials": continuation_trials,
                   "swivel_continuation_candidate_cap_per_failed_segment": 14,
                   "held_tool_frames": [side+"_tool0" for side in hold_sides],
                   "max_line_error_mm": line_error*1000,
                   "max_orientation_error_deg": math.degrees(orientation_error)}


def reference_latest_compact_candidates(self):
    """Adapt the latest placement atlas to the exact door attachment transforms."""
    atlas_path = ROOT / self.policy['side']['atlas_path']
    atlas = np.load(atlas_path, allow_pickle=False)['joints'].astype(np.float32)
    # ponytail: fixed local cloud around the approved atlas; replace with an offline regenerated
    # attachment-conditioned atlas if this central door scenario becomes a production family.
    rng = np.random.default_rng(20261104)
    noise_scale = np.r_[.015, np.full(14, math.radians(6))]
    rows = np.concatenate([atlas] + [atlas+rng.normal(0., noise_scale, atlas.shape)
                                    for _ in range(32)]).astype(np.float32)
    validity = self.validity()
    limits = validity.checker.kinematics.get_joint_limits().position
    low, high = limits[0].cpu().numpy()+1e-5, limits[1].cpu().numpy()-1e-5
    rows = np.clip(rows, low, high)
    valid = []
    for offset in range(0, len(rows), 4096):
        valid.extend(validity.mask(self.tensor(rows[offset:offset+4096])).cpu().tolist())
    rows = rows[np.asarray(valid)]
    collision_valid = len(rows)
    kept, radii = [], []
    for offset in range(0, len(rows), 1024):
        radius = self.envelope(rows[offset:offset+1024])
        good = radius <= self.radius
        kept.append(rows[offset:offset+1024][good])
        radii.extend(radius[good].tolist())
    rows = np.concatenate(kept) if kept and any(len(item) for item in kept) else np.empty((0, 15))
    if not len(rows):
        return rows, dict(atlas_rows=len(atlas), sampled_rows=33*len(atlas),
                          collision_valid=collision_valid, envelope_valid=0)
    target_poses = {side: matrix_pose(target) for side, target in self.targets.items()}
    scores, pose_errors = [], []
    for offset in range(0, len(rows), 1024):
        values = self.tensor(rows[offset:offset+1024])
        state = validity.checker.kinematics.compute_kinematics(
            JointState.from_position(values[None], joint_names=list(ACTIVE_JOINTS)))
        tools = state.tool_poses.to_dict()
        batch_score = torch.zeros(len(values), device='cuda')
        batch_error = torch.zeros(len(values), device='cuda')
        for side, target in target_poses.items():
            pose = tools[side+'_tool0']
            position = torch.linalg.vector_norm(
                pose.position.reshape(-1, 3)-self.tensor(target.position), dim=-1)
            quaternion = pose.quaternion.reshape(-1, 4)
            cosine = (quaternion*self.tensor(target.quaternion_wxyz)).sum(-1).abs().clamp(0., 1.)
            angle = 2*torch.acos(cosine)
            batch_score += 10*position+angle
            batch_error = torch.maximum(batch_error, position+angle)
        scores.extend(batch_score.cpu().tolist())
        pose_errors.extend(batch_error.cpu().tolist())
    order = np.lexsort((np.linalg.norm(rows-self.q, axis=1), np.asarray(scores)))[:64]
    selected = rows[order]
    details = dict(atlas_path=str(atlas_path), atlas_sha256=hashlib.sha256(atlas_path.read_bytes()).hexdigest(),
                   atlas_rows=len(atlas), sampled_rows=33*len(atlas), collision_valid=collision_valid,
                   envelope_valid=len(rows), selected=len(selected),
                   selected_radius_max_m=float(np.max(np.asarray(radii)[order])),
                   selected_pose_score_max=float(np.max(np.asarray(scores)[order])),
                   selected_pose_error_proxy_max=float(np.max(np.asarray(pose_errors)[order])),
                   policy_radius_m=self.radius, sampling_seed=20261104,
                   noise_updown_std_m=.015, noise_joint_std_deg=6.)
    return selected, details



METRIC_TOLERANCE = {'max_line_error_mm': 1e-4, 'max_orientation_error_deg': 1e-5}


def fk_state(robot):
    forest = robot._scene.graph.transforms
    return {key: dict(value) for key, value in forest.edge_data.items()}, np.array(robot._cfg)


def restore_fk_state(robot, state):
    edges, cfg = state
    forest = robot._scene.graph.transforms
    forest.edge_data.clear()
    forest.edge_data.update({key: dict(value) for key, value in edges.items()})
    forest._hash = None
    robot._cfg[:] = cfg


def same_details(first, second):
    """Exactly equal details, except FK-read metrics within METRIC_TOLERANCE."""
    if not (isinstance(first, dict) and isinstance(second, dict)):
        return same(first, second)
    return (first.keys() == second.keys()
            and all(same(first[k], second[k]) for k in first if k not in METRIC_TOLERANCE)
            and all(abs(first[k]-second[k]) <= limit for k, limit in METRIC_TOLERANCE.items() if k in first))


def same(first, second, metric_tolerance=0.):
    """Exact equality; floats may differ by `metric_tolerance` (relative), arrays never do."""
    if isinstance(first, np.ndarray) or isinstance(second, np.ndarray):
        return (first is not None and second is not None and np.shape(first) == np.shape(second)
                and np.array_equal(np.asarray(first), np.asarray(second)))
    if isinstance(first, dict) and isinstance(second, dict):
        return first.keys() == second.keys() and all(same(first[k], second[k], metric_tolerance) for k in first)
    if isinstance(first, (list, tuple)) and isinstance(second, (list, tuple)):
        return len(first) == len(second) and all(same(a, b, metric_tolerance) for a, b in zip(first, second))
    if isinstance(first, float) and isinstance(second, float):
        return abs(first-second) <= metric_tolerance*max(1., abs(first), abs(second))
    return first == second


def differing_keys(first, second, metric_tolerance=0.):
    if isinstance(first, dict) and isinstance(second, dict):
        return sorted(k for k in set(first) | set(second) if not same(first.get(k), second.get(k), metric_tolerance))
    return None if same(first, second, metric_tolerance) else ['<value>']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets-root', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--seed', type=int, default=11)
    p.add_argument('--wait-for-gpu-seconds', type=float, default=0.)
    a = p.parse_args()
    a.output_dir.mkdir(parents=True, exist_ok=True)
    if any(a.output_dir.iterdir()):
        p.error('output directory must be empty')
    stats = {name: dict(calls=0, mismatches=0, examples=[]) for name in ('analytic_extract', 'atlas', 'exact_pose_mask')}
    current_extract = DoorExtraction.analytic_extract
    current_atlas = DoorTask.latest_compact_candidates
    current_mask = DoorTask.exact_pose_mask

    def record(name, equal, detail):
        stats[name]['calls'] += 1
        if not equal:
            stats[name]['mismatches'] += 1
            if len(stats[name]['examples']) < 5:
                stats[name]['examples'].append(detail)

    def shadow_extract(self, contact, empty_validity, loaded_validity, **options):
        if options.get('step_m', .01) != .01:
            # Finer-step fallback for flush boxes postdates 405b4d5: no reference to compare with.
            stats['analytic_extract']['finer_step_calls'] = stats['analytic_extract'].get('finer_step_calls', 0)+1
            return current_extract(self, contact, empty_validity, loaded_validity, **options)
        options.pop('step_m', None)
        state = fk_state(self.fk_robot)
        reference = reference_analytic_extract(self, contact, empty_validity, loaded_validity, **options)
        reference_effects = (np.asarray(self.last_extract_frames).copy(), json.dumps(self.last_cartesian_failure, sort_keys=True))
        restore_fk_state(self.fk_robot, state)
        result = current_extract(self, contact, empty_validity, loaded_validity, **options)
        effects = (np.asarray(self.last_extract_frames).copy(), json.dumps(self.last_cartesian_failure, sort_keys=True))
        if isinstance(reference[1], dict) and isinstance(result[1], dict):
            deltas = stats['analytic_extract'].setdefault('max_metric_delta', dict.fromkeys(METRIC_TOLERANCE, 0.))
            for key in METRIC_TOLERANCE:
                deltas[key] = max(deltas[key], abs(reference[1][key]-result[1][key]))
        # Every joint row, decision, swivel and count must be identical.
        record('analytic_extract', same(reference[0], result[0]) and same_details(reference[1], result[1]) and
               same(reference_effects[0], effects[0]) and reference_effects[1] == effects[1],
               dict(rows_equal=same(reference[0], result[0]), frames_equal=same(reference_effects[0], effects[0]),
                    failure_equal=reference_effects[1] == effects[1], detail_keys=differing_keys(reference[1], result[1]),
                    reference={k: reference[1].get(k) for k in differing_keys(reference[1], result[1]) or ()} if isinstance(reference[1], dict) else reference[1],
                    current={k: result[1].get(k) for k in differing_keys(reference[1], result[1]) or ()} if isinstance(result[1], dict) else result[1]))
        return result

    def shadow_atlas(self):
        reference = reference_latest_compact_candidates(self)
        result = current_atlas(self)
        keys = ('atlas_rows', 'selected', 'selected_radius_max_m', 'selected_pose_score_max', 'selected_pose_error_proxy_max')
        record('atlas', same(np.asarray(reference[0]), np.asarray(result[0])) and
               all(reference[1].get(k) == result[1].get(k) for k in keys),
               {k: (reference[1].get(k), result[1].get(k)) for k in keys})
        return result

    def shadow_mask(self, rows, side, target, **options):
        result = current_mask(self, rows, side, target, **options)
        tolerance_m, tolerance_rad = options.get('translation_m', .001), options.get('angle_rad', math.radians(.5))
        reference = np.asarray([(lambda error: error[0] <= tolerance_m and error[1] <= tolerance_rad)(
            pose_error(self.tool(row, side), target)) for row in np.asarray(rows, dtype=float).reshape(-1, 15)], dtype=bool)
        record('exact_pose_mask', same(reference, result), dict(rows=len(reference), reference=int(reference.sum()), current=int(result.sum())))
        return result

    DoorExtraction.analytic_extract = shadow_extract
    DoorTask.latest_compact_candidates = shadow_atlas
    DoorTask.exact_pose_mask = shadow_mask
    log = lambda text: print(text, flush=True)
    with open('/tmp/sevenova-curobo-gpu.lock', 'a') as lock:
        acquire_gpu(lock, a.wait_for_gpu_seconds)
        assets = load_assets(a.assets_root)
        policy_path = ROOT/'config/placement_task_policy.json'
        policy = json.loads(policy_path.read_text())
        planner = DoorPlanner(assets)
        began = time.perf_counter()
        aggregate = run_sequence(planner, a.seed, policy, policy_path, assets, a.output_dir/'sequence', progress=log)
    output = dict(success=aggregate['success'] and all(item['calls'] > 0 and item['mismatches'] == 0 for item in stats.values()),
                  sequence_success=aggregate['success'], sequence_ms=(time.perf_counter()-began)*1000,
                  rounds=[{k: item[k] for k in ('column', 'success', 'total_ms', 'error')} for item in aggregate['rounds']],
                  checks=stats)
    (a.output_dir/'search-equivalence.json').write_text(json.dumps(output, ensure_ascii=False, indent=1, default=str)+'\n')
    log(json.dumps(output, ensure_ascii=False, default=str)[:4000])
    return 0 if output['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
