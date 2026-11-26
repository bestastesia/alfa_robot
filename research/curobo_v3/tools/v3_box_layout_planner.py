"""Joint 15-axis planning for two independently sized/oriented suction boxes."""
from dataclasses import asdict, replace
import hashlib
import itertools
import json
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch
from curobo.types import JointState
from curobo._src.solver.seed_ik.seed_ik_solver import SeedIKSolver
from curobo._src.geom.transform import torch_quaternion_to_matrix
from curobo_core.payload_geometry import box_overlap_depth_tensor, box_overlap_depth, box_sphere_depth

from curobo_core.layout_contract import validate_pair, carry_configuration, carry_reference
from curobo_core.adapter import matrix_pose
from curobo_core.contact_reach import urdf_reach_model, contact_reach_witnesses
from curobo_core.box_pick import actual_owner_ids, payload_fit, paired_carry_candidates, layout_model_key, fixed_extraction_blockages, extraction_endpoint_proposals, source_slot_process_checks, source_slot_observation
from curobo_core.scene import AttachedObject, Pose, SceneObject
from v3_batched_loaded_search import GpuValidity, densify
from v3_box_rotation_cost import BoxRotationCost
from v3_fast_three_layers import FastThreeLayerPlanner, POLICY
from v3_lm_conveyor import SequentialPlacementLm, box_poses, interpolate_boxes
from v3_multiroot_search import batched_rrt_connect_multi_goal
from v3_payload_safety import exact_payload_audit
from v3_two_box_pick import TwoBoxPickPlanner, ROOT, ACTIVE_JOINTS
from v3_layout_native import time_layout_pick


def adaptive_lateral(lm, case, progress=lambda event:None):
    """Keep checked continuation; refine only the next failed Cartesian interval."""
    began = time.perf_counter()
    grid = np.linspace(0., 1., 128)
    rows = np.asarray(case['start'], dtype=np.float32)[None]
    fractions = np.array([0.])
    trials, reports = [], []
    timing = dict(lm_ms=0., candidate_filter_ms=0., edge_audit_ms=0.)
    maximum_position = maximum_angle = 0.
    nodes = 0; last = {}; failure = None
    original_samples = lm.samples
    try:
        while fractions[-1] < 1.-1e-12:
            # ponytail: bounded local refinement, then the caller retries the original512 grid.
            if nodes >= 512 or len(trials) >= 48:
                failure = 'adaptive continuation budget exhausted; not input illegality'; break
            start_f = float(fractions[-1])
            remaining = grid[grid > start_f+1e-12]
            local = np.r_[0., (remaining-start_f)/(1.-start_f)]
            starts = {s:T[0] for s,T in interpolate_boxes(case['starts'], case['ends'], np.array([start_f])).items()}
            lm.samples = len(local)
            last = lm.plan(dict(case, start=rows[-1].copy(), starts=starts,
                reference_fractions=local, edge_resolution=.001))
            nodes += len(last['point_reports'])
            reports.extend(dict(point, global_progress=start_f+(1.-start_f)*point['progress']) for point in last['point_reports'])
            for key in timing:timing[key] += last[key]
            maximum_position = max(maximum_position, last['max_reference_position_error_m'])
            maximum_angle = max(maximum_angle, last['max_reference_orientation_error_deg'])
            partial = np.asarray(last['frames'], dtype=np.float32)
            if np.max(np.abs(partial[0]-rows[-1])) > 1e-5:
                raise RuntimeError('disconnected adaptive lateral continuation')
            rows = np.concatenate((rows, partial[1:]))
            fractions = np.r_[fractions, start_f+(1.-start_f)*np.asarray(last['progress'])[1:]]
            trial = dict(from_fraction=start_f, to_fraction=float(fractions[-1]), reference_points=len(local),
                node_attempts=len(last['point_reports']), success=last['success'], failure=last['failure'],
                total_ms=last['total_ms'], failed_point=last['point_reports'][-1] if not last['success'] and last['point_reports'] else None)
            trials.append(trial)
            progress(dict(state='done' if last['success'] else 'rejected', wall_seconds=last['total_ms']/1000,
                piece=f"adaptive {start_f:.6f}→{fractions[-1]:.6f}", details=trial,
                reason=None if last['success'] else str(last['failure'])+'; refining this candidate, not declaring the layout illegal'))
            if last['success']:break
            if not last['point_reports']:
                failure = last['failure'] or 'adaptive start rejected'; break
            left = float(fractions[-1]); right = float(grid[np.searchsorted(grid, left+1e-12)])
            if right-left < 1e-6:
                failure = 'adaptive interval below numerical target resolution; not input illegality'; break
            grid = np.sort(np.r_[grid, (left+right)/2])
        success = bool(failure is None and last.get('success', False) and abs(fractions[-1]-1.) < 1e-9)
        if not success and failure is None:failure = last.get('failure') or 'incomplete adaptive continuation'
        return dict(last, success=success, frames=rows.tolist(), progress=fractions.tolist(), failure=failure,
            point_reports=reports, completed_progress=float(fractions[-1]), total_ms=(time.perf_counter()-began)*1000,
            **timing, max_reference_position_error_m=maximum_position, max_reference_orientation_error_deg=maximum_angle,
            reference_samples=len(grid), reference_node_attempts=nodes, reference_policy='adaptive128; every edge checked at .001', adaptive_trials=trials)
    finally:lm.samples = original_samples


class BoxLayoutPlanner(TwoBoxPickPlanner):
    full_payload_trajopt = True

    def checker(self, boxes, payload=False, mobile=False):
        checker = super().checker(boxes, payload=payload, mobile=mobile)
        if payload:
            # Payload may rest on the floor; exact full-box ground audits remain mandatory.
            # The robot's 2mm native ground margin must not be applied to carton spheres.
            ids = []
            names = checker.kinematics.config.kinematics_config
            for side in checker.task_tool_to_box:
                original = len(self.robot['kinematics']['collision_spheres'][side+'_link7'])
                indices = names.get_sphere_index_from_link_name(side+'_link7').cpu().tolist()
                if len(indices) <= original:raise RuntimeError('loaded checker has no payload sphere suffix')
                ids.extend(indices[original:])
            checker.payload_ground_exempt_sphere_indices = tuple(ids)
        return checker

    def configure_layout(self, picks, owner_to_physical_id=None):
        if set(picks) != {'left', 'right'}:
            raise ValueError('layout requires exactly one box per arm')
        validate_pair(picks)
        self.picks = picks
        self.static_load_filter_statistics = dict(rows=0, rejected=0)
        self.pick = picks['left']
        self.owner_to_physical_id = actual_owner_ids({} if owner_to_physical_id is None else
            dict(owner_to_physical_id=owner_to_physical_id))
        self.selected_task = {s: int(pid[-2:]) for s, pid in self.owner_to_physical_id.items()}
        self.native_project_margins = True
        self.base_advance = 0.
        self.top_grasp = False  # Each actual attachment supplies its own suction convention.
        self.box_fit = {s: payload_fit(self.frozen_box_fit, p.dimensions_m) for s, p in picks.items()}
        self.snapshot = replace(self.original_snapshot, meshes=(), attachments=(), objects=self.environment + tuple(
            SceneObject(f'wall_box_{i:02d}', picks[s].dimensions_m, Pose(picks[s].center_m))
            for s, i in self.selected_task.items()), revision=self.snapshot.revision+1)
        self.target_poses = {s+'_tool0': dict(position=(pose := matrix_pose(p.carry_tool())).position,
                                             quaternion=pose.quaternion_wxyz) for s, p in picks.items()}

    def contact_poses_for_task(self, boxes):
        return {s+'_tool0': dict(position=(pose := matrix_pose(p.contact_tool())).position,
                                 quaternion=pose.quaternion_wxyz) for s, p in self.picks.items()}

    def task_attachments(self, boxes):
        return tuple(AttachedObject(f'wall_box_{boxes[s]:02d}', p.dimensions_m, s+'_tool0',
            matrix_pose(p.tool_to_box()), (s+'_link7', s+'_tool0')) for s, p in self.picks.items())

    def loaded_audit(self, checker, rows):
        sizes = {s: p.dimensions_m for s, p in self.picks.items()}
        return exact_payload_audit(checker, rows, checker.task_tool_to_box, sizes, self.robot, check_ground=True)

    def source_observations_cpu(self, rows):
        from curobo_core.native_transport import cpu_source_observations
        if not hasattr(self, '_source_urdf_model'):
            import yourdfpy
            self._source_urdf_model = yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
        return cpu_source_observations(self._source_urdf_model, rows, ACTIVE_JOINTS, self.picks)

    def source_slot_depths(self, checker, values):
        """GPU SAT for each carton and its own source volume; search only."""
        depths = []
        for offset in range(0, len(values), 1024):
            rows = values[offset:offset+1024].contiguous()
            state = checker.kinematics.compute_kinematics(JointState.from_position(rows[None], joint_names=ACTIVE_JOINTS))
            tools = state.tool_poses.to_dict()
            batch = []
            for side, pick in self.picks.items():
                if side not in checker.task_tool_to_box:continue
                pose = tools[side+'_tool0']
                rotation = torch_quaternion_to_matrix(pose.quaternion.reshape(-1, 4).double())
                attachment = torch.as_tensor(checker.task_tool_to_box[side], device=values.device, dtype=torch.float64)
                center = pose.position.reshape(-1, 3).double()+(rotation@attachment[:3, 3, None])[..., 0]
                rotation = rotation@attachment[:3, :3]
                half = torch.as_tensor(pick.dimensions_m, device=values.device, dtype=torch.float64)/2
                original = torch.as_tensor(pick.center_m, device=values.device, dtype=torch.float64)[None]
                depth, _, _ = box_overlap_depth_tensor(center, rotation, half, original,
                    torch.eye(3, device=values.device, dtype=torch.float64)[None], half[None])
                batch.append(depth.reshape(-1))
            depths.append(torch.stack(batch, dim=-1))
        return torch.cat(depths)

    def transport_source_slots(self, checker, start):
        depth = self.source_slot_depths(checker, torch.as_tensor(start, device='cuda', dtype=torch.float32)[None])[0]
        active = [(s,p) for s,p in self.picks.items() if s in checker.task_tool_to_box]
        return {side:(pick.center_m, np.asarray(pick.dimensions_m)/2, max(0., float(depth[i]))+1e-6)
            for i,(side,pick) in enumerate(active)}

    def static_load_mask(self, checker, values):
        """Search proposals must support a rest hold; final timed RNEA remains authoritative."""
        key = layout_model_key(self.picks)
        cached_key, engine = getattr(self, 'layout_native', (None, None))
        if cached_key != key or engine is None:
            engine = FastThreeLayerPlanner(planner=self)
            self.layout_native = (key, engine)
        dynamics = engine.dynamics._model(tuple(checker.task_tool_to_box), checker.task_tool_to_box)
        limits = torch.as_tensor(POLICY['arm_continuous_torque_nm']*2,
                                 device=values.device, dtype=values.dtype)*POLICY['planning_limit_scale']
        # ponytail: static-rest-safe proposals; add inertial-compensation search only for a known feasible task that needs it.
        masks = []
        with torch.no_grad():
            for rows in values.detach().split(512):
                rows = rows.contiguous()
                dynamics.setup_batch_size(len(rows))
                zeros = torch.zeros_like(rows)
                torque = dynamics.compute_inverse_dynamics(JointState(position=rows, velocity=zeros, acceleration=zeros))
                masks.append((torque[:, 1:].abs() <= limits).all(-1))
        mask = torch.cat(masks) if masks else torch.empty(0, device=values.device, dtype=torch.bool)
        self.static_load_filter_statistics['rows'] += len(values)
        self.static_load_filter_statistics['rejected'] += int((~mask).sum())
        return mask

    def path_validity(self, checker, strict=None, start=None, goal=None):
        validity = strict if strict is not None else GpuValidity(checker, check_ground=True)
        validity.scene_penetration_tolerance_m = .001
        validity.contact_penetration_tolerance_m = .001
        if getattr(checker, 'guidance_payload', False):
            original = validity.evaluate
            def evaluate(values, base_margin_m=0.):
                mask, stability = original(values, base_margin_m)
                indices = mask.nonzero().reshape(-1)
                if len(indices):
                    mask[indices] &= self.payload_search_mask(checker, values[indices].detach())
                mask &= self.native_margin_mask(checker, values)
                indices = mask.nonzero().reshape(-1)
                if len(indices):
                    mask[indices] &= self.static_load_mask(checker, values[indices])
                return mask, stability
            validity.evaluate = evaluate
            if start is not None:
                validity.box_rotation_cost = BoxRotationCost(checker, ACTIVE_JOINTS,
                    torch.as_tensor(start, device='cuda', dtype=torch.float32), torch.as_tensor(goal, device='cuda', dtype=torch.float32), False,
                    attachments=checker.task_tool_to_box)
                # The constrained seam may admit only sub-degree moves. Keep the
                # coarse branch too; all scales still use the same checked edges.
                validity.steering_steps = (.28, .03, .01, .003)
                initial_depth = self.source_slot_depths(checker, torch.as_tensor(start, device='cuda', dtype=torch.float32)[None])
                limit = initial_depth.clamp(min=0)+1e-6
                physical_evaluate = validity.evaluate
                def process_evaluate(values, base_margin_m=0.):
                    mask, stability = physical_evaluate(values, base_margin_m)
                    mask &= (self.source_slot_depths(checker, values) <= limit).all(-1)
                    return mask, stability
                validity.evaluate = process_evaluate
                def edge_process_quality(points):
                    depths = self.source_slot_depths(checker, points.reshape(-1, points.shape[-1])).reshape(*points.shape[:2], -1)
                    cleared = depths <= 1e-6
                    returned = (depths > 1e-6) & (torch.cumsum(cleared.to(torch.int32), dim=1) > 0)
                    return ~returned.any(dim=2).any(dim=1)
                validity.edge_process_quality = edge_process_quality
        evaluate = validity.evaluate
        validity.search_padding_frames = 0
        def padded_evaluate(values, base_margin_m=0.):
            count = len(values); extra = (-count) % 512
            # Row-independent search queries only: bound Warp's tail-horizon
            # template set. Final native4001-point gates are not padded here.
            batch = values if not extra else torch.cat((values, values[-1:].expand(extra, -1)))
            mask, stability = evaluate(batch, base_margin_m)
            validity.search_padding_frames += extra
            return mask[:count], stability[:count]
        validity.evaluate = padded_evaluate
        validity.validate_returned_path = True
        return validity

    def carry_lift_blocked(self, targets):
        """Reject only this IK proposal's strong fixed-to-lift overlap, not the input layout."""
        spheres=[]
        for link in ('head','head_yaw','arm_carriage'):
            transform=self.fk_robot.get_transform(link,self.fk_robot.base_link)
            local=np.asarray([[*v['center'],v['radius']] for v in self.robot['kinematics']['collision_spheres'][link] if v['radius']>0])
            spheres.append(np.c_[local[:,:3]@transform[:3,:3].T+transform[:3,3],local[:,3]])
        spheres=np.concatenate(spheres)
        for side,pick in self.picks.items():
            box=targets[side]@pick.tool_to_box();half=np.asarray(pick.dimensions_m)/2
            depth=box_sphere_depth(spheres[None,:,:3],spheres[None,:,3],box[None,:3,3],box[None,:3,:3],half)
            # A conservative pose envelope, NOT a relaxed collision acceptance tolerance.
            envelope=.006+2*np.linalg.norm(half)*np.sin(np.deg2rad(1.)/2)+1e-5
            if (depth>envelope+1e-6).any():return True
        return False

    def independent_arm_mask(self, checker, rows, side):
        """Check the arm/own-payload subset independent of its partner before pairing."""
        values = torch.as_tensor(np.round(rows, 7), device='cuda', dtype=torch.float32)
        state = checker.kinematics.compute_kinematics(JointState.from_position(values[None], joint_names=ACTIVE_JOINTS))
        spheres = state.robot_spheres.clone()
        keep = torch.zeros(spheres.shape[-2], device=values.device, dtype=torch.bool)
        names = checker.kinematics.config.kinematics_config
        for link, original in self.robot['kinematics']['collision_spheres'].items():
            if link.startswith(('left_', 'right_')) and not link.startswith(side+'_'):
                continue
            indices = names.get_sphere_index_from_link_name(link)
            if not link.startswith(side+'_'):indices = indices[:len(original)]
            keep[indices] = True
        padding = checker.kinematics.get_self_collision_config().sphere_padding
        spheres[..., ~keep, 3] = -1. - padding.abs().max()
        from copy import copy
        subset = copy(state); subset.robot_spheres = spheres
        valid = checker.get_self_collision(spheres).reshape(len(values), -1).sum(-1) == 0
        valid &= self.native_margin_mask(checker, values, state=subset)
        return valid.cpu().numpy()

    def enumerate_pair(self, targets, reference, checker, contact=False, limit=256, *,
                       lift_values=None, swivel_step=10, arm_limit=32, held=True):
        """Reuse fixed-Updown analytic arms, then pair at the SAME shared lift coordinate."""
        lower, upper = [v.cpu().numpy() for v in checker.kinematics.get_joint_limits().position]
        candidates = []
        pruned_lifts = []
        arm_checked = dict(left=0, right=0); arm_rejected = dict(left=0, right=0)
        weights = np.r_[np.deg2rad(15)/.1, np.ones(14)]
        # ponytail: finite 5cm/10deg lattice; refine this lattice if a certified feasible pose is missed.
        for lift in (np.round(np.arange(-1., .0001, .05), 2) if lift_values is None else lift_values):
            row = np.asarray(reference, dtype=float).copy(); row[0] = lift
            self.fk(row)
            if not contact and held and self.carry_lift_blocked(targets):
                pruned_lifts.append(float(lift));continue
            carriage = np.linalg.inv(self.fk_robot.get_transform('arm_carriage', self.fk_robot.base_link))
            arms = []
            for arm, side in enumerate(('left', 'right')):
                offset = 1+7*arm
                solutions = [self.analytic.solve(arm, carriage@targets[side], np.deg2rad(angle),
                                                 row[offset:offset+7]) for angle in range(-180, 180, swivel_step)]
                values = np.concatenate(solutions)
                values = values[np.all((values >= lower[offset:offset+7]) & (values <= upper[offset:offset+7]), axis=1)]
                if not len(values):break
                values = np.unique(np.round(values, 7), axis=0)
                if not contact and arm_limit is None:
                    probes = np.repeat(row[None], len(values), axis=0)
                    probes[:, offset:offset+7] = values
                    mask = self.independent_arm_mask(checker, probes, side)
                    arm_checked[side] += len(values); arm_rejected[side] += int((~mask).sum())
                    values = values[mask]
                    if not len(values):break
                values = values[np.argsort(np.linalg.norm(values-reference[offset:offset+7], axis=1))]
                # Retain both elbow families rather than spending the pool on near-duplicates.
                families = [values[values[:, 3] < 0], values[values[:, 3] >= 0]]
                ordered = [v for pair in itertools.zip_longest(*families) for v in pair if v is not None]
                arms.append(np.asarray(ordered[:arm_limit]))
            if len(arms) != 2:continue
            left, right = arms
            pairs = np.repeat(row[None], len(left)*len(right), axis=0)
            pairs[:, 1:8] = np.repeat(left, len(right), axis=0)
            pairs[:, 8:] = np.tile(right, (len(left), 1))
            candidates.append(pairs)
        self.last_pair_enumeration = dict(raw_pairs=sum(len(c) for c in candidates), hard_valid=0, exact_valid=0, native_valid=0, envelope_valid=0,
            independent_arm_rows_checked=arm_checked, independent_arm_rows_rejected=arm_rejected,
            pruned_lift_m=pruned_lifts,carry_candidate_pool_policy='strong fixed-to-lift overlap rejected before analytic IK; remaining512-row blocks and elbow-family selection; not a global layout rejection')
        if not candidates:return np.empty((0, 15), dtype=np.float32)
        rows = np.round(np.concatenate(candidates), 7)
        # Arms are already unique at7 decimals. Distinct rounded lifts prove the
        # full Cartesian product unique; retain the original path for duplicates.
        fine_unique = arm_limit is None and len({float(np.round(c[0,0],7)) for c in candidates}) == len(candidates)
        if not fine_unique:rows = np.unique(rows, axis=0)
        if arm_limit is None:self.last_pair_enumeration['fine_pair_uniqueness_proved'] = fine_unique
        rows = rows.astype(np.float32)
        rows = rows[np.argsort(np.linalg.norm((rows-reference)*weights, axis=1))]
        validity = self.target_validity(checker, allow_contact=True) if contact else self.path_validity(checker)
        validity.scene_penetration_tolerance_m = .001
        validity.contact_penetration_tolerance_m = .001
        accepted = []
        for offset in range(0, len(rows), 512):
            batch = torch.as_tensor(rows[offset:offset+512], device='cuda')
            if not contact:
                hard = GpuValidity(checker, check_ground=True, scene_penetration_tolerance_m=.001).mask(batch)
                self.last_pair_enumeration['hard_valid'] += int(hard.sum())
                indices = hard.nonzero().reshape(-1)
                if len(indices):
                    audit = self.loaded_audit(checker, batch[indices])
                    hard[indices[audit['invalid_frame_indices']]] = False
                valid_count = int(hard.sum())
                self.last_pair_enumeration['exact_valid'] += valid_count
                if not valid_count:continue
                hard &= self.native_margin_mask(checker, batch)
                valid_count = int(hard.sum())
                self.last_pair_enumeration['native_valid'] += valid_count
                if not valid_count:continue
                mask = hard & (self.full_envelope_radius(checker, batch) <= self.carry_radius_limit) if held else hard
                self.last_pair_enumeration['envelope_valid'] += int(mask.sum())
            else:
                mask = validity.mask(batch)
            accepted.extend(batch[mask].cpu().numpy())
            if len(accepted) >= limit:break
        if not accepted:return np.empty((0, 15), dtype=np.float32)
        rows = np.asarray(accepted)
        families = [rows[((rows[:, 4] < 0) == a) & ((rows[:, 11] < 0) == b)]
                    for a, b in itertools.product((True, False), repeat=2)]
        return np.asarray([r for group in itertools.zip_longest(*families) for r in group if r is not None][:limit])

    def nominal_extraction(self, lm, case, progress=lambda event:None):
        """Try64 reference IK nodes, retaining original density for difficult proposals."""
        original_samples=lm.samples;began=time.perf_counter()
        try:
            lm.samples=64
            coarse=lm.plan(case)
        finally:lm.samples=original_samples
        detail={k:v for k,v in coarse.items() if k not in ('frames','point_reports','failed_candidates','success')}
        detail['solver_success']=coarse['success']
        reasons=[];audit=None
        if not coarse['success']:reasons.append('coarse LM: '+str(coarse['failure']))
        else:
            rows=np.asarray(coarse['frames'],dtype=np.float32)
            audit=self.loaded_audit(case['checker'],rows)
            native=bool(self.native_margin_mask(case['checker'],torch.as_tensor(rows[-1:],device='cuda')).all())
            observations={side:source_slot_observation(self.picks[side],poses) for side,poses in
                box_poses(case['checker'],rows[-1:],case['checker'].task_tool_to_box).items()}
            # Proposal choice only: leave numerical headroom, or use the ORIGINAL sampler.
            clear=all(v['initial_slot_overlap_m']<=v['observation_epsilon_m']/2 for v in observations.values())
            detail.update(payload_proposal_audit=audit,native_endpoint_margin_valid=native,
                source_endpoint_observations=observations,source_quality_valid=clear)
            if not audit['valid']:reasons.append('full-size payload proposal audit rejected')
            if not native:reasons.append('native transport-start base/ground margin rejected')
            if not clear:reasons.append('coarse endpoint near source-slot numerical boundary')
        accepted=not reasons;detail['proposal_accepted']=accepted
        progress(dict(stage='extract_reference',kind='geometry_substage',state='done' if accepted else 'rejected',
            wall_seconds=time.perf_counter()-began,details=detail,
            reason=None if accepted else '; '.join(reasons)+f'; retry original{original_samples} reference nodes, not input illegality'))
        if accepted:
            coarse['proposal_payload_audit']=audit  # Same fresh rows; the caller reuses this actual audit once.
            coarse['nominal_reference_policy']='64-node proposal; original native4001 acceptance unchanged'
            coarse['proposal_wall_seconds']=time.perf_counter()-began
            return coarse
        fallback=lm.plan(case)
        fallback['coarse_reference_attempt']=detail
        fallback['nominal_reference_policy']=f'64-node proposal rejected; original{original_samples}-node fallback'
        fallback['proposal_wall_seconds']=time.perf_counter()-began
        return fallback

    def reverse_extractions(self, contacts, empty, loaded, lm, audit_log, progress):
        """Bounded recovery after coarse forward failures; all original gates remain."""
        preparation = time.perf_counter()
        original = self.fk_robot.cfg.copy()
        spheres = []; sphere_links = []
        try:
            row = np.asarray(self.home_values).copy(); row[0] = 0.; self.fk(row)
            for link in ('head', 'head_yaw', 'arm_carriage'):
                transform = self.fk_robot.get_transform(link, self.fk_robot.base_link)
                for sphere in self.robot['kinematics']['collision_spheres'][link]:
                    if sphere['radius'] > 0:
                        spheres.append([*(transform[:3, :3]@sphere['center']+transform[:3, 3]), sphere['radius']])
                        sphere_links.append(link)
        finally:
            self.fk_robot.update_cfg(original)
        lower, upper = loaded.kinematics.get_joint_limits().position
        bounds = float(lower[0]), float(upper[0])
        seeds = contacts[:, 0] if len(contacts) else np.linspace(*bounds, 5)
        proposals = extraction_endpoint_proposals(self.picks, spheres, self._contact_reach_model['lift_axis'], bounds, seeds)
        progress(dict(stage='recover_pose_search', kind='geometry', state='done', wall_seconds=time.perf_counter()-preparation, proposals=len(proposals)))
        contact_validity = self.target_validity(empty, allow_contact=True)
        contact_validity.scene_penetration_tolerance_m = .001
        contact_validity.contact_penetration_tolerance_m = .001
        weights = np.r_[np.deg2rad(15)/.1, np.ones(14)]
        ends = {s:np.eye(4) for s in self.picks}
        for side in ends:ends[side][:3, 3] = self.picks[side].center_m
        def proposal_stream():
            yield from proposals  # Original recovered candidates are tried before any new guidance.
            nominal = {side:np.eye(4) for side in self.picks}
            for side, pick in self.picks.items():
                nominal[side][:3, 3] = np.asarray(pick.center_m)-[pick.extraction_distance_m, 0., 0.]
            # ponytail: 5cm/10deg nominal lattice; existing fine/tolerance-guided proposals remain the fallback.
            yield dict(lift_m=float(np.mean(bounds)), boxes={s:T.tolist() for s,T in nominal.items()},
                nominal_lift_grid=True, scope='exact requested extraction endpoint over the full shared-lift grid; proposal only')
            began = time.perf_counter()
            piece = 'original terminal-link geometry fallback'
            progress(dict(stage='recover_pose_search', kind='geometry_substage', piece=piece, state='running'))
            ignore = self.robot['kinematics']['self_collision_ignore']; terminals = {}
            for side, pick in self.picks.items():
                link_to_tool = self.fk_robot.get_transform(side+'_tool0', side+'_link7')
                box_to_link = np.linalg.inv(pick.tool_to_box())@np.linalg.inv(link_to_tool)
                local = np.asarray([[*v['center'],v['radius']] for v in self.robot['kinematics']['collision_spheres'][side+'_link7'] if v['radius']>0])
                terminals[side] = dict(spheres=np.c_[local[:,:3]@box_to_link[:3,:3].T+box_to_link[:3,3],local[:,3]],
                    mask=np.asarray([link not in ignore.get(side+'_link7',[]) and side+'_link7' not in ignore.get(link,[]) for link in sphere_links]))
            extra = extraction_endpoint_proposals(self.picks, spheres, self._contact_reach_model['lift_axis'], bounds, seeds,
                terminal_geometry=terminals)
            progress(dict(stage='recover_pose_search', kind='geometry_substage', piece=piece, state='done',
                wall_seconds=time.perf_counter()-began, proposals=len(extra), reason='additional proposal guidance only; all original IK/native/process gates retained'))
            yield from extra
            began = time.perf_counter(); piece = 'original payload/native-base geometry fallback'
            progress(dict(stage='recover_pose_search', kind='geometry_substage', piece=piece, state='running'))
            boxes = list(loaded.base_cuboid_layer.scene.cuboid)
            base_geometry = dict(centers=[box.pose[:3] for box in boxes],
                rotations=Rotation.from_quat(np.roll([box.pose[3:7] for box in boxes], -1, axis=1)).as_matrix(),
                halves=np.asarray([box.dims for box in boxes])/2,
                spheres={side:np.c_[self.box_fit[side]['centers'], self.box_fit[side]['radii']] for side in self.picks},
                activation_m=float(loaded.base_cuboid_layer.mesh_checker.collision_cost.config.activation_distance.max()))
            extra = extraction_endpoint_proposals(self.picks, spheres, self._contact_reach_model['lift_axis'], bounds, seeds,
                terminal_geometry=terminals, native_base_geometry=base_geometry)
            progress(dict(stage='recover_pose_search', kind='geometry_substage', piece=piece, state='done',
                wall_seconds=time.perf_counter()-began, proposals=len(extra), reason='native-base proposal guidance only; all original IK/native/process gates retained'))
            yield from extra
        for proposal in proposal_stream():
            began = time.perf_counter()
            targets = {s:np.asarray(pose)@np.linalg.inv(self.picks[s].tool_to_box()) for s,pose in proposal['boxes'].items()}
            nominal_grid = proposal.get('nominal_lift_grid', False)
            lifts = (np.linspace(*bounds, 21) if nominal_grid else
                np.clip(proposal['lift_m']+np.array([0., .001, -.001, .002, -.002, .004, -.004, .008, -.008]), *bounds))
            reference = self.home_values if nominal_grid or not len(contacts) else contacts[0]
            goals = self.enumerate_pair(targets, reference, loaded,
                lift_values=np.unique(lifts), swivel_step=10 if nominal_grid else 2,
                arm_limit=32 if nominal_grid else None, held=False)
            detail = dict(proposal=proposal, endpoints=len(goals), enumeration=dict(self.last_pair_enumeration), trials=[])
            audit_log.append(detail)
            progress(dict(stage='recover_endpoint_ik', kind='geometry', state='done' if len(goals) else 'rejected', wall_seconds=time.perf_counter()-began, endpoints=len(goals),
                reason=None if len(goals) else 'no valid endpoint in this finite proposal/IK lattice; not input illegality'))
            selected = []
            if len(goals):selected.append(0)
            while len(selected) < min(8, len(goals)):
                distances = np.min(np.linalg.norm((goals[:, None]-goals[selected][None])*weights, axis=-1), axis=1)
                distances[selected] = -1
                selected.append(int(distances.argmax()))
            for index in selected:
                began = time.perf_counter()
                tools = self._tool_poses(loaded, torch.as_tensor(goals[index], device='cuda'))
                attachments = loaded.task_tool_to_box
                result = lm.plan(dict(start=goals[index], checker=loaded, attachments=attachments,
                    starts={s:tools[s]@a for s,a in attachments.items()}, ends=ends))
                trial = dict(endpoint_index=index, success=result['success'], failure=result['failure'], progress=result['completed_progress'])
                detail['trials'].append(trial)
                if result['success']:
                    frames = np.asarray(result['frames'][::-1], dtype=np.float32)
                    trial['payload_audit'] = self.loaded_audit(loaded, frames)
                    trial['contact_valid'] = bool(contact_validity.mask(torch.as_tensor(frames[:1], device='cuda')).all())
                    trial['cartesian_valid'] = True
                    for side, poses in box_poses(loaded, frames, attachments).items():
                        delta = poses[:, :3, 3]-self.picks[side].center_m
                        axial, lateral = -delta[:, 0], np.linalg.norm(delta[:, 1:], axis=1)
                        angles = np.rad2deg(Rotation.from_matrix(poses[:, :3, :3]).magnitude())
                        pull = np.linalg.norm(poses[-1, :3, 3]-poses[0, :3, 3]+[self.picks[side].extraction_distance_m, 0., 0.])
                        trial['cartesian_valid'] &= bool(lateral.max() <= .002 and angles.max() <= 1. and pull <= .002
                            and axial.min() >= -.002 and axial.max() <= self.picks[side].extraction_distance_m+.002)
                    trial['success'] = bool(trial['payload_audit']['valid'] and trial['contact_valid'] and trial['cartesian_valid'])
                progress(dict(stage='reverse_extract', kind='geometry', state='done' if trial['success'] else 'rejected',
                    wall_seconds=time.perf_counter()-began, reason=trial['failure'] or 'inspect reverse candidate audits'))
                if trial['success']:
                    yield dict(success=True, frames=frames.tolist(), failure=None, method='tolerance-aware endpoint IK and reverse shared-lift LM',
                        endpoint_proposal=proposal, reverse_lm={k:v for k,v in result.items() if k not in ('frames', 'point_reports', 'failed_candidates')})

    def precontact(self, contact, distances, checker):
        self.fk(contact)
        carriage = np.linalg.inv(self.fk_robot.get_transform('arm_carriage', self.fk_robot.base_link))
        poses = {s: self.fk_robot.get_transform(s+'_tool0', self.fk_robot.base_link).copy() for s in self.picks}
        swivels = [self.analytic.swivel(i, contact[1+7*i:8+7*i]) for i in range(2)]
        rows = [contact.copy()]
        for f in np.linspace(0, 1, int(np.ceil(max(distances.values())/.005))+1)[1:]:
            row = rows[-1].copy()
            for arm, side in enumerate(('left', 'right')):
                if distances[side] == 0.:continue  # Keep the actual held-arm q and box C.
                target = poses[side].copy(); target[:3, 3] -= target[:3, 2]*distances[side]*f
                offset = 1+7*arm
                solutions = self.analytic.solve(arm, carriage@target, swivels[arm], row[offset:offset+7])
                if not len(solutions):return None
                row[offset:offset+7] = solutions[np.argmin(np.linalg.norm(solutions-row[offset:offset+7], axis=1))]
            rows.append(row)
        rows = np.asarray(densify(rows))[::-1].copy()
        validity = self.target_validity(checker, allow_contact=True)
        validity.scene_penetration_tolerance_m = .001
        validity.contact_penetration_tolerance_m = .001
        return rows if bool(validity.mask(torch.as_tensor(rows, device='cuda', dtype=torch.float32)).all()) else None

    def forward_lateral_prefix(self, start, checker, progress=lambda event:None, *, adaptive=True, align_moving=False):
        """Generate a fresh shared-lift clearance proposal, never a reverse/root cache."""
        began=time.perf_counter()
        attachments=checker.task_tool_to_box
        starts={side:pose[0] for side,pose in box_poses(checker,np.asarray(start,dtype=np.float32)[None],attachments).items()}
        initial={side:source_slot_observation(pick,starts[side][None]) for side,pick in self.picks.items()}
        moving=[side for side in self.picks if initial[side]['initial_slot_overlap_m']>initial[side]['observation_epsilon_m']]
        report=dict(stages=[],reference_samples=128 if adaptive else 512,adaptive_lateral=adaptive,moving_box_orientation='both_boxes_to_world_axes' if align_moving else 'fixed',initial_source_observation=initial,method='fresh Cartesian lateral shared-lift LM; no stored trajectory')
        if not moving:return None,dict(report,skipped='both actual cartons already clear',wall_seconds=time.perf_counter()-began)
        if adaptive and all(initial[side]['initial_slot_overlap_m'] <= 10*initial[side]['observation_epsilon_m'] for side in moving):
            # Sampler choice only: still check the same original1um process gates.
            adaptive=False
            report.update(reference_samples=512,adaptive_lateral=False,
                adaptive_skip_reason='only numerical-scale source overlap; retain original512 sampler, not a relaxed acceptance gate')
        def failed(reason):
            report.update(failure=reason,wall_seconds=time.perf_counter()-began)
            if not adaptive:return None,report
            progress(dict(stage='lateral_'+side,kind='geometry_substage',state='rejected',piece='adaptive fallback',
                reason='adaptive prefix rejected; retry '+('original512 grid' if align_moving else 'moving-box alignment')+': '+reason,
                wall_seconds=report['wall_seconds']))
            fallback,detail=self.forward_lateral_prefix(start,checker,progress=progress,
                adaptive=not align_moving,align_moving=not align_moving)
            if detail.get('adaptive_attempt') is not None:report['next_attempt']=detail['adaptive_attempt']
            detail['adaptive_attempt']=report
            detail['wall_seconds']=time.perf_counter()-began
            return fallback,detail
        # Leave spline fitting numerical room; original512 fallback retains its bound.
        limits={side:value['initial_slot_overlap_m']+(0.5 if adaptive else 1.)*value['observation_epsilon_m'] for side,value in initial.items()}
        filter_statistics=dict(rows=0,source_rejected=0,native_margin_rejected=0,full_payload_rejected=0,joint_bound_rejected=0,proposal_clearance_rejected=0)
        report['candidate_filter_statistics']=filter_statistics
        lower,upper=checker.kinematics.get_joint_limits().position
        clearance_layer=None
        if adaptive or align_moving:
            self.payload_search_mask(checker,torch.as_tensor(start,device=lower.device,dtype=lower.dtype)[None])
            clearance_layer=checker._payload_search_resource[1]
            report['proposal_box_robot_clearance_m']=5e-6
        def candidate_filter(values,state,actual):
            # Consume this query's FK before payload classification's rare CPU
            # fallback can overwrite the checker buffers. No cross-query cache.
            native=self.native_margin_mask(checker,values,state=state)
            source=np.ones(len(values),dtype=bool)
            for side,pick in self.picks.items():
                T=actual[side];half=np.asarray(pick.dimensions_m)/2
                depth=box_overlap_depth(T[:,:3,3],T[:,:3,:3],half,np.asarray(pick.center_m),np.eye(3),half)
                source &= depth<=limits[side]
            clearance=torch.ones(len(values),device=values.device,dtype=torch.bool)
            classification=None
            if clearance_layer is not None:
                valid,uncertain,robot_depth=clearance_layer.classify_with_robot_depth(
                    SimpleNamespace(cuda_robot_model_state=state,tool_poses=state.tool_poses))
                classification=(valid,uncertain)
                clearance=robot_depth.reshape(-1)<=-5e-6
                # Query-local results, consumed before the original CPU fallback overwrites FK.
            payload=self.payload_search_mask(checker,values,state=state,classification=classification)
            joint=((values>=lower+1e-5)&(values<=upper-1e-5)).all(-1)
            checks=torch.stack((native,payload,joint,clearance)).cpu().numpy()
            filter_statistics['rows']+=len(values)
            filter_statistics['source_rejected']+=int((~source).sum())
            filter_statistics['native_margin_rejected']+=int((~checks[0]).sum())
            filter_statistics['full_payload_rejected']+=int((~checks[1]).sum())
            filter_statistics['joint_bound_rejected']+=int((~checks[2]).sum())
            filter_statistics['proposal_clearance_rejected']+=int((~checks[3]).sum())
            return checks[0]&checks[1]&checks[2]&checks[3]&source
        if not hasattr(self,'layout_lateral_lm'):
            lm=SequentialPlacementLm(self,samples=512,active_sides=('left','right'))
            # Only the new lateral proposal needs micrometre-level convergence.
            # Preserve the baseline LM constructor/budgets and all physical gates.
            self.layout_lateral_baseline_margin=lm.solver.config.joint_limit_margin
            precise=replace(lm.solver.config,position_tolerance=1e-6,orientation_tolerance=1e-5,
                convergence_position_tolerance=1e-6,convergence_orientation_tolerance=1e-5,
                joint_limit_margin=0.,inner_iterations=32)
            lm.solver.destroy();lm.solver=SeedIKSolver(precise);lm.owner.seed_ik_solver=lm.solver
            self.layout_lateral_lm=lm
        lm=self.layout_lateral_lm
        report['lateral_solver_tolerance']=dict(position_m=1e-6,orientation_rad=1e-5,
            convergence_position_m=1e-6,convergence_orientation_rad=1e-5,
            joint_soft_margin_fraction=0.,hard_joint_sampling_margin=1e-5,
            max_iterations=lm.iterations,inner_iterations=32,seeds=lm.seeds)
        margin=getattr(self,'layout_lateral_baseline_margin',.001)
        raw=torch.as_tensor(start,device=lower.device,dtype=lower.dtype)
        excluded=((raw<lower+(upper-lower)*margin)|(raw>upper-(upper-lower)*margin)).cpu().numpy()
        report['valid_start_excluded_by_old_lm_optimization_margin']=[ACTIVE_JOINTS[i] for i in np.flatnonzero(excluded)]
        rows=np.asarray(start,dtype=np.float32)[None]
        current=starts
        # Clear the higher source carton first; a failed proposal never changes
        # the requested layout or excludes the original single-root RRT search.
        for side in sorted(moving,key=lambda name:self.picks[name].center_m[2],reverse=True):
            progress(dict(stage='lateral_'+side,kind='geometry',state='running'))
            before_filter=dict(filter_statistics)
            pick=self.picks[side];ends={name:T.copy() for name,T in current.items()}
            held_y=pick.carry_box()[1,3];delta=held_y-current[side][1,3]
            direction=np.sign(delta) if abs(delta)>1e-9 else (1 if side=='left' else -1)
            half=np.asarray(pick.dimensions_m)/2
            clearance=half[1]+np.abs(current[side][1,:3])@half+.005
            ends[side][1,3]=current[side][1,3]+direction*max(abs(delta),clearance)
            if align_moving:
                for box in ends.values():box[:3,:3]=np.eye(3)  # Both held cartons may rotate during transport; final45deg stays unchanged.
            case=dict(start=rows[-1].copy(),checker=checker,attachments=attachments,
                starts=current,ends=ends,candidate_filter=candidate_filter)
            result=(adaptive_lateral(lm,case,progress=lambda event:progress(dict(event,stage='lateral_'+side,kind='geometry_substage')))
                    if adaptive else lm.plan(case))
            stage=dict(side=side,target_y_m=float(ends[side][1,3]),
                lm={k:v for k,v in result.items() if k not in ('frames','point_reports','failed_candidates')})
            stage['candidate_filter_statistics']={name:value-before_filter[name] for name,value in filter_statistics.items()}
            report['stages'].append(stage)
            progress(dict(stage='lateral_'+side,kind='geometry',state='done' if result['success'] else 'rejected',
                wall_seconds=result['total_ms']/1000,details=stage,
                reason=None if result['success'] else 'forward LM combined physical/source rejection: '+str(result['failure'])+'; '+str(stage['candidate_filter_statistics'])))
            if not result['success']:
                stage['failed_point']=result['point_reports'][-1] if result['point_reports'] else None
                return failed('forward lateral LM: '+str(result['failure']))
            proposal=np.asarray(result['frames'],dtype=np.float32)
            if np.max(np.abs(proposal[0]-rows[-1]))>1e-5:raise RuntimeError('disconnected forward lateral LM proposal')
            rows=np.concatenate((rows,proposal[1:]))
            stage['end_joint_values']=rows[-1].tolist()
            current={name:T[0] for name,T in box_poses(checker,rows[-1:],attachments).items()}
        dense=np.asarray(densify(rows,maximum_weighted_step=.001),dtype=np.float32)
        report['native_section_ends']=[int(np.argmin(np.max(np.abs(dense-np.asarray(stage['end_joint_values'])),axis=1))) for stage in report['stages']]
        validity=self.path_validity(checker,start=start,goal=dense[-1])
        mask=validity.mask(torch.as_tensor(dense,device='cuda')).cpu().numpy()
        observations={side:source_slot_observation(self.picks[side],T) for side,T in box_poses(checker,dense,attachments).items()}
        process=source_slot_process_checks(observations)
        report.update(checked_rows=len(dense),invalid_frames=np.flatnonzero(~mask).tolist(),process=process,
            wall_seconds=time.perf_counter()-began)
        if not mask.all() or not process['valid']:
            return failed('fresh forward lateral prefix failed original gates/source clearance; not input infeasibility')
        return dense,report

    def pair_path(self, start, goals, checker, strict=None, seed=20261119, budget_s=8., progress=lambda event:None):
        if not np.isfinite(budget_s) or budget_s <= 0:raise ValueError('RRT budget must be positive and finite')
        if not len(goals):return None, dict(failure='no compatible dual IK goals')
        actual_start = np.asarray(start, dtype=np.float32)
        roots=actual_start
        prefix=None;prefix_detail=None
        if getattr(checker,'guidance_payload',False):
            prefix,prefix_detail=self.forward_lateral_prefix(actual_start,checker,progress=progress)
            if prefix is not None:
                if np.max(np.abs(prefix[0]-actual_start))>1e-5:raise RuntimeError('forward prefix does not start at actual extraction endpoint')
                # Both are true reachable roots: preserve the original search,
                # and connect the extra root only via its checked forward path.
                roots=np.stack((actual_start,prefix[-1]))
        validity = self.path_validity(checker, strict, actual_start, goals[0])
        lower, upper = checker.kinematics.get_joint_limits().position
        if getattr(checker,'guidance_payload',False):
            def process_quality(values):
                depth=self.source_slot_depths(checker,values)
                cleared=depth<=1e-6
                returned=(depth>1e-6)&(torch.cumsum(cleared.to(torch.int32),dim=0)>0)
                if bool(returned.any()):return False
                if prefix is not None:
                    selected=int(torch.argmin(torch.max(torch.abs(torch.as_tensor(roots,device=values.device)-values[0]),dim=1).values))
                    if selected==1 and bool((depth>1e-6).any()):return False
                return bool((depth[-1]<=1e-6).all())
            validity.path_quality_callback=process_quality
        before = dict(getattr(checker, 'payload_search_statistics', {}))
        path, details = batched_rrt_connect_multi_goal(torch.as_tensor(roots, device='cuda', dtype=torch.float32),
            torch.as_tensor(goals, device='cuda', dtype=torch.float32), lower+1e-5, upper-1e-5,
            validity, budget_s, seed, stop_at_first_valid=True)
        details.update(seed=int(seed), budget_s=float(budget_s), padded_query_frames=getattr(validity, 'search_padding_frames', 0),
            steering_steps=list(getattr(validity, 'steering_steps', (.28,))),
            steering_multiscale_used=bool(getattr(validity, 'steering_multiscale_used', False)))
        after = getattr(checker, 'payload_search_statistics', {})
        if after:details['full_payload_search'] = {key: value-before.get(key, 0) for key, value in after.items()}
        selected=0
        if path is not None:
            advertised=roots.reshape(-1,15)
            if not len(path):raise RuntimeError('RRT returned an empty path')
            selected=int(np.argmin(np.max(np.abs(advertised-path[0]),axis=1)))
            if np.max(np.abs(advertised[selected]-path[0]))>1e-5:
                raise RuntimeError('RRT path does not start at the actual requested state or checked forward endpoint')
        if prefix_detail is not None:details['fresh_forward_lateral_prefix']=prefix_detail
        details['selected_start_root']=selected if path is not None else None
        if path is not None and prefix is not None and selected==1:path=np.concatenate((prefix[:-1],path))
        return (None if path is None else np.asarray(densify(path))), details

    def plan_layout(self, picks, output, max_contacts=16, progress=lambda event: None, adapt_carry=True, adapt_grasp=True):
        """Keep requested boxes fixed; retry footprint-valid contact points and near-arm side faces."""
        validate_pair(picks)
        candidates = [picks]
        left, right = picks['left'], picks['right']
        centered = all(not np.any(p.suction_offset_m) and p.suction_roll_deg == 0. and p.side_face == 'x-' for p in picks.values())
        if (adapt_grasp and centered and not left.top and not right.top
                and abs(left.center_m[1]-right.center_m[1]) < (left.dimensions_m[1]+right.dimensions_m[1])/2-1e-9
                and abs(left.center_m[2]-right.center_m[2]) > 1e-9):
            middle = (left.center_m[2]+right.center_m[2])/2
            # ponytail: bounded planar grasp proposals; use continuous grasp search if these miss a known feasible contact.
            for vertical_shift in (0., .01, -.01):
                candidates.append({s:replace(p, suction_offset_m=(0., 0.,
                    (.08 if p.center_m[2] > middle else -.08)+vertical_shift)) for s, p in picks.items()})
        if adapt_grasp and centered:
            for side, face in (('right', 'y-'), ('left', 'y+')):
                if not picks[side].top:
                    candidate = dict(picks)
                    candidate[side] = replace(picks[side], side_face=face, suction_roll_deg=90.)
                    candidates.append(candidate)
        trials = []
        original_candidates = candidates
        for swapped in ((False, True) if adapt_grasp else (False,)):
            owners = {'left': 'wall_box_01', 'right': 'wall_box_00'} if swapped else {'left': 'wall_box_00', 'right': 'wall_box_01'}
            for candidate in original_candidates:
                if swapped:
                    try:
                        candidate = {s: replace(candidate[other], side=s) for s, other in (('left', 'right'), ('right', 'left'))}
                    except ValueError as error:
                        progress(dict(stage='actual_arm_assignment_recovery', kind='geometry', state='rejected',
                            owner_to_physical_id=owners, reason=str(error)))
                        continue
                index = len(trials)
                folder = Path(output) if index == 0 else Path(output)/f'grasp_{index:02d}'
                folder.mkdir(parents=True, exist_ok=True)
                if index:
                    progress(dict(stage='actual_arm_assignment_recovery' if swapped else 'suction_grasp_recovery',
                        kind='geometry', state='running', grasp_trial=index, owner_to_physical_id=owners,
                        reason='retry physical carton contacts with original poses and immutable IDs'))
                result = self._plan_layout(candidate, folder, max_contacts, progress, adapt_carry,
                    **(dict(owner_to_physical_id=owners) if swapped else {}))
                record = folder/'grasp_result.json'
                record.write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
                trials.append(dict(index=index, success=result['success'], failure=result.get('failure'),
                    owner_to_physical_id=owners,
                    suction={s:dict(face='z+' if p.top else p.side_face, offset_m=p.suction_offset_m, roll_deg=p.suction_roll_deg) for s, p in candidate.items()},
                    result_file=str(record.resolve()), candidate_budget=max_contacts))
                if result['success']:break
            if result['success']:break
        stacked_top = (any(p.top for p in picks.values())
            and abs(left.center_m[1]-right.center_m[1]) < (left.dimensions_m[1]+right.dimensions_m[1])/2-1e-9)
        if not result['success'] and stacked_top:
            progress(dict(stage='ordered_pick_recovery',kind='geometry',state='running',
                reason='synchronous contact candidates exhausted; keep original boxes and try upper-first with real changing payload state'))
            from v3_ordered_layout import plan_ordered_pick
            ordered=plan_ordered_pick(picks,Path(output)/'ordered',max_contacts,progress,adapt_grasp=adapt_grasp)
            trace=Path(output)/'ordered_result.json';trace.write_text(json.dumps(ordered,indent=2,ensure_ascii=False)+'\n')
            trials.append(dict(index=len(trials),mode='ordered_upper_first',success=ordered['success'],
                failure=ordered.get('failure'),result_file=str(trace.resolve()),candidate_budget=max_contacts))
            if ordered['success']:result=ordered
            else:result['ordered_recovery']=dict(success=False,result_file=str(trace.resolve()),failure=ordered.get('failure'))
        if not result['success'] and adapt_grasp:
            progress(dict(stage='handover_pick_recovery',kind='geometry',state='running',
                reason='try a fresh single-box handover and second pickup with the original physical cartons'))
            from v3_handover_layout import plan_handover_pick
            handover=plan_handover_pick(picks,Path(output)/'handover',max_contacts,progress)
            trace=Path(output)/'handover_result.json';trace.write_text(json.dumps(handover,indent=2,ensure_ascii=False)+'\n')
            trials.append(dict(index=len(trials),mode='handover_same_side',success=handover['success'],
                failure=handover.get('failure'),result_file=str(trace.resolve()),candidate_budget=max_contacts))
            if handover['success']:result=handover
            else:
                result['handover_recovery']=dict(success=False,result_file=str(trace.resolve()),failure=handover.get('failure'))
                result['failure_before_handover']=result.get('failure')
                result['failure']='automatic handover recovery rejected: '+str(handover.get('failure') or 'inspect handover_result.json')
                result['failure_kind']='handover_recovery_failed'
        result['grasp_trials'] = trials
        result['requested_tasks'] = {s:asdict(p) for s, p in picks.items()}
        result['grasp_selection_policy'] = 'original request first; bounded outward vertical8cm contact-height variants for stacked side grasps and near-arm Y side-face alternatives; per-grasp candidate budget; then swap actual arms preserving original physical IDs; blocked top stacks additionally try genuine upper-first ordered pickup; unresolved automatic requests additionally try a fresh one-box handover; source poses/global threshold/user joint goal and all physical gates retained'
        return result

    def _plan_layout(self, picks, output, max_contacts=16, progress=lambda event: None, adapt_carry=True, *, owner_to_physical_id=None):
        began = time.perf_counter()
        self.configure_layout(picks, owner_to_physical_id)
        output = Path(output)
        source_files = [Path(__file__), ROOT/'tools/v3_multiroot_search.py', ROOT/'tools/curobo_core/box_pick.py', ROOT/'tools/curobo_core/layout_contract.py', ROOT/'tools/curobo_core/suction_grasp.py', ROOT/'tools/curobo_core/backend.py', ROOT/'tools/v3_contact_validity.py', ROOT/'tools/v3_task_cycle_planner.py', ROOT/'tools/v3_batched_loaded_search.py',
            ROOT/'tools/curobo_core/contact_reach.py', ROOT/'tools/curobo_core/native_transport.py', ROOT/'generated/fast_runtime_assets/fixed.urdf', ROOT/'tools/v3_two_box_pick.py', ROOT/'tools/v3_lm_conveyor.py', ROOT/'tools/v3_full_payload_cost.py', ROOT/'tools/v3_fast_geometry_cost.py', ROOT/'tools/v3_payload_safety.py', ROOT/'tools/curobo_core/payload_geometry.py', ROOT/'tools/curobo_core/torque.py', ROOT/'tools/v3_box_rotation_cost.py',
            ROOT/'tools/v3_box_layout.py', ROOT/'tools/v3_lm_refinement.py', ROOT/'tools/curobo_core/lm_refinement.py',
            ROOT/'config/fast_three_layers.json', ROOT/'config/two_box_layout/environment.json', self.args.urdf, self.args.robot_config]
        report = dict(task={s: asdict(p) for s, p in picks.items()},
            owner_to_physical_id=dict(self.owner_to_physical_id), joint_names=ACTIVE_JOINTS,
            frame_id='world', world_from_robot_base=np.eye(4).tolist(), success=False, geometry_success=False,
            real_robot_execution=False, scene=self.snapshot.to_dict(), attempts=[], frames=[], phases=[], stage_timings=[],
            contacts_attempted_limit=max_contacts, search_budget_s=8.,
            transport_budget=dict(total_s=8.*max_contacts, spent_s=0., recovery_calls=0, first_recovery_cap_s=32.),
            search_budget_policy='ordinary calls8s; first reverse-recovery transport up to32s, shared transport total8*max_contacts',
            source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files},
            carry_target_contract='2026-11-23 user joint angles; free shared updown',
            layout_contract=validate_pair(picks))
        def measured(stage, function, attempt=None):
            progress(dict(stage=stage, kind='geometry', state='running', attempt=attempt))
            start = time.perf_counter()
            try:
                value = function()
            except Exception as error:
                event = dict(stage=stage, kind='geometry', state='rejected', attempt=attempt,
                             wall_seconds=time.perf_counter()-start, reason=str(error))
                report['stage_timings'].append(event); progress(event)
                raise
            timing = dict(stage=stage, kind='geometry', state='done', attempt=attempt,
                          wall_seconds=time.perf_counter()-start)
            report['stage_timings'].append(timing); progress(timing)
            return value
        def finish(reason=None):
            report['failure'] = reason
            report['wall_seconds'] = time.perf_counter()-began
            report['static_load_search'] = dict(self.static_load_filter_statistics,
                planning_limit_scale=POLICY['planning_limit_scale'],
                policy='static-rest-safe search proposals; final timed RNEA unchanged; exhausted search is not input illegality')
            return report
        report['requested_tasks'] = report['task']
        empty = measured('initialize_checkers', lambda: self.checker(self.selected_task))
        loaded = measured('initialize_loaded_checker', lambda: self.checker(self.selected_task, payload=True))
        witnesses = fixed_extraction_blockages(picks, loaded.base_cuboid_layer.scene.cuboid)
        if witnesses:
            report['failure_kind'] = 'required_extraction_endpoint_blocked'
            report['extraction_impossibility_witnesses'] = witnesses
            reason = 'required full-width -X extraction endpoint intersects a fixed body even allowing the Cartesian tolerance envelope'
            progress(dict(stage='extraction_endpoint', kind='geometry', state='rejected', reason=reason, witnesses=witnesses))
            return finish(reason)
        def reach_witnesses():
            if not hasattr(self, '_contact_reach_model'):
                self._contact_reach_model = urdf_reach_model(self.fk_robot, self.analytic.offsets, self.analytic.signs)
            lower, upper = empty.kinematics.get_joint_limits().position
            return contact_reach_witnesses(picks, self._contact_reach_model, (float(lower[0]), float(upper[0])))
        reach = measured('contact_reach_bounds', reach_witnesses)
        if reach:
            report['failure_kind'] = 'assigned_contact_outside_reach'
            report['contact_reach_impossibility_witnesses'] = reach
            reason = 'required contact exceeds the assigned arm/shared-lift URDF reach bound, including the pose tolerance envelope; not a global rejection of the box position'
            progress(dict(stage='contact_reach_bounds', kind='geometry', state='rejected', reason=reason, witnesses=reach))
            return finish(reason)
        if not bool(self.target_validity(empty).mask(torch.as_tensor(self.home_values[None], device='cuda')).all()):
            return finish('Home collides with the configured two-box layout or fails native base/ground margins')
        native_key = layout_model_key(picks)
        cached_key, engine = getattr(self, 'layout_native', (None, None))
        native_reused = cached_key == native_key and engine is not None
        if not native_reused:
            engine = measured('initialize_native', lambda: FastThreeLayerPlanner(planner=self))
            self.layout_native = (native_key, engine)
        def joint_carry_goals():
            lower, upper = carry_reference()[2]
            # ponytail: 1cm lift proposals; refine if a narrow feasible interval is observed.
            lifts = np.unique(np.r_[np.linspace(lower, upper, 101), self.home_values[0]])
            goals = np.asarray([[carry_configuration(lift)[name] for name in ACTIVE_JOINTS] for lift in lifts], dtype=np.float32)
            values = torch.as_tensor(goals, device='cuda')
            mask = self.target_validity(loaded).mask(values).cpu().numpy()
            mask &= self.native_margin_mask(loaded, values).cpu().numpy()
            mask &= self.static_load_mask(loaded, values).cpu().numpy()
            before_source = int(mask.sum())
            mask &= (self.source_slot_depths(loaded, values) <= 1e-6).all(-1).cpu().numpy()
            report['carry_goal_filter'] = dict(lift_proposals=len(goals), physically_valid_before_exact_audit=before_source,
                source_clear_after_physical_filters=int(mask.sum()), source_clearance_m=1e-6)
            for i in np.flatnonzero(mask):
                mask[i] = self.loaded_audit(loaded, goals[i:i+1])['valid']
            return goals[mask]
        carry_goals = measured('carry_joint_goals', joint_carry_goals)
        report['carry_goal_count'] = len(carry_goals)
        report['carry_goal_updown_m'] = carry_goals[:, 0].tolist()
        if not len(carry_goals):
            report['failure_kind'] = 'joint_carry_goal_grid_exhausted'
            return finish('specified joint carry has no goal passing collision and final source-clearance gates on the 1cm lift grid; not proof the layout is illegal')
        contact_validity = self.target_validity(empty, allow_contact=True)
        distances = {s: p.precontact_distance(contact_validity.targets[s]['tool_cap_reach_m']) for s, p in picks.items()}
        report['precontact_distance_m'] = distances
        contacts = measured('contact_ik', lambda: self.enumerate_pair({s: p.contact_tool() for s, p in picks.items()},
                                                                 self.home_values, empty, contact=True))
        report['contact_count'] = len(contacts)
        def extraction_engine():
            if ('left', 'right') not in self.extraction_engines:
                self.extraction_engines[('left', 'right')] = SequentialPlacementLm(self, active_sides=('left', 'right'))
            return self.extraction_engines[('left', 'right')]
        report['resource_reuse'] = dict(extraction_lm=('left', 'right') in self.extraction_engines)
        lm = measured('initialize_extraction_lm', extraction_engine)
        report['resource_reuse']['native_engine'] = native_reused
        report['extraction_recovery'] = []
        report['contact_trial_policy'] = 'four nominal contacts first, then up to eight reverse recoveries on extraction or transport geometry failure; combined attempts never exceed max_contacts'
        def trials():
            first = min(4, max_contacts, len(contacts))
            for contact in contacts[:first]:yield contact, None
            failures = [a.get('failure', '') for a in report['attempts']]
            recover = not len(contacts) or (failures and all(f.startswith(('dual precontact', 'dual LM extraction',
                'full-size boxes', 'extracted endpoint', 'joint transport RRT exhausted')) for f in failures))
            if recover and len(report['attempts']) < max_contacts:
                for reverse in itertools.islice(self.reverse_extractions(contacts, empty, loaded, lm,
                        report['extraction_recovery'], progress), min(8, max_contacts-len(report['attempts']))):
                    if report['transport_budget']['spent_s'] >= report['transport_budget']['total_s']:break
                    yield np.asarray(reverse['frames'][0], dtype=np.float32), reverse
            for contact in contacts[first:]:yield contact, None
        for index, (contact, reverse) in enumerate(itertools.islice(trials(), max_contacts)):
            attempt = dict(contact_index=index, contact_source='reverse recovery' if reverse is not None else 'forward contact IK',
                           contact_joint_values=np.asarray(contact).tolist())
            report['attempts'].append(attempt)
            def reject(reason):
                attempt['failure'] = reason
                progress(dict(stage='candidate', state='rejected', attempt=index, reason=reason))
            contact_path = measured('precontact', lambda: self.precontact(contact, distances, empty), index)
            if contact_path is None:
                reject('dual precontact line has no fixed-lift analytic continuation or collides with a box'); continue
            tools = self._tool_poses(loaded, torch.as_tensor(contact, device='cuda'))
            attachments = loaded.task_tool_to_box
            starts = {s: tools[s]@a for s, a in attachments.items()}
            ends = {s: pose.copy() for s, pose in starts.items()}
            for s in ends:ends[s][0, 3] -= picks[s].extraction_distance_m
            extraction = measured('extract', lambda: reverse if reverse is not None else self.nominal_extraction(lm,
                dict(start=contact, checker=loaded, attachments=attachments, starts=starts, ends=ends),
                progress=lambda event:progress(dict(event,attempt=index))), index)
            attempt['extraction'] = {k: v for k, v in extraction.items() if k not in ('frames', 'point_reports', 'failed_candidates', 'proposal_payload_audit')}
            if not extraction['success']:
                attempt['extraction']['last_valid_joint_values'] = extraction.get('frames', [])[-1] if extraction.get('frames') else None
                attempt['extraction']['failed_point'] = extraction.get('point_reports', [])[-1] if extraction.get('point_reports') else None
                reject('dual LM extraction: '+str(extraction['failure'])); continue
            extracted = np.asarray(extraction['frames'], dtype=np.float32)
            attempt['extraction_audit'] = (extraction['proposal_payload_audit'] if extraction.get('proposal_payload_audit') is not None
                else self.loaded_audit(loaded, extracted))
            if not attempt['extraction_audit']['valid']:
                reject('full-size boxes collide during extraction (including box vs box)'); continue
            if not bool(self.native_margin_mask(loaded, torch.as_tensor(extracted[-1:], device='cuda')).all()):
                reject('extracted endpoint fails native transport-start base/ground margins'); continue
            if not bool(self.static_load_mask(loaded, torch.as_tensor(extracted[-1:], device='cuda')).all()):
                reject('extracted endpoint exceeds static rest-hold load; retry extraction candidates, not input illegality'); continue
            weights = np.r_[np.deg2rad(15)/.1, np.ones(14)]
            goals = carry_goals[np.argsort(np.linalg.norm((carry_goals-extracted[-1])*weights, axis=1))]
            attempt['carry_candidates'] = len(goals)
            budget = report['transport_budget']
            cap = 32. if reverse is not None and not budget['recovery_calls'] else 8.
            remaining = max(0., budget['total_s']-budget['spent_s'])
            if not remaining:
                reject('joint transport aggregate search budget exhausted; not proof the layout is illegal'); continue
            allotted = min(cap, remaining)
            transport, attempt['transport_search'] = measured('transport',
                lambda: self.pair_path(extracted[-1], goals, loaded,
                    seed=20261119+index-min(4, len(contacts)) if reverse is not None else 20261119, budget_s=allotted,
                    progress=lambda event:progress(dict(event,attempt=index))), index)
            budget['spent_s'] += attempt['transport_search'].get('search_ms', 0.)/1000
            budget['recovery_calls'] += int(reverse is not None)
            if transport is None:
                reject(attempt['transport_search'].get('failure') or
                    f'joint transport RRT exhausted its {allotted:g}s search budget; not proof the layout is illegal'); continue
            approach, attempt['approach_search'] = measured('approach',
                lambda: self.pair_path(self.home_values, contact_path[:1], empty, self.target_validity(empty),
                    seed=20261119+index-min(4, len(contacts)) if reverse is not None else 20261119), index)
            if approach is None:
                reject('joint Home-to-precontact RRT exhausted its 8s search budget'); continue
            chosen_lift = float(transport[-1, 0])
            picks = {s: replace(p, carry_updown_m=chosen_lift) for s, p in picks.items()}
            self.picks = picks
            report['task'] = {s: asdict(p) for s, p in picks.items()}
            self.target_poses = {s+'_tool0': dict(position=(pose := matrix_pose(p.carry_tool())).position,
                                                 quaternion=pose.quaternion_wxyz) for s, p in picks.items()}
            physical_labels = {s: {'wall_box_00': 'left', 'wall_box_01': 'right'}[pid]
                               for s, pid in self.owner_to_physical_id.items()}
            progress(dict(stage='carry_target_selected', kind='geometry', state='done',
                owner_to_physical_id=dict(self.owner_to_physical_id),
                updown_m=chosen_lift, targets={physical_labels[s]: p.carry_box()[:3, 3].tolist() for s, p in picks.items()},
                box_matrices={physical_labels[s]:p.carry_box().tolist() for s,p in picks.items()},
                suctions={physical_labels[s]:dict(face='z+' if p.top else p.side_face, offset_m=p.suction_offset_m,
                                roll_deg=p.suction_roll_deg) for s,p in picks.items()}))
            report['frames'] = []; report['phases'] = []
            for phase, rows in [('approach', approach), ('contact', contact_path), ('extract', extracted), ('transport', transport)]:
                if report['frames'] and np.max(np.abs(np.asarray(report['frames'][-1])-rows[0])) > 1e-5:
                    raise RuntimeError('disconnected dual stage seam')
                report['frames'].extend(rows.tolist()); report['phases'].extend([phase]*len(rows))
            report['geometry_success'] = True
            if engine is None:
                engine = measured('initialize_native', lambda: FastThreeLayerPlanner(planner=self))
                self.layout_native = (native_key, engine)
            native = time_layout_pick(engine, report, output/f'candidate_{index:02d}_native',
                stop_on_failure=True, progress=lambda e: progress(dict(e, attempt=index)))
            attempt['native'] = native
            report['timing'] = native
            if not native['success']:
                failed = next(s for s in native['stages'] if not s['success'])
                reject(f"native {failed['phase']}: "+'; '.join(failed.get('rejection_reasons', ['constraint or sampled-path validation failed']))); continue
            process_started = time.perf_counter()
            report['process_validation'] = source_slot_process_checks(report['process_validation']['source_slot_observation'])
            if report['process_validation']['valid']:
                gpu_process = report['process_validation']
                with np.load(native['stages'][-1]['trajectory_file'], allow_pickle=False) as arrays:
                    observations = self.source_observations_cpu(arrays['position'])
                report['process_validation'] = source_slot_process_checks(observations)
                report['process_validation']['gpu_pose_process'] = gpu_process
                report['process_validation']['pose_backend'] = 'stateless_float64_urdf_with_scalar_boundary_reference'
            attempt['process_validation'] = report['process_validation']
            progress(dict(stage='transport_process', kind='process', state='accepted' if report['process_validation']['valid'] else 'rejected',
                attempt=index, wall_seconds=time.perf_counter()-process_started,
                reason='; '.join(report['process_validation']['rejection_reasons']), details=report['process_validation']))
            if not report['process_validation']['valid']:
                reject('transport process: '+'; '.join(report['process_validation']['rejection_reasons'])); continue
            report['success'] = True
            return finish()
        return finish('contact candidate budget exhausted; see per-attempt rejection reasons, not a proof of illegality')
