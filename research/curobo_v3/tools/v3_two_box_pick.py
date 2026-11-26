"""PR82 two-size extraction-to-45deg experiment (no placement or real execution)."""
import argparse
from dataclasses import asdict, replace
import json
import hashlib
from pathlib import Path
import time

import numpy as np
import torch
from scipy.spatial.transform import Rotation
from curobo.types import JointState

from curobo_core.adapter import matrix_pose
from curobo_core.box_pick import BOX_DIMENSIONS, BoxPick, payload_fit, diverse_contact_order
from curobo_core.scene import AttachedObject, Pose, SceneObject
from curobo_core import box_pick
from v3_fast_assets import prepare_fast_assets
from v3_fixed_body_boxes import InflatedPlanningPlanner
from v3_lm_conveyor import SequentialPlacementLm, box_poses
from v3_payload_safety import exact_payload_audit, payload_search_mask
from v3_single_arm_planner import SingleArmPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS

ROOT = Path(__file__).resolve().parents[1]


class TwoBoxPickPlanner(InflatedPlanningPlanner):
    # Reuse PR82 candidate rejection/continuation, rather than aborting on a coarse first path.
    dense_path_rechecks = True

    def __init__(self):
        assets, _ = prepare_fast_assets()
        super().__init__(assets=assets)
        self.frozen_box_fit = self.box_fit
        self.environment = tuple(item for item in self.original_snapshot.objects
                                 if item.object_id in ('ground', 'left_wall', 'right_wall', 'front_wall', 'ceiling'))
        self.extraction_engines = {}
        self.native_project_margins = False
        self.carry_radius_limit = json.loads((ROOT/'config/placement_task_policy.json').read_text())['side']['placement_radius_m']

    def configure_pick(self, task):
        self.pick = task
        self.selected_task = {task.side: 0}
        self.top_grasp = task.top
        self.base_advance = 0.
        self.planner_kind = 'informed_connect'
        self.box_fit = payload_fit(self.frozen_box_fit, task.dimensions_m)
        objects = self.environment
        self.snapshot = replace(self.original_snapshot, meshes=(), attachments=(), objects=objects + (
            SceneObject('wall_box_00', task.dimensions_m, Pose(task.center_m)),),
            revision=self.snapshot.revision + 1)
        pose = matrix_pose(task.carry_tool())
        self.target_poses = {task.side+'_tool0': dict(position=pose.position, quaternion=pose.quaternion_wxyz)}
        self.last_partial = None

    def native_margin_mask(self, checker, values, *, state=None):
        """Same PR82 zero set; optional fresh query-local FK avoids recomputation."""
        def check(current, count):
            base=checker.base_cuboid_layer.cost(current).reshape(count,-1).sum(-1)
            spheres=current.robot_spheres.reshape(count,-1,4)
            exempt = set(getattr(checker, 'payload_ground_exempt_sphere_indices', ()))
            ground_spheres = spheres[:, [i for i in range(spheres.shape[1]) if i not in exempt]] if exempt else spheres
            ground=torch.where(ground_spheres[...,3]>0,
                torch.relu(ground_spheres[...,3]-ground_spheres[...,2]+.002),0.).square().sum(-1)
            return (base==0)&(ground==0)
        if state is not None:
            if state.robot_spheres.shape[:2] != (1,len(values)):
                raise ValueError('native-margin FK state must match this exact joint-row batch')
            return check(state,len(values))
        masks=[]
        for chunk in values.split(512):
            if not len(chunk):continue
            current=checker.kinematics.compute_kinematics(JointState.from_position(chunk[None],joint_names=ACTIVE_JOINTS))
            masks.append(check(current,len(chunk)))
        return torch.cat(masks) if masks else torch.empty(0,device=values.device,dtype=torch.bool)

    def target_validity(self, checker, allow_contact=False):
        validity=super().target_validity(checker,allow_contact)
        if self.native_project_margins and not allow_contact:
            ordinary=validity.evaluate
            def evaluate(values,base_margin_m=0.):
                mask,stability=ordinary(values,base_margin_m)
                return mask & self.native_margin_mask(checker,values),stability
            validity.evaluate=evaluate
        return validity

    def contact_poses_for_task(self, boxes):
        pose = matrix_pose(self.pick.contact_tool())
        return {self.pick.side+'_tool0': dict(position=pose.position, quaternion=pose.quaternion_wxyz)}

    def task_attachments(self, boxes):
        task = self.pick
        return (AttachedObject('wall_box_00', task.dimensions_m, task.side+'_tool0',
            matrix_pose(task.tool_to_box()), (task.side+'_link7', task.side+'_tool0')),)

    def loaded_audit(self, checker, rows):
        return exact_payload_audit(checker, rows, checker.task_tool_to_box,
                                   self.pick.dimensions_m, self.robot, check_ground=True)

    def payload_search_mask(self, checker, values, *, state=None, classification=None):
        fits = self.box_fit
        sizes = {s: (fits[s] if s in fits else fits)['dimensions_m'] for s in checker.task_tool_to_box}
        return payload_search_mask(checker, values, sizes, self.robot,
                                   lambda rows: self.loaded_audit(checker, rows), state=state, classification=classification)

    def enumerate_single(self, matrix, reference, checker, limit=64):
        loaded = getattr(checker, 'guidance_payload', False)
        rows = super().enumerate_single(matrix, reference, checker, limit=max(limit, 256) if loaded else limit)
        if loaded and len(rows):
            audit = self.loaded_audit(checker, rows)
            self.single_enumeration_report['exact_payload_audit'] = audit
            mask = np.ones(len(rows), dtype=bool)
            mask[audit['invalid_frame_indices']] = False
            rows = rows[mask]
        return rows[:limit]

    def single_validity(self, checker, start, goal, rotation=False):
        validity = super().single_validity(checker, start, goal, rotation)
        if getattr(checker, 'guidance_payload', False):
            baseline_evaluate = validity.evaluate

            def evaluate(values, base_margin_m=0.):
                mask, stability = baseline_evaluate(values, base_margin_m)
                indices = mask.nonzero().reshape(-1)
                if len(indices):
                    mask[indices] &= self.payload_search_mask(checker, values[indices].detach())
                if self.native_project_margins:mask &= self.native_margin_mask(checker,values)
                return mask, stability

            validity.evaluate = evaluate
        return validity

    def extract(self, contact, checker):
        key = self.pick.side
        if key not in self.extraction_engines:
            self.extraction_engines[key] = SequentialPlacementLm(self, active_sides=(key,))
        engine = self.extraction_engines[key]
        start = np.asarray(contact, dtype=np.float32)
        tools = self._tool_poses(checker, torch.tensor(start, device='cuda'))
        attachments = checker.task_tool_to_box
        starts = {side: tools[side] @ matrix for side, matrix in attachments.items()}
        ends = {side: matrix.copy() for side, matrix in starts.items()}
        for matrix in ends.values():matrix[0, 3] -= self.pick.extraction_distance_m
        return engine.plan(dict(start=start, checker=checker, attachments=attachments, starts=starts, ends=ends))

    def plan_pick(self, task, max_contacts=16, candidate_validator=None):
        began = time.perf_counter()
        validation_seconds = 0.
        self.configure_pick(task)
        report = dict(task=asdict(task), dimensions_xyz_m=task.dimensions_m,
            carry_target_contract='2026-11-19-v2; resolved offset relative to original PR82 anchor',
            requested_carry_box_matrix=task.carry_box().tolist(),
            baseline_commit='ee6dba3', source_sha256={str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in (Path(__file__), Path(box_pick.__file__), self.args.robot_config, ROOT/'tools/v3_base_mesh_layer.py')},
            suction='top' if task.top else 'side', extraction_distance_m=task.extraction_distance_m,
            success=False, geometry_success=False, real_robot_execution=False, joint_names=ACTIVE_JOINTS,
            scene=self.snapshot.to_dict(), attempts=[], frames=[], phases=[],
            native_project_margins=self.native_project_margins,
            verification_scope='sampled kinematic candidate; no dynamics certification or continuum coverage')
        footprint=self.fk_robot.get_transform('base_footprint',self.fk_robot.base_link)[:2,3]
        envelope=task.carry_payload_envelope(self.box_fit,footprint)
        report['requested_carry_envelope']=dict(limit_m=self.carry_radius_limit,**envelope)
        if envelope['legacy_radius_lower_bound_with_pose_tolerance_m']>self.carry_radius_limit:
            report.update(failure='requested carry target exceeds the PR82 envelope; not a rejection of the input box location',
                failure_kind='carry_target_contract_conflict',wall_seconds=time.perf_counter()-began)
            return report
        empty = self.checker(self.selected_task)
        loaded = self.checker(self.selected_task, payload=True)
        home = torch.tensor(self.home_values[None], device='cuda', dtype=torch.float32)
        if not bool(self.target_validity(empty).mask(home).all()):
            report['failure'] = 'initial Home fails configured scene/target or margin checks'
            report['wall_seconds'] = time.perf_counter() - began
            return report
        contacts = self.enumerate_single(task.contact_tool(), self.home_values, empty, limit=256)
        contact_order=diverse_contact_order(contacts,task.side)
        contacts=contacts[contact_order]
        report['contact_selection']=dict(policy='round-robin elbow families, nearest Home within each; retain the whole supplied pool',
            original_home_ranks=contact_order.tolist(),pool_limit=256,candidates_discarded_by_reordering=0)
        report['contact_enumeration'] = self.single_enumeration_report
        report['contact_count'] = len(contacts)
        report['contacts_attempted_limit'] = max_contacts
        contact_validity = self.target_validity(empty, allow_contact=True)
        contact_validity.scene_penetration_tolerance_m = .001
        contact_validity.contact_penetration_tolerance_m = .001
        precontact_distance = task.precontact_distance(contact_validity.targets[task.side]['tool_cap_reach_m'])
        report['precontact_distance_m'] = precontact_distance
        for index, contact in enumerate(contacts[:max_contacts]):
            attempt = dict(contact_index=index,contact_home_rank=int(contact_order[index]))
            report['attempts'].append(attempt)
            if not bool(contact_validity.mask(torch.tensor(contact[None], device='cuda')).all()):
                attempt['failure'] = 'contact pose intersects task box outside suction cap'
                continue
            reverse, detail = SingleArmPlanner.cartesian_single(self, contact, np.linspace(.005, precontact_distance, int(np.ceil(precontact_distance/.005))), along_normal=True)
            if reverse is None:
                attempt.update(failure='precontact line IK', detail=detail)
                continue
            contact_path = np.asarray(reverse[::-1], dtype=np.float32)
            validity = self.target_validity(empty, allow_contact=True)
            validity.scene_penetration_tolerance_m = .001
            if not bool(validity.mask(torch.tensor(contact_path, device='cuda')).all()):
                attempt['failure'] = 'precontact line collides'
                continue
            extraction = self.extract(contact, loaded)
            attempt['extraction'] = {k: v for k, v in extraction.items() if k not in ('frames', 'tracks', 'point_reports', 'failed_candidates')}
            if not extraction['success']:
                attempt['failure'] = 'LM extraction failed'
                continue
            extracted = np.asarray(extraction['frames'], dtype=np.float32)
            attempt['extraction_audit'] = self.loaded_audit(loaded, extracted)
            if not attempt['extraction_audit']['valid']:
                attempt['failure'] = 'full cuboid extraction audit failed'
                continue
            if self.native_project_margins:
                end = torch.as_tensor(extracted[-1:], device='cuda', dtype=torch.float32)
                attempt['native_transport_start_valid'] = bool(self.native_margin_mask(loaded, end).all())
                if not attempt['native_transport_start_valid']:
                    # An invalid RRT root cannot leave itself: every outgoing edge includes it.
                    # Keep the frozen native margins; do not burn the search budget or call this input illegal.
                    attempt['failure'] = 'extracted state fails native transport-start base/ground margins'
                    continue
            goals = self.enumerate_single(task.carry_tool(), extracted[-1], loaded, limit=256)
            if len(goals):
                radii=self.full_envelope_radius(loaded,goals).detach().cpu().numpy()
                attempt['carry_goal_envelope']=dict(limit_m=self.carry_radius_limit,
                    before_filter=len(goals),retained=int((radii<=self.carry_radius_limit).sum()),
                    minimum_radius_m=float(radii.min()),maximum_radius_m=float(radii.max()))
                goals=goals[radii<=self.carry_radius_limit]
            goals=goals[:64]
            attempt['carry_candidates'] = len(goals)
            attempt['carry_enumeration'] = self.single_enumeration_report
            if not len(goals):
                attempt['failure'] = '45deg carry goal has no candidate satisfying geometry and the baseline envelope'
                continue
            transport, stats = self.single_path(extracted[-1], goals, loaded, rotation=True, budget=5.)
            attempt['transport_search'] = stats
            if transport is None:
                attempt['failure'] = 'transport search exhausted'
                continue
            attempt['transport_audit'] = self.loaded_audit(loaded, transport)
            if not attempt['transport_audit']['valid']:
                attempt['failure'] = 'full cuboid transport audit failed'
                continue
            approach, stats = self.single_path(self.home_values, contact_path[:1], empty,
                                               strict=self.target_validity(empty), budget=5.)
            attempt['approach_search'] = stats
            if approach is None:
                attempt['failure'] = 'Home to precontact search exhausted'
                continue
            final_box = box_poses(loaded, transport[-1:], loaded.task_tool_to_box)[task.side][0]
            position_error = float(np.linalg.norm(final_box[:3, 3] - task.carry_box()[:3, 3]))
            angle_error = float(np.rad2deg(Rotation.from_matrix(task.carry_box()[:3, :3].T @ final_box[:3, :3]).magnitude()))
            initial_box = box_poses(loaded, extracted[:1], loaded.task_tool_to_box)[task.side][0]
            end_box = box_poses(loaded, extracted[-1:], loaded.task_tool_to_box)[task.side][0]
            displacement = end_box[:3, 3] - initial_box[:3, 3]
            extraction_error = float(np.linalg.norm(displacement - [-task.extraction_distance_m, 0., 0.]))
            endpoint_checks = dict(carry_position_error_m=position_error,
                carry_orientation_error_deg=angle_error, actual_extraction_translation_m=displacement.tolist(),
                extraction_translation_error_m=extraction_error)
            if position_error > .002 or angle_error > 1. or extraction_error > .002:
                attempt['failure'] = 'actual FK endpoint differs from extraction/carry contract'
                continue
            report['frames']=[];report['phases']=[];report['endpoint_checks']=endpoint_checks
            for phase, rows in (('approach', approach), ('contact', contact_path), ('extract', extracted), ('transport', transport)):
                if report['frames'] and np.max(np.abs(np.asarray(report['frames'][-1]) - rows[0])) > 1e-5:
                    raise RuntimeError('stage seam mismatch; refusing disconnected trajectory')
                report['frames'].extend(np.asarray(rows).tolist())
                report['phases'].extend([phase]*len(rows))
            report['geometry_success'] = True
            report['success'] = True
            if candidate_validator is not None:
                validation_started = time.perf_counter()
                report['success'] = False
                validation = candidate_validator(report)
                validation_seconds += time.perf_counter()-validation_started
                attempt['candidate_validation'] = validation
                report['success'] = bool(validation['success'])
                if not report['success']:
                    attempt['failure'] = 'native validation rejected this geometric candidate; continue contacts'
                    continue
            break
        report['failure'] = None if report['success'] else 'candidate budget exhausted; NOT proof of illegality'
        report['wall_seconds'] = time.perf_counter() - began
        report['candidate_validation_seconds'] = validation_seconds
        report['geometry_wall_seconds'] = report['wall_seconds']-validation_seconds
        return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--size', choices=BOX_DIMENSIONS, required=True)
    parser.add_argument('--center', type=float, nargs=3, required=True, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--side', choices=('left', 'right'), required=True)
    parser.add_argument('--side-min-height', type=float, default=.8)
    parser.add_argument('--carry-offset', type=float, nargs=3, default=None, metavar=('X', 'Y', 'Z'),
                        help='Override offset from original PR82 anchor; omitted uses size/mode defaults, zero restores original XYZ')
    parser.add_argument('--precontact-margin', type=float, default=.005, help='Clearance beyond actual cap forward extent, metres')
    parser.add_argument('--max-contacts', type=int, default=16)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.max_contacts < 1:
        parser.error('--max-contacts must be positive')
    task = BoxPick(args.size, args.center, args.side, args.side_min_height, args.carry_offset, args.precontact_margin)
    report = TwoBoxPickPlanner().plan_pick(task, max_contacts=args.max_contacts)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps({key: report.get(key) for key in ('success', 'failure', 'wall_seconds', 'endpoint_checks')}, ensure_ascii=False))
    raise SystemExit(0 if report['success'] else 1)


if __name__ == '__main__':
    main()
