"""PR82 native TrajOpt/TimeOpt for the four requested pick-to-carry phases."""
import hashlib
from pathlib import Path
import time
import traceback

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from curobo_core.adapter import to_curobo_scene
from curobo_core.box_pick import timed_path_checks, native_rejection_reasons, source_slot_observation
from v3_batched_loaded_search import GpuValidity
from v3_fast_path import fit_path, save
from v3_fast_three_layers import POLICY
from v3_lm_conveyor import box_poses
from v3_lm_refinement import optimize_lm_stage
from curobo_core.ordered_pick import OrderedStage, cartesian_check

ROOT = Path(__file__).resolve().parents[1]
PHASES = ('approach', 'contact', 'extract', 'transport')


def time_pick(engine, source, output, stop_on_failure=False, progress=lambda event: None, *, phase_names=PHASES, final_transport=True, seed_only=False, stage_context=None):
    phase_names=tuple(phase_names)
    if not phase_names or len(set(phase_names))!=len(phase_names) or any(p not in PHASES for p in phase_names):
        raise ValueError('expected distinct known native phases')
    if (not final_transport or seed_only) and phase_names!=('transport',):
        raise ValueError('intermediate-held deferral and seed retries are only for a transport substage')
    planner = engine.planner
    if stage_context is not None and (not isinstance(stage_context, OrderedStage) or phase_names != (stage_context.phase,)
            or stage_context != getattr(planner, 'native_stage', None)):
        raise ValueError('ordered timing requires one declared, configured physical stage')
    tasks = getattr(planner, 'picks', None) or {planner.pick.side: planner.pick}
    sides = tuple(tasks)
    task = next(iter(tasks.values()))
    if not source['geometry_success']:
        raise ValueError('native timing requires a successful freshly planned geometry')
    dimensions = engine.dynamics.payload_dimensions
    for side, pick in tasks.items():
        actual_size = dimensions[side] if isinstance(dimensions, dict) else dimensions
        if not np.array_equal(actual_size, pick.dimensions_m):
            raise ValueError('timing resource payload dimensions differ from this request')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    for path in (Path(__file__), ROOT/'tools/v3_fast_three_layers.py', ROOT/'tools/v3_fast_geometry_cost.py', ROOT/'tools/v3_box_rotation_cost.py', ROOT/'tools/v3_base_mesh_layer.py'):
        source['source_sha256'][str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    attachments = {side: pick.tool_to_box() for side, pick in tasks.items()}
    result = dict(success=False, payload_kg=POLICY['payload_kg_per_box'],
        payload_dimensions_m=({side: value.tolist() for side, value in dimensions.items()}
                              if isinstance(dimensions, dict) else list(dimensions)), stages=[], seam_checks=[],
        lm_refinement_skipped=None if len(tasks) == 2 else 'PR82 single-arm adaptation has not been validated', real_robot_execution=False,
        is_full_cycle=phase_names==PHASES,final_transport=final_transport,
        verification_scope='native timed candidates, 4001 sampled points per requested phase; partial reports are not full cycles; not CCD or hardware certification')
    frames, phases = np.asarray(source['frames'], dtype=np.float32), np.asarray(source['phases'])
    previous = None
    started = time.perf_counter()
    for phase in phase_names:
        stage_started = time.perf_counter()
        progress(dict(stage=phase, state='running', kind='native'))
        stage = dict(phase=phase, success=False)
        active = frames[phases == phase]
        active_sides = stage_context.payload_sides if stage_context is not None else (sides if phase in ('extract', 'transport') else ())
        loaded = bool(active_sides)
        payload_attachments = ({side:attachments[side] for side in active_sides} if stage_context is not None else attachments)
        checker = planner.checker(planner.selected_task, payload=loaded)
        if stage_context is not None:
            if set(checker.task_tool_to_box) != set(active_sides):
                raise ValueError('actual stage checker attachments differ from declared payloads')
            validity = (planner.target_validity(checker, allow_contact=phase == 'contact')
                        if stage_context.incoming_side is not None else GpuValidity(checker, check_ground=True))
            stage.update(stage_id=stage_context.stage_id, payload_sides=list(active_sides),
                         pending_sides=list(stage_context.pending_sides), physical_scene=planner.snapshot.to_dict())
        else:
            validity = (GpuValidity(checker, check_ground=True) if loaded else
                        planner.target_validity(checker, allow_contact=phase == 'contact'))
        validity.scene_penetration_tolerance_m = .001
        validity.contact_penetration_tolerance_m = .001
        try:
            if phase in ('approach', 'transport'):
                # Approach must retain the real target box; it is removed only after attachment.
                scene = to_curobo_scene(planner.snapshot) if phase == 'approach' else checker.guidance_scene
                geometry_source = dict(task=planner.selected_task)
                if stage_context is not None:geometry_source['active_payload_sides'] = list(active_sides)
                controls, start, end, knot_dt, details = engine.optimize_path(
                    geometry_source, phase, active, validity, scene, **(dict(seed_only=True) if seed_only else {}))
                stage['trajectory_optimization'] = details
            elif phase == 'extract' and (len(stage_context.cartesian_displacements) if stage_context is not None else len(tasks)) == 2 and POLICY.get('lm_refinement', {}).get('enabled', False):
                translations = (dict(stage_context.cartesian_displacements) if stage_context is not None else
                                {s:[-p.extraction_distance_m, 0., 0.] for s,p in tasks.items()})
                references = dict(source, task={s:source['task'][s] for s in translations},
                    extraction_reference_frame='box_center', visual_tool_to_box={s:attachments[s].tolist() for s in translations},
                    selected_lm_review=dict(lm_detail=dict(translation_local=translations)))
                controls, start, end, knot_dt, details = optimize_lm_stage(
                    planner, references, phase, active, validity, POLICY['lm_refinement'])
                stage['lm_joint_refinement'] = details
                stage['lm_refinement_applied'] = True
            else:
                values, first, last, fit, _ = fit_path(dict(positions=active), 128)
                controls, start, end = [torch.tensor(v[None], device='cuda', dtype=torch.float32).contiguous()
                                        for v in (values, first, last)]
                knot_dt = None
                stage['representation_adaptation'] = fit
            control_file = output/f'{phase}_controls.npz'
            np.savez_compressed(control_file, controls=controls.detach().cpu().numpy(),
                start=start.detach().cpu().numpy(), end=end.detach().cpu().numpy(),
                knot_dt=np.empty(0) if knot_dt is None else knot_dt.detach().cpu().numpy())
            stage['control_file'] = str(control_file.resolve())
            arrays, native = engine.time_path(controls, start, end, knot_dt,
                active_sides, payload_attachments, benchmark_recompute=False)
            stage['time_optimization'] = native
            if arrays is not None:
                filename = output/f'{phase}.npz'
                np.savez_compressed(filename, **arrays)
                stage['trajectory_file'] = str(filename.resolve())
                stage['array_checks'] = timed_path_checks(arrays, active[0], active[-1], POLICY)
                values = torch.as_tensor(arrays['position'], device='cuda', dtype=torch.float32)
                mask = validity.mask(values).cpu().numpy()
                if phase in ('extract','transport') and planner.native_project_margins:
                    mask &= planner.native_margin_mask(checker,values).cpu().numpy()
                stage['native_project_margins_checked'] = bool(planner.native_project_margins and phase in ('approach','extract','transport'))
                stage['collision_invalid_frames'] = np.flatnonzero(~mask).tolist()
                geometry_valid = bool(mask.all())
                if loaded:
                    stage['payload_audit'] = planner.loaded_audit(checker, arrays['position'])
                    geometry_valid &= stage['payload_audit']['valid']
                    if phase=='transport' and (final_transport or (stage_context is not None and stage_context.final_joint_sides)):
                        radius=float(planner.full_envelope_radius(checker,arrays['position'][-1:]).item())
                        joint_carry = all(p.carry_joint_home for p in tasks.values())
                        stage['carry_envelope_final']=dict(radius_m=radius,limit_m=planner.carry_radius_limit,
                            enforced=not joint_carry, within_legacy_limit=radius<=planner.carry_radius_limit,
                            valid=joint_carry or radius<=planner.carry_radius_limit)
                        if joint_carry:
                            from curobo_core.layout_contract import carry_configuration
                            expected = carry_configuration(float(arrays['position'][-1, 0]))
                            error = max(abs(float(arrays['position'][-1, i])-expected[name])
                                        for i, name in enumerate(source['joint_names']) if name != 'updown'
                                        and (stage_context is None or any(name.startswith(side+'_') for side in stage_context.final_joint_sides)))
                            stage['carry_joint_endpoint'] = dict(max_error_rad=error, valid=error<=1e-5,
                                                                updown_m=float(arrays['position'][-1, 0]))
                            geometry_valid &= stage['carry_joint_endpoint']['valid']
                        geometry_valid &= stage['carry_envelope_final']['valid']
                if stage_context is not None and phase in ('contact', 'extract'):
                    line_sides = ((stage_context.incoming_side,) if phase == 'contact' else tuple(dict(stage_context.cartesian_displacements)))
                    line_attachments = {side:attachments[side] for side in line_sides}
                    actual_boxes = box_poses(checker, arrays['position'], line_attachments)
                    geometric_starts = box_poses(checker, active[:1], line_attachments)
                    checks = {}
                    for side in line_sides:
                        if phase == 'contact':
                            end_box = np.eye(4);end_box[:3, 3] = tasks[side].center_m
                            start_box = end_box.copy()
                            stand_off = source['precontact_distance_m']
                            if isinstance(stand_off, dict):stand_off = stand_off[side]
                            start_box[:3, 3] -= tasks[side].contact_tool()[:3, 2]*stand_off
                        else:
                            start_box = geometric_starts[side][0]
                            end_box = start_box.copy();end_box[:3, 3] += np.asarray(dict(stage_context.cartesian_displacements)[side])
                        checks[side] = cartesian_check(actual_boxes[side], start_box, end_box)
                    stage['cartesian_line_checks'] = checks
                    geometry_valid &= all(check['valid'] for check in checks.values())
                elif phase in ('contact', 'extract'):
                    all_boxes = box_poses(checker, arrays['position'], attachments)
                    checks = {}
                    for side, pick in tasks.items():
                        boxes = all_boxes[side]
                        direction = pick.contact_tool()[:3, 2] if phase == 'contact' else np.array([-1., 0., 0.])
                        delta = boxes[:, :3, 3]-pick.center_m
                        axial = delta @ direction
                        lateral = np.linalg.norm(delta-axial[:, None]*direction, axis=1)
                        angles = np.rad2deg(Rotation.from_matrix(boxes[:, :3, :3]).magnitude())
                        stand_off = source['precontact_distance_m']
                        if isinstance(stand_off, dict):stand_off = stand_off[side]
                        low, high = (-stand_off, 0.) if phase == 'contact' else (0., pick.extraction_distance_m)
                        line_valid = bool(lateral.max() <= .002 and angles.max() <= 1.
                                          and axial.min() >= low-.002 and axial.max() <= high+.002)
                        checks[side] = dict(valid=line_valid, max_lateral_error_m=float(lateral.max()),
                            max_orientation_error_deg=float(angles.max()), progress_min_m=float(axial.min()),
                            progress_max_m=float(axial.max()))
                        geometry_valid &= line_valid
                    stage['cartesian_line_checks'] = checks if len(tasks) > 1 else checks[task.side]
                torque, torque_ms = engine.dynamics.torque(arrays['position'], arrays['velocity'], arrays['acceleration'],
                    active_sides, payload_attachments)
                ratio = float(np.max(np.abs(torque[:, 1:]) / (POLICY['arm_continuous_torque_nm']*2)))
                difference = float(np.max(np.abs(torque[:, 1:]-arrays['torque'])))
                stage['independent_rnea'] = dict(peak_torque_ratio=ratio,
                    max_native_difference_nm=difference, valid=bool(ratio <= 1.001 and difference <= .01), compute_ms=torque_ms)
                seam = dict(phase=phase, valid=True)
                if previous is not None:
                    for field in ('position', 'velocity', 'acceleration'):
                        seam[field] = float(np.max(np.abs(arrays[field][0]-previous[field][-1])))
                    seam['valid'] = bool(seam['position'] <= 1e-5 and seam['velocity'] <= 1e-4 and seam['acceleration'] <= 1e-4)
                result['seam_checks'].append(seam)
                previous = arrays
                stage['geometry_valid'] = geometry_valid
                physical_success = bool(geometry_valid and stage['array_checks']['valid'] and native['dynamic_valid']
                    and stage['independent_rnea']['valid'] and seam['valid']
                    and stage.get('trajectory_optimization', {}).get('solver_success', True))
                stage['physical_success'] = physical_success
                if physical_success and phase == 'transport':
                    boxes = box_poses(checker, arrays['position'], payload_attachments)
                    observations = {side:source_slot_observation(tasks[side], boxes[side]) for side in active_sides}
                    stage['source_slot_observation'] = observations
                    source['process_validation'] = dict(status='unconfirmed', diagnostic_only=True,
                        source_slot_observation=observations)
                stage['success'] = physical_success
        except torch.OutOfMemoryError:
            raise
        except Exception as error:
            traceback.print_exc()
            stage['error'] = repr(error)
        stage['rejection_reasons'] = native_rejection_reasons(stage)
        stage['wall_seconds'] = time.perf_counter()-stage_started
        progress(dict(stage=phase, state='accepted' if stage['success'] else 'rejected',
            kind='native', wall_seconds=stage['wall_seconds'], details=stage))
        result['stages'].append(stage)
        save(output/'report.json', result)
        print('TWO_BOX_NATIVE', phase, stage.get('success'), stage.get('error', ''), flush=True)
        if stop_on_failure and not stage['success']:break
    complete_requested = len(result['stages']) == len(phase_names) and all('trajectory_file' in s for s in result['stages'])
    result['complete_candidate'] = phase_names==PHASES and complete_requested
    result['all_native_geometry_results_success'] = all(s.get('trajectory_optimization', {}).get('solver_success', True) for s in result['stages'])
    result['all_trajopt_success'] = all(s.get('trajectory_optimization', {}).get('optimizer_success', s.get('trajectory_optimization', {}).get('solver_success', True)) is True for s in result['stages'])
    result['success'] = len(result['stages']) == len(phase_names) and all(s['success'] for s in result['stages'])
    result['wall_seconds'] = time.perf_counter()-started
    if complete_requested:
        result['motion_seconds'] = sum(s['time_optimization']['duration_s'] for s in result['stages'])
    save(output/'report.json', result)
    return result
