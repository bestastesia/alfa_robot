"""gpuq only: native acceptance of the retained four-phase left-owned Y85 prefix."""
from dataclasses import replace
from pathlib import Path
import argparse
import hashlib
import importlib.metadata
import json
import os
import subprocess
import sys
import time
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
BASE = ROOT / 'generated/two_box_layout/joint_contract_20261123'
OUT = ROOT / 'generated/two_box_layout/joint_contract_20261124_prefix_native'
sys.path.insert(0, str(ROOT / 'tools'))
PHASES = ('approach', 'contact', 'extract', 'transport')
KEYS = ('approach', 'contact', 'extract', 'owner_transport')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def inputs():
    manifest = json.loads((BASE/'source_default_full31/manifest.json').read_text())
    mismatches = [p for p, h in manifest['files'].items() if sha(ROOT/p) != h]
    assert len(manifest['files']) == 284 and not mismatches, mismatches
    prior = json.loads((BASE/'transfer_owner_transport_probe.json').read_text())
    assert prior['success'] and prior['physical_source_box_id'] == 'right' and prior['actual_owner_arm'] == 'left'
    with np.load(BASE/'transfer_owner_transport_geometry.npz', allow_pickle=False) as data:
        geometry = {key: data[key].copy() for key in KEYS}
    for i, rows in enumerate(geometry.values()):
        assert rows.ndim == 2 and rows.shape[1] == 15 and np.isfinite(rows).all()
        if i: assert np.max(np.abs(rows[0]-geometry[KEYS[i-1]][-1])) <= 1e-5
    paths = [BASE/name for name in ('transfer_owner_transport_geometry.npz', 'transfer_owner_transport_probe.json',
             'probe_transfer_owner_transport.py', 'audit_transfer_owner_geometry.py', 'source_default_full31/manifest.json')]
    paths += list((ROOT/'generated/fast_runtime_assets').glob('*.yaml'))
    paths += list((ROOT/'generated/fast_runtime_assets').glob('*.urdf'))
    return prior, geometry, dict(existing_manifest_files=284, mismatches=mismatches,
        original_source_stamp=manifest['source_stamp'], source_files={str(p.relative_to(ROOT)): sha(p) for p in paths})


def prepared_assets():
    from curobo_core.contracts import PlannerAssets
    folder = ROOT/'generated/fast_runtime_assets'
    assets = replace(PlannerAssets(), robot_config=folder/'fixed.yaml', mobile_robot_config=folder/'mobile.yaml',
        urdf=folder/'fixed.urdf', named_poses=ROOT/'generated/zero_model_candidate_20261018/named_poses.yaml')
    assert all(p.is_file() for p in (assets.robot_config, assets.mobile_robot_config, assets.urdf, folder/'dynamics.yaml'))
    return assets, folder/'dynamics.yaml'


def gpu_state():
    return subprocess.run(['nvidia-smi', '--query-gpu=index,name,memory.used,utilization.gpu', '--format=csv,noheader'],
        capture_output=True, text=True, check=True).stdout.strip()


def run(output, report, initial_from=None):
    import curobo
    import torch
    import v3_two_box_pick
    import v3_fast_three_layers
    from curobo_core.box_pick import BoxPick, source_slot_process_checks, timed_path_checks
    from curobo_core.scene import Pose, SceneObject
    from curobo_core.native_transport import concatenate_native, cpu_source_observations, reference_box_poses, source_progress
    from v3_box_layout import session_source_stamp
    from v3_layout_native import time_initial_with_approach_recovery, time_pick
    from v3_wall_ik_benchmark import ACTIVE_JOINTS
    import yourdfpy

    prior, geometry, hashes = inputs()
    stamp = session_source_stamp()
    report.update(provenance=hashes, old_geometry_source_stamp=prior['source_stamp'], session_source_stamp=stamp,
        executable=sys.executable, curobo_file=curobo.__file__, curobo_version=importlib.metadata.version('nvidia-curobo'),
        torch_version=torch.__version__, gpu_start=gpu_state(), request=prior['request'],
        physical_source_box_id='right', actual_owner_arm='left', real_robot_execution=False,
        scope='Retained successful geometry tested as native prefix candidate; not fresh production full-cycle planning. '
              'Stops at unchanged right-receiver standoff; handover/contact/release and final 14-angle return remain incomplete.')
    v3_two_box_pick.prepare_fast_assets = prepared_assets
    v3_fast_three_layers.prepare_fast_assets = prepared_assets
    v3_fast_three_layers.OUTPUT = output
    original = {b['side']: b for b in prior['request']['boxes']}
    owner = BoxPick(**dict(original['right'], side='left'))
    pending = BoxPick(**original['left'])
    planner = v3_two_box_pick.TwoBoxPickPlanner()
    planner.configure_pick(owner)
    planner.picks = {'left': owner}
    planner.native_project_margins = True
    planner.full_payload_trajopt = True
    planner.snapshot = replace(planner.snapshot, objects=planner.snapshot.objects + (
        SceneObject('pending_original_left', pending.dimensions_m, Pose(pending.center_m)),))
    report['physical_snapshot'] = planner.snapshot.to_dict()
    scenes = {}
    for loaded in (False, True):
        checker = planner.checker(planner.selected_task, payload=loaded)
        assert any(o.name == 'pending_original_left' for o in checker.guidance_scene.cuboid)
        assert bool(checker.guidance_payload) == loaded
        assert set(checker.task_tool_to_box) == {'left'}
        sphere_ids = checker.kinematics.config.kinematics_config.get_sphere_index_from_link_name('left_link7')
        appended = len(sphere_ids)-len(planner.robot['kinematics']['collision_spheres']['left_link7'])
        assert (appended > 0) == loaded
        scenes['loaded' if loaded else 'empty'] = dict(world_cuboids=[o.name for o in checker.guidance_scene.cuboid],
            nominal_attachment_sides=list(checker.task_tool_to_box), physical_payload=loaded, appended_payload_spheres=appended)
    report['phase_scene_checks'] = scenes
    valid = planner.target_validity(planner.checker(planner.selected_task), allow_contact=True)
    source = dict(geometry_success=True, geometry_origin='retained NPZ, not fresh production full-cycle planning',
        frames=np.concatenate(list(geometry.values())).tolist(),
        phases=[phase for phase, key in zip(PHASES, KEYS) for _ in geometry[key]],
        joint_names=list(ACTIVE_JOINTS), task=planner.selected_task,
        precontact_distance_m=owner.precontact_distance(valid.targets['left']['tool_cap_reach_m']),
        source_sha256=dict(hashes['source_files']))
    engine = v3_fast_three_layers.FastThreeLayerPlanner(planner=planner)
    save(output/'source.json', source)

    def progress(event):
        if event.get('state') != 'running':
            detail = event.get('details', {})
            print('PREFIX', event.get('stage'), event.get('piece', ''), event.get('state'),
                detail.get('rejection_reasons', event.get('reason')), flush=True)

    if initial_from:
        initial_file = OUT/initial_from/'initial/report.json'
        initial = json.loads(initial_file.read_text())
        assert initial['success'] and [s['phase'] for s in initial['stages']] == list(PHASES[:3])
        report['reused_initial'] = {str(p.relative_to(ROOT)): sha(p) for p in [initial_file,
            *[Path(s['trajectory_file']) for s in initial['stages']]]}
    else:
        initial = time_initial_with_approach_recovery(engine, source, output/'initial',
            stop_on_failure=True, progress=progress, phase_names=PHASES[:3])
    report['initial'] = initial
    save(output/'prefix_report.json', report)
    if not initial['success']: return
    robot = yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
    from v3_box_layout_planner import BoxLayoutPlanner
    planner.source_slot_depths = BoxLayoutPlanner.source_slot_depths.__get__(planner)
    loaded = planner.checker(planner.selected_task, payload=True)
    slots = BoxLayoutPlanner.transport_source_slots(planner, loaded, geometry['owner_transport'][0])
    # Fixed original source reference across every native rest piece.
    planner.transport_source_slots = lambda checker, start: slots
    report['original_source_slot_guidance'] = {k:[np.asarray(v[0]).tolist(),np.asarray(v[1]).tolist(),v[2]] for k,v in slots.items()}
    rows = geometry['owner_transport']
    observations = cpu_source_observations(robot, rows[:1], ACTIVE_JOINTS, {'left': owner})
    reference = {k:v['initial_slot_overlap_m'] for k,v in observations.items()}
    cleared = {k:v['first_clear_frame'] is not None for k,v in observations.items()}
    pending = [(0,len(rows)-1,'whole')]
    accepted, rejected, pieces = [], [], []
    while pending:
        a,b,name = pending.pop(0)
        request = dict(source, frames=rows[a:b+1].tolist(), phases=['transport']*(b-a+1))
        result = time_pick(engine, request, output/('transport_'+name), stop_on_failure=True,
            progress=progress, phase_names=('transport',), final_transport=False)
        stage = result['stages'][0]
        record = dict(name=name, start=a, end=b, native=result)
        valid_piece = result['success']
        if 'trajectory_file' in stage:
            with np.load(stage['trajectory_file'], allow_pickle=False) as f:
                data = {key:f[key].copy() for key in f.files}
            obs = cpu_source_observations(robot, data['position'], ACTIVE_JOINTS, {'left': owner})
            valid_history, _, next_clear, checks = source_progress(obs, reference, cleared)
            record.update(source_observations=obs, original_source_history_checks=checks)
            valid_piece = bool(valid_piece and valid_history)
        record['accepted'] = valid_piece
        if valid_piece:
            accepted.append(record); pieces.append(data); cleared = next_clear
        else:
            rejected.append(record)
            print('PREFIX_REFINE', name, stage['rejection_reasons'], record.get('original_source_history_checks'), flush=True)
            # ponytail: bounded to eight rest pieces; exhaustion is unresolved native representation, not task illegality.
            if b-a <= 1 or len(pending)+len(accepted)+2 > 8:
                report.update(transport_rejected=rejected, transport_pieces=accepted,
                    failure='bounded native transport refinement exhausted')
                return
            mid = (a+b)//2
            pending[:0] = [(a,mid,name+'a'),(mid,b,name+'b')]
        report.update(transport_rejected=rejected, transport_pieces=accepted)
        save(output/'prefix_report.json', report)
    transport_arrays, internal_seams = concatenate_native(pieces)
    transport_file = output/'transport_native.npz'
    np.savez_compressed(transport_file, **transport_arrays)
    transport_stages = [r['native']['stages'][0] for r in accepted]
    transport = dict(success=True, stages=[dict(phase='transport', success=True, trajectory_file=str(transport_file),
        pieces=transport_stages, seams=internal_seams)], all_native_geometry_results_success=all(
        r['native']['all_native_geometry_results_success'] for r in accepted),
        all_trajopt_success=all(r['native']['all_trajopt_success'] for r in accepted))
    report['transport'] = transport
    stages = initial['stages'] + transport['stages']
    arrays = []
    for stage in stages:
        with np.load(stage['trajectory_file'], allow_pickle=False) as data:
            arrays.append({key: data[key].copy() for key in data.files})
    merged, seams = concatenate_native(arrays)
    np.savez_compressed(output/'prefix_native.npz', **merged)
    robot = yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
    attachment = {'left': owner.tool_to_box()}
    contact_box = reference_box_poses(robot, arrays[1]['position'][-1], ACTIVE_JOINTS, attachment)['left']
    extract_start = reference_box_poses(robot, arrays[2]['position'][0], ACTIVE_JOINTS, attachment)['left']
    extract_end = reference_box_poses(robot, arrays[2]['position'][-1], ACTIVE_JOINTS, attachment)['left']
    end_box = reference_box_poses(robot, merged['position'][-1], ACTIVE_JOINTS, attachment)['left']
    pull = extract_end[:3, 3]-extract_start[:3, 3]
    observations = cpu_source_observations(robot, arrays[3]['position'], ACTIVE_JOINTS, {'left': owner})
    process = source_slot_process_checks(observations)
    checks = timed_path_checks(merged, geometry['approach'][0], geometry['owner_transport'][-1], v3_fast_three_layers.POLICY)
    physical = dict(source_absorption_error_m=float(np.linalg.norm(contact_box[:3,3]-owner.center_m)),
        extract_start_source_error_m=float(np.linalg.norm(extract_start[:3,3]-owner.center_m)),
        actual_extraction_world_m=pull.tolist(), extraction_vector_error_m=float(np.linalg.norm(pull-[-.255,0.,0.])),
        standoff_center_error_m=float(np.linalg.norm(end_box[:3,3]-prior['transfer_center_m'])),
        original_endpoint_joint_error=float(np.max(np.abs(merged['position'][-1]-geometry['owner_transport'][-1]))),
        attachment=owner.tool_to_box().tolist(), source_process=process)
    physical['valid'] = bool(max(physical[k] for k in ('source_absorption_error_m', 'extract_start_source_error_m',
        'extraction_vector_error_m', 'standoff_center_error_m')) <= .002 and physical['original_endpoint_joint_error'] <= 1e-5 and process['valid'])
    report.update(success=bool(checks['valid'] and physical['valid'] and all(s['success'] for s in stages)),
        motion_seconds=float(merged['times'][-1]), native_samples=len(merged['times']),
        stage_samples={s['phase']: len(a['times']) for s,a in zip(stages,arrays)}, seams=seams,
        array_checks=checks, physical_checks=physical,
        all_native_geometry_results_success=initial['all_native_geometry_results_success'] and transport['all_native_geometry_results_success'],
        all_trajopt_success=initial['all_trajopt_success'] and transport['all_trajopt_success'])
    report['source_unchanged_during_workload'] = session_source_stamp() == stamp
    assert inputs()[2] == hashes, 'retained inputs or immutable prepared assets changed'


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--attempt', default='attempt_01')
    parser.add_argument('--initial-from')
    parser.add_argument('--check-inputs', action='store_true')
    args = parser.parse_args()
    if args.check_inputs:
        prior, geometry, hashes = inputs()
        print(json.dumps(dict(manifest=hashes['existing_manifest_files'], mismatches=hashes['mismatches'],
            geometry_shapes={k:list(v.shape) for k,v in geometry.items()})))
    else:
        assert Path(args.attempt).name == args.attempt
        output = OUT/args.attempt
        output.mkdir(parents=True, exist_ok=False)
        report = dict(success=False)
        began = time.perf_counter()
        try:
            run(output, report, args.initial_from)
        except Exception as error:
            report.update(error=repr(error), traceback=traceback.format_exc())
            traceback.print_exc()
        finally:
            report['wall_seconds'] = time.perf_counter()-began
            report['gpu_before_process_exit'] = gpu_state()
            save(output/'prefix_report.json', report)
            print('PREFIX_RESULT', report['success'], 'wall', report['wall_seconds'], 'motion', report.get('motion_seconds'), flush=True)
        sys.exit(0 if report['success'] else 1)
