"""CPU-only audit and unchanged-array assembly of the smallest two-piece candidate."""
from pathlib import Path
import json
import os
import sqlite3
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy
from probe_prefix import ROOT, OUT, PHASES, KEYS, inputs, save, sha
from curobo_core.box_pick import BoxPick, timed_path_checks, source_slot_process_checks
from curobo_core.native_transport import concatenate_native, cpu_source_observations, reference_box_poses


def load(path):
    with np.load(path,allow_pickle=False) as data:
        return {key:data[key].copy() for key in data.files}


def main():
    prior, geometry, provenance = inputs()
    initial = json.loads((OUT/'attempt_02/initial/report.json').read_text())
    transport_stages = [json.loads((OUT/'attempt_03'/('transport_'+name)/'report.json').read_text())['stages'][0]
                        for name in ('wholea','wholeb')]
    stages = initial['stages']+transport_stages
    pieces = [load(stage['trajectory_file']) for stage in stages]
    transport, transport_seams = concatenate_native(pieces[3:])
    arrays = pieces[:3]+[transport]
    merged, seams = concatenate_native(arrays)
    output = OUT/'review_candidate'
    output.mkdir(exist_ok=True)
    # Rebase clocks and remove duplicate seam rows only; q/v/a/jerk/torque samples are unchanged.
    np.savez_compressed(output/'transport.npz',**transport)
    np.savez_compressed(output/'prefix_native_candidate.npz',**merged)
    names = ['updown']+[f'{s}_joint{i}' for s in ('left','right') for i in range(1,8)]
    robot = yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    owner = BoxPick(**dict(next(b for b in prior['request']['boxes'] if b['side']=='right'),side='left'))
    attachment = {'left':owner.tool_to_box()}
    def box(row):return reference_box_poses(robot,row,names,attachment)['left']
    contact, extract_start, extract_end, end = [box(row) for row in (
        arrays[1]['position'][-1],arrays[2]['position'][0],arrays[2]['position'][-1],arrays[3]['position'][-1])]
    pull = extract_end[:3,3]-extract_start[:3,3]
    obs = cpu_source_observations(robot,transport['position'],names,{'left':owner})
    process = source_slot_process_checks(obs)
    physical = dict(source_absorption_error_m=float(np.linalg.norm(contact[:3,3]-owner.center_m)),
        source_absorption_angle_deg=float(np.rad2deg(Rotation.from_matrix(contact[:3,:3]).magnitude())),
        extract_start_source_error_m=float(np.linalg.norm(extract_start[:3,3]-owner.center_m)),
        actual_extraction_world_m=pull.tolist(),extraction_vector_error_m=float(np.linalg.norm(pull-[-.255,0.,0.])),
        standoff_center_error_m=float(np.linalg.norm(end[:3,3]-prior['transfer_center_m'])),
        endpoint_joint_error=float(np.max(np.abs(transport['position'][-1]-geometry['owner_transport'][-1]))),
        tool_to_box=owner.tool_to_box().tolist(),source_process=process)
    assert max(physical[k] for k in ('source_absorption_error_m','extract_start_source_error_m',
        'extraction_vector_error_m','standoff_center_error_m'))<=.002
    assert physical['source_absorption_angle_deg']<=1 and physical['endpoint_joint_error']<=1e-5 and process['valid']
    policy = json.loads((ROOT/'config/fast_three_layers.json').read_text())
    checks = timed_path_checks(merged,geometry['approach'][0],geometry['owner_transport'][-1],policy)
    assert checks['valid'] and all(s['valid'] for s in seams+transport_seams)
    bound_failures = []
    for i,name in enumerate(names):
        limit = robot.joint_map[name].limit
        bad = np.flatnonzero((merged['position'][:,i]<limit.lower)|(merged['position'][:,i]>limit.upper))
        if len(bad):bound_failures.append(dict(joint=name,lower=limit.lower,upper=limit.upper,
            frames=bad.tolist(),values=merged['position'][bad,i].tolist()))
    assert len(bound_failures)==1 and bound_failures[0]['joint']=='updown' and len(bound_failures[0]['frames'])==1
    files = [Path(s['trajectory_file']) for s in stages]
    reports = [OUT/'attempt_02/initial/report.json',*[OUT/'attempt_03'/('transport_'+n)/'report.json' for n in ('wholea','wholeb')]]
    artifact_hashes = {str(p.relative_to(ROOT)):sha(p) for p in [*files,*reports,output/'transport.npz',output/'prefix_native_candidate.npz']}
    terminal = {}
    with sqlite3.connect('file:/home/astesia/.local/state/gpu-session-monitor/monitor.sqlite3?mode=ro',uri=True) as db:
        db.row_factory = sqlite3.Row
        terminal['queue_runs'] = [dict(r) for r in db.execute("SELECT id,pid,name,state,started,ended,exit_code,estimate_seconds FROM runs WHERE name IN (?,?,?,?) ORDER BY submitted",
            ('Y85左取高箱原生四段前缀首轮','Y85四段原生前缀修正状态断言','Y85前缀源位guidance原生有界细分','Y85原终点边沿逐项门禁诊断'))]
        terminal['monitor_sample'] = dict(db.execute('SELECT * FROM samples ORDER BY ts DESC LIMIT 1').fetchone())
    inode = str(os.stat('/tmp/sevenova-curobo-gpu.lock').st_ino)
    terminal['shared_lock_records'] = [line for line in Path('/proc/locks').read_text().splitlines()
        if any(token.endswith(':'+inode) for token in line.split())]
    terminal['this_experiment_active'] = any(r['state'] in ('running','queued') for r in terminal['queue_runs'])
    assert not terminal['this_experiment_active']
    diag = json.loads((OUT/'endpoint_component_diagnosis.json').read_text())
    trials = [json.loads((OUT/f'attempt_{i:02d}/prefix_report.json').read_text()) for i in (1,2,3)]
    report = dict(success=False, physical_source_box_id='right',actual_owner_arm='left',payload_kg=10.,
        payload_dimensions_m=list(owner.dimensions_m),request=prior['request'],provenance=provenance,
        cpu_audit_executable=sys.executable,gpu_executable=trials[2]['executable'],curobo_file=trials[2]['curobo_file'],
        curobo_version=trials[2]['curobo_version'],scope='Native prefix candidate only; strict sample bound rejects it. '
        'No data repair, time scaling, endpoint change, contact/handover/release, or final 14-angle return acceptance.',
        native_samples=len(merged['times']),stage_samples={p:len(a['times']) for p,a in zip(PHASES,arrays)},
        motion_seconds=float(merged['times'][-1]),stage_motion_seconds={p:float(a['times'][-1]) for p,a in zip(PHASES,arrays)},
        native_pieces=5,transport_native_pieces=2,phase_arrays={p:str(f.relative_to(ROOT)) for p,f in zip(PHASES,
            [Path(s['trajectory_file']) for s in stages[:3]]+[output/'transport.npz'])},
        native_artifact_sha256=artifact_hashes,
        experiment_script_sha256={str(p.relative_to(ROOT)):sha(p) for p in Path(__file__).parent.glob('*.py')},
        endpoint_diagnosis_sha256=sha(OUT/'endpoint_component_diagnosis.json'),
        array_checks=checks,physical_checks=physical,seams=seams,
        internal_transport_seams=transport_seams,joint_bound_failures=bound_failures,
        failed_gate='Strict joint bound at one actual native sample: updown=-1.0000001192092896 < -1.0m.',
        original_endpoint_and_last_edge_gpu_valid=diag['original_endpoint_and_last_edge_valid'],
        all_native_trajopt_accepted=all(s.get('trajectory_optimization',{}).get('optimizer_success',True) for s in stages),
        all_timeopt_success=all(s['time_optimization']['success'] for s in stages),
        all_independent_rnea_valid=all(s['independent_rnea']['valid'] for s in stages),
        max_rnea_difference_nm=max(s['independent_rnea']['max_native_difference_nm'] for s in stages),
        payload_obb_audits=[s['payload_audit'] for s in stages if 'payload_audit' in s],
        phase_scene_checks=trials[2]['phase_scene_checks'],selected_native_stage_wall_seconds=sum(s['wall_seconds'] for s in stages),
        trial_wall_seconds=[r['wall_seconds'] for r in trials],endpoint_diagnosis_wall_seconds=diag['wall_seconds'],
        all_experiment_internal_wall_seconds=sum(r['wall_seconds'] for r in trials)+diag['wall_seconds'],
        gpu_terminal=terminal,audited_at_unix=time.time())
    save(OUT/'acceptance.json',report)
    print(json.dumps({k:report[k] for k in ('success','native_samples','motion_seconds','stage_samples','failed_gate',
        'original_endpoint_and_last_edge_gpu_valid','all_independent_rnea_valid','array_checks')},ensure_ascii=False))


if __name__ == '__main__':main()
