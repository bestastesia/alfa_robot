"""Independent CPU checks of accepted native arrays; no GPU import or output repair."""
from pathlib import Path
import json
import os
import sqlite3
import sys

import numpy as np
import yourdfpy
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from probe_prefix import ROOT, OUT, inputs, save, sha
from curobo_core.box_pick import BoxPick,timed_path_checks,source_slot_process_checks
from curobo_core.native_transport import concatenate_native,cpu_source_observations,reference_box_poses


if __name__=='__main__':
    output=OUT/'numeric_fix/attempt_02'
    prefix=output/'prefix'
    report=json.loads((output/'numeric_fix_report.json').read_text())
    native=json.loads((prefix/'prefix_report.json').read_text())
    prior,geometry,hashes=inputs()
    assert report['success'] and native['success'] and report['shared_library_unchanged']
    before=json.loads((output/'shared_library_before.json').read_text())
    after=json.loads((output/'shared_library_after.json').read_text())
    assert before==after
    source=Path(report['curobo_file']).parent
    assert all(sha(source/key)==digest for key,digest in before.items())
    tree=output/'kernels'
    changes=[str(p.relative_to(tree)) for p in tree.rglob('*') if p.is_file()
        and sha(p)!=sha(source/'_src/curobolib/kernels'/p.relative_to(tree))]
    assert changes==['trajectory/bspline/bspline_context.cuh'],changes
    stages=native['initial']['stages']+[r['native']['stages'][0] for r in native['transport_pieces']]
    def load(path):
        with np.load(path,allow_pickle=False) as data:return {k:data[k].copy() for k in data.files}
    pieces=[load(s['trajectory_file']) for s in stages]
    transport,internal=concatenate_native(pieces[3:])
    merged,seams=concatenate_native(pieces[:3]+[transport])
    saved=load(prefix/'prefix_native.npz')
    assert all(np.array_equal(merged[k],saved[k]) for k in saved), 'saved arrays differ from native pieces'
    robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    names=['updown']+[f'{s}_joint{i}' for s in ('left','right') for i in range(1,8)]
    for i,name in enumerate(names):
        limit=robot.joint_map[name].limit
        assert np.all((saved['position'][:,i]>=limit.lower)&(saved['position'][:,i]<=limit.upper)),name
    policy=json.loads((ROOT/'config/fast_three_layers.json').read_text())
    array_checks=timed_path_checks(saved,geometry['approach'][0],geometry['owner_transport'][-1],policy)
    assert array_checks['valid']
    owner=BoxPick(**dict(next(b for b in prior['request']['boxes'] if b['side']=='right'),side='left'))
    def box(q):return reference_box_poses(robot,q,names,{'left':owner.tool_to_box()})['left']
    contact=box(pieces[1]['position'][-1]);first=box(pieces[2]['position'][0]);last=box(pieces[2]['position'][-1]);end=box(saved['position'][-1])
    process=source_slot_process_checks(cpu_source_observations(robot,transport['position'],names,{'left':owner}))
    fk=dict(absorption_error_m=float(np.linalg.norm(contact[:3,3]-owner.center_m)),
        extract_start_source_error_m=float(np.linalg.norm(first[:3,3]-owner.center_m)),
        extraction_vector_m=(last[:3,3]-first[:3,3]).tolist(),
        extraction_vector_error_m=float(np.linalg.norm(last[:3,3]-first[:3,3]-[-.255,0,0])),
        standoff_center_error_m=float(np.linalg.norm(end[:3,3]-prior['transfer_center_m'])))
    assert max(fk[k] for k in fk if k.endswith('error_m'))<=.002 and process['valid']
    assert all(s['success'] and s['independent_rnea']['valid'] for s in stages)
    assert all(s['payload_audit']['valid'] for s in stages[2:])
    assert all(s['valid'] for s in seams+internal)
    baseline=json.loads((OUT/'acceptance.json').read_text())
    probe=Path(__file__).resolve().parent.parent/'probe_prefix.py'
    assert baseline['experiment_script_sha256'][str(probe.relative_to(ROOT))]==sha(probe)
    original_context='_src/curobolib/kernels/trajectory/bspline/bspline_context.cuh'
    important=[original_context,'_src/curobolib/backends/cuda_core_backend/trajectory_config.py',
        '_src/curobolib/backends/cuda_core_backend/trajectory.py','_src/curobolib/backends/cuda_core_backend/trajectory_timeopt.py',
        '_src/trajectory/timeopt/timeopt_solver.py','_src/motion/motion_trajectory_timeopt.py']
    artifacts=[prefix/'prefix_native.npz',prefix/'prefix_report.json',output/'numeric_fix_report.json',output/'bspline_context.patch',
        *[Path(s['trajectory_file']) for s in stages]]
    with sqlite3.connect('file:/home/astesia/.local/state/gpu-session-monitor/monitor.sqlite3?mode=ro',uri=True) as db:
        db.row_factory=sqlite3.Row
        run=dict(db.execute("SELECT id,pid,state,started,ended,exit_code FROM runs WHERE name=? ORDER BY submitted DESC LIMIT 1",
            ('Y85局部样条数值修复完整复验-provisional',)).fetchone())
    assert run['state']=='completed' and run['exit_code']==0
    result=dict(success=True,physical_source_box_id='right',owner='left',payload_kg=10.,
        native_samples=len(saved['times']),stage_samples=native['stage_samples'],motion_seconds=native['motion_seconds'],
        full_replan_wall_seconds=report['prefix_wall_seconds'],workload_wall_seconds=report['wall_seconds'],
        gpuq_lock_wall_seconds=run['ended']-run['started'],gpuq_completed_run=run,
        same_geometry_and_request_provenance=hashes,same_native_probe_code_sha256=sha(probe),
        shared_library_files_unchanged=len(before),key_shared_library_sha256={p:before[p] for p in important},
        changed_isolated_kernel_files=changes,patched_context_sha256=report['patched_context_sha256'],
        compiled_trajectory_sources=report['compiled_trajectory_sources'],
        exact_native_array_concatenation=True,all_sample_joint_limits_valid=True,updown_min_m=float(saved['position'][:,0].min()),
        array_checks=array_checks,fk=fk,source_process=process,seams=seams,internal_transport_seams=internal,
        all_native_geometry_optimizer_success=native['all_trajopt_success'],all_independent_rnea_valid=True,
        maximum_rnea_difference_nm=max(s['independent_rnea']['max_native_difference_nm'] for s in stages),
        payload_obb_audits=[s['payload_audit'] for s in stages[2:]],numeric_regression=report['patched_regression'],
        artifact_sha256={str(p.relative_to(ROOT)):sha(p) for p in artifacts},
        scope='Accepted retained-geometry native prefix under isolated numeric fix; no handover or final 14-angle return acceptance.')
    save(output/'acceptance.json',result)
    print(json.dumps({k:result[k] for k in ('success','native_samples','motion_seconds','workload_wall_seconds',
        'shared_library_files_unchanged','updown_min_m','array_checks','fk')},ensure_ascii=False))
