"""Prepare on CPU; --run only after explicit GO, inside one foreground gpuq for the entire batch."""
from collections import OrderedDict
from contextlib import redirect_stdout,redirect_stderr
from pathlib import Path
from types import SimpleNamespace
import argparse
import copy
import difflib
import hashlib
import importlib.metadata
import importlib.util
import json
import os
import re
import shutil
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[4]
REPO=ROOT.parents[1]
ORIGINAL_OUT=ROOT/'generated/two_box_layout/default_handover_regression_20261124'
FINAL_OUT=ROOT/'generated/two_box_layout/default_handover_final35_20261124'
FINAL36_OUT=ROOT/'generated/two_box_layout/default_handover_final36_20261124'
SKU_RAW=ROOT/'generated/two_box_layout/sku_order_handover_20261124/raw_request.json'
NEGATIVE=ROOT/'generated/two_box_layout/role_generalization_20261124/raw_request.json'
OUT=ORIGINAL_OUT
SUITE='original34'
OLD=ROOT/'generated/two_box_layout/joint_contract_20261123'
TOP=ROOT/'generated/two_box_layout/joint_contract_20261124_top_handover'
CASE_COUNT=34
TOP_REQUESTS=('y85_top70_high130','y85_both_top20_high70')
RUNTIME_CACHE=ROOT/'generated/default_handover_runtime_cache_20261124'
HARNESS_FILES=tuple(str((Path(__file__).parent/name).resolve()) for name in ('regression.py','audit_batch.py'))
sys.path.insert(0,str(ROOT/'tools'))


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def document_sha(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def save(path,value):
    path=Path(path);temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n');temporary.replace(path)


def select_suite(name):
    global OUT,CASE_COUNT,SUITE
    if name not in ('original34','final35','final36'):raise ValueError('unsupported regression suite')
    SUITE=name;OUT=FINAL36_OUT if name=='final36' else FINAL_OUT if name=='final35' else ORIGINAL_OUT;CASE_COUNT=36 if name=='final36' else 35 if name=='final35' else 34


def catalog():
    if SUITE=='final36':
        prior=json.loads((FINAL_OUT/'catalog.json').read_text())['cases']
        if len(prior)!=35:raise ValueError('prior35 raw catalog incomplete')
        return [dict(name=c['name'],request=c['request']) for c in prior]+[dict(name='positive_y85_cube_upper_tall_lower',request=json.loads(SKU_RAW.read_text()))]
    if SUITE=='final35':
        prior=json.loads((ORIGINAL_OUT/'catalog.json').read_text())['cases']
        if len(prior)!=34:raise ValueError('original34 catalog incomplete')
        return [dict(name=r['name'],request=r['request']) for r in prior]+[dict(name='negative_y85_actual_roles',request=json.loads(NEGATIVE.read_text()))]
    source=json.loads((OLD/'default_full31/catalog.json').read_text())['cases']
    if len(source)!=31:raise ValueError('expected all original31 cases without filtering')
    y85=json.loads((OLD/'broader_same_side_y85_request.json').read_text())
    return source+[dict(name='broader_same_side_y85',request=y85)]+[
        dict(name=name,request=json.loads((TOP/(name+'_request.json')).read_text())) for name in TOP_REQUESTS]


def prepare():
    from v3_box_layout import parse_layout
    from curobo_core.layout_contract import validate_pair
    OUT.mkdir(parents=True,exist_ok=True);(OUT/'requests').mkdir(exist_ok=True)
    cases=[]
    for index,case in enumerate(catalog(),1):
        name=case['name']
        if not re.fullmatch('[a-z][a-z0-9_]*',name):raise ValueError('unsafe catalog case name')
        document=case['request'];picks=parse_layout(document);contract=validate_pair(picks)
        if abs(contract['front_distance_m']-.66)>1e-9 or any(p.side_min_height_m!=.79 for p in picks.values()):
            raise ValueError('original catalog is not the requested global66/79 regression')
        path=OUT/'requests'/f'{index:02d}_{name}.json'
        if path.exists() and json.loads(path.read_text())!=document:raise ValueError('prepared original request changed')
        if not path.exists():save(path,document)
        if json.loads(path.read_text())!=document:raise ValueError('request serialization changed values')
        cases.append(dict(index=index,name=name,request=document,request_file=str(path),request_sha256=sha(path),
            document_sha256=document_sha(document),origin='default_full31' if index<=31 else 'broader_same_side_y85' if index==32 else 'original_top_handover'))
        if index>=33:
            original=TOP/(TOP_REQUESTS[index-33]+'_request.json')
            cases[-1].update(source_request_file=str(original),source_request_sha256=sha(original))
            cases[-1]['prior_observation']=(dict(fresh13_native_success=True,worker_wall_seconds=132.28) if index==33 else
                dict(fresh13_native_success=True,worker_wall_seconds=386.1306,native_samples=60011,motion_seconds=39.639646590,
                     previous_bounded_failure='fixed-J7 local exit at4mm; resolved by actual all7-axis/admission work on the unchanged input',
                     input_illegality_established=False,global_unreachable_established=False))
            cases[-1]['prior_observation']['scope']='User update only; not a regression result, hint or expected-failure exemption'
    if len({r['name'] for r in cases})!=CASE_COUNT:raise ValueError('duplicate or missing cases')
    result=dict(cases=cases,cases_count=CASE_COUNT,entry='v3_box_layout.solve_file',max_contacts=16,
        original_catalog_sha256=sha(OLD/'default_full31/catalog.json'),
        original_y85_request_sha256=sha(OLD/'broader_same_side_y85_request.json'),
        scope='Finite original31 plus raw Y85 plus two unmodified Top requests; prior successful and failed inputs both retained; no face/offset/q/workflow hints, no universal claim.')
    save(OUT/'catalog.json',result)
    script=Path(__file__).resolve()
    command=f'''cd {REPO}
gpuq --name "default34 current-default regression provisional" --estimate 44m -- env \\
  MAKEFLAGS='-j8 -l12' CMAKE_BUILD_PARALLEL_LEVEL=8 MAX_JOBS=8 \\
  TORCHINDUCTOR_COMPILE_THREADS=8 PYTHONDONTWRITEBYTECODE=1 \\
  WARP_CACHE_PATH={RUNTIME_CACHE}/warp \\
  TORCHINDUCTOR_CACHE_DIR={RUNTIME_CACHE}/inductor \\
  TRITON_CACHE_DIR={RUNTIME_CACHE}/triton \\
  CUDA_CACHE_PATH={RUNTIME_CACHE}/cuda \\
  TORCH_EXTENSIONS_DIR={RUNTIME_CACHE}/torch_extensions \\
  /home/astesia/Sevenova/curobo_v2_fast/.venv/bin/python \\
  {script} --run --run-id run_001 \\
  > {OUT}/gpuq_run_001.log 2>&1
'''
    (OUT/'launch_command.txt').write_text(command)
    save(OUT/'preparation.json',dict(prepared=True,cases=CASE_COUNT,gpu_submitted=False,gpu_initialized=False,
        awaiting_explicit_go=True,source_frozen=False,freeze_policy='Freeze current tools/config/tests/models/vendor and public patch at authorized batch start, not the old284 manifest.',
        estimate='44m provisional; measured31=1348s +side149s +Top132s +BothTop386s,30% cold/admission margin=~43.7m; estimate is not timeout',
        script_sha256=sha(script),catalog_sha256=sha(OUT/'catalog.json')))
    if 'torch' in sys.modules or 'curobo' in sys.modules:raise RuntimeError('CPU preparation imported GPU library')
    return result


def prepare_final35():
    """CPU-only current-suite preparation; final source freeze intentionally waits for explicit GO."""
    from v3_box_layout import parse_layout
    from curobo_core.layout_contract import validate_pair
    OUT.mkdir(parents=True,exist_ok=True);(OUT/'requests').mkdir(exist_ok=True)
    old=json.loads((ORIGINAL_OUT/'catalog.json').read_text())
    cases=[]
    for index,row in enumerate(catalog(),1):
        document=row['request'];picks=parse_layout(document);contract=validate_pair(picks)
        if abs(contract['front_distance_m']-.66)>1e-9 or any(p.side_min_height_m!=.79 for p in picks.values()):
            raise ValueError('final35 raw global66/79 contract changed')
        source=Path(old['cases'][index-1]['request_file']) if index<=34 else NEGATIVE
        if json.loads(source.read_text())!=document:raise ValueError('source request differs from raw catalog')
        target=OUT/'requests'/f"{index:02d}_{row['name']}.json"
        if target.exists() and target.read_bytes()!=source.read_bytes():raise ValueError('prepared raw request differs')
        if not target.exists():target.write_bytes(source.read_bytes())
        case=dict(index=index,name=row['name'],request=document,request_file=str(target),request_sha256=sha(target),
            document_sha256=document_sha(document),source_request_file=str(source),source_request_sha256=sha(source))
        cases.append(case)
    if len(cases)!=35 or len({c['name'] for c in cases})!=35:raise ValueError('missing/reordered final35 cases')
    equality=[]
    for previous,current in zip(old['cases'],cases[:34]):
        same=all(previous[k]==current[k] for k in ('index','name','request','request_sha256','document_sha256'))
        if not same:raise ValueError('previous34 request changed')
        equality.append(dict(index=current['index'],name=current['name'],equal=True,document_sha256=current['document_sha256'],
            original_request_sha256=previous['request_sha256'],new_request_sha256=current['request_sha256']))
    suite=dict(suite='final35',cases_count=35,cases=cases,entry='v3_box_layout.solve_file',max_contacts=16,
        old34_catalog_sha256=sha(ORIGINAL_OUT/'catalog.json'),negative_raw_sha256=sha(NEGATIVE),
        scope='Fresh full current-source35 finite regression, no inherited old31 results, no face/flow/q hints, no universal claim.')
    save(OUT/'catalog.json',suite)
    save(OUT/'old34_request_equality.json',dict(valid=True,rows=equality,old_catalog=str(ORIGINAL_OUT/'catalog.json')))
    smoke=ROOT/'generated/two_box_layout/role_generalization_20261124/default_admission_attempt_07/default/result.json'
    checked=inspect_result(cases[-1],smoke)
    checked['scope']='Existing real main negative-Y default result reopened under current CPU dispatcher; smoke only, NOT final35 GPU evidence.'
    save(OUT/'negative_default_cpu_smoke.json',checked)
    original_proof=json.loads((OUT/'old34_immutable_before.json').read_text())
    if any(sha(p)!=h for p,h in original_proof.items()):raise ValueError('old34 artifacts changed')
    save(OUT/'preparation.json',dict(prepared=True,cases=35,gpu_submitted=False,gpu_initialized=False,
        source_frozen=False,awaiting_explicit_go=True,hold_reason='Cube upper/Tall lower original-request coverage pending; append as36 regardless success/failure before final source freeze',
        no_prior_results_inherited=True,old34_unchanged=True,negative_raw_unchanged=True,
        proposed_estimate='52m provisional:old31 1660s+171+132+386+222=2571s,20% margin~51.4m,not timeout; revise to~58m for added36 after measurement',
        runtime_cache=str(RUNTIME_CACHE),catalog_sha256=sha(OUT/'catalog.json')))
    slash=chr(92)
    command=[f'cd {REPO}',"gpuq --name 'current-default final35 provisional' --estimate 52m -- env "+slash,
        "  MAKEFLAGS='-j8 -l12' CMAKE_BUILD_PARALLEL_LEVEL=8 MAX_JOBS=8 "+slash,
        '  TORCHINDUCTOR_COMPILE_THREADS=8 PYTHONDONTWRITEBYTECODE=1 '+slash]
    command.extend(f'  {key}={RUNTIME_CACHE/name} '+slash for key,name in (('WARP_CACHE_PATH','warp'),
        ('TORCHINDUCTOR_CACHE_DIR','inductor'),('TRITON_CACHE_DIR','triton'),('CUDA_CACHE_PATH','cuda'),('TORCH_EXTENSIONS_DIR','torch_extensions')))
    command.extend(['  /home/astesia/Sevenova/curobo_v2_fast/.venv/bin/python '+slash,
        f'  {Path(__file__).resolve()} --suite final35 --run --run-id run_001 '+slash,f'  > {OUT}/gpuq_run_001.log 2>&1'])
    (OUT/'proposed_launch_command.txt').write_text('\n'.join(command)+'\n')
    for filename in ('regression.py','audit_batch.py'):
        previous=(OUT/('before_'+filename+'.txt')).read_text();current=(Path(__file__).parent/filename).read_text()
        (OUT/(filename+'.diff')).write_text(''.join(difflib.unified_diff(previous.splitlines(True),current.splitlines(True),
            fromfile='before/'+filename,tofile='final35/'+filename)))
    if 'torch' in sys.modules or 'curobo' in sys.modules:raise RuntimeError('CPU preparation imported GPU library')
    print('FINAL35 CPU PREPARED: original34 exact +negative raw; current actual arrays smoke PASS; source UNFROZEN; HOLD/noGPU')


def prepare_final36():
    """CPU only: preserve36 raw documents, reopen current-role samples, snapshot current source."""
    from v3_box_layout import parse_layout
    from curobo_core.layout_contract import validate_pair
    suite=json.loads((OUT/'catalog.json').read_text());cases=suite['cases']
    prior=json.loads((FINAL_OUT/'catalog.json').read_text())['cases']
    if len(cases)!=36 or [c['request'] for c in cases]!=[c['request'] for c in catalog()]:
        raise ValueError('final36 catalog differs from original35 plus original SKU raw')
    for i,case in enumerate(cases):
        document=json.loads(Path(case['request_file']).read_text())
        if document!=case['request'] or sha(case['request_file'])!=case['request_sha256'] or document_sha(document)!=case['document_sha256']:
            raise ValueError('original raw request bytes/document changed')
        contract=validate_pair(parse_layout(document))
        if abs(contract['front_distance_m']-.66)>1e-9 or any(p.side_min_height_m!=.79 for p in parse_layout(document).values()):
            raise ValueError('final36 raw global66/79 contract changed')
        if i<35 and any(case[k]!=prior[i][k] for k in ('index','name','request','request_sha256','document_sha256')):
            raise ValueError('prior35 immutable request changed')
    if Path(cases[35]['request_file']).read_bytes()!=SKU_RAW.read_bytes():raise ValueError('SKU raw copy changed')
    old_proof=json.loads((OUT/'prior35_artifact_hashes.json').read_text())
    if any(sha(p)!=h for p,h in old_proof.items()):raise ValueError('old final35 artifacts changed')
    old34_proof=json.loads((FINAL_OUT/'old34_immutable_before.json').read_text())
    if any(sha(p)!=h for p,h in old34_proof.items()):raise ValueError('old34 artifacts changed')
    smoke_sources=(ROOT/'generated/two_box_layout/role_generalization_20261124/default_admission_attempt_07/default/result.json',
        ROOT/'generated/two_box_layout/sku_order_handover_20261124/default_attempt_01/default/result.json')
    for index,label,path in ((34,'negative',smoke_sources[0]),(35,'sku_order',smoke_sources[1])):
        checked=inspect_result(cases[index],path)
        checked['scope']='Existing actual default result reopened with current CPU dispatcher; smoke only, not new final36 batch evidence.'
        save(OUT/(label+'_current_cpu_smoke.json'),checked)
    for filename in ('regression.py','audit_batch.py'):
        before=(OUT/('before_apply_'+filename+'.txt')).read_text();after=(Path(__file__).parent/filename).read_text()
        (OUT/(filename+'.diff')).write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),
            fromfile='before/'+filename,tofile='final36/'+filename)))
    snapshot=OUT/'preflight';snapshot.mkdir(exist_ok=False);frozen=freeze(snapshot)
    save(OUT/'preparation.json',dict(prepared=True,cases=36,generated_only=False,gpu_submitted=False,gpu_initialized=False,
        sidecar_patch_applied=True,source_frozen=True,awaiting_final_go=True,old35_unchanged=True,old34_artifacts_unchanged=True,
        sku_raw_byte_equal=True,no_prior_successes_inherited=True,source_digest=frozen['source_digest'],
        source_manifest=str(snapshot/'source_manifest.json'),public_native_spline=frozen['public_native_spline'],
        sidecar_sha256={str(p):sha(p) for p in Path(__file__).parent.glob('*.py')},catalog_sha256=sha(OUT/'catalog.json'),
        cpu_executable=sys.executable,runtime_cache=str(RUNTIME_CACHE),
        estimate='56m provisional:1660+171+132+386+222+163=2734s,+20%=54.7min;56min allows cold/native-admission variation;not timeout'))
    slash=chr(92)
    lines=[f'cd {REPO}',"gpuq --name 'current-default final36 provisional' --estimate 56m -- env "+slash,
        "  MAKEFLAGS='-j8 -l12' CMAKE_BUILD_PARALLEL_LEVEL=8 MAX_JOBS=8 "+slash,
        '  TORCHINDUCTOR_COMPILE_THREADS=8 PYTHONDONTWRITEBYTECODE=1 '+slash]
    lines.extend(f'  {key}={RUNTIME_CACHE/name} '+slash for key,name in (('WARP_CACHE_PATH','warp'),
        ('TORCHINDUCTOR_CACHE_DIR','inductor'),('TRITON_CACHE_DIR','triton'),('CUDA_CACHE_PATH','cuda'),('TORCH_EXTENSIONS_DIR','torch_extensions')))
    lines.extend(['  /home/astesia/Sevenova/curobo_v2_fast/.venv/bin/python '+slash,
        f'  {Path(__file__).resolve()} --suite final36 --run --run-id run_001 '+slash,f'  > {OUT}/gpuq_run_001.log 2>&1'])
    (OUT/'launch_command.txt').write_text('\n'.join(lines)+'\n')
    (OUT/'external_cpu_command.txt').write_text(f'cd {REPO}\nPYTHONDONTWRITEBYTECODE=1 /home/astesia/Sevenova/curobo_v2_fast/.venv/bin/python {Path(__file__).parent}/audit_batch.py --run {OUT}/run_001\n')
    if 'torch' in sys.modules or 'curobo' in sys.modules:raise RuntimeError('CPU final36 preparation imported GPU library')
    print('FINAL36 CPU READY;36raws exact;negative+SKU actual smokePASS;current source frozen '+frozen['source_digest']+';NO GPU submitted')


def inspect_result(case,path,patch=None):
    """Independent actual CPU dispatcher+sample check, not cached success flags alone."""
    import numpy as np
    from v3_box_layout import verify_layout_result,parse_layout
    from curobo_core.layout_contract import CARRY_DEGREES,validate_pair
    from experiments.two_box_pick_20261119.replay_native import native_samples
    record=json.loads(Path(path).read_text())
    if json.loads(Path(case['request_file']).read_text())!=case['request'] or sha(case['request_file'])!=case['request_sha256']:
        raise ValueError('original request file changed')
    if record.get('request')!=case['request']:raise ValueError('default entry changed original request document')
    if not all(record.get(k) is True for k in ('success','physical_success','task_success')):
        raise ValueError('default entry success/physical/task not all accepted: '+str(record.get('failure')))
    check=verify_layout_result(record)
    if check.get('valid') is not True:raise ValueError('CPU dispatcher rejected actual result')
    times,q,labels=native_samples(record)
    if len(times)<2 or q.shape!=(len(times),15) or not np.isfinite(q).all() or not np.all(np.diff(times)>0):
        raise ValueError('actual replay arrays/clock rejected')
    error=float(np.max(np.abs(q[-1,1:]-np.deg2rad(np.r_[CARRY_DEGREES,-CARRY_DEGREES]))))
    tree=ET.parse(ROOT/'generated/fast_runtime_assets/fixed.urdf')
    bound=tree.find("joint[@name='updown']/limit")
    if error>1e-5 or not float(bound.get('lower'))<=q[-1,0]<=float(bound.get('upper')):
        raise ValueError('actual final14 angles/updown rejected')
    picks=parse_layout(case['request']);physical=validate_pair(picks)
    if abs(physical['front_distance_m']-.66)>1e-9 or any(p.side_min_height_m!=.79 for p in picks.values()):
        raise ValueError('global66/79 contract changed')
    spline=record['native_spline']
    if patch is not None:
        for key in ('source_tree_sha256','kernel_tree_sha256','patched_header_sha256','patch_sha256'):
            if spline[key]!=patch[key]:raise ValueError('public patched kernel metadata changed: '+key)
    stages=record['timing']['stages']
    provenance=[]
    for stage in stages:
        trajectory=Path(stage['trajectory_file'])
        with np.load(trajectory,allow_pickle=False) as arrays:
            for key,width in (('position',15),('velocity',15),('acceleration',15),('jerk',15),('torque',14)):
                if arrays[key].shape!=(len(arrays['times']),width) or not np.isfinite(arrays[key]).all():
                    raise ValueError('invalid real stage field '+key)
        provenance.append(dict(stage_id=stage.get('stage_id',stage.get('phase')),trajectory_file=str(trajectory),
            sha256=sha(trajectory),optimizer_success=stage.get('trajectory_optimization',{}).get('optimizer_success')))
    return dict(valid=True,workflow=record.get('workflow') or 'synchronous',workflow_raw=record.get('workflow'),
        stage_count=len(stages),native_samples=len(times),motion_seconds=float(times[-1]),final14_max_error_rad=error,
        final_updown_m=float(q[-1,0]),front_distance_m=physical['front_distance_m'],side_min_height_m=.79,
        original_request_unchanged=True,request_document_sha256=document_sha(record['request']),
        flags={k:record.get(k) for k in ('success','physical_success','task_success')},
        actual_extraction_checks=check.get('extraction',check.get('endpoints')),
        actual_mass_switch_checks=check.get('mass_switches'),actual_source_checks=check.get('process_validation',record.get('process_validation')),
        actual_support_transfer=check.get('support_transfer'),
        native_spline=spline,trajectory_provenance=provenance,independent_cpu_check=check,
        result_sha256=sha(path),worker_wall_seconds=record.get('worker_wall_seconds'))


def _paths(kernel_tree, library=None):
    paths=set()
    for folder in (ROOT/'tools',ROOT/'config',ROOT/'models',ROOT/'tests',
                   ROOT/'generated/fast_runtime_assets',ROOT/'generated/zero_model_candidate_20261018',
                   ROOT/'generated/pr71_current_dynamics',ROOT/'generated/v3_analytic_071cb95',
                   ROOT/'generated/v3_suction_v322_6bb184b/frozen_collision_model',Path(kernel_tree)):
        paths.update(p.resolve() for p in folder.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc')
    for path in (ROOT/'generated/current_carry_target_6d.json',ROOT/'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json',
                 OUT/'catalog.json',OLD/'default_full31/catalog.json',OLD/'broader_same_side_y85_request.json'):
        paths.add(path.resolve())
    paths.update(p.resolve() for p in (OUT/'requests').glob('*.json'))
    paths.update((TOP/(name+'_request.json')).resolve() for name in TOP_REQUESTS)
    if SUITE in ('final35','final36'):
        paths.add(NEGATIVE.resolve());paths.add((ORIGINAL_OUT/'catalog.json').resolve())
        paths.update(p.resolve() for p in (ORIGINAL_OUT/'requests').glob('*.json'))
    if SUITE=='final36':
        paths.add(SKU_RAW.resolve());paths.add((FINAL_OUT/'catalog.json').resolve())
        paths.update(p.resolve() for p in (FINAL_OUT/'requests').glob('*.json'))
    # Config-selected atlases/models and URDF meshes are frozen as real files, too.
    def references(value):
        if isinstance(value,dict):
            for item in value.values():references(item)
        elif isinstance(value,list):
            for item in value:references(item)
        elif isinstance(value,str) and value.startswith(('generated/','models/')) and (ROOT/value).is_file():paths.add((ROOT/value).resolve())
    for path in (ROOT/'config').rglob('*.json'):references(json.loads(path.read_text()))
    for path in list(paths):
        if path.suffix=='.urdf':
            for mesh in ET.parse(path).findall('.//mesh'):
                target=Path(mesh.get('filename'))
                if not target.is_absolute():target=path.parent/target
                if target.is_file():paths.add(target.resolve())
    library=Path(library) if library is not None else Path(importlib.util.find_spec('curobo').origin).parent
    paths.update(p.resolve() for p in library.rglob('*') if p.is_file() and p.suffix in ('.py','.cuh','.cu','.h','.cpp','.json','.yaml','.yml'))
    return sorted(paths)


def live_manifest(kernel_tree,library=None):return {str(path):sha(path) for path in _paths(kernel_tree,library)}


def _source_diff(before,current):
    return [dict(path=p,before=before.get(p),after=current.get(p))
        for p in sorted(set(current)|set(before)) if current.get(p)!=before.get(p)]


def changed_sources(frozen):
    return _source_diff(frozen['files'],live_manifest(frozen['public_native_spline']['kernel_tree'],frozen['curobo_path']))


def freeze(run):
    from curobo_core.native_spline import prepare_kernel_tree
    from v3_box_layout import session_source_stamp
    patch=prepare_kernel_tree();before=live_manifest(patch['kernel_tree'])
    library=Path(importlib.util.find_spec('curobo').origin).parent.resolve()
    for value,digest in before.items():
        path=Path(value)
        if path.is_relative_to(ROOT):relative=Path('project')/path.relative_to(ROOT)
        elif path.is_relative_to(library):relative=Path('vendor_curobo')/path.relative_to(library)
        else:relative=Path('external_assets')/str(path).lstrip('/')
        target=run/'source_snapshot'/relative;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,target)
        if sha(target)!=digest:raise ValueError('source changed while snapshotting '+value)
    frozen=dict(files=before,public_native_spline=patch,session_source_stamp=session_source_stamp(),
        executable=sys.executable,curobo_path=str(library),curobo_version=importlib.metadata.version('nvidia-curobo'),
        torch_version=importlib.metadata.version('torch'),source_digest=document_sha(before),
        environment={k:os.environ.get(k) for k in ('MAKEFLAGS','CMAKE_BUILD_PARALLEL_LEVEL','MAX_JOBS','TORCHINDUCTOR_COMPILE_THREADS')},
        scope='Current source/config/tests/models/public kernel and vendor snapshot; no old284 equality gate.')
    if changed_sources(frozen):raise ValueError('source changed before the batch began')
    save(run/'source_manifest.json',frozen)
    return frozen


def require_gpuq():
    """Read the existing global lock only; never acquire another lock or bypass gpuq."""
    inode=str(Path(os.environ.get('GPU_MONITOR_LOCK','/tmp/sevenova-curobo-gpu.lock')).stat().st_ino)
    ancestors=set();pid=os.getpid()
    while pid>1:
        ancestors.add(pid);pid=int(Path(f'/proc/{pid}/stat').read_text().rsplit(') ',1)[1].split()[1])
    for line in Path('/proc/locks').read_text().splitlines():
        fields=line.split()
        if '->' not in fields and 'FLOCK' in fields and 'WRITE' in fields and any(f.endswith(':'+inode) for f in fields):
            owner=int(fields[4])
            if owner in ancestors:return owner
    raise RuntimeError('--run must be a child of PATH gpuq holding the existing shared global lock')


def continuation_baseline(frozen,parent_run,selected_indices):
    parent_run=Path(parent_run).resolve()
    if parent_run!=OUT/'run_001' or selected_indices!=[32,33,34]:
        raise ValueError('only explicit run_001 continuation of original indices32,33,34 is authorized')
    parent=json.loads((parent_run/'source_manifest.json').read_text())
    summary=json.loads((parent_run/'summary.json').read_text())
    audit=json.loads((parent_run/'independent_cpu34.json').read_text())
    if summary['passed']!=31 or summary['state']!='incomplete_failed_case' or not audit['current_source_matches']:
        raise ValueError('parent31 baseline or terminal source audit is unverified')
    changes=_source_diff(parent['files'],frozen['files'])
    if {r['path'] for r in changes}!=set(HARNESS_FILES):
        raise ValueError('continuation may change only the two sidecar harnesses: '+str(changes))
    evidence=[]
    for row,checked in zip(summary['cases'][:31],audit['cases'][:31]):
        if not row['success'] or not checked['success'] or sha(row['output'])!=checked['result_sha256']:
            raise ValueError('parent actual result changed or not independently accepted')
        for stage in checked['trajectory_provenance']:
            if sha(stage['trajectory_file'])!=stage['sha256']:raise ValueError('parent actual native arrays changed')
        evidence.append(dict(index=row['index'],result=row['output'],result_sha256=checked['result_sha256'],
            trajectories=checked['trajectory_provenance']))
    if parent['public_native_spline']!=frozen['public_native_spline']:
        raise ValueError('public kernel metadata must remain exactly source-equivalent')
    protected={p:h for p,h in parent['files'].items() if p not in HARNESS_FILES}
    return dict(parent_run_001=str(parent_run),selected_indices=selected_indices,parent_source_digest=parent['source_digest'],
        new_source_digest=frozen['source_digest'],changed_files=changes,unchanged_protected_files=len(protected),
        protected_source_digest=document_sha(protected),public_native_spline_unchanged=parent['public_native_spline']==frozen['public_native_spline'],
        parent_verified31=evidence,scope='Additive continuation; parent run001 failure remains failed, not overwritten. Same production/vendor/model/kernel sources; only sidecar cache/continuation changes.')


def prepare_continuation():
    preflight=OUT/'continuation_002_preflight';preflight.mkdir(exist_ok=False)
    frozen=freeze(preflight);comparison=continuation_baseline(frozen,OUT/'run_001',[32,33,34])
    for path in HARNESS_FILES:
        relative=Path(path).relative_to(ROOT)
        old=(OUT/'run_001/source_snapshot/project'/relative).read_text()
        new=Path(path).read_text()
        (preflight/(Path(path).name+'.diff')).write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),
            fromfile='run_001/'+str(relative),tofile='run_002/'+str(relative))))
    save(preflight/'baseline_comparison.json',comparison)
    cache_values={k:str(RUNTIME_CACHE/name) for k,name in (('WARP_CACHE_PATH','warp'),('TORCHINDUCTOR_CACHE_DIR','inductor'),
        ('TRITON_CACHE_DIR','triton'),('CUDA_CACHE_PATH','cuda'),('TORCH_EXTENSIONS_DIR','torch_extensions'))}
    slash=chr(92)
    lines=[f'cd {REPO}',"gpuq --name 'default34 continuation32-34 provisional' --estimate 18m -- env "+slash,
        "  MAKEFLAGS='-j8 -l12' CMAKE_BUILD_PARALLEL_LEVEL=8 MAX_JOBS=8 "+slash,
        '  TORCHINDUCTOR_COMPILE_THREADS=8 PYTHONDONTWRITEBYTECODE=1 '+slash]
    lines.extend(f'  {key}={value} '+slash for key,value in cache_values.items())
    lines.extend(['  /home/astesia/Sevenova/curobo_v2_fast/.venv/bin/python '+slash,
        f'  {Path(__file__).resolve()} --run --run-id run_002 '+slash,
        f'  --parent-run {OUT}/run_001 --indices 32 33 34 '+slash,f'  > {OUT}/gpuq_run_002.log 2>&1'])
    (preflight/'launch_command.txt').write_text('\n'.join(lines)+'\n')
    save(preflight/'preparation.json',dict(ready=True,gpu_submitted=False,awaiting_explicit_go=True,
        parent_run_001=str(OUT/'run_001'),selected_indices=[32,33,34],cache_paths=cache_values,
        history_namespace_conflict_removed=not RUNTIME_CACHE.is_relative_to(ROOT/'generated/two_box_layout'),
        estimate='18m provisional; side149+Top132+Both386~667s plus cold cache and30-50% variability; not timeout',
        source_digest=frozen['source_digest'],production_equivalent_to_parent=True))
    if 'torch' in sys.modules or 'curobo' in sys.modules:raise RuntimeError('CPU continuation preparation imported GPU library')
    print('CONTINUATION PREFLIGHT READY;1394 protected files unchanged;selected32,33,34;18m provisional;NO GPU submitted')


def run_batch(run_id,parent_run=None,selected_indices=None):
    owner=require_gpuq()
    if parent_run is not None:
        for key,name in (('WARP_CACHE_PATH','warp'),('TORCHINDUCTOR_CACHE_DIR','inductor'),('TRITON_CACHE_DIR','triton'),
                         ('CUDA_CACHE_PATH','cuda'),('TORCH_EXTENSIONS_DIR','torch_extensions')):
            if os.environ.get(key)!=str(RUNTIME_CACHE/name):raise ValueError('continuation runtime cache path changed: '+key)
    for key,value in dict(MAKEFLAGS='-j8 -l12',CMAKE_BUILD_PARALLEL_LEVEL='8',MAX_JOBS='8',TORCHINDUCTOR_COMPILE_THREADS='8').items():
        if os.environ.get(key)!=value:raise ValueError('required compile cap missing: '+key)
    if SUITE in ('final35','final36') and (parent_run is not None or selected_indices is not None):
        raise ValueError('final current-source suite must freshly execute every request; no prior baseline inheritance')
    if not re.fullmatch('[a-zA-Z0-9_-]+',run_id):raise ValueError('unsafe run id')
    run=OUT/run_id;run.mkdir(exist_ok=False);began=time.perf_counter()
    suite=json.loads((OUT/'catalog.json').read_text());cases=suite['cases']
    if len(cases)!=CASE_COUNT or [c['request'] for c in cases]!=[c['request'] for c in catalog()]:raise ValueError(str(CASE_COUNT)+' original requests changed')
    frozen=freeze(run)
    if SUITE=='final36':
        expected=json.loads((OUT/'preflight/source_manifest.json').read_text())
        if frozen['source_digest']!=expected['source_digest']:raise ValueError('source differs from reviewed final36 snapshot')
    continuation=None
    if parent_run is not None:
        continuation=continuation_baseline(frozen,parent_run,selected_indices)
        expected=json.loads((OUT/'continuation_002_preflight/source_manifest.json').read_text())
        if frozen['source_digest']!=expected['source_digest']:raise ValueError('source changed since reviewed continuation preflight')
        save(run/'continuation_baseline.json',continuation)
    elif selected_indices is not None:raise ValueError('selected indices require explicit parent run')
    summary=dict(state='running',suite=SUITE,cases_count=CASE_COUNT,catalog_file=str(OUT/'catalog.json'),scope=suite['scope'],queue_owner_pid=owner,batch_pid=os.getpid(),source_digest=frozen['source_digest'],
        cases=[dict(index=c['index'],name=c['name'],state='not_started',success=False) for c in cases],completed=0,passed=0)
    if continuation is not None:
        parent=json.loads((Path(parent_run)/'summary.json').read_text())
        summary.update(parent_run_001=str(Path(parent_run).resolve()),selected_indices=selected_indices,
            additive_source_equivalent=True,continuation_baseline=str(run/'continuation_baseline.json'))
        for i in range(31):
            inherited=copy.deepcopy(parent['cases'][i]);inherited.update(state='inherited_passed',
                inherited_from_parent_run=str(Path(parent_run).resolve()),parent_result_sha256=sha(inherited['output']),
                parent_source_digest=parent['source_digest'])
            summary['cases'][i]=inherited
        for i in (31,32,33):summary['cases'][i]['parent_state']=parent['cases'][i]['state']
        summary.update(completed=31,passed=31,executed_in_this_run=0)
    save(run/'summary.json',summary)
    from v3_box_layout import solve_file
    planners=OrderedDict();session=dict(source_stamp=frozen['session_source_stamp'],id=run_id)
    for index,case in enumerate(cases):
        if continuation is not None and case['index'] not in selected_indices:continue
        changes=changed_sources(frozen)
        if changes:
            summary.update(state='incomplete_source_changed',source_changes=changes);break
        folder=run/'cases'/f"{case['index']:02d}_{case['name']}";folder.mkdir(parents=True)
        row=summary['cases'][index];row.update(state='running',output=str(folder/'default/result.json'),
            source_digest_before=frozen['source_digest'],source_manifest=str(run/'source_manifest.json'))
        save(run/'summary.json',summary);started=time.perf_counter()
        code=None
        try:
            with (folder/'worker.log').open('w') as log,redirect_stdout(log),redirect_stderr(log):
                code=solve_file(SimpleNamespace(solve_file=Path(case['request_file']),output=folder/'default',max_contacts=16),planners=planners,session=session)
            row['entry_return_code']=code
            status=json.loads((folder/'default/status.json').read_text())
            row['status']={key:status.get(key) for key in ('state','elapsed_s','reason')}
            row['status_file']=str(folder/'default/status.json')
            raw=json.loads((folder/'default/result.json').read_text())
            row.update(raw_flags={k:raw.get(k) for k in ('success','physical_success','task_success')},
                workflow=raw.get('workflow') or 'synchronous',failure=raw.get('failure'),planner_reused=raw.get('planner_reused'))
            if code!=0 or status.get('state')!='accepted':
                raise RuntimeError('default solve_file returned '+str(code)+': '+str(raw.get('failure')))
            checked=inspect_result(case,folder/'default/result.json',frozen['public_native_spline'])
            save(folder/'cpu_verification.json',checked)
            row.update(state='passed',success=True,verification_file=str(folder/'cpu_verification.json'),
                motion_seconds=checked['motion_seconds'],native_samples=checked['native_samples'],stage_count=checked['stage_count'],
                final14_max_error_rad=checked['final14_max_error_rad'],final_updown_m=checked['final_updown_m'],
                request_document_sha256=checked['request_document_sha256'])
        except BaseException as error:
            row.update(state='failed' if isinstance(error,Exception) else 'interrupted',success=False,
                entry_return_code=code,error=repr(error),traceback=traceback.format_exc())
        row['wall_seconds']=time.perf_counter()-started
        actual_sources=live_manifest(frozen['public_native_spline']['kernel_tree'])
        changes=_source_diff(frozen['files'],actual_sources)
        row.update(source_digest_after=document_sha(actual_sources),source_matches_snapshot=not changes)
        if changes:row.update(success=False,state='source_changed',source_changes=changes)
        summary.update(completed=index+1,passed=sum(r['success'] for r in summary['cases']),
            executed_in_this_run=summary.get('executed_in_this_run',0)+1,wall_seconds=time.perf_counter()-began)
        save(folder/'case_summary.json',row);save(run/'summary.json',summary)
        if continuation is not None or (index+1)%5==0 or not row['success'] or index+1==CASE_COUNT:
            print(f"REGRESSION_PROGRESS completed={index+1}/{CASE_COUNT} passed={summary['passed']} elapsed_s={summary['wall_seconds']:.3f}",flush=True)
            print('LATEST',case['name'],row['state'],row.get('error',''),flush=True)
        if not row['success']:
            summary['state']='incomplete_source_changed' if changes else 'incomplete_failed_case';break
    else:summary['state']='completed'
    summary.update(success=summary['state']=='completed' and summary['passed']==CASE_COUNT,wall_seconds=time.perf_counter()-began,
        source_changes_final=changed_sources(frozen),all_cases_retained=True)
    if summary['source_changes_final']:summary.update(success=False,state='incomplete_source_changed')
    save(run/'summary.json',summary)
    print('REGRESSION_TERMINAL',summary['state'],summary['passed'],'/'+str(CASE_COUNT),flush=True)
    return 0 if summary['success'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group(required=True);mode.add_argument('--prepare',action='store_true');mode.add_argument('--run',action='store_true');mode.add_argument('--prepare-continuation',action='store_true')
    parser.add_argument('--suite',choices=('original34','final35','final36'),default='original34')
    parser.add_argument('--run-id',default='run_001');parser.add_argument('--parent-run',type=Path);parser.add_argument('--indices',nargs='+',type=int);args=parser.parse_args()
    select_suite(args.suite)
    if args.prepare and SUITE=='final36':prepare_final36()
    elif args.prepare and SUITE=='final35':prepare_final35()
    elif args.prepare:
        suite=prepare()
        smoke=ROOT/'generated/two_box_layout/takeover_20261124/default_y85/result.json'
        if smoke.exists():
            checked=inspect_result(suite['cases'][31],smoke);checked['scope']='Existing main default Y85 CPU smoke only; not a new regression case execution.'
            save(OUT/'cpu_harness_smoke.json',checked)
        if 'torch' in sys.modules or 'curobo' in sys.modules:raise RuntimeError('CPU harness imported a GPU library')
        print('PREPARED 34 exact original requests; CPU smoke passed; NO GPU submitted; awaiting explicit GO; source freeze deferred')
    elif args.prepare_continuation:prepare_continuation()
    else:sys.exit(run_batch(args.run_id,args.parent_run,args.indices))
