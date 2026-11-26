"""Fresh request -> fresh13 geometry -> actual native13 under one caller-owned gpuq lock."""
from pathlib import Path
import argparse
import copy
import gc
import hashlib
import importlib.metadata
import json
import sys
import time
import traceback
import numpy as np

ROOT=Path(__file__).resolve().parents[4]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'generated/two_box_layout/joint_contract_20261124_fresh_native'
REQUEST=ROOT/'generated/two_box_layout/joint_contract_20261123/broader_same_side_y85_request.json'
sys.path[:0]=[str(ROOT/'tools'),str(HERE),str(HERE.parent/'transfer_fresh'),str(HERE.parent/'transfer_suffix'),str(HERE.parent/'transfer_native'),str(HERE.parent/'transfer_prefix_native')]


def save(path,obj):
    path.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')


def plan_document(document,output,progress=lambda event:None,*,max_contacts=16):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    started=time.perf_counter()
    record=dict(workflow='handover_same_side',request=copy.deepcopy(document),success=False,physical_success=False,task_success=False,
        timing=dict(success=False,stages=[]),failure=None,real_robot_execution=False,max_contacts=max_contacts,
        scope='One fresh original-request13-stage native run; other inputs unknown, no universal or hardware claim.')
    guard_state={'active':False};historical_reads=[];restore_modules=[]
    def publish():
        record['wall_seconds']=time.perf_counter()-started;save(output/'record.json',record)
    def event(value):
        value=dict(value,elapsed_seconds=time.perf_counter()-started);progress(value)
        print('FRESH_NATIVE_EVENT',json.dumps({k:v for k,v in value.items() if k!='details'}),flush=True)
    try:
        # First GPU-related action: shared public implementation, never the experiment numeric_fix tree.
        from curobo_core.native_spline import install_native_spline
        from curobo._src.curobolib.backends.cuda_core_backend.trajectory_config import TrajectoryKernelCfg
        installed=getattr(TrajectoryKernelCfg,'_native_spline_install',None)
        kernel_cache=OUT/'native_kernel_cache' if installed is None else Path(installed['metadata']['kernel_tree']).parent
        patch=install_native_spline(cache_root=kernel_cache)
        assert install_native_spline(cache_root=kernel_cache)==patch
        record['native_spline']=patch;record['native_spline_initial_idempotent']=True
        import curobo,torch
        import plan_fresh as fresh
        from native_stages import prefix,transfer,suffix,STANDARD
        from curobo_core.native_transport import native_seam,concatenate_native
        from curobo_core.box_pick import BoxPick
        from dataclasses import asdict
        record['runtime']=dict(executable=sys.executable,curobo_path=curobo.__file__,curobo_version=importlib.metadata.version('nvidia-curobo'),torch_version=torch.__version__)
        if not isinstance(max_contacts,int) or isinstance(max_contacts,bool) or max_contacts<1:
            raise ValueError('max_contacts must be a positive integer')
        record['effective_contact_budget']=min(max_contacts,16)
        if any(b.get('swap_length_height',False) for b in document.get('boxes',[])):
            raise ValueError('unknown: fresh handover has not validated swap_length_height=True; physical dimensions will not be substituted')
        physical=copy.deepcopy(document)
        for b in physical.get('boxes',[]):b.pop('swap_length_height',None)
        baseline=fresh.source_hashes()
        # Construction guard survives only this call. Original manifest may be hashed as provenance.
        allowed={REQUEST.resolve(),(ROOT/'generated/two_box_layout/joint_contract_20261123/source_default_full31/manifest.json').resolve()}
        def guard(name,args):
            if not guard_state['active'] or name!='open' or not isinstance(args[0],(str,bytes)):return
            path=Path(args[0]).resolve()
            if path.suffix not in ('.json','.npz','.npy') or not path.is_relative_to(ROOT/'generated/two_box_layout') or path.is_relative_to(output) or path.is_relative_to(OUT/'native_kernel_cache'):return
            if path not in allowed:raise PermissionError('fresh native cannot consume old solution '+str(path))
            historical_reads.append(str(path.relative_to(ROOT)))
        sys.addaudithook(guard);guard_state['active']=True
        geometry_output=output/'geometry';geometry_output.mkdir()
        geo_report=dict(success=False);event(dict(stage='fresh_geometry',state='running'))
        import curobo_core.box_pick as box_pick
        import v3_two_box_pick,v3_fast_three_layers,probe_suffix
        for module,names in [(v3_two_box_pick,('prepare_fast_assets',)),(v3_fast_three_layers,('prepare_fast_assets','OUTPUT')),(probe_suffix,('OUT','inputs','hashes'))]:
            restore_modules.extend((module,name,getattr(module,name)) for name in names)
        original_order=box_pick.diverse_contact_order
        # Preserve the existing first16 ordering; a smaller caller budget only limits source trials.
        box_pick.diverse_contact_order=lambda rows,side:original_order(rows,side)[:record['effective_contact_budget']]
        try:geometry=fresh.plan_document(physical,geometry_output,geo_report)
        finally:box_pick.diverse_contact_order=original_order
        np.savez_compressed(geometry_output/'fresh_geometry.npz',**geometry);save(geometry_output/'fresh_report.json',geo_report)
        interface=json.loads((geometry_output/'stage_interface.json').read_text());interface['request']=copy.deepcopy(document)
        record['stage_interface']=interface;record['joint_names']=fresh.NAMES
        record['frames']=np.concatenate([geometry[k] for k in fresh.PHASES]).tolist()
        record['phases']=[k for k in fresh.PHASES for _ in geometry[k]]
        record['task']={b['side']:asdict(BoxPick(**dict(b,carry_joint_home=True))) for b in physical['boxes']}
        record['fresh_geometry_success']=bool(geo_report['success']);record['fresh_geometry_seconds']=time.perf_counter()-started
        record['fresh_geometry_file']=str((geometry_output/'fresh_geometry.npz').resolve())
        event(dict(stage='fresh_geometry',state='accepted',samples=sum(geo_report['stage_samples'].values())));publish()
        assert record['fresh_geometry_success']
        entries={stage['stage_id']:stage for stage in interface['stages']};native_arrays=[]
        def commit(key,data,details):
            index=len(record['timing']['stages']);assert key==fresh.PHASES[index],(index,key)
            assert all(k in data for k in STANDARD)
            first_last=[0,-1];rest=float(max(np.abs(data['velocity'][first_last]).max(),np.abs(data['acceleration'][first_last]).max()))
            assert rest<=1e-4,(key,'rest',rest)
            seam=dict(valid=True) if not native_arrays else native_seam(native_arrays[-1],data)
            assert seam['valid'],(key,seam)
            stage=dict(details,stage_id=key,physical_scene=entries[key]['scene'],success=True,physical_success=True,
                physical_payload_kg=entries[key]['payload_mass_kg'],actual_qva_seam=seam,
                samples=len(data['times']),motion_seconds=float(data['times'][-1]))
            assert Path(stage['trajectory_file']).resolve().is_relative_to(output)
            record['timing']['stages'].append(stage);native_arrays.append(data)
            publish();event(dict(stage=key,state='accepted',samples=stage['samples'],motion_seconds=stage['motion_seconds']))
        for name,run in [('prefix',prefix),('transfer',transfer),('suffix',suffix)]:
            folder=output/name;folder.mkdir();began=time.perf_counter();event(dict(stage=name,state='running'))
            run(geometry,geo_report,physical,folder,commit,event)
            record[name+'_seconds']=time.perf_counter()-began
            gc.collect();torch.cuda.empty_cache()
        assert len(native_arrays)==13 and [s['stage_id'] for s in record['timing']['stages']]==list(fresh.PHASES)
        # Clocks are rebased and duplicate seam rows removed; actual q/v/a/jerk/torque are never repaired.
        merged,seams=concatenate_native([{k:data[k] for k in STANDARD} for data in native_arrays])
        np.savez_compressed(output/'native_chain.npz',**merged)
        record['timing'].update(success=True,trajectory_file=str((output/'native_chain.npz').resolve()),
            native_samples=len(merged['times']),motion_seconds=float(merged['times'][-1]),qva_seams=seams)
        final_lift=float(merged['position'][-1,0])
        for pick in record['task'].values():pick['carry_updown_m']=final_lift
        record.update(success=True,physical_success=True)
        # Public CPU verifier independently joins native qva, derives masses/boxes from each scene,
        # and validates both complete original source histories, including static support alpha.
        from curobo_core.handover import verify_handover_result
        check=verify_handover_result(record)
        record['cpu_verification']=check;record['process_validation']=check['process_validation']
        record['task_success']=bool(check['valid'] and record['process_validation']['valid'])
        record['success']=record['physical_success']=record['task_success']
        from curobo._src.context import get_runtime
        cache=get_runtime().get_cuda_core_cache()
        compiled=sorted(set(str(path) for key in cache.compiled_kernels for path,_ in key[0] if '/trajectory/' in str(path)))
        assert compiled and all(Path(p).is_relative_to(Path(patch['kernel_tree'])) for p in compiled)
        assert install_native_spline(cache_root=kernel_cache)==patch
        record.update(compiled_trajectory_sources=compiled,native_spline_final_idempotent=True,historical_layout_data_reads=historical_reads)
        after=fresh.source_hashes();loaded={str(Path(m.__file__).resolve().relative_to(ROOT)) for m in list(sys.modules.values()) if getattr(m,'__file__',None) and Path(m.__file__).resolve().is_relative_to(ROOT)}
        tracked={p for p in baseline if not p.endswith('.py') or p in loaded}
        record['source_sha256']={p:baseline[p] for p in sorted(tracked)}
        changed=[p for p in tracked if after.get(p)!=baseline[p]];assert not changed,changed
        record['source_files_unchanged']=True
    except Exception as error:
        record.update(success=False,physical_success=False,task_success=False,failure=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        guard_state['active']=False
        for module,name,value in reversed(restore_modules):setattr(module,name,value)
        publish()
    return record


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--request',type=Path,default=REQUEST);p.add_argument('--attempt',default='attempt_01');p.add_argument('--max-contacts',type=int,default=16);args=p.parse_args()
    assert Path(args.attempt).name==args.attempt
    document=json.loads(args.request.read_text())
    result=plan_document(document,OUT/args.attempt,max_contacts=args.max_contacts)
    print('FRESH_NATIVE_RESULT',result['success'],'wall',result['wall_seconds'],'motion',result['timing'].get('motion_seconds'),flush=True)
    sys.exit(0 if result['success'] else 1)
