"""gpuq only: unchanged saved single-box transfer candidate, native timing gates."""
from pathlib import Path
from dataclasses import replace
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
HERE = Path(__file__).resolve().parent
OLD = ROOT/'generated/two_box_layout/joint_contract_20261124_transfer'
OUT = ROOT/'generated/two_box_layout/joint_contract_20261124_transfer_native'
sys.path[:0] = [str(ROOT/'tools'), str(HERE.parent/'transfer_followup'), str(HERE.parent/'transfer_prefix_native')]
from audit_transfer_geometry import source_hashes, inputs, fk, NAMES
from probe_prefix import prepared_assets, gpu_state

def save(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False)+'\n')

def run(output, report, optimize=False, reuse=None):
    import curobo
    import torch
    import v3_two_box_pick
    import v3_fast_three_layers as native
    from curobo_core.box_pick import BoxPick
    from curobo_core.scene import Pose, SceneObject
    from v3_batched_loaded_search import GpuValidity
    from diagnose_endpoint import components
    from v3_fast_path import fit_path
    from v3_box_layout import session_source_stamp
    hashes, prior, owner, _ = inputs()
    with np.load(OLD/'transfer_geometry.npz', allow_pickle=False) as data:
        geometry = {k:data[k].copy() for k in data.files}
    assert json.loads((OLD/'transfer_probe.json').read_text())['success']
    report.update(script_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*.py')},runtime=dict(executable=sys.executable,curobo_path=curobo.__file__,
        curobo_version=importlib.metadata.version('nvidia-curobo'),torch_version=torch.__version__),
        source_hashes_before=hashes, source_stamp_start=session_source_stamp(), gpu_start=gpu_state(),
        input_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in OLD.glob('*') if p.is_file()},
        scope='Saved single original right box candidate only; no prefix, suffix, fresh full flow or production default.',
        physical_box_id='right',payload_mass_kg=10.,pending_original_box=next(b for b in prior['request']['boxes'] if b['side']=='left'),
        original_prefix_match=True, endpoint_policy='Unchanged; no output sample repair or manual time scaling.',phases={})
    v3_two_box_pick.prepare_fast_assets = prepared_assets
    native.prepare_fast_assets = prepared_assets
    native.OUTPUT = output
    planner = v3_two_box_pick.TwoBoxPickPlanner()
    planner.configure_pick(owner)
    pending = BoxPick(**report['pending_original_box'])
    planner.snapshot = replace(planner.snapshot, objects=planner.snapshot.objects+(
        SceneObject('pending_original_left',pending.dimensions_m,Pose(pending.center_m)),))
    planner.native_project_margins = True
    checker = planner.checker(planner.selected_task,payload=False)
    assert not checker.guidance_payload
    report['original_endpoint_gates'] = {key:components(planner,checker,geometry[key][[0,-1]])
        for key in ('receiver_approach','donor_release','donor_continue')}
    print('ORIGINAL_ENDPOINT_GATES',json.dumps(report['original_endpoint_gates']),flush=True)
    save(output/'native_report.json',report)
    report['physical_identity'] = dict(object_id='wall_box_01',original_side='right',dimensions_m=geometry['box_dimensions_m'].tolist(),
        checker_geometry_name_to_physical_id={'pending_original_left':'wall_box_00'}, pending_object_id='wall_box_00',
        phases={key:dict(object_id='wall_box_01',parent_link=side+'_tool0',tool_to_object=(geometry['left_attachment'] if side=='left' else geometry['right_measured_attachment']).tolist())
            for key,side in [('receiver_approach','left'),('support_transfer','right'),('donor_release','right'),('donor_continue','right')]})
    from curobo_core.suction_grasp import contact_cap_spheres
    kin=checker.kinematics.config.kinematics_config
    cap_ids={};manifest=[]
    for side in ('left','right'):
        origin=planner.fk_robot.get_transform(side+'_tool0',side+'_link7')[:3,3]
        ids=kin.get_sphere_index_from_link_name(side+'_link7').cpu().tolist()
        entries=planner.robot['kinematics']['collision_spheres'][side+'_link7']
        assert len(ids)==len(entries)
        cap_ids[side]=[i for i,e in zip(ids,entries) if e['radius']>0 and e['center'][2]+e['radius']>origin[2]]
        assert len(cap_ids[side])==len(contact_cap_spheres(side))
    for link in planner.robot['kinematics']['collision_link_names']:
        ids=kin.get_sphere_index_from_link_name(link).cpu().tolist()
        entries=planner.robot['kinematics']['collision_spheres'][link]
        assert len(ids)==len(entries)
        for i,e in zip(ids,entries):
            side=next((s for s in cap_ids if i in cap_ids[s]),None)
            manifest.append(dict(link=link,center=e['center'],radius=e['radius'],cap_side=side,
                tool_cap_reach_m=None if side is None else float(np.max(contact_cap_spheres(side)[:,2]+contact_cap_spheres(side)[:,3]))))
    obstacles=[dict(name=o.name,pose=list(o.pose),dims=list(o.dims)) for o in list(checker.guidance_scene.cuboid)+list(checker.base_cuboid_layer.scene.cuboid)]
    save(output/'physical_scene.json',dict(sphere_manifest=manifest,obstacles=obstacles,physical_identity=report['physical_identity']))
    engine = native.FastThreeLayerPlanner(planner)
    from physical_cpu import geometry_audit
    import yourdfpy
    cpu_robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    if optimize:
        from native_cost import TransferCost,install
        from v3_fast_geometry_cost import GeometricCostManager
        install()
    report['sample_audits']={}
    attachments = dict(left=geometry['left_attachment'],right=geometry['right_measured_attachment'])
    for key,side in [('receiver_approach','left'),('donor_release','right'),('donor_continue','right')]:
        rows = geometry[key]
        values,start,goal,fit,_ = fit_path(dict(positions=rows),32)
        controls,first,last = [torch.tensor(x[None],device='cuda',dtype=torch.float32).contiguous() for x in (values,start,goal)]
        trajectory=None
        if optimize:
            GeometricCostManager.transfer_layer=TransferCost(checker,geometry,side,rows,cap_ids,owner)
            validity=GpuValidity(checker,check_ground=True,scene_penetration_tolerance_m=.001)
            phase={'receiver_approach':'approach','donor_release':'transport','donor_continue':'return_home'}[key]
            controls,first,last,_,trajectory=engine.optimize_path(dict(task=planner.selected_task,active_payload_sides=[]),phase,rows,validity,checker.guidance_scene)
        if reuse is None:
            arrays,timing = engine.time_path(controls,first,last,None,(side,),attachments,benchmark_recompute=False)
        else:
            retained=OUT/reuse
            with np.load(retained/(key+'.npz'),allow_pickle=False) as data:arrays={k:data[k].copy() for k in data.files}
            timing=json.loads((retained/'native_report.json').read_text())['phases'][key]['time_optimization']
            report['retained_native_arrays_from']=str(retained.relative_to(ROOT))
        stage = dict(support_side=side,experiment_full_single_obb_guidance=bool(optimize),representation_adaptation=fit,trajectory_optimization=trajectory,time_optimization=timing,success=False)
        report['phases'][key] = stage
        if arrays is not None:
            np.savez_compressed(output/(key+'.npz'),**arrays)
            stage.update(samples=len(arrays['times']),endpoint_error=float(abs(arrays['position'][[0,-1]]-rows[[0,-1]]).max()),
                updown_min=float(arrays['position'][:,0].min()),updown_max=float(arrays['position'][:,0].max()))
            components_all=[]
            for offset in range(0,len(arrays['position']),512):
                components_all.append(components(planner,checker,arrays['position'][offset:offset+512]))
            collision_names=['self_collision','world_collision','base_hard','native_base','ground_0mm','native_ground_2mm']
            robot_components={name:dict(maximum=max(c['components'][name]['maximum'] for c in components_all),
                positive_count=sum(len(c['components'][name]['positive_indices']) for c in components_all)) for name in components_all[0]['components']}
            full,C=geometry_audit(cpu_robot,arrays['position'],side,attachments,dict(left=geometry['left_attachment'],right=geometry['right_nominal_attachment']),geometry['box_dimensions_m'],manifest,obstacles,owner)
            independent,_=engine.dynamics.torque(arrays['position'],arrays['velocity'],arrays['acceleration'],(side,),attachments)
            delta=float(np.max(np.abs(independent[:,1:]-arrays['torque'])))
            ratio=float(np.max(np.abs(independent[:,1:])/geometry['scaled_torque_limits_nm']))
            stage.update(robot_components=robot_components,full_single_obb=full,independent_gpu_rnea=dict(max_native_difference_nm=delta,max_scaled_ratio=ratio,valid=delta<=.002 and ratio<=1.),
                all_sample_robot_collision_valid=all(robot_components[k]['positive_count']==0 for k in collision_names),
                strict_joint_bounds_valid=robot_components['joint_bound']['positive_count']==0)
            stage['success']=bool(stage['all_sample_robot_collision_valid'] and stage['strict_joint_bounds_valid'] and full['valid'] and stage['independent_gpu_rnea']['valid'] and timing['success'] and (trajectory is None or trajectory['solver_success']))
        print('NATIVE_TIME_GATE' ,key,json.dumps(stage),flush=True)
        save(output/'native_report.json',report)
    q=geometry['support_transfer'].copy();zeros=np.zeros_like(q)
    torques=[]
    for side in ('left','right'):
        tau,_=engine.dynamics.torque(q,zeros,zeros,(side,),attachments);torques.append(tau[:,1:])
    alpha=geometry['support_alpha'];tau=(1-alpha[:,None])*torques[0]+alpha[:,None]*torques[1]
    np.savez_compressed(output/'support_transfer.npz',position=q,velocity=zeros,acceleration=zeros,jerk=zeros,
        times=np.linspace(0.,1.,len(q)),torque=tau,support_alpha=alpha)
    report['stationary_support_transfer']=dict(success=bool(np.all(np.abs(tau)<=geometry['scaled_torque_limits_nm'])),
        samples=len(q),duration_s=1.,duration_policy='Explicit stationary 1s support ramp, not manual motion time scaling.',physical_mass_kg=10.,
        extreme_max_scaled_ratio={s:float(np.max(np.abs(t)/geometry['scaled_torque_limits_nm'])) for s,t in zip(('left','right'),torques)},
        mass_allocation='10*(1-alpha)kg left + 10*alpha kg right, only at fixed q/v/a=rest; endpoint inertia models each contain one box.')
    report['success']=bool(all(s['success'] for s in report['phases'].values()) and report['stationary_support_transfer']['success'])
    report['failure_classification']='native_float32_joint_bound_only' if all(s.get('all_sample_robot_collision_valid') and s.get('full_single_obb',{}).get('valid') and s.get('independent_gpu_rnea',{}).get('valid') for s in report['phases'].values()) and not report['success'] else 'inspect_per_phase'
    report.update(source_hashes_after=source_hashes(),source_stamp_end=session_source_stamp())

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--attempt',default='attempt_01')
    parser.add_argument('--optimize',action='store_true')
    parser.add_argument('--reuse')
    args=parser.parse_args()
    assert Path(args.attempt).name==args.attempt
    output=OUT/args.attempt
    output.mkdir(parents=True,exist_ok=False)
    began=time.perf_counter()
    report=dict(success=False,estimate='2m provisional; geometry4.26s/ordered-native93s plus cold initialization margin')
    try:
        run(output,report,args.optimize,args.reuse)
    except Exception as error:
        report.update(error=repr(error),traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        report.update(wall_seconds=time.perf_counter()-began,gpu_before_exit=gpu_state())
        save(output/'native_report.json',report)
        print('TRANSFER_NATIVE_RESULT',report['success'],'wall',report['wall_seconds'],flush=True)
    sys.exit(0 if report['success'] else 1)
