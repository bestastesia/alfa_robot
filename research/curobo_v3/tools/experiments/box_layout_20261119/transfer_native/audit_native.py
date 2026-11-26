"""Independent CPU check of actual native q/v/a, full OBB, contact, seams and RNEA."""
from pathlib import Path
import argparse
import json
import sys
import time
import importlib.metadata
import numpy as np
import yourdfpy
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[4]
HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'tools'),str(HERE.parent/'transfer_followup')]
from audit_transfer_geometry import inputs,source_hashes
from physical_cpu import NAMES,geometry_audit,rnea,forward

def main(folder):
    began=time.perf_counter();hashes,_,owner,_=inputs()
    metadata=json.loads((folder/'physical_scene.json').read_text())
    with np.load(ROOT/'generated/two_box_layout/joint_contract_20261124_transfer/transfer_geometry.npz',allow_pickle=False) as data:g={k:data[k].copy() for k in data.files}
    robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/dynamics.urdf',load_meshes=False)
    attachments=dict(left=g['left_attachment'],right=g['right_measured_attachment'])
    nominal=dict(left=g['left_attachment'],right=g['right_nominal_attachment'])
    result=dict(valid=False,runtime=dict(executable=sys.executable,numpy_version=np.__version__,yourdfpy_version=importlib.metadata.version('yourdfpy')),source_hashes=hashes,phases={},seams=[])
    previous=None;previous_C=None;allvalid=True;physicalvalid=True;duration=0.;samples=0
    for key,side in [('receiver_approach','left'),('support_transfer','right'),('donor_release','right'),('donor_continue','right')]:
        path=folder/(key+'.npz')
        if not path.exists():continue
        with np.load(path,allow_pickle=False) as d:a={k:d[k].copy() for k in d.files}
        q,v,acc=a['position'],a['velocity'],a['acceleration']
        check,C=geometry_audit(robot,q,side,attachments,nominal,g['box_dimensions_m'],metadata['sphere_manifest'],metadata['obstacles'],owner)
        assert q.shape==v.shape==acc.shape and q.shape[1]==15 and np.isfinite(q).all() and np.isfinite(v).all() and np.isfinite(acc).all()
        assert len(a['times'])==len(q) and a['times'][0]==0 and np.all(np.diff(a['times'])>0)
        limits=g['scaled_torque_limits_nm']
        if key=='support_transfer':
            zero=np.zeros_like(q);tl=rnea(robot,q,zero,zero,'left',attachments['left'],g['box_dimensions_m']);tr=rnea(robot,q,zero,zero,'right',attachments['right'],g['box_dimensions_m'])
            tau=(1-a['support_alpha'][:,None])*tl+a['support_alpha'][:,None]*tr
            check['support_extreme_max_scaled_ratio']={s:float(np.max(np.abs(t)/limits)) for s,t in [('left',tl),('right',tr)]}
            check['stationary']=bool(np.max(np.abs(q-q[0]))==0 and np.max(np.abs(v))==0 and np.max(np.abs(acc))==0)
            check['support_alpha_valid']=bool(a['support_alpha'][0]==0 and a['support_alpha'][-1]==1 and np.all(np.diff(a['support_alpha'])>=0))
            check['valid'] &= check['stationary'] and check['support_alpha_valid'] and all(n==len(q) for n in check['cap_permitted_frames'].values())
        else:tau=rnea(robot,q,v,acc,side,attachments[side],g['box_dimensions_m'])
        check['max_box_pose_drift_element']=float(np.max(np.abs(C-C[0])))
        check['valid'] &= check['max_box_pose_drift_element']<=2e-5
        if key in ('donor_release','donor_continue'):
            T=forward(robot,q)[0]['left_tool0'];delta=T[:,:3,3]-T[0,:3,3];axis=-T[0,:3,2];axial=delta@axis
            lateral=np.linalg.norm(delta-axial[:,None]*axis,axis=1)
            angle=Rotation.from_matrix(T[0,:3,:3].T@T[:,:3,:3]).magnitude()
            final_depth=geometry_audit(robot,q[-1:],side,attachments,nominal,g['box_dimensions_m'],metadata['sphere_manifest'],metadata['obstacles'],owner)[0]['minimum_cap_clearance_m']['left']
            check['donor_retreat']=dict(axial_m=float(axial[-1]),minimum_axial_m=float(axial.min()),max_lateral_m=float(lateral.max()),max_angle_deg=float(np.rad2deg(angle.max())),final_cap_clearance_m=final_depth)
            check['valid'] &= bool(axial[-1]>=(.059 if key=='donor_release' else .0099) and axial.min()>=-1e-6 and np.diff(axial).min()>=-1e-5 and lateral.max()<=.002 and angle.max()<=np.deg2rad(1) and final_depth>0)
            if key=='donor_continue':check['valid'] &= check['minimum_cap_clearance_m']['left']>0
        check['native_velocity_jerk_valid']=bool(np.abs(v[:,1:]).max()<=np.deg2rad(60)*.995+1e-5 and np.abs(a['jerk'][:,1:]).max()<=500*.995+1e-4)
        check['valid'] &= check['native_velocity_jerk_valid']
        err=float(np.max(np.abs(tau-a['torque'])))
        ratio=float(np.max(np.abs(tau)/limits))
        check['independent_cpu_rnea']=dict(valid=err<=.002 and ratio<=1.,max_native_difference_nm=err,max_scaled_ratio=ratio)
        check['native_endpoint_error']=float(np.max(np.abs(q[[0,-1]]-g[key][[0,-1]])))
        bound=[]
        for i,name in enumerate(NAMES):
            limit=robot.joint_map[name].limit
            bad=np.flatnonzero((q[:,i]<limit.lower)|(q[:,i]>limit.upper))
            if len(bad):bound.append(dict(joint=name,lower=limit.lower,upper=limit.upper,count=len(bad),first_indices=bad[:12].tolist(),minimum=float(q[:,i].min()),maximum=float(q[:,i].max())))
        check['strict_joint_bound_failures']=bound
        check['rest_boundaries']=bool(np.max(np.abs(v[[0,-1]]))==0 and np.max(np.abs(acc[[0,-1]]))==0)
        check['dynamic_and_physical_valid']=bool(check['valid'] and check['independent_cpu_rnea']['valid'] and check['rest_boundaries'] and check['native_endpoint_error']<=1e-5)
        if previous is not None:
            seam=dict(to_phase=key,q=float(np.max(np.abs(q[0]-previous['position'][-1]))),v=float(np.max(np.abs(v[0]-previous['velocity'][-1]))),a=float(np.max(np.abs(acc[0]-previous['acceleration'][-1]))),box=float(np.max(np.abs(C[0]-previous_C))))
            seam['valid']=max(seam[k] for k in ('q','v','a','box'))<=1e-5
            result['seams'].append(seam);allvalid &= seam['valid'];physicalvalid &= seam['valid']
        check['valid']=bool(check['dynamic_and_physical_valid'] and not bound)
        result['phases'][key]=check;allvalid &= check['valid'];physicalvalid &= check['dynamic_and_physical_valid']
        previous=a;previous_C=C[-1];duration+=float(a['times'][-1]);samples+=len(q)
    result.update(valid=bool(allvalid and len(result['phases'])==4),all_dynamic_and_physical_valid=bool(physicalvalid and len(result['phases'])==4),motion_seconds=duration,samples=samples,wall_seconds=time.perf_counter()-began)
    (folder/'cpu_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    assert result['all_dynamic_and_physical_valid'], 'physical/dynamic/rest gate rejected; inspect cpu_audit.json'
    assert all(all(b['joint']=='updown' and b['minimum']==-1.0000001192092896 for b in stage['strict_joint_bound_failures']) for stage in result['phases'].values()), 'unexpected strict bound failure'
    print(json.dumps({k:v for k,v in result.items() if k!='phases'}))
    for k,v in result['phases'].items():print(k,json.dumps({n:v[n] for n in ('valid','dynamic_and_physical_valid','strict_joint_bound_failures','independent_cpu_rnea')}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);main(p.parse_args().folder)
