"""CPU root test: old Side state regression and fresh analytic source-Top warm starts."""
from pathlib import Path
import hashlib
import json
import sys
import time
import numpy as np
import yaml
import yourdfpy
ROOT=Path(__file__).resolve().parents[4];OUT=ROOT/'generated/two_box_layout/joint_contract_20261124_top_handover'
sys.path[:0]=[str(ROOT/'tools'),str(Path(__file__).resolve().parent.parent/'transfer_fresh'),str(Path(__file__).resolve().parent.parent/'transfer_native')]
from plan_fresh import donor_retreat,dense,NAMES
from physical_cpu import forward,geometry_audit
from v3_rezero_analytic import RezeroAnalytic
from curobo_core.box_pick import BoxPick


def inspect(robot,start):
    began=time.perf_counter();rows,detail=donor_retreat(robot,start)
    result=dict(ik=detail,wall_seconds=time.perf_counter()-began)
    if rows is not None:
        path=dense(rows);poses=forward(robot,path)[0]['left_tool0'];delta=poses[:,:3,3]-poses[0,:3,3];axis=-poses[0,:3,2];axial=delta@axis
        lateral=np.linalg.norm(delta-axial[:,None]*axis,axis=1)
        assert axial[-1]>=.06999 and axial.min()>=-1e-6 and np.diff(axial).min()>=-1e-5 and lateral.max()<=.002
        bounds=np.asarray([[robot.joint_map[n].limit.lower,robot.joint_map[n].limit.upper] for n in NAMES]).T
        assert np.all(path>=bounds[0]) and np.all(path<=bounds[1])
        assert np.array_equal(path[:,8:],np.repeat(start[None,8:],len(path),axis=0)) and np.all(path[:,0]==start[0])
        result.update(samples=len(path),normal_retreat_m=float(axial[-1]),max_lateral_m=float(lateral.max()),donor_J7_motion_rad=float(np.ptp(path[:,7])))
    return result


def main():
    robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    # Existing states are CPU regression fixtures only; fresh construction never calls this file.
    with np.load(ROOT/'generated/two_box_layout/joint_contract_20261124_transfer/transfer_geometry.npz',allow_pickle=False) as data:side=data['donor_release'][0].copy()
    result=dict(side=inspect(robot,side),source_top=[])
    assert result['side']['ik']['success']
    request=json.loads((OUT/'y85_both_top20_high70_request.json').read_text());source=BoxPick(**dict(next(b for b in request['boxes'] if b['side']=='right'),side='left'));C=np.eye(4);C[:3,3]=(.85,.2,1.2);T=C@np.linalg.inv(source.tool_to_box());analytic=RezeroAnalytic()
    home=yaml.safe_load((ROOT/'generated/zero_model_candidate_20261018/named_poses.yaml').read_text())['named_poses']['home'];seed=np.asarray([home[f'left_joint{i}'] for i in range(1,8)])
    for lift in (-.7,-.65,-.6):
        row=dict(home,updown=lift);robot.update_cfg(row);carriage=np.linalg.inv(robot.get_transform('arm_carriage',robot.base_link));values=np.concatenate([analytic.solve(0,carriage@T,np.deg2rad(angle),seed) for angle in range(-180,180,10)]);values=np.unique(np.round(values,7),axis=0);values=values[np.argsort(np.linalg.norm(values-seed,axis=1))]
        q=np.asarray([row[n] for n in NAMES],dtype=np.float32);q[1:8]=values[0]
        result['source_top'].append(dict(lift=lift,first_sorted_candidate=inspect(robot,q)))
    result['source_sha256']=hashlib.sha256((ROOT/'tools/experiments/box_layout_20261119/transfer_fresh/plan_fresh.py').read_bytes()).hexdigest()
    (OUT/'retreat7_cpu_attempt01.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Side',result['side']['ik']['success'],'normal',result['side'].get('normal_retreat_m'),'J7 motion',result['side'].get('donor_J7_motion_rad'))
    for case in result['source_top']:
        t=case['first_sorted_candidate'];print('source Top',case['lift'],t['ik']['success'],t['ik'].get('failed_distance_m'),t['ik'].get('steps',[])[-1:],t['wall_seconds'])

if __name__=='__main__':main()
