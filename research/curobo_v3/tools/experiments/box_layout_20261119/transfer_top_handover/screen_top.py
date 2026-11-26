"""CPU necessary bounds and fresh analytic IK counts; no collision/native success claim."""
from pathlib import Path
from dataclasses import replace
import argparse
import hashlib
import json
import sys
import time
import numpy as np
import yaml
import yourdfpy
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[4];OUT=ROOT/'generated/two_box_layout/joint_contract_20261124_top_handover'
sys.path.insert(0,str(ROOT/'tools'))
from curobo_core.box_pick import BoxPick
from curobo_core.contact_reach import urdf_reach_model,contact_reach_witnesses
from curobo_core.layout_contract import validate_pair,base_front_x
from curobo_core.payload_geometry import box_overlap_depth
from curobo_core.suction_grasp import contact_cap_spheres
from v3_rezero_analytic import RezeroAnalytic
from v3_zero_model_candidate import OFFSETS_DEG,JOINT_SIGNS

def main(y,low,high,label):
    began=time.perf_counter();x=base_front_x()+.66
    document=dict(frame_id='world',front_distance_m=.66,boxes=[dict(size=size,side=side,center_m=[x,y,z],side_min_height_m=.79) for side,size,z in [('left','40x40x30',low),('right','46.5x25.5x58.2',high)]])
    picks={b['side']:BoxPick(**dict(b,carry_joint_home=True)) for b in document['boxes']};contract=validate_pair(picks)
    robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    analytic=RezeroAnalytic();offsets=np.deg2rad([OFFSETS_DEG[s] for s in ('left','right')]);models=urdf_reach_model(robot,offsets,JOINT_SIGNS)
    home=yaml.safe_load((ROOT/'generated/zero_model_candidate_20261018/named_poses.yaml').read_text())['named_poses']['home']
    environment=json.loads((ROOT/'config/tcp_scene_preview.json').read_text())['snapshot']['objects'][:5]
    base=json.loads((ROOT/'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json').read_text())['cuboids']
    obstacles=[dict(name=o['object_id'],pose=[*o['pose']['position'],*o['pose']['quaternion_wxyz']],dims=o['dimensions_m']) for o in environment]+base
    initial={}
    for side,pick in picks.items():
        depths={o['name']:float(box_overlap_depth(np.asarray(pick.center_m)[None],np.eye(3)[None],np.asarray(pick.dimensions_m)/2,np.asarray(o['pose'][:3]),Rotation.from_quat(np.roll(o['pose'][3:7],-1)).as_matrix(),np.asarray(o['dims'])/2)[0]) for o in obstacles}
        assert max(depths.values())<=0.,depths;initial[side]=depths
    box_depth=float(box_overlap_depth(np.asarray(picks['left'].center_m)[None],np.eye(3)[None],np.asarray(picks['left'].dimensions_m)/2,np.asarray(picks['right'].center_m),np.eye(3),np.asarray(picks['right'].dimensions_m)/2)[0]);assert box_depth<=0
    screens={}
    for label2,pick,arm in [('low_left',picks['left'],0),('source_high_left',replace(picks['right'],side='left'),0),('source_high_right',picks['right'],1)]:
        witnesses=contact_reach_witnesses({pick.side:pick},models,(-1.,0.));lifts=[];total=0;fkerror=0.;angleerror=0.
        for lift in np.linspace(-1.,0.,21):
            row=dict(home,updown=float(lift));robot.update_cfg(row);carriage=np.linalg.inv(robot.get_transform('arm_carriage',robot.base_link));target=pick.contact_tool();seed=np.asarray([row[f'{pick.side}_joint{i}'] for i in range(1,8)])
            values=np.concatenate([analytic.solve(arm,carriage@target,np.deg2rad(swivel),seed) for swivel in range(-180,180,10)])
            if not len(values):continue
            total+=len(values);lifts.append(dict(updown_m=float(lift),raw_solutions=len(values)))
            robot.update_cfg(dict(row,**{f'{pick.side}_joint{i}':float(q) for i,q in enumerate(values[0],1)}));T=robot.get_transform(pick.side+'_tool0',robot.base_link)
            ep=float(np.linalg.norm(T[:3,3]-target[:3,3]));er=float(Rotation.from_matrix(T[:3,:3].T@target[:3,:3]).magnitude());assert ep<2e-5 and er<2e-5,(label2,ep,er)
            fkerror=max(fkerror,ep);angleerror=max(angleerror,er)
        screens[label2]=dict(top=bool(pick.top),raw_ik_solutions=total,lifts=lifts,necessary_witnesses=witnesses,max_cpu_urdf_fk_error_m=fkerror,max_orientation_error_rad=angleerror)
    arm=models['right'];direction=arm['direction'];c=arm['cosine'];dy=direction[1]
    upper=arm['upper']*(1. if dy>=c else c*dy+np.sqrt(1-c*c)*np.sqrt(1-dy*dy))
    wrist_upper=float(arm['shoulder'][1]+upper+arm['fore'])
    footprint=contact_cap_spheres('right');extra=float(np.max(footprint[:,2]+footprint[:,3]));envelope=.006+2*np.linalg.norm(arm['tool_offset'])*np.sin(np.deg2rad(.5))+arm['geometry_error_m']
    high_gap=float(y-picks['right'].dimensions_m[1]/2-np.linalg.norm(arm['tool_offset'])-extra-wrist_upper-envelope)
    bindings=[]
    for size,z in [('46.5x25.5x58.2',1.4),('46.5x25.5x58.2',.7)]:
        source=BoxPick(size,(x,y,z),'left',side_min_height_m=.79);A=source.tool_to_box()
        for center in ((.85,.2,1.2),(.90,.15,1.2),(.95,.10,1.25)):
            C=np.eye(4);C[:3,3]=center;actual=C@np.linalg.inv(A);old=BoxPick(size,center,'left',side_min_height_m=.79).contact_tool()
            error=float(np.max(np.abs(actual@A-C)));assert error<1e-12
            if not source.top:assert np.array_equal(actual,old)
            bindings.append(dict(source_top=bool(source.top),center_m=center,preserved_attachment_box_error=error,old_mid_target_difference=float(np.max(np.abs(actual-old)))))
    result=dict(request=document,contract=contract,initial_world_base_depths_m=initial,initial_box_box_depth_m=box_depth,contact_screens=screens,
        right_any_contact_necessary_bound=dict(wrist_y_upper_m=wrist_upper,extra_cap_reach_m=extra,pose_envelope_m=float(envelope),high_box_any_contact_gap_m=high_gap,proven_initial_right_unreachable=high_gap>1e-5),
        donor_binding_checks=bindings,construction_success=None,scope='CPU initial OBB/world/base, continuous necessary right support bound and finite fresh analytic IK only. No self/world collision path or static/native success claim.',
        executable=sys.executable,wall_seconds=time.perf_counter()-began,model_sha256=hashlib.sha256((ROOT/'generated/fast_runtime_assets/fixed.urdf').read_bytes()).hexdigest())
    (OUT/(label+'_request.json')).write_text(json.dumps(document,indent=2)+'\n');(OUT/(label+'_cpu_screen.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(label=label,request=document,raw_ik_counts={k:v['raw_ik_solutions'] for k,v in screens.items()},necessary_witnesses={k:v['necessary_witnesses'] for k,v in screens.items()},right_any_contact_gap=high_gap,wall_seconds=result['wall_seconds']),indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--y',type=float,default=.85);p.add_argument('--low',type=float,default=.7);p.add_argument('--high',type=float,default=1.3);p.add_argument('--label',default='y85_top70_high130');a=p.parse_args();main(a.y,a.low,a.high,a.label)
