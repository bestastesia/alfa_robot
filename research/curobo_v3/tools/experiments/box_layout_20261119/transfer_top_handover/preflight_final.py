"""CPU actual-box clearance at original Home and fixed14-angle endpoint candidates."""
from pathlib import Path
import json
import sys
import numpy as np
import yaml
import yourdfpy
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[4];OUT=ROOT/'generated/two_box_layout/joint_contract_20261124_top_handover'
sys.path[:0]=[str(ROOT/'tools'),str(Path(__file__).resolve().parent.parent/'transfer_native'),str(Path(__file__).resolve().parent.parent/'transfer_suffix')]
from curobo_core.box_pick import BoxPick
from curobo_core.payload_geometry import box_sphere_depth,box_overlap_depth
from curobo_core.layout_contract import CARRY_DEGREES
from curobo_core.suction_grasp import contact_cap_spheres
from physical_cpu import forward,NAMES
from audit_suffix import gravity

def main():
    request=json.loads((OUT/'y85_top70_high130_request.json').read_text());picks={b['side']:BoxPick(**b) for b in request['boxes']}
    robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False);cfg=yaml.safe_load((ROOT/'generated/fast_runtime_assets/fixed.yaml').read_text())['kinematics']
    home=yaml.safe_load((ROOT/'generated/zero_model_candidate_20261018/named_poses.yaml').read_text())['named_poses']['home']
    def spheres(poses):
        values=[];capids={s:[] for s in ('left','right')}
        for link in cfg['collision_link_names']:
            if link=='base_link':continue  # Exact four base cuboids are checked separately.
            origin=robot.get_transform(link.removesuffix('_link7')+'_tool0',link)[2,3] if link in ('left_link7','right_link7') else None
            for sphere in cfg['collision_spheres'][link]:
                if sphere['radius']<=0:continue
                T=poses[link];center=np.einsum('nij,j->ni',T[:,:3,:3],sphere['center'])+T[:,:3,3];values.append(np.column_stack((center,np.full(len(T),sphere['radius']))))
                if origin is not None and sphere['center'][2]+sphere['radius']>origin:capids[link.removesuffix('_link7')].append(len(values)-1)
        return np.stack(values,axis=1),capids
    qhome=np.asarray([[home[n] for n in NAMES]]);homeposes=forward(robot,qhome)[0];S,_=spheres(homeposes)
    initial={s:float(box_sphere_depth(S[...,:3],S[...,3],np.asarray(p.center_m)[None],np.eye(3)[None],np.asarray(p.dimensions_m)/2).max()) for s,p in picks.items()}
    assert max(initial.values())<=1e-6,initial
    low=picks['left'];high=BoxPick(picks['right'].size,(.85,.2,1.2),'right',side_min_height_m=.79,side_face='y-',suction_roll_deg=90.)
    attachments={'left':low.tool_to_box(),'right':high.tool_to_box()};rows=np.tile(np.r_[0.,np.deg2rad(np.r_[CARRY_DEGREES,-CARRY_DEGREES])],(101,1));rows[:,0]=np.linspace(-1.,0.,101);poses=forward(robot,rows)[0];S,caps=spheres(poses)
    boxes={s:poses[s+'_tool0']@A for s,A in attachments.items()};half={s:np.asarray(picks[s].dimensions_m)/2 for s in picks};valid=np.ones(len(rows),dtype=bool);own={}
    world=json.loads((ROOT/'config/tcp_scene_preview.json').read_text())['snapshot']['objects'][:5];obstacles=[dict(name=o['object_id'],pose=[*o['pose']['position'],*o['pose']['quaternion_wxyz']],dims=o['dimensions_m']) for o in world]
    for raw in json.loads((ROOT/'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json').read_text())['cuboids']:
        T=np.eye(4);T[:3,3]=raw['pose'][:3];T[:3,:3]=Rotation.from_quat(np.roll(raw['pose'][3:7],-1)).as_matrix();T=poses['base_link'][0]@T
        obstacles.append(dict(name=raw['name'],pose=[*T[:3,3],*np.roll(Rotation.from_matrix(T[:3,:3]).as_quat(),1)],dims=raw['dims']))
    for side,C in boxes.items():
        depth=box_sphere_depth(S[...,:3],S[...,3],C[:,:3,3],C[:,:3,:3],half[side]);ids=[i for i in range(S.shape[1]) if i not in caps[side]]
        own[side]=float(depth[:,ids].max());valid&=(depth[:,ids]<=1e-6).all(-1)
        for o in obstacles:
            d=box_overlap_depth(C[:,:3,3],C[:,:3,:3],half[side],np.asarray(o['pose'][:3]),Rotation.from_quat(np.roll(o['pose'][3:7],-1)).as_matrix(),np.asarray(o['dims'])/2);valid&=d<=.0010001
    d=box_overlap_depth(boxes['left'][:,:3,3],boxes['left'][:,:3,:3],half['left'],boxes['right'][:,:3,3],boxes['right'][:,:3,:3],half['right']);valid&=d<=.0010001
    dynamics=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/dynamics.urdf',load_meshes=False);tau=gravity(dynamics,rows[0],attachments);limits=np.asarray([294,294,178,178,87,52,10]*2)*.995;ratio=float(np.max(np.abs(tau)/limits))
    result=dict(initial_home_max_box_robot_depth_m=initial,final_payload_robot_max_noncap_depth_m=own,final_payload_only_candidate_lifts=rows[valid,0].tolist(),static_scaled_ratio=ratio,
        scope='Home actual robot spheres vs original boxes; necessary final payload OBB/other box/world/base/ground and actual static gravity at fixed14 angles. Robot self/world endpoint and all paths remain unproven; no success claim.')
    (OUT/'top_final_cpu_preflight.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
