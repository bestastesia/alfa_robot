"""CPU-only real cap geometry checks of the shared function, without importing GPU planners."""
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
import ast
import json
import sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[4]
OUT=ROOT/'generated/two_box_layout/joint_contract_20261124_top_handover'
sys.path.insert(0,str(ROOT/'tools'))
from curobo_core.box_pick import BoxPick
from curobo_core.suction_grasp import contact_cap_spheres

def extract(path,name='validate_attachment'):
    tree=ast.parse(path.read_text());node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
    namespace=dict(np=np,contact_cap_spheres=contact_cap_spheres)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),namespace)
    return namespace[name]

def fake(pick,T):
    return SimpleNamespace(side=pick.side,dimensions_m=pick.dimensions_m,tool_to_box=lambda:np.linalg.inv(T))

def main():
    current=extract(ROOT/'tools/experiments/box_layout_20261119/transfer_suffix/probe_suffix.py');before=extract(OUT/'validate_attachment_before.py');tests={}
    sides=[BoxPick('46.5x25.5x58.2',(.85,.2,1.2),'left',side_min_height_m=.79),BoxPick('46.5x25.5x58.2',(.85,.2,1.2),'right',side_min_height_m=.79,side_face='y-',suction_roll_deg=90.)]
    for p in sides:
        a,b=before(p),current(p);assert a==b;tests[p.side+'_original_side']=dict(identical=True,result=b)
    from curobo_core.adapter import pose_matrix
    prior=json.loads((ROOT/'generated/two_box_layout/joint_contract_20261124_fresh_native/attempt_01/record.json').read_text())
    for key in ('extract','donor_release'):
        stage=next(s for s in prior['stage_interface']['stages'] if s['stage_id']==key)
        attachment=next(a for a in stage['scene']['attachments'] if a['object_id']=='wall_box_01')
        side=attachment['parent_link'].removesuffix('_tool0');base=next(p for p in sides if p.side==side)
        actual=fake(base,np.linalg.inv(pose_matrix(__import__('curobo_core.scene',fromlist=['Pose']).Pose(**attachment['tool_to_object']))))
        old,new=before(actual),current(actual);assert old==new
        tests[side+'_actual_Y85_attachment_regression']=dict(identical=True,result=new,purpose='Prior interface read for CPU function regression only, never fresh construction input')
    for side in ('left','right'):
        for face,roll in [('x+',0.),('y+',90.)]:
            p=BoxPick('46.5x25.5x58.2',(.85,.2,1.2),side,side_min_height_m=.79,side_face=face,suction_roll_deg=roll)
            tests[side+'_'+face]=dict(passes=True,result=current(p))
    for side in ('left','right'):
        p=BoxPick('40x40x30',(1.168052051,.85,.7),side,side_min_height_m=.79)
        top=current(p);tests[side+'_top_roll0']=dict(passes=True,result=top)
        try:before(p)
        except AssertionError:pass
        else:raise AssertionError('original arm-axis bug did not reproduce')
        T=np.linalg.inv(p.tool_to_box());bad=T.copy();bad[2,3]+=.0011
        cases={'wrong_plane_1p1mm':bad}
        caps=contact_cap_spheres(side);centers=caps[:,:3]@T[:3,:3].T+T[:3,3]
        bad=T.copy();bad[0,3]+=.15-np.max(centers[:,0]+caps[:,3])+.0011;cases['footprint_outside_1p1mm']=bad
        bad=T.copy();bad[:3,:3]=Rotation.from_euler('y',1.01,degrees=True).as_matrix()@bad[:3,:3];cases['normal_tilt_1p01deg']=bad
        for name,bad in cases.items():
            try:current(fake(p,bad))
            except AssertionError:tests[side+'_'+name]=dict(rejected=True)
            else:raise AssertionError('invalid real cap accepted '+side+' '+name)
        try:replace(p,suction_roll_deg=90.)
        except ValueError:tests[side+'_roll90']=dict(footprint_rejected=True)
        else:raise AssertionError('unexpected roll90 fit')
    result=dict(valid=True,tests=tests,policy=dict(plane_tolerance_m=.001,footprint_tolerance_m=.001,normal_tolerance_deg=1.),default_top_roll_unchanged=0.,executable=sys.executable)
    (OUT/'attachment_cpu.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
