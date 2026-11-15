import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import torch
import yourdfpy

from curobo_core.scene import Pose
from v3_batched_loaded_search import GpuValidity
from v3_rezero_task_planner import FOLDER, RezeroTaskPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source-dir',type=Path,required=True)
    args=parser.parse_args()
    planner=RezeroTaskPlanner()
    model=yourdfpy.URDF.load(FOLDER/'robot_rezero_candidate.urdf',load_meshes=False)
    reports=[]
    for path in sorted(args.source_dir.glob('L*.json')):
        result=json.loads(path.read_text())
        assert result['success'],path
        planner.configure_task(result['task'])
        snapshot=planner.snapshot
        names=result['joint_names']
        rows=np.asarray(result['frames'])
        indices=[names.index(name) for name in ACTIVE_JOINTS]
        max_step=float(np.rad2deg(np.abs(np.diff(rows[:,indices[1:]],axis=0))).max())
        assert max_step<=.5001,(path,max_step)
        assert np.max(np.abs(rows[-1,indices]-planner.home_values))<1e-6
        phases=np.asarray(result['phases'])
        checked=0
        for phase in ('home','approach','contact_approach','extract','transport','conveyor_place','release','return_home'):
            selected=rows[phases==phase]
            if not len(selected):
                continue
            if phase!='home':
                assert np.max(np.abs(selected[:,:3]-selected[:1,:3]))<1e-8,(path.name,phase,'base must stay frozen')
            yaw=selected[0,names.index('base_yaw')]
            base=Pose((selected[0,names.index('base_x')]-planner.base_advance,selected[0,names.index('base_y')],0.),
                      (float(np.cos(yaw/2)),0.,0.,float(np.sin(yaw/2))))
            planner.snapshot=replace(snapshot,state=snapshot.state.with_positions(dict(zip(names,selected[0])),base))
            checker=planner.checker(result['task'],payload=phase in ('transport','conveyor_place'))
            validity=(planner.target_validity(checker)
                      if phase=='approach' else GpuValidity(checker,check_ground=True))
            for offset in range(0,len(selected),512):
                values=torch.tensor(selected[offset:offset+512,indices],device='cuda',dtype=torch.float32).contiguous()
                mask=validity.mask(values)
                assert bool(mask.all()),(path.name,phase,offset,torch.where(~mask)[0].tolist())
            checked+=len(selected)
            if phase=='extract':
                loaded=GpuValidity(planner.checker(result['task'],payload=True),check_ground=True)
                assert bool(loaded.mask(torch.tensor(selected[-1:,indices],device='cuda',dtype=torch.float32).contiguous()).all())
        errors={}
        for side in result['task']:
            matrices=[]
            extract_indices=np.where(phases=='extract')[0]
            for row in rows[[extract_indices[0]-1,extract_indices[-1]]]:
                values=dict(zip(names,row))
                model.update_cfg({name:values.get(name,0.) for name in model.actuated_joint_names})
                matrices.append(model.get_transform(side+'_tool0',model.base_link).copy())
            displacement=matrices[1][:3,3]-matrices[0][:3,3]
            assert np.linalg.norm(displacement-[-.35,0.,0.])<.002,(path,side,displacement)
            row=rows[np.where(phases=='conveyor_place')[0][-1]]
            values=dict(zip(names,row))
            model.update_cfg({name:values.get(name,0.) for name in model.actuated_joint_names})
            world=np.eye(4)
            world[:3,:3]=Rotation.from_euler('z',values['base_yaw']).as_matrix()
            world[:2,3]=[values['base_x'],values['base_y']]
            box=world@model.get_transform(side+'_tool0',model.base_link)@np.asarray(result['tool_to_box'][side])
            target=result['conveyor_box_targets'][side]
            position_error=float(np.linalg.norm(box[:3,3]-target['position']))
            rotation_error=float((Rotation.from_quat(np.roll(target['quaternion_wxyz'],-1)).inv()*Rotation.from_matrix(box[:3,:3])).magnitude())
            assert position_error<.002 and rotation_error<np.deg2rad(1.),(path,side,position_error,rotation_error)
            errors[side]=dict(place_error_mm=position_error*1000,place_error_deg=float(np.rad2deg(rotation_error)))
        planner.snapshot=snapshot
        reports.append(dict(file=path.name,checked_frames=checked,max_adjacent_step_deg=max_step,
                            home_max_error=float(np.max(np.abs(rows[-1,indices]-planner.home_values))),boxes=errors,
                            collision_scope='original policy: final contact excludes targets; extraction checks empty robot throughout plus loaded endpoint; transport/place loaded throughout'))
        print('AUDIT',reports[-1],flush=True)
    (args.source_dir/'audit.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
