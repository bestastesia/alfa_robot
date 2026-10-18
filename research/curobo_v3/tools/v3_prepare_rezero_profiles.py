import itertools
import json
from pathlib import Path

import numpy as np
import torch
from curobo.types import JointState
from curobo.collision_checking import RobotCollisionChecker,RobotCollisionCheckerCfg
from curobo.scene import Scene
import yaml
import yourdfpy

from v3_zero_model_candidate import ROOT,OLD,build,OFFSETS_DEG,J2_LIMITS_DEG,JOINT_SIGNS
from v3_rezero_analytic import RezeroAnalytic


def convert_row(names,values):
    result=np.asarray(values).copy()
    for side in ('left','right'):
        indices=[names.index(f'{side}_joint{index}') for index in range(1,8)]
        result[...,indices]=(result[...,indices]-np.deg2rad(OFFSETS_DEG[side]))*JOINT_SIGNS
        result[...,indices]=(result[...,indices]+np.pi)%(2*np.pi)-np.pi
    return result


def main():
    folder=ROOT/'generated/zero_model_candidate_20261018'
    fixed=build(folder)
    mobile=build(folder/'mobile',ROOT/'generated/v3_analytic_071cb95/mobile18/model/robot_meter_meshes_mobile18.urdf')
    profiles={}
    for label,source,urdf in [('fixed',ROOT/'generated/v3_analytic_071cb95/alfa_v322_suction_final.yml',fixed),
                               ('mobile',ROOT/'generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml',mobile)]:
        config=yaml.safe_load(source.read_text())
        config['kinematics']['urdf_path']=str(urdf)
        names=config['kinematics']['cspace']['joint_names']
        if config['kinematics']['cspace'].get('nominal_joint_position') is not None:
            config['kinematics']['cspace']['nominal_joint_position']=convert_row(names,config['kinematics']['cspace']['nominal_joint_position']).tolist()
        profiles[label]=config
    old_model=yourdfpy.URDF.load(OLD,load_meshes=False)
    solver=RezeroAnalytic()
    named=yaml.safe_load((ROOT/'models/robot_description_analytic_071cb95/config/named_poses_analytic_proxy.yaml').read_text())
    reports=[]
    for name,pose in named['named_poses'].items():
        names=list(pose);converted=convert_row(names,list(pose.values()))
        output=dict(zip(names,converted.tolist()))
        if any(not np.deg2rad(J2_LIMITS_DEG[side][0])-1e-8<=output[f'{side}_joint2']<=np.deg2rad(J2_LIMITS_DEG[side][1])+1e-8 for side in ('left','right')):
            old_model.update_cfg({joint:pose.get(joint,0.) for joint in old_model.actuated_joint_names})
            inverse=np.linalg.inv(old_model.get_transform('arm_carriage',old_model.base_link))
            arms=[]
            for arm,side in enumerate(('left','right')):
                target=inverse@old_model.get_transform(side+'_tool0',old_model.base_link)
                seed=np.array([output[f'{side}_joint{index}'] for index in range(1,8)])
                values=[]
                for swivel in np.deg2rad(np.arange(-180,180,5)):
                    values.extend(solver.solve(arm,target,swivel,seed))
                if not values:
                    reports.append(dict(name=name,success=False,side=side,reason='No equivalent TCP within new limits'))
                    print('BLOCKED',reports[-1],flush=True)
                    (folder/'named_pose_migration.json').write_text(json.dumps(reports,indent=2))
                    return
                values=np.unique(np.round(values,8),axis=0)
                values=values[np.argsort(np.linalg.norm(values-seed,axis=1))[:32]]
                arms.append(values)
            config=profiles['fixed'];joint_names=config['kinematics']['cspace']['joint_names']
            checker=RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(robot_config=config,scene_model=Scene(),n_cuboids=1,n_meshes=0,collision_activation_distance=0.0))
            candidates=[]
            for left,right in itertools.product(*arms):
                mapping=output.copy()
                mapping.update({f'left_joint{index}':value for index,value in enumerate(left,1)})
                mapping.update({f'right_joint{index}':value for index,value in enumerate(right,1)})
                candidates.append([mapping[joint] for joint in joint_names])
            candidates=np.asarray(candidates,dtype=np.float32)
            tensor=torch.tensor(candidates,device='cuda').contiguous()
            state=checker.kinematics.compute_kinematics(JointState.from_position(tensor[None],joint_names=joint_names))
            checker.setup_batch_tensors(1,len(candidates))
            valid=(checker.get_self_collision(state.robot_spheres).reshape(len(candidates),-1).sum(-1)<=0)&(checker.get_bound(tensor[None]).reshape(len(candidates),-1).sum(-1)<=0)
            accepted=candidates[valid.cpu().numpy()]
            if not len(accepted):
                self_cost=checker.get_self_collision(state.robot_spheres).reshape(len(candidates),-1).sum(-1)
                bound_cost=checker.get_bound(tensor[None]).reshape(len(candidates),-1).sum(-1)
                reports.append(dict(name=name,success=False,reason='Equivalent TCP candidates all fail native collision/bounds',
                    checked=len(candidates),self_collision_free=int((self_cost<=0).sum()),bound_valid=int((bound_cost<=0).sum()),
                    minimum_self_cost=float(self_cost.min()),minimum_bound_cost=float(bound_cost.min())))
                (folder/'named_pose_migration.json').write_text(json.dumps(reports,indent=2));print('BLOCKED',reports[-1],flush=True);return
            reference=np.array([output[joint] for joint in joint_names])
            chosen=accepted[np.linalg.norm(accepted-reference,axis=1).argmin()]
            output.update(dict(zip(joint_names,chosen.tolist())))
        named['named_poses'][name]=output
        reports.append(dict(name=name,success=True,left_deg=np.rad2deg([output[f'left_joint{i}'] for i in range(1,8)]).tolist(),
            right_deg=np.rad2deg([output[f'right_joint{i}'] for i in range(1,8)]).tolist()))
        print('POSE',name,reports[-1],flush=True)
    home_policy=json.loads((ROOT/'config/rezero_home.json').read_text())
    named['named_poses']['home']['updown']=home_policy['updown_m']
    for side in ('left','right'):
        named['named_poses']['home'].update({f'{side}_joint{index}':float(np.deg2rad(value)) for index,value in enumerate(home_policy[side+'_deg'],1)})
    for report in reports:
        if report['name']=='home':
            report['coordinate_converted_previous_home']={key:report[key] for key in ('left_deg','right_deg')}
            report.update(left_deg=home_policy['left_deg'],right_deg=home_policy['right_deg'],
                          updown_m=home_policy['updown_m'],source=home_policy['source'])
    named['metadata']['model_revision']='rezero-20261018-asymmetric-j2-reversed-j6'
    named['metadata']['home_source']='config/rezero_home.json: explicitly selected new Home, not a coordinate redefinition'
    (folder/'named_poses.yaml').write_text(yaml.safe_dump(named,sort_keys=False))
    (folder/'named_pose_migration.json').write_text(json.dumps(reports,indent=2))
    for label,config in profiles.items():
        config['kinematics']['cspace'].pop('retract_config',None)
        (folder/f'{label}.yaml').write_text(yaml.safe_dump(config,sort_keys=False))


if __name__=='__main__':main()
