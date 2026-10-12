import argparse
import copy
import json
import math
import time
from pathlib import Path

import numpy as np
import torch
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.types import JointState
from curobo_core.backend import goal
from v3_endpoint_optimizer_compare import EndpointObjective, EnvelopeCostConfig, EnvelopeCostManager, unique_legal
from v3_task_cycle_planner import TaskCyclePlanner
from v3_base_mesh_layer import BaseMeshLayer, replace_arm_base_pairs
from v3_batched_loaded_search import loaded_robot
from v3_interactive_60_tasks import DEFAULT_ROBOT_CONFIG, DEFAULT_URDF, DEFAULT_NAMED_POSES, DEFAULT_BOX_FIT, DEFAULT_MOBILE_ROBOT_CONFIG
from v3_wall_ik_benchmark import ACTIVE_JOINTS


class NativeBaseSpheres:
    def cost(self,state,hard=False):
        return state.robot_spheres[...,0,0].unsqueeze(-1)*0


def main():
    folder=Path('generated/endpoint_optimizer_compare/base_geometry_compare')
    folder.mkdir(parents=True,exist_ok=True)
    source=json.loads(Path('generated/extract_to_place/live/1791452385002705270_side.json').read_text())
    start=torch.tensor(source['start'],device='cuda',dtype=torch.float32)
    variants={}
    for name in ['mesh','spheres']:
        planner=TaskCyclePlanner(argparse.Namespace(robot_config=DEFAULT_ROBOT_CONFIG,urdf=DEFAULT_URDF,named_poses=DEFAULT_NAMED_POSES,box_fit=DEFAULT_BOX_FIT,mobile_robot_config=DEFAULT_MOBILE_ROBOT_CONFIG))
        planner.configure_task(source['task'])
        if name=='mesh':
            replace_arm_base_pairs(planner.robot)
        checker=planner.checker(source['task'],payload=True)
        checker.base_mesh_layer=BaseMeshLayer(checker,planner.robot) if name=='mesh' else NativeBaseSpheres()
        objective=EndpointObjective(checker,source['target_tool_poses'],start)
        robot=loaded_robot(planner.robot,planner.box_fit,attachments=planner.task_attachments(source['task']))
        config=InverseKinematicsCfg.create(robot=copy.deepcopy(robot),scene_model=planner.scene(source['task']),collision_cache={'cuboid':max(40,len(planner.snapshot.objects)),'mesh':len(planner.snapshot.meshes)},num_seeds=512,self_collision_check=True,use_cuda_graph=False,position_tolerance=.002,orientation_tolerance=math.radians(1),override_iters_for_multi_link_ik=160,optimizer_collision_activation_distance=.005,cost_manager_config_instance_type=EnvelopeCostConfig)
        config.exit_early=False
        solver=InverseKinematics(config)
        pairs=checker.kinematics.get_self_collision_config().collision_pairs.long()
        kinematics=checker.kinematics.config.kinematics_config
        base_indices=kinematics.get_sphere_index_from_link_name('base_link')
        arm_indices=torch.cat([kinematics.get_sphere_index_from_link_name(side+'_link'+str(index)) for side in ['left','right'] for index in range(1,8)])
        base_arm_pairs=int(((torch.isin(pairs[:,0],base_indices)&torch.isin(pairs[:,1],arm_indices))|(torch.isin(pairs[:,1],base_indices)&torch.isin(pairs[:,0],arm_indices))).sum())
        if name=='spheres':
            assert base_arm_pairs>0
        variants[name]=(objective,solver)
        variants[name+'_pairs']=base_arm_pairs
    torch.manual_seed(621648851)
    objective=variants['mesh'][0]
    seeds=objective.lower+(objective.upper-objective.lower)*torch.rand(1024,15,device='cuda')
    seeds[0]=start
    report={'full_explicit_lm_seeds':1024,'optimizer_seeds':512,'base_arm_sphere_pairs':{name:variants[name+'_pairs'] for name in ['mesh','spheres']},'warmup':[],'runs':[],'default_geometry_changed':False}
    def solve(name,repeat):
        objective,solver=variants[name]
        EnvelopeCostManager.objective=objective
        torch.cuda.synchronize()
        began=time.perf_counter()
        result=solver.solve_pose(goal(source['target_tool_poses']),current_state=JointState.from_position(start[None],joint_names=ACTIVE_JOINTS),seed_config=seeds[None],return_seeds=512)
        torch.cuda.synchronize()
        elapsed=(time.perf_counter()-began)*1000
        names=result.js_solution.joint_names
        values=result.js_solution.position.reshape(-1,len(names))[:,[names.index(joint) for joint in ACTIVE_JOINTS]]
        legal,diagnostics=unique_legal(values,objective)
        cross_pass=None
        if len(legal):
            cross_pass=int(variants['mesh'][0].audit(legal[:64])[0].sum())
        entry={'geometry':name,'repeat':repeat,'solve_ms':elapsed,'distinct_legal':len(legal),'diagnostics':diagnostics,'selected_count':min(64,len(legal)),'selected_pass_mesh_full_audit':cross_pass}
        (report['warmup'] if repeat<0 else report['runs']).append(entry)
        (folder/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(entry,flush=True)
        if repeat==0:
            (folder/(name+'_candidates.json')).write_text(json.dumps({'candidates':legal[:64].cpu().tolist(),'joint_names':ACTIVE_JOINTS},indent=2))
    for name in ['mesh','spheres']:
        solve(name,-1)
    for repeat in range(3):
        for name in ['mesh','spheres']:
            solve(name,repeat)
    report['mean_ms']={name:float(np.mean([item['solve_ms'] for item in report['runs'] if item['geometry']==name])) for name in ['mesh','spheres']}
    (folder/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print('summary',report['mean_ms'],flush=True)


if __name__=='__main__':
    main()
