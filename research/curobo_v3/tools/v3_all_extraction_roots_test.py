import argparse,json,time,gc,math
import hashlib
from pathlib import Path
import numpy as np
import torch
from curobo.types import JointState
from curobo_core.backend import goal
from v3_cuboid_full_cycle_planner import BallFullCyclePlanner
from v3_batched_loaded_search import GpuValidity,densify
from v3_multiroot_search import batched_rrt_connect_multi_goal
from v3_box_rotation_cost import BoxRotationCost,quaternion_multiply,rotation_vector
from v3_wall_ik_benchmark import ACTIVE_JOINTS
from v3_interactive_60_tasks import DEFAULT_ROBOT_CONFIG,DEFAULT_URDF,DEFAULT_NAMED_POSES,DEFAULT_BOX_FIT,DEFAULT_MOBILE_ROBOT_CONFIG
from curobo_core.planner import CycleBlocked
from v3_goal_matching import match_goals


from v3_research_assets import ROOT, planner_identity

def main(planner=None,task=None,prepare_only=False):
    folder=(ROOT/'generated/all_extraction_roots_20261009');folder.mkdir(parents=True,exist_ok=True)
    p=planner if planner is not None else BallFullCyclePlanner(argparse.Namespace(robot_config=DEFAULT_ROBOT_CONFIG,urdf=DEFAULT_URDF,named_poses=DEFAULT_NAMED_POSES,box_fit=DEFAULT_BOX_FIT,mobile_robot_config=DEFAULT_MOBILE_ROBOT_CONFIG));boxes=task or {'left':22,'right':17}
    if not prepare_only:p.configure_task(boxes)
    began=time.perf_counter();empty=p.checker(boxes);loaded=p.checker(boxes,payload=True)
    empty_validity=GpuValidity(empty,check_ground=True)
    loaded_validity=GpuValidity(loaded,check_ground=True)
    solver=p.ik_solver(boxes)
    torch.cuda.synchronize();setup_ms=(time.perf_counter()-began)*1000
    goal_pose=goal(p.contact_poses_for_task(boxes));home=JointState.from_position(torch.tensor(p.home_values[None],device='cuda'),joint_names=ACTIVE_JOINTS)
    if not prepare_only:
        with torch.enable_grad():solver.solve_pose(goal_pose,home,return_seeds=32)
    torch.cuda.synchronize();began=time.perf_counter()
    with torch.enable_grad():solved=solver.solve_pose(goal_pose,home,return_seeds=32)
    torch.cuda.synchronize();ik_ms=(time.perf_counter()-began)*1000
    names=solved.js_solution.joint_names
    contacts=solved.js_solution.position.reshape(-1,len(names))[solved.success.reshape(-1).bool()][:,[names.index(name) for name in ACTIVE_JOINTS]].cpu().numpy()
    if not prepare_only:torch.set_grad_enabled(False)
    rows=[];attempts=[];began=time.perf_counter()
    for index,contact in enumerate(contacts):
        path,detail=p.analytic_extract(contact,empty_validity,loaded_validity)
        attempts.append({'contact_index':index,'success':path is not None,'detail':detail})
        if path is not None:
            rows.append({'contact_index':index,'contact':contact.tolist(),'extraction_frames':path.tolist(),'start':path[-1].tolist()})
        print('extract',index,path is not None,flush=True)
    torch.cuda.synchronize();extract_ms=(time.perf_counter()-began)*1000
    if not rows:
        partial={'success':False,'task':boxes,'frames':[],'phases':[],'payload':[],'contact_ik_count':len(contacts),'attempts':attempts,'candidate_preparation_timing_ms':{'prepare_checkers':setup_ms,'contact_ik':ik_ms,'all_extract_and_validation':extract_ms}}
        raise CycleBlocked('抽离候选验收',f'{len(contacts)}个接触候选均未通过解析抽离及验收',partial)
    mode='top' if p.top_grasp else 'side'
    signature=planner_identity(p, boxes)[:16]
    cache=(ROOT/'generated/placement_analytic_atlas_collision_free/tasks')/f"{mode}_L{boxes['left']}_R{boxes['right']}_{signature}.npz"
    if not cache.exists():
        raw=dict(np.load((ROOT/'generated/placement_analytic_atlas')/(mode+'.npz')))
        masks=[]
        for offset in range(0,len(raw['joints']),1024):
            masks.append(loaded_validity.mask(torch.tensor(raw['joints'][offset:offset+1024],device='cuda',dtype=torch.float32)).cpu().numpy())
        mask=np.concatenate(masks)
        cache.parent.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(cache,**{key:value[mask] for key,value in raw.items()})
    atlas=dict(np.load(cache))
    if not len(atlas['joints']):raise RuntimeError('当前任务没有无碰撞放置解')
    candidates=torch.tensor(atlas['joints'],device='cuda',dtype=torch.float32).contiguous()
    weights=torch.tensor([math.radians(15)/.1]+[1.]*14,device='cuda')
    roots=torch.tensor([row['start'] for row in rows],device='cuda',dtype=torch.float32).contiguous()
    selected_goals=[];began=time.perf_counter()
    for root in roots:
        selected_goals.extend(match_goals(p,root,candidates,weights,loaded))
    unique=np.unique(np.round(np.asarray(selected_goals),6),axis=0,return_index=True)[1]
    goals=torch.tensor(np.asarray(selected_goals)[unique],device='cuda',dtype=torch.float32).contiguous()
    torch.cuda.synchronize();match_ms=(time.perf_counter()-began)*1000
    report={'task':boxes,'setup_ms':setup_ms,'contact_ik_ms':ik_ms,'contact_count':len(contacts),'extraction_ms':extract_ms,'matching_ms':match_ms,'valid_extraction_count':len(rows),'root_records':rows,'attempts':attempts,'goal_count':len(goals),'goals':goals.cpu().tolist(),'runs':[],'path_quality_thresholds_diagnostic':{'extra_deg':90,'backtracking_proxy_deg':15,'arc_deviation_proxy_deg':45},'home_approach_checked':False}
    if prepare_only:return report,roots,goals,solved
    validity=GpuValidity(loaded,check_ground=True,weights=weights.cpu().tolist());validity.collision_weights=torch.tensor([5.]+[1.]*14,device='cuda')
    common=BoxRotationCost(loaded,ACTIVE_JOINTS,roots[0],goals[0],False)
    for root in roots:
        actual=common.orientations(root[None]);reference=common.initial
        assert all(float((actual[side]*reference[side]).sum(-1).abs().min())>math.cos(math.radians(.1)/2) for side in actual)
    validity.box_rotation_cost=common
    rejected=[]
    def quality(path):
        metric=BoxRotationCost(loaded,ACTIVE_JOINTS,path[0],path[-1],False);metrics=metric.metrics(path)
        accepted=all(item['total_rotation_deg']-item['required_rotation_deg']<=90 and item['backtracking_deg']<=15 and item['max_deviation_deg']<=45 for item in metrics.values())
        if not accepted:rejected.append(metrics)
        return accepted
    validity.path_quality_callback=quality
    for repeat,seed in enumerate([621648851,20261009,20261010]):
        rejected.clear();torch.cuda.synchronize();began=time.perf_counter()
        path,stats=batched_rrt_connect_multi_goal(roots,goals,loaded.kinematics.get_joint_limits().position[0]+1e-5,loaded.kinematics.get_joint_limits().position[1]-1e-5,validity,2.,seed,informed_sampling=False,stop_at_first_valid=True)
        torch.cuda.synchronize();search_ms=(time.perf_counter()-began)*1000
        row={'repeat':repeat,'seed':seed,'success':path is not None,'search_ms':search_ms,'ik_extract_match_search_ms':ik_ms+extract_ms+match_ms+search_ms,'stats':stats,'quality_rejected_connections':len(rejected)}
        if path is not None:
            values=torch.tensor(np.asarray(densify(path)),device='cuda',dtype=torch.float32).contiguous();assert validity.mask(values).all()
            root_index=int(torch.linalg.vector_norm(roots-values[0],dim=-1).argmin());goal_index=int(torch.linalg.vector_norm(goals-values[-1],dim=-1).argmin());assert torch.allclose(roots[root_index],values[0]) and torch.allclose(goals[goal_index],values[-1])
            metric=BoxRotationCost(loaded,ACTIVE_JOINTS,values[0],values[-1],False);row['rotation']=metric.metrics(values);row['selected_root']=root_index;row['selected_contact']=rows[root_index]['contact_index'];row['selected_goal']=goal_index
            (folder/f'path_{repeat}.json').write_text(json.dumps({'frames':values.cpu().tolist(),'selected_root':root_index,'selected_goal':goal_index,'rotation':row['rotation']},indent=2))
        report['runs'].append(row);(folder/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print('RESULT',row,flush=True)
        loaded.base_cuboid_layer.query_cache.clear();gc.collect();torch.cuda.empty_cache()


if __name__=='__main__':main()
