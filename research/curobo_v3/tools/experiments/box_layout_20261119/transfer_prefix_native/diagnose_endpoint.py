"""gpuq only: distinguish raw endpoint legality from native sampling errors."""
from dataclasses import replace
import importlib.metadata
import sys
import time

import numpy as np
import torch
import curobo
from probe_prefix import ROOT, OUT, inputs, prepared_assets, save, gpu_state
from curobo.types import JointState
from curobo_core.box_pick import BoxPick
from curobo_core.scene import Pose, SceneObject
from v3_batched_loaded_search import GpuValidity, scene_query_spheres
import v3_two_box_pick
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def components(planner, checker, rows):
    values = torch.as_tensor(rows, device='cuda', dtype=torch.float32)
    validity = GpuValidity(checker, check_ground=True, scene_penetration_tolerance_m=.001)
    mask = validity.mask(values).cpu().numpy()
    margins = planner.native_margin_mask(checker, values).cpu().numpy()
    checker.setup_batch_tensors(1, len(values))
    state = checker.kinematics.compute_kinematics(JointState.from_position(values[None], joint_names=ACTIVE_JOINTS))
    spheres = state.robot_spheres.reshape(1,len(values),-1,4)
    checker.collision_constraint.update_num_spheres(spheres.shape[2], batch_size=1, horizon=len(values))
    tensors = dict(joint_bound=checker.get_bound(values[None]), self_collision=checker.get_self_collision(spheres),
        world_collision=checker.get_collision_constraint(scene_query_spheres(spheres,.001)),
        base_hard=checker.base_mesh_layer.cost(state,hard=True), native_base=checker.base_cuboid_layer.cost(state))
    costs = {k:v.reshape(len(values),-1).sum(-1).detach().cpu().numpy().copy() for k,v in tensors.items()}
    ground_ids = [i for i in range(spheres.shape[2]) if i not in validity.ground_exempt_sphere_indices]
    ground = spheres[0,:,ground_ids,:]
    costs['ground_0mm'] = ((ground[...,3]>0)&(ground[...,2]-ground[...,3]<0)).sum(-1).cpu().numpy()
    native_ids = [i for i in range(spheres.shape[2]) if i not in getattr(checker,'payload_ground_exempt_sphere_indices',())]
    native_ground = spheres[0,:,native_ids,:]
    costs['native_ground_2mm'] = torch.where(native_ground[...,3]>0,
        torch.relu(native_ground[...,3]-native_ground[...,2]+.002),0.).square().sum(-1).cpu().numpy()
    result = dict(samples=len(values), valid=bool(mask.all() and margins.all()),
        validity_invalid_indices=np.flatnonzero(~mask).tolist(), native_margin_invalid_indices=np.flatnonzero(~margins).tolist(),
        components={k:dict(maximum=float(v.max()), positive_indices=np.flatnonzero(v>0).tolist()) for k,v in costs.items()},
        updown_min_m=float(values[:,0].min()), updown_max_m=float(values[:,0].max()))
    if checker.guidance_payload:
        result['full_payload_obb_audit'] = planner.loaded_audit(checker, rows)
    return result


if __name__ == '__main__':
    began = time.perf_counter()
    prior, geometry, hashes = inputs()
    rows = geometry['owner_transport']
    v3_two_box_pick.prepare_fast_assets = prepared_assets
    planner = v3_two_box_pick.TwoBoxPickPlanner()
    owner = BoxPick(**dict(next(b for b in prior['request']['boxes'] if b['side']=='right'),side='left'))
    pending = BoxPick(**next(b for b in prior['request']['boxes'] if b['side']=='left'))
    planner.configure_pick(owner)
    planner.native_project_margins = True
    planner.snapshot = replace(planner.snapshot, objects=planner.snapshot.objects+(
        SceneObject('pending_original_left',pending.dimensions_m,Pose(pending.center_m)),))
    edge = rows[-2]+np.linspace(0,1,65)[:,None]*(rows[-1]-rows[-2])
    with np.load(OUT/'attempt_02/transport/transport.npz',allow_pickle=False) as data:
        native_rows = data['position'][[3998,3999,4000]].copy()
    cases = dict(original_endpoint=rows[-1:], original_last_edge_65=edge, rejected_native_tail=native_rows)
    report = dict(executable=sys.executable,curobo_file=curobo.__file__,
        curobo_version=importlib.metadata.version('nvidia-curobo'), provenance=hashes,
        physical_source_box_id='right',actual_owner_arm='left',original_endpoint=rows[-1].tolist(),
        last_edge_start=rows[-2].tolist(),cases={})
    for loaded in (False,True):
        checker = planner.checker(planner.selected_task,payload=loaded)
        assert checker.guidance_payload == loaded and any(o.name=='pending_original_left' for o in checker.guidance_scene.cuboid)
        report['cases']['loaded' if loaded else 'empty_proxy_comparison'] = {
            name:components(planner,checker,values) for name,values in cases.items()}
    report['original_endpoint_and_last_edge_valid'] = all(report['cases']['loaded'][k]['valid']
        and report['cases']['loaded'][k]['full_payload_obb_audit']['valid'] for k in ('original_endpoint','original_last_edge_65'))
    report.update(wall_seconds=time.perf_counter()-began,gpu_before_exit=gpu_state())
    save(OUT/'endpoint_component_diagnosis.json',report)
    for name,result in report['cases']['loaded'].items():
        print(name,'valid',result['valid'],'failed_components',
            {k:v for k,v in result['components'].items() if v['positive_indices']},flush=True)
    print('ENDPOINT_DIAGNOSIS',report['original_endpoint_and_last_edge_valid'],'wall',report['wall_seconds'],flush=True)
