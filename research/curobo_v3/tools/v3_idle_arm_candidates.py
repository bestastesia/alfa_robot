import time

import numpy as np
import torch
from curobo.types import JointState

from v3_batched_loaded_search import GpuValidity
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def solve_idle_candidates(planner, fixed_rows, reference, checker, limit=64, radius_limit=None,
                          sample_counts=(32, 128, 512), optimization_iterations=40):
    began = time.perf_counter()
    fixed_rows = np.asarray(fixed_rows, dtype=np.float32).reshape(-1,15)
    idle = planner.idle_indices
    active = [index for index in range(15) if index not in idle]
    unique = np.unique(np.round(fixed_rows[:,active],6),axis=0,return_index=True)[1]
    fixed_rows = fixed_rows[np.sort(unique)]
    weights = np.array([np.deg2rad(15)/.1]+[1.]*14)
    fixed_rows = fixed_rows[np.argsort(np.sum(((fixed_rows-reference)*weights)**2,axis=1))]
    validity = GpuValidity(checker,check_ground=True)
    lower,upper = checker.kinematics.get_joint_limits().position
    home = torch.tensor(planner.home_values[idle],device='cuda',dtype=torch.float32)
    current = torch.tensor(np.asarray(reference)[idle],device='cuda',dtype=torch.float32)
    accepted = {}
    checked = 0
    stages = []
    generator = torch.Generator(device='cuda').manual_seed(20261015)

    def accept(values):
        nonlocal checked
        for offset in range(0,len(values),512):
            batch = values[offset:offset+512].contiguous()
            mask = validity.mask(batch)
            if radius_limit is not None:
                mask &= planner.full_envelope_radius(checker,batch)<=radius_limit
            checked += len(batch)
            for row in batch[mask].detach().cpu().numpy():
                accepted[tuple(np.round(row,6))] = row.copy()

    fixed = torch.tensor(fixed_rows,device='cuda',dtype=torch.float32)
    for label,pose in [('reference',current),('home',home)]:
        values = fixed.clone()
        values[:,idle] = pose
        accept(values)
    stages.append(dict(stage='reference_and_home',checked=checked,legal=len(accepted)))
    for count in sample_counts:
        if len(accepted)>=limit:
            break
        samples = home+torch.randn((count,7),device='cuda',generator=generator)*(.35 if count<=32 else .75)
        global_samples = lower[idle]+torch.rand((count,7),device='cuda',generator=generator)*(upper[idle]-lower[idle])
        samples = torch.where((torch.arange(count,device='cuda')%4==0)[:,None],global_samples,samples)
        samples = torch.clamp(samples,lower[idle],upper[idle])
        before = checked
        for offset in range(0,len(fixed),16):
            source = fixed[offset:offset+16]
            values = source[:,None].expand(-1,count,-1).clone()
            values[:,:,idle] = samples[None]
            accept(values.reshape(-1,15))
            if len(accepted)>=limit:
                break
        stages.append(dict(stage='batch_screen',idle_samples=count,checked=checked-before,legal=len(accepted)))
    iterations = 0
    if not accepted and len(fixed) and optimization_iterations:
        count = min(32,len(fixed))
        fixed_opt = fixed[:count].repeat_interleave(4,dim=0)
        with torch.enable_grad():
            initial = home+torch.randn((len(fixed_opt),7),device='cuda',generator=generator)*.5
            parameter = torch.nn.Parameter(torch.clamp(initial,lower[idle],upper[idle]))
            optimizer = torch.optim.Adam([parameter],lr=.06)
            pairs_config = checker.kinematics.get_self_collision_config()
            pairs = pairs_config.collision_pairs.long()
            for iteration in range(optimization_iterations):
                optimizer.zero_grad()
                values = fixed_opt.index_copy(1,torch.tensor(idle,device='cuda'),parameter)
                checker.setup_batch_tensors(1,len(values))
                state = checker.kinematics.compute_kinematics(JointState.from_position(values[None],joint_names=ACTIVE_JOINTS))
                spheres = state.robot_spheres.reshape(len(values),-1,4)
                checker.collision_cost.update_num_spheres(spheres.shape[1],batch_size=1,horizon=len(values))
                world = checker.collision_cost.forward(state).reshape(len(values),-1).sum(-1)
                first,second = spheres[:,pairs[:,0]],spheres[:,pairs[:,1]]
                gap = torch.linalg.vector_norm(first[...,:3]-second[...,:3],dim=-1)-first[...,3]-second[...,3]
                gap -= pairs_config.sphere_padding[pairs[:,0]]+pairs_config.sphere_padding[pairs[:,1]]
                self_cost = torch.relu(.003-gap).square().sum(-1)
                indices = [index for index in range(spheres.shape[1]) if index not in validity.ground_exempt_sphere_indices]
                ground = spheres[:,indices]
                ground_cost = torch.where(ground[...,3]>0,torch.relu(ground[...,3]+.003-ground[...,2]),0.).square().sum(-1)
                internal = checker.base_cuboid_layer.cost(state).reshape(len(values),-1).sum(-1)
                loss = (10000*(world+self_cost+ground_cost+internal)+.01*(parameter-home).square().sum(-1)).sum()
                if radius_limit is not None:
                    radial = torch.linalg.vector_norm(spheres[...,:2],dim=-1)+spheres[...,3]
                    loss = loss+10000*torch.where(spheres[...,3]>0,torch.relu(radial-radius_limit),0.).square().sum()
                loss.backward()
                optimizer.step()
                with torch.no_grad():
                    parameter.clamp_(lower[idle],upper[idle])
                iterations = iteration+1
                if iterations%5==0:
                    with torch.no_grad():
                        accept(fixed_opt.index_copy(1,torch.tensor(idle,device='cuda'),parameter).detach())
                    if accepted:
                        break
        stages.append(dict(stage='short_gpu_optimization',iterations=iterations,legal=len(accepted)))
    if accepted:
        values = np.asarray(list(accepted.values()))
        costs = np.sum(((values-reference)*weights)**2,axis=1)+3*np.sum((values[:,idle]-planner.home_values[idle])**2,axis=1)
        values = values[np.argsort(costs)[:limit]]
    else:
        values = np.empty((0,15),dtype=np.float32)
    torch.cuda.synchronize()
    report = dict(fixed_active_configurations=len(fixed),checked_candidates=checked,accepted_candidates=len(values),
                  legal_unique_screened=len(accepted),optimization_iterations=iterations,stages=stages,
                  total_ms=(time.perf_counter()-began)*1000,working_joints_and_updown_fixed=True,radius_limit_m=radius_limit)
    return values,report
