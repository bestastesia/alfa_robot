import copy
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from v3_tcp_navigation import alignment_metrics, box_arrays, forward_route, peak_normal_acceleration


def from_matrix(matrix):
    return np.r_[matrix[:3,3],Rotation.from_matrix(matrix[:3,:3]).as_quat()]


def interpolate(first, second, fractions):
    fractions = np.asarray(fractions).reshape(-1,1)
    initial, final = first[3:].copy(), second[3:].copy()
    cosine = float(np.dot(initial,final))
    if cosine < 0:
        final = -final
        cosine = -cosine
    angle = np.arccos(np.clip(cosine,-1,1))
    if angle < 1e-7:
        quaternion = (1-fractions)*initial+fractions*final
        quaternion /= np.linalg.norm(quaternion,axis=1,keepdims=True)
    else:
        quaternion = (np.sin((1-fractions)*angle)*initial+np.sin(fractions*angle)*final)/np.sin(angle)
    return np.column_stack(((1-fractions)*first[:3]+fractions*second[:3],quaternion))


class ClusterCollision:
    def __init__(self, model, objects, margin=.005, stationary=()):
        self.local = np.asarray(model['centers_tool'],dtype=float)
        self.radii = np.asarray(model['radii'],dtype=float)
        if self.local.shape != (len(self.radii),3) or not len(self.radii) or np.any(self.radii<=0):
            raise ValueError('Expected a nonempty native positive-radius sphere group')
        self.centers = np.asarray([item['position'] for item in objects]).reshape(-1,3)
        self.rotations = Rotation.from_quat(np.array([np.roll(item['quaternion_wxyz'],-1) for item in objects])).as_matrix() if objects else np.empty((0,3,3))
        self.half = np.asarray([item['dimensions'] for item in objects]).reshape(-1,3)/2
        self.margin = margin
        self.stationary = list(stationary)
        self.lever = float(np.linalg.norm(self.local,axis=1).max())
        self.queries = 0
        self.edge_checks = 0

    def sphere_centers(self, poses):
        poses = np.asarray(poses).reshape(-1,7)
        return np.einsum('bij,sj->bsi',Rotation.from_quat(poses[:,3:]).as_matrix(),self.local)+poses[:,:3][:,None]

    def clearance(self, poses):
        poses = np.asarray(poses).reshape(-1,7)
        self.queries += len(poses)
        centers = self.sphere_centers(poses)
        output = np.full(len(poses),np.inf)
        if len(self.centers):
            local = np.einsum('bsoi,oij->bsoj',centers[:,:,None]-self.centers[None,None],self.rotations)
            delta = np.abs(local)-self.half[None,None]
            distance = np.linalg.norm(np.maximum(delta,0),axis=-1)+np.minimum(delta.max(-1),0)
            output = (distance-self.radii[None,:,None]-self.margin).min(axis=(1,2))
        for other_centers,other_radii in self.stationary:
            distance = np.linalg.norm(centers[:,:,None]-other_centers[None,None],axis=-1)-self.radii[None,:,None]-other_radii[None,None]-self.margin
            output = np.minimum(output,distance.min(axis=(1,2)))
        return output

    def edge_clear(self, first, second):
        self.edge_checks += 1
        if np.any(self.clearance(np.stack([first,second]))<=0):
            return False
        starts, ends = np.asarray(first)[None],np.asarray(second)[None]
        for depth in range(17):
            middle = np.array([interpolate(start,end,[.5])[0] for start,end in zip(starts,ends)])
            clearance = self.clearance(middle)
            if np.any(clearance<=0):
                return False
            angle = 2*np.arccos(np.clip(np.abs(np.sum(starts[:,3:]*ends[:,3:],axis=1)),0,1))
            bound = np.linalg.norm(ends[:,:3]-starts[:,:3],axis=1)*.5+2*self.lever*np.sin(angle*.25)
            unresolved = clearance <= bound+1e-10
            if not unresolved.any():
                return True
            if depth==16:
                return False
            starts,ends,middle = starts[unresolved],ends[unresolved],middle[unresolved]
            starts,ends = np.concatenate([starts,middle]),np.concatenate([middle,ends])
        return False

    def path_clear(self, poses):
        return all(self.edge_clear(first,second) for first,second in zip(poses[:-1],poses[1:]))


def path_score(poses, forward):
    displacement = np.diff(poses[:,:3],axis=0)
    lengths = np.linalg.norm(displacement,axis=1)
    angles = 2*np.arccos(np.clip(np.abs(np.sum(poses[:-1,3:]*poses[1:,3:],axis=1)),0,1))
    score = lengths.sum()+.15*angles.sum()
    if forward:
        middle = np.array([interpolate(first,second,[.5])[0] for first,second in zip(poses[:-1],poses[1:])])
        direction = Rotation.from_quat(middle[:,3:]).as_matrix()[:,:,2]
        cosine = np.sum(displacement*direction,axis=1)/np.maximum(lengths,1e-10)
        score += .5*np.sum(lengths*(1-cosine)**2)
    return float(score)


def search_pose_path(start, goal, collision, forward=True, seed=20261021, budget_s=6.):
    began = time.perf_counter()
    for label,pose in [('起点',start),('目标',goal)]:
        clearance = float(collision.clearance(pose)[0])
        if clearance<=0:
            raise ValueError(f'{label}真实吸盘球组碰撞（含余量），净空{clearance*1000:.2f}mm；不会忽略碰撞或退回单球')
    candidates = []
    if collision.edge_clear(start,goal):
        candidates.append(np.stack([start,goal]))
    for height in [.15,.3,.5,.7]:
        upper = max(start[2],goal[2])+height
        first,second = start.copy(),goal.copy()
        first[2]=upper
        second[2]=upper
        candidate = np.stack([start,first,second,goal])
        if collision.path_clear(candidate):
            candidates.append(candidate)
    nodes = 0
    if not candidates:
        generator = np.random.default_rng(seed)
        lower = np.minimum(start[:3],goal[:3])-[.4,.5,.2]
        upper = np.maximum(start[:3],goal[:3])+[.4,.5,.8]
        lower[2] = max(.05,lower[2])
        trees = [[start.copy()],[goal.copy()]]
        parents = [[-1],[-1]]
        def nearest(tree,target):
            values = np.asarray(tree)
            angle = 2*np.arccos(np.clip(np.abs(values[:,3:]@target[3:]),0,1))
            return int(np.argmin(np.sum((values[:,:3]-target[:3])**2,axis=1)+(.18*angle)**2))
        def extend(tree,parent,target):
            index = nearest(tree,target)
            current = tree[index]
            distance = np.linalg.norm(current[:3]-target[:3])
            angle = 2*np.arccos(np.clip(abs(current[3:]@target[3:]),0,1))
            fraction = min(1.,.12/max(distance,1e-12),.35/max(angle,1e-12))
            reached = interpolate(current,target,[fraction])[0]
            if not collision.edge_clear(current,reached):
                return None,False
            tree.append(reached)
            parent.append(index)
            return len(tree)-1,fraction>=1.-1e-9
        while time.perf_counter()-began<budget_s:
            active = nodes%2
            if generator.random()<.3:
                target = trees[1-active][generator.integers(len(trees[1-active]))]
            elif generator.random()<.7:
                target = interpolate(start,goal,[generator.random()])[0]
                target[:3] += generator.normal(0,.2,3)
                target[2] += abs(generator.normal(0,.25))
                target[3:] = (Rotation.from_rotvec(generator.normal(0,.35,3))*Rotation.from_quat(target[3:])).as_quat()
            else:
                target = np.r_[generator.uniform(lower,upper),Rotation.random(random_state=generator).as_quat()]
            node,reached = extend(trees[active],parents[active],target)
            nodes += 1
            if node is None:
                continue
            for connection_attempt in range(60):
                other,connected = extend(trees[1-active],parents[1-active],trees[active][node])
                if other is None:
                    break
                if connected:
                    branches=[]
                    for tree_index,last in [(active,node),(1-active,other)]:
                        chain=[]
                        while last>=0:
                            chain.append(trees[tree_index][last])
                            last=parents[tree_index][last]
                        branches.append(chain[::-1])
                    candidate = np.array(branches[0]+branches[1][::-1]) if active==0 else np.array(branches[1]+branches[0][::-1])
                    candidates.append(candidate)
                    break
                if time.perf_counter()-began>=budget_s:
                    break
            if candidates:
                break
    if not candidates:
        raise RuntimeError('SE(3)真实吸盘球组搜索未找到通路，未输出可播放轨迹')
    selected = min(candidates,key=lambda poses:path_score(poses,forward))
    if forward:
        positions = selected[:,:3]
        retained = np.r_[True,np.linalg.norm(np.diff(positions,axis=0),axis=1)>1e-7]
        positions = positions[retained]
        if len(positions)>1:
            try:
                def accept(points,rotations):
                    return collision.path_clear(np.column_stack([points,Rotation.from_matrix(rotations).as_quat()]))
                points,rotations,strength = forward_route(positions,Rotation.from_quat(start[3:]).as_matrix(),Rotation.from_quat(goal[3:]).as_matrix(),box_arrays([],0),pose_validator=accept)
                curved = np.column_stack([points,Rotation.from_matrix(rotations).as_quat()])
                if path_score(curved,True)<path_score(selected,True):
                    selected = curved
            except (ValueError,RuntimeError):
                pass
    if not collision.path_clear(selected):
        raise RuntimeError('真实吸盘球组最终路径复核失败')
    return selected,dict(expanded_nodes=nodes,search_ms=(time.perf_counter()-began)*1000,edge_checks=collision.edge_checks,pose_queries=collision.queries)


def time_pose_path(poses, speed, acceleration, normal_acceleration):
    steps = np.linalg.norm(np.diff(poses[:,:3],axis=0),axis=1)
    angles = 2*np.arccos(np.clip(np.abs(np.sum(poses[:-1,3:]*poses[1:,3:],axis=1)),0,1))
    measure = np.r_[0,np.cumsum(np.maximum(steps,.15*angles))]
    retained = np.r_[True,np.diff(measure)>1e-10]
    poses,measure = poses[retained],measure[retained]
    if len(poses)<2:
        raise ValueError('Expected nonzero rigid-body motion')
    progress = measure/measure[-1]
    duration = max(.5,1.875*steps.sum()/speed,1.875*angles.sum()/np.deg2rad(45))
    samples = np.unique(np.r_[np.linspace(0,1,int(np.ceil(duration*60))+1),progress])
    low,high = np.zeros(len(samples)),np.ones(len(samples))
    for iteration in range(50):
        middle=(low+high)*.5
        value=10*middle**3-15*middle**4+6*middle**5
        low=np.where(value<samples,middle,low)
        high=np.where(value>=samples,middle,high)
    times=(low+high)*.5*duration
    times[0],times[-1]=0.,duration
    positions=np.column_stack([np.interp(samples,progress,poses[:,axis]) for axis in range(3)])
    orientations=Slerp(progress,Rotation.from_quat(poses[:,3:]))(samples)
    intervals=np.diff(times)
    velocities=np.diff(positions,axis=0)/intervals[:,None]
    center_intervals=(intervals[:-1]+intervals[1:])*.5
    accelerations=np.diff(velocities,axis=0)/center_intervals[:,None]
    angular_velocity=orientations[:-1].apply((orientations[:-1].inv()*orientations[1:]).as_rotvec())/intervals[:,None]
    angular_acceleration=np.diff(angular_velocity,axis=0)/center_intervals[:,None]
    metrics=dict(peak_speed_m_s=float(np.linalg.norm(velocities,axis=1).max()),peak_acceleration_m_s2=float(np.linalg.norm(accelerations,axis=1).max()),peak_normal_acceleration_m_s2=peak_normal_acceleration(positions,times),peak_angular_speed_deg_s=float(np.rad2deg(np.linalg.norm(angular_velocity,axis=1).max())),peak_angular_acceleration_deg_s2=float(np.rad2deg(np.linalg.norm(angular_acceleration,axis=1).max())))
    scale=max(1.,metrics['peak_speed_m_s']/speed,np.sqrt(metrics['peak_acceleration_m_s2']/acceleration),metrics['peak_angular_speed_deg_s']/45.,np.sqrt(metrics['peak_angular_acceleration_deg_s2']/90.))
    if normal_acceleration is not None:
        scale=max(scale,np.sqrt(metrics['peak_normal_acceleration_m_s2']/normal_acceleration))
    if scale>1:
        scale*=1.01
    times*=scale
    metrics={name:value/(scale**2 if 'acceleration' in name else scale) for name,value in metrics.items()}
    return dict(positions=positions.tolist(),quaternions_wxyz=np.roll(orientations.as_quat(),1,axis=1).tolist(),times_s=times.tolist(),duration_s=float(times[-1]),velocity_z_alignment=alignment_metrics(positions,np.roll(orientations.as_quat(),1,axis=1)),**metrics)


def plan_cluster_navigation(snapshot, geometry, margin=.005, offset=.08, speed=.25, acceleration=.5, motion_mode='forward_z', normal_acceleration=.05):
    began=time.perf_counter()
    objects=copy.deepcopy(snapshot['objects'])
    base=np.asarray(snapshot.get('robot_base_world_xyz',[0,0,0]))
    for item in geometry['internal_boxes']:
        box=copy.deepcopy(item)
        box['position']=(np.asarray(box['position'])+base).tolist()
        box['name']='robot_internal/'+box['name']
        objects.append(box)
    endpoints={side:from_matrix(np.asarray(matrix)) for side,matrix in snapshot['tcp_world'].items()}
    sides=list(snapshot['targets'])
    failures=[]
    for order in [sides,sides[::-1]] if len(sides)>1 else [sides]:
        current={side:pose.copy() for side,pose in endpoints.items()}
        paths={}
        elapsed=0.
        try:
            for side in order:
                stationary=[]
                for other,pose in current.items():
                    if other==side:
                        continue
                    model=geometry['arms'][other]
                    centers=np.asarray(model['centers_tool'])@Rotation.from_quat(pose[3:]).as_matrix().T+pose[:3]
                    stationary.append((centers,np.asarray(model['radii'])))
                collision=ClusterCollision(geometry['arms'][side],objects,margin,stationary)
                target=np.asarray(snapshot['targets'][side]).copy()
                target[:3,3]-=target[:3,2]*offset
                waypoints,stats=search_pose_path(current[side],from_matrix(target),collision,forward=motion_mode=='forward_z')
                path=time_pose_path(waypoints,speed,acceleration,normal_acceleration)
                timed=np.column_stack([path['positions'],np.roll(path['quaternions_wxyz'],-1,axis=1)])
                if not collision.path_clear(timed):
                    raise RuntimeError('时间插值后的吸盘球组连续检查失败')
                path.update(side=side,start_time_s=elapsed,raw_waypoints=waypoints[:,:3].tolist(),rounded_waypoints=waypoints[:,:3].tolist(),waypoint_poses=waypoints.tolist(),search_and_timing_ms=stats['search_ms'],**stats)
                paths[side]=path
                elapsed+=path['duration_s']
                current[side]=from_matrix(target)
            break
        except (ValueError,RuntimeError) as error:
            failures.append(dict(order=order,error=str(error)))
    else:
        raise RuntimeError(str(failures))
    times=np.unique(np.concatenate([np.asarray(path['times_s'])+path['start_time_s'] for path in paths.values()]))
    tracks={}
    for side,initial in endpoints.items():
        if side in paths:
            path=paths[side]
            local=np.clip(times-path['start_time_s'],0,path['duration_s'])
            position=np.column_stack([np.interp(local,path['times_s'],np.asarray(path['positions'])[:,axis]) for axis in range(3)])
            quaternion=np.roll(Slerp(path['times_s'],Rotation.from_quat(np.roll(path['quaternions_wxyz'],-1,axis=1)))(local).as_quat(),1,axis=1)
        else:
            position=np.repeat(initial[None,:3],len(times),axis=0)
            quaternion=np.repeat(np.roll(initial[None,3:],1,axis=1),len(times),axis=0)
        tracks[side]=dict(positions=position.tolist(),quaternions_wxyz=quaternion.tolist())
    reference=copy.deepcopy(snapshot)
    reference['objects']=objects
    return dict(success=True,motion_mode=motion_mode,collision_model='native_suction_sphere_group',tool_geometry=geometry,snapshot=reference,margin_m=margin,target_offset_m=offset,order=order,paths=paths,tracks=tracks,times_s=times.tolist(),total_ms=(time.perf_counter()-began)*1000,min_tcp_distance_m=None,synchronized_metrics={side:{name:value for name,value in path.items() if name.startswith('peak_') or name=='velocity_z_alignment'} for side,path in paths.items()},failed_orders=failures,limits=dict(speed_m_s=speed,acceleration_m_s2=acceleration,normal_acceleration_m_s2=normal_acceleration),collision_contract='Native rigid link7/tool0 sphere groups vs all exported OBBs and fixed base cuboids, and other stationary tool group; conservative adaptive swept-pose edge certificates; full arms and IK not checked')
