import numpy as np
import torch
from scipy.spatial.transform import Rotation
from curobo.types import JointState

from v3_wall_ik_benchmark import ACTIVE_JOINTS
from curobo_core.payload_geometry import box_overlap_depth, box_sphere_depth


def base_world_audit(checker,tolerance_m=.001):
    failures=[]
    for body in checker.base_cuboid_layer.scene.cuboid:
        rotation=Rotation.from_quat(np.roll(body.pose[3:7],-1)).as_matrix()
        for obstacle in checker.guidance_scene.cuboid or []:
            other=Rotation.from_quat(np.roll(obstacle.pose[3:7],-1)).as_matrix()
            depth=box_overlap_depth(np.asarray(body.pose[:3])[None],rotation[None],np.asarray(body.dims)/2,
                np.asarray(obstacle.pose[:3]),other,np.asarray(obstacle.dims)/2)[0]
            if depth>tolerance_m+1e-7:
                failures.append(dict(body=body.name,obstacle=obstacle.name,overlap_m=float(depth)))
    return dict(valid=not failures,failures=failures,scope='fixed footprint internal base cuboids versus full world cuboids')


def exact_payload_audit(checker,rows,attachments,dimensions,original_robot,tolerance_m=.001,*,check_ground=False):
    if not 0<=tolerance_m<=.001:
        raise ValueError('payload environment penetration limit exceeds approved1mm')
    names=checker.kinematics.config.kinematics_config
    sphere_count=checker.kinematics.total_spheres
    payload_indices=set()
    excluded={}
    for side in attachments:
        indices=names.get_sphere_index_from_link_name(side+'_link7').cpu().numpy()
        original_count=len(original_robot['kinematics']['collision_spheres'][side+'_link7'])
        if len(indices)<=original_count:
            raise RuntimeError('payload audit called without attached collision spheres')
        payload_indices.update(indices[original_count:].tolist())
        excluded[side]=set(indices[:original_count].tolist())
    robot_ids=np.array([index for index in range(sphere_count) if index not in payload_indices],dtype=int)
    scene=list(checker.guidance_scene.cuboid or [])
    internal=list(checker.base_cuboid_layer.scene.cuboid)
    obstacles=[(obstacle.name,np.asarray(obstacle.pose[:3]),
        Rotation.from_quat(np.roll(obstacle.pose[3:7],-1)).as_matrix(),np.asarray(obstacle.dims)/2)
        for obstacle in scene+internal]
    selected_ids={side:np.asarray([index for index in robot_ids if index not in excluded[side]],dtype=int)
        for side in attachments}
    ground={side:np.inf for side in attachments}
    failures=[]
    invalid_frames=set()
    maximum=dict(box_world=0.,box_robot=0.,box_box=0.)
    for offset in range(0,len(rows),256):
        tensor=torch.as_tensor(rows[offset:offset+256],device='cuda',dtype=torch.float32)[None].contiguous()
        state=checker.kinematics.compute_kinematics(JointState.from_position(tensor,joint_names=ACTIVE_JOINTS))
        spheres=state.robot_spheres.reshape(-1,sphere_count,4).detach().cpu().numpy()
        boxes={}
        for side,attachment in attachments.items():
            pose=state.tool_poses.to_dict()[side+'_tool0']
            rotations=Rotation.from_quat(np.roll(pose.quaternion.reshape(-1,4).cpu().numpy(),-1,axis=1)).as_matrix()
            attachment=np.asarray(attachment)
            positions=pose.position.reshape(-1,3).cpu().numpy()+np.einsum('bij,j->bi',rotations,attachment[:3,3])
            rotations=rotations@attachment[:3,:3]
            boxes[side]=(positions,rotations)
            size = dimensions[side] if isinstance(dimensions, dict) else dimensions
            if check_ground:
                bottom=positions[:,2]-(np.abs(rotations[:,2,:])*np.asarray(size)/2).sum(-1)
                ground[side]=min(ground[side],float(bottom.min()))
                bad=np.flatnonzero(bottom<-.001)
                invalid_frames.update((offset+bad).tolist())
                if len(bad):
                    failures.append(dict(kind='box_ground',side=side,frame=offset+int(bad[0]),clearance_m=float(bottom[bad].min())))
            extent=(np.abs(rotations)*np.asarray(size)/2).sum(-1)
            for name,position,rotation,half in obstacles:
                other_extent=np.abs(rotation)@half
                possible=(np.abs(positions-position)<=extent+other_extent+1e-10).all(-1)
                if not possible.any():continue
                depth=np.full(len(positions),-np.inf)
                depth[possible]=box_overlap_depth(positions[possible],rotations[possible],np.asarray(size)/2,position,rotation,half)
                if (np.abs(depth-(tolerance_m+1e-7))<1e-12).any():
                    depth=box_overlap_depth(positions,rotations,np.asarray(size)/2,position,rotation,half)
                maximum['box_world']=max(maximum['box_world'],float(depth.max(initial=0)))
                bad=np.flatnonzero(depth>tolerance_m+1e-7)
                invalid_frames.update((offset+bad).tolist())
                if len(bad):
                    failures.append(dict(kind='box_world',side=side,obstacle=name,frame=offset+int(bad[0]),depth_m=float(depth[bad].max())))
            selected=selected_ids[side]
            centers,radii=spheres[:,selected,:3],spheres[:,selected,3]
            possible=(radii>0)&(np.abs(centers-positions[:,None])<=extent[:,None]+radii[...,None]+1e-10).all(-1)
            depth=np.full(radii.shape,-np.inf)
            frame,sphere=np.nonzero(possible)
            if len(frame):
                depth[frame,sphere]=box_sphere_depth(centers[frame,sphere][:,None],radii[frame,sphere][:,None],
                    positions[frame],rotations[frame],np.asarray(size)/2)[:,0]
                if (np.abs(depth-1e-6)<1e-12).any():
                    depth=box_sphere_depth(centers,radii,positions,rotations,np.asarray(size)/2)
            depth=np.where(spheres[:,selected,3]>0,depth,-np.inf)
            maximum['box_robot']=max(maximum['box_robot'],float(depth.max(initial=0)))
            bad=np.flatnonzero((depth>1e-6).any(-1))
            invalid_frames.update((offset+bad).tolist())
            if len(bad):
                failures.append(dict(kind='box_robot',side=side,frame=offset+int(bad[0]),depth_m=float(depth[bad].max())))
        if len(boxes)==2:
            first,second=boxes.values()
            sizes = [dimensions[s] if isinstance(dimensions, dict) else dimensions for s in boxes]
            depth=box_overlap_depth(first[0],first[1],np.asarray(sizes[0])/2,second[0],second[1],np.asarray(sizes[1])/2)
            maximum['box_box']=max(maximum['box_box'],float(depth.max(initial=0)))
            bad=np.flatnonzero(depth>1e-6)
            invalid_frames.update((offset+bad).tolist())
            if len(bad):
                failures.append(dict(kind='box_box',frame=offset+int(bad[0]),depth_m=float(depth[bad].max())))
    audit=dict(valid=not invalid_frames,failures=failures[:30],invalid_frame_indices=sorted(invalid_frames),maximum_overlap_m=maximum,frames=len(rows),
        scope='independent CPU OBB/SAT cuboid payload vs all scene/internal cuboids, other payload and original robot spheres excluding own link7 touch link; world1mm, payload self1um numeric epsilon')
    if check_ground:
        audit['minimum_box_ground_clearance_m']=ground if isinstance(dimensions,dict) else min(ground.values())
        audit['scope']+='; full box ground clearance >= -1mm'
    return audit


def payload_search_mask(checker, rows, dimensions, original_robot, cpu_audit, *, state=None, classification=None):
    """Fast search-only classification; ambiguous rows keep the existing CPU decision.

    Final sampled trajectories still run exact_payload_audit independently.
    Resources store geometry, never mutable FK output or previous classifications.
    """
    from types import SimpleNamespace
    from v3_full_payload_cost import FullPayloadCost
    if classification is not None and (state is None or len(classification) != 2
            or any(value.shape != (1, len(rows)) for value in classification)):
        raise ValueError('payload classification must match this exact FK row batch')
    attachments = checker.task_tool_to_box
    sizes = dimensions if isinstance(dimensions, dict) else {s: dimensions for s in attachments}
    boxes = list(checker.guidance_scene.cuboid or [])+list(checker.base_cuboid_layer.scene.cuboid)
    key = (tuple((s, tuple(sizes[s]), tuple(np.asarray(a).ravel())) for s, a in attachments.items()),
           tuple((tuple(box.pose), tuple(box.dims)) for box in boxes))
    resource = getattr(checker, '_payload_search_resource', None)
    if resource is None or resource[0] != key:
        resource = (key, FullPayloadCost(checker, original_robot, sizes, checker.guidance_scene, dtype=torch.float64))
        checker._payload_search_resource = resource
    layer = resource[1]
    statistics = getattr(checker, 'payload_search_statistics', dict(frames=0, cpu_fallback_frames=0))
    checker.payload_search_statistics = statistics
    def classify(values, current):
        valid, uncertain = (layer.classify(SimpleNamespace(cuda_robot_model_state=current, tool_poses=current.tool_poses))
                            if classification is None else classification)
        valid = valid.reshape(-1); indices = uncertain.reshape(-1).nonzero().reshape(-1)
        if len(indices):
            audit = cpu_audit(values[indices])
            valid[indices] = True
            valid[indices[audit['invalid_frame_indices']]] = False
        statistics['frames'] += len(values)
        statistics['cpu_fallback_frames'] += len(indices)
        return valid
    masks = []
    with torch.no_grad():
        if state is not None:
            if state.robot_spheres.shape[:2] != (1, len(rows)):
                raise ValueError('payload FK state must match this exact joint-row batch')
            # Query-local only. The CPU fallback may overwrite the FK buffers;
            # callers must consume every other view of this state first.
            return classify(rows, state)
        for offset in range(0, len(rows), 1024):
            values = torch.as_tensor(rows[offset:offset+1024], device='cuda', dtype=torch.float32).contiguous()
            current = checker.kinematics.compute_kinematics(JointState.from_position(values[None], joint_names=ACTIVE_JOINTS))
            masks.append(classify(values, current))
    return torch.cat(masks) if masks else torch.empty(0, device='cuda', dtype=torch.bool)
