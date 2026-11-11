import numpy as np
import torch
from scipy.spatial.transform import Rotation
from curobo.types import JointState

from v3_wall_ik_benchmark import ACTIVE_JOINTS


def box_sphere_depth(centers,radii,positions,rotations,half):
    local=np.einsum('bsi,bij->bsj',centers-positions[:,None],rotations)
    delta=np.abs(local)-np.asarray(half)
    distance=np.linalg.norm(np.maximum(delta,0),axis=-1)+np.minimum(delta.max(-1),0)
    return radii-distance


def box_overlap_depth(first_position,first_rotation,first_half,second_position,second_rotation,second_half):
    batch=len(first_position)
    other=np.broadcast_to(second_rotation,(batch,3,3))
    axes=np.concatenate((first_rotation.transpose(0,2,1),other.transpose(0,2,1),
        np.cross(first_rotation.transpose(0,2,1)[:,:,None],other.transpose(0,2,1)[:,None]).reshape(batch,9,3)),axis=1)
    lengths=np.linalg.norm(axes,axis=-1)
    normalized=axes/np.maximum(lengths[:,:,None],1e-15)
    first_extent=np.sum(np.abs(np.einsum('bai,bij->baj',normalized,first_rotation))*first_half,axis=-1)
    second_extent=np.sum(np.abs(np.einsum('bai,bij->baj',normalized,other))*second_half,axis=-1)
    separation=np.abs(np.einsum('bi,bai->ba',np.asarray(second_position)-first_position,normalized))
    overlap=first_extent+second_extent-separation
    overlap[lengths<1e-10]=np.inf
    return overlap.min(-1)


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


def exact_payload_audit(checker,rows,attachments,dimensions,original_robot,tolerance_m=.001):
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
    failures=[]
    invalid_frames=set()
    maximum=dict(box_world=0.,box_robot=0.,box_box=0.)
    for offset in range(0,len(rows),256):
        tensor=torch.tensor(np.asarray(rows[offset:offset+256])[None],device='cuda',dtype=torch.float32)
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
            for obstacle in scene+internal:
                other_rotation=Rotation.from_quat(np.roll(obstacle.pose[3:7],-1)).as_matrix()
                depth=box_overlap_depth(positions,rotations,np.asarray(dimensions)/2,np.asarray(obstacle.pose[:3]),other_rotation,np.asarray(obstacle.dims)/2)
                maximum['box_world']=max(maximum['box_world'],float(depth.max(initial=0)))
                bad=np.flatnonzero(depth>tolerance_m+1e-7)
                invalid_frames.update((offset+bad).tolist())
                if len(bad):
                    failures.append(dict(kind='box_world',side=side,obstacle=obstacle.name,frame=offset+int(bad[0]),depth_m=float(depth[bad].max())))
            selected=np.asarray([index for index in robot_ids if index not in excluded[side]],dtype=int)
            depth=box_sphere_depth(spheres[:,selected,:3],spheres[:,selected,3],positions,rotations,np.asarray(dimensions)/2)
            depth=np.where(spheres[:,selected,3]>0,depth,-np.inf)
            maximum['box_robot']=max(maximum['box_robot'],float(depth.max(initial=0)))
            bad=np.flatnonzero((depth>1e-6).any(-1))
            invalid_frames.update((offset+bad).tolist())
            if len(bad):
                failures.append(dict(kind='box_robot',side=side,frame=offset+int(bad[0]),depth_m=float(depth[bad].max())))
        if len(boxes)==2:
            first,second=boxes.values()
            depth=box_overlap_depth(first[0],first[1],np.asarray(dimensions)/2,second[0],second[1],np.asarray(dimensions)/2)
            maximum['box_box']=max(maximum['box_box'],float(depth.max(initial=0)))
            bad=np.flatnonzero(depth>1e-6)
            invalid_frames.update((offset+bad).tolist())
            if len(bad):
                failures.append(dict(kind='box_box',frame=offset+int(bad[0]),depth_m=float(depth[bad].max())))
    return dict(valid=not failures,failures=failures[:30],invalid_frame_indices=sorted(invalid_frames),maximum_overlap_m=maximum,frames=len(rows),
        scope='independent CPU OBB/SAT cuboid payload vs all scene/internal cuboids, other payload and original robot spheres excluding own link7 touch link; world1mm, payload self1um numeric epsilon')
