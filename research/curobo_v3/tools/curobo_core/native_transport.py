"""CPU-only native validation and bookkeeping; no planner/GPU import or physical gate override."""
import numpy as np


def source_progress(observations, reference=None, cleared=None):
    """Keep the ORIGINAL transport reference and clearance history across pieces."""
    reference = ({s:v['initial_slot_overlap_m'] for s,v in observations.items()}
                 if reference is None else reference)
    cleared = {} if cleared is None else cleared
    checks, next_clear = {}, {}
    for side,value in observations.items():
        epsilon=value['observation_epsilon_m']
        deepening=max(0.,value['maximum_slot_overlap_m']-reference[side])
        returned=value['returns_after_clear'] or (cleared.get(side,False) and value['maximum_slot_overlap_m']>epsilon)
        checks[side]=dict(valid=deepening<=epsilon and not returned,
            global_initial_overlap_m=reference[side],global_further_intrusion_m=deepening,
            returned_after_any_prior_clear=bool(returned))
        next_clear[side]=cleared.get(side,False) or value['first_clear_frame'] is not None
    return all(v['valid'] for v in checks.values()),reference,next_clear,checks


def native_seam(previous, current):
    values={k:float(np.max(np.abs(current[k][0]-previous[k][-1]))) for k in ('position','velocity','acceleration')}
    values['valid']=values['position']<=1e-5 and values['velocity']<=1e-4 and values['acceleration']<=1e-4
    return values


def concatenate_native(pieces):
    """Join unchanged real arrays at validated rest seams; never repair/interpolate."""
    if not pieces:raise ValueError('cannot concatenate an empty native transport')
    keys=set(pieces[0]);clocks=[];elapsed=0.;seams=[]
    for index,data in enumerate(pieces):
        t=np.asarray(data['times'])
        if set(data)!=keys or t.ndim!=1 or len(t)<2 or t[0]!=0 or not np.isfinite(t).all() or not np.all(np.diff(t)>0):
            raise ValueError('invalid native time/array schema')
        if any(len(value)!=len(t) for value in data.values()):raise ValueError('native arrays have inconsistent frame counts')
        if index:
            seam=native_seam(pieces[index-1],data);seams.append(seam)
            if not seam['valid']:raise ValueError('disconnected native rest seam')
        clocks.append(t[(1 if index else 0):].astype(np.float64)+elapsed)
        elapsed+=float(t[-1])
    merged={k:np.concatenate([data[k][(1 if i else 0):] for i,data in enumerate(pieces)]) for k in keys if k!='times'}
    merged['times']=np.concatenate(clocks)
    if not np.all(np.diff(merged['times'])>0):raise ValueError('merged native time is not strictly increasing')
    return merged,seams


def urdf_box_poses(robot, rows, joint_names, attachments):
    """Float64 batched fixed-tree FK; no mutation of the parsed URDF or its configuration."""
    from scipy.spatial.transform import Rotation
    rows=np.asarray(rows,dtype=np.float64)
    if (rows.ndim!=2 or not len(rows) or rows.shape[1]!=len(joint_names)
            or len(set(joint_names))!=len(joint_names) or not np.isfinite(rows).all()
            or not set(joint_names).issubset(robot.actuated_joint_names)):
        raise ValueError('CPU source FK requires finite complete native joint rows')
    columns={name:rows[:,i] for i,name in enumerate(joint_names)}
    parents={joint.child:joint for joint in robot.joint_map.values()}
    poses={robot.base_link:np.broadcast_to(np.eye(4),(len(rows),4,4))}
    def visit(link):
        if link not in poses:
            joint=parents[link]
            origin=np.eye(4) if joint.origin is None else joint.origin
            if joint.mimic is not None:raise ValueError('CPU source FK does not support mimic joints')
            if joint.type=='fixed':local=origin
            else:
                if joint.name not in columns:raise ValueError('missing native joint on source tool chain')
                q=columns[joint.name];axis=np.asarray(joint.axis,dtype=np.float64)
                local=np.broadcast_to(np.eye(4),(len(rows),4,4)).copy()
                if joint.type in ('revolute','continuous'):
                    local[:,:3,:3]=Rotation.from_rotvec(q[:,None]*axis/np.linalg.norm(axis)).as_matrix()
                elif joint.type=='prismatic':local[:,:3,3]=q[:,None]*axis
                else:raise ValueError('unsupported joint in CPU source FK')
                local=origin@local
            poses[link]=visit(joint.parent)@local
        return poses[link]
    return {side:visit(side+'_tool0')@attachment for side,attachment in attachments.items()}


def cpu_source_observations(robot, rows, joint_names, picks):
    """Same process limits; stateless scalar URDF FK resolves boundary rows."""
    from curobo_core.box_pick import source_slot_observation
    from curobo_core.payload_geometry import box_overlap_depth
    rows=np.asarray(rows,dtype=np.float64)
    attachments={side:pick.tool_to_box() for side,pick in picks.items()}
    poses=urdf_box_poses(robot,rows,joint_names,attachments)
    def reference(index):
        for side,matrix in reference_box_poses(robot,rows[index],joint_names,attachments).items():
            poses[side][index]=matrix
    reference(0)
    indices=set()
    for side,pick in picks.items():
        half=np.asarray(pick.dimensions_m)/2
        depth=box_overlap_depth(poses[side][:,:3,3],poses[side][:,:3,:3],half,np.asarray(pick.center_m),np.eye(3),half)
        initial=source_slot_observation(pick,poses[side][:1])
        epsilon=initial['observation_epsilon_m']
        # Dispatch guard only: recompute FK, never change the1um process limits.
        near=(np.abs(depth-epsilon)<=1e-7)|(np.abs(depth-(initial['initial_slot_overlap_m']+epsilon))<=1e-7)
        indices.update(np.flatnonzero(near).tolist())
    for index in sorted(indices-{0}):reference(index)
    return {side:source_slot_observation(picks[side],value) for side,value in poses.items()}


def reference_box_poses(robot, row, joint_names, attachments):
    """Scalar yourdfpy joint transforms, without the display scene-graph update cache."""
    values=dict(zip(joint_names,row))
    parents={joint.child:joint for joint in robot.joint_map.values()}
    poses={robot.base_link:np.eye(4)}
    def visit(link):
        if link not in poses:
            joint=parents[link]
            local,_=robot._forward_kinematics_joint(joint,q=float(values.get(joint.name,0.)))
            poses[link]=visit(joint.parent)@local
        return poses[link]
    return {side:visit(side+'_tool0')@attachment for side,attachment in attachments.items()}
