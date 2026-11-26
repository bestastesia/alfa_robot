"""NumPy URDF Newton-Euler and full single-OBB contact checks; no CUDA imports."""
import numpy as np
from scipy.spatial.transform import Rotation
from curobo_core.payload_geometry import box_sphere_depth, box_overlap_depth
from curobo_core.box_pick import source_slot_observation, source_slot_process_checks

NAMES=['updown']+[f'{s}_joint{i}' for s in ('left','right') for i in range(1,8)]

def forward(robot, q, v=None, a=None):
    q=np.asarray(q,dtype=np.float64)
    v=np.zeros_like(q) if v is None else np.asarray(v,dtype=np.float64)
    a=np.zeros_like(q) if a is None else np.asarray(a,dtype=np.float64)
    n=len(q); columns={name:i for i,name in enumerate(NAMES)}
    parents={j.child:j for j in robot.joint_map.values()}
    identity=np.broadcast_to(np.eye(4),(n,4,4)).copy()
    zero=np.zeros((n,3))
    poses={robot.base_link:identity}; motion={robot.base_link:(zero,zero,zero)}; axes={}; order=[]
    def visit(link):
        if link in poses:return
        j=parents[link];visit(j.parent)
        T=poses[j.parent]; origin=np.eye(4) if j.origin is None else j.origin
        local=np.broadcast_to(origin,(n,4,4)).copy()
        axis=np.asarray(j.axis,dtype=float)
        col=columns.get(j.name)
        values=np.zeros(n) if col is None else q[:,col]
        if j.type in ('revolute','continuous'):
            local[:,:3,:3]=origin[:3,:3]@Rotation.from_rotvec(values[:,None]*axis).as_matrix()
        elif j.type=='prismatic':local[:,:3,3]+=values[:,None]*(origin[:3,:3]@axis)
        poses[link]=T@local
        direction=np.einsum('nij,j->ni',poses[link][:,:3,:3],axis);axes[link]=direction
        w,alpha,acc=motion[j.parent]
        r=poses[link][:,:3,3]-T[:,:3,3]
        angular=w.copy(); angular_a=alpha.copy()
        linear_a=acc+np.cross(alpha,r)+np.cross(w,np.cross(w,r))
        if col is not None:
            if j.type in ('revolute','continuous'):
                angular+=direction*v[:,col,None]
                angular_a+=direction*a[:,col,None]+np.cross(w,direction*v[:,col,None])
            elif j.type=='prismatic':linear_a+=direction*a[:,col,None]+2*np.cross(w,direction*v[:,col,None])
        motion[link]=(angular,angular_a,linear_a);order.append(link)
    for link in robot.link_map:visit(link)
    return poses,motion,axes,order,parents

def rnea(robot,q,v,a,support,attachment,dimensions,mass=10.):
    poses,motion,axes,order,parents=forward(robot,q,v,a)
    forces={};moments={}
    for link in [robot.base_link]+order:
        T=poses[link];R=T[:,:3,:3];w,alpha,acc=motion[link]
        F=np.zeros((len(q),3));M=F.copy()
        bodies=[];inertial=robot.link_map[link].inertial
        if inertial is not None:
            Iorigin=np.eye(4) if inertial.origin is None else inertial.origin
            bodies.append((inertial.mass,Iorigin[:3,3],Iorigin[:3,:3]@inertial.inertia@Iorigin[:3,:3].T))
        if link==support+'_tool0':
            dims=np.asarray(dimensions);I=np.diag(mass/12*np.array([dims[1]**2+dims[2]**2,dims[0]**2+dims[2]**2,dims[0]**2+dims[1]**2]))
            bodies.append((mass,attachment[:3,3],attachment[:3,:3]@I@attachment[:3,:3].T))
        for m,com,inertia in bodies:
            r=np.einsum('nij,j->ni',R,com)
            force=m*(acc+np.cross(alpha,r)+np.cross(w,np.cross(w,r))-[0,0,-9.81])
            I=R@inertia@R.transpose(0,2,1)
            moment=np.einsum('nij,nj->ni',I,alpha)+np.cross(w,np.einsum('nij,nj->ni',I,w))+np.cross(r,force)
            F+=force;M+=moment
        forces[link]=F;moments[link]=M
    torque=np.zeros_like(q,dtype=np.float64)
    for link in reversed(order):
        j=parents[link]
        if j.name in NAMES:
            torque[:,NAMES.index(j.name)]=np.sum((forces[link] if j.type=='prismatic' else moments[link])*axes[link],axis=-1)
        r=poses[link][:,:3,3]-poses[j.parent][:,:3,3]
        forces[j.parent]+=forces[link];moments[j.parent]+=moments[link]+np.cross(r,forces[link])
    return torque[:,1:]

def geometry_audit(robot,rows,support,attachments,nominal,dimensions,sphere_manifest,obstacles,owner):
    poses,_,_,_,_=forward(robot,rows)
    C=poses[support+'_tool0']@attachments[support]
    half=np.asarray(dimensions)/2
    centers=[];radii=[];caps={s:[] for s in ('left','right')}
    for entry in sphere_manifest:
        T=poses[entry['link']];centers.append(np.einsum('nij,j->ni',T[:,:3,:3],entry['center'])+T[:,:3,3]);radii.append(entry['radius'])
        for s in caps:
            if entry['cap_side']==s:caps[s].append(len(radii)-1)
    centers=np.stack(centers,axis=1);radii=np.asarray(radii)
    depths=box_sphere_depth(centers,np.broadcast_to(radii,(len(rows),len(radii))),C[:,:3,3],C[:,:3,:3],half)
    blocked=(depths>1e-6)&(radii>0)
    cap_counts={};cap_clear={}
    for s,ids in caps.items():
        S=poses[s+'_tool0'];desired=C@np.linalg.inv(nominal[s]);inward=desired[:,:3,2]
        delta=desired[:,:3,3]-S[:,:3,3];axial=np.sum(delta*inward,axis=-1)
        lateral=np.linalg.norm(delta-axial[:,None]*inward,axis=-1)
        normal=S[:,:3,2];extent=np.sum(np.abs(np.einsum('ni,nij->nj',normal,C[:,:3,:3]))*half,axis=-1)
        signed=np.sum((C[:,:3,3]-S[:,:3,3])*normal,axis=-1)-extent
        reach=max(entry['tool_cap_reach_m'] for entry in sphere_manifest if entry['cap_side']==s)
        plane=(np.sum(normal*inward,axis=-1)>=np.cos(np.deg2rad(1)))&(lateral<=.005)&(axial>=-.001)&(axial<=reach+.005)&(signed>=-.001)
        local=np.einsum('nsi,nij->nsj',centers[:,ids]-C[:,None,:3,3],C[:,:3,:3])
        axis=np.argmax(np.abs(np.einsum('nji,nj->ni',C[:,:3,:3],inward)),axis=-1)
        excess=np.abs(local)+radii[ids][None,:,None]-half
        excess=np.where(np.arange(3)[None,None,:]==axis[:,None,None],-np.inf,excess)
        allowed=plane&(excess.max(axis=(1,2))<=.001)
        blocked[:,ids]&=~allowed[:,None]
        cap_counts[s]=int(allowed.sum());cap_clear[s]=float(-depths[:,ids].max())
    world={o['name']:float(box_overlap_depth(C[:,:3,3],C[:,:3,:3],half,np.asarray(o['pose'][:3]),
        Rotation.from_quat(np.roll(o['pose'][3:7],-1)).as_matrix(),np.asarray(o['dims'])/2).max()) for o in obstacles}
    bottom=C[:,2,3]-np.sum(np.abs(C[:,2,:3])*half,axis=-1)
    source=source_slot_process_checks({'left':source_slot_observation(owner,C)})
    bad=np.flatnonzero(blocked.any(-1))
    return dict(valid=bool(not len(bad) and max(world.values())<=.0010001 and bottom.min()>=-.001 and source['valid']),
        samples=len(rows),box_robot_invalid_count=len(bad),box_robot_first_invalid_indices=bad[:20].tolist(),
        maximum_world_obb_depth_m=world,minimum_box_bottom_m=float(bottom.min()),source_process=source,
        cap_permitted_frames=cap_counts,minimum_cap_clearance_m=cap_clear),C
