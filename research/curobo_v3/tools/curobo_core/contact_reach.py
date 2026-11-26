"""Conservative contact infeasibility witnesses for the frozen SRS arms.

No witness means unknown, not reachable. These bounds never replace IK/collision
checks: they only distinguish a geometric impossibility from a search failure.
"""
import numpy as np


def _axis_center(points, axes):
    projectors = np.eye(3)-axes[:, :, None]*axes[:, None, :]
    center = np.linalg.solve(projectors.sum(0), (projectors@points[:, :, None]).sum(0)[:, 0])
    residual = np.linalg.norm((projectors@(center-points)[:, :, None])[:, :, 0], axis=1).max()
    return center, float(residual)


def urdf_reach_model(robot, offsets, signs):
    """Read actual URDF geometry at legacy zero; restore the caller's FK state."""
    original = robot.cfg.copy()
    if robot.joint_map['updown'].type != 'prismatic':raise ValueError('reach certificate requires a prismatic shared lift')
    reference = {name: 0. for name in robot.actuated_joint_names}
    for arm, side in enumerate(('left', 'right')):
        reference.update({side+'_joint'+str(i+1): float(-offsets[arm, i]/signs[i]) for i in range(7)})
    models = {}
    try:
        robot.update_cfg(reference)
        origins = {s: robot.get_transform(s+'_link1', robot.base_link)[:3, 3].copy() for s in ('left','right')}
        for arm, side in enumerate(('left', 'right')):
            transforms = [robot.get_transform(side+'_link'+str(i+1), robot.base_link) for i in range(7)]
            points = np.asarray([t[:3, 3] for t in transforms])
            axes = np.asarray([t[:3, :3]@robot.joint_map[side+'_joint'+str(i+1)].axis for i, t in enumerate(transforms)])
            axes /= np.linalg.norm(axes, axis=1)[:, None]
            centers = [_axis_center(points[list(ids)], axes[list(ids)]) for ids in ((0,1,2), (2,3,4), (4,5,6))]
            shoulder, elbow, wrist = [c[0] for c in centers]
            residual = max(c[1] for c in centers)
            upper, fore = np.linalg.norm(elbow-shoulder), np.linalg.norm(wrist-elbow)
            direction = (elbow-shoulder)/upper
            alignment = np.linalg.norm(np.cross(direction, axes[0]))
            if residual > 1e-5 or alignment > 1e-5 or min(upper, fore) < .01:
                raise ValueError('URDF is not the certified spherical shoulder/elbow/wrist model')
            tool = robot.get_transform(side+'_tool0', robot.base_link)
            offset = tool[:3, :3].T@(tool[:3, 3]-wrist)
            limit = robot.joint_map[side+'_joint2'].limit
            lo, hi = sorted(offsets[arm, 1]+signs[1]*np.asarray([limit.lower, limit.upper]))
            contains_pi = np.ceil((lo-np.pi)/(2*np.pi)) <= np.floor((hi-np.pi)/(2*np.pi))
            cosine = -1. if contains_pi else min(np.cos(lo), np.cos(hi))
            models[side] = dict(shoulder=shoulder, upper=float(upper), fore=float(fore), direction=direction,
                tool_offset=offset, cosine=float(cosine), geometry_error_m=max(1e-5, 100*residual+2*upper*alignment))
        robot.update_cfg(dict(reference, updown=.1))
        axis = (robot.get_transform('left_link1', robot.base_link)[:3, 3]-origins['left'])/.1
        right_axis = (robot.get_transform('right_link1', robot.base_link)[:3, 3]-origins['right'])/.1
        if not np.allclose(axis,right_axis,atol=1e-8,rtol=0):raise ValueError('arms do not share this lift')
        if not np.isclose(np.linalg.norm(axis), 1., atol=1e-6):raise ValueError('lift has an unexpected scale')
        models['lift_axis'] = axis
        return models
    finally:robot.update_cfg(original)


def _cap_projection(point, direction, radius, cosine):
    """Euclidean projection onto ball(radius) intersect dot(direction,x)>=radius*cosine."""
    height = radius*cosine
    length = np.linalg.norm(point)
    if length <= radius and point@direction >= height:return point
    if length > 0:
        radial = point*min(1., radius/length)
        if radial@direction >= height:return radial
    parallel = point@direction
    transverse = point-parallel*direction
    disk_radius = np.sqrt(max(0., radius*radius-height*height))
    return height*direction+transverse*min(1., disk_radius/max(float(np.linalg.norm(transverse)), 1e-300))


def contact_reach_witnesses(picks, model, lift_bounds):
    """Prove only strong unreachable assigned-arm contacts, including pose tolerance.

    Joint2 restricts the elbow to a spherical cap. Relax that cap to a solid
    convex cap and let the forearm point anywhere. Even this larger set must
    reach the desired wrist. The squared distance to a convex set is convex;
    a supporting line gives a lower bound over the WHOLE shared-lift interval.
    """
    axis = np.asarray(model['lift_axis'])
    norm2 = float(axis@axis)
    lo, hi = map(float, lift_bounds)
    ranges, deltas, envelopes = {}, {}, {}
    for side, pick in picks.items():
        arm = model[side]; target = pick.contact_tool()
        wrist = target[:3, 3]-target[:3, :3]@arm['tool_offset']
        delta = wrist-arm['shoulder']
        envelope = .006+2*np.linalg.norm(arm['tool_offset'])*np.sin(np.deg2rad(1.)/2)+arm['geometry_error_m']
        square = float(delta@delta)
        if not np.isfinite(square):return []  # No numerical-overflow "proof".
        along = float(delta@axis)/norm2
        perpendicular2 = max(0., square-float(delta@axis)**2/norm2)
        radius = arm['upper']+arm['fore']+envelope
        if perpendicular2 > (radius+1e-5)**2:
            return [dict(kind='assigned_arm_wrist_outside_reach', side=side,
                wrist_distance_lower_m=float(np.sqrt(perpendicular2)), arm_reach_upper_m=radius,
                pose_envelope_m=float(envelope))]
        span = np.sqrt(max(0., radius*radius-perpendicular2)/norm2)
        ranges[side] = [along-span, along+span]
        lo, hi = max(lo, along-span), min(hi, along+span)
        deltas[side], envelopes[side] = delta, float(envelope)
    if lo > hi+1e-6:
        return [dict(kind='no_shared_lift_reach_interval', allowed_lift_m=list(map(float,lift_bounds)),
                     arm_lift_intervals_m={s:list(map(float,v)) for s,v in ranges.items()})]
    if lo > hi:return []
    witnesses = []
    for side, delta in deltas.items():
        arm = model[side]
        def value_gradient(lift):
            point = delta-lift*axis
            projection = _cap_projection(point, arm['direction'], arm['upper'], arm['cosine'])
            residual = point-projection
            return float(residual@residual), float(-2*axis@residual)
        left, right = lo, hi
        if value_gradient(left)[1] >= 0:point = left
        elif value_gradient(right)[1] <= 0:point = right
        else:
            for _ in range(64):
                point = (left+right)/2
                if value_gradient(point)[1] < 0:left = point
                else:right = point
            point = (left+right)/2
        value, gradient = value_gradient(point)
        # Valid at ANY point, not a claim that finite minimization found the optimum.
        squared_lower = value+min(gradient*(lo-point), gradient*(hi-point))-1e-10
        lower = float(np.sqrt(max(0., squared_lower)))
        gap = lower-arm['fore']-envelopes[side]
        if gap > 1e-5:
            witnesses.append(dict(kind='assigned_arm_joint2_reach_bound', side=side,
                shared_lift_interval_m=[float(lo),float(hi)], wrist_to_elbow_distance_lower_m=lower,
                forearm_length_m=arm['fore'], pose_envelope_m=envelopes[side], unreachable_gap_m=float(gap),
                scope='required contact for this assigned arm, not a global rejection of the box position'))
    return witnesses
