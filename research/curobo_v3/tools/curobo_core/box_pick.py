"""Two PR82 acceptance boxes; all coordinates are metres in the robot base frame."""
from dataclasses import dataclass
import copy
import itertools

import numpy as np
from scipy.spatial.transform import Rotation

# Input label is not XYZ: X is the requested extraction depth, Y face width, Z height.
BOX_DIMENSIONS = {'40x40x30': (.30, .40, .40), '46.5x25.5x58.2': (.255, .465, .582)}


def actual_owner_ids(record):
    """Actual arm -> immutable original carton ID; legacy records use label order."""
    owners = record.get('owner_to_physical_id', {'left': 'wall_box_00', 'right': 'wall_box_01'})
    if (not isinstance(owners, dict) or set(owners) != {'left', 'right'}
            or set(owners.values()) != {'wall_box_00', 'wall_box_01'}):
        raise ValueError('bijective actual owner to original physical ID required')
    return dict(owners)


@dataclass(frozen=True)
class BoxPick:
    size: str
    center_m: tuple
    side: str
    side_min_height_m: float = .80
    carry_offset_m: tuple | None = None
    precontact_margin_m: float = .005
    swap_length_height: bool = False
    carry_joint_home: bool = False
    carry_updown_m: float | None = None
    suction_offset_m: tuple = (0., 0., 0.)
    suction_roll_deg: float = 0.
    side_face: str = 'x-'

    def __post_init__(self):
        if self.size not in BOX_DIMENSIONS or self.side not in ('left', 'right'):
            raise ValueError('expected one of the two acceptance sizes and left/right arm')
        if not isinstance(self.swap_length_height, bool):
            raise ValueError('swap_length_height must be a boolean')
        center = np.asarray(self.center_m, dtype=float)
        if center.shape != (3,) or not np.isfinite(center).all():
            raise ValueError('box center must contain three finite coordinates')
        if not np.isfinite(self.side_min_height_m) or self.side_min_height_m < 0:
            raise ValueError('side suction height threshold must be finite and nonnegative')
        if center[2] < self.dimensions_m[2] / 2 - 1e-9:
            raise ValueError('box intersects ground')
        if not np.isfinite(self.precontact_margin_m) or self.precontact_margin_m < .001:
            raise ValueError('precontact margin must be finite and at least 1 mm')
        # None selects the user-approved size/mode profile (2026-11-19-v2).
        # Explicit offsets ALWAYS use the original PR82 anchor, including zero:
        # old task records must replay without adding the new default twice.
        if not isinstance(self.carry_joint_home, bool):
            raise ValueError("carry_joint_home must be a boolean")
        if self.carry_updown_m is not None:
            if not self.carry_joint_home:
                raise ValueError("carry updown requires joint-defined carry")
            from curobo_core.layout_contract import carry_tool
            carry_tool(self.side, self.carry_updown_m)
        offset = self.carry_offset_m
        if offset is None and self.carry_joint_home:
            offset = (0., 0., 0.)
        if offset is None:
            sign = 1 if self.side == 'left' else -1
            offset = (0., 0., 0.) if self.size == '40x40x30' else (
                (-.04, sign*.0135, 0.) if self.top else
                (-.21 if self.side == 'left' else -.17, sign*.135, 0.))
        offset = np.asarray(offset, dtype=float)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError('carry offset must contain three finite coordinates')
        if self.carry_joint_home and np.any(offset != 0.):
            raise ValueError('Cartesian carry offsets cannot override the specified joint angles')
        from curobo_core.suction_grasp import validate_grasp
        suction_offset = validate_grasp(self.dimensions_m, self.top, self.side, self.suction_offset_m, self.suction_roll_deg, self.side_face)
        object.__setattr__(self, 'suction_offset_m', suction_offset)
        object.__setattr__(self, 'center_m', tuple(center))
        object.__setattr__(self, 'carry_offset_m', tuple(offset))

    @property
    def dimensions_m(self):
        x, y, z = BOX_DIMENSIONS[self.size]
        return (x, z, y) if self.swap_length_height else (x, y, z)

    @property
    def top(self):
        # Equality belongs to side suction; high-top rejection is intentionally deferred.
        return self.center_m[2] < self.side_min_height_m

    @property
    def extraction_distance_m(self):
        return self.dimensions_m[0]

    def precontact_distance(self, tool_cap_reach_m):
        return max(.03, tool_cap_reach_m + self.precontact_margin_m)

    def tool_to_box(self):
        from curobo_core.suction_grasp import tool_in_box
        return np.linalg.inv(tool_in_box(self.dimensions_m, self.top, self.suction_offset_m, self.suction_roll_deg, self.side_face))

    def contact_tool(self):
        box = np.eye(4)
        box[:3, 3] = self.center_m
        return box @ np.linalg.inv(self.tool_to_box())

    def carry_box(self):
        if self.carry_joint_home:
            from curobo_core.layout_contract import carry_tool
            return carry_tool(self.side, 0. if self.carry_updown_m is None else self.carry_updown_m) @ self.tool_to_box()
        # PR82's actual box pitch is -45deg (the policy label says 45deg).
        # Physical box pose is identical for side/top suction, not the old flat top carry.
        box = np.eye(4)
        box[:3, :3] = Rotation.from_euler('y', -45, degrees=True).as_matrix()
        box[:3, 3] = np.asarray([.55, .425 if self.side == 'left' else -.425, 1.2]) + self.carry_offset_m
        return box

    def carry_tool(self):
        return self.carry_box() @ np.linalg.inv(self.tool_to_box())


    def carry_payload_envelope(self, fit, footprint_xy=(0.,0.)):
        box=self.carry_box();footprint=np.asarray(footprint_xy)
        centers=np.asarray(fit['centers'])@box[:3,:3].T+box[:3,3]
        radii=np.linalg.norm(centers[:,:2]-footprint,axis=1)+fit['radii']
        proxy=np.max(radii)
        displacement=.002+2*np.linalg.norm(fit['centers'],axis=1)*np.sin(np.deg2rad(1.)/2)
        lower_bound=np.max(radii-displacement)
        corners=np.asarray(list(itertools.product((-1.,1.),repeat=3)))*np.asarray(self.dimensions_m)/2
        corners=corners@box[:3,:3].T+box[:3,3]
        exact=np.max(np.linalg.norm(corners[:,:2]-footprint,axis=1))
        return dict(legacy_payload_sphere_radius_m=float(proxy),box_corner_radius_m=float(exact),
            legacy_radius_lower_bound_with_pose_tolerance_m=float(lower_bound))


def payload_fit(source, dimensions):
    """Reuse the frozen 40-sphere topology; exact OBB audit must certify each output."""
    fit = copy.deepcopy(source)
    scale = np.asarray(dimensions) / source['dimensions_m']
    fit['dimensions_m'] = list(dimensions)
    fit['centers'] = (np.asarray(source['centers']) * scale).tolist()
    # ponytail: inscribed scaled spheres leave gaps; exact_payload_audit checks full cuboids.
    fit['radii'] = (np.asarray(source['radii']) * scale.min()).tolist()
    fit['size_adaptation'] = 'inscribed anisotropic scaling; not a cuboid coverage certificate'
    return fit


def timed_path_checks(arrays, start, end, policy):
    """Check native sampled output, not the input polyline or solver success flag."""
    times = np.asarray(arrays['times'])
    count = len(times)
    for name, width in (('position', 15), ('velocity', 15), ('acceleration', 15), ('jerk', 15), ('torque', 14)):
        values = np.asarray(arrays[name])
        if values.shape != (count, width) or not np.isfinite(values).all():
            raise ValueError(f'invalid sampled {name}')
    if count < 2 or times.ndim != 1 or not np.isfinite(times).all() or times[0] != 0 or not np.all(np.diff(times) > 0):
        raise ValueError('invalid sampled clock')
    endpoint_error = float(np.max(np.abs(arrays['position'][[0, -1]] - np.asarray([start, end]))))
    endpoint_speed = float(np.max(np.abs(arrays['velocity'][[0, -1]])))
    endpoint_acceleration = float(np.max(np.abs(arrays['acceleration'][[0, -1]])))
    velocity_ratio = float(np.max(
        np.abs(arrays['velocity']) / np.r_[1e4, np.full(14, np.deg2rad(policy['arm_velocity_deg_s']))]))
    jerk_ratio = float(np.max(np.abs(arrays['jerk']) / np.r_[1e6, np.full(14, policy['arm_jerk_rad_s3'])]))
    torque_ratio = float(np.max(np.abs(arrays['torque']) / (policy['arm_continuous_torque_nm'] * 2)))
    acceleration_ratio = float(np.max(np.abs(arrays['acceleration'])) / 1e6)
    return dict(valid=bool(endpoint_error <= 1e-5 and endpoint_speed <= 1e-4 and endpoint_acceleration <= 1e-4
        and max(velocity_ratio, jerk_ratio, torque_ratio, acceleration_ratio) <= 1.001),
        endpoint_error_rad=endpoint_error, endpoint_velocity=endpoint_speed, endpoint_acceleration=endpoint_acceleration,
        peak_velocity_ratio=velocity_ratio, peak_jerk_ratio=jerk_ratio, peak_torque_ratio=torque_ratio,
        peak_acceleration_ratio=acceleration_ratio, samples=count, duration_s=float(times[-1]))


def diverse_contact_order(rows, side):
    """Keep every IK candidate, but try an alternate elbow family before near-duplicates."""
    rows=np.asarray(rows)
    if side not in ('left','right') or rows.ndim!=2 or rows.shape[1]!=15 or not np.isfinite(rows).all():
        raise ValueError('expected finite 15-axis candidates and a known arm')
    order=np.arange(len(rows))
    if len(rows)<2:return order
    joint4=4 if side=='left' else 11
    other=np.flatnonzero((rows[:,joint4]>=0)!=(rows[0,joint4]>=0))
    if not len(other):return order
    nearest=np.flatnonzero((rows[:,joint4]>=0)==(rows[0,joint4]>=0))
    selected=[]
    for index in range(max(len(nearest),len(other))):
        if index<len(nearest):selected.append(nearest[index])
        if index<len(other):selected.append(other[index])
    return np.asarray(selected,dtype=int)


def native_rejection_reasons(stage):
    """Expose actual failed gates, not a generic solver rejection or a claim of illegality."""
    if stage.get('success'):
        return []
    reasons = []
    if stage.get('error'):reasons.append(stage['error'])
    traj = stage.get('trajectory_optimization', {})
    if not traj.get('solver_success', True):
        reasons.append('TrajOpt solver rejected its optimized path')
        for name, values in traj.get('native_constraint_details', {}).items():
            if values['maximum'] > 0:reasons.append(f"{name}: max={values['maximum']:.6g}")
    native = stage.get('time_optimization', {})
    if not native.get('success', False):
        reasons.append(f"TimeOpt status={native.get('status')}: {native.get('failure_diagnostics', {})}")
    if stage.get('collision_invalid_frames'):
        reasons.append(f"sampled robot/world/self collision or native clearance failed: {len(stage['collision_invalid_frames'])} samples")
    payload = stage.get('payload_audit', {})
    if payload.get('valid') is False:
        reasons.append(f"full-size payload collision/ground: {payload.get('failures', [])[:3]}")
    checks = stage.get('cartesian_line_checks', {})
    if 'valid' in checks:checks = {'arm': checks}
    for side, check in checks.items():
        if not check['valid']:reasons.append(f'{side} Cartesian line/orientation: {check}')
    for key in ('array_checks', 'independent_rnea', 'carry_envelope_final'):
        if stage.get(key, {}).get('valid') is False:reasons.append(f'{key}: {stage[key]}')
    return reasons or ['native dynamics, seam, or sampled-path gate rejected the candidate; inspect saved report']


def paired_carry_candidates(picks, adapt=True):
    """Keep 45deg and X extraction widths; only adapt held XYZ for the shared lift."""
    from dataclasses import replace
    if not adapt or all(p.carry_joint_home for p in picks.values()):
        yield picks
        return
    # ponytail: finite calibration proposals, not a proof of joint reachability.
    # A tall side-gripped box needs a different shoulder height than a cube/top grip.
    shifts = ((0., 0.), (-.1, .1), (.1, -.1), (-.2, .2), (.2, -.2),
              (-.3, .3), (.3, -.3), (0., .4), (.4, 0.), (0., .6), (.6, 0.),
              (0., .8), (.8, 0.), (-.2, .8), (.8, -.2))
    side_gripped_tall = [s for s, p in picks.items() if p.size == '46.5x25.5x58.2' and not p.top]
    if side_gripped_tall:
        # Lift long side grips above the frozen column while keeping the other
        # arm's proven carry posture; each proposal still passes all native gates.
        preferred = tuple(.8 if side in side_gripped_tall else 0. for side in ('left', 'right'))
        shifts = (preferred, *(item for item in shifts if item != preferred))
    for shifts_z in shifts:
        candidate = {}
        for side, dz in zip(('left', 'right'), shifts_z):
            pick = picks[side]
            xyz = pick.carry_box()[:3, 3].copy()
            if pick.size == '46.5x25.5x58.2' and pick.swap_length_height:
                # Wider Y: avoid the column for top grip and keep side-grip box corners inside the cylinder.
                x, y = (.43, .50) if pick.top else (.34, .56)
                xyz[:2] = (x, y if side == 'left' else -y)
            xyz[2] += dz
            anchor = np.array([.55, .425 if side == 'left' else -.425, 1.2])
            candidate[side] = replace(pick, carry_offset_m=tuple(xyz-anchor))
        yield candidate


def layout_model_key(picks):
    """Payload dynamics/collision topology, independent of box/carry world XYZ."""
    return tuple((side, tuple(picks[side].dimensions_m), tuple(picks[side].tool_to_box().ravel()))
                 for side in ('left', 'right'))


def fixed_extraction_blockages(picks, cuboids):
    """Strong endpoint impossibility witnesses, not a test of IK search success.

    All input boxes start world-axis aligned. The final extraction endpoint must
    be one X width behind the original center and within the existing Cartesian
    tolerances. Enclose those tolerances generously; reject only penetration
    deeper than that entire pose envelope plus the world penetration allowance.
    """
    from curobo_core.payload_geometry import box_overlap_depth
    failures = []
    for side, pick in picks.items():
        half = np.asarray(pick.dimensions_m)/2
        endpoint = np.asarray(pick.center_m)-[pick.extraction_distance_m, 0., 0.]
        # 6mm conservatively includes contact/extraction tolerances and FK seams;
        # this is a rejection-proof envelope, NOT a relaxed trajectory gate.
        envelope = .006+2*np.linalg.norm(half)*np.sin(np.deg2rad(1.)/2)+1e-6
        for box in cuboids:
            rotation = Rotation.from_quat(np.roll(box.pose[3:7], -1)).as_matrix()
            depth = float(box_overlap_depth(endpoint[None], np.eye(3)[None], half,
                np.asarray(box.pose[:3]), rotation, np.asarray(box.dims)/2)[0])
            if depth > envelope+.0010001:
                failures.append(dict(side=side, obstacle=box.name, endpoint_m=endpoint.tolist(),
                    overlap_m=depth, pose_envelope_m=float(envelope), world_allowance_m=.0010001,
                    kind='fixed_body_blocks_required_extraction_endpoint'))
    return failures


def extraction_endpoint_proposals(picks, lift_spheres, lift_axis, lift_bounds, lift_seeds, *, terminal_geometry=None, native_base_geometry=None):
    """Search seeds inside the existing box-pose tolerances; never certify reachability.

    These lift-attached spheres only guide the search. Every proposal still needs
    joint IK, all robot/world/full-payload gates and a continuous extraction path.
    """
    from scipy.optimize import minimize
    from curobo_core.payload_geometry import box_sphere_depth
    sides = tuple(picks)
    spheres = np.asarray(lift_spheres, dtype=float)
    axis = np.asarray(lift_axis, dtype=float)
    if (spheres.ndim != 2 or spheres.shape[1] != 4 or not len(spheres)
            or not np.isfinite(spheres).all() or axis.shape != (3,) or not np.isfinite(axis).all()
            or not np.isclose(np.linalg.norm(axis), 1.)):
        raise ValueError('endpoint seeds require finite nonempty lift-attached spheres and a unit lift axis')
    terminals = None
    if terminal_geometry is not None:
        if set(terminal_geometry) != set(sides):raise ValueError('terminal guidance must match requested sides')
        terminals = {}
        for side, geometry in terminal_geometry.items():
            local = np.asarray(geometry['spheres'], dtype=float)
            mask = np.asarray(geometry['mask'], dtype=bool)
            if (local.ndim != 2 or local.shape[1] != 4 or not len(local) or not np.isfinite(local).all()
                    or not (local[:, 3] > 0).all() or mask.shape != (len(spheres),)):
                raise ValueError('terminal guidance requires finite positive original spheres and a matching body-pair mask')
            terminals[side] = (local, mask)
    base = None
    if native_base_geometry is not None:
        base = native_base_geometry
        centers, rotations, halves = [np.asarray(base[name], dtype=float) for name in ('centers', 'rotations', 'halves')]
        activation = float(base['activation_m'])
        payload = {side: np.asarray(base['spheres'][side], dtype=float) for side in sides}
        if (centers.ndim != 2 or centers.shape[1:] != (3,) or not len(centers)
                or rotations.shape != (len(centers), 3, 3) or halves.shape != centers.shape
                or not all(np.isfinite(v).all() for v in (centers, rotations, halves))
                or not (halves > 0).all() or not np.isfinite(activation) or activation < 0
                or any(v.ndim != 2 or v.shape[1:] != (4,) or not len(v)
                       or not np.isfinite(v).all() or not (v[:, 3] > 0).all() for v in payload.values())):
            raise ValueError('native base guidance requires finite original cuboids, payload spheres and activation')
    lo, hi = map(float, lift_bounds)
    if not np.isfinite([lo, hi]).all() or lo > hi:
        raise ValueError('invalid lift bounds')
    origins = {s: np.asarray(p.center_m)-[p.extraction_distance_m, 0., 0.] for s, p in picks.items()}
    # Reserve 0.2mm / 0.05deg for IK/numerics; this is NOT a new acceptance gate.
    angle, translation = .95, 1.8  # optimization units: degrees and millimetres
    def boxes(values):
        result = {}
        for i, side in enumerate(sides):
            pose = np.eye(4)
            pose[:3, :3] = Rotation.from_rotvec(np.deg2rad(values[1+6*i:4+6*i])).as_matrix()
            pose[:3, 3] = origins[side]+values[4+6*i:7+6*i]/1000
            result[side] = pose
        return result
    def residual(values):
        moved = spheres[:, :3]+values[0]*axis
        poses = boxes(values)
        overlaps = [float(box_sphere_depth(moved[None], spheres[None, :, 3], pose[None, :3, 3],
            pose[None, :3, :3], np.asarray(picks[side].dimensions_m)/2).max()) for side, pose in poses.items()]
        if terminals is not None:
            for side, pose in poses.items():
                local, mask = terminals[side]
                if mask.any():
                    end = local[:, :3]@pose[:3, :3].T+pose[:3, 3]
                    depth = local[:, None, 3]+spheres[None, mask, 3]-np.linalg.norm(end[:, None]-moved[None, mask], axis=-1)
                    overlaps.append(float(depth.max()))
        if base is not None:
            for side, pose in poses.items():
                local = payload[side]
                world = local[:, :3]@pose[:3, :3].T+pose[:3, 3]
                delta = np.einsum('sbi,bij->sbj', world[:, None]-centers[None], rotations)
                outside = np.abs(delta)-halves[None]
                distance = np.linalg.norm(np.maximum(outside, 0.), axis=-1)+np.minimum(outside.max(-1), 0.)
                overlaps.append(float((local[:, 3, None]+activation-distance).max()))
        return 1000*max(overlaps)
    def constraints(values):
        return np.array([limit**2-np.sum(values[start:start+3]**2) for i in range(len(sides))
            for start, limit in ((1+6*i, angle), (4+6*i, translation))])
    proposals = []
    # ponytail: four local seeds, not an exhaustive pose/IK search or illegality proof.
    seeds = list(dict.fromkeys(float(np.clip(seed, lo, hi)) for seed in lift_seeds))[:4]
    for lift in seeds:
        values = np.zeros(1+6*len(sides)); values[0] = lift
        solution = minimize(residual, values, method='SLSQP',
            bounds=[(lo, hi)]+([(-angle, angle)]*3+[(-translation, translation)]*3)*len(sides),
            constraints=[dict(type='ineq', fun=constraints)], options=dict(maxiter=350, ftol=1e-8))
        values = solution.x.copy()
        if not np.isfinite(values).all():continue
        values[0] = np.clip(values[0], lo, hi)
        for i in range(len(sides)):
            for start, limit in ((1+6*i, angle), (4+6*i, translation)):
                vector = values[start:start+3]
                vector *= min(1., limit/max(float(np.linalg.norm(vector)), 1e-300))
        proposals.append(dict(lift_m=float(values[0]), boxes={s:p.tolist() for s,p in boxes(values).items()},
            guide_overlap_m=residual(values)/1000, optimizer_success=bool(solution.success),
            seed_lift_m=lift, scope='search proposal only; no IK, collision or path certification'))
        if terminals is not None:proposals[-1]['guidance_geometry'] = 'box_and_original_terminal_self'
        if base is not None:proposals[-1]['native_base_guidance'] = True
    return sorted(proposals, key=lambda p:p['guide_overlap_m'])


def source_slot_observation(pick, poses):
    """Observe an empty source region; do not silently turn it into an obstacle.

    Physical success is separate from a process rule about full clearance or
    later reentry. The latter has not yet been agreed with the user.
    """
    from curobo_core.payload_geometry import box_overlap_depth
    poses = np.asarray(poses, dtype=float)
    if poses.ndim != 3 or poses.shape[1:] != (4, 4) or not len(poses) or not np.isfinite(poses).all():
        raise ValueError('source-slot observation requires finite nonempty [N,4,4] poses')
    half = np.asarray(pick.dimensions_m)/2
    depths = box_overlap_depth(poses[:, :3, 3], poses[:, :3, :3], half,
        np.asarray(pick.center_m), np.eye(3), half)
    epsilon = 1e-6  # Observation resolution, not an approved process tolerance.
    clear = np.flatnonzero(depths <= epsilon)
    first_clear = int(clear[0]) if len(clear) else None
    returned = np.flatnonzero((depths > epsilon) & np.maximum.accumulate(depths <= epsilon))
    initial = max(0., float(depths[0]))
    maximum = max(0., float(depths.max()))
    plane = float(pick.center_m[0]-half[0])
    front = float(poses[0, 0, 3]+np.abs(poses[0, 0, :3])@half)
    return dict(diagnostic_only=True, observation_epsilon_m=epsilon,
        extraction_end_front_plane_excess_m=front-plane,
        initial_slot_overlap_m=initial, maximum_slot_overlap_m=maximum,
        deeper_than_initial_m=max(0., maximum-initial),
        max_center_forward_m=float((poses[:, 0, 3]-poses[0, 0, 3]).max()),
        first_clear_frame=first_clear, first_return_frame=int(returned[0]) if len(returned) else None,
        returns_after_clear=bool(len(returned)))


def source_slot_process_checks(observations):
    """Transport may finish clearing extraction residuals, never deepen or reenter.

    The existing 1um observation resolution is numerical, not physical clearance.
    This does not impose the stronger entire-carton/front-plane condition.
    """
    checks, reasons = {}, []
    for side, observation in observations.items():
        failures = []
        if observation['deeper_than_initial_m'] > observation['observation_epsilon_m']:
            failures.append(f"{side} transport deepens original-slot overlap by {observation['deeper_than_initial_m']*1000:.3f}mm")
        if observation['returns_after_clear']:
            failures.append(f"{side} transport reenters original slot at frame {observation['first_return_frame']}")
        if observation['first_clear_frame'] is None:
            failures.append(f"{side} transport never fully separates from original slot")
        checks[side] = dict(valid=not failures, rejection_reasons=failures)
        reasons.extend(failures)
    return dict(status='passed' if checks and not reasons else 'failed', valid=bool(checks and not reasons),
        policy='no_deeper_overlap_and_no_return_after_clear_v1', diagnostic_only=False,
        checks=checks, rejection_reasons=reasons, source_slot_observation=observations,
        scope='each carton versus its own original volume; no full-front-plane condition; sampled trajectories, not CCD')
