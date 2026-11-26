"""gpuq only: bounded saved-seed contact, static single-box support switch and release."""
from dataclasses import replace
import json
import sys
import hashlib
import curobo
import importlib.metadata
import time
import traceback

import numpy as np
import torch
from scipy.spatial.transform import Rotation

from audit_transfer_geometry import ROOT, BASE, OUT, NAMES, inputs, source_hashes, fk
from curobo.types import JointState
from curobo_core.contracts import PlannerAssets
from curobo_core.scene import Pose, SceneObject
from curobo_core.box_pick import BoxPick, source_slot_observation, source_slot_process_checks
from curobo_core.payload_geometry import box_sphere_depth, box_overlap_depth
from curobo_core.suction_grasp import contact_cap_spheres
from curobo_core.torque import TorqueModel
import v3_two_box_pick
from v3_batched_loaded_search import GpuValidity
from v3_contact_validity import target_in_front_of_contact_plane
from v3_box_layout import atomic_json, session_source_stamp

# Reuse immutable prepared assets: the production helper rewrites files outside this task's scope.
def prepared_assets():
    folder = ROOT / 'generated/fast_runtime_assets'
    assets = replace(PlannerAssets(), robot_config=folder/'fixed.yaml', mobile_robot_config=folder/'mobile.yaml',
                     urdf=folder/'fixed.urdf', named_poses=ROOT/'generated/zero_model_candidate_20261018/named_poses.yaml')
    assert all(p.is_file() for p in (assets.robot_config, assets.mobile_robot_config, assets.urdf, folder/'dynamics.yaml'))
    return assets, folder/'dynamics.yaml'


def dense(rows, step=.001):
    # ponytail: sampled geometry at <=1mrad joint steps; use swept collision/CCD for continuous certification.
    result = [rows[0]]
    for a, b in zip(rows[:-1], rows[1:]):
        count = max(1, int(np.ceil(np.max(np.abs(b-a))/step)))
        result.extend(a + (b-a)*t for t in np.linspace(0., 1., count+1)[1:-1])
        result.append(b)
    return np.asarray(result, dtype=np.float32)


def main(report, arrays):
    hashes, prior, owner, old = inputs()
    arrays.update(old)
    stamp = session_source_stamp()
    report.update(source_hashes_before=hashes, source_stamp_at_start=stamp,
                  old_geometry_source_stamp=prior['source_stamp'], stamp_changed_since_old=stamp != prior['source_stamp'])
    v3_two_box_pick.prepare_fast_assets = prepared_assets
    planner = v3_two_box_pick.TwoBoxPickPlanner()
    planner.configure_pick(owner)
    pending_row = next(b for b in prior['request']['boxes'] if b['side'] == 'left')
    pending = BoxPick(**pending_row)
    planner.snapshot = replace(planner.snapshot, objects=planner.snapshot.objects + (
        SceneObject('pending_original_left', pending.dimensions_m, Pose(pending.center_m)),))
    checker = planner.checker(planner.selected_task, payload=False)
    assert not getattr(checker, 'guidance_payload', False)
    assert any(o.name == 'pending_original_left' for o in checker.guidance_scene.cuboid)
    assert not any(o.name == 'wall_box_00' for o in checker.guidance_scene.cuboid), 'physical box duplicated in world'
    validity = GpuValidity(checker, check_ground=True, scene_penetration_tolerance_m=.001)
    names = checker.kinematics.config.kinematics_config
    cap_ids = {}
    for side in ('left', 'right'):
        origin = planner.fk_robot.get_transform(side+'_tool0', side+'_link7')[:3, 3]
        ids = names.get_sphere_index_from_link_name(side+'_link7').cpu().tolist()
        spheres = planner.robot['kinematics']['collision_spheres'][side+'_link7']
        assert len(ids) == len(spheres), 'empty checker has appended payload spheres'
        cap_ids[side] = [i for i, s in zip(ids, spheres) if s['radius'] > 0 and s['center'][2]+s['radius'] > origin[2]]
        assert len(cap_ids[side]) == len(contact_cap_spheres(side)) > 0
    A_left = owner.tool_to_box()
    A_right_nominal = np.asarray(prior['selected_contact_goal']['right_attachment'])
    arrays['left_attachment'] = A_left
    arrays['right_nominal_attachment'] = A_right_nominal
    approach = dense(old['receiver_contact_seed'])
    assert np.max(np.abs(approach[0]-old['owner_transport'][-1])) <= 1e-5
    tools = fk(planner.fk_robot, approach)
    C_switch = tools['left'][-1] @ A_left
    # Same physical C on both sides of the switch; never substitute the nominal receiver attachment.
    A_right = np.linalg.inv(tools['right'][-1]) @ C_switch
    arrays['right_measured_attachment'] = A_right
    arrays['receiver_approach'] = approach
    arrays['support_transfer'] = np.repeat(approach[-1:], 11, axis=0)
    arrays['support_alpha'] = np.linspace(0., 1., 11)
    with np.load(OUT/'donor_release_cpu_plan.npz', allow_pickle=False) as plan:
        for key in ('donor_release', 'donor_continue'):
            arrays[key] = dense(plan[key])
    assert np.array_equal(arrays['donor_release'][0], approach[-1])
    assert np.array_equal(arrays['donor_release'][-1], arrays['donor_continue'][0])
    report['donor_release_ik'] = json.loads((OUT/'donor_release_cpu_plan.json').read_text())
    for key in ('extract', 'owner_transport', 'receiver_approach', 'support_transfer', 'donor_release', 'donor_continue'):
        T = fk(planner.fk_robot, arrays[key])
        arrays[key+'_box'] = T['right'] @ A_right if key in ('donor_release', 'donor_continue') else T['left'] @ A_left
    pending_C = np.eye(4)
    pending_C[:3, 3] = pending.center_m
    arrays['pending_box_matrix'] = pending_C
    arrays['physical_box_id'] = np.array('right')
    arrays['pending_box_id'] = np.array('left')
    arrays['box_dimensions_m'] = np.array(owner.dimensions_m)
    policy = json.loads((ROOT/'config/fast_three_layers.json').read_text())
    assert policy['payload_kg_per_box'] == 10. and policy['planning_limit_scale'] == .995
    limits = np.asarray(policy['arm_continuous_torque_nm']*2)*policy['planning_limit_scale']
    arrays['scaled_torque_limits_nm'] = limits
    dynamics = TorqueModel(ROOT/'generated/fast_runtime_assets/dynamics.yaml', payload_mass_kg=10., payload_dimensions=owner.dimensions_m)
    models = {s: dynamics._model((s,), {s: A}) for s, A in [('left', A_left), ('right', A_right)]}
    obstacles = list(checker.guidance_scene.cuboid) + list(checker.base_cuboid_layer.scene.cuboid)
    report.update(physical_box_id='right', physical_box_count=1, physical_payload_kg=10.,
                  original_request=prior['request'], pending_original_box=pending_row,
                  receiver_actual_attachment=A_right.tolist(), receiver_nominal_attachment=A_right_nominal.tolist(),
                  receiver_attachment_position_delta_m=float(np.linalg.norm(A_right[:3, 3]-A_right_nominal[:3, 3])),
                  receiver_attachment_angle_delta_deg=float(np.rad2deg(Rotation.from_matrix(A_right_nominal[:3, :3].T @ A_right[:3, :3]).magnitude())),
                  cap_sphere_ids=cap_ids, scene_obstacle_names=[o.name for o in obstacles],
                  original_robot_spheres_only=True, scene_tolerance_m=.001, noncap_obb_tolerance_m=1e-6,
                  phase_audits={}, static_support_extremes={})

    def gravity(rows, side):
        values = torch.as_tensor(rows, device='cuda', dtype=torch.float32).contiguous()
        zeros = torch.zeros_like(values)
        model = models[side]
        model.setup_batch_size(len(values))
        return model.compute_inverse_dynamics(JointState(position=values, velocity=zeros, acceleration=zeros))[:, 1:].detach().cpu().numpy().copy()

    def check_phase(key, support):
        rows = arrays[key]
        C = arrays[key+'_box']
        T = fk(planner.fk_robot, rows)
        half = np.asarray(owner.dimensions_m)/2
        robot_mask, box_mask, cap_masks, depths_all = [], [], {s: [] for s in cap_ids}, []
        fk_difference = 0.
        for offset in range(0, len(rows), 512):
            q = torch.tensor(rows[offset:offset+512], device='cuda')
            good = validity.mask(q) & planner.native_margin_mask(checker, q)
            robot_mask.extend(good.cpu().tolist())
            state = checker.kinematics.compute_kinematics(JointState.from_position(q[None], joint_names=NAMES))
            spheres = state.robot_spheres.reshape(len(q), -1, 4).cpu().numpy().copy()
            B = C[offset:offset+len(q)]
            depth = box_sphere_depth(spheres[..., :3], spheres[..., 3], B[:, :3, 3], B[:, :3, :3], half)
            blocked = (depth > 1e-6) & (spheres[..., 3] > 0)
            for s, A in [('left', A_left), ('right', A_right_nominal)]:
                S = T[s][offset:offset+len(q)]
                pose = state.tool_poses.to_dict()[s+'_tool0']
                fk_difference = max(fk_difference, float(np.max(np.abs(pose.position.reshape(-1, 3).cpu().numpy()-S[:, :3, 3]))))
                allowed = []
                reach = float(np.max(contact_cap_spheres(s)[:, 2]+contact_cap_spheres(s)[:, 3]))
                for i in range(len(q)):
                    desired = B[i] @ np.linalg.inv(A)
                    target = dict(center=B[i, :3, 3], rotation=B[i, :3, :3], dimensions=owner.dimensions_m,
                                  contact=desired[:3, 3], inward_normal=desired[:3, 2], tool_cap_reach_m=reach)
                    plane = bool(target_in_front_of_contact_plane(torch.tensor(S[i:i+1, :3, 3], dtype=torch.float64),
                        torch.tensor(S[i:i+1, :3, 2], dtype=torch.float64), target, .001, True).all())
                    # Full actual cap footprint on the correct box face, in addition to the TCP plane.
                    local = (spheres[i, cap_ids[s], :3]-B[i, :3, 3]) @ B[i, :3, :3]
                    axis = int(np.argmax(np.abs(B[i, :3, :3].T @ desired[:3, 2])))
                    tangent = [j for j in range(3) if j != axis]
                    footprint = np.max(np.abs(local[:, tangent])+spheres[i, cap_ids[s], 3, None]-half[tangent])
                    allowed.append(plane and footprint <= .001)
                allowed = np.asarray(allowed)
                blocked[:, cap_ids[s]] &= ~allowed[:, None]
                cap_masks[s].extend(allowed.tolist())
            depths_all.append(depth)
            cap_collision = blocked.any(-1)
            box_mask.extend((~cap_collision).tolist())
        world = {}
        for obstacle in obstacles:
            R = Rotation.from_quat(np.roll(obstacle.pose[3:7], -1)).as_matrix()
            depth = box_overlap_depth(C[:, :3, 3], C[:, :3, :3], half, np.array(obstacle.pose[:3]), R, np.array(obstacle.dims)/2)
            world[obstacle.name] = float(depth.max())
        bottom = C[:, 2, 3] - (np.abs(C[:, 2, :3])*half).sum(-1)
        tau = gravity(rows, support)
        ratio = np.abs(tau)/limits
        arrays[key+'_gravity_torques_nm'] = tau
        all_depth = np.concatenate(depths_all)
        noncap_ids = [i for i in range(all_depth.shape[1]) if i not in cap_ids['left']+cap_ids['right']]
        record = dict(frames=len(rows), robot_invalid_indices=np.flatnonzero(~np.asarray(robot_mask)).tolist(),
                      box_robot_invalid_indices=np.flatnonzero(~np.asarray(box_mask)).tolist(),
                      support=support, physical_payload_kg=10., scaled_limit=.995, max_scaled_gravity_ratio=float(ratio.max()),
                      static_invalid_indices=np.flatnonzero((ratio > 1.).any(-1)).tolist(),
                      max_world_obb_depth_m=world, minimum_box_bottom_m=float(bottom.min()),
                      maximum_noncap_obb_depth_m=float(all_depth[:, noncap_ids].max()),
                      cap_plane_and_footprint_permitted_frames={s: int(np.sum(m)) for s, m in cap_masks.items()},
                      cpu_gpu_tool_position_max_error_m=fk_difference)
        record['valid'] = (not record['robot_invalid_indices'] and not record['box_robot_invalid_indices'] and
                           not record['static_invalid_indices'] and max(world.values()) <= .0010001 and bottom.min() >= -.001 and fk_difference <= 2e-5)
        if key in ('donor_release', 'donor_continue'):
            clearance = -all_depth[-1, cap_ids['left']]
            record['final_donor_cap_clearances_m'] = clearance.tolist()
            record['all_final_donor_caps_disjoint'] = bool((clearance > 0.).all())
            record['valid'] &= record['all_final_donor_caps_disjoint']
            if key == 'donor_continue':
                record['all_continuation_donor_caps_disjoint'] = bool((all_depth[:, cap_ids['left']] < 0.).all())
                record['valid'] &= record['all_continuation_donor_caps_disjoint']
        report['phase_audits'][key] = record
        print('TRANSFER_PHASE', key, json.dumps(record), flush=True)

    check_phase('receiver_approach', 'left')
    check_phase('support_transfer', 'left')
    tau = np.stack([gravity(approach[-1:], s)[0] for s in ('left', 'right')])
    arrays['support_extreme_torques_nm'] = tau
    alpha = arrays['support_alpha']
    arrays['support_torques_nm'] = (1-alpha[:, None])*tau[0] + alpha[:, None]*tau[1]
    for i, side in enumerate(('left', 'right')):
        report['static_support_extremes'][side] = dict(physical_payload_kg=10., scaled_limit=.995,
            max_scaled_gravity_ratio=float((np.abs(tau[i])/limits).max()), actual_arm_gravity_torques_nm=tau[i].tolist())
    report['support_convexity'] = dict(valid=bool((np.abs(tau) <= limits).all()),
        statement='At fixed q, qd=qdd=0, tau(alpha)=(1-alpha)*tau_left10kg+alpha*tau_right10kg. The symmetric per-joint .995 torque box is convex; both endpoints imply all alpha in [0,1]. No duplicate mass or dynamic force-sharing claim.',
        max_sampled_scaled_ratio=float((np.abs(arrays['support_torques_nm'])/limits).max()))
    check_phase('donor_release', 'right')
    check_phase('donor_continue', 'right')
    process_boxes = np.concatenate([arrays[k+'_box'] for k in ('owner_transport', 'receiver_approach', 'support_transfer', 'donor_release', 'donor_continue')])
    report['source_process'] = source_slot_process_checks({'left': source_slot_observation(owner, process_boxes)})
    contact_checks = {}
    for side, A in [('left', A_left), ('right', A_right_nominal)]:
        target = C_switch @ np.linalg.inv(A)
        actual = tools[side][-1]
        contact_checks[side] = dict(position_error_m=float(np.linalg.norm(actual[:3, 3]-target[:3, 3])),
            angle_error_deg=float(np.rad2deg(Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).magnitude())))
        contact_checks[side]['valid'] = (contact_checks[side]['position_error_m'] <= .002 and
            contact_checks[side]['angle_error_deg'] <= 1. and
            report['phase_audits']['support_transfer']['cap_plane_and_footprint_permitted_frames'][side] == 11)
    report['stationary_dual_contact'] = contact_checks
    release_tools = fk(planner.fk_robot, arrays['donor_release'])
    retreat_delta = release_tools['left'][-1, :3, 3]-release_tools['left'][0, :3, 3]
    retreat_axis = -release_tools['left'][0, :3, 2]
    report['donor_retreat_axial_m'] = float(retreat_delta @ retreat_axis)
    report['support_switch_pose_jump_max_element'] = float(np.max(np.abs(release_tools['right'][0] @ A_right-C_switch)))
    assert report['donor_retreat_axial_m'] >= .05
    assert report['support_switch_pose_jump_max_element'] <= 2e-5
    report['source_hashes_after'] = source_hashes()
    report['source_stamp_at_end'] = session_source_stamp()
    report['aggregate_stamp_changed_during_run'] = report['source_stamp_at_end'] != stamp
    report['success'] = bool(all(r['valid'] for r in report['phase_audits'].values()) and report['support_convexity']['valid'] and report['source_process']['valid'] and all(c['valid'] for c in contact_checks.values()))
    report['failure'] = None if report['success'] else 'sampled geometry or static gate failed; inspect phase_audits'


if __name__ == '__main__':
    began = time.perf_counter()
    arrays = {}
    report = dict(runtime=dict(sys_executable=sys.executable, sys_version=sys.version,
        curobo_module_path=curobo.__file__, curobo_version=getattr(curobo, '__version__', None), curobo_distribution_version=importlib.metadata.version('nvidia-curobo'),
        torch_module_path=torch.__file__, torch_version=torch.__version__, numpy_version=np.__version__),
        script_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in __import__('pathlib').Path(__file__).parent.glob('*.py')},
        input_geometry_sha256=hashlib.sha256((BASE/'transfer_owner_transport_geometry.npz').read_bytes()).hexdigest(),
        success=False, real_robot_execution=False, native_dynamic_handover_success=False,
                  full_two_box_transport_success=False, estimate='2m provisional, based on owner23.715s/static11s plus initialization',
                  scope='One physical original Y=.85 high box: recheck saved right approach, stationary two-cap contact, 10kg static support switch and left normal retreat. Sampled geometry only; no native timed or dynamic handover.')
    status = 1
    try:
        main(report, arrays)
        status = 0 if report['success'] else 1
    except Exception as error:
        report['failure'] = repr(error)
        report['traceback'] = traceback.format_exc()
        traceback.print_exc()
    finally:
        report['wall_seconds'] = time.perf_counter()-began
        if arrays:
            np.savez_compressed(OUT/'transfer_geometry.npz', **arrays)
        atomic_json(OUT/'transfer_probe.json', report)
        print('TRANSFER_RESULT', report['success'], 'wall', report['wall_seconds'], flush=True)
    raise SystemExit(status)
