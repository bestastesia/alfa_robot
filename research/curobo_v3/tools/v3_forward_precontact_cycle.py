import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from curobo_core.planner import CycleBlocked
from v3_batched_loaded_search import GpuValidity
from v3_lm_chained_cycle import LmChainedCyclePlanner


def offset_precontact(poses, distance=.03):
    result = {}
    for side, pose in poses.items():
        target = np.asarray(pose).copy()
        target[:3, 3] -= target[:3, 2] * distance
        result[side] = target
    return result


class ForwardPrecontactPlanner(LmChainedCyclePlanner):
    forward_precontact_policy = True

    def pose_candidates(self, targets, previous, checker, target_obstacles):
        validity = self.target_validity(checker) if target_obstacles else GpuValidity(checker, check_ground=True)
        margin = self.search_base_margin_m if target_obstacles else 0.
        if len(self.selected_task) == 1:
            rows = self.enumerate_single(targets[self.active_side], previous, checker)
            rows = np.asarray(rows).reshape(-1, 15)
            if len(rows):
                mask = validity.evaluate(torch.tensor(rows, device='cuda', dtype=torch.float32), base_margin_m=margin)[0]
                rows = rows[mask.cpu().numpy()]
            return rows
        swivels = {side: self.analytic.swivel(arm, previous[1 + arm * 7:8 + arm * 7]) for arm, side in enumerate(('left', 'right'))}
        case = dict(task=dict(self.selected_task), contact_joints=np.asarray(previous).tolist(), stage='endpoint_ik',
            payload=False, tcp_start_local={side: np.asarray(matrix).tolist() for side, matrix in targets.items()},
            translation_local={side: [0., 0., 0.] for side in targets}, fixed_swivels_rad=swivels,
            sample_count=16, endpoint_base_margin_m=margin, endpoint_return_limit=64)
        self.cartesian_engine.prepare(case)
        self.cartesian_engine.validity = validity
        return self.cartesian_engine.endpoint_bank()

    def initial_motion(self, boxes, empty, report, append, progress):
        geometry = self.contact_poses_for_task(boxes)
        contacts = {}
        for side in boxes:
            target = geometry[side + '_tool0']
            matrix = np.eye(4)
            matrix[:3, 3] = target['position']
            matrix[:3, :3] = Rotation.from_quat(np.roll(target['quaternion'], -1)).as_matrix()
            contacts[side] = matrix
        pretargets = offset_precontact(contacts)
        report['contact_target_poses_local'] = {side: matrix.tolist() for side, matrix in contacts.items()}
        report['precontact_target_poses_local'] = {side: matrix.tolist() for side, matrix in pretargets.items()}
        report['precontact_policy'] = dict(mode='direct_pose_ik_then_forward_lm', distance_m=.03,
            precontact_reference='initial_joint_state', reverse_segment_computed=False,
            contact_ik_after_home_approach=True, precontact_target_collision=True, final_approach_target_collision=False)
        report['endpoint_selection_policy'] = 'precontact Pose offset from task target, IK referenced only to initial joints; contact IK selected after actual Home approach; no reverse-motion lookahead'
        progress('任务吸附Pose外退3cm：仅参考初始构型直接求预接触IK')
        began = time.perf_counter()
        candidates = self.pose_candidates(pretargets, self.home_values, empty, target_obstacles=True)
        report['timing_ms']['precontact_ik_ms'] = (time.perf_counter() - began) * 1000
        report['precontact_ik_count'] = len(candidates)
        if not len(candidates):
            raise CycleBlocked('直接预接触IK', '没有满足预接触目标及碰撞条件的构型，不反推接近路径', report)
        precontact = self.record_endpoint('precontact', candidates, self.home_values, 'direct task precontact Pose IK; nearest initial configuration; no contact IK or reverse path used')
        progress('Home→已选预接触构型：LM先验＋Informed Connect')
        approach, stats = self.joint_plan(self.home_values, precontact, empty, self.target_validity(empty), 20261023)
        report['approach_search_stats'] = stats
        if approach is None:
            raise CycleBlocked('Home到直接预接触', f'未找到合法连接，不求下一段：{stats}', report)
        append(approach, 'approach', False)
        arrived = np.asarray(approach[-1]).copy()
        progress('已到预接触：仅参考实际到达构型求吸附IK')
        began = time.perf_counter()
        candidates = self.pose_candidates(contacts, arrived, empty, target_obstacles=False)
        report['timing_ms']['contact_candidates_ms'] = (time.perf_counter() - began) * 1000
        report['contact_ik_count'] = len(candidates)
        report['contact_candidates'] = candidates.tolist()
        if not len(candidates):
            raise CycleBlocked('前向吸附IK', '从实际预接触构型未找到合法吸附IK，不回选上一段', report)
        contact = self.record_endpoint('contact', candidates, arrived, 'contact Pose IK selected after actual Home-to-precontact motion; nearest actual arrived configuration')
        report['selected_candidate'] = self.serial_records[-1]['selected_index'] + 1
        progress('实际预接触→吸附：前向3cm软参考＋LM先验＋Informed Connect')
        forward = self.cartesian_segment(arrived, 'contact_forward', target_joints=contact)
        append(forward, 'contact_approach', False)
        actual_contact = forward[-1].copy()
        append([actual_contact], 'attach', True)
        return actual_contact

    def full_cycle(self, boxes, progress=lambda text: None):
        result = super().full_cycle(boxes, progress)
        result['forward_precontact_policy'] = True
        return result
