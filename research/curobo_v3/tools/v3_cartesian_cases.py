import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from curobo_core.contracts import PlannerAssets
from v3_batched_loaded_search import GpuValidity
from v3_serial_endpoint_demo import TeleportPlanner


ROOT = Path('/mnt/mydisk/ALFA/alfa_robot_v3_curobo/research/curobo_v3/generated/serial_endpoints_20261023')
OUTPUT = ROOT / 'cartesian_four_methods'
WEIGHTS = np.array([np.deg2rad(15) / .1] + [1.] * 14)


def line_samples(case, count=None):
    count = count or case['sample_count']
    progress = np.linspace(0., 1., count)
    samples = {}
    for side, start in case['tcp_start_local'].items():
        matrices = np.repeat(np.asarray(start)[None], count, axis=0)
        matrices[:, :3, 3] += progress[:, None] * np.asarray(case['translation_local'][side])[None]
        samples[side] = matrices
    return samples


def freeze():
    OUTPUT.mkdir(exist_ok=True)
    source_root = ROOT / 'base_margin_2mm_all117'
    manifest = json.loads((source_root / 'manifest.json').read_text())
    planner = TeleportPlanner(PlannerAssets())
    assets = PlannerAssets()
    asset_hashes = {name: dict(path=str(getattr(assets, name)), sha256=hashlib.sha256(Path(getattr(assets, name)).read_bytes()).hexdigest())
                    for name in ('robot_config', 'urdf', 'box_fit', 'named_poses')}
    cases = []
    for index, item in enumerate(manifest):
        document = json.loads((source_root / item['path']).read_text())
        planner.configure_task(item['task'])
        contact = np.asarray(document['serial_endpoint_records'][0]['joints'])
        extraction = next(record for record in document['serial_endpoint_records'] if record['name'] == 'extraction')
        checker = planner.checker(item['task'])
        tensor = torch.tensor(contact, device='cuda', dtype=torch.float32)
        poses = planner._tool_poses(checker, tensor)
        planner.fk(contact)
        swivels = {side: planner.analytic.swivel(arm, contact[1 + arm * 7:8 + arm * 7]) for arm, side in enumerate(('left', 'right'))}
        base = np.array([planner.base_advance, 0., 0.])
        objects = [dict(name=box.name, dimensions=list(box.dims), position=list(np.asarray(box.pose[:3]) + base),
                        quaternion_wxyz=list(box.pose[3:7])) for box in checker.guidance_scene.cuboid or []]
        internal = [dict(name=box.name, dimensions=list(box.dims), position=list(np.asarray(box.pose[:3]) + base),
                        quaternion_wxyz=list(box.pose[3:7])) for box in checker.base_cuboid_layer.scene.cuboid]
        for stage, distance in (('precontact_reverse', .03), ('extraction', .35)):
            translation = {}
            for side in item['task']:
                translation[side] = (-np.asarray(poses[side])[:3, 2] * distance if stage == 'precontact_reverse'
                                     else np.array([-distance, 0., 0.])).tolist()
            world_start, world_end = {}, {}
            for side in item['task']:
                start = np.asarray(poses[side]).copy()
                start[:3, 3] += base
                end = start.copy()
                end[:3, 3] += translation[side]
                world_start[side] = start.tolist()
                world_end[side] = end.tolist()
            began = time.perf_counter()
            stage_checker = planner.checker(item['task'], payload=stage == 'extraction')
            valid = GpuValidity(stage_checker, check_ground=True).mask(tensor[None])
            torch.cuda.synchronize()
            case = dict(case_id=f'{index:03d}_{stage}', task=item['task'], label=item['label'], stage=stage,
                contact_joints=contact.tolist(), fixed_updown_m=float(contact[0]), fixed_swivels_rad=swivels,
                endpoint_joint_hint=extraction['joints'] if stage == 'extraction' else None,
                tcp_start_local={side: poses[side] for side in item['task']},
                tcp_start_world=world_start, tcp_end_world=world_end, translation_local=translation,
                base_world_xyz=base.tolist(), payload=stage == 'extraction', top_grasp=planner.top_grasp,
                sample_count=16 if stage == 'precontact_reverse' else 128,
                scene_objects_world=objects + internal, source_file=str(source_root / item['path']),
                source_sha256=hashlib.sha256((source_root / item['path']).read_bytes()).hexdigest(),
                original_model_id=document['model_id'], start_collision_valid=bool(valid[0]),
                initial_checker_query_ms=(time.perf_counter() - began) * 1000,
                attachment_tool_to_box={side: np.asarray(stage_checker.task_tool_to_box[side]).tolist() for side in item['task']} if stage == 'extraction' else {},
                collision_contract='Original robot/self/ground/internal base cuboids; target boxes excluded for 3cm final approach; attached boxes active throughout 35cm extraction. No full-cycle execution.')
            cases.append(case)
        print('FROZEN', index + 1, len(manifest), item['label'], cases[-1]['start_collision_valid'], flush=True)
    document = dict(total_tasks=len(manifest), total_cartesian_cases=len(cases), assets=asset_hashes, cases=cases,
        geometry_contract='Fixed current contact joint configuration; identical TCP start/end targets and fixed orientation for every method; end joint configuration is free, old extraction end is only a seed hint.')
    (OUTPUT / 'cases.json').write_text(json.dumps(document, ensure_ascii=False))
    print('FROZEN_ALL', len(cases), 'invalid_payload_starts', sum(not case['start_collision_valid'] for case in cases if case['payload']), flush=True)


if __name__ == '__main__':
    freeze()
