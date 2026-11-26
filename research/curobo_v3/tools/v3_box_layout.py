"""Viser two-box layout editor; all CUDA work runs in a foreground gpuq worker."""
import argparse
from collections import OrderedDict
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
import time
import traceback

import numpy as np
from curobo_core.box_pick import BOX_DIMENSIONS, BoxPick, layout_model_key
from curobo_core.layout_contract import validate_pair, carry_configuration, base_front_x, DEFAULT_FRONT_DISTANCE_M, DEFAULT_SIDE_MIN_HEIGHT_M

ROOT = Path(__file__).resolve().parents[1]


def parse_layout(document):
    if document.get('frame_id') != 'world':
        raise ValueError('布局坐标系必须是 world；机器人基座固定在世界原点')
    base = np.asarray(document.get('world_from_robot_base', np.eye(4)), dtype=float)
    if base.shape != (4, 4) or not np.array_equal(base, np.eye(4)):
        raise ValueError('本入口不移动底盘：world_from_robot_base 必须为单位矩阵')
    boxes = document.get('boxes', [])
    if len(boxes) != 2 or {row.get('side') for row in boxes} != {'left', 'right'}:
        raise ValueError('必须配置左、右臂各一个箱体，不允许缺箱或重复手臂')
    for row in boxes:
        if row.get('carry_joint_home', True) is not True:
            raise ValueError('双箱任务必须使用指定关节回中姿态')
        if row.get('carry_updown_m') is not None:
            raise ValueError('回中updown由规划器逐任务选择，输入不能固定该值')
    picks = {row['side']: BoxPick(**dict(row, carry_joint_home=True, side_min_height_m=row.get('side_min_height_m', DEFAULT_SIDE_MIN_HEIGHT_M))) for row in boxes}
    contract = validate_pair(picks)
    if 'front_distance_m' in document:
        requested_distance = float(document['front_distance_m'])
        if not np.isfinite(requested_distance) or abs(requested_distance-contract['front_distance_m']) > 1e-9:
            raise ValueError('front_distance_m must be finite and match the actual box-center plane')
    a, b = picks['left'], picks['right']
    separation = np.abs(np.asarray(a.center_m)-b.center_m)
    if np.all(separation < (np.asarray(a.dimensions_m)+b.dimensions_m)/2-1e-9):
        raise ValueError('两个完整箱体发生重叠；拒绝解算，不忽略箱间碰撞')
    return picks


def apply_carry_preview(ghosts, event):
    """Apply the selected full carton transforms, including automatic face/roll changes."""
    from scipy.spatial.transform import Rotation
    updates = {}
    for side, xyz in event['targets'].items():
        position = np.asarray(xyz, dtype=float)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('carry preview position must be finite XYZ')
        quaternion = None
        if side in event.get('box_matrices', {}):
            matrix = np.asarray(event['box_matrices'][side], dtype=float)
            if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                    or not np.allclose(matrix[3], [0., 0., 0., 1.], atol=1e-8, rtol=0.)
                    or not np.allclose(matrix[:3, 3], position, atol=1e-8, rtol=0.)
                    or not np.allclose(matrix[:3, :3].T@matrix[:3, :3], np.eye(3), atol=1e-6, rtol=0.)
                    or abs(np.linalg.det(matrix[:3, :3])-1.) > 1e-6):
                raise ValueError('carry preview must contain consistent rigid carton transforms')
            quaternion = np.roll(Rotation.from_matrix(matrix[:3, :3]).as_quat(), 1)
        updates[side] = position, quaternion
    for side, (position, quaternion) in updates.items():
        if side in ghosts:
            ghosts[side].position = position
            if quaternion is not None:ghosts[side].wxyz = quaternion


def scaled_estimate(value, contacts):
    """Same s/m/h units as gpuq; conservative growth from measured 16-contact requests."""
    text = str(value).strip().lower()
    unit = text[-1:] if text[-1:] in ('s', 'm', 'h') else 'm'
    amount = float(text[:-1] if text[-1:] in ('s', 'm', 'h') else text)
    if not np.isfinite(amount) or amount <= 0 or contacts < 1:
        raise ValueError('GPU estimate must be positive finite s/m/h; contact budget must be positive')
    return f'{amount*max(1., contacts/16):g}{unit}'


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')
    temporary.replace(path)


def layout_signature(document):
    picks = parse_layout(document)
    return json.dumps(dict(frame_id='world', boxes=[asdict(picks[s]) for s in ('left', 'right')]),
                      sort_keys=True)


def verify_layout_result(record):
    if record.get('workflow')=='handover_same_side':
        from curobo_core.handover import verify_handover_result
        return verify_handover_result(record)
    if record.get('workflow')=='ordered_upper_first':
        from curobo_core.ordered_pick import verify_ordered_result
        return verify_ordered_result(record)
    """Independent CPU URDF endpoints/array limits; native full-path collision/RNEA stay separate."""
    import hashlib
    from itertools import product
    from scipy.spatial.transform import Rotation
    import yourdfpy
    import yaml
    from curobo_core.box_pick import actual_owner_ids, payload_fit, timed_path_checks
    from experiments.two_box_pick_20261119.replay_native import native_samples
    times, rows, phases = native_samples(record)
    picks = {side: BoxPick(**task) for side, task in record['task'].items()}
    owners = actual_owner_ids(record)
    originals = {('wall_box_00' if b['side'] == 'left' else 'wall_box_01'): b
                 for b in record.get('request', {}).get('boxes', [])}
    if owners != {'left': 'wall_box_00', 'right': 'wall_box_01'} and set(originals) != set(owners.values()):
        raise ValueError('reassigned actual arms require both original physical cartons')
    for side, pick in picks.items():
        if pick.side != side:
            raise ValueError('task key must match its actual arm')
        if originals:
            source = originals[owners[side]]
            if (source['size'] != pick.size or not np.array_equal(source['center_m'], pick.center_m)
                    or source.get('swap_length_height', False) != pick.swap_length_height
                    or source.get('side_min_height_m', DEFAULT_SIDE_MIN_HEIGHT_M) != pick.side_min_height_m):
                raise ValueError('actual arm assignment changed the original physical carton')
            for key in ('side_face', 'suction_offset_m', 'suction_roll_deg'):
                if key in source and not np.array_equal(source[key], getattr(pick, key)):
                    raise ValueError('actual arm assignment changed explicit suction choice')
    robot = yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
    config = yaml.safe_load((ROOT/'generated/fast_runtime_assets/fixed.yaml').read_text())['kinematics']
    source_fit = json.loads((ROOT/'generated/v3_suction_v322_6bb184b/frozen_collision_model/box_voxel_edge_corner_final.json').read_text())
    policy = json.loads((ROOT/'config/fast_three_layers.json').read_text())
    raw, labels = np.asarray(record['frames']), np.asarray(record['phases'])
    endpoints, checks = {}, []
    for stage in record['timing']['stages']:
        with np.load(stage['trajectory_file'], allow_pickle=False) as f:
            arrays = {key: f[key].copy() for key in f.files}
        expected = raw[labels == stage['phase']]
        check = timed_path_checks(arrays, expected[0], expected[-1], policy)
        if not check['valid']:raise ValueError(f"independent native array limits/rest/endpoints failed: {stage['phase']}")
        checks.append(dict(phase=stage['phase'], **check))
        ends = []
        for q in arrays['position'][[0, -1]]:
            robot.update_cfg(dict(zip(record['joint_names'], q)))
            ends.append({side: robot.get_transform(side+'_tool0', robot.base_link)@pick.tool_to_box()
                         for side, pick in picks.items()})
        endpoints[stage['phase']] = ends
    fk_checks = {}
    for side, pick in picks.items():
        first, last = [frame[side] for frame in endpoints['extract']]
        actual = endpoints['transport'][-1][side]
        target = pick.carry_box()
        source_error = float(np.linalg.norm(first[:3, 3]-pick.center_m))
        pull = float(np.linalg.norm(last[:3, 3]-first[:3, 3]-[-pick.extraction_distance_m, 0., 0.]))
        position = float(np.linalg.norm(actual[:3, 3]-target[:3, 3]))
        angle = float(np.rad2deg(Rotation.from_matrix(target[:3, :3].T@actual[:3, :3]).magnitude()))
        if source_error > .002 or pull > .002 or position > .002 or angle > 1.:
            raise ValueError(f'independent {side} extraction/carry FK failed: {pull}, {position}, {angle}')
        fk_checks[side] = dict(physical_id=owners[side], source_position_error_m=source_error, extraction_error_m=pull, carry_position_error_m=position, carry_angle_error_deg=angle)
    robot.update_cfg(dict(zip(record['joint_names'], rows[-1])))
    footprint = robot.get_transform('base_footprint', robot.base_link)[:2, 3]
    radii, corners = [], {}
    for link, spheres in config['collision_spheres'].items():
        if link == 'base_link':continue  # PR82 four-cuboid base replaces its disabled 71 sphere slots.
        transform = robot.get_transform(link, robot.base_link)
        for sphere in spheres:
            if sphere['radius'] <= 0:continue
            center = transform[:3, :3]@sphere['center']+transform[:3, 3]
            radii.append(float(np.linalg.norm(center[:2]-footprint)+sphere['radius']))
    for side, pick in picks.items():
        actual = endpoints['transport'][-1][side]
        fit = payload_fit(source_fit, pick.dimensions_m)
        centers = np.asarray(fit['centers'])@actual[:3, :3].T+actual[:3, 3]
        radii.extend((np.linalg.norm(centers[:, :2]-footprint, axis=1)+fit['radii']).tolist())
        vertices = np.asarray(list(product((-1., 1.), repeat=3)))*np.asarray(pick.dimensions_m)/2
        vertices = vertices@actual[:3, :3].T+actual[:3, 3]
        corners[side] = float(np.linalg.norm(vertices[:, :2]-footprint, axis=1).max())
    radius = max(radii)
    inherited = record['timing']['stages'][-1]['carry_envelope_final']
    joint_carry = all(p.carry_joint_home for p in picks.values())
    if not joint_carry and inherited.get('enforced', True) is False:
        raise ValueError('legacy carry envelope cannot be disabled for a Cartesian carry request')
    if ((not joint_carry and radius > inherited['limit_m']+1e-5)
            or abs(radius-inherited['radius_m']) > 1e-4):
        raise ValueError('independent full-robot held sphere envelope failed')
    carry_joints = None
    if joint_carry:
        validate_pair(picks)
        expected = carry_configuration(float(rows[-1, 0]))
        error = max(abs(float(rows[-1, i])-expected[name])
                    for i, name in enumerate(record['joint_names']) if name != 'updown')
        if error > 1e-5:
            raise ValueError(f'independent carry joint endpoint failed: {error}rad')
        carry_joints = dict(valid=True, max_error_rad=error, updown_m=float(rows[-1, 0]))
    return dict(valid=True, carry_joint_endpoint=carry_joints, samples=len(times), duration_s=float(times[-1]), arrays=checks, endpoints=fk_checks,
        final_legacy_sphere_radius_m=radius, box_corner_radii_diagnostic_m=corners,
        urdf_sha256=hashlib.sha256((ROOT/'generated/fast_runtime_assets/fixed.urdf').read_bytes()).hexdigest(),
        scope='independent CPU URDF endpoints/array limits/held sphere envelope; collision and RNEA are native sampled gates, not independently repeated')


def solve_file(args, planners=None, session=None):
    # Also cover requests from an already-running editor with the older environment.
    os.environ['TORCHINDUCTOR_COMPILE_THREADS'] = '8'
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    events = []
    began = time.perf_counter()
    def progress(event):
        event = dict(event, elapsed_s=time.perf_counter()-began)
        events.append(event)
        atomic_json(output/'status.json', dict(state='running', elapsed_s=event['elapsed_s'], events=events))
        print(json.dumps({k: v for k, v in event.items() if k != 'details'}, ensure_ascii=False), flush=True)
    report = dict(success=False, real_robot_execution=False)
    try:
        document = json.loads(args.solve_file.read_text())
        picks = parse_layout(document)
        progress(dict(stage='initialize_planner', state='running', kind='initialization'))
        started = time.perf_counter()
        from curobo_core.native_spline import install_native_spline
        spline_metadata = install_native_spline()
        from v3_box_layout_planner import BoxLayoutPlanner
        planners = OrderedDict() if planners is None else planners
        key = layout_model_key(picks)
        reused = key in planners
        if not reused:
            # ponytail: two complete topology planners; bound VRAM, not a graph-pool backend.
            if len(planners) >= 2:
                planners.popitem(last=False)
                import gc
                gc.collect()
            planners[key] = BoxLayoutPlanner()
        planner = planners[key]
        planners.move_to_end(key)
        progress(dict(stage='initialize_planner', state='done', kind='initialization',
            wall_seconds=time.perf_counter()-started, reused=reused))
        report = planner.plan_layout(picks, output, args.max_contacts, progress,
            adapt_carry=all('carry_offset_m' not in row for row in document['boxes']),
            adapt_grasp=all('suction_offset_m' not in row and 'suction_roll_deg' not in row and 'side_face' not in row for row in document['boxes']))
        report['request'] = document
        report['native_spline'] = spline_metadata
        report['request_signature'] = layout_signature(document)
        report['planner_reused'] = reused
        if report['success']:
            # Replay is allowed only when every declared real native stage and actual seam is intact.
            started = time.perf_counter()
            progress(dict(stage='independent_verification', state='running', kind='verification'))
            report['replay_checks'] = verify_layout_result(report)
            progress(dict(stage='independent_verification', state='done', kind='verification',
                          wall_seconds=time.perf_counter()-started))
            atomic_json(output/'independent_verification.json', report['replay_checks'])
    except Exception as error:
        traceback.print_exc()
        report.update(success=False, physical_success=False, failure=f'{type(error).__name__}: {error}', traceback=traceback.format_exc())
    if session is not None:
        report['gpu_session'] = session
        if session_source_stamp() != session['source_stamp']:
            report.update(success=False, physical_success=False, failure='source/config changed during GPU session; start a new session', session_abort=True)
    report['physical_success'] = bool(report.get('physical_success', report.get('timing', {}).get('success', False)))
    report['task_success'] = bool(report['success']) if report.get('process_validation', {}).get('policy') else None
    report['worker_wall_seconds'] = time.perf_counter()-began
    report['events'] = events
    atomic_json(output/'result.json', report)
    atomic_json(output/'status.json', dict(state='accepted' if report['success'] else 'rejected',
        elapsed_s=time.perf_counter()-began, reason=report.get('failure'), events=events))
    return 0 if report['success'] else 1



def session_source_stamp():
    """Expire retained Python/config resources after edits; never claim new-file hashes for old code."""
    import hashlib
    paths = sorted([*Path(__file__).parent.rglob('*.py'), *ROOT.joinpath('config').rglob('*.json')])
    for directory in ('fast_runtime_assets', 'zero_model_candidate_20261018', 'pr71_current_dynamics'):
        paths += sorted(p for p in (ROOT/'generated'/directory).glob('*') if p.suffix in ('.urdf', '.yaml'))
    paths += [ROOT/'generated/v3_suction_v322_6bb184b/frozen_collision_model/box_voxel_edge_corner_final.json']
    paths = [p for p in paths if p.is_file()]
    return hashlib.sha256(b''.join(str(p.relative_to(ROOT)).encode()+p.read_bytes() for p in paths)).hexdigest()


def serve_session(args):
    """One foreground gpuq owns the whole bounded session, including idle waits."""
    session_dir = args.gpu_session.resolve()
    session_dir.mkdir(parents=True, exist_ok=True)
    if (session_dir/'status.json').exists():raise ValueError('GPU session directory was already started; use a fresh directory')
    began = idle_since = time.monotonic()
    deadline = began+args.session_seconds
    stamp = session_source_stamp()
    planners = OrderedDict()
    done = set()
    def state(value, **extra):
        atomic_json(session_dir/'status.json', dict(state=value, pid=os.getpid(), requests=len(done),
            deadline_unix=time.time()+max(0., deadline-time.monotonic()),
            idle_deadline_unix=time.time()+max(0., args.session_idle_seconds-(time.monotonic()-idle_since)), source_stamp=stamp, **extra))
    state('ready')
    reason = 'session_limit'
    try:
        while time.monotonic() < deadline and len(done) < args.session_max_requests:
            requests = [p for p in sorted(session_dir.glob('request_*.json')) if p.name not in done]
            if not requests:
                if time.monotonic()-idle_since >= args.session_idle_seconds:
                    reason = 'idle_timeout'; break
                time.sleep(.05); continue
            if session_source_stamp() != stamp:
                reason = 'source_changed'; break
            path = requests[0]
            request = json.loads(path.read_text())
            state('busy', request=path.name)
            job = argparse.Namespace(solve_file=Path(request['input']), output=Path(request['output']),
                                     max_contacts=int(request['max_contacts']))
            if job.max_contacts < 1:raise ValueError('max_contacts must be positive')
            solve_file(job, planners, dict(id=session_dir.name, request=path.name, source_stamp=stamp))
            done.add(path.name)
            result = json.loads((job.output/'result.json').read_text())
            if result.get('traceback') or result.get('session_abort'):
                reason = 'worker_exception_or_source_change'; break
            idle_since = time.monotonic()
            state('idle')
    except KeyboardInterrupt:
        reason = 'user_stop'
    except Exception:
        reason = 'worker_exception'
        raise
    finally:
        state('stopped', reason=reason)
    return 0

def status_markdown(status):
    names = dict(ordered_pick_recovery='遮挡箱位顺序取箱恢复',suction_grasp_recovery='箱位不变的吸附点恢复', recover_pose_search='容差内末态种子搜索', recover_endpoint_ik='精细共享升降末态IK', reverse_extract='反向抽离恢复', contact_reach_bounds='接触可达性证明', extraction_endpoint='抽离末态几何证明', independent_verification='独立CPU核验', initialize_planner='规划器初始化/热复用', initialize_checkers='空载碰撞资源',
        initialize_loaded_checker='双箱载荷碰撞资源', carry_target_selected='指定关节回中目标已选定', carry_joint_goals='指定关节姿态/自由升降候选', contact_ik='双臂接触IK',
        initialize_extraction_lm='抽离LM初始化', precontact='预接触直线', extract='抽离', extract_reference='抽离参考节点提案（计入抽离总耗时）', carry_ik='双臂持箱IK',
        lateral_left='左箱正向横移候选', lateral_right='右箱正向横移候选', approach_piece='接近原生子段', approach_refine='接近保形细分', transport_piece='搬运原生子段', transport_refine='搬运保形细分', transport_process='抽离后不回插检查', transport='搬运回中', approach='Home→预接触', initialize_native='原生优化初始化', contact='贴合吸附', candidate='候选')
    rows = ['|候选|子阶段|解算墙钟(s)|运动时长(s)|状态|', '|---:|---|---:|---:|---|']
    # ponytail: show the whole bounded request history; paginate if custom budgets make the panel too large.
    for event in status.get('events', []):
        if 'wall_seconds' not in event:continue
        stage = names.get(event['stage'], event['stage'])
        if event['stage']=='transport' and event.get('kind')=='geometry':stage+='（含横移/搜索，子项不重复累加）'
        if event['stage'] in ('approach','transport') and event.get('details',{}).get('representation')=='piecewise_native_bspline32_with_rest_seams':stage+='（含全部原生子段及拒绝）'
        if event.get('piece') is not None:stage+=' / '+str(event['piece'])
        prefix = {'native':'原生', 'native_substage':'原生子段', 'process':'工艺'}.get(event.get('kind'), '几何/初始化')
        motion = event.get('details', {}).get('time_optimization', {}).get('duration_s')
        motion_text = f'{motion:.3f}' if motion is not None else '—'
        candidate = event.get('attempt')
        rows.append(f"|{candidate if candidate is not None else '—'}|{prefix} · {stage}|{event['wall_seconds']:.3f}|"
                    f"{motion_text}|{event['state']}|")
    failures = [e for e in status.get('events', []) if e.get('state') == 'rejected']
    if failures:
        rows += ['\n**候选/门禁拒绝记录（包含提案筛选，搜索预算耗尽不代表箱位非法）**']
        for event in failures:
            reasons = event.get('reason') or '; '.join(event.get('details', {}).get('rejection_reasons', []))
            rows.append(f"- 候选{event.get('attempt', '—')} / {event['stage']}: {reasons}")
            for witness in event.get('witnesses', []):
                side = {'left':'左臂','right':'右臂'}.get(witness.get('side'), '')
                if witness['kind'] == 'assigned_arm_joint2_reach_bound':
                    required = witness['wrist_to_elbow_distance_lower_m']*1000
                    available = (witness['forearm_length_m']+witness['pose_envelope_m'])*1000
                    rows.append(f"  - {side}：腕点到可行肘部距离至少{required:.2f}mm，前臂连同位姿容差最多{available:.2f}mm；不改箱位、不改手臂分配时无法接触。")
                elif witness['kind'] == 'fixed_body_blocks_required_extraction_endpoint':
                    rows.append(f"  - {side}：抽离末端与{witness['obstacle']}重叠{witness['overlap_m']*1000:.2f}mm，已计入位姿误差包络。")
                else:rows.append('  - '+json.dumps(witness, ensure_ascii=False))

    retained = [e for e in status.get('events', []) if (e.get('details', {}).get('trajectory_optimization', {}).get('seed_retention') or {}).get('success')]
    if retained:
        rows += ['\n**原优化输出拒绝后，以下输入种子由同一原生API重新核验；并非所有优化候选都成功。**']
        for event in retained:
            detail=event['details']['trajectory_optimization']['seed_retention']
            rows.append(f"- 子段{event.get('piece','—')}：原生种子校验通过；原优化校验记录：{detail.get('optimizer_validation','完整候选外层物理/工艺拒绝后重验')}。")
    observations = [e.get('details', {}).get('source_slot_observation') for e in status.get('events', [])]
    observations = [value for value in observations if value]
    if observations:
        process_events = [e for e in status.get('events', []) if e.get('stage') == 'transport_process']
        if process_events:
            value = process_events[-1]['details']
            rows += [f"\n**工艺检查：不加深原位重叠、完全离开后不回插 · {value['status']}（不是整箱过前沿面要求）。**"]
        else:
            rows += ['\n**工艺诊断：以下仅观测，尚无最终工艺检查结果；不是新增拒绝判据。**']
        for side, value in observations[-1].items():
            name = {'left':'左箱','right':'右箱'}.get(side, side)
            rows.append(f"- {name}：抽离末态X前沿差{value['extraction_end_front_plane_excess_m']*1000:+.3f}mm；"
                f"运输最大前探{value['max_center_forward_m']*1000:.3f}mm；原位重叠（初始/最大）"
                f"{value['initial_slot_overlap_m']*1000:.3f}/{value['maximum_slot_overlap_m']*1000:.3f}mm；"
                f"进一步深入{value['deeper_than_initial_m']*1000:.3f}mm；"
                f"几何离开后再进入：{'是' if value['returns_after_clear'] else '否'}。")
    return '\n'.join(rows)



class LayoutGpuWorker:
    """CPU-side owner of one queued process; no CUDA is initialized in the editor."""
    def __init__(self, args):
        self.args = args
        self.process = self.log = self.directory = None
        self.command = []
        self.stopping = False
        self.estimate = args.estimate

    def alive(self):
        return self.process is not None and self.process.poll() is None

    def status(self):
        path = self.directory/'status.json' if self.directory else None
        return json.loads(path.read_text()) if path and path.is_file() else {}

    def stop(self):
        import signal
        if self.alive() and not self.stopping:
            self.stopping = True
            try:os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:pass

    def close(self):
        import signal
        import subprocess
        self.stop()
        if self.process is not None:
            try:self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait()
        if self.log is not None:self.log.close(); self.log = None

    def submit(self, run, budget):
        import math
        import shlex
        import subprocess
        import tempfile
        state = self.status()
        reusable = (self.alive() and not self.stopping and self.directory is not None and
            state.get('state') in ('ready', 'idle', 'busy') and
            state.get('deadline_unix', 0) > time.time()+2 and
            (state.get('state') == 'busy' or state.get('idle_deadline_unix', 0) > time.time()+2) and
            state.get('requests', 0)+(state.get('state') == 'busy') < self.args.session_max_requests and
            state.get('source_stamp') == session_source_stamp())
        if not reusable:
            self.close()
            self.directory = None
            self.estimate = scaled_estimate(self.args.estimate, budget)
            command = ['gpuq', '--name', 'Viser双箱GPU交互会话（空闲持锁/有限时长，估时暂定）', '--estimate', self.estimate,
                       '--', sys.executable, '-u', str(Path(__file__).resolve())]
            if self.args.session_idle_seconds > 0:
                self.directory = Path(tempfile.mkdtemp(prefix='session_', dir=self.args.runs_root.resolve()))
                seconds = float(self.estimate[:-1])*dict(s=1, m=60, h=3600)[self.estimate[-1]]
                self.estimate = f'{math.ceil(self.args.session_seconds+seconds)}s'
                command[4] = self.estimate
                command += ['--gpu-session', str(self.directory), '--session-idle-seconds', str(self.args.session_idle_seconds),
                    '--session-seconds', str(self.args.session_seconds), '--session-max-requests', str(self.args.session_max_requests)]
                logfile = self.directory/'worker.log'
            else:
                command += ['--solve-file', str(run/'input.json'), '--output', str(run/'plan'), '--max-contacts', str(budget)]
                logfile = run/'worker.log'
            self.command = command
            self.log = logfile.open('w')
            if self.directory is not None:
                atomic_json(self.directory/'command.json', dict(command=command))
            environment = dict(os.environ, MAKEFLAGS='-j8 -l12', CMAKE_BUILD_PARALLEL_LEVEL='8', MAX_JOBS='8',
                               TORCHINDUCTOR_COMPILE_THREADS='8', OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='2')
            self.stopping = False
            self.process = subprocess.Popen(command, cwd=ROOT.parents[1], env=environment,
                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        if self.directory is not None:
            request = dict(input=str(run/'input.json'), output=str(run/'plan'), max_contacts=budget)
            atomic_json(run/'request.json', request)
            atomic_json(self.directory/f'request_{time.time_ns()}_{run.name}.json', request)
            (run/'worker.log').symlink_to(self.directory/'worker.log')
        (run/'command.txt').write_text(shlex.join(self.command)+'\n')
        return reusable

def serve(args):
    import os
    import queue
    import shlex
    import tempfile
    import viser
    from viser.extras import ViserUrdf
    import yourdfpy
    import yaml
    from scipy.spatial.transform import Rotation
    from experiments.two_box_pick_20261119.replay_native import native_samples

    def initial_document():
        return dict(frame_id='world', front_distance_m=args.front_distance, boxes=[dict(size=getattr(args, s+'_size'), side=s,
            center_m=getattr(args, s+'_center'), swap_length_height=getattr(args, s+'_swap_length_height'),
            side_min_height_m=args.side_min_height) for s in ('left', 'right')])
    parse_layout(initial_document())
    urdf = ROOT/'generated/fast_runtime_assets/fixed.urdf'
    if not urdf.is_file():raise FileNotFoundError('Prepare the existing PR82 runtime assets before starting the editor')
    robot = yourdfpy.URDF.load(urdf, load_meshes=False)
    home = yaml.safe_load((ROOT/'generated/zero_model_candidate_20261018/named_poses.yaml').read_text())['named_poses']['home']
    server = viser.ViserServer(host=args.host, port=args.port, label='PR82 · 双箱世界坐标布局与实时联合解算')
    visual = ViserUrdf(server, urdf, root_node_name='/robot')
    server.scene.add_frame('/world', axes_length=.3, axes_radius=.006)
    for obj in json.loads((ROOT/'config/two_box_layout/environment.json').read_text())['objects']:
        server.scene.add_box('/environment/'+obj['object_id'], dimensions=obj['dimensions_m'], color=(110, 130, 150),
            position=obj['pose']['position'], wxyz=obj['pose']['quaternion_wxyz'], opacity=.12)
    # Show the old cylinder only as a reference; the specified joints define the endpoint.
    angle = np.linspace(0, 2*np.pi, 96, endpoint=False)
    circle = np.c_[1.05*np.cos(angle), 1.05*np.sin(angle), np.full(len(angle), 1.2)]
    server.scene.add_line_segments('/carry_envelope', points=np.stack((circle, np.roll(circle, -1, axis=0)), axis=1),
                                   colors=(90, 180, 210), thickness=2., thickness_units='screen')
    server.gui.add_markdown('世界系 **world=机器人基座系**，底盘固定原点。**X=箱宽/抽离深度**，抽离沿−X；'
        '长高切换仅交换Y/Z，X不变。左右箱是真正共享升降轴的15轴联合规划，完整箱间碰撞、'
        '每箱10kg。回中严格使用用户指定的双臂关节角，升降轴逐任务选择；旧1.05m圆柱仅为参考。线框预览使用updown=0，成功后显示实际升降终点。参数修改即时预览；点击解算后异步GPU排队，'
        '不宣称毫秒级规划，不连接实机。')
    events = queue.SimpleQueue()
    distance_control = server.gui.add_number('固定箱心平面距底盘前沿 (cm)', initial_value=round(args.front_distance*100), disabled=True)
    controls, gizmos, boxes, ghosts = {}, {}, {}, {}
    for side, label in [('left', '左箱 / 左臂'), ('right', '右箱 / 右臂')]:
        with server.gui.add_folder(label):
            controls[side] = dict(size=server.gui.add_dropdown('尺寸标签', options=list(BOX_DIMENSIONS),
                initial_value=getattr(args, side+'_size')),
                swap=server.gui.add_checkbox('长高切换（Y/Z对调，X宽度不变）', getattr(args, side+'_swap_length_height')),
                xyz=[server.gui.add_number('世界 '+axis+' (m)', initial_value=float(value), step=.005, disabled=axis=='X')
                     for axis, value in zip('XYZ', getattr(args, side+'_center'))],
                info=server.gui.add_markdown(''))
        gizmo = server.scene.add_transform_controls('/layout/'+side+'/center', position=getattr(args, side+'_center'),
            scale=.17, active_axes=(False, True, True), disable_rotations=True)
        async def drag(event, side=side, gizmo=gizmo):
            for handle, value in zip(controls[side]['xyz'][1:], gizmo.position[1:]):handle.value = float(value)
        gizmo.on_update(drag)
        gizmos[side] = gizmo
    threshold = server.gui.add_number('侧吸→顶吸高度阈值 (m，待标定)', initial_value=args.side_min_height, min=0., step=.01)
    max_contacts = server.gui.add_number('每种吸附型最大接触候选数', initial_value=args.max_contacts, min=1, step=1)
    compute = server.gui.add_button('按当前双箱布局解算（重新计算）')
    cancel = server.gui.add_button('取消本次排队/解算', disabled=True)
    release = server.gui.add_button('结束GPU会话并释放锁', disabled=True)
    session_info = server.gui.add_markdown('GPU会话未启动；首次解算冷启动，后续同拓扑复用。空闲期间也持有全局GPU锁。')
    async def submit(event):
        # Viser awaits async callbacks in its message handler: freeze before a later edit.
        events.put(('compute', document(), int(max_contacts.value)))
    async def cancel_request(event):
        events.put(('cancel',))
    compute.on_click(submit)
    cancel.on_click(cancel_request)
    release.on_click(cancel_request)
    play = server.gui.add_checkbox('播放已通过的实际原生轨迹', False, disabled=True)
    phase = server.gui.add_dropdown('回放子阶段', options=['全流程', 'approach', 'contact', 'extract', 'transport'])
    speed = server.gui.add_slider('播放倍速', min=.1, max=2., step=.1, initial_value=1.)
    timeline = server.gui.add_slider('原生时间 (s)', min=0., max=1., step=.01, initial_value=0., disabled=True)
    show_targets = server.gui.add_checkbox('显示指定关节回中持箱目标（线框）', True)
    status = server.gui.add_markdown('修改XYZ或拖拽箱心坐标轴，然后点击解算。')
    timings = server.gui.add_markdown('各子阶段耗时与拒绝理由将在这里实时更新。')
    current_frame = server.gui.add_markdown('布局预览 / Home')
    command_info = server.gui.add_markdown('')
    worker = LayoutGpuWorker(args)
    active = False
    run = None
    request_signature = result_signature = None
    data = None
    view = None
    last_raw = last_view = last_frame = None
    status_stamp = None
    selected_targets = None
    submitted = 0.
    estimate_for_run = args.estimate
    cancelling = False
    previous_time = time.monotonic()

    def document():
        return dict(frame_id='world', front_distance_m=float(distance_control.value)/100, boxes=[dict(side=s, size=c['size'].value,
            swap_length_height=c['swap'].value, center_m=[float(h.value) for h in c['xyz']],
            side_min_height_m=float(threshold.value)) for s, c in controls.items()])

    def reset_pose():
        visual.update_cfg({name: home.get(name, 0.) for name in robot.actuated_joint_names})
        robot.update_cfg(home)

    def invalidate():
        nonlocal data, view, last_frame
        data = view = None; last_frame = None
        play.value = False; play.disabled = True; timeline.disabled = True
        reset_pose()
        current_frame.content = '布局预览 / Home：旧轨迹已失效'

    def stop_worker():
        nonlocal cancelling
        if worker.alive():
            worker.stop()
            cancelling = True
            status.content = '正在取消本次请求/结束GPU会话；已通过的历史轨迹不被改写。'

    reset_pose()
    try:
        while True:
            pending_compute = None
            cancel_requested = False
            while not events.empty():
                action = events.get()
                if action[0] == 'compute':pending_compute = action[1:]
                elif action[0] == 'cancel':cancel_requested = True
            doc = document()
            raw = json.dumps(doc, sort_keys=True)
            try:
                picks = parse_layout(doc); signature = layout_signature(doc); invalid = None
            except (ValueError, TypeError, KeyError) as error:
                picks = None; signature = None; invalid = str(error)
            if raw != last_raw:
                last_raw = raw; invalidate(); selected_targets = None
                status.content = ('**布局拒绝：** '+invalid) if invalid else '布局已更新；旧轨迹失效。点击解算生成新路径。'
                with server.atomic():
                    for side, c in controls.items():
                        dims = BOX_DIMENSIONS[c['size'].value]
                        if c['swap'].value:dims = (dims[0], dims[2], dims[1])
                        center = tuple(float(h.value) for h in c['xyz'])
                        if not np.isfinite(center).all():continue
                        if side in boxes:boxes[side].remove()
                        if side in ghosts:ghosts.pop(side).remove()
                        boxes[side] = server.scene.add_box('/boxes/'+side, dimensions=dims, position=center,
                            color=((200, 70, 65) if invalid else ((75, 160, 220) if side == 'left' else (215, 165, 80))))
                        gizmos[side].position = center
                        mode = '顶吸' if center[2] < threshold.value else '侧吸'
                        c['info'].content = f'实际XYZ尺寸：**{dims} m**；{mode}；抽离 **{dims[0]:.3f}m**。'
                        if picks:
                            target = picks[side].carry_box()
                            ghosts[side] = server.scene.add_box('/carry_target/'+side, dimensions=dims, wireframe=True,
                                position=target[:3, 3], wxyz=np.roll(Rotation.from_matrix(target[:3, :3]).as_quat(), 1),
                                color=(80, 205, 150), opacity=.5, visible=show_targets.value)
            for node in ghosts.values():node.visible = show_targets.value
            if cancel_requested:stop_worker()
            if pending_compute is not None and not active and not cancel_requested:
                request_doc, budget = pending_compute
                try:
                    submission_signature = layout_signature(request_doc)
                    submission_error = None
                except (ValueError, TypeError, KeyError) as error:
                    submission_error = str(error)
                if submission_error:
                    status.content = '**本次提交布局拒绝：** '+submission_error
                elif budget < 1:
                    status.content = '**参数拒绝：** 最大接触候选数必须为正整数。'
                else:
                    invalidate()
                    phase.value = '全流程'
                    args.runs_root.mkdir(parents=True, exist_ok=True)
                    run = Path(tempfile.mkdtemp(prefix='layout_', dir=args.runs_root.resolve()))
                    atomic_json(run/'input.json', request_doc)
                    request_signature = submission_signature; result_signature = None; status_stamp = None
                    try:
                        reused_session = worker.submit(run, budget)
                        estimate_for_run = worker.estimate
                        submitted = time.monotonic(); cancelling = False; active = True
                        compute.disabled = True; cancel.disabled = False
                        command_info.content = ('GPU会话复用' if reused_session else '新GPU队列作业')+'（含空闲等待的同一全局锁）：\n```bash\n'+shlex.join(worker.command)+'\n```\n结果目录：`'+str(run)+'`'
                        print('GPU_LAYOUT_COMMAND', shlex.join(worker.command), flush=True)
                    except Exception:
                        worker.close()
                        status.content = '**启动失败：** '+traceback.format_exc()
            release.disabled = not worker.alive()
            session_state = worker.status()
            if worker.alive() and session_state:
                remaining = max(0., min(session_state['idle_deadline_unix'], session_state['deadline_unix'])-time.time())
                session_info.content = (f"GPU会话 **{session_state['state']}** · 已完成{session_state['requests']}次 · "
                    f"空闲自动释放倒计时{remaining:.0f}s（忙时不适用）；总接收窗口{args.session_seconds:g}s，"
                    f"最多{args.session_max_requests}次。**空闲也持有全局GPU锁**；可点击结束会话。")
            elif worker.alive():
                session_info.content = 'GPU全局队列等待/进程启动中；尚无GPU会话就绪确认。'
            elif worker.process is not None:
                session_info.content = 'GPU会话已退出，锁已释放；下次解算将重新排队。原因：'+str(session_state.get('reason', 'one-shot/exit'))
                if not active and cancelling:
                    status.content = 'GPU会话已结束；已通过的轨迹仍可播放。'
                    cancelling = False
            if active:
                state_file = run/'plan/status.json'
                state = None
                if state_file.is_file():
                    stamp = state_file.stat().st_mtime_ns
                    if stamp != status_stamp:
                        state = json.loads(state_file.read_text()); status_stamp = stamp
                        timings.content = status_markdown(state)
                        chosen = [e for e in state.get('events', []) if e.get('stage') == 'carry_target_selected']
                        selected = ({key:chosen[-1].get(key) for key in ('targets', 'box_matrices', 'suctions')}
                                    if chosen else None)
                        if chosen and signature == request_signature and selected != selected_targets:
                            selected_targets = selected
                            apply_carry_preview(ghosts, chosen[-1])
                            for side, xyz in chosen[-1]['targets'].items():
                                grip = chosen[-1].get('suctions', {}).get(side, {})
                                base_info = controls[side]['info'].content.split('\n\n回中目标箱心：')[0]
                                controls[side]['info'].content = base_info+f'\n\n回中目标箱心：**{np.asarray(xyz).round(4).tolist()}m**；按指定关节角，updown={chosen[-1].get("updown_m", "—")}m。吸附面：{grip.get("face", "—")}。'
                    else:state = json.loads(state_file.read_text())
                result_file = run/'plan/result.json'
                finished = (state or {}).get('state') in ('accepted', 'rejected')
                if worker.alive() and not finished:
                    if not cancelling:
                        stale = '**输入已改动：本次仍在计算提交快照，结果不会用于新布局。**\n\n' if signature != request_signature else ''
                        if state is None:
                            status.content = stale+f'GPU全局队列等待中：{time.monotonic()-submitted:.1f}s；估时{estimate_for_run}为暂估，不是超时。'
                        else:
                            running = state['events'][-1] if state['events'] else {}
                            elapsed = state['elapsed_s']+max(0., time.time()-state_file.stat().st_mtime)
                            status.content = stale+f"解算进行中 {elapsed:.1f}s · {running.get('kind')} / {running.get('stage')} / 候选{running.get('attempt', '—')}"
                else:
                    compute.disabled = False; cancel.disabled = True
                    if cancelling:
                        status.content = '本次请求已取消；没有可播放的成功结果。'
                    elif not result_file.is_file():
                        tail = (run/'worker.log').read_text()[-2000:]
                        status.content = f'**工作进程退出，代码{worker.process.returncode}：**\n```text\n{tail}\n```'
                    else:
                        record = json.loads(result_file.read_text())
                        if record['success'] and signature == request_signature:
                            times, rows, phases = native_samples(record)
                            robot.update_cfg(dict(zip(record['joint_names'], rows[-1])))
                            from curobo_core.ordered_pick import replay_box_poses
                            matrices = replay_box_poses(robot, record, str(phases[-1]))
                            apply_carry_preview(ghosts, dict(box_matrices=matrices,
                                targets={side:matrix[:3, 3].tolist() for side,matrix in matrices.items()}))
                            data = (times, rows, phases, record)
                            phase.options = ['全流程']+list(dict.fromkeys(phases.tolist()))
                            phase.value = '全流程'
                            result_signature = request_signature; last_view = None
                            play.disabled = False; timeline.disabled = False
                            process_label = '物理及抽离后不回插检查通过' if record.get('task_success') else '仅物理门禁，工艺规则待确认'
                            mode_label={'ordered_upper_first':'先上箱后下箱','handover_same_side':'单箱换手后取另一箱'}.get(record.get('workflow'),'双箱联合')
                            reuse_label={'ordered_upper_first':'顺序分支新规划资源','handover_same_side':'换手分支新规划资源'}.get(record.get('workflow'), '同拓扑热复用' if record.get('planner_reused') else '首次/新拓扑冷启动')
                            status.content = f"**{mode_label}解算通过（{process_label}）** · {reuse_label} · 墙钟{record['worker_wall_seconds']:.3f}s · 实际运动{times[-1]:.3f}s；点击播放。"
                            pieces=record['timing']['stages'][-1].get('pieces',[])
                            if pieces:status.content += f'\n\n**分段原生表示：{len(pieces)}个搬运子段，含中途停顿；不是单条优化曲线，原优化拒绝/种子保留记录见下方。**'
                        elif record['success']:
                            status.content = '旧布局快照已解算通过，但当前参数已改变；禁止播放旧结果，请重新解算。'
                        else:
                            status.content = '**本次解算拒绝：** '+str(record.get('failure'))+'；具体候选/阶段原因见下方。'
                    active = False
            if data is not None and signature == result_signature:
                times, rows, phases, record = data
                if last_view != phase.value:
                    last_view = phase.value
                    ids = np.arange(len(times)) if phase.value == '全流程' else np.flatnonzero(phases == phase.value)
                    if len(ids) and ids[0] > 0:ids = np.r_[ids[0]-1, ids]
                    view = (times[ids]-times[ids[0]], ids)
                    play.value = False; timeline.value = 0.; timeline.max = float(view[0][-1]); last_frame = None
                now = time.monotonic()
                if play.value:
                    timeline.value = min(timeline.max, timeline.value+(now-previous_time)*speed.value)
                    if timeline.value >= timeline.max:play.value = False
                clock, ids = view
                i = min(int(np.searchsorted(clock, timeline.value)), len(clock)-1)
                if i > 0 and timeline.value-clock[i-1] < clock[i]-timeline.value:i -= 1
                index = int(ids[i])
                if index != last_frame:
                    last_frame = index
                    cfg = dict(zip(record['joint_names'], rows[index]))
                    robot.update_cfg(cfg)
                    with server.atomic():
                        visual.update_cfg({name: cfg.get(name, home.get(name, 0.)) for name in robot.actuated_joint_names})
                        from curobo_core.ordered_pick import replay_box_poses
                        actual_boxes=replay_box_poses(robot,record,str(phases[index]))
                        for side, pose in actual_boxes.items():
                            boxes[side].position = pose[:3, 3]
                            boxes[side].wxyz = np.roll(Rotation.from_matrix(pose[:3, :3]).as_quat(), 1)
                    shown_phase = phases[index] if phase.value == '全流程' else phase.value
                    seam = f'（接缝原始标签：{phases[index]}）' if shown_phase != phases[index] else ''
                    current_frame.content = f'**{shown_phase}**{seam} · 实际原生采样{index}/{len(times)-1} · {times[index]:.3f}s；无补帧/伪造成功。'
            previous_time = time.monotonic()
            time.sleep(.05)
    finally:
        worker.close()
        server.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8295)
    parser.add_argument('--left-size', choices=BOX_DIMENSIONS, default='40x40x30')
    parser.add_argument('--right-size', choices=BOX_DIMENSIONS, default='40x40x30')
    parser.add_argument('--front-distance', type=float, default=DEFAULT_FRONT_DISTANCE_M, help='fixed box-center distance from actual base front; measured default0.66m, override only for explicit calibration (0.60–0.90m in0.02m steps)')
    parser.add_argument('--left-center', type=float, nargs=3, default=None)
    parser.add_argument('--right-center', type=float, nargs=3, default=None)
    parser.add_argument('--left-swap-length-height', action='store_true')
    parser.add_argument('--right-swap-length-height', action='store_true')
    parser.add_argument('--side-min-height', type=float, default=DEFAULT_SIDE_MIN_HEIGHT_M)
    parser.add_argument('--max-contacts', type=int, default=16)
    parser.add_argument('--estimate', default='6m', help='provisional gpuq estimate for 16-contact requests (measured dual 75-255s); grows with a larger UI candidate budget, not a timeout')
    parser.add_argument('--runs-root', type=Path, default=ROOT/'generated/two_box_layout')
    parser.add_argument('--gpu-session', type=Path, help='GPU worker session directory; invoke ONLY through gpuq')
    parser.add_argument('--session-idle-seconds', type=float, default=120., help='idle lock hold; 0 disables warm sessions in the editor')
    parser.add_argument('--session-seconds', type=float, default=600., help='stop accepting new jobs after this session wall time; finish active work')
    parser.add_argument('--session-max-requests', type=int, default=8)
    parser.add_argument('--solve-file', type=Path, help='GPU worker mode; invoke ONLY through gpuq')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.max_contacts < 1:parser.error('--max-contacts must be positive')
    try:scaled_estimate(args.estimate, args.max_contacts)
    except ValueError as error:parser.error(str(error))
    if not np.isfinite([args.session_idle_seconds, args.session_seconds]).all() or args.session_idle_seconds < 0 or args.session_seconds <= 0 or args.session_max_requests < 1:
        parser.error('session limits must be finite/positive (idle may be zero)')
    if args.gpu_session:
        if args.solve_file or args.session_idle_seconds <= 0:parser.error('--gpu-session requires positive idle and no --solve-file')
        raise SystemExit(serve_session(args))
    if args.solve_file:
        if args.output is None:parser.error('--solve-file requires --output')
        raise SystemExit(solve_file(args))
    plane = base_front_x()+args.front_distance
    if args.left_center is None:args.left_center = (plane, .45, 1.02)
    if args.right_center is None:args.right_center = (plane, -.45, 1.02)
    serve(args)


if __name__ == '__main__':main()
