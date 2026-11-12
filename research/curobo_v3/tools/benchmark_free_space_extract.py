"""Paired PR82 counterfactual: free XYZ extraction, same contact and carry goal.

Research only. Does not change PR82 defaults or claim hardware/continuous safety.
GPU entry point must be run under the shared gpuq lock.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from types import MethodType

import numpy as np

# IDs are zero-based wall cells (y column=id%5, z row=id//5).
BASELINE_COMMIT = 'ee6dba3280437ea33002ea87a5f8afb8725eadfb'

CASES = {
    'top_outer': dict(task={'left': 24, 'right': 20}, removed=[21, 22, 23]),
    'upper_inner': dict(task={'left': 18, 'right': 16}, removed=[20, 21, 23, 24]),
    'upper_outer': dict(task={'left': 19, 'right': 15}, removed=[20, 21, 23, 24]),
}


def occupied_cells(case):
    task, removed = set(case['task'].values()), set(case['removed'])
    if len(task) != 2 or task & removed or not (task | removed) <= set(range(25)):
        raise ValueError('Expected two distinct targets and valid non-target removed cells')
    return sorted(set(range(25)) - task - removed)


def loaded_rows(source):
    phases = np.asarray(source['phases'])
    ids = np.flatnonzero(np.isin(phases, ['extract', 'transport']))
    if len(ids) == 0 or np.any(np.diff(ids) != 1):
        raise ValueError('Extraction and transport must be contiguous')
    names = source['joint_names']
    from curobo_core.contracts import ACTIVE_JOINTS
    columns = [names.index(name) for name in ACTIVE_JOINTS]
    return np.asarray(source['frames'], dtype=np.float32)[max(0, ids[0] - 1):ids[-1] + 1, columns]


def install_scene(planner, case):
    from curobo_core.scene import SceneObject, Pose
    from v3_task_scene import wall_center
    original = planner.configure_task
    def configure(self, boxes):
        centers, _ = original(boxes)
        if boxes != case['task']:
            raise ValueError('Unexpected task in paired scene')
        objects = [item for item in self.snapshot.objects if not item.object_id.startswith('task_fill_')]
        obstacles = []
        for cell in occupied_cells(case):
            center = wall_center(self.front, .9, cell)
            center[0] -= self.base_advance
            center[2] += .001
            dims = tuple(self.box_fit['dimensions_m'])
            objects.append(SceneObject(f'task_fill_cell_{cell:02d}', dims, Pose(tuple(center))))
            obstacles.append(dict(center=center.tolist(), dimensions=list(dims)))
        self.snapshot = replace(self.snapshot, objects=tuple(objects))
        return centers, obstacles
    planner.configure_task = MethodType(configure, planner)


def install_seed(planner, seed, calls):
    original = planner.verified_rrt
    def search(self, *args, **kwargs):
        args = list(args)
        if len(args) > 6:
            args[6] = int(args[6]) + seed * 100003
            calls.append(args[6])
        elif 'seed' in kwargs:
            kwargs['seed'] = int(kwargs['seed']) + seed * 100003
            calls.append(kwargs['seed'])
        else:
            raise ValueError('Search seed was not supplied explicitly')
        return original(*args, **kwargs)
    planner.verified_rrt = MethodType(search, planner)


def collision_diagnostic(validity, rows):
    import torch
    bad = 0
    # Diagnostic only, same check and same sample count for baseline and variants.
    sample = rows[np.linspace(0, len(rows) - 1, min(513, len(rows))).astype(int)]
    for offset in range(0, len(sample), 128):
        tensor = torch.tensor(np.ascontiguousarray(sample[offset:offset + 128]), device='cuda')
        bad += int((~validity.mask(tensor)).sum())
    return dict(samples=len(sample), invalid=bad, continuous_certificate=False)


def adaptive_xyz(engine, source, rows, output, case, seed, clearance_only=False):
    """Search a bounded XYZ endpoint grid, then use the unchanged loaded LM/RRT."""
    import copy
    import itertools
    import torch
    from v3_fast_path import stage_setup
    from v3_fast_three_layers import stamp, POLICY
    from v3_lm_refinement import optimize_lm_stage
    from v3_task_scene import wall_center
    from v3_fixed_body_boxes import WorldMarginChecker
    from curobo_core.distance_metric import joint_distance_weights
    from v3_fast_path import ACTIVE_JOINTS
    began = stamp()
    planner = engine.planner
    _, _, validity, scene = stage_setup(planner, source, 'transport')
    checker = validity.checker
    start, goal = rows[0].copy(), rows[-1].copy()
    attachments = {side: np.asarray(matrix) for side, matrix in checker.task_tool_to_box.items()}
    tools = {side: np.asarray(matrix) for side, matrix in planner._tool_poses(
        checker, torch.tensor(start, device='cuda')).items()}
    starts = {side: tools[side] @ matrix for side, matrix in attachments.items()}
    signs = {}
    for side, cell in case['task'].items():
        center = wall_center(planner.front, .9, cell)
        nearby = sorted((np.linalg.norm(wall_center(planner.front, .9, hole)-center), hole)
                        for hole in case['removed'] if hole % 5 != cell % 5)
        signs[side] = np.sign((nearby[0][1] % 5) - (cell % 5)) if nearby else 0.
    lower, upper = checker.kinematics.get_joint_limits().position
    lift_lower, lift_upper = float(lower[0]), float(upper[0])
    weights = np.asarray(joint_distance_weights(ACTIVE_JOINTS))
    search_validity = copy.copy(validity)
    search_validity.checker = WorldMarginChecker(checker, planner.radius_scale, .001)
    candidates = []
    # ponytail: bounded XYZ grid, three directional families; continuous endpoints if evidence warrants it.
    for x, lateral, z in itertools.product([0., -.04, -.10, -.18, -.26], [0., .10, .18], [0., .12, .24, .44]):
        displacements = {side: np.array([x, signs[side]*lateral, z]) for side in source['task']}
        if all(np.linalg.norm(value) < 1e-8 for value in displacements.values()):
            continue
        predictions = []
        for lift in sorted(set([float(start[0]), float(np.clip(start[0]+z/2, lift_lower+1e-5, lift_upper-1e-5)),
                                float(np.clip(start[0]+z, lift_lower+1e-5, lift_upper-1e-5))])):
            predicted = start.copy()
            predicted[0] = lift
            planner.fk(predicted)
            carriage = np.linalg.inv(planner.fk_robot.get_transform('arm_carriage', planner.fk_robot.base_link))
            for side_index, side in enumerate(('left', 'right')):
                target = tools[side].copy()
                target[:3, 3] += displacements[side]
                offset = 1 + side_index*7
                current = start[offset:offset+7]
                swivel = planner.analytic.swivel(side_index, current)
                solutions = planner.analytic.solve(side_index, carriage @ target, swivel, current)
                if not len(solutions):
                    break
                predicted[offset:offset+7] = min(solutions, key=lambda value: np.sum((value-current)**2))
            else:
                if bool(search_validity.mask(torch.tensor(predicted[None], device='cuda')).all()):
                    score = np.linalg.norm((predicted-start)*weights) + np.linalg.norm((goal-predicted)*weights)
                    predictions.append(float(score))
        if predictions:
            candidates.append((min(predictions), displacements))
    candidates.sort(key=lambda item: item[0])
    front = [c for c in candidates if all(np.allclose(d, [-.10, 0, 0]) for d in c[1].values())]
    upper = [c for c in candidates if all(d[2] > 0 and d[1] == 0 for d in c[1].values())]
    upper_side = [c for c in candidates if all(d[2] > 0 and abs(d[1]) > 0 for d in c[1].values())]
    # Preserve a short-X control and explore upward and XYZ-diagonal families, not the three shortest X pulls.
    clearance = [c for c in candidates if all(d[2] >= .41 and d[1] == 0 for d in c[1].values())]
    shortlist = ([clearance[0]] if clearance else []) if clearance_only else [family[0] for family in (front, upper, upper_side) if family]
    attempts = []
    engine_lm = planner.placement_solver()
    lower, upper = checker.kinematics.get_joint_limits().position
    for rank, (score, displacements) in enumerate(shortlist):
        trial_began = stamp()
        ends = {side: matrix.copy() for side, matrix in starts.items()}
        for side in ends:
            ends[side][:3, 3] += displacements[side]
        result = engine_lm.plan(dict(start=start, checker=checker, attachments=attachments, starts=starts, ends=ends))
        attempt = dict(rank=rank, heuristic_score=score,
            displacement_m={side: value.tolist() for side, value in displacements.items()},
            lm={key: value for key, value in result.items() if key not in ('frames', 'tracks')}, usable=False)
        attempts.append(attempt)
        if not result['success']:
            attempt['wall_s'] = stamp() - trial_began
            continue
        extract = np.asarray(result['frames'], dtype=np.float32)
        attempt['inflated_search_start_valid'] = bool(search_validity.mask(torch.tensor(extract[-1:], device='cuda')).all())
        if not attempt['inflated_search_start_valid']:
            attempt['wall_s'] = stamp() - trial_began
            attempt['failure'] = 'actual LM endpoint invalid under unchanged RRT world inflation'
            continue
        path, search = planner.verified_rrt(torch.tensor(extract[-1:], device='cuda').contiguous(),
            torch.tensor(goal[None], device='cuda').contiguous(), lower+1e-5, upper-1e-5,
            validity, 2., 20261111+seed+rank, stop_at_first_valid=True)
        attempt['search'] = search
        if path is None:
            attempt['wall_s'] = stamp() - trial_began
            continue
        updated = copy.deepcopy(source)
        updated['selected_lm_review']['lm_detail']['translation_local'] = attempt['displacement_m']
        extract_controls, first, last, dt, details = optimize_lm_stage(
            planner, updated, 'extract', extract, validity, POLICY['lm_refinement'])
        extract_arrays, extract_timing = engine.time_path(extract_controls, first, last, dt,
            tuple(source['task']), source['visual_tool_to_box'], benchmark_recompute=False)
        attempt['extract_refinement'] = details
        attempt['extract_timing'] = extract_timing
        path = np.ascontiguousarray(path, dtype=np.float32)
        controls, first, last, dt, geometry = engine.trajopt_path(source, 'transport', path, validity, scene)
        transport_arrays, transport_timing = engine.time_path(controls, first, last, dt,
            tuple(source['task']), source['visual_tool_to_box'], benchmark_recompute=False)
        attempt['geometry'] = geometry
        attempt['transport_timing'] = transport_timing
        attempt['wall_s'] = stamp() - trial_began
        if extract_arrays is None or transport_arrays is None:
            continue
        attempt['collision'] = dict(extract=collision_diagnostic(validity, extract_arrays['position']),
                                    transport=collision_diagnostic(validity, transport_arrays['position']))
        attempt['endpoint_error'] = float(max(abs(extract_arrays['position'][0]-start).max(),
            abs(extract_arrays['position'][-1]-transport_arrays['position'][0]).max(),
            abs(transport_arrays['position'][-1]-goal).max()))
        attempt['duration_s'] = extract_timing['duration_s'] + transport_timing['duration_s']
        attempt['usable'] = bool(extract_timing['dynamic_valid'] and transport_timing['dynamic_valid']
            and geometry['solver_success'] and attempt['endpoint_error'] < 1e-4
            and all(item['invalid'] == 0 for item in attempt['collision'].values()))
        for phase, arrays in [('extract', extract_arrays), ('transport', transport_arrays)]:
            np.savez_compressed(output / f'adaptive_rank{rank}_{phase}.npz', **arrays)
    accepted = [attempt for attempt in attempts if attempt['usable']]
    record = dict(name='adaptive_xyz', method='59-point XYZ endpoint grid → short-X control + upward + upper-side candidates; unchanged inflated RRT start validation',
        clearance_only=clearance_only, endpoint_grid_count=59, eligible_endpoint_count=len(candidates), shortlist_families=['short_x_control','upper','upper_side'], attempts=attempts,
        wall_s=stamp()-began, usable=bool(accepted), fixed_35cm_endpoint=False,
        fixed_attitude='during Cartesian extraction; original free transport afterwards',
        loaded_stops=2, endpoints_fixed=True)
    if accepted:
        selected = min(accepted, key=lambda attempt: attempt['duration_s'])
        record.update(selected_rank=selected['rank'], displacement_m=selected['displacement_m'],
                      timing=dict(duration_s=selected['duration_s']), endpoint_error=selected['endpoint_error'])
    return record



def restore_baseline(directory, output, case, seed, index):
    """Refuse cross-scene/seed reuse before copying any prior experiment files."""
    import shutil
    prior = json.loads((directory/'comparison.json').read_text())
    report_path = directory/f'{index:03d}_report.json'
    baseline = json.loads(report_path.read_text())
    if (prior['task'] != case['task'] or prior['removed_cells'] != case['removed']
            or prior['occupied_cells'] != occupied_cells(case) or prior['seed'] != seed
            or prior['baseline'] != baseline or baseline['index'] != index):
        raise ValueError('Reused baseline does not match the requested scene, seed and task')
    paths = [report_path, directory/f'{index:03d}_source.json.gz']
    if baseline.get('complete_candidate'):
        paths += [directory/f'{index:03d}_{phase}.npz' for phase in (
            'approach', 'contact_approach', 'extract', 'transport', 'conveyor_place', 'return_home')]
    if not all(path.is_file() for path in paths):
        raise FileNotFoundError('Incomplete baseline source/report/TimeOpt arrays')
    for path in paths:
        shutil.copyfile(path, output/path.name)
    return baseline


def run(args):
    import torch
    from v3_fast_three_layers import FastThreeLayerPlanner, stamp
    from v3_fast_path import read, save, stage_setup
    # Never overwrite another run's provenance or partial evidence.
    args.output.mkdir(parents=True, exist_ok=False)
    import subprocess
    save(args.output / 'provenance.json', dict(
        baseline_commit=BASELINE_COMMIT,
        source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        source_file=str(Path(__file__).resolve()),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        cases=args.cases, seeds=args.seeds, clearance_only=args.clearance_only, baseline_reuse=str(args.reuse_baselines) if args.reuse_baselines else None,
        attitude_policy='held during selected Cartesian extraction; original free transport afterwards', same_contact_and_carry_goal=True,
        primary_metric='whole candidate motion duration, not hardware or end-to-end planning latency'))
    init = stamp()
    engine = FastThreeLayerPlanner()
    initialization_s = stamp() - init
    native_configure = engine.planner.configure_task
    calls = []
    seeded_search = engine.planner.verified_rrt
    records = []
    for case_name in args.cases:
        case = CASES[case_name]
        engine.planner.configure_task = native_configure
        install_scene(engine.planner, case)
        index = next(index for index, (_, task) in enumerate(engine.planner.task_options) if task == case['task'])
        for seed in args.seeds:
            output = args.output / f'{case_name}_s{seed}'
            output.mkdir(exist_ok=False)
            engine.planner.verified_rrt = seeded_search
            calls.clear()
            install_seed(engine.planner, seed, calls)
            np.random.seed(seed)
            torch.manual_seed(seed)
            began = stamp()
            if args.reuse_baselines:
                baseline_directory = args.reuse_baselines / f'{case_name}_s{seed}'
                baseline = restore_baseline(baseline_directory, output, case, seed, index)
            else:
                baseline = engine.full_cycle(index, output_directory=output)
            source = read(output / f'{index:03d}_source.json.gz')
            record = dict(case=case_name, task=case['task'], removed_cells=case['removed'],
                          occupied_cells=occupied_cells(case), seed=seed, baseline=baseline,
                          variants=[], baseline_reused=bool(args.reuse_baselines), rrt_seeds=list(calls), scene=source.get('visual_scene_snapshot'))
            save(output / 'comparison.json', record)
            if baseline.get('complete_candidate'):
                rows = loaded_rows(source)
                stages = {stage['phase']: stage for stage in baseline['stages']}
                record['baseline_loaded_s'] = sum(stages[phase]['time_optimization']['duration_s'] for phase in ('extract', 'transport'))
                _, _, validity, _ = stage_setup(engine.planner, source, 'transport')
                record['baseline_loaded_collision'] = {}
                for phase in ('extract', 'transport'):
                    positions = np.load(output / f'{index:03d}_{phase}.npz')['position']
                    record['baseline_loaded_collision'][phase] = collision_diagnostic(validity, positions)
                try:
                    adaptive = adaptive_xyz(engine, source, rows, output, case, seed, args.clearance_only)
                    record['variants'].append(adaptive)
                    for name, nonaxial in [('adaptive_xyz_nonaxial', True), ('short_x_control', False)]:
                        accepted = [a for a in adaptive['attempts'] if a['usable'] and
                            any(abs(d[1])+abs(d[2]) > 1e-8 for d in a['displacement_m'].values()) == nonaxial]
                        choice = dict(name=name, usable=bool(accepted), attempts=adaptive['attempts'],
                                      wall_s=adaptive['wall_s'], eligible_endpoint_count=adaptive['eligible_endpoint_count'])
                        if accepted:
                            best = min(accepted, key=lambda a: a['duration_s'])
                            choice.update(selected_rank=best['rank'], displacement_m=best['displacement_m'],
                                          timing=dict(duration_s=best['duration_s']))
                        record['variants'].append(choice)
                except Exception as error:
                    import traceback
                    traceback.print_exc()
                    record['variants'].extend(dict(name=name, error=repr(error), usable=False, attempts=[])
                        for name in ('adaptive_xyz', 'adaptive_xyz_nonaxial', 'short_x_control'))
                for variant in record['variants']:
                    duration = variant.get('timing', {}).get('duration_s')
                    if duration is not None:
                        variant['whole_cycle_s'] = baseline['motion_s'] - record['baseline_loaded_s'] + duration
                        variant['whole_cycle_improvement_pct'] = 100 * (baseline['motion_s'] - variant['whole_cycle_s']) / baseline['motion_s']
                        variant['loaded_improvement_pct'] = 100 * (record['baseline_loaded_s'] - duration) / record['baseline_loaded_s']
            record['trial_wall_s'] = stamp() - began
            record['rrt_seeds'] = list(calls)
            save(output / 'comparison.json', record)
            records.append(record)
            save(args.output / 'summary.json', dict(baseline_commit=BASELINE_COMMIT,
                 initialization_s=initialization_s, records=records,
                 policy='Paired same contact, same carry goal, same other four phases. Counterfactual motion comparison; baseline shared planning is not a fresh end-to-end free planner timing. Explicit cleared 25-cell scenes; original PR82 algorithm unchanged. Diagnostic collision samples are not continuous safety certification.'))
            print('COMPARISON', case_name, seed, json.dumps(dict(
                baseline=baseline.get('motion_s'), variants=[dict(name=v['name'], usable=v['usable'],
                duration=v.get('timing', {}).get('duration_s'), error=v.get('error')) for v in record['variants']],
                wall_s=record['trial_wall_s'])), flush=True)


def self_check():
    for case in CASES.values():
        occupied = set(occupied_cells(case))
        assert occupied.isdisjoint(case['task'].values())
        assert occupied.isdisjoint(case['removed'])
        assert len(occupied) + len(case['removed']) + 2 == 25
    try:
        occupied_cells(dict(task={'left': 1, 'right': 2}, removed=[1]))
    except ValueError:
        pass
    else:
        raise AssertionError('Target removal must be rejected')
    from curobo_core.contracts import ACTIVE_JOINTS
    source = dict(joint_names=['base_x', *ACTIVE_JOINTS],
                  phases=['contact_approach', 'extract', 'extract', 'transport', 'return_home'],
                  frames=np.arange(80).reshape(5, 16).tolist())
    np.testing.assert_array_equal(loaded_rows(source), np.asarray(source['frames'])[:4, 1:])
    source['phases'][2] = 'approach'
    try:
        loaded_rows(source)
    except ValueError:
        pass
    else:
        raise AssertionError('Noncontiguous loaded phases must be rejected')
    from types import SimpleNamespace
    calls = []
    fake = SimpleNamespace(verified_rrt=lambda *args, **kwargs: (args, kwargs))
    install_seed(fake, 7, calls)
    args, _ = fake.verified_rrt(0, 0, 0, 0, 0, 2., 123)
    assert calls == [700144] and args[6] == 700144
    print('CPU self-check passed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', nargs='+', choices=list(CASES), default=list(CASES))
    parser.add_argument('--seeds', type=int, nargs='+', default=[11, 23, 37, 51, 71, 89, 101])
    parser.add_argument('--clearance-only', action='store_true')
    parser.add_argument('--reuse-baselines', type=Path, help='Explicit counterfactual: reuse identical previously computed baseline sources/other four phases')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--self-check', action='store_true')
    arguments = parser.parse_args()
    if arguments.self_check:
        self_check()
    elif arguments.output is None:
        parser.error('--output is required for GPU runs')
    elif arguments.clearance_only and arguments.reuse_baselines is None:
        parser.error('--clearance-only requires --reuse-baselines')
    elif len(set(arguments.seeds)) != len(arguments.seeds) or len(set(arguments.cases)) != len(arguments.cases):
        parser.error('Duplicate cases or seeds would overwrite trial outputs')
    else:
        run(arguments)
