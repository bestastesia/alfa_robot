import csv
from copy import deepcopy
import json
import subprocess
from pathlib import Path

import pytest

from alfa_robot_rerun.sequence_timeline import SequenceTimeline, payload_execution_times


REPO_ROOT = Path(__file__).resolve().parents[4]
COMPARE = REPO_ROOT / 'tools' / 'compare_wall_trajectory_variants.sh'


def segment(index, begin, times, *, timing_valid=True):
    frames = [dict(stage=f's{index}_{i}', joints=[float(i)], scene_index=index,
                   box_attached=False, time_from_start_s=value)
              for i, value in enumerate(times)]
    return dict(kind='segment', sequence=True, task_id='task', publisher_id='publisher', generation=1,
                segment_index=index, frame_begin=begin, frame_end=begin + len(frames),
                segment_count=2, joint_names=['joint'], scenes=[{}] * (index + 1),
                success=True, frames=frames, diagnostic_frames=[], timing_valid=timing_valid,
                execution_duration_s=times[-1] if timing_valid else 0.0,
                trajectory_sample_period=0.05)


def test_timed_duration_must_equal_final_frame_time():
    payload = segment(0, 0, [0.0, 1.0])
    payload['execution_duration_s'] = 2.0
    with pytest.raises(ValueError, match='does not match final frame time'):
        payload_execution_times(payload, payload['frames'])


def test_real_and_nominal_segment_times_are_concatenated():
    timeline = SequenceTimeline()
    first = segment(0, 0, [0.0, 1.0])
    second = segment(1, 2, [0.0, 2.0])
    assert timeline.select(first)
    assert timeline.append(first)[0]['execution_times_s'] == [0.0, 1.0]
    assert timeline.append(second)[0]['execution_times_s'] == [1.0, 3.0]
    final = dict(first, kind='result', frame_begin=0, frame_end=4, segment_count=2,
                 scenes=[{}, {}], frames=first['frames'] + second['frames'],
                 boxes=[dict(first, frame_begin=0, frame_end=2),
                        dict(second, frame_begin=2, frame_end=4)])
    bad_final = deepcopy(final)
    bad_final['boxes'][0]['execution_duration_s'] = 2.0
    with pytest.raises(ValueError, match='execution duration does not match final frame time'):
        timeline.append(bad_final)
    assert not timeline.append(final)

    nominal = SequenceTimeline()
    first = segment(0, 0, [99.0, 99.0], timing_valid=False)
    second = segment(1, 2, [99.0, 99.0], timing_valid=False)
    assert nominal.select(first)
    nominal.append(first); nominal.append(second)
    assert nominal.execution_times_s == pytest.approx([0.0, 0.05, 0.05, 0.10])


def test_failed_sequence_final_appends_diagnostic_timeline():
    timeline = SequenceTimeline()
    first = segment(0, 0, [0.0, 1.0])
    assert timeline.select(first)
    assert timeline.append(first)[0]['execution_times_s'] == [0.0, 1.0]

    failed = segment(1, 2, [99.0, 99.0], timing_valid=False)
    failed.update(success=False, frames=[], diagnostic_frames=[
        dict(stage='failure_0', joints=[0.0], scene_index=1, box_attached=False),
        dict(stage='failure_1', joints=[0.1], scene_index=1, box_attached=False),
    ], frame_end=4)
    assert timeline.append(failed)[0]['execution_times_s'] == pytest.approx([1.0, 1.05])

    final = dict(first, kind='result', success=False, frame_begin=0, frame_end=4,
                 segment_count=2, scenes=[{}, {}], frames=[],
                 diagnostic_frames=first['frames'] + failed['diagnostic_frames'],
                 boxes=[dict(first, frame_begin=0, frame_end=2),
                        dict(failed, frame_begin=2, frame_end=2)])
    assert not timeline.append(final)
    assert timeline.execution_times_s == pytest.approx([0.0, 1.0, 1.0, 1.05])


def test_summary_reports_2x2_artifact_matrix_and_requires_complete(tmp_path):
    def task(variant, seed):
        return dict(success=True, completed_count=25, segment_count=15, dual_success_count=10,
                    fallback_count=0, full_dual_pass=True, total_ms=123.0,
                    planning_seed=seed, requested_trajectory_variant=variant,
                    metrics=dict(shortcut_ms=4.0, chomp_ms=5.0, totg_ms=6.0, ruckig_ms=7.0),
                    boxes=[
                        dict(success=True, execution_duration_s=2.0, selection_score=3.0,
                             requested_trajectory_variant=variant, effective_trajectory_variant=variant,
                             max_velocity=1.0, max_acceleration=2.0, max_jerk=3.0),
                        dict(success=True, execution_duration_s=4.0, selection_score=5.0,
                             requested_trajectory_variant=variant, effective_trajectory_variant=variant,
                             max_velocity=1.5, max_acceleration=2.5, max_jerk=3.5),
                    ])

    matrix = {
        ('topk', 1): (json.dumps(task('topk', 1)), b'rrd', 'command', '0\n'),
        ('topk', 2): ('', b'rrd', '', '0\n'),
        ('shortcut_ruckig', 1): (None, b'', 'command', '1\n'),
        ('shortcut_ruckig', 2): (json.dumps(task('shortcut_ruckig', 2)), b'rrd', 'command', '0\n'),
    }
    for (variant, seed), (sequence, rrd, command, exit_code) in matrix.items():
        artifacts = tmp_path / 'artifacts' / variant / f'seed_{seed}'
        artifacts.mkdir(parents=True)
        if sequence is not None:
            (artifacts / 'sequence.json').write_text(sequence)
        (artifacts / 'live.rrd').write_bytes(rrd)
        (artifacts / 'command.txt').write_text(command)
        (artifacts / 'runner_exit_code.txt').write_text(exit_code)

    base = [str(COMPARE), '--variants', 'topk,shortcut_ruckig', '--seeds', '1,2',
            '--output-dir', str(tmp_path), '--summarize-only']
    subprocess.run([*base, '--allow-incomplete'], check=True)
    rows = json.loads((tmp_path / 'summary.json').read_text())['runs']
    assert len(rows) == 4
    rows = {(row['requested_variant'], row['seed']): row for row in rows}
    assert rows['topk', 1]['execution_duration_s'] == 6.0
    assert rows['topk', 1]['selection_score_sum'] == 8.0
    assert rows['topk', 1]['max_jerk'] == 3.5
    assert rows['topk', 1]['run_complete'] is True
    assert rows['shortcut_ruckig', 2]['run_complete'] is True
    assert (rows['topk', 2]['sequence_exists'], rows['topk', 2]['sequence_nonempty']) == (True, False)
    assert (rows['topk', 2]['command_exists'], rows['topk', 2]['command_nonempty']) == (True, False)
    assert (rows['shortcut_ruckig', 1]['rrd_exists'], rows['shortcut_ruckig', 1]['rrd_nonempty']) == (True, False)
    assert rows['shortcut_ruckig', 1]['sequence_exists'] is False
    assert rows['shortcut_ruckig', 1]['run_complete'] is False
    degraded = task('shortcut_ruckig', 2)
    degraded.update(fallback_count=1, full_dual_pass=False)
    artifacts = tmp_path / 'artifacts' / 'shortcut_ruckig' / 'seed_2'
    (artifacts / 'sequence.json').write_text(json.dumps(degraded))
    subprocess.run([*base, '--allow-incomplete'], check=True)
    rerun_rows = json.loads((tmp_path / 'summary.json').read_text())['runs']
    rerun_rows = {(row['requested_variant'], row['seed']): row for row in rerun_rows}
    assert rerun_rows['shortcut_ruckig', 2]['run_complete'] is False
    with (tmp_path / 'summary.csv').open() as stream:
        assert {'sequence_exists', 'sequence_nonempty', 'rrd_exists', 'rrd_nonempty',
                'command_exists', 'command_nonempty', 'variant_identity_valid',
                'seed_identity_valid', 'run_complete'} <= set(next(csv.reader(stream)))

    required = subprocess.run(base, capture_output=True, text=True)
    assert required.returncode == 1
    assert (tmp_path / 'summary.md').stat().st_size > 0


def test_summarize_only_rejects_variant_mismatch_and_allows_chomp_fallback(tmp_path):
    task = dict(
        success=True, completed_count=25, segment_count=15, dual_success_count=10,
        fallback_count=0, full_dual_pass=True, planning_seed=7,
        requested_trajectory_variant='chomp_ruckig',
        boxes=[
            dict(success=True, requested_trajectory_variant='chomp_ruckig',
                 effective_trajectory_variant='chomp_ruckig'),
            dict(success=True, requested_trajectory_variant='chomp_ruckig',
                 effective_trajectory_variant='shortcut_ruckig'),
        ])
    artifacts = tmp_path / 'artifacts' / 'chomp_ruckig' / 'seed_7'
    artifacts.mkdir(parents=True)
    (artifacts / 'sequence.json').write_text(json.dumps(task))
    (artifacts / 'live.rrd').write_bytes(b'rrd')
    (artifacts / 'command.txt').write_text('command')
    (artifacts / 'runner_exit_code.txt').write_text('0\n')
    command = [str(COMPARE), '--variants', 'chomp_ruckig', '--seeds', '7',
               '--output-dir', str(tmp_path), '--summarize-only']

    subprocess.run(command, check=True)
    row = json.loads((tmp_path / 'summary.json').read_text())['runs'][0]
    assert row['variant_identity_valid'] is True
    assert row['seed_identity_valid'] is True
    assert row['effective_variant'] == 'chomp_ruckig|shortcut_ruckig'
    assert row['run_complete'] is True

    task['planning_seed'] = 8
    (artifacts / 'sequence.json').write_text(json.dumps(task))
    rejected = subprocess.run(command, capture_output=True, text=True)
    assert rejected.returncode == 1
    row = json.loads((tmp_path / 'summary.json').read_text())['runs'][0]
    assert row['seed_identity_valid'] is False
    assert row['run_complete'] is False

    task['planning_seed'] = 7
    task['requested_trajectory_variant'] = 'shortcut_ruckig'
    for box in task['boxes']:
        box['requested_trajectory_variant'] = 'shortcut_ruckig'
        box['effective_trajectory_variant'] = 'shortcut_ruckig'
    (artifacts / 'sequence.json').write_text(json.dumps(task))
    rejected = subprocess.run(command, capture_output=True, text=True)
    assert rejected.returncode == 1
    row = json.loads((tmp_path / 'summary.json').read_text())['runs'][0]
    assert row['requested_variant'] == 'shortcut_ruckig'
    assert row['variant_identity_valid'] is False
    assert row['run_complete'] is False


def test_rejects_all_unsupported_dynamics_variants(tmp_path):
    for variants in ('dynamics', 'dynamic_optimization', 'topk,dynamics'):
        rejected = subprocess.run(
            [str(COMPARE), '--variants', variants, '--seeds', '7',
             '--output-dir', str(tmp_path), '--summarize-only'],
            capture_output=True, text=True)
        assert rejected.returncode == 2
        unsupported = next(value for value in variants.split(',')
                           if value not in {'topk', 'shortcut_ruckig', 'chomp_ruckig'})
        assert f'unsupported variant: {unsupported}' in rejected.stderr
    assert not (tmp_path / 'artifacts').exists()
