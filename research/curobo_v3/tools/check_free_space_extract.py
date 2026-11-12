"""CPU-only check of committed MOTION-447 evidence; optional archive/raw audit."""
import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path
import statistics
import tempfile
import zipfile

import numpy as np

from benchmark_free_space_extract import BASELINE_COMMIT, CASES, loaded_rows, occupied_cells, restore_baseline
from summarize_free_space_extract import render, sign_p, summarize
from v3_free_space_extract_viewer import joint_at, load_pair, timeline

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT/'evidence/free_space_extract_pr82_20261111'


def check(results=None, archive=None):
    report = json.loads((EVIDENCE/'report.json').read_text())
    metadata = json.loads((EVIDENCE/'archive.json').read_text())
    rows = list(csv.DictReader((EVIDENCE/'all-21-pairs.csv').open(encoding='utf-8-sig')))
    expected = {(case, seed) for case in CASES for seed in (11, 23, 37, 51, 71, 89, 101)}
    assert len(report['trials']) == len(rows) == 21
    assert {(t['case'], t['seed']) for t in report['trials']} == expected
    by_key = {(r['case'], int(r['seed'])): r for r in rows}
    assert len(by_key) == 21
    for trial in report['trials']:
        row = by_key[trial['case'], trial['seed']]
        assert trial['paired'] == (row['strict_speed_pair'] == 'True')
        for key, column in [('baseline_motion_s', 'baseline_cycle_s'), ('candidate_motion_s', 'candidate_cycle_s')]:
            assert abs(trial[key]-float(row[column])) < 1e-10
        gain = trial['baseline_motion_s']-trial['candidate_motion_s']
        assert abs(gain-float(row['gain_s'])) < 1e-10
        assert abs(100*gain/trial['baseline_motion_s']-trial['improvement_pct']) < 1e-10
    for group in report['cases']:
        paired = [t for t in report['trials'] if t['case'] == group['case'] and t['paired']]
        assert group['paired'] == len(paired)
        assert abs(statistics.median(t['improvement_pct'] for t in paired)-group['improvement_median_pct']) < 1e-10
    assert (report['paired'], report['baseline_full_success'], report['candidate_full_success']) == (18, 18, 20)
    assert report['significant_practical_gain'] is False
    assert sign_p([1]*7) == .015625 and sign_p([1]*5) == .0625 and sign_p([1, -1]) == 1.
    with tempfile.TemporaryDirectory() as temp:
        page = Path(temp)/'report.html'
        render(report, page)
        assert page.stat().st_size < 100_000
    bad = dict(position=np.zeros((2, 15)), times=np.array([0., 1.]))
    try:
        timeline(bad, dict(bad, position=bad['position']+1), '抽离＋运输')
    except ValueError:
        pass
    else:
        raise AssertionError('Discontinuous phase seam was accepted')
    with tempfile.TemporaryDirectory() as temp:
        directory = Path(temp)
        output = directory/'copy'
        output.mkdir()
        baseline = dict(index=0, complete_candidate=False)
        case = CASES['top_outer']
        (directory/'000_report.json').write_text(json.dumps(baseline))
        (directory/'000_source.json.gz').write_bytes(b'untouched fixture')
        prior = dict(task=case['task'], removed_cells=case['removed'], occupied_cells=occupied_cells(case), seed=11, baseline=baseline)
        (directory/'comparison.json').write_text(json.dumps(prior))
        assert restore_baseline(directory, output, case, 11, 0) == baseline
        assert (output/'000_source.json.gz').read_bytes() == b'untouched fixture'
        for name, value in [('seed', 23), ('removed_cells', []), ('task', CASES['upper_inner']['task'])]:
            wrong = dict(prior, **{name: value})
            (directory/'comparison.json').write_text(json.dumps(wrong))
            try:
                restore_baseline(directory, output, case, 11, 0)
            except ValueError:
                pass
            else:
                raise AssertionError(f'Wrong reused {name} was accepted')
    print('PASS: compact 21-pair evidence, CSV, statistics, HTML and seam rejection')

    if archive is not None:
        assert archive.stat().st_size == metadata['size']
        assert hashlib.sha256(archive.read_bytes()).hexdigest() == metadata['sha256']
        with zipfile.ZipFile(archive) as data:
            assert len(data.infolist()) == metadata['members']
            manifest = json.loads(data.read('MOTION-447/manifest-sha256.json'))
            for name, expected_file in manifest.items():
                payload = data.read(name)
                assert len(payload) == expected_file['size']
                assert hashlib.sha256(payload).hexdigest() == expected_file['sha256'], name
        print('PASS: immutable archive SHA256, 280 entries and every payload checksum')

    if results is None:
        return
    first = json.loads((results/'multiseed-20261111/summary.json').read_text())
    second = json.loads((results/'clearance-20261111/summary.json').read_text())
    assert first['baseline_commit'] == second['baseline_commit'] == BASELINE_COMMIT
    rebuilt = summarize([first, second])
    compact = {key: value for key, value in report.items() if key != 'evidence_summaries'}
    assert json.loads(json.dumps(rebuilt)) == compact
    bad = copy.deepcopy(second)
    bad['records'][0]['baseline']['planning_ms'] += 1
    try:
        summarize([first, bad])
    except ValueError:
        pass
    else:
        raise AssertionError('Different baseline solves were silently pooled')
    for case in CASES:
        traces = [tuple(r['rrt_seeds']) for r in first['records'] if r['case'] == case]
        assert len(set(traces)) == 7 and all(traces)
    for document in (first, second):
        assert {(r['case'], r['seed']) for r in document['records']} == expected
        for record in document['records']:
            case = CASES[record['case']]
            cells = {int(o['object_id'].rsplit('_', 1)[1]) for o in record['scene']['objects'] if o['object_id'].startswith('task_fill_cell_')}
            assert cells == set(occupied_cells(case))
    stages = 0
    for trial in report['trials']:
        pair = load_pair(results, trial)
        source = pair['source']
        original = loaded_rows(source)
        assert np.max(abs(pair['data']['baseline']['extract']['position'][0]-original[0])) < 1e-4
        for variant in pair['data'].values():
            for scope in ('抽离＋运输', '仅抽离', '仅运输'):
                track = timeline(variant['extract'], variant['transport'], scope)
                np.testing.assert_allclose(joint_at(track, 0), track['position'][0], atol=1e-7)
                np.testing.assert_allclose(joint_at(track, track['duration']+1), track['position'][-1], atol=1e-7)
        for phase in ('extract', 'transport'):
            path = Path(pair['files']['xyz'][phase]['path'])
            with np.load(path) as arrays:
                assert len(arrays['times']) == 4001 and np.isfinite(arrays['position']).all()
                assert np.max(abs(np.rad2deg(arrays['velocity'][:, 1:]))) <= 60.01
                assert np.max(abs(arrays['jerk'][:, 1:])) <= 500.1
                assert np.max(abs(arrays['torque'])/np.array([294, 294, 178, 178, 87, 52, 10]*2)) <= 1.001
                assert np.max(abs(arrays['velocity'][[0, -1]])) < 1e-5
                assert np.max(abs(arrays['acceleration'][[0, -1]])) < 1e-5
            stages += 1
    assert stages == 42
    print('PASS: archived summaries rebuild unchanged, 42 selected stages/4001-point dynamics, 126 scoped timelines, scenes and real seed diversity')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, help='Extracted generated/free_space_extract data root')
    parser.add_argument('--archive', type=Path, help='Downloaded Linear ZIP to checksum (no extraction/mutation)')
    args = parser.parse_args()
    check(args.results, args.archive)
