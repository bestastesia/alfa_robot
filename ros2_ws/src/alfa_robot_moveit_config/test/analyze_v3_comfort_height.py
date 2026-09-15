#!/usr/bin/env python3
"""Aggregate benchmark evidence. Training selection never reads held-out cases."""
import argparse
from collections import Counter
import json
from pathlib import Path
from statistics import median


def load(root, prefix):
    runs = []
    for path in sorted(root.glob(prefix+'*/summary.json')):
        run = json.loads(path.read_text())
        run['path'] = str(path.parent)
        assert run['count'] == len(run['rows'])
        runs.append(run)
    return runs


def group(runs):
    groups = {}
    for run in runs:
        c = run['config']
        key = 'fixed' if c['policy'] == 'fixed_offset' else f'{c["branch"]}:{c["ratio"]:.2f}'
        entry = groups.setdefault(key, dict(rows={}, seeds=set(), runs=[]))
        entry['runs'].append(run['path'])
        entry['seeds'].add(c['seed'])
        for row in run['rows']:
            k = (c['seed'], row['case'])
            assert k not in entry['rows'], f'duplicate request {key} {k}'
            entry['rows'][k] = row
    return groups


def describe(entry):
    rows = list(entry['rows'].values())
    ok = [r for r in rows if r['success']]
    def med(key):
        return median(r[key] for r in ok) if ok else None
    return dict(count=len(rows), success=len(ok), rate=len(ok)/len(rows),
                seeds=sorted(entry['seeds']), failure_stages=dict(Counter(
                    r['failure_stage'] for r in rows if not r['success'])),
                margin_median=med('margin'), motion_median=med('motion'),
                lift_median=med('lift_travel'), total_ms_median=med('total_ms'),
                outside_reasons=dict(Counter(r['outside_reason'] for r in rows)),
                runs=entry['runs'])


def compare(a, b):
    assert a.keys() == b.keys(), 'Comparison must have identical seeds/cases'
    common = [k for k in a if a[k]['success'] and b[k]['success']]
    result = dict(common_success=len(common),
                  newly_successful=[list(k) for k in a if not a[k]['success'] and b[k]['success']],
                  regressed=[list(k) for k in a if a[k]['success'] and not b[k]['success']])
    for metric in ('margin', 'motion', 'lift_travel', 'total_ms'):
        result[metric] = dict(baseline=median(a[k][metric] for k in common) if common else None,
                              candidate=median(b[k][metric] for k in common) if common else None,
                              paired_delta_median=median(b[k][metric]-a[k][metric] for k in common)
                              if common else None)
    result['per_seed'] = {str(seed): dict(
        baseline=sum(v['success'] for k,v in a.items() if k[0]==seed),
        candidate=sum(v['success'] for k,v in b.items() if k[0]==seed))
        for seed in sorted({k[0] for k in a})}
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--prefix', default='train_')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--freeze', action='store_true')
    args=parser.parse_args()
    runs=load(args.root,args.prefix)
    assert runs
    groups=group(runs)
    report={k:describe(v) for k,v in groups.items()}
    if args.freeze:
        assert all(set(r['config']['distances']) == {.8,.9,1.} for r in runs), 'Training only'
        assert all(r['config']['arms'] == ['left', 'right'] and
                   r['config']['boxes'] == list(range(25)) and r['count'] == 150 for r in runs)
        candidates={k:v for k,v in groups.items() if k.startswith('auto:')}
        best=max(report[k]['rate'] for k in candidates)
        near=[k for k in candidates if report[k]['rate'] >= best-.02-1e-12]
        assert all(candidates[k]['seeds']=={104729,130363,155921} for k in near)
        assert groups['fixed']['seeds']=={104729,130363,155921}
        assert all(candidates[k]['rows'].keys()==groups['fixed']['rows'].keys() for k in near)
        common=set(k for k,v in groups['fixed']['rows'].items() if v['success'])
        for k in near:
            common &= {case for case,row in candidates[k]['rows'].items() if row['success']}
        def rank(k):
            rows=candidates[k]['rows']
            return (-report[k]['rate'],
                    -median(rows[c]['margin'] for c in common) if common else 0,
                    median(rows[c]['motion'] for c in common) if common else 0,
                    median(rows[c]['lift_travel'] for c in common) if common else 0,
                    abs(float(k.split(':')[1])-.8), k)
        preferred=min(near,key=rank)
        ordered=sorted(candidates,key=lambda k:float(k.split(':')[1]))
        index=ordered.index(preferred); low=high=index
        while low>0 and ordered[low-1] in near: low-=1
        while high+1<len(ordered) and ordered[high+1] in near: high+=1
        result=dict(ratio_preferred=float(preferred.split(':')[1]),
                    ratio_min=float(ordered[low].split(':')[1]), ratio_max=float(ordered[high].split(':')[1]),
                    branch='auto', common_success_ranking_cases=len(common),
                    eligible=near, selection_order=['full_success','common_margin','common_motion',
                                                    'common_lift','distance_to_0.8'],
                    baseline_comparison=compare(groups['fixed']['rows'],candidates[preferred]['rows']))
        report['frozen']=result
    elif 'fixed' in groups:
        report['comparisons']={k:compare(groups['fixed']['rows'],v['rows']) for k,v in groups.items()
                               if k!='fixed' and v['rows'].keys()==groups['fixed']['rows'].keys()}
    args.output.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:{n:v[n] for n in ('count','success','rate')} for k,v in report.items()
                      if 'rate' in v},indent=2))
    if 'frozen' in report: print(json.dumps(report['frozen'],indent=2))


if __name__=='__main__':
    main()
