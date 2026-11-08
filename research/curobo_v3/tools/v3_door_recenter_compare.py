#!/usr/bin/env python3
"""Summarize saved results; only equal recenter input identities are paired."""
import argparse
import json
from pathlib import Path
import statistics


def summarize(paths):
    runs=[]; stages={}
    for path in paths:
        result=json.loads(path.read_text())
        runs.append(dict(file=str(path),seed=result['seed'],algorithm=result['recenter_algorithm'],
            success=result['success'],error=result.get('error'),model_id=result.get('model_id'),
            time_parameterized=result.get('time_parameterized',False),motion_duration_s=result.get('motion_duration_s')))
        for stage in result.get('recenter_searches',[]):
            key=(stage['input_identity'],stage['algorithm'])
            if key in stages: raise ValueError('duplicate input/algorithm result; select one evidence version')
            stages[key]=stage
    paired=[]
    for identity in sorted({key[0] for key in stages}):
        pair={algorithm:stages.get((identity,algorithm)) for algorithm in ('joint_rrt','cartesian_rrt')}
        if all(pair.values()): paired.append(dict(input_identity=identity,**pair))
    aggregates={}
    for algorithm in ('joint_rrt','cartesian_rrt'):
        records=[x for x in runs if x['algorithm']==algorithm]
        metrics={}
        for field in ('actual_search_ms','tcp_length_m','extra_rotation_deg','joint_motion_rad'):
            values=[pair[algorithm][field] for pair in paired if all(pair[a]['success'] for a in ('joint_rrt','cartesian_rrt')) and field in pair[algorithm]]
            metrics[field]=statistics.median(values) if values else None
        aggregates[algorithm]=dict(attempted=len(records),succeeded=sum(x['success'] for x in records),paired_success_medians=metrics)
    return dict(runs=runs,summary=aggregates,paired_inputs=paired,
                note='No default change. Unmatched end-to-end stages are not paired; navigation duration is excluded.')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('results',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('refusing to overwrite evidence')
    a.output.write_text(json.dumps(summarize(a.results),ensure_ascii=False,indent=2)+'\n')

if __name__=='__main__':main()
