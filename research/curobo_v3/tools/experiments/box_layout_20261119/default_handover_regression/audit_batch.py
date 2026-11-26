"""Independent CPU reopening of every selected-suite catalog slot, including failures and never-started cases."""
from pathlib import Path
import argparse
import json
import sys
import time
import regression
from regression import inspect_result,changed_sources,save,sha,continuation_baseline


def audit(run):
    began=time.perf_counter();summary=json.loads((run/'summary.json').read_text())
    suite=json.loads(Path(summary.get('catalog_file',run.parent/'catalog.json')).read_text())
    regression.select_suite(suite.get('suite','original34'));count=suite['cases_count']
    if suite.get('suite')=='final36' and (summary.get('parent_run_001') or any(r.get('state')=='inherited_passed' for r in summary['cases'])):
        raise ValueError('final36 current-source audit cannot inherit old native successes')
    frozen=json.loads((run/'source_manifest.json').read_text())
    source_changes=changed_sources(frozen);rows=[]
    baseline=None
    if summary.get('parent_run_001'):
        baseline=continuation_baseline(frozen,summary['parent_run_001'],summary['selected_indices'])
    for case,worker in zip(suite['cases'],summary['cases']):
        if (case['index'],case['name'])!=(worker['index'],worker['name']):raise ValueError('catalog slots reordered or filtered')
        row=dict(index=case['index'],name=case['name'],worker_state=worker['state'],success=False,entry_return_code=worker.get('entry_return_code'))
        if worker['state']=='not_started':row['state']='not_started'
        elif not worker.get('output') or not Path(worker['output']).is_file():row.update(state='missing_result',worker_error=worker.get('error'))
        else:
            try:
                checked=inspect_result(case,worker['output'],frozen['public_native_spline'])
                row.update(state='verified',success=worker['state'] in ('passed','inherited_passed') and worker.get('entry_return_code')==0,**checked)
                row['success']=bool(worker['state'] in ('passed','inherited_passed') and worker.get('entry_return_code')==0 and checked['valid'])
            except Exception as error:
                record=json.loads(Path(worker['output']).read_text())
                row.update(state='rejected',error=repr(error),workflow=record.get('workflow'),failure=record.get('failure'),
                    raw_flags={k:record.get(k) for k in ('success','physical_success','task_success')})
        if worker['state']=='inherited_passed':
            row.update(evidence_origin='parent_run_001',parent_result_sha256=worker['parent_result_sha256'],
                parent_source_digest=worker['parent_source_digest'])
            if row.get('result_sha256')!=worker['parent_result_sha256']:row.update(success=False,state='parent_result_changed')
        else:row['evidence_origin']='this_run'
        rows.append(row)
    if len(rows)!=count or len(summary['cases'])!=count:raise ValueError('full catalog not preserved')
    source_changes_after=changed_sources(frozen)
    source_changes=source_changes or source_changes_after
    report=dict(success=summary.get('success') is True and summary.get('state')=='completed'
        and not source_changes and len(rows)==count and all(r['success'] for r in rows),cases=rows,
        passed=sum(r['success'] for r in rows),not_started=sum(r['state']=='not_started' for r in rows),
        current_source_matches=not source_changes,source_changes=source_changes,
        additive_continuation=baseline is not None,parent_baseline=baseline,
        cpu_executable=sys.executable,gpu_initialized=False,source_digest=frozen['source_digest'],
        catalog_sha256=sha(Path(summary.get('catalog_file',run.parent/'catalog.json'))),wall_seconds=time.perf_counter()-began,
        scope='Independent CPU current dispatcher and actual native arrays for every catalog slot; failed/stopped/unstarted are not success. Finite regression only.')
    if 'torch' in sys.modules or 'curobo' in sys.modules:raise RuntimeError('independent CPU audit imported a GPU library')
    save(run/f'independent_cpu{count}.json',report)
    print(json.dumps({k:report[k] for k in ('success','passed','not_started','current_source_matches','wall_seconds')}))
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True);args=parser.parse_args()
    report=audit(args.run.resolve());sys.exit(0 if report['success'] else 1)
