"""Explicit left/right, SKU, suction and height combinations. Run only via gpuq."""
from collections import OrderedDict
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools'))
from v3_box_layout import parse_layout, solve_file, atomic_json, session_source_stamp
from curobo_core.box_pick import layout_model_key
from curobo_core.layout_contract import base_front_x, DEFAULT_FRONT_DISTANCE_M


def cases():
    x = base_front_x()+DEFAULT_FRONT_DISTANCE_M
    cube, tall = '40x40x30', '46.5x25.5x58.2'
    def case(name, left, right):
        boxes = [dict(side=side, size=values[0], center_m=values[1], swap_length_height=values[2] if len(values)>2 else False)
                 for side, values in [('left', left), ('right', right)]]
        return name, dict(frame_id='world', boxes=boxes)
    return [
        case('tall_left_cube_right', (tall,(x,.49,1.06)), (cube,(x,-.47,1.00))),
        case('tall_left_cube_right_different_heights', (tall,(x,.58,1.38)), (cube,(x,-.45,.85))),
        case('cube_left_tall_right_different_heights', (cube,(x,.45,.85)), (tall,(x,-.58,1.38))),
        case('swapped_tall_left_cube_right', (tall,(x,.52,1.05),True), (cube,(x,-.47,1.00))),
        case('both_tall_swapped', (tall,(x,.55,1.10),True), (tall,(x,-.52,1.00),True)),
        case('tall_top_left_cube_side_right', (tall,(x,.52,.60)), (cube,(x,-.48,1.05))),
        case('cube_side_left_tall_top_right', (cube,(x,.48,1.05)), (tall,(x,-.52,.60))),
        case('cube_top_left_tall_side_right', (cube,(x,.55,.52)), (tall,(x,-.50,1.08))),
        case('tall_side_left_cube_top_right', (tall,(x,.50,1.08)), (cube,(x,-.55,.52))),
        case('swapped_tall_top_left_cube_side_right', (tall,(x,.52,.60),True), (cube,(x,-.48,1.05))),
        case('cube_side_left_swapped_tall_top_right', (cube,(x,.48,1.05)), (tall,(x,-.52,.60),True)),
        case('tall_side_left_swapped_tall_top_right', (tall,(x,.55,1.15)), (tall,(x,-.55,.60),True)),
        case('swapped_tall_top_left_tall_side_right', (tall,(x,.55,.60),True), (tall,(x,-.55,1.15))),
        case('both_tall_top_swapped', (tall,(x,.55,.60),True), (tall,(x,-.55,.65),True)),
    ]


def select_cases(catalog, only=None):
    names=[name for name,_ in catalog]
    if len(set(names))!=len(names):raise ValueError('duplicate case names')
    for name,document in catalog:
        if not name or Path(name).name!=name or name in ('.','..'):
            raise ValueError('case names must be safe directory basenames')
        parse_layout(document)
    if only:
        unknown=set(only)-set(names)
        if unknown:raise ValueError('unknown selected cases: '+', '.join(sorted(unknown)))
        catalog=[item for item in catalog if item[0] in only]
    if not catalog:raise ValueError('no selected cases')
    return catalog


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-contacts',type=int,default=16)
    parser.add_argument('--catalog',type=Path,help='explicit {cases:[{name,request}]} JSON; no hidden case selection')
    parser.add_argument('--only',nargs='+')
    args=parser.parse_args()
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    catalog=cases() if args.catalog is None else [(r['name'],r['request']) for r in json.loads(args.catalog.read_text())['cases']]
    catalog=select_cases(catalog,args.only)
    # Group topology to avoid deliberate cache churn; never reuse a trajectory.
    catalog=sorted(catalog,key=lambda item:repr(layout_model_key(parse_layout(item[1]))))
    atomic_json(output/'catalog.json',dict(cases=[dict(name=n,request=d) for n,d in catalog]))
    pool=OrderedDict();results=[];stamp=session_source_stamp()
    for name,document in catalog:
        assert session_source_stamp()==stamp,'source changed during coverage batch'
        source=output/(name+'.json');atomic_json(source,document)
        job=argparse.Namespace(solve_file=source,output=output/name,max_contacts=args.max_contacts)
        solve_file(job,pool,dict(id=output.name,request=name,source_stamp=stamp))
        record=json.loads((job.output/'result.json').read_text())
        results.append(dict(case=name,success=record['success'],failure=record.get('failure'),failure_kind=record.get('failure_kind'),
            contacts=record.get('contact_count'),attempts=len(record.get('attempts',[])),
            physical_success=record.get('physical_success'),task_success=record.get('task_success'),process_validation=record.get('process_validation'),
            wall_seconds=record['worker_wall_seconds'],suction={s:'top' if p.top else 'side' for s,p in parse_layout(document).items()},
            request=document,source_sha256=record.get('source_sha256'),verification=record.get('replay_checks')))
        atomic_json(output/'summary.json',dict(cases=results,all_cases_done=len(results)==len(catalog),
            all_passed=all(r['success'] for r in results),scope='physical and recorded process gates per request; finite layouts are not arbitrary XYZ proof; failures unknown unless certified'))
        print('ASYMMETRIC_LAYOUT',json.dumps({k:v for k,v in results[-1].items() if k not in ('request','source_sha256','verification')},ensure_ascii=False),flush=True)


if __name__=='__main__':main()
