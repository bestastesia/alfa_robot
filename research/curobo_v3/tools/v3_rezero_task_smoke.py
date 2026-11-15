import json
import argparse
from pathlib import Path

from v3_rezero_task_planner import RezeroTaskPlanner,migrate_atlases


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output-dir',type=Path,default=Path('/tmp/rezero_tasks_20261018'))
    args=parser.parse_args()
    migrate_atlases()
    planner=RezeroTaskPlanner()
    planner.warmup_resources()
    folder=args.output_dir;folder.mkdir(parents=True,exist_ok=True)
    for boxes in [dict(left=18,right=22),dict(left=7,right=5),dict(left=9,right=1),dict(left=4,right=5),dict(left=8,right=7),dict(left=4,right=2)]:
        result=planner.full_cycle(boxes,lambda message:print(boxes,message,flush=True))
        name='_'.join(('L' if side=='left' else 'R')+str(value) for side,value in boxes.items())
        (folder/(name+'.json')).write_text(json.dumps(result,ensure_ascii=False))
        print('TASK',boxes,result['success'],result.get('blocked_stage'),result.get('blocked_details'),result.get('total_ms'),flush=True)
        if not result['success']:
            print('STOP: task failure; no fallback or constraint changes',flush=True)
            break


if __name__=='__main__':main()
