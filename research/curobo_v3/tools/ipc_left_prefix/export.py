import argparse
import json
from pathlib import Path

from model import JOINT_NAMES,MOTOR_SIGNS,export_prefix


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    tasks={path.stem:export_prefix(path) for path in sorted(args.source_dir.glob('L*.json'))}
    if not tasks:
        raise ValueError('No successful recorded tasks')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(dict(schema_version=1,joint_names=JOINT_NAMES,motor_signs=MOTOR_SIGNS,
        execution_scope='pending_user_choice',tasks=tasks),ensure_ascii=False,separators=(',',':')))
    print('Recorded',len(tasks),'prefixes:',args.output)


if __name__=='__main__':
    main()
