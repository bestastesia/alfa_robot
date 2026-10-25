#!/usr/bin/env python3
"""CPU-only progress/convergence summary from immutable episode/checkpoint metadata, not a success claim."""
import argparse
import json
from pathlib import Path
from collections import Counter

import numpy as np


def training_summary(folder):
    folder = Path(folder)
    config_path = folder / 'config.json'
    config = json.loads(config_path.read_text()) if config_path.exists() else {}
    records = [json.loads(path.read_text()) for path in sorted((folder / 'episodes').glob('*.json'))]
    records += [json.loads(path.read_text()) for path in sorted((folder / 'scene-failures').glob('*.json'))]
    records.sort(key=lambda row: row.get('episode_index', -1))
    buckets = {}
    for record in records:
        stage = record.get('curriculum_start_stage', 0)
        buckets.setdefault(stage, []).append(record)
    output = dict(config=config, recorded_episodes=len(records), curriculum={})
    for stage, rows in sorted(buckets.items()):
        success = [bool((row.get('transitions') or [{}])[-1].get('info', {}).get('success')) for row in rows]
        failures = Counter((row.get('transitions') or [{}])[-1].get('info', {}).get('reason', row.get('reason', 'success')) for row in rows)
        returns = [sum(t['reward'] for t in row.get('transitions', [])) for row in rows]
        output['curriculum'][str(stage)] = dict(episodes=len(rows), successful_suffixes=sum(success),
                    success_rate=float(np.mean(success)), mean_return=float(np.mean(returns)), reasons=dict(failures),
                    note='only stage0 measures an entire current-policy cycle; exploration policy still active')
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('folders', nargs='+', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = {str(folder): training_summary(folder) for folder in args.folders}
    if args.output:
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=1)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
