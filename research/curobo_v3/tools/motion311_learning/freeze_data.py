#!/usr/bin/env python3
"""Freeze completed teacher trees for reproducible CPU fitting and RL warm starts.

Copies complete scenes, not sampled leaves. In-progress tree leaves never enter the snapshot. The chosen set and
all content hashes are immutable; later collection continues in its own directory.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def freeze(source, output, max_scenes=64):
    source, output = Path(source), Path(output)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('snapshot output must be empty')
    output.mkdir(parents=True, exist_ok=True)
    done = sorted((source / 'done').glob('*.json'))
    # Prefer more distinct tasks before more offsets; all splits are already train-only.
    def order(path):
        value = json.loads(path.read_text())
        instance = value['instance']
        task, variant = instance.rsplit('-v', 1)
        return (int(variant), value['repeat'], task)
    done = sorted(done, key=order)[:max_scenes]
    manifest = dict(source=str(source.resolve()), complete_trees=[], files={})
    for marker in done:
        scene = marker.stem
        record = json.loads(marker.read_text())
        files = [marker] + list((source / 'episodes').glob(scene + '-*.json')) + list((source / 'nodes' / scene).glob('*'))
        if not list((source / 'episodes').glob(scene + '-*.json')):
            continue
        manifest['complete_trees'].append(scene)
        for path in files:
            relative = path.relative_to(source)
            target = output / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
            manifest['files'][str(relative)] = hashlib.sha256(target.read_bytes()).hexdigest()
    for name in ('config.json', 'initial-source-sha256.json', 'provenance.json'):
        path = source / name
        if path.exists():
            shutil.copy2(path, output / name)
            manifest['files'][name] = hashlib.sha256(path.read_bytes()).hexdigest()
    (output / 'snapshot.json').write_text(json.dumps(manifest, sort_keys=True, indent=1)+'\n')
    return manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-scenes', type=int, default=64)
    args = parser.parse_args()
    report = freeze(args.source, args.output, args.max_scenes)
    print(json.dumps(dict(trees=len(report['complete_trees']), files=len(report['files']))), flush=True)


if __name__ == '__main__':
    main()
