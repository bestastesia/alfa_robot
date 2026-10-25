"""CPU-only fingerprints of the actual learning/planning source and asset bytes.

No planner imports, CUDA calls, or executable deserialization are needed.
code_root is the research/curobo_v3 directory. paths is an optional replacement
logical-name -> path mapping for small fixtures; registry is always included.
Missing analytic .so files are explicitly not_prepared, never given fake hashes.
"""
from __future__ import annotations

import ast
import hashlib
import json
import os
from pathlib import Path
import stat
from typing import Mapping
import xml.etree.ElementTree as ET


VERSION = 1


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def stream_hash(path, chunk_size=1024 * 1024):
    """Hash a regular file in bounded memory; fail if it changes while reading."""
    if chunk_size <= 0:
        raise ValueError('chunk_size must be positive')
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        before = os.fstat(handle.fileno())
        if not stat.S_ISREG(before.st_mode):
            raise ValueError(f'not a regular file: {path}')
        while True:
            block = handle.read(chunk_size)
            if not block:
                break
            digest.update(block)
        after = os.fstat(handle.fileno())
    identity = lambda value: (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns)
    if identity(before) != identity(after) or identity(before) != identity(Path(path).stat()):
        raise RuntimeError(f'file changed while fingerprinting: {path}')
    return digest.hexdigest(), before.st_size


def _source_closure(root):
    tools = root / 'tools'
    seeds = [tools / 'motion311_learning' / name for name in
             ('stage_env.py', 'redundancy.py', 'contracts.py', 'features.py')]
    seeds += [tools / 'curobo_core' / name for name in ('planner.py', 'backend.py', 'trajectory.py', 'contracts.py')]
    seeds.append(tools / 'v3_analytic_bridge.cpp')
    pending, found = list(seeds), {}
    while pending:
        path = pending.pop()
        label = 'code/' + path.relative_to(root).as_posix()
        if label in found:
            continue
        found[label] = path
        if not path.is_file() or path.suffix != '.py':
            continue
        tree = ast.parse(path.read_text(), filename=str(path))
        for node in ast.walk(tree):
            modules = []
            if isinstance(node, ast.Import):
                modules = [(tools, alias.name) for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                if node.level:
                    base = path.parent
                    for _ in range(node.level - 1):
                        base = base.parent
                else:
                    base = tools
                if node.module:
                    modules.append((base, node.module))
                else:
                    modules.extend((base, alias.name) for alias in node.names if alias.name != '*')
            for base, module in modules:
                module_path = base.joinpath(*module.split('.'))
                candidates = (module_path.with_suffix('.py'), module_path / '__init__.py')
                for candidate in candidates:
                    if candidate.is_file() and candidate.is_relative_to(tools):
                        pending.append(candidate)
                        # Include package initialization when it exists, too.
                        parent = candidate.parent
                        while parent != tools and parent.is_relative_to(tools):
                            init = parent / '__init__.py'
                            if init.is_file():
                                pending.append(init)
                            parent = parent.parent
                        break
    return found


def _planner_assets(root):
    """Read the declarative PlannerAssets assignments without importing modules."""
    source = root / 'tools/curobo_core/contracts.py'
    if not source.is_file():
        raise FileNotFoundError(source)
    tree = ast.parse(source.read_text(), filename=str(source))
    definitions = [node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'PlannerAssets']
    if len(definitions) != 1:
        raise ValueError('cannot identify PlannerAssets declaration')

    def literal_path(node):
        if isinstance(node, ast.Name) and node.id == 'WORKSPACE_ROOT':
            return root
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            return node.value
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div):
            return Path(literal_path(node.left)) / literal_path(node.right)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'Path' and len(node.args) == 1 and not node.keywords:
            return Path(literal_path(node.args[0]))
        raise ValueError('unsupported dynamic PlannerAssets path; explicit provenance update required')

    values = {}
    for node in definitions[0].body:
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.value is not None:
            values['asset/' + node.target.id] = Path(literal_path(node.value))
    required = {'asset/robot_config', 'asset/mobile_robot_config', 'asset/urdf',
                'asset/named_poses', 'asset/box_fit', 'asset/target_poses'}
    if not required <= values.keys():
        raise ValueError('PlannerAssets is missing required runtime paths')
    return values


def _referenced_path(value, base):
    if not isinstance(value, str) or not value:
        raise ValueError('asset reference must be a nonempty path string')
    if value.startswith('file://'):
        value = value[7:]
    if '://' in value:
        raise ValueError('unresolved asset URI cannot be fingerprinted: ' + value)
    path = Path(value).expanduser()
    return path if path.is_absolute() else base / path


def _linked_assets(paths):
    """Follow the YAML/URDF references actually consumed, never relocate stale paths."""
    output, queue, visited = {}, list(paths.items()), set()
    while queue:
        label, path = queue.pop()
        path = Path(path)
        identity = str(path.resolve())
        if identity in visited or not path.is_file():
            continue
        visited.add(identity)
        linked = []
        if path.suffix.lower() in ('.yml', '.yaml'):
            import yaml
            config = yaml.safe_load(path.read_text())
            body = config.get('robot_cfg', config) if isinstance(config, dict) else {}
            kinematics = body.get('kinematics', {}) if isinstance(body, dict) else {}
            if isinstance(kinematics, dict):
                if isinstance(kinematics.get('urdf_path'), str):
                    linked.append(('urdf', _referenced_path(kinematics['urdf_path'], path.parent)))
                for key in ('collision_spheres', 'collision_spheres_file', 'collision_spheres_path'):
                    if isinstance(kinematics.get(key), str):
                        linked.append((key, _referenced_path(kinematics[key], path.parent)))
        elif path.suffix.lower() == '.urdf':
            root = ET.parse(path).getroot()
            references = sorted({mesh.attrib['filename'] for mesh in root.iter('mesh') if 'filename' in mesh.attrib})
            linked.extend((f'mesh-{index:04d}', _referenced_path(value, path.parent)) for index, value in enumerate(references))
        for name, dependency in linked:
            key = label + '/' + name
            output[key] = dependency
            queue.append((key, dependency))
    return output


def _default_paths(root):
    paths = _source_closure(root)
    paths.update(_planner_assets(root))
    for name in ('placement_task_policy.json', 'base_link_collision_frozen.json', 'arm_carriage_collision_frozen.json'):
        paths['asset/config/' + name] = root / 'config' / name
    paths['asset/base_cuboids'] = root / 'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json'
    paths['asset/conveyor_collision_mesh'] = root / 'models/conveyor/roller_conveyor_collision_solid_v2.stl'
    paths['asset/analytic_so'] = root / 'generated/v3_analytic_071cb95/libv3_analytic_bridge.so'
    paths.update(_linked_assets(paths))
    return paths


def fingerprint(registry_path, code_root=None, *, paths: Mapping[str, str | Path] | None = None):
    """Return {version, files, prepared, digest}; no missing bytes masquerade as SHA.

    Missing sources/assets remain explicit records so an unprepared checkout can
    be inspected. assert_fingerprint rejects unprepared fingerprints, even if two
    copies have identical missing paths. This deliberately fails closed.
    """
    root = Path(code_root).resolve() if code_root is not None else Path(__file__).resolve().parents[2]
    selected = _default_paths(root) if paths is None else dict(paths)
    if 'registry' in selected:
        raise ValueError('paths must not override the registry entry')
    selected['registry'] = Path(registry_path)
    files = {}
    for label in sorted(selected):
        if not isinstance(label, str) or not label:
            raise ValueError('fingerprint labels must be nonempty strings')
        path = Path(selected[label]).expanduser().resolve()
        try:
            content_hash, size = stream_hash(path)
        except FileNotFoundError:
            status = 'not_prepared' if path.suffix == '.so' else 'missing_required'
            files[label] = {'path': str(path), 'status': status, 'sha256': None, 'size_bytes': None}
        else:
            files[label] = {'path': str(path), 'status': 'ok', 'sha256': content_hash, 'size_bytes': size}
    manifest = {'version': VERSION, 'files': files, 'prepared': all(row['status'] == 'ok' for row in files.values())}
    return {**manifest, 'digest': _digest(manifest)}


def _validate_fingerprint(value):
    if not isinstance(value, dict) or set(value) != {'version', 'files', 'prepared', 'digest'}:
        raise ValueError('invalid fingerprint envelope')
    if value['version'] != VERSION or not isinstance(value['files'], dict) or 'registry' not in value['files']:
        raise ValueError('invalid fingerprint version/manifest')
    manifest = {key: value[key] for key in ('version', 'files', 'prepared')}
    if value['digest'] != _digest(manifest):
        raise ValueError('fingerprint digest does not match its manifest')
    for label, row in value['files'].items():
        if (not isinstance(label, str) or not isinstance(row, dict)
                or set(row) != {'path', 'status', 'sha256', 'size_bytes'}):
            raise ValueError('invalid fingerprint file entry')
        if row['status'] != 'ok':
            raise ValueError(f'fingerprint is not prepared: {label} ({row["status"]})')
        content_hash = row['sha256']
        if (not isinstance(content_hash, str) or len(content_hash) != 64
                or any(character not in '0123456789abcdef' for character in content_hash)
                or not isinstance(row['size_bytes'], int) or row['size_bytes'] < 0
                or not isinstance(row['path'], str) or not Path(row['path']).is_absolute()):
            raise ValueError('invalid fingerprint hash/path/size')
    if value['prepared'] is not True:
        raise ValueError('fingerprint is not prepared')


def assert_fingerprint(expected, current):
    """Reject missing, malformed, unprepared, or changed runtime provenance."""
    _validate_fingerprint(expected)
    _validate_fingerprint(current)
    if expected['digest'] != current['digest']:
        labels = sorted(set(expected['files']) | set(current['files']))
        changed = [label for label in labels if expected['files'].get(label) != current['files'].get(label)]
        raise ValueError('runtime fingerprint changed: ' + ', '.join(changed[:12]))
