import hashlib
import json
from pathlib import Path


def verify_frontend_package():
    import alfa_motion_viser
    package = Path(alfa_motion_viser.__file__).resolve().parent
    lock = json.loads((Path(__file__).resolve().parents[1] / 'config/visualization_dependency.json').read_text())
    for relative, field in [('assets/manifest.json', 'assets_manifest_sha256'), ('readonly_scene.py', 'renderer_sha256')]:
        if hashlib.sha256((package / relative).read_bytes()).hexdigest() != lock[field]:
            raise ValueError(f'frontend package differs from frozen dependency: {relative}; install commit {lock["commit"]}')
    return lock
