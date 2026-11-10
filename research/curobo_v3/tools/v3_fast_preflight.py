import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def verify_hashes(root, expected):
    for relative, checksum in expected.items():
        path = root / relative
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != checksum:
            raise ValueError(f'Release dependency mismatch: {path}')


def preflight(visualization=True, gpu=False):
    lock = json.loads((ROOT / 'config/fast_release_manifest.json').read_text())
    verify_hashes(ROOT, lock['assets_sha256'])
    import curobo
    runtime = Path(curobo.__file__).resolve().parent.parent
    verify_hashes(runtime, lock['runtime_sha256'])
    from curobo.trajectory_optimizer import TrajectoryOptimizerCfg
    import inspect
    if 'enable_timeopt' not in inspect.signature(TrajectoryOptimizerCfg.create).parameters:
        raise ValueError('Wrong cuRobo generation: require pinned Fast runtime on PYTHONPATH')
    from v3_fast_assets import prepare_fast_assets
    assets, dynamics = prepare_fast_assets()
    library = ROOT / 'generated/v3_analytic_071cb95/libv3_analytic_bridge.so'
    if not library.is_file():
        raise FileNotFoundError('Run tools/prepare_checkout.py --fast-only --compiler g++ first')
    import yaml
    from v3_fixed_body_boxes import disable_fixed_body_spheres
    robot = disable_fixed_body_spheres(yaml.safe_load(assets.robot_config.read_text()))
    spheres = robot['kinematics']['collision_spheres']
    if len(spheres['base_link']) != 71 or any(item['radius'] > 0 for item in spheres['base_link']):
        raise ValueError('Fixed body sphere slots are not disabled')
    if sum(item['radius'] > 0 for item in spheres['arm_carriage']) != 20:
        raise ValueError('Carriage must retain20 spheres')
    policy = json.loads((ROOT / 'config/fast_three_layers.json').read_text())
    frontend = None
    if visualization:
        from v3_visualization_dependency import verify_frontend_package
        frontend = verify_frontend_package()
    from curobo_core.scene import digest
    report = dict(ready=True,release=lock['release'],baseline=lock['baseline'],python=sys.version,
        curobo_root=str(runtime),policy_digest=digest(policy),policy=policy,
        robot_config=str(assets.robot_config),display_urdf=str(assets.urdf),dynamics_config=str(dynamics),
        collision=dict(fixed_body='four boxes, base71 sphere slots disabled',carriage_spheres=20,payload_spheres_per_box=40),
        visualization=frontend,version_differences={},gpu_checked=False)
    for package, version in lock['tested_versions'].items():
        try:actual=importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:actual='missing'
        if actual != version:report['version_differences'][package]=dict(tested=version,actual=actual)
    if gpu:
        from v3_fast_three_layers import FastThreeLayerPlanner
        from v3_batched_loaded_search import GpuValidity
        import numpy as np
        import torch
        planner=FastThreeLayerPlanner().planner
        planner.configure_task(dict(left=24,right=20))
        checker=planner.checker(planner.selected_task)
        valid=GpuValidity(checker,check_ground=True)
        home=torch.tensor(np.asarray(planner.home_values)[None],device='cuda',dtype=torch.float32)
        report['home_valid']=bool(valid.mask(home).all())
        report['fixed_body_box_world_enabled']=hasattr(checker,'fixed_body_world')
        report['gpu_checked']=True
        if not report['home_valid']:raise ValueError('Current release Home is invalid')
    return report


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--skip-visualization',action='store_true')
    parser.add_argument('--gpu',action='store_true')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    report=preflight(not args.skip_visualization,args.gpu)
    text=json.dumps(report,ensure_ascii=False,indent=2)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(text)


if __name__=='__main__':main()
