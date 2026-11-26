"""Install only inside a fresh gpuq workload, before constructing any native planner."""
from pathlib import Path
import hashlib
import json

ROOT = Path(__file__).resolve().parents[5]
DEFAULT_TREE = ROOT/'generated/two_box_layout/joint_contract_20261124_prefix_native/numeric_fix/attempt_02/kernels'


def install_kernel_tree(tree=DEFAULT_TREE):
    """Select the accepted local kernels for both geometric TrajOpt and TimeOpt in this process."""
    from curobo._src.context import get_runtime
    from curobo._src.curobolib.backends.cuda_core_backend.trajectory_config import TrajectoryKernelCfg
    tree = Path(tree).resolve()
    manifest = json.loads((tree.parent/'numeric_fix_report.json').read_text())
    context = tree/'trajectory/bspline/bspline_context.cuh'
    digest = hashlib.sha256(context.read_bytes()).hexdigest()
    if digest != manifest['patched_context_sha256']:
        raise RuntimeError('isolated B-spline source differs from the accepted kernel hash')
    cache = get_runtime().get_cuda_core_cache()
    old_sources = [str(path) for key in cache.compiled_kernels for path,_ in key[0]
        if '/trajectory/' in str(path) and not Path(path).is_relative_to(tree)]
    if old_sources:
        raise RuntimeError('Install before constructing native planners in a fresh gpuq process: '+str(old_sources))
    original_init = TrajectoryKernelCfg.__init__
    def isolated_init(self):
        original_init(self)
        self._kernel_dir = tree/'trajectory'
    # All four import aliases share this class, including trajectory.py and the TimeOpt solver/sampler.
    TrajectoryKernelCfg.__init__ = isolated_init
    assert TrajectoryKernelCfg().kernel_dir == tree/'trajectory'
    return dict(kernel_tree=str(tree),context_sha256=digest,shared_library_modified=False)
