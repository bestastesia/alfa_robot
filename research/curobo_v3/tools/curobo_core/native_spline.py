"""Validated affine-stable native splines; prepare is CPU-only, install belongs inside gpuq.

Call install_native_spline() before constructing native planners in a fresh process.
Only process-local TrajectoryKernelCfg is redirected; vendor sources stay untouched.
"""
from pathlib import Path
import difflib
import hashlib
import importlib.metadata
import importlib.util
import json
import tempfile

VERSION = '0.8.0.post1+fast76c2f4b6'
HEADER = 'trajectory/bspline/bspline_context.cuh'
SOURCE_HEADER_SHA256 = '7c28f1fb7f6c55f56dda5da8fa58f715a8e2c07438ec0c931a61fd41f34c031c'
PATCHED_HEADER_SHA256 = '656179255eccb049481c07c396eead7a1da77c8e986f977b83f917b6826069be'
DEFAULT_CACHE = Path(__file__).resolve().parents[2]/'generated/native_spline_affine'


def _sha(content):
    return hashlib.sha256(content).hexdigest()


def _files(root):
    return {p.relative_to(root).as_posix():p.read_bytes() for p in sorted(root.rglob('*')) if p.is_file()}


def _tree_sha(files):
    return _sha(json.dumps({name:_sha(value) for name,value in files.items()},sort_keys=True).encode())


def _patch(original):
    if _sha(original) != SOURCE_HEADER_SHA256:
        raise ValueError('Unvalidated native B-spline header; refusing to apply patch')
    text = original.decode()
    anchor = '            /**\n             * @brief Compute position using the basis implementation'
    helper = '''            // Use this span's first control, including locally constant spans far from start.
            __device__ __forceinline__ float centered_dot(const float* knots) {
                float centered[splineSupportSize];
                #pragma unroll
                for (int i = 0; i < splineSupportSize; ++i) centered[i] = knots[i] - knots[0];
                return curobo::common::dot_product<splineSupportSize>(centered, basis);
            }

'''
    old = 'curobo::common::dot_product<splineSupportSize>(knots, basis)'
    patched = text.replace(anchor,helper+anchor)
    patched = patched.replace('return '+old+';', 'return knots[0] + centered_dot(knots);')
    patched = patched.replace(old,'centered_dot(knots)').encode()
    if _sha(patched) != PATCHED_HEADER_SHA256:
        raise ValueError('Native B-spline patch differs from the GPU-validated source')
    diff = ''.join(difflib.unified_diff(text.splitlines(True),patched.decode().splitlines(True),
        fromfile='original/'+HEADER,tofile='isolated/'+HEADER)).encode()
    return patched,_sha(diff)


def prepare_kernel_tree(source_root=None, cache_root=DEFAULT_CACHE):
    """Copy installed kernels to cache/<source hash>; return JSON-safe provenance.

    source_root optionally names a vendor kernels directory, e.g. for CPU checks.
    Every file participates in the source hash and cached output verification.
    """
    version = importlib.metadata.version('nvidia-curobo')
    if version != VERSION:
        raise ValueError(f'Unvalidated cuRobo version: {version}; expected {VERSION}')
    if source_root is None:
        spec = importlib.util.find_spec('curobo')
        if spec is None or spec.origin is None:
            raise ValueError('Installed cuRobo source location is unavailable')
        source_root = Path(spec.origin).parent/'_src/curobolib/kernels'
    files = _files(Path(source_root))
    if HEADER not in files:
        raise ValueError('Vendor kernel tree has no native B-spline header')
    source_sha = _tree_sha(files)
    files[HEADER],patch_sha = _patch(files[HEADER])
    tree_sha = _tree_sha(files)
    cache_root = Path(cache_root).resolve()
    tree = cache_root/source_sha
    if not tree.exists():
        cache_root.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(dir=cache_root,prefix='.prepare-') as temporary:
            staged = Path(temporary)/'kernels'
            for name,content in files.items():
                target = staged/name
                target.parent.mkdir(parents=True,exist_ok=True)
                target.write_bytes(content)
            try:
                staged.rename(tree)
            except OSError:
                if not tree.is_dir():raise
    if _tree_sha(_files(tree)) != tree_sha:
        raise ValueError('Cached native kernel tree differs from current validated sources')
    return dict(kernel_tree=str(tree),source_version=version,source_tree_sha256=source_sha,
        kernel_tree_sha256=tree_sha,source_header_sha256=SOURCE_HEADER_SHA256,
        patched_header_sha256=PATCHED_HEADER_SHA256,patch_sha256=patch_sha)


def install_native_spline(source_root=None, cache_root=DEFAULT_CACHE):
    """Inside gpuq, install before native trajectory compilation; same-tree calls are idempotent."""
    metadata = prepare_kernel_tree(source_root,cache_root)
    # Lazy imports: preparing and validating the source tree never initializes a GPU.
    from curobo._src.context import get_runtime
    from curobo._src.curobolib.backends.cuda_core_backend.trajectory_config import TrajectoryKernelCfg
    tree = Path(metadata['kernel_tree'])
    state = getattr(TrajectoryKernelCfg,'_native_spline_install',None)
    if state is not None and (state['metadata'] != metadata or TrajectoryKernelCfg.__init__ is not state['init']):
        raise RuntimeError('Native spline installation cannot change trees or constructor; use a fresh gpuq process')
    cache = get_runtime().get_cuda_core_cache()
    native = [key for key in cache.compiled_kernels
        if key[1].startswith('curobo::trajectory::') or any('/trajectory/' in str(p) for p,_ in key[0])]
    prepared = {key[0] for key in cache._prepared_kernels if key[0].startswith('curobo::trajectory::')}
    if ((state is None and (native or prepared))
            or any(not Path(p).is_relative_to(tree) for key in native for p,_ in key[0])
            or prepared-{key[1] for key in native}):
        raise RuntimeError('Install native spline before compiling any native trajectory; mixed kernels require a fresh gpuq process')
    if state is None:
        original_init = TrajectoryKernelCfg.__init__
        if original_init.__module__ != TrajectoryKernelCfg.__module__ or original_init.__qualname__ != 'TrajectoryKernelCfg.__init__':
            raise RuntimeError('TrajectoryKernelCfg is already overridden; use a fresh gpuq process')
        def isolated_init(self):
            original_init(self)
            self._kernel_dir = tree/'trajectory'
        # All TrajOpt, seed fitter, TimeOpt solver and sampler aliases share this class.
        TrajectoryKernelCfg.__init__ = isolated_init
        TrajectoryKernelCfg._native_spline_install = dict(metadata=metadata.copy(),init=isolated_init)
    return metadata
