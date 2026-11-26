"""CPU-only check against the installed vendor header; runnable directly, no GPU imports."""
from pathlib import Path
import hashlib
import importlib.util
import shutil
import sys
import tempfile
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from curobo_core.native_spline import prepare_kernel_tree


def test_native_spline():
    source = Path(importlib.util.find_spec('curobo').origin).parent/'_src/curobolib/kernels'
    header = 'trajectory/bspline/bspline_context.cuh'
    digest = lambda data:hashlib.sha256(data).hexdigest()
    before = {p.relative_to(source).as_posix():digest(p.read_bytes()) for p in source.rglob('*') if p.is_file()}
    gpu_modules = {name for name in sys.modules if name.split('.')[0] in ('torch','curobo','cuda')}
    with tempfile.TemporaryDirectory() as temporary:
        temporary = Path(temporary)
        first = prepare_kernel_tree(source,temporary/'cache')
        output = Path(first['kernel_tree'])/header
        expected = '656179255eccb049481c07c396eead7a1da77c8e986f977b83f917b6826069be'
        assert digest(output.read_bytes()) == first['patched_header_sha256'] == expected
        assert first['patch_sha256'] == '429ab60c96e6a4236a8392c5799c43bf4b1642506368eb08bb3a08bf46a0bf41'
        assert output.read_bytes().split(b'#pragma once')[0] == (source/header).read_bytes().split(b'#pragma once')[0]
        stat = output.stat()
        assert prepare_kernel_tree(source,temporary/'cache') == first
        assert (output.stat().st_ino,output.stat().st_mtime_ns) == (stat.st_ino,stat.st_mtime_ns)
        changed = temporary/'vendor'
        shutil.copytree(source,changed)
        other = changed/'common/math.cuh'
        other.write_bytes(other.read_bytes()+b'\n// CPU cache invalidation check\n')
        second = prepare_kernel_tree(changed,temporary/'cache')
        assert second['source_tree_sha256'] != first['source_tree_sha256']
        assert second['kernel_tree'] != first['kernel_tree']
        assert (Path(second['kernel_tree'])/'common/math.cuh').read_bytes() == other.read_bytes()
        (changed/header).write_bytes((changed/header).read_bytes()+b'\n// unknown header\n')
        try:prepare_kernel_tree(changed,temporary/'cache')
        except ValueError as error:assert 'Unvalidated' in str(error)
        else:raise AssertionError('unknown vendor header was accepted')
        with patch('importlib.metadata.version',return_value='unknown'):
            try:prepare_kernel_tree(source,temporary/'cache')
            except ValueError as error:assert 'version' in str(error)
            else:raise AssertionError('unknown source version was accepted')
        output.write_bytes(output.read_bytes()+b'\n// corrupt cached tree\n')
        try:prepare_kernel_tree(source,temporary/'cache')
        except ValueError as error:assert 'Cached' in str(error)
        else:raise AssertionError('corrupt cached tree was reused')
    assert before == {p.relative_to(source).as_posix():digest(p.read_bytes()) for p in source.rglob('*') if p.is_file()}
    assert gpu_modules == {name for name in sys.modules if name.split('.')[0] in ('torch','curobo','cuda')}


if __name__=='__main__':
    test_native_spline()
    print('native_spline CPU check passed: validated C++ SHA, cache reuse/invalidation, refusal gates, unchanged vendor source; no GPU imports')
