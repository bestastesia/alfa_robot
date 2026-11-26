"""gpuq only: process-local centered B-spline kernels, numeric regression, full prefix."""
from pathlib import Path
import difflib
import hashlib
import json
import shutil
import sys
import time
import traceback

HERE = Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from probe_prefix import ROOT, OUT, run, save, sha, inputs, gpu_state
OUT = OUT/'numeric_fix'


def source_hashes(library):
    return {str(p.relative_to(library)):sha(p) for p in library.rglob('*')
        if p.is_file() and p.suffix in ('.py','.cuh','.cu','.h','.cpp','.yaml','.yml','.json')}


def isolate(library, output):
    source = library/'_src/curobolib/kernels'
    tree = output/'kernels'
    shutil.copytree(source,tree,ignore=shutil.ignore_patterns('__pycache__'))
    path = tree/'trajectory/bspline/bspline_context.cuh'
    original = path.read_text()
    anchor = '            /**\n             * @brief Compute position using the basis implementation'
    assert original.count(anchor)==1
    helper = '''            // Use this span's first control, including locally constant spans far from start.
            __device__ __forceinline__ float centered_dot(const float* knots) {
                float centered[splineSupportSize];
                #pragma unroll
                for (int i = 0; i < splineSupportSize; ++i) centered[i] = knots[i] - knots[0];
                return curobo::common::dot_product<splineSupportSize>(centered, basis);
            }

'''
    patched = original.replace(anchor,helper+anchor)
    old = 'curobo::common::dot_product<splineSupportSize>(knots, basis)'
    assert patched.count(old)==4
    patched = patched.replace('return '+old+';', 'return knots[0] + centered_dot(knots);')
    patched = patched.replace(old,'centered_dot(knots)')
    path.write_text(patched)
    (output/'bspline_context.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True),patched.splitlines(True),
        fromfile='original/trajectory/bspline/bspline_context.cuh',tofile='isolated/trajectory/bspline/bspline_context.cuh')))
    for folder in ('baseline_regression','patched_regression'):
        (output/folder).mkdir()
        namespace='prefix_numeric_'+folder
        (output/folder/'regression_kernel.cuh').write_text((HERE/'regression_kernel.cuh').read_text().replace('namespace prefix_numeric {','namespace '+namespace+' {'))
    return tree


def regression(cfg, wrapper, strict):
    import numpy as np
    import torch
    from scipy.interpolate import BSpline
    from cuda.core import LaunchConfig
    from curobo._src.context import get_runtime
    from curobo._src.curobolib.backends.cuda_core_backend.launch_helper import launch_kernel
    cache = get_runtime().get_cuda_core_cache()
    stream = cache.get_stream_wrapper(torch.cuda.current_stream())
    def launch(name,count,*args):
        qualified='prefix_numeric_'+wrapper.parent.name+'::'+name
        kernel=cache.get_or_compile_kernel(source_files=[wrapper],kernel_name=qualified,
            include_dirs=cfg.get_include_dirs(),compile_flags=cfg.get_compile_flags())
        launch_kernel(qualified,stream,LaunchConfig(grid=(count+255)//256,block=256),kernel,*args)
    t=np.linspace(0,1,4097,dtype=np.float32)
    ts=torch.tensor(t,device='cuda')
    records=[]
    for degree in (3,4,5):
        affine=-.75+np.arange(degree+1)*.125
        nonlinear=np.sin(np.arange(degree+1)*1.17)*.8
        controls=np.zeros((5,6),dtype=np.float32)
        controls[:,:degree+1]=np.array([np.full(degree+1,-1),np.full(degree+1,2.5),affine,nonlinear,nonlinear+4],dtype=np.float32)
        cpu=BSpline(np.arange(2*degree+2,dtype=np.float64),controls[:,:degree+1].astype(np.float64),degree,axis=1)
        reference=np.array([cpu(degree+t.astype(np.float64),nu=k)/.75**k for k in range(4)])
        c=torch.tensor(controls,device='cuda')
        for backend in range(3):
            result=torch.empty((4,len(controls),len(t)),device='cuda')
            launch('context_check',len(controls)*len(t),c.data_ptr(),ts.data_ptr(),result.data_ptr(),len(t),len(controls),degree,backend)
            actual=result.cpu().numpy()
            constants_exact=bool(np.all(actual[0,0]==-1)&np.all(actual[0,1]==2.5)&np.all(actual[1:,:2]==0))
            errors=np.max(np.abs(actual-reference),axis=(1,2))
            valid=bool(all(np.allclose(actual[k],reference[k],rtol=2e-5,atol=tol)
                for k,tol in enumerate((2e-6,2e-5,2e-4,2e-3))))
            endpoints=np.max(np.abs(actual[:,:,[0,-1]]-reference[:,:,[0,-1]]),axis=(1,2))
            records.append(dict(degree=degree,basis=('polynomial','factored','matrix')[backend],
                constants_exact=constants_exact,cpu_reference_valid=valid,max_abs_errors=errors.tolist(),endpoint_max_abs_errors=endpoints.tolist()))
            if strict:assert constants_exact and valid,records[-1]
    c=torch.linspace(2,-1,32,device='cuda');c[-4:]=-1
    start=torch.full((4,),2.,device='cuda');goal=torch.tensor([-1.],device='cuda')
    result=torch.empty((4,len(t)),device='cuda')
    launch('local_tail_check',len(t),c.data_ptr(),start.data_ptr(),goal.data_ptr(),ts.data_ptr(),result.data_ptr(),len(t))
    actual=result.cpu().numpy()
    local_exact=bool(np.all(actual[0]==-1)&np.all(actual[1:]==0))
    if strict:assert local_exact,'non-start local constant tail failed'
    return dict(success=all(r['constants_exact'] and r['cpu_reference_valid'] for r in records) and local_exact,
        independent_reference='SciPy float64 uniform B-spline and analytic derivatives, dt=.75; includes both span endpoints',
        samples_per_curve=len(t),records=records,global_start_2_local_tail_minus1_exact=local_exact,
        local_tail_max_position_error=float(np.max(np.abs(actual[0]+1))),local_tail_max_derivative=float(np.max(np.abs(actual[1:]))))


if __name__=='__main__':
    output=OUT/(sys.argv[1] if len(sys.argv)>1 else 'attempt_02')
    output.mkdir(parents=True,exist_ok=False)
    began=time.perf_counter();report=dict(success=False)
    before=None;library=None
    try:
        import curobo
        import torch
        import numpy as np
        library=Path(curobo.__file__).parent
        before=source_hashes(library)
        save(output/'shared_library_before.json',before)
        report.update(executable=sys.executable,curobo_file=curobo.__file__,shared_library_files=len(before),
            gpu_start=gpu_state(),old_manifest=inputs()[2])
        control_evidence=[]
        for folder in ('attempt_02/transport','attempt_03/transport_wholeb'):
            with np.load(OUT.parent/folder/'transport_controls.npz',allow_pickle=False) as data:
                control_evidence.append(dict(folder=folder,updown_control_min=float(data['controls'][...,0].min()),
                    start=float(data['start'][0,0]),goal=float(data['end'][0,0])))
        report['original_control_evidence']=control_evidence
        tree=isolate(library,output)
        from curobo._src.curobolib.backends.cuda_core_backend.trajectory_config import TrajectoryKernelCfg
        original_init=TrajectoryKernelCfg.__init__
        baseline_cfg=TrajectoryKernelCfg()
        report['baseline_regression']=regression(baseline_cfg,output/'baseline_regression/regression_kernel.cuh',False)
        assert not report['baseline_regression']['success'],'regression did not reproduce old failure'
        def isolated_init(self):
            original_init(self)
            self._kernel_dir=tree/'trajectory'
        TrajectoryKernelCfg.__init__=isolated_init
        report['patched_regression']=regression(TrajectoryKernelCfg(),output/'patched_regression/regression_kernel.cuh',True)
        report['isolated_kernel_tree']=str(tree)
        report['patched_context_sha256']=sha(tree/'trajectory/bspline/bspline_context.cuh')
        report['patch_sha256']=sha(output/'bspline_context.patch')
        report['regression_wall_seconds']=time.perf_counter()-began
        save(output/'numeric_fix_report.json',report)
        print('NUMERIC_REGRESSION',report['patched_regression']['success'],'wall',report['regression_wall_seconds'],flush=True)
        prefix=output/'prefix';prefix.mkdir()
        candidate=dict(success=False)
        prefix_began=time.perf_counter()
        run(prefix,candidate)
        candidate['wall_seconds']=time.perf_counter()-prefix_began
        save(prefix/'prefix_report.json',candidate)
        report.update(prefix_success=candidate['success'],prefix_wall_seconds=candidate['wall_seconds'],
            motion_seconds=candidate.get('motion_seconds'),native_samples=candidate.get('native_samples'),success=candidate['success'])
        from curobo._src.context import get_runtime
        compiled=[str(p) for key in get_runtime().get_cuda_core_cache().compiled_kernels for p,_ in key[0]
            if '/trajectory/' in str(p)]
        assert compiled and all(str(tree) in p for p in compiled),compiled
        report['compiled_trajectory_sources']=sorted(set(compiled))
    except Exception as error:
        report.update(error=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        if before is not None:
            after=source_hashes(library)
            save(output/'shared_library_after.json',after)
            report['shared_library_unchanged']=before==after
            report['shared_library_changed_files']=[p for p in set(before)|set(after) if before.get(p)!=after.get(p)]
            report['success']=bool(report['success'] and before==after)
        report.update(wall_seconds=time.perf_counter()-began,gpu_before_exit=gpu_state())
        save(output/'numeric_fix_report.json',report)
        print('NUMERIC_FIX_RESULT',report['success'],'wall',report['wall_seconds'],'motion',report.get('motion_seconds'),flush=True)
    sys.exit(0 if report['success'] else 1)
