"""CPU regression: native cubic fitter must retain clearance from real donor bounds."""
from pathlib import Path
import ast
import sys
import numpy as np
from scipy.interpolate import BSpline
import yourdfpy
ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT/'tools'),str(Path(__file__).resolve().parent.parent/'transfer_fresh'),str(Path(__file__).resolve().parent.parent/'transfer_native')]
from plan_fresh import donor_retreat,dense,NAMES


def main():
    tree=ast.parse((ROOT/'tools/v3_fast_path.py').read_text());fit=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='fit_path');ns=dict(np=np,BSpline=BSpline)
    exec(compile(ast.Module(body=[fit],type_ignores=[]),'actual_fit_path','exec'),ns)
    robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    bounds=np.asarray([[robot.joint_map[n].limit.lower,robot.joint_map[n].limit.upper] for n in NAMES[1:8]]).T
    paths=[ROOT/'generated/two_box_layout/default_handover_regression_20261124/run_002/cases/32_broader_same_side_y85/default/handover/geometry/fresh_geometry.npz',ROOT/'generated/two_box_layout/joint_contract_20261124_top_handover/attempt_04/geometry/fresh_geometry.npz']
    for path in paths:
        with np.load(path,allow_pickle=False) as z:start=z['donor_release'][0].copy()
        saved=start.copy();rows,detail=donor_retreat(robot,start);assert rows is not None,detail;assert np.array_equal(start,saved)
        for subset in (rows[:31],rows[30:]):
            polyline=dense(subset);_,_,_,_,spline=ns['fit_path'](dict(positions=polyline),32);curve=spline(np.linspace(0.,1.,32001))
            assert np.minimum(polyline[:,1:8]-bounds[0],bounds[1]-polyline[:,1:8]).min()>=.000999
            assert np.minimum(curve[:,1:8]-bounds[0],bounds[1]-curve[:,1:8]).min()>.0005
        assert np.array_equal(rows[:,8:],np.repeat(start[None,8:],len(rows),axis=0)) and np.all(rows[:,0]==start[0])
        invalid=start.copy();invalid[7]=bounds[1,6]-.0005;copy=invalid.copy();result,detail=donor_retreat(robot,invalid)
        assert result is None and 'margin' in detail['failure'] and np.array_equal(invalid,copy)
    print('PASS: Side and Both-top exact-pose seven-axis exits, native cubic clearance, immutable recipient/lift, inadmissible boundary rejected without clipping')

if __name__=='__main__':main()
