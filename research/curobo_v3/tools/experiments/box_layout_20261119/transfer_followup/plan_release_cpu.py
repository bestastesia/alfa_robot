"""CPU local redundant IK: keep donor J7 on its existing bounded branch."""
import importlib.metadata
import json
import sys
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yourdfpy

from audit_transfer_geometry import ROOT, OUT, NAMES, inputs, fk


def main():
    began = time.perf_counter()
    hashes, _, _, old = inputs()
    robot = yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
    start = old['receiver_contact_seed'][-1].astype(np.float64)
    target = fk(robot, start[None])['left'][0]
    bounds = np.array([[robot.joint_map[n].limit.lower, robot.joint_map[n].limit.upper] for n in NAMES[1:7]]).T
    rows, checks = [start], []
    for distance in np.linspace(.002, .07, 35):
        desired = target.copy()
        desired[:3, 3] -= distance*target[:3, 2]
        row = rows[-1].copy()
        def residual(values):
            row[1:7] = values
            T = fk(robot, row[None])['left'][0]
            return np.r_[T[:3, 3]-desired[:3, 3], Rotation.from_matrix(desired[:3, :3].T @ T[:3, :3]).as_rotvec()]
        result = least_squares(residual, rows[-1][1:7], bounds=bounds, ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=60)
        error = residual(result.x)
        assert np.linalg.norm(error[:3]) <= 1e-6 and np.linalg.norm(error[3:]) <= 1e-5, (distance, error)
        rows.append(row.copy())
        checks.append(dict(distance_m=float(distance), position_error_m=float(np.linalg.norm(error[:3])), angle_error_rad=float(np.linalg.norm(error[3:])), evaluations=result.nfev))
    path = np.asarray(rows, dtype=np.float32)
    np.savez_compressed(OUT/'donor_release_cpu_plan.npz', donor_release=path[:31], donor_continue=path[30:])
    report = dict(success=True, sys_executable=sys.executable, numpy_version=importlib.metadata.version('numpy'),
        scipy_version=importlib.metadata.version('scipy'), hashes=hashes, fixed_donor_joint7_rad=float(start[7]),
        reason='Fixed-swivel analytic IK switches bounded branches at12mm because donor J7 starts at3.1328425rad near +pi. Hold J7, solve the remaining six joints locally against actual URDF FK; no contact/collision gates relaxed.',
        waypoints=checks, wall_seconds=time.perf_counter()-began, scope='Kinematic CPU plan only; GPU collision/static gates still required.')
    (OUT/'donor_release_cpu_plan.json').write_text(json.dumps(report, indent=2)+'\n')
    print('CPU_RELEASE_PLAN', len(path), 'wall', report['wall_seconds'], 'max q step', np.abs(np.diff(path, axis=0)).max())


if __name__ == '__main__':
    main()
