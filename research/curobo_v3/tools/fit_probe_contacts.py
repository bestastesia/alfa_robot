import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yourdfpy

from probe_touch_ik import ROOT


class ContactKinematics:
    def __init__(self, urdf, updown):
        self.urdf = Path(urdf)
        model = yourdfpy.URDF.load(urdf, load_meshes=False)
        model.update_cfg(dict(updown=updown))
        self.prefix = model.get_transform('arm_carriage', model.base_link)
        root = ET.parse(urdf).getroot()
        self.origins = []
        self.axes = []
        for index in range(1, 8):
            joint = root.find(f"joint[@name='left_joint{index}']")
            origin = joint.find('origin')
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            matrix[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            self.origins.append(matrix)
            self.axes.append(np.fromstring(joint.find('axis').get('xyz'), sep=' '))

    def transforms(self, degrees):
        angles = np.deg2rad(degrees)
        matrices = np.broadcast_to(self.prefix, (len(angles), 4, 4)).copy()
        for index, (origin, axis) in enumerate(zip(self.origins, self.axes)):
            motion = np.broadcast_to(np.eye(4), (len(angles), 4, 4)).copy()
            motion[:, :3, :3] = Rotation.from_rotvec(angles[:, index, None] * axis).as_matrix()
            matrices = matrices @ origin @ motion
        return matrices

    def positions(self, degrees, offset, tcp_mm):
        frames = self.transforms(degrees + offset)
        return frames[:, :3, 3] * 1000 + frames[:, :3, :3] @ np.asarray(tcp_mm)


def scores(points):
    center = points.mean(axis=0)
    errors = points - center
    lengths = np.linalg.norm(errors, axis=1)
    return dict(center_footprint_mm=center.tolist(), rms_mm=float(np.sqrt(np.mean(lengths**2))),
        maximum_mm=float(lengths.max()), per_sample_mm=lengths.tolist(), errors_xyz_mm=errors.tolist())


def fit(model, degrees, mode, seed=None):
    if mode == 'tcp_only':
        frames = model.transforms(degrees)
        rotations = frames[:, :3, :3]
        translations = frames[:, :3, 3] * 1000
        matrix = (rotations - rotations.mean(axis=0)).reshape(-1, 3)
        target = -(translations - translations.mean(axis=0)).reshape(-1)
        tcp = np.linalg.lstsq(matrix, target, rcond=None)[0]
        return np.zeros(7), tcp, dict(linear_rank=int(np.linalg.matrix_rank(matrix)))
    fit_tcp = mode == 'zero_and_tcp'
    axes = [1, 2, 3, 4, 5]
    initial = np.r_[np.zeros(5), [0.,0.,70.]] if fit_tcp else np.zeros(5)
    lower = np.r_[np.full(5, -5.), [-30.,-30.,40.]] if fit_tcp else np.full(5,-5.)
    upper = np.r_[np.full(5, 5.), [30.,30.,100.]] if fit_tcp else np.full(5,5.)

    def decode(values):
        offset = np.zeros(7)
        offset[axes] = values[:5]
        return offset, values[5:] if fit_tcp else np.array([0.,0.,70.])

    def residual(values):
        offset, tcp = decode(values)
        points = model.positions(degrees, offset, tcp)
        return (points - points.mean(axis=0)).reshape(-1)

    result = least_squares(residual, initial if seed is None else seed, bounds=(lower,upper),
        xtol=1e-12, ftol=1e-12, gtol=1e-10, max_nfev=500, x_scale='jac')
    offset, tcp = decode(result.x)
    singular = np.linalg.svd(result.jac, compute_uv=False)
    diagnostics = dict(success=bool(result.success), optimizer_message=result.message,
        objective=float(np.sum(result.fun**2)), jacobian_singular_values=singular.tolist(),
        jacobian_condition=float(singular[0]/singular[-1]) if singular[-1] > 1e-12 else None,
        active_bounds=result.active_mask.tolist(), parameters=result.x.tolist())
    return offset, tcp, diagnostics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, default=ROOT/'generated/probe_touch_runs/export_001_010_20261020/contacts_001_010_first_only.json')
    parser.add_argument('--output', type=Path, default=ROOT/'generated/probe_contact_fit_20261020')
    args = parser.parse_args()
    data = json.loads(args.input.read_text())
    rows = data['samples']
    assert len(rows) == 10 and len({row['point_id'] for row in rows}) == 10
    urdf = Path(rows[0]['urdf_path'])
    identity = hashlib.sha256(urdf.read_bytes()).hexdigest()
    assert all(row['urdf_sha256'] == identity and row['updown_reference_m'] == -.3 for row in rows)
    degrees = np.array([row['model_joint_deg'] for row in rows])
    raw = np.array([row['raw_motor_deg'] for row in rows])
    prior = np.array(rows[0]['motor_minus_model_deg'])
    np.testing.assert_allclose(degrees, raw-prior, atol=1e-10)
    model = ContactKinematics(urdf, -.3)
    frames = model.transforms(degrees)
    np.testing.assert_allclose(frames, [row['joint7_in_footprint'] for row in rows], atol=1e-9)
    baseline = model.positions(degrees, np.zeros(7), [0.,0.,70.])
    np.testing.assert_allclose(baseline, np.array([row['nominal_tip_in_footprint'] for row in rows])[:, :3,3]*1000, atol=1e-8)
    report = dict(source=str(args.input), urdf_sha256=identity, physical_constraint='User confirmed all ten poses touch same fixed physical point',
        point_ids=[row['point_id'] for row in rows], input_model_deg=degrees.tolist(), input_motor_deg=raw.tolist(),
        existing_motor_minus_model_deg=prior.tolist(), baseline=scores(baseline), fits={})
    report['without_existing_j2_compensation'] = scores(model.positions(raw, np.zeros(7), [0.,0.,70.]))
    rng = np.random.default_rng(20261020)
    for mode in ('tcp_only', 'zero_only', 'zero_and_tcp'):
        offset, tcp, diagnostics = fit(model, degrees, mode)
        item = dict(additional_model_angle_correction_deg=offset.tolist(), total_motor_angle_correction_deg=(offset-prior).tolist(),
            sign_convention='physical FK uses q_true=q_model+additional=q_motor+total; motor_to_model bias=prior-additional',
            recommended_motor_minus_model_deg=(prior-offset).tolist(), tcp_in_joint7_mm=tcp.tolist(),
            training=scores(model.positions(degrees,offset,tcp)), diagnostics=diagnostics)
        predictions = []
        parameters = []
        for held in range(len(rows)):
            keep = np.arange(len(rows)) != held
            fold_offset, fold_tcp, fold_diagnostics = fit(model,degrees[keep],mode)
            center = model.positions(degrees[keep],fold_offset,fold_tcp).mean(axis=0)
            predicted = model.positions(degrees[held:held+1],fold_offset,fold_tcp)[0]
            predictions.append(float(np.linalg.norm(predicted-center)))
            parameters.append(dict(held=rows[held]['point_id'], offset_deg=fold_offset.tolist(),tcp_mm=fold_tcp.tolist()))
        item['leave_one_out'] = dict(rms_mm=float(np.sqrt(np.mean(np.square(predictions)))),maximum_mm=max(predictions),
            errors_mm=predictions, parameter_fits=parameters)
        if mode == 'zero_and_tcp':
            starts = []
            for _trial in range(8):
                seed = np.r_[rng.uniform(-2,2,5),rng.uniform(-5,5,2),rng.uniform(60,80)]
                alternate, tool, detail = fit(model,degrees,mode,seed)
                starts.append(dict(offset_deg=alternate.tolist(),tcp_mm=tool.tolist(),rms_mm=scores(model.positions(degrees,alternate,tool))['rms_mm']))
            item['multistart'] = starts
        report['fits'][mode] = item
        print(mode, json.dumps({key:value for key,value in item.items() if key not in ('diagnostics','leave_one_out','multistart')},ensure_ascii=False),flush=True)
        print('LOO',item['leave_one_out']['rms_mm'],item['leave_one_out']['maximum_mm'],flush=True)
    joint_fit = report['fits']['zero_and_tcp']
    offset = np.array(joint_fit['additional_model_angle_correction_deg'])
    tcp = np.array(joint_fit['tcp_in_joint7_mm'])
    points = model.positions(degrees,offset,tcp)
    first_shift = offset.copy()
    first_shift[0] += 10.
    shifted_score = scores(model.positions(degrees,first_shift,tcp))
    wrist_shift = offset.copy()
    wrist_shift[6] += 15.
    transformed_tool = Rotation.from_euler('z', -15., degrees=True).apply(tcp)
    equivalent = model.positions(degrees,wrist_shift,transformed_tool)
    report['identifiability'] = dict(fixed_gauge_joints=['J1','J7'],
        reason_j1='Unknown fixed contact point can rotate around J1 with all predicted points; same-point distances do not identify J1 zero',
        reason_j7='J7 zero and tool XY orientation trade exactly; axial tool is itself invariant to J7',
        j1_plus10deg_rms_change_mm=shifted_score['rms_mm']-scores(points)['rms_mm'],
        j7_plus15deg_compensated_tcp_max_coordinate_difference_mm=float(np.max(np.abs(points-equivalent))))
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'fit_results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print('RESULT',args.output/'fit_results.json',flush=True)


if __name__ == '__main__':
    main()
