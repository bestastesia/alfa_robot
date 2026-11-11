import time
import numpy as np
from curobo_core.new_warp_reference import smooth
from curobo_core.contracts import ACTIVE_JOINTS


def swivel_schedule(analytic, start, goal, count):
    ends = np.array([[analytic.swivel(arm, row[1 + arm * 7:8 + arm * 7])
                      for arm in range(2)] for row in (start, goal)])
    delta = (ends[1] - ends[0] + np.pi) % (2 * np.pi) - np.pi
    values = ends[0] + smooth(np.linspace(0., 1., count))[:, None] * delta
    return values, ends, delta


def enumerate_smooth(engine, analytic, case, samples, lift):
    began = time.perf_counter()
    count = len(lift)
    swivels, ends, delta = swivel_schedule(analytic, case['start'], case['goal'], count)
    references = case['start'] + smooth(np.linspace(0., 1., count))[:, None] * (case['goal'] - case['start'])
    model = engine.arm.model
    model.update_cfg(dict(zip(ACTIVE_JOINTS, case['start'])))
    initial = model.get_transform('arm_carriage', model.base_link).copy()
    shifted = case['start'].copy()
    shifted[0] += .01
    model.update_cfg(dict(zip(ACTIVE_JOINTS, shifted)))
    displacement = (model.get_transform('arm_carriage', model.base_link)[:3, 3] - initial[:3, 3]) / .01
    inverse = np.repeat(np.linalg.inv(initial)[None], count, axis=0)
    inverse[:, :3, 3] -= (lift - case['start'][0])[:, None] * (inverse[0, :3, :3] @ displacement)
    targets = {side: inverse @ matrices for side, matrices in samples.items()}
    rows, ids, counts, nearest, nearest_ids = [], [], [], [], []
    endpoint_error = []
    for point, height in enumerate(lift):
        arms = []
        for arm, side in enumerate(('left', 'right')):
            seed = references[point, 1 + arm * 7:8 + arm * 7]
            values = analytic.solve(arm, targets[side][point], swivels[point, arm], seed)
            values = np.unique(np.round(values, 8), axis=0)
            order = np.argsort(np.linalg.norm(values - seed, axis=1))
            arms.append(values[order])
        counts.append([len(values) for values in arms])
        pairs = len(arms[0]) * len(arms[1])
        if pairs:
            combined = np.empty((pairs, 15), dtype=np.float32)
            combined[:, 0] = height
            combined[:, 1:8] = np.repeat(arms[0], len(arms[1]), axis=0)
            combined[:, 8:] = np.tile(arms[1], (len(arms[0]), 1))
            rows.append(combined)
            ids.append(np.full(pairs, point, dtype=np.int32))
            nearest.append(combined[0])
            nearest_ids.append(point)
            if point in (0, count - 1):
                endpoint_error.append(float(np.linalg.norm(combined - references[point], axis=1).min()))
    return dict(rows=np.concatenate(rows) if rows else np.empty((0, 15), np.float32),
                ids=np.concatenate(ids) if ids else np.empty(0, np.int32),
                nearest=np.asarray(nearest, dtype=np.float32).reshape(-1, 15), nearest_ids=np.asarray(nearest_ids, dtype=np.int32),
                swivels=swivels, references=references,
                statistics=dict(enumeration_ms=(time.perf_counter() - began) * 1000, calls=2 * count,
                    counts=counts, swivel_endpoints_deg=np.rad2deg(ends).tolist(),
                    swivel_delta_deg=np.rad2deg(delta).tolist(), endpoint_min_joint_error=endpoint_error))


def nearest_chain(rows, ids, legal, start, goal):
    references = start + smooth(np.linspace(0., 1., 128))[:, None] * (goal - start)
    chosen = np.full((128, 15), np.nan)
    for point in range(128):
        indices = np.flatnonzero((ids == point) & legal)
        if len(indices):
            selected = indices[np.argmin(np.linalg.norm(rows[indices] - references[point], axis=1))]
            chosen[point] = rows[selected]
    present = np.isfinite(chosen).all(-1)
    filled = chosen.copy()
    filled[0], filled[-1] = start, goal
    available = np.flatnonzero(np.isfinite(filled).all(-1))
    for joint in range(15):
        filled[:, joint] = np.interp(np.arange(128), available, filled[available, joint])
    return filled, present
