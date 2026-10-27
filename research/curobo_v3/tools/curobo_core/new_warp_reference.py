import time
import numpy as np
from scipy.spatial.transform import Rotation
from v3_suction_cluster_navigation import ClusterCollision


def smooth(value):
    return 10 * value**3 - 15 * value**4 + 6 * value**5


def motion_metrics(points):
    displacement = np.diff(points, axis=0)
    speed = np.linalg.norm(displacement, axis=1)
    cosine = (displacement[:-1] * displacement[1:]).sum(-1) / np.maximum(speed[:-1] * speed[1:], 1e-20)
    turns = np.rad2deg(np.arccos(np.clip(cosine, -1., 1.)))
    return dict(length_m=float(speed.sum()), max_direction_step_deg=float(turns[25:-25].max()),
                late_direction_step_deg=float(turns[300:490].max()),
                late_min_step_mm=float(speed[300:490].min() * 1000),
                speed=speed.tolist())


class PolarCurve:
    def __init__(self, original, context, strength, bend, lift_ratio):
        self.original = original
        self.context = context
        self.strength = strength
        self.bend = bend
        self.lift_ratio = lift_ratio
        endpoints = original.evaluate([0., 1.])['centers'] - context['shoulders']
        self.radius = np.linalg.norm(endpoints, axis=1)
        self.directions = endpoints / self.radius[:, None]
        self.angle = float(np.arccos(np.clip(np.dot(*self.directions), -1., 1.)))
        normal = np.cross(*self.directions)
        if np.linalg.norm(normal) < 1e-7:
            normal = np.cross(self.directions[0], [0., 0., 1.])
            if np.linalg.norm(normal) < 1e-7:
                normal = np.cross(self.directions[0], [0., 1., 0.])
        self.normal = normal / np.linalg.norm(normal)
        self.middle = (1 - strength) * self.radius.mean() + strength * context['preferred_radius']
        lift_speed = np.linalg.norm(context['shoulders'][1] - context['shoulders'][0]) * 1.875 * max(lift_ratio, 1 / lift_ratio)
        radius_speed = max(abs(self.middle - self.radius[0]), abs(self.radius[1] - self.middle)) * 1.875 / .35
        direction_speed = self.angle * 1.875 + abs(bend) * np.pi * 1.875
        self.center_bound = lift_speed + radius_speed + max(*self.radius, self.middle) * direction_speed
        self.omega_bound = original.angle * 1.875 * max(original.timing_ratio, 1 / original.timing_ratio)

    def evaluate(self, parameter):
        parameter = np.asarray(parameter)
        phase = smooth(parameter)
        warped = self.lift_ratio * parameter / (1 + (self.lift_ratio - 1) * parameter)
        lift_phase = smooth(warped)
        shoulder = self.context['shoulders'][0] + lift_phase[:, None] * np.diff(self.context['shoulders'], axis=0)
        radial_phase = smooth(np.minimum(parameter / .35, 1.))
        final_phase = smooth(np.maximum((parameter - .65) / .35, 0.))
        radius = self.radius[0] + (self.middle - self.radius[0]) * radial_phase + (self.radius[1] - self.middle) * final_phase
        direction = Rotation.from_rotvec(phase[:, None] * self.angle * self.normal).apply(np.tile(self.directions[0], (len(parameter), 1)))
        tilt = self.bend * np.sin(np.pi * phase)**2
        direction = np.cos(tilt)[:, None] * direction + np.sin(tilt)[:, None] * self.normal
        wrist = shoulder + radius[:, None] * direction
        original = self.original.evaluate(parameter)
        position = original['positions'] + wrist - original['centers']
        return dict(wrists=wrist, shoulders=shoulder, radius=radius, positions=position,
                    rotations=original['rotations'], lift_phase=lift_phase)


def certify(curve, collision):
    lever = float(np.linalg.norm(collision.local - curve.original.center_tool, axis=1).max())
    bound = curve.center_bound + lever * curve.omega_bound
    def clearance(parameter):
        state = curve.evaluate(parameter)
        return collision.clearance(np.column_stack((state['positions'], state['rotations'].as_quat())))
    if np.any(clearance(np.array([0., 1.])) <= 0):
        return False
    lower, upper = np.array([0.]), np.array([1.])
    for depth in range(18):
        middle = (lower + upper) / 2
        gap = clearance(middle)
        if np.any(gap <= 0):
            return False
        unresolved = gap <= bound * (upper - lower) / 2 + 1e-10
        if not unresolved.any():
            return True
        if depth == 17:
            return False
        lower, upper, middle = lower[unresolved], upper[unresolved], middle[unresolved]
        lower, upper = np.r_[lower, middle], np.r_[middle, upper]
    return False


def polar_plan(engine, label, phase, components=None, allow_colliding_reference=False):
    began = time.perf_counter()
    case, context, originals, models, axis, limits = engine.prepare(label, phase) if components is None else components
    preparation_ms = (time.perf_counter() - began) * 1000
    began = time.perf_counter()
    parameter = np.linspace(0, 1, 513)
    candidates = []
    soft_candidates = []
    checked = 0
    for ratio in (1., .75, 1.333333333):
        selected = {}
        soft_selected = {}
        score = 0.
        soft_score = 0.
        for side in ('left', 'right'):
            collision = ClusterCollision(models[side], case['event']['review_scene_objects'], 0.)
            options = []
            soft_options = []
            preferred_found = False
            for strength in (1., .75, .5, 0.):
                for bend in (0., .2, -.2):
                    curve = PolarCurve(originals[side], context[side], strength, bend, ratio)
                    state = curve.evaluate(parameter)
                    checked += 1
                    gaps = collision.clearance(np.column_stack((state['positions'][::4], state['rotations'].as_quat()[::4])))
                    if np.any(gaps <= 0) and not allow_colliding_reference:
                        continue
                    metrics = motion_metrics(state['wrists'])
                    mid_error = float(np.abs(state['radius'][102:411] - context[side]['preferred_radius']).mean())
                    cost = metrics['length_m'] + 10 * mid_error + .01 * metrics['max_direction_step_deg']
                    if allow_colliding_reference:
                        penetration = np.maximum(-gaps, 0.)
                        soft_cost = cost + 100 * float(penetration.mean() + penetration.max())
                        soft_options.append((soft_cost, curve, state, metrics, mid_error, float(gaps.min())))
                    if np.all(gaps > 0) and certify(curve, collision):
                        options.append((cost, curve, state, metrics, mid_error))
                        if metrics['max_direction_step_deg'] < 2.:
                            preferred_found = True
                            break
                if preferred_found:
                    break
            for cost, curve, state, metrics, error in sorted(options, key=lambda item: item[0]):
                selected[side] = dict(wrists=state['wrists'].tolist(), shoulders=state['shoulders'].tolist(),
                    radius=state['radius'].tolist(), positions=state['positions'].tolist(),
                    quaternions_wxyz=np.roll(state['rotations'].as_quat(), 1, axis=1).tolist(),
                    target_radius=context[side]['preferred_radius'], mid_error_mm=error * 1000,
                    continuous_external_collision_clear=True, metrics=metrics,
                    strength=curve.strength, bend=curve.bend)
                score += cost
                break
            if soft_options:
                soft_cost, curve, state, metrics, error, clearance = min(soft_options, key=lambda item: item[0])
                soft_selected[side] = dict(wrists=state['wrists'].tolist(), shoulders=state['shoulders'].tolist(),
                    radius=state['radius'].tolist(), positions=state['positions'].tolist(),
                    quaternions_wxyz=np.roll(state['rotations'].as_quat(), 1, axis=1).tolist(),
                    target_radius=context[side]['preferred_radius'], mid_error_mm=error * 1000,
                    continuous_external_collision_clear=False, reference_only=True,
                    sampled_minimum_clearance_m=clearance, metrics=metrics, strength=curve.strength, bend=curve.bend)
                soft_score += soft_cost
        if len(soft_selected) == 2:
            warped = ratio * parameter / (1 + (ratio - 1) * parameter)
            lift = case['start'][0] + (case['goal'][0] - case['start'][0]) * smooth(warped)
            soft_candidates.append((soft_score, soft_selected, lift, ratio))
        if len(selected) == 2:
            warped = ratio * parameter / (1 + (ratio - 1) * parameter)
            lift = case['start'][0] + (case['goal'][0] - case['start'][0]) * smooth(warped)
            candidates.append((score, selected, lift, ratio))
            if all(track['metrics']['max_direction_step_deg'] < 2. for track in selected.values()):
                break
    result = dict(case=dict(label=label, phase=phase), preparation_ms=preparation_ms,
                  compute_ms=(time.perf_counter() - began) * 1000, checked_candidates=checked,
                  accepted=bool(candidates), guidance_available=bool(candidates or soft_candidates),
                  used_colliding_reference=not bool(candidates) and bool(soft_candidates),
                  objects=case['event']['review_scene_objects'])
    if candidates:
        score, paths, lift, ratio = min(candidates, key=lambda item: item[0])
        result.update(paths=paths, lift=lift.tolist(), lift_ratio=ratio)
    elif soft_candidates:
        score, paths, lift, ratio = min(soft_candidates, key=lambda item: item[0])
        result.update(paths=paths, lift=lift.tolist(), lift_ratio=ratio)
    return result
