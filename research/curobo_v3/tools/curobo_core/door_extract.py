"""Existing door Cartesian bridge, isolated from the ordinary cycle defaults."""
import math
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from v3_batched_loaded_search import densify

class DoorExtraction:
    def analytic_extract(self, contact, empty_validity, loaded_validity, *,
                         sides=("left", "right"), direction=(-1., 0., 0.), distance_m=.35,
                         lift_end=None, hold_sides=(), swivel_continuation=False):
        """Parameterization of f044bf1's fixed-swivel, 1cm Cartesian bridge.

        No reverse construction, per-point angle search, or globally forced
        1mm discretization. The original 35cm/two-arm defaults are unchanged.
        """
        self.last_cartesian_failure = None
        direction = np.asarray(direction, dtype=float)
        moving_sides = tuple(sides)
        sides = moving_sides+tuple(hold_sides)
        if (not moving_sides or len(set(sides)) != len(sides)
                or any(side not in ("left", "right") for side in sides)
                or direction.shape != (3,) or not np.isfinite(direction).all()
                or abs(np.linalg.norm(direction)-1.) > 1e-9
                or not np.isfinite(distance_m) or distance_m <= 0
                or (lift_end is not None and not np.isfinite(lift_end))
                or (swivel_continuation and len(moving_sides) != 1)):
            raise ValueError("invalid Cartesian sides, direction, distance or lift")
        self.fk(contact)
        world_to_carriage = np.linalg.inv(self.fk_robot.get_transform("arm_carriage", self.fk_robot.base_link))
        targets = {side: self.fk_robot.get_transform(side+"_tool0", self.fk_robot.base_link).copy() for side in sides}
        swivels = [self.analytic.swivel(side, contact[1+side*7:8+side*7]) for side in range(2)]
        rows = [np.asarray(contact, dtype=np.float64)]
        self.last_extract_frames=np.asarray(rows)
        continuation_trials = 0
        path_swivels = list(swivels)
        continuation_validity = None
        if swivel_continuation:
            if callable(loaded_validity):
                loaded_validity = loaded_validity()
            continuation_validity = (empty_validity if self.snapshot.policy.defer_payload_until_extract_end
                                     else loaded_validity)
            if callable(continuation_validity):
                continuation_validity = continuation_validity()
        distances = np.linspace(.01, .35, 35) if distance_m == .35 else np.linspace(
            0., distance_m, math.ceil(distance_m/.01)+1)[1:]
        for distance in distances:
            next_row = rows[-1].copy()
            if lift_end is not None:
                next_row[0] = contact[0]+distance/distance_m*(lift_end-contact[0])
                self.fk(next_row)
                world_to_carriage = np.linalg.inv(self.fk_robot.get_transform("arm_carriage", self.fk_robot.base_link))
            for side in sides:
                side_index = ("left", "right").index(side)
                target = targets[side].copy()
                if side in moving_sides:
                    target[:3, 3] += direction*distance
                offset = 1+side_index*7
                seed = rows[-1][offset:offset+7]
                swivel = path_swivels[side_index]
                solutions = self.analytic.solve(side_index, world_to_carriage @ target, swivel, seed)
                if len(solutions) == 0 and swivel_continuation and side in moving_sides:
                    for delta in (.005, -.005, .01, -.01, .02, -.02, .04, -.04,
                                  .08, -.08, .16, -.16, .32, -.32):
                        continuation_trials += 1
                        trial = swivel+delta
                        solutions = self.analytic.solve(side_index, world_to_carriage @ target, trial, seed)
                        if len(solutions):
                            swivel = trial
                            path_swivels[side_index] = trial
                            break
                if len(solutions) == 0:
                    self.last_extract_frames=np.asarray(densify(np.asarray(rows)))
                    self.last_cartesian_failure = {"joint_positions": rows[-1].tolist(),
                        "requested_tool_matrix": target.tolist(), "distance_m": float(distance), "reason": "fixed_swivel_ik_no_solution"}
                    return None, f"{side}固定冗余角解析无解@{distance*100:.1f}cm"
                nearest = np.argmin(np.sum((solutions-seed)**2, axis=1))
                next_row[offset:offset+7] = solutions[nearest]
            if swivel_continuation:
                segment = np.asarray(densify(np.asarray((rows[-1], next_row))))
                if not bool(continuation_validity.mask(torch.tensor(
                        segment, device="cuda", dtype=torch.float32)).all().item()):
                    side = moving_sides[0]
                    side_index = ("left", "right").index(side)
                    offset = 1+side_index*7
                    seed = rows[-1][offset:offset+7]
                    target = targets[side].copy()
                    target[:3, 3] += direction*distance
                    alternatives = []
                    trial_swivels = []
                    base_swivel = path_swivels[side_index]
                    for delta in (.005, -.005, .01, -.01, .02, -.02, .04, -.04,
                                  .08, -.08, .16, -.16, .32, -.32):
                        continuation_trials += 1
                        trial = base_swivel+delta
                        trial_solutions = self.analytic.solve(
                            side_index, world_to_carriage @ target, trial, seed)
                        if len(trial_solutions):
                            candidate = next_row.copy()
                            nearest = np.argmin(np.sum((trial_solutions-seed)**2, axis=1))
                            candidate[offset:offset+7] = trial_solutions[nearest]
                            alternatives.append(np.asarray(densify(np.asarray((rows[-1], candidate)))))
                            trial_swivels.append((trial, candidate))
                    selected = None
                    if alternatives:
                        sizes = [len(path) for path in alternatives]
                        good = continuation_validity.mask(torch.tensor(
                            np.concatenate(alternatives), device="cuda", dtype=torch.float32)).cpu().numpy()
                        begin = 0
                        for size, candidate in zip(sizes, trial_swivels):
                            if good[begin:begin+size].all():
                                selected = candidate
                                break
                            begin += size
                    if selected is None:
                        self.last_extract_frames=np.asarray(densify(np.asarray(rows)))
                        self.last_cartesian_failure = {"joint_positions": rows[-1].tolist(),
                            "requested_tool_matrix": target.tolist(), "distance_m": float(distance),
                            "reason": "bounded_swivel_continuation_exhausted"}
                        return None, f"{side}局部冗余角续解失败@{distance*100:.1f}cm"
                    path_swivels[side_index], next_row = selected
            rows.append(next_row)
        dense = np.asarray(densify(np.asarray(rows)))
        if callable(loaded_validity):
            loaded_validity = loaded_validity()
        extraction_validity = empty_validity if self.snapshot.policy.defer_payload_until_extract_end else loaded_validity
        if callable(extraction_validity):
            extraction_validity = extraction_validity()
        valid = extraction_validity.mask(torch.tensor(dense, device="cuda", dtype=torch.float32))
        if not bool(valid.all().item()):
            index = int((~valid).nonzero()[0].item())
            self.last_extract_frames=dense[:max(1,index)]
            self.last_cartesian_failure = {"joint_positions": dense[index].tolist(), "segment_frame": index, "reason": "path_validation"}
            return None, f"笛卡尔路径碰撞/稳定性失败@frame{index}"
        if not bool(loaded_validity.mask(torch.tensor(dense[-1:], device="cuda", dtype=torch.float32)).item()):
            self.last_extract_frames=dense[:-1]
            return None, "笛卡尔终点携箱校验失败"
        line_error = orientation_error = 0.
        for values in dense:
            self.fk(values)
            for side in sides:
                actual = self.fk_robot.get_transform(side+"_tool0", self.fk_robot.base_link)
                delta = actual[:3, 3]-targets[side][:3, 3]
                deviation = delta-direction*np.dot(delta, direction) if side in moving_sides else delta
                line_error = max(line_error, float(np.linalg.norm(deviation)))
                orientation_error = max(orientation_error, float(Rotation.from_matrix(actual[:3, :3].T @ targets[side][:3, :3]).magnitude()))
        if line_error > .002 or orientation_error > math.radians(1.):
            return None, f"笛卡尔插值超出基线2mm/1deg门限: {line_error*1000:.3f}mm"
        self.last_extract_frames=dense
        return dense, {"strategy": "f044_fixed_swivel", "cartesian_step_m": .01,
                       "fixed_updown_m": float(contact[0]) if lift_end is None else None,
                       "start_updown_m": float(contact[0]), "end_updown_m": lift_end,
                       "fixed_psi_rad": swivels if not swivel_continuation else None,
                       "start_psi_rad": swivels,
                       "end_psi_rad": path_swivels if swivel_continuation else None,
                       "swivel_continuation_trials": continuation_trials,
                       "swivel_continuation_candidate_cap_per_failed_segment": 14,
                       "held_tool_frames": [side+"_tool0" for side in hold_sides],
                       "max_line_error_mm": line_error*1000,
                       "max_orientation_error_deg": math.degrees(orientation_error)}
