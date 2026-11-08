"""Existing door Cartesian bridge, isolated from the ordinary cycle defaults."""
import math
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from v3_batched_loaded_search import densify

class DoorExtraction:
    def analytic_extract(self, contact, empty_validity, loaded_validity, *,
                         sides=("left", "right"), direction=(-1., 0., 0.), distance_m=.35,
                         lift_end=None, hold_sides=(), swivel_continuation=False, step_m=.01):
        """Parameterization of f044bf1's fixed-swivel, 1cm Cartesian bridge.

        No reverse construction, per-point angle search, or globally forced
        1mm discretization. The original 35cm/two-arm defaults are unchanged.
        `step_m` shortens the Cartesian step between exact IK points (default 1 cm).
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
                or (swivel_continuation and len(moving_sides) != 1)
                or not (0. < step_m <= .01)):
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
        distances = np.linspace(.01, .35, 35) if distance_m == .35 and step_m == .01 else np.linspace(
            0., distance_m, math.ceil(distance_m/step_m)+1)[1:]
        deltas = (.005, -.005, .01, -.01, .02, -.02, .04, -.04, .08, -.08, .16, -.16, .32, -.32)
        batch_fk = getattr(self, "batch_fk", None)
        carriages = {}

        def solve_step(previous, swivels, distance):
            """Fixed-swivel IK for one 1cm step: (row, swivels, carriage, trials) or a failure record."""
            swivels, trials = list(swivels), 0
            next_row = previous.copy()
            carriage = world_to_carriage
            if lift_end is not None:
                next_row[0] = contact[0]+distance/distance_m*(lift_end-contact[0])
                # The carriage pose depends on the lift alone; repeat speculation reuses it exactly.
                if next_row[0] not in carriages:
                    self.fk(next_row)
                    carriages[next_row[0]] = np.linalg.inv(self.fk_robot.get_transform("arm_carriage", self.fk_robot.base_link))
                carriage = carriages[next_row[0]]
            for side in sides:
                side_index = ("left", "right").index(side)
                target = targets[side].copy()
                if side in moving_sides:
                    target[:3, 3] += direction*distance
                offset = 1+side_index*7
                seed = previous[offset:offset+7]
                swivel = swivels[side_index]
                solutions = self.analytic.solve(side_index, carriage @ target, swivel, seed)
                if len(solutions) == 0 and swivel_continuation and side in moving_sides:
                    for delta in deltas:
                        trials += 1
                        trial = swivel+delta
                        solutions = self.analytic.solve(side_index, carriage @ target, trial, seed)
                        if len(solutions):
                            swivels[side_index] = trial
                            break
                if len(solutions) == 0:
                    return None, dict(side=side, target=target, trials=trials, carriage=carriage)
                nearest = np.argmin(np.sum((solutions-seed)**2, axis=1))
                next_row[offset:offset+7] = solutions[nearest]
            return next_row, dict(swivels=swivels, carriage=carriage, trials=trials)

        def no_solution(state, distance):
            self.last_extract_frames=np.asarray(densify(np.asarray(rows)))
            self.last_cartesian_failure = {"joint_positions": rows[-1].tolist(),
                "requested_tool_matrix": state["target"].tolist(), "distance_m": float(distance), "reason": "fixed_swivel_ik_no_solution"}
            return None, f"{state['side']}固定冗余角解析无解@{distance*100:.1f}cm"

        step = 0
        while step < len(distances):
            if not swivel_continuation:
                next_row, state = solve_step(rows[-1], path_swivels, distances[step])
                if next_row is None:
                    return no_solution(state, distances[step])
                path_swivels = state["swivels"]
                rows.append(next_row)
                step += 1
                continue
            # Same per-step decisions as validating each 1cm segment in turn, but the IK-only
            # prefix is checked in one batch; stepwise continuation resumes at its first invalid
            # segment, whose IK row and swivel state equal the stepwise ones.
            speculative, states, previous, speculative_swivels = [], [], rows[-1], path_swivels
            for distance in distances[step:]:
                next_row, state = solve_step(previous, speculative_swivels, distance)
                states.append(state)
                if next_row is None:
                    break
                speculative.append(next_row)
                previous, speculative_swivels = next_row, state["swivels"]
            segments = [np.asarray(densify(np.asarray(pair))) for pair in zip([rows[-1]]+speculative[:-1], speculative)]
            accepted = len(segments)
            if segments:
                good = continuation_validity.mask(torch.tensor(
                    np.concatenate(segments), device="cuda", dtype=torch.float32)).cpu().numpy()
                begin = 0
                for index, segment in enumerate(segments):
                    if not good[begin:begin+len(segment)].all():
                        accepted = index
                        break
                    begin += len(segment)
            for next_row, state in zip(speculative[:accepted], states[:accepted]):
                continuation_trials += state["trials"]
                path_swivels = state["swivels"]
                rows.append(next_row)
            step += accepted
            if step == len(distances):
                break
            state = states[accepted]
            continuation_trials += state["trials"]
            if accepted == len(speculative):
                return no_solution(state, distances[step])
            next_row, path_swivels = speculative[accepted], state["swivels"]
            side = moving_sides[0]
            side_index = ("left", "right").index(side)
            offset = 1+side_index*7
            seed = rows[-1][offset:offset+7]
            target = targets[side].copy()
            target[:3, 3] += direction*distances[step]
            alternatives = []
            trial_swivels = []
            base_swivel = path_swivels[side_index]
            for delta in deltas:
                continuation_trials += 1
                trial = base_swivel+delta
                trial_solutions = self.analytic.solve(
                    side_index, state["carriage"] @ target, trial, seed)
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
                    "requested_tool_matrix": target.tolist(), "distance_m": float(distances[step]),
                    "reason": "bounded_swivel_continuation_exhausted"}
                return None, f"{side}局部冗余角续解失败@{distances[step]*100:.1f}cm"
            trial, next_row = selected
            path_swivels = list(path_swivels)
            path_swivels[side_index] = trial
            rows.append(next_row)
            step += 1
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
        for side in sides:
            if batch_fk is not None:
                actual = batch_fk.transforms(dense, side+"_tool0")
            else:
                actual = []
                for values in dense:
                    self.fk(values)
                    actual.append(self.fk_robot.get_transform(side+"_tool0", self.fk_robot.base_link))
                actual = np.asarray(actual)
            delta = actual[:, :3, 3]-targets[side][:3, 3]
            deviation = delta-np.outer(delta@direction, direction) if side in moving_sides else delta
            line_error = max(line_error, float(np.linalg.norm(deviation, axis=1).max()))
            orientation_error = max(orientation_error, float(Rotation.from_matrix(
                np.transpose(actual[:, :3, :3], (0, 2, 1))@targets[side][:3, :3]).magnitude().max()))
        if line_error > .002 or orientation_error > math.radians(1.):
            return None, f"笛卡尔插值超出基线2mm/1deg门限: {line_error*1000:.3f}mm"
        self.last_extract_frames=dense
        return dense, {"strategy": "f044_fixed_swivel", "cartesian_step_m": step_m,
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
