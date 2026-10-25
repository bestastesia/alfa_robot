"""MOTION-311 v2 (MOTION-337): stage-wise contact -> carry -> place selection behind a switch.

arm "A"   the unchanged baseline planner
arm "B0"  T_lb only: P = 1 everywhere, contact value with the explicit V2 over the baseline goal pool
arm "B"   networks: N1 (P1 and its distilled V2), N2 (P2), N3 (P3 and the time correction of the place choice)
arm "Bx"  as B with the explicit V2 (min over the goal pool with N2) instead of N1's prediction (ablation)
The downstream value V3(g) of the carry and contact stages is the analytic table (assets/<mode>/v3_b0.npz) in every
arm: N3 was trained on 150 realized carry IKs per mode and extrapolates to the rest of the carry atlas (A10); it
decides the place for the realized carry, where its inputs are in distribution.
A stage ranks its candidates by value (seconds, v3_triple_ranker), keeps those with P >= theta and attempts its
first choice alone, as the labels were made: contact = singleton lazy batch with `allocation_ms` of the shared 2 s
search (dual) or first in the order (single); carry = the single goal of that root; place = single-goal place
search, the next place on failure (`place_attempts`), then the baseline place rule. The baseline rule's own
candidate is an option of the carry and place ranking; ranked first, the stage runs the unchanged baseline rule.
No stage revisits an earlier one; a failed round is re-planned by arm A and both rounds count. Inference is numpy
on the CPU; V3 tables and models are loaded once by `setup_selection`.
"""
import time
from pathlib import Path

import numpy as np
import torch

import v3_candidate_ranker as ranker
import v3_triple_ranker as triple
from curobo_core.planner import CycleBlocked
from v3_goal_pool import exact_goal_pool
from v3_place_ik import resolve
from v3_timed_single_arm_planner import TimedSingleArmPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS

PLACE_RETRY = {'龙头车放置目标IK', '龙头车放置RRT', '龙头车放置RRT细验'}


class TripleSelectionPlanner(TimedSingleArmPlanner):
    arm = "A"
    m = 1
    place_attempts = 2
    theta = .5
    z_limit = triple.Z_LIMIT
    allocation_ms = 350.
    place_keep = triple.PLACE_KEEP
    contact_value = "distilled"  # arm B: N1's V2; "explicit": the min over the goal pool (always so for B0 and Bx)

    def setup_selection(self, assets, models=None):
        """assets/<mode>/{place_set.npz, v3_b0.npz}; models {"n1", "n2", "n3"}: v3_candidate_ranker.Ensemble."""
        began = time.perf_counter()
        from v3_research_assets import atlas_path
        self.models, self._tables, self._place_sets = models or {}, {}, {}
        for folder in sorted(path for path in Path(assets).iterdir() if (path / "place_set.npz").exists()):
            places = np.load(folder / "place_set.npz")
            self._place_sets[folder.name] = (places["rows"].astype(np.float64), places["params"].astype(np.float64))
            table = triple.RowTable(*(np.load(folder / "v3_b0.npz")[key] for key in ("carries", "v3")))
            table.weighted = torch.tensor(table.rows, device='cuda') * torch.tensor(ranker.WEIGHTS, device='cuda', dtype=torch.float32)
            self._tables[folder.name, "b0"] = table
        self._velocity = self._timed_limits[0][[self._timed_joint_names.index(name) for name in ACTIVE_JOINTS]]
        atlases = {}
        for top in (False, True):
            saved, self.top_grasp = self.top_grasp, top
            try:
                atlases["top" if top else "side"] = np.load(atlas_path(self), allow_pickle=False)["joints"]
            finally:
                self.top_grasp = saved
        self._kinematics = ranker.Kinematics(self.robot["kinematics"]["urdf_path"], analytic=self.analytic,
                                             carriage=ranker.CarriageModel(self, self.home_values), atlases=atlases)
        self.selection_setup_ms = (time.perf_counter() - began) * 1000

    # values ----------------------------------------------------------------------------------------------
    def mode(self, boxes):
        grasp = "top" if self.top_grasp else "side"
        return f"dual-{grasp}" if len(boxes) == 2 else f"single-{next(iter(boxes))}-{grasp}"

    def context(self, boxes):
        return dict(kind="dual" if len(boxes) == 2 else "single", sides=sorted(boxes), top_grasp=bool(self.top_grasp),
                    box_centers={side: list(self.snapshot.object(f"wall_box_{box:02d}").pose.position) for side, box in boxes.items()},
                    contact_poses={name: dict(position=list(pose["position"]), quaternion=list(pose["quaternion"]))
                                   for name, pose in self.contact_poses_for_task(boxes).items()},
                    home=self.home_values)

    def model(self, name):
        return self.models.get(name) if self.arm in ("B", "Bx") else None

    def table(self, boxes):
        return self._tables[self.mode(boxes), "b0"]

    def downstream(self, carries, boxes):
        """V3 of carry IKs: the arm's table, rows outside it (single-arm idle variant of the extraction end) computed."""
        table = self.table(boxes)
        index = table.lookup(carries)
        values = table.values[np.maximum(index, 0)]
        missing = np.flatnonzero(index < 0)
        if len(missing):
            places, _ = self._place_sets[self.mode(boxes)]
            values[missing], _ = triple.downstream_values(np.asarray(carries, dtype=np.float64)[missing], places, self.home_values,
                                                          self._velocity, self.context(boxes))
        return values, len(missing)

    # contact stage ---------------------------------------------------------------------------------------
    def contact_order(self, boxes, contacts):
        """Attempt order by Q1 (None: abstain) and the selection report of the round."""
        began = time.perf_counter()
        context = self.context(boxes)
        features, ends, eligible = triple.contact_features(contacts, context, self._kinematics, self._velocity)
        n1 = self.model("n1")
        if n1 is not None:
            probability, distilled, max_z = n1.predict(features)
            eligible = eligible & (max_z <= self.z_limit)
        else:
            probability, distilled = np.ones(len(contacts)), None
        if self.arm == "B" and self.contact_value == "distilled":
            downstream = distilled * triple.V2_SCALE_S
        else:
            table = self.table(boxes)
            masked = np.where(eligible[:, None], ends, np.nan)
            finite = np.isfinite(masked).all(axis=1)
            near = None
            if finite.any():  # the baseline goal pool of every end, on the GPU like v3_goal_matching
                weighted = torch.tensor(masked[finite], device='cuda', dtype=torch.float32) * table.weighted.new_tensor(ranker.WEIGHTS)
                near = exact_goal_pool(weighted, table.weighted).cpu().numpy()
            downstream = triple.contact_downstream(masked, table.rows, table.values, self._velocity, context, self.model("n2"), near=near)
        home = np.asarray(self.home_values, dtype=np.float64)
        values = (triple.time_bound(home, contacts, self._velocity) + triple.time_bound(contacts, ends, self._velocity)
                  + downstream + (1. - probability) * triple.C_FAIL_S)
        order = triple.ranked(np.where(eligible, values, np.inf), np.where(eligible, probability, 0.), self.theta)
        self.selection_report = dict(arm=self.arm, contact=dict(ms=(time.perf_counter() - began) * 1000, order=order[:self.m],
                                     probability=[float(probability[index]) for index in order[:self.m]],
                                     value_s=[float(values[index]) for index in order[:self.m]], eligible=int(eligible.sum())),
                                     abstained=not order, carry=[], place=[])
        return order or None

    def rank_contacts(self, boxes):
        self._selection_order = self._selection_batch = None
        if self.arm == "A":
            return super().rank_contacts(boxes)
        contacts = self.generate_contacts(boxes)
        self._selection_contacts = contacts
        order = self.contact_order(boxes, contacts)
        if order is None:
            return self.baseline_ranking(boxes, contacts)
        self._selection_order = order[:self.m]
        self.prediction_records = []
        return self._selection_order + [index for index in range(len(contacts)) if index not in self._selection_order]

    def lazy_batches(self, boxes, ranked):
        if self._selection_order is None:
            yield from super().lazy_batches(boxes, ranked)
            return
        for index in self._selection_order:
            self._selection_batch = True
            yield [index], 1, self.allocation_ms, "selection"
        self._selection_batch = False
        baseline = self.baseline_ranking(boxes, self._selection_contacts)
        self.selection_report["baseline_ranking"] = baseline
        yield from super().lazy_batches(boxes, baseline)

    def single_contact_order(self, boxes, contacts):
        self._selection_single = None
        if self.arm == "A":
            return super().single_contact_order(boxes, contacts)
        order = self.contact_order(boxes, np.asarray(contacts, dtype=np.float64))
        if order is None:
            return super().single_contact_order(boxes, contacts)
        top = order[:self.m]
        self._selection_single = {np.asarray(contacts[index], dtype=np.float32).tobytes() for index in top}
        return top + [index for index in super().single_contact_order(boxes, contacts) if index not in top]

    def cartesian_single(self, contact, distances, along_normal=False):
        if not along_normal:
            self._selection_active = np.asarray(contact, dtype=np.float32).tobytes() in (getattr(self, "_selection_single", None) or set())
        return super().cartesian_single(contact, distances, along_normal)

    def selection_active(self):
        return self.arm != "A" and bool(getattr(self, "_selection_batch", False) or getattr(self, "_selection_active", False))

    # carry stage -----------------------------------------------------------------------------------------
    def carry_choice(self, root, rows, baseline_index):
        """Index of the chosen carry IK among `rows`, or None for the baseline rule (abstain or baseline ranked first)."""
        began = time.perf_counter()
        boxes = self.selected_task
        downstream, computed = self.downstream(rows, boxes)
        values, probability = triple.carry_values(root, rows, downstream, self._velocity, self.context(boxes), self.model("n2"))
        order = triple.ranked(values, probability, self.theta)
        choice = order[0] if order and order[0] != baseline_index else None
        self.selection_report["carry"].append(dict(ms=(time.perf_counter() - began) * 1000, choice=choice, baseline=baseline_index,
                                                   abstained=not order, computed_v3=computed,
                                                   value_s=None if not order else float(values[order[0]]),
                                                   baseline_value_s=float(values[baseline_index]),
                                                   probability=None if not order else float(probability[order[0]])))
        return choice

    def placement_goal_order(self, root, shortlisted, distances, baseline_scores):
        if not self.selection_active():
            return None
        began = time.perf_counter()
        baseline_index = int(torch.argmin(baseline_scores()).item())
        baseline_ms = (time.perf_counter() - began) * 1000
        choice = self.carry_choice(root.cpu().numpy().astype(np.float64), shortlisted.cpu().numpy(), baseline_index)
        record = self.selection_report["carry"][-1]
        record.update(value_ms=record["ms"], baseline_score_ms=baseline_ms, ms=(time.perf_counter() - began) * 1000)
        return None if choice is None else [choice]

    def order_transport_goals(self, start, candidates):
        began = time.perf_counter()
        order = super().order_transport_goals(start, candidates)
        baseline_ms = (time.perf_counter() - began) * 1000
        if not self.selection_active():
            return order
        shortlist = order[:512]
        choice = self.carry_choice(np.asarray(start, dtype=np.float64), np.asarray(candidates)[shortlist], 0)
        record = self.selection_report["carry"][-1]
        record.update(value_ms=record["ms"], baseline_score_ms=baseline_ms, ms=(time.perf_counter() - began) * 1000)
        return order if choice is None else shortlist[[choice]]

    # place stage -----------------------------------------------------------------------------------------
    def place_options(self, start, targets, baseline_row):
        """Places to attempt in order (re-solved stored IKs); None marks the baseline rule (also reached on abstention)."""
        boxes = self.selected_task
        rows, params = self._place_sets[self.mode(boxes)]
        bound = triple.time_bound(start, rows, self._velocity) + triple.time_bound(rows, self.home_values, self._velocity)
        shortlist = np.argsort(bound, kind="stable")[:self.place_keep]
        options = np.concatenate([rows[shortlist], np.asarray(baseline_row, dtype=np.float64)[None]]) if baseline_row is not None else rows[shortlist]
        values, probability = triple.place_values(start, options, self.home_values, self._velocity, self.context(boxes), self.model("n3"))
        order = triple.ranked(values, probability, self.theta)
        attempts, record = [], dict(abstained=not order, values_s=[], probability=[], attempt_ranks=[], resolution=[])
        sides = {side: targets[side] for side in (boxes if len(boxes) == 1 else targets)}
        for rank, index in enumerate(order):
            record["values_s"].append(float(values[index]))
            record["probability"].append(float(probability[index]))
            if index == len(shortlist):
                break
            solved = resolve(self, sides, rows[shortlist[index]], params[shortlist[index]])
            record["resolution"].append(dict(rank=rank, place_index=int(shortlist[index]), success=solved is not None))
            if solved is not None:
                attempts.append(solved)
                record["attempt_ranks"].append(rank)
            if len(attempts) == self.place_attempts:
                break
        record["baseline_value_s"] = None if baseline_row is None else float(values[len(shortlist)])
        return attempts, record

    def plan_conveyor_target(self, start, targets, checker, validity, solver, report, append):
        self._baseline_places, self._forced_place = None, None
        if not self.selection_active():
            return super().plan_conveyor_target(start, targets, checker, validity, solver, report, append)
        began = time.perf_counter()
        start = np.asarray(start, dtype=np.float64)
        if len(self.selected_task) == 2:
            baseline = super().dual_place_candidates(start, targets, validity, solver, report)
            distances = torch.linalg.vector_norm((baseline - torch.tensor(start, device='cuda', dtype=torch.float32)) * validity.weights, dim=-1)
            baseline_row = baseline[int(distances.argmin())].cpu().numpy() if len(baseline) else None
        else:
            baseline = super().single_place_candidates(targets, start, checker)
            baseline_row = baseline[0] if len(baseline) else None
        self._baseline_places = (baseline, dict(report.get('conveyor_goal_ik') or {}))
        selected = time.perf_counter()
        attempts, record = self.place_options(start, targets, baseline_row)
        record.update(ms=(time.perf_counter() - selected) * 1000, baseline_ik_ms=(selected - began) * 1000, outcomes=[])
        self.selection_report["place"].append(record)
        for row in attempts:
            self._forced_place = row
            try:
                placed = super().plan_conveyor_target(start, targets, checker, validity, solver, report, append)
                record["outcomes"].append("success")
                return placed
            except CycleBlocked as error:
                if error.stage not in PLACE_RETRY:
                    raise
                record["outcomes"].append(error.stage)
            finally:
                self._forced_place = None
        record["outcomes"].append("baseline_rule")
        return super().plan_conveyor_target(start, targets, checker, validity, solver, report, append)

    def dual_place_candidates(self, start, targets, validity, solver, report):
        if getattr(self, "_forced_place", None) is not None:
            row = torch.tensor(self._forced_place[None], device='cuda', dtype=torch.float32)
            valid = validity.mask(row)
            report['conveyor_goal_ik'] = dict(mode='MOTION-311 selected place IK', collision_valid=int(valid.sum()))
            return row[valid].contiguous()
        if getattr(self, "_baseline_places", None) is not None:
            report['conveyor_goal_ik'] = self._baseline_places[1]
            return self._baseline_places[0]
        return super().dual_place_candidates(start, targets, validity, solver, report)

    def single_place_candidates(self, targets, start, checker):
        if getattr(self, "_forced_place", None) is not None:
            from v3_batched_loaded_search import GpuValidity
            row = np.asarray(self._forced_place, dtype=np.float32)[None]
            return row if bool(GpuValidity(checker, check_ground=True).mask(torch.tensor(row, device='cuda')).all()) else row[:0]
        if getattr(self, "_baseline_places", None) is not None:
            return self._baseline_places[0]
        return super().single_place_candidates(targets, start, checker)

    # round -----------------------------------------------------------------------------------------------
    def full_cycle(self, boxes, progress=lambda text: None):
        self.selection_report, self._selection_active, self._selection_batch = None, False, None
        self._baseline_places = self._forced_place = None
        if self.arm == "A":
            return super().full_cycle(boxes, progress)
        result = super().full_cycle(boxes, progress)
        report = self.selection_report
        if result.get("success") or report is None or report["abstained"]:
            result["selection"] = report
            return result
        arm, self.arm = self.arm, "A"
        try:
            self.configure_task(boxes)
            fallback = super().full_cycle(boxes, progress)
        finally:
            self.arm = arm
        fallback["selection"] = dict(report, fallback=dict(first_stage=result.get("blocked_stage"), first_total_ms=result.get("total_ms")))
        fallback["total_ms"] = (fallback.get("total_ms") or 0.) + (result.get("total_ms") or 0.)
        return fallback
