"""GPU-backed hierarchical candidate pools for the B2 group selector."""
from collections import Counter
import math
import time

import numpy as np
import torch

from motion306_learning.group_selector import CandidatePool, ContactPool, PlacePool
from motion311_learning.stage_env import EpisodeFailure
from v3_batched_loaded_search import GpuValidity


def physical_redundancy(rows, actions):
    """Return [Updown metres, left psi degrees, right psi degrees]."""
    rows = np.asarray(rows, dtype=np.float64).reshape(-1, 15)
    actions = np.asarray(actions, dtype=np.float64).reshape(len(rows), 3)
    return np.column_stack([rows[:, 0], actions[:, 1] * 180., actions[:, 2] * 180.])


class LookaheadGroupBackend:
    """Preview hard-gated pools without executing RRT or mutating the episode stage."""

    def __init__(self, env, edge_collision_samples=9):
        if env.done or env.stage != 0 or env.observation.stage != 0:
            raise ValueError("B2 preview requires a freshly reset stage-0 environment")
        if isinstance(edge_collision_samples, bool) or int(edge_collision_samples) != edge_collision_samples or edge_collision_samples < 2:
            raise ValueError("edge collision samples must be an integer >= 2")
        self.env = env
        self.home = np.asarray(env.home, dtype=np.float64).copy()
        self.velocity = np.asarray(env.velocity, dtype=np.float64).copy()
        self.lower = np.asarray(env.decoder.lower, dtype=np.float64).copy()
        self.upper = np.asarray(env.decoder.upper, dtype=np.float64).copy()
        self.samples = int(edge_collision_samples)
        self.original_snapshot = env.planner.snapshot
        self.timings_ms = Counter()
        self.rejections = Counter()

    def _risk(self, starts, goals, validity):
        began = time.perf_counter()
        result = self.env._direct_collision_fraction(starts, goals, validity, self.samples)
        self.timings_ms["edge_risk"] += (time.perf_counter() - began) * 1000
        return result

    def contact_pool(self):
        began = time.perf_counter()
        observation = self.env.observation
        rows = np.asarray(observation.candidates, dtype=np.float64)
        actions = np.asarray(observation.actions, dtype=np.float64)
        valid = self.env.contact_collision_free(rows)
        extraction = np.zeros_like(rows)
        self.rejections["contact_collision"] += int((~valid).sum())
        for index in np.flatnonzero(valid):
            endpoint = self.env.extraction_endpoint(rows[index])
            if endpoint is None:
                valid[index] = False
                self.rejections["extraction_analytic_no_solution"] += 1
            else:
                extraction[index] = endpoint
        risk = self._risk(self.home, rows, self.env.empty_valid) if len(rows) else np.zeros(0)
        self.timings_ms["contact_pool"] += (time.perf_counter() - began) * 1000
        return ContactPool(rows, physical_redundancy(rows, actions), risk, np.arange(len(rows)), valid, extraction)

    def validate_contact(self, contact_index, row, approximate_extraction):
        began = time.perf_counter()
        try:
            path, _ = self.env.contact_checks(row)
            endpoint = np.asarray(path[-1], dtype=np.float64)
            if not np.allclose(endpoint, approximate_extraction, atol=2e-5, rtol=0.):
                raise RuntimeError("analytic extraction preview differs from the hard-gated endpoint")
            return endpoint
        except EpisodeFailure as error:
            self.rejections[error.reason] += 1
            return None
        finally:
            self.timings_ms["contact_validation"] += (time.perf_counter() - began) * 1000

    def carry_pool(self, contact_index, extraction):
        began = time.perf_counter()
        rows, actions = self.env.decoder.candidate_set(self.env.carry_targets, extraction,
                                                       self.env.caps[1], 1)
        reasons = self.env.carry_checks(rows)
        valid = np.asarray([reason is None for reason in reasons], dtype=bool)
        self.rejections.update(reason for reason in reasons if reason is not None)
        risk = self._risk(extraction, rows, self.env.loaded_valid) if len(rows) else np.zeros(0)
        self.timings_ms["carry_pool"] += (time.perf_counter() - began) * 1000
        return CandidatePool(rows, physical_redundancy(rows, actions), risk, np.arange(len(rows)), valid)

    def place_pool(self, contact_index, carry_index, carry_row):
        began = time.perf_counter()
        targets = self.env.planner.conveyor_place_targets(np.asarray(carry_row, dtype=np.float64))[0]
        rows, actions = self.env.decoder.candidate_set(targets, carry_row, self.env.caps[2], 2)
        if not len(rows):
            valid = np.zeros(0, dtype=bool)
            place_risk = home_risk = np.zeros(0)
        else:
            parking = self.env._placement_snapshot(carry_row)
            if parking is None:
                place_risk = home_risk = np.ones(len(rows), dtype=np.float64)
                valid = np.zeros(len(rows), dtype=bool)
                self.rejections["placement_parking"] += len(rows)
            else:
                planner, original = self.env.planner, self.env.planner.snapshot
                planner.snapshot = parking
                try:
                    loaded = GpuValidity(planner.checker(self.env.boxes, payload=True), check_ground=True)
                    empty = GpuValidity(planner.checker(self.env.boxes), check_ground=True)
                    valid = loaded.mask(torch.tensor(np.asarray(rows, dtype=np.float32), device="cuda")).cpu().numpy()
                    self.rejections["place_collision"] += int((~valid).sum())
                    place_risk = self._risk(carry_row, rows, loaded)
                    home_risk = self._risk(rows, self.home, empty)
                finally:
                    planner.snapshot = original
        self.timings_ms["place_pool"] += (time.perf_counter() - began) * 1000
        return PlacePool(rows, physical_redundancy(rows, actions), place_risk, np.arange(len(rows)), valid,
                         home_risk)

    def audit(self):
        return {"timings_ms": dict(self.timings_ms), "rejections": dict(self.rejections),
                "edge_collision_samples": self.samples}

    def finalize(self):
        """Restore persistent checker geometry after conveyor-snapshot previews."""
        began = time.perf_counter()
        planner = self.env.planner
        planner.snapshot = self.original_snapshot
        self.env.empty = planner.checker(self.env.boxes)
        self.env.loaded = planner.checker(self.env.boxes, payload=True)
        self.env.empty_valid = GpuValidity(self.env.empty, check_ground=True)
        self.env.loaded_valid = GpuValidity(self.env.loaded, check_ground=True)
        self.timings_ms["restore_execution_scene"] += (time.perf_counter() - began) * 1000
