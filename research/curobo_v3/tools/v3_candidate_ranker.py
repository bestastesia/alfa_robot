"""MOTION-311 candidate ranker: cost-tiered features, a numpy MLP ensemble and the attempt policy.

S1 (contact candidates of one round) features
  T0  no kinematics: candidate joints, weighted Home distance (with in-set rank and z-score), swivel angle per
      arm, branch signs sign(q2, q4, q6), singularity and joint-limit margins, the velocity-limited time from
      Home, box centres in the planner frame and task flags
  T1  + the 35 cm extraction endpoint (analytic IK of the task contact pose shifted 35 cm, closed-form
      carriage) and its weighted distance to the nearest raw placement-atlas goals (k-d tree built once)
S2 (placement goals of one root) features
  root and goal joints, weighted difference and its norm, goal-to-Home distance, distance rank in the pool
Inference is numpy on the CPU from a sha256-checked .npz ensemble: no CUDA kernels, no random numbers.
"""
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

WEIGHTS = np.array([math.radians(15) / .1] + [1.] * 14)
ACTIVE_JOINTS = ["updown", *[f"left_joint{i}" for i in range(1, 8)], *[f"right_joint{i}" for i in range(1, 8)]]
ARMS = {"left": slice(1, 8), "right": slice(8, 15)}
SINGULAR = (1, 3, 5)


class Kinematics:
    """Joint limits and velocities from the URDF; the analytic bridge and carriage model are optional (T1)."""

    atlas_cap = 4096  # T1 distance features use a fixed subsample of larger atlases (top: 44032 rows) to stay in budget

    def __init__(self, urdf, analytic=None, carriage=None, atlases=None):
        joints = {joint.attrib["name"]: joint for joint in ET.parse(urdf).getroot().findall("joint")}
        limits = [joints[name].find("limit").attrib for name in ACTIVE_JOINTS]
        self.lower = np.array([float(item["lower"]) for item in limits])
        self.upper = np.array([float(item["upper"]) for item in limits])
        self.velocity = np.array([float(item["velocity"]) for item in limits])
        self.analytic, self.carriage = analytic, carriage
        self.trees = {}
        for mode, joints_array in (atlases or {}).items():
            from scipy.spatial import cKDTree
            if len(joints_array) > self.atlas_cap:
                joints_array = joints_array[np.sort(np.random.default_rng(311).choice(len(joints_array), self.atlas_cap, replace=False))]
            self.trees[mode] = dict(full=cKDTree(joints_array * WEIGHTS),
                                    **{side: cKDTree(joints_array[:, [0, *range(ARMS[side].start, ARMS[side].stop)]] *
                                                     WEIGHTS[[0, *range(ARMS[side].start, ARMS[side].stop)]]) for side in ARMS})


def swivels(kinematics, contacts):
    if kinematics.analytic is None:
        return np.zeros((len(contacts), 2))
    return np.array([[kinematics.analytic.swivel(index, row[ARMS[side]]) for index, side in enumerate(ARMS)] for row in contacts])


def s1_t0(contacts, context, kinematics):
    """(N, F) T0 features; context: kind, sides (active arms), top_grasp, box_centers {side: xyz}, home (15)."""
    contacts = np.asarray(contacts, dtype=np.float64)
    home = np.asarray(context["home"], dtype=np.float64)
    delta = (contacts - home) * WEIGHTS
    per_arm = np.stack([np.linalg.norm(delta[:, ARMS[side]], axis=1) for side in ARMS], axis=1)
    total = np.linalg.norm(delta, axis=1)
    rank = np.argsort(np.argsort(total, kind="stable"), kind="stable") / max(len(total) - 1, 1)
    z = (total - total.mean()) / (total.std() + 1e-9)
    psi = swivels(kinematics, contacts)
    signs = np.concatenate([np.sign(contacts[:, ARMS[side]][:, SINGULAR]) for side in ARMS], axis=1)
    singular = np.concatenate([np.abs(contacts[:, ARMS[side]][:, SINGULAR]) for side in ARMS], axis=1)
    margin = np.minimum(contacts - kinematics.lower, kinematics.upper - contacts)
    margins = np.stack([margin[:, 0], *[margin[:, ARMS[side]].min(axis=1) for side in ARMS]], axis=1)
    seconds = np.abs(contacts - home) / kinematics.velocity
    reach = np.stack([seconds.max(axis=1), *[seconds[:, ARMS[side]].max(axis=1) for side in ARMS]], axis=1)
    flags = np.array([float(context["top_grasp"]), float(context["kind"] == "dual"),
                      float("left" in context["sides"]), float("right" in context["sides"])])
    boxes = np.concatenate([np.asarray(context["box_centers"].get(side, (0., 0., 0.)), dtype=np.float64) for side in ARMS])
    return np.concatenate([contacts, per_arm, total[:, None], rank[:, None], z[:, None], np.sin(psi), np.cos(psi), signs,
                           singular, margins, reach, np.repeat(flags[None], len(contacts), 0),
                           np.repeat(boxes[None], len(contacts), 0)], axis=1)


def endpoints(contacts, context, kinematics):
    """35 cm extraction endpoints with fixed updown and swivel (NaN rows where the analytic chain has no solution)."""
    contacts = np.asarray(contacts, dtype=np.float64)
    output = contacts.copy()
    carriage = kinematics.carriage.inverse(contacts[:, 0])
    for side_index, side in enumerate(ARMS):
        if side not in context["sides"]:
            continue
        pose = context["contact_poses"][side + "_tool0"]
        target = np.eye(4)
        target[:3, :3] = quaternion_matrix(pose["quaternion"])
        target[:3, 3] = pose["position"]
        target[0, 3] -= .35
        for row, contact in enumerate(contacts):
            seed = contact[ARMS[side]]
            solutions = kinematics.analytic.solve(side_index, carriage[row] @ target, kinematics.analytic.swivel(side_index, seed), seed)
            output[row, ARMS[side]] = (solutions[np.argmin(np.sum((solutions - seed) ** 2, axis=1))] if len(solutions) else np.nan)
    return output


def quaternion_matrix(wxyz):
    w, x, y, z = wxyz
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def s1_t1(contacts, context, kinematics, ends=None):
    """(N, F) T1 extras: endpoint delta, endpoint existence, nearest raw-atlas goal distances, endpoint-to-Home."""
    contacts = np.asarray(contacts, dtype=np.float64)
    ends = endpoints(contacts, context, kinematics) if ends is None else np.asarray(ends, dtype=np.float64)
    exists = np.isfinite(ends).all(axis=1)
    filled = np.where(np.isfinite(ends), ends, contacts)
    mode = "top" if context["top_grasp"] else "side"
    trees = kinematics.trees[mode]
    if context["kind"] == "dual":
        distances, _ = trees["full"].query(filled * WEIGHTS, k=8)
    else:
        side = context["sides"][0]
        columns = [0, *range(ARMS[side].start, ARMS[side].stop)]
        distances, _ = trees[side].query(filled[:, columns] * WEIGHTS[columns], k=8)
    home = np.asarray(context["home"], dtype=np.float64)
    return np.concatenate([(filled - contacts) * WEIGHTS, exists[:, None].astype(float), distances[:, :1], distances.mean(axis=1, keepdims=True),
                           np.linalg.norm((filled - home) * WEIGHTS, axis=1, keepdims=True)], axis=1), exists


def s2_t0(root, pool, context):
    """(P, F) features of the goals of one root."""
    root, pool = np.asarray(root, dtype=np.float64), np.asarray(pool, dtype=np.float64)
    home = np.asarray(context["home"], dtype=np.float64)
    delta = (pool - root) * WEIGHTS
    norm = np.linalg.norm(delta, axis=1)
    rank = np.argsort(np.argsort(norm, kind="stable"), kind="stable") / max(len(norm) - 1, 1)
    flags = np.array([float(context["top_grasp"]), float(context["kind"] == "dual"),
                      float("left" in context["sides"]), float("right" in context["sides"])])
    return np.concatenate([np.repeat(root[None], len(pool), 0), pool, delta, norm[:, None], rank[:, None],
                           np.linalg.norm((pool - home) * WEIGHTS, axis=1, keepdims=True), np.repeat(flags[None], len(pool), 0)], axis=1)


def silu(values):
    return values / (1. + np.exp(-values))


class Ensemble:
    """Members share standardisation; each is Linear-SiLU-...-Linear with 2 outputs (success logit, log cost)."""

    def __init__(self, arrays, meta):
        self.meta = meta
        self.mean, self.scale = arrays["mean"], arrays["scale"]
        self.members = []
        for member in range(meta["members"]):
            layers, depth = [], 0
            while f"m{member}_w{depth}" in arrays:
                layers.append((arrays[f"m{member}_w{depth}"], arrays[f"m{member}_b{depth}"]))
                depth += 1
            self.members.append(layers)

    @classmethod
    def load(cls, path, sha256=None, feature_names=None):
        data = Path(path).read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        if sha256 is not None and digest != sha256:
            raise ValueError(f"model {path} sha256 {digest} does not match the frozen {sha256}")
        arrays = dict(np.load(path, allow_pickle=False))
        meta = json.loads(str(arrays.pop("meta")))
        if feature_names is not None and meta["feature_hash"] != feature_hash(feature_names):
            raise ValueError("model was trained on a different feature list")
        ensemble = cls(arrays, meta)
        ensemble.sha256 = digest
        return ensemble

    def standardise(self, features):
        return (np.asarray(features, dtype=np.float64) - self.mean) / self.scale

    def predict(self, features):
        """(probability of success, predicted cost) averaged over members; also returns the max |z| per row (before the
        optional input clipping). The mean logit passes the optional Platt calibration (a, b) of the model."""
        x = self.standardise(features)
        max_z = np.abs(x).max(axis=1)
        clip = (self.meta.get("config") or {}).get("clip")
        if clip:
            x = np.clip(x, -clip, clip)
        logits, costs = [], []
        for layers in self.members:
            hidden = x
            for depth, (weight, bias) in enumerate(layers):
                hidden = hidden @ weight + bias
                if depth < len(layers) - 1:
                    hidden = silu(hidden)
            logits.append(hidden[:, 0])
            costs.append(hidden[:, 1])
        slope, offset = self.meta.get("calibration") or (1., 0.)
        probability = 1. / (1. + np.exp(-(slope * np.mean(logits, axis=0) + offset)))
        return probability, np.exp(np.mean(costs, axis=0)), max_z


def feature_hash(names):
    return hashlib.sha256(json.dumps(list(names)).encode()).hexdigest()


def attempt_order(probability, cost, eligible=None, theta=.5, max_z=None, z_limit=6.):
    """Recommended candidates (P >= theta) by ascending expected cost S = P*J + (1-P)*max(J);
    None means abstain (use the baseline order): no candidate reaches theta, or one is out of distribution."""
    probability, cost = np.asarray(probability, dtype=np.float64), np.asarray(cost, dtype=np.float64)
    eligible = np.ones(len(cost), dtype=bool) if eligible is None else np.asarray(eligible, dtype=bool)
    if max_z is not None and eligible.any() and np.max(np.asarray(max_z)[eligible]) > z_limit:
        return None
    eligible = eligible & (probability >= theta)
    if not eligible.any():
        return None
    score = probability * cost + (1. - probability) * cost[eligible].max()
    score = np.where(eligible, score, np.inf)
    return [int(index) for index in np.argsort(score, kind="stable") if eligible[index]]


S1_T0_NAMES = ([f"q_{name}" for name in ACTIVE_JOINTS] + ["home_dist_left", "home_dist_right", "home_dist", "home_rank", "home_z"]
               + [f"sin_psi_{side}" for side in ARMS] + [f"cos_psi_{side}" for side in ARMS]
               + [f"sign_{side}_q{index + 1}" for side in ARMS for index in SINGULAR]
               + [f"abs_{side}_q{index + 1}" for side in ARMS for index in SINGULAR]
               + ["margin_updown", "margin_left", "margin_right", "reach_s", "reach_left_s", "reach_right_s",
                  "top_grasp", "dual", "left_active", "right_active"]
               + [f"box_{side}_{axis}" for side in ARMS for axis in "xyz"])
S1_T1_NAMES = ([f"end_delta_{name}" for name in ACTIVE_JOINTS] + ["end_exists", "atlas_d1", "atlas_d8_mean", "end_home_dist"])
S2_T0_NAMES = ([f"root_{name}" for name in ACTIVE_JOINTS] + [f"goal_{name}" for name in ACTIVE_JOINTS]
               + [f"delta_{name}" for name in ACTIVE_JOINTS] + ["goal_dist", "goal_rank", "goal_home_dist",
                                                               "top_grasp", "dual", "left_active", "right_active"])


class UrdfFk:
    """Minimal planner-like FK (yourdfpy) for building the carriage model outside the planner."""

    def __init__(self, urdf):
        import yourdfpy
        self.fk_robot = yourdfpy.URDF.load(urdf, load_meshes=False, build_scene_graph=True)

    def fk(self, values):
        mapping = dict(zip(ACTIVE_JOINTS, values))
        self.fk_robot.update_cfg({name: mapping.get(name, 0.0) for name in self.fk_robot.actuated_joint_names})


class CarriageModel:
    """arm_carriage pose as a closed-form translation of updown, checked against URDF FK (from MOTION-323 c26e49f)."""

    def __init__(self, planner, reference):
        row = np.asarray(reference, dtype=np.float64).copy()

        def carriage(updown):
            row[0] = updown
            planner.fk(row)
            return np.asarray(planner.fk_robot.get_transform("arm_carriage", planner.fk_robot.base_link), dtype=np.float64)

        zero, low = carriage(0.), carriage(-.5)
        self.rotation, self.origin = zero[:3, :3], zero[:3, 3]
        self.axis = (low[:3, 3] - zero[:3, 3]) / -.5
        for updown in (-1., -.3, -.05):
            if np.max(np.abs(carriage(updown) - self.matrix(np.array([updown]))[0])) > 1e-9:
                raise RuntimeError("arm_carriage is not a pure translation of updown")

    def matrix(self, updown):
        output = np.repeat(np.eye(4)[None], len(updown), axis=0)
        output[:, :3, :3] = self.rotation
        output[:, :3, 3] = self.origin + np.asarray(updown)[:, None] * self.axis
        return output

    def inverse(self, updown):
        output = np.repeat(np.eye(4)[None], len(updown), axis=0)
        output[:, :3, :3] = self.rotation.T
        output[:, :3, 3] = -(self.origin + np.asarray(updown)[:, None] * self.axis) @ self.rotation
        return output
