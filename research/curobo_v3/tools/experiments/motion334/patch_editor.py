#!/usr/bin/env python3
"""MOTION-334: interactive collision-patch editor in viser.

Starts from a patch on top of the frozen model (spheres appended per link, cuboids appended to the base
cuboid layer). Every element can be kept or dropped, moved or resized, and new spheres (link7, link6,
link5, carriage) or base cuboids can be placed with a transform gizmo. For the current selection the
editor shows, on the frames cached by editor_cache.py (every 2nd frame of the audited cycles):
  detection         run-phases with a mesh contact (phase_audit.py records; transport: the MOTION-326
                    true-depth records, deepest frame +-1) that the patch rejects inside the contact
                    range, per contact class, baseline and Cartesian apart;
  false rejections  rejected run-phases without any mesh contact (patch_detect.py's mesh-clean);
  planning time     per box, summed over the kept elements with a cost per element group (--costs).
Per element it shows what only that element detects and which false rejections only it causes, so
that redundant elements can be dropped; "精简" drops elements greedily (most costly first) while
every detection stays. Undetected contacts and false rejections can be opened at their frame.

A frame is rejected as by the CPU replica (experiments/motion326/cpu_spheres.py) plus the task-box
check of approach (target_agree.py): a kept sphere overlaps a world cuboid of the attach snapshot
(ground excluded), the ground (base_link exempt), a frozen base cuboid (base_link and arm_carriage
exempt), the task box in approach (axis-aligned), a frozen sphere of a link the planner checks it
against, the payload spheres in transport and conveyor_place (as link7), or a kept sphere on such a
link; or a kept base cuboid overlaps a sphere of a non-exempt link (payload included). Hits are
cached next to --state. The selection is exported as patch.json, cuboids.json and patched robot
configs (frozen + selection) for the exact GPU check and A/B runs.

  python patch_editor.py --cache DIR --audit A.jsonl ... --transport-depth TD.jsonl --frozen MOBILE18_YML \
      --frozen-cuboids FROZEN.json --patch patch.json --patched-cuboids cuboids.json --costs costs.json \
      --state state.json --export DIR --run-dir LABEL DIR ... --pose LABEL RUN_JSON PHASE INDEX NOTE ...
  ... --validate GPU.jsonl CPU.jsonl   # every element kept vs the patched checks, frame by frame, then exit
"""
import argparse
import collections
import datetime
import functools
import hashlib
import json
from pathlib import Path
import sys
import threading
import time
import zlib

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import viser
from viser.extras import ViserUrdf
import yaml
import yourdfpy

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from experiments.motion326.cpu_spheres import BASE_CUBOID_EXEMPT, planner_ignored
from experiments.motion334.model_viewer import ViewPose, cuboids, wxyz
from experiments.motion334.patch_detect import EXEMPT, contact_class, expand, load_checked

PHASES = ("approach", "contact_approach", "extract", "transport", "conveyor_place", "return_home")
APPROACH, TRANSPORT, CONVEYOR_PLACE = (PHASES.index(phase) for phase in ("approach", "transport", "conveyor_place"))
THRESHOLDS_MM = (0., 2.)
GROUPS = ("基线", "笛卡尔")
CLASSES = {"target": "approach 作业臂↔待抓箱", "base": "approach 闲置臂↔底座", "world": "approach 作业臂↔填充箱/前墙",
           "return": "return_home link7↔输送线", "transport-base": "transport 底座↔臂/箱", "transport-carriage": "transport 升降架↔箱/臂"}
COST_GROUPS = {"link7_sphere": "link7 球", "carriage_sphere": "升降架球", "arm_sphere": "link5/6 球", "base_cuboid": "底座长方体"}
SPHERE_LINKS = ("right_link7", "left_link7", "arm_carriage", "right_link6", "left_link6", "right_link5", "left_link5")
CUBOID = "底座长方体"
UNSELECTED = "（未选）"
COLORINGS = ("按作用（红：有独占检出；黄：检出都可被替代；灰：没检出）", "按来源（橙：自动；绿：手动/改过）")
VIEWS = {"整机": ((-2.2, 1.8, 1.0), "base_link"), "右臂 link7": ((0.15, -0.75, 0.35), "right_link7"),
         "左臂 link7": ((0.15, 0.75, 0.35), "left_link7"), "底座": ((-1.6, 1.3, 0.9), "base_link"),
         "升降架": ((-1.2, -0.9, 0.6), "arm_carriage")}
CHUNK = 200_000
EMPTY = np.zeros(0, np.int64)


def box_distance(points, rotation, center, half):
    """Signed distance of points (..., 3) to one box (negative inside), as cpu_spheres.box_distance."""
    local = np.abs((points - center) @ rotation) - half
    return np.linalg.norm(np.maximum(local, 0.), axis=-1) + np.minimum(local.max(axis=-1), 0.)


def editor_class(phase, pair, cls):
    """The editor's class of a mesh contact, or None when no robot sphere or base cuboid can catch it."""
    if phase == "approach":
        return "target" if cls == "robot-target" else "base" if "robot:base_link" in pair else "world" if cls == "robot-world" else None
    if phase == "return_home":
        return "return" if cls == "robot-world" else None
    if phase == "transport":
        return "transport-base" if "robot:base_link" in pair else "transport-carriage" if "arm_carriage" in pair else None
    return None


class Frames:
    """The cached frames, what each run's frames are checked against, and the mesh contacts on them."""

    def __init__(self, caches, sample):
        paths = sorted(path for cache in caches for path in cache.glob("*/*.json"))
        if sample > 1:
            paths = [path for path in paths if zlib.crc32(f"{path.parent.name}/{path.stem}".encode()) % sample == 0]
        self.meta = [json.loads(path.read_text()) for path in paths]
        counts = np.asarray([meta["frames"] for meta in self.meta])
        self.links = self.meta[0]["links"]
        self.link_index = {link: index for index, link in enumerate(self.links)}
        self.base_index = self.link_index["base_link"]
        self.count = int(counts.sum())
        self.starts = np.concatenate([[0], np.cumsum(counts)])
        self.poses = np.empty((self.count, len(self.links), 3, 4), np.float32)
        self.phase = np.empty(self.count, np.int8)
        self.local = np.empty(self.count, np.int32)
        for index, path in enumerate(paths):
            data = np.load(path.with_suffix(".npz"))
            rows = slice(self.starts[index], self.starts[index + 1])
            self.poses[rows], self.phase[rows], self.local[rows] = data["poses"], data["phase"], data["local"]
        self.run = np.repeat(np.arange(len(paths), dtype=np.int64), counts)
        self.rp = self.run * len(PHASES) + self.phase  # sorted: runs in order, phases in PHASES order within a run
        codes = np.arange(len(self.meta) * len(PHASES))
        self.bounds = np.stack([np.searchsorted(self.rp, codes), np.searchsorted(self.rp, codes, side="right")], axis=1)
        self.index = {(meta["set"], meta["run"]): index for index, meta in enumerate(self.meta)}
        self.group = np.asarray([0 if meta["arm"] == "joint_rrt" else 1 for meta in self.meta])
        self.world, self.targets, self.payload = [], [], []
        for meta in self.meta:
            poses = np.asarray([pose for _, pose, _ in meta["world"]], np.float32).reshape(-1, 4, 4)
            self.world.append((poses[:, :3, :3], poses[:, :3, 3], np.asarray([half for _, _, half in meta["world"]], np.float32).reshape(-1, 3)))
            self.targets.append([np.asarray(task, np.float32) for task in meta["tasks"]])
            groups = []
            for link, centers, radii in meta["payload"]:
                centers, radii = np.asarray(centers, np.float32), np.asarray(radii, np.float32)
                centroid = centers.mean(axis=0)
                groups.append((self.link_index[link], centers, radii, centroid, float(np.max(np.linalg.norm(centers - centroid, axis=1) + radii))))
            self.payload.append(groups)
        self.ground = np.asarray([meta["ground_z"] for meta in self.meta], np.float32)

    def segment(self, index, code):
        return self.bounds[index * len(PHASES) + code]

    def loaded(self, index):
        """Rows of the payload phases (transport, conveyor_place: consecutive, so one span)."""
        return self.segment(index, TRANSPORT)[0], self.segment(index, CONVEYOR_PLACE)[1]

    def points(self, link, centers, rows=slice(None)):
        pose = self.poses[rows, self.link_index[link]]
        return np.einsum("nij,kj->nki", pose[:, :, :3], centers) + pose[:, None, :, 3]

    def to_base(self, points, rows=slice(None)):
        base = self.poses[rows, self.base_index]
        if points.ndim == 2:
            return np.einsum("nji,nj->ni", base[:, :, :3], points - base[:, :, 3])
        return np.einsum("nji,nkj->nki", base[:, :, :3], points - base[:, None, :, 3])

    def name(self, rp):
        meta = self.meta[rp // len(PHASES)]
        return f"{meta['set']}/{meta['run']}"

    def load_contacts(self, audits, transport_depth):
        found = {}
        for path in audits:  # a later file replaces the phases it covers
            for record in map(json.loads, path.read_text().splitlines()):
                index = self.index.get((record["set"], record["run"]))
                if index is not None:
                    for phase, data in record["phases"].items():
                        found[(index, phase)] = [(pair, value["cls"], value["true_depth_mm"], value["first"], value["last"],
                                                  value.get("true_frame", value["first"]), data["frames"])
                                                 for pair, value in data["pairs"].items()]
        for record in map(json.loads, transport_depth.read_text().splitlines()):
            index = self.index.get((record["set"], record["run"]))
            if index is not None:
                found[(index, "transport")] = [(pair, value["cls"], value["true_depth_mm"], value["frame"] - 1, value["frame"] + 1,
                                                value["frame"], None) for pair, value in record["pairs"].items()]
        self.touched, self.in_contact = set(), np.zeros(self.count, bool)
        self.masks = {}                              # (class, threshold) -> frames inside such a contact
        self.totals = collections.defaultdict(dict)  # (class, group, threshold) -> rp -> (depth mm, deepest frame)
        for (index, phase), contacts in found.items():
            rp = index * len(PHASES) + PHASES.index(phase)
            start, end = self.bounds[rp]
            local = self.local[start:end]
            for pair, cls, depth, first, last, deepest, frames in contacts:
                if depth <= 0. or end == start or contact_class(phase, pair, cls, last, frames) in EXEMPT:
                    continue
                inside = np.flatnonzero((local >= first) & (local <= last))
                if not len(inside):  # a short contact between two cached frames
                    inside = np.flatnonzero((local >= first - 1) & (local <= last + 1))
                rows = start + inside
                self.touched.add(rp)
                self.in_contact[rows] = True
                key = editor_class(phase, pair, cls)
                for threshold in THRESHOLDS_MM:
                    if key is not None and depth > threshold:
                        self.masks.setdefault((key, threshold), np.zeros(self.count, bool))[rows] = True
                        slot = self.totals[(key, int(self.group[index]), threshold)]
                        if rp not in slot or depth > slot[rp][0]:
                            slot[rp] = (depth, deepest)


class Model:
    """The frozen collision model as the planner checks it: spheres per link, ignored link pairs, base cuboids."""

    def __init__(self, config, frozen_cuboids):
        self.kinematics = yaml.safe_load(Path(config).read_text())["kinematics"]
        self.ignored = planner_ignored(self.kinematics)
        self.spheres, self.centroid, self.reach = {}, {}, {}
        for link, balls in self.kinematics["collision_spheres"].items():
            kept = [ball for ball in balls if ball["radius"] > 0]
            if kept:
                centers = np.asarray([ball["center"] for ball in kept], np.float32)
                radii = np.asarray([ball["radius"] for ball in kept], np.float32)
                self.spheres[link], self.centroid[link] = (centers, radii), centers.mean(axis=0)
                self.reach[link] = float(np.max(np.linalg.norm(centers - self.centroid[link], axis=1) + radii))
        self.base_cuboids = [(pose[:3, :3].astype(np.float32), pose[:3, 3].astype(np.float32), (dimensions / 2).astype(np.float32))
                             for _, pose, dimensions in cuboids(frozen_cuboids)]

    def checked(self, a, b):
        return a != b and frozenset((a, b)) not in self.ignored


class Element:
    """An appended sphere (center in its link's frame) or base cuboid (pose in the base_link frame)."""

    def __init__(self, id, kind, origin, link="base_link", center=None, radius=None, pose=None, half=None, kept=True, cost_group=None):
        self.id, self.kind, self.origin, self.link, self.kept = id, kind, origin, link, kept
        self.center = None if center is None else np.asarray(center, np.float32)
        self.radius = None if radius is None else float(radius)
        self.pose = None if pose is None else np.asarray(pose, np.float64)
        self.half = None if half is None else np.asarray(half, np.float32)
        self.cost_group = cost_group or ("base_cuboid" if kind == "cuboid" else "carriage_sphere" if link == "arm_carriage"
                                         else "link7_sphere" if link.endswith("link7") else "arm_sphere")

    def to_dict(self):
        return dict(id=self.id, kind=self.kind, origin=self.origin, link=self.link, kept=self.kept, cost_group=self.cost_group,
                    center=None if self.center is None else [round(float(v), 6) for v in self.center],
                    radius=None if self.radius is None else round(self.radius, 6),
                    pose=None if self.pose is None else self.pose.round(6).tolist(),
                    half=None if self.half is None else [round(float(v), 6) for v in self.half])

    def digest(self):
        geometry = {key: value for key, value in self.to_dict().items() if key in ("kind", "link", "center", "radius", "pose", "half")}
        return hashlib.sha1(json.dumps(geometry, sort_keys=True).encode()).hexdigest()[:20]

    def describe(self):
        if self.kind == "sphere":
            return f"半径 {self.radius * 1000:.1f} mm，中心 (" + ", ".join(f"{v * 1000:.0f}" for v in self.center) + f") mm（{self.link} 系）"
        angles = Rotation.from_matrix(self.pose[:3, :3]).as_euler("xyz", degrees=True)
        return ("尺寸 " + "×".join(f"{v * 2000:.0f}" for v in self.half) + " mm，中心 (" + ", ".join(f"{v * 1000:.0f}" for v in self.pose[:3, 3])
                + ") mm（base_link 系）" + ("，转角 " + "/".join(f"{v:.0f}" for v in angles) + "°" if np.abs(angles).max() > 0.5 else ""))


class Detector:
    """Frames one element (or a pair of elements) rejects, with the contacts and run-phases they fall in."""

    def __init__(self, frames, detected, rejected):
        self.frames, self.detected, self.rejected = frames, detected, rejected


class Engine:
    def __init__(self, frames, model, memo, signature):
        self.f, self.m = frames, model
        self.elements, self.static, self.partners = {}, {}, collections.defaultdict(dict)
        self.memo_path, self.signature, self.memo, self.dirty = memo, signature, {}, False
        if memo.exists():
            stored = np.load(memo)
            if str(stored["signature"]) == signature:
                self.memo = {key: stored[key] for key in stored.files if key != "signature"}
        self.world_bounds, self.base_bounds = {}, {}

    def save(self):
        if self.dirty:
            np.savez(self.memo_path, signature=np.asarray(self.signature), **self.memo)
            self.dirty = False

    # --- hits ---------------------------------------------------------------------------------------
    def world_bound(self, link):
        """World position of the link's frozen-sphere centroid per frame."""
        if link not in self.world_bounds:
            self.world_bounds[link] = self.f.points(link, self.m.centroid[link][None])[:, 0]
        return self.world_bounds[link]

    def base_bound(self, link):
        if link not in self.base_bounds:
            self.base_bounds[link] = self.f.to_base(self.world_bound(link))
        return self.base_bounds[link]

    def mates(self, others, exempt=()):
        out = collections.defaultdict(list)
        for other in others:
            if other.kind == "sphere" and other.link not in exempt:
                out[other.link].append(other)
        return out

    def reach(self, link, mates):
        """Radius around the link's centroid that holds its frozen spheres and the given elements on it."""
        return max([self.m.reach[link]] + [float(np.linalg.norm(o.center - self.m.centroid[link])) + o.radius for o in mates])

    def sphere_hits(self, element, others):
        f, m = self.f, self.m
        p = f.points(element.link, element.center[None])[:, 0]
        r = np.float32(element.radius)
        hit, pairs = np.zeros(f.count, bool), collections.defaultdict(list)
        for index in range(len(f.meta)):  # world cuboids, ground, task box, payload: per run
            start, end = f.starts[index], f.starts[index + 1]
            q = p[start:end]
            rotation, center, half = f.world[index]
            if len(center):
                local = np.abs(np.einsum("kji,nkj->nki", rotation, q[:, None, :] - center[None])) - half[None]
                distance = np.linalg.norm(np.maximum(local, 0.), axis=-1) + np.minimum(local.max(axis=-1), 0.)
                hit[start:end] |= (r - distance > 0).any(axis=1)
            if element.link != "base_link":
                hit[start:end] |= f.ground[index] - (q[:, 2] - r) > 0
            a0, a1 = f.segment(index, APPROACH)
            for target in f.targets[index] if a1 > a0 else ():
                delta = np.abs(p[a0:a1] - target[:3]) - target[3:]
                hit[a0:a1] |= r - (np.linalg.norm(np.maximum(delta, 0.), axis=-1) + np.minimum(delta.max(axis=-1), 0.)) > 0
            l0, l1 = f.loaded(index)
            for link_index, centers, radii, centroid, bound in f.payload[index] if l1 > l0 else ():
                if not m.checked(element.link, f.links[link_index]):
                    continue
                pose = f.poses[l0:l1, link_index]
                near = np.flatnonzero(np.linalg.norm(pose[:, :, :3] @ centroid + pose[:, :, 3] - p[l0:l1], axis=1) < bound + r)
                if len(near):
                    x = np.einsum("nij,kj->nki", pose[near, :, :3], centers) + pose[near, None, :, 3]
                    hit[l0 + near[(r + radii[None] - np.linalg.norm(x - p[l0 + near, None], axis=-1) > 0).any(axis=1)]] = True
        if element.link not in BASE_CUBOID_EXEMPT:
            q = f.to_base(p)
            for rotation, center, half in m.base_cuboids:
                hit |= r - box_distance(q, rotation, center, half) > 0
            for other in others:
                if other.kind == "cuboid":
                    rows = np.flatnonzero(r - box_distance(q, other.pose[:3, :3].astype(np.float32), other.pose[:3, 3].astype(np.float32), other.half) > 0)
                    if len(rows):
                        pairs[other.id].append(rows)
        mates = self.mates(others)
        for link in f.links:
            frozen, group = m.spheres.get(link), mates.get(link, [])
            if not m.checked(element.link, link) or (frozen is None and not group):
                continue
            near = np.flatnonzero(np.linalg.norm(self.world_bound(link) - p, axis=1) < self.reach(link, group) + r)
            for rows in np.array_split(near, max(1, len(near) // CHUNK)):
                if not len(rows):
                    continue
                pose = f.poses[rows, f.link_index[link]]
                if frozen is not None:
                    x = np.einsum("nij,kj->nki", pose[:, :, :3], frozen[0]) + pose[:, None, :, 3]
                    hit[rows[(r + frozen[1][None] - np.linalg.norm(x - p[rows, None], axis=-1) > 0).any(axis=1)]] = True
                if group:
                    centers, radii = np.stack([o.center for o in group]), np.asarray([o.radius for o in group], np.float32)
                    x = np.einsum("nij,kj->nki", pose[:, :, :3], centers) + pose[:, None, :, 3]
                    overlap = r + radii[None] - np.linalg.norm(x - p[rows, None], axis=-1) > 0
                    for column, other in enumerate(group):
                        if overlap[:, column].any():
                            pairs[other.id].append(rows[overlap[:, column]])
        return np.flatnonzero(hit), {key: np.unique(np.concatenate(value)) for key, value in pairs.items()}

    def cuboid_hits(self, element, others):
        f, m = self.f, self.m
        rotation, center, half = element.pose[:3, :3].astype(np.float32), element.pose[:3, 3].astype(np.float32), element.half
        extent = float(np.linalg.norm(half))
        hit, pairs = np.zeros(f.count, bool), collections.defaultdict(list)
        mates = self.mates(others, BASE_CUBOID_EXEMPT)
        for link in f.links:
            frozen, group = m.spheres.get(link), mates.get(link, [])
            if link in BASE_CUBOID_EXEMPT or (frozen is None and not group):
                continue
            near = np.flatnonzero(np.linalg.norm(self.base_bound(link) - center, axis=1) < self.reach(link, group) + extent)
            for rows in np.array_split(near, max(1, len(near) // CHUNK)):
                if not len(rows):
                    continue
                if frozen is not None:
                    q = f.to_base(f.points(link, frozen[0], rows), rows)
                    hit[rows[(frozen[1][None] - box_distance(q, rotation, center, half) > 0).any(axis=1)]] = True
                if group:
                    q = f.to_base(f.points(link, np.stack([o.center for o in group]), rows), rows)
                    overlap = np.asarray([o.radius for o in group], np.float32)[None] - box_distance(q, rotation, center, half) > 0
                    for column, other in enumerate(group):
                        if overlap[:, column].any():
                            pairs[other.id].append(rows[overlap[:, column]])
        for index in range(len(f.meta)):  # payload spheres count as link7, which is not exempt
            l0, l1 = f.loaded(index)
            for link_index, centers, radii, centroid, bound in f.payload[index] if l1 > l0 else ():
                pose = f.poses[l0:l1, link_index]
                middle = f.to_base(pose[:, :, :3] @ centroid + pose[:, :, 3], slice(l0, l1))
                near = l0 + np.flatnonzero(np.linalg.norm(middle - center, axis=1) < bound + extent)
                if len(near):
                    q = f.to_base(f.points(f.links[link_index], centers, near), near)
                    hit[near[(radii[None] - box_distance(q, rotation, center, half) > 0).any(axis=1)]] = True
        return np.flatnonzero(hit), {key: np.unique(np.concatenate(value)) for key, value in pairs.items()}

    def detector(self, rows):
        f, detected = self.f, set()
        for (key, threshold), mask in f.masks.items():
            for rp in np.unique(f.rp[rows[mask[rows]]]).tolist():
                detected.add((key, int(f.group[rp // len(PHASES)]), threshold, rp))
        return Detector(rows, frozenset(detected), frozenset(np.unique(f.rp[rows]).tolist()))

    # --- elements -------------------------------------------------------------------------------------
    def add(self, element):
        """Adds an element and evaluates it, with its pairs against the elements already there."""
        others = list(self.elements.values())
        self.elements[element.id] = element
        digest = element.digest()
        keys = {other.id: "p:" + ":".join(sorted((digest, other.digest()))) for other in others}
        if "s:" + digest in self.memo and all(key in self.memo for key in keys.values()):
            static, pairs = self.memo["s:" + digest], {other: self.memo[key] for other, key in keys.items()}
        else:
            static, pairs = (self.sphere_hits if element.kind == "sphere" else self.cuboid_hits)(element, others)
            self.memo["s:" + digest] = static
            for other, key in keys.items():
                self.memo[key] = pairs.get(other, EMPTY)
            self.dirty = True
        self.static[element.id] = self.detector(static)
        for other, rows in pairs.items():
            if len(rows):
                self.partners[element.id][other] = self.partners[other][element.id] = self.detector(rows)

    def remove(self, element_id):
        self.elements.pop(element_id)
        self.static.pop(element_id)
        for other in self.partners.pop(element_id, {}):
            self.partners[other].pop(element_id, None)

    def restore(self, items):
        wanted = [Element(**item) for item in items]
        digests = {element.id: element.digest() for element in wanted}
        for element_id in list(self.elements):
            if digests.get(element_id) != self.elements[element_id].digest():
                self.remove(element_id)
        for element in wanted:
            if element.id in self.elements:
                current = self.elements[element.id]
                current.kept, current.origin = element.kept, element.origin
            else:
                self.add(element)

    def state(self):
        return [element.to_dict() for element in self.elements.values()]

    # --- selections -----------------------------------------------------------------------------------
    def kept(self):
        return frozenset(element.id for element in self.elements.values() if element.kept)

    def involved(self, element_id, selection):
        return [self.static[element_id]] + [detector for other, detector in self.partners[element_id].items() if other in selection]

    def active(self, selection):
        out = [self.static[element_id] for element_id in selection]
        out += [detector for element_id in selection for other, detector in self.partners[element_id].items()
                if other in selection and element_id < other]
        return out

    def detected(self, selection):
        return set().union(*(detector.detected for detector in self.active(selection)))

    def summary(self, selection):
        f, detectors = self.f, self.active(selection)
        rejected = set().union(*(detector.rejected for detector in detectors))
        mask = np.zeros(f.count, bool)
        for detector in detectors:
            mask[detector.frames] = True
        rows = np.flatnonzero(mask)
        approach = rows[(f.phase[rows] == APPROACH) & (f.group[f.run[rows]] == 0)]
        return dict(detected=set().union(*(detector.detected for detector in detectors)), rejected=rejected,
                    clean=rejected - f.touched, rows=rows, approach=(len(approach), int(f.in_contact[approach].sum())))

    def contributions(self, selection):
        """Per kept element: the detections and mesh-clean rejected run-phases no other kept element (or pair) has."""
        cover, reject = collections.Counter(), collections.Counter()
        for detector in self.active(selection):
            cover.update(detector.detected)
            reject.update(detector.rejected)
        out = {}
        for element_id in selection:
            mine, mine_reject = collections.Counter(), collections.Counter()
            for detector in self.involved(element_id, selection):
                mine.update(detector.detected)
                mine_reject.update(detector.rejected)
            out[element_id] = ({item for item, count in mine.items() if count == cover[item]},
                               {rp for rp, count in mine_reject.items() if count == reject[rp]} - self.f.touched)
        return out

    def prune(self, selection, threshold, cost):
        """Drops elements, most costly first, while every detection over the threshold stays."""
        goal = {item for item in self.detected(selection) if item[2] >= threshold}
        current = set(selection)
        for element_id in sorted(selection, key=lambda item: (-cost(self.elements[item]), len(self.static[item].detected), item)):
            trial = frozenset(current - {element_id})
            if goal <= self.detected(trial):
                current = set(trial)
        return frozenset(current)


def initial_elements(patch, patched_cuboids):
    elements = []
    for link, value in json.loads(Path(patch).read_text())["links"].items():
        for index, ball in enumerate(value["added_spheres"]):
            elements.append(Element(f"{link}#{index + 1:02d}", "sphere", "auto", link=link, center=ball["center"], radius=ball["radius"]))
    added = set(json.loads(Path(patched_cuboids).read_text())["patch"]["added"])
    for name, pose, dimensions in cuboids(patched_cuboids, added):
        elements.append(Element(f"cuboid:{name}", "cuboid", "auto", pose=pose, half=dimensions / 2))
    return elements


def export_selection(engine, model, args, costs, out):
    """The kept elements as patch.json, cuboids.json (frozen + kept base cuboids), both patched robot configs, summary.json."""
    (out / "mobile18").mkdir(parents=True, exist_ok=True)
    selection = [element for element in engine.elements.values() if element.kept]
    links = collections.defaultdict(list)
    for element in selection:
        if element.kind == "sphere":
            links[element.link].append(dict(center=[round(float(v), 6) for v in element.center], radius=round(element.radius, 6)))
    (out / "patch.json").write_text(json.dumps(dict(
        parameters=dict(note="MOTION-334 patch editor export", state=str(args.state)),
        links={link: dict(frozen_spheres=len(model.kinematics["collision_spheres"].get(link, [])), added_spheres=balls)
               for link, balls in links.items()}), indent=1) + "\n")
    document = json.loads(args.frozen_cuboids.read_text())
    added = []
    for element in selection:
        if element.kind == "cuboid":
            center, dimensions = [round(float(v), 6) for v in element.pose[:3, 3]], [round(float(v) * 2, 6) for v in element.half]
            rotation = Rotation.from_matrix(element.pose[:3, :3])
            quaternion = rotation.as_quat()
            added.append(dict(name=element.id.replace("cuboid:", "").replace("#", "_"), center=center, dimensions=dimensions,
                              rotation_deg=[round(float(v), 4) for v in rotation.as_euler("xyz", degrees=True)],
                              pose=center + [round(float(v), 6) for v in (quaternion[3], *quaternion[:3])], dims=dimensions))
    (out / "cuboids.json").write_text(json.dumps(dict(document, cuboids=document["cuboids"] + added, patch=dict(
        source=str(args.frozen_cuboids), added=[item["name"] for item in added], script="patch_editor.py")), indent=1) + "\n")
    fixed = args.frozen.parent.parent / "alfa_v322_suction_final.yml"
    for target, config in ((out / fixed.name, fixed), (out / "mobile18" / args.frozen.name, args.frozen)):
        document = yaml.safe_load(config.read_text())
        for link, balls in links.items():
            document["kinematics"]["collision_spheres"][link] = document["kinematics"]["collision_spheres"][link] + balls
        target.write_text(yaml.safe_dump(document, sort_keys=False))
    (out / "mobile18" / "model").symlink_to(args.frozen.parent / "model")
    summary = engine.summary(engine.kept())
    (out / "summary.json").write_text(json.dumps(dict(
        detected={f"{key}/{GROUPS[group]}/>{threshold:g}mm": [sum((key, group, threshold, rp) in summary["detected"] for rp in total), len(total)]
                  for (key, group, threshold), total in sorted(engine.f.totals.items())},
        mesh_clean_rejected_phases=len(summary["clean"]), cost_ms_per_box=round(sum(costs["ms_per"][element.cost_group] for element in selection), 1),
        costs=costs, elements=[element.to_dict() for element in selection]), ensure_ascii=False, indent=1) + "\n")
    return out


def validate(engine, paths):
    """Every element kept vs the patched checks (GPU first, CPU replica after), on the cached frames."""
    f = engine.f
    ours = np.zeros(f.count, bool)
    ours[engine.summary(engine.kept())["rows"]] = True
    theirs, covered = np.zeros(f.count, bool), np.zeros(f.count, bool)
    for entry in load_checked(paths):
        index = f.index.get((entry["set"], entry["run"]))
        for phase, value in entry["phases"].items() if index is not None else ():
            start, end = f.segment(index, PHASES.index(phase))
            covered[start:end] = True
            theirs[start:end] = np.isin(f.local[start:end], sorted(expand(value["invalid"])))
    print("phase            group   frames   both  editor-only  check-only")
    for code, phase in enumerate(PHASES):
        for group, label in enumerate(GROUPS):
            mask = covered & (f.phase == code) & (f.group[f.run] == group)
            if mask.any():
                print(f"{phase:16s} {label}  {mask.sum():7d} {(ours & theirs & mask).sum():6d} {(ours & ~theirs & mask).sum():6d} "
                      f"{(~ours & theirs & mask).sum():6d}")
    print("frames not covered by the checks:", int((~covered).sum()))
    detected = engine.detected(engine.kept())
    for (key, group, threshold), total in sorted(f.totals.items()):
        found = sum((key, group, threshold, rp) in detected for rp in total)
        print(f"detected {key:20s} {GROUPS[group]} >{threshold:g} mm: {found}/{len(total)}")


def add_data_arguments(parser):
    """The options that say what the editor works on (shared with lean_select.py)."""
    parser.add_argument("--cache", type=Path, nargs="+", required=True, help="editor_cache.py outputs")
    parser.add_argument("--sample", type=int, default=1, help="keep every K-th run only (by name hash), for tests")
    parser.add_argument("--audit", type=Path, nargs="+", required=True, help="phase_audit.py records (a later file replaces phases)")
    parser.add_argument("--transport-depth", type=Path, required=True, help="MOTION-326 true-depth-records.jsonl")
    parser.add_argument("--frozen", type=Path, required=True, help="frozen mobile18 robot config")
    parser.add_argument("--frozen-cuboids", type=Path, required=True)
    parser.add_argument("--patch", type=Path, required=True, help="initial patch.json (appended spheres)")
    parser.add_argument("--patched-cuboids", type=Path, required=True, help="initial cuboids.json (frozen + appended)")
    parser.add_argument("--costs", type=Path, required=True, help="planning time per element group (JSON)")
    parser.add_argument("--hits", type=Path, help="hit cache (default: next to --state)")


def load_engine(args, state=None):
    """Frames, frozen model and the engine with every element evaluated: those of the state file if given and
    present, else the initial patch."""
    started = time.time()
    frames = Frames(args.cache, args.sample)
    frames.load_contacts(args.audit, args.transport_depth)
    model = Model(args.frozen, args.frozen_cuboids)
    signature = hashlib.sha1(json.dumps([[meta["set"], meta["run"], meta["frames"]] for meta in frames.meta]
                                        + [model.kinematics["collision_spheres"], sorted(map(sorted, model.ignored)),
                                           [[a.tolist(), b.tolist(), c.tolist()] for a, b, c in model.base_cuboids]]).encode()).hexdigest()
    hits = args.hits or args.state.with_suffix(".hits.npz")
    hits.parent.mkdir(parents=True, exist_ok=True)
    engine = Engine(frames, model, hits, signature)
    elements = ([Element(**item) for item in json.loads(state.read_text())["elements"]] if state is not None and state.exists()
                else initial_elements(args.patch, args.patched_cuboids))
    print(f"{frames.count} frames of {len(frames.meta)} runs loaded in {time.time() - started:.0f} s", flush=True)
    for count, element in enumerate(elements, 1):
        engine.add(element)
        if count % 10 == 0 or count == len(elements):
            print(f"  {count}/{len(elements)} elements evaluated, {time.time() - started:.0f} s", flush=True)
            engine.save()
    return frames, model, engine


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_data_arguments(parser)
    parser.add_argument("--state", type=Path, required=True, help="editor state (JSON): loaded if present, saved on every change")
    parser.add_argument("--export", type=Path, required=True, help="directory for exported patches")
    parser.add_argument("--run-dir", nargs=2, action="append", default=[], metavar=("LABEL", "DIR"), help="run directories, to open frames")
    parser.add_argument("--pose", nargs=5, action="append", default=[], metavar=("LABEL", "RUN_JSON", "PHASE", "INDEX", "NOTE"))
    parser.add_argument("--validate", type=Path, nargs="+", help="patched check JSONL files (GPU first): compare and exit")
    parser.add_argument("--port", type=int, default=8336)
    args = parser.parse_args()
    args.state.parent.mkdir(parents=True, exist_ok=True)
    frames, model, engine = load_engine(args, args.state)
    if args.validate:
        validate(engine, args.validate)
        return
    build_editor(args, frames, model, engine)
    print(f"Ready at http://localhost:{args.port}", flush=True)
    while True:
        time.sleep(1.)


def build_editor(args, frames, model, engine):
    """The viser server with the editor's scene and GUI; returns the GUI handles (for tests)."""
    costs = json.loads(args.costs.read_text())
    urdf = yourdfpy.URDF.load(model.kinematics["urdf_path"], load_meshes=True, build_scene_graph=True)
    locked = dict(model.kinematics.get("lock_joints", {}))
    presets = {item[0]: item for item in args.pose}
    run_dirs = {label: Path(directory) for label, directory in args.run_dir}
    lock = threading.RLock()
    history = []
    state = dict(selected=None, editing=None, colors={}, browse_contacts={}, browse_clean={})

    server = viser.ViserServer(host="127.0.0.1", port=args.port, label="MOTION-334 · 碰撞补丁编辑器")
    server.gui.configure_theme(control_width="large", show_share_button=False)
    server.scene.set_up_direction("+z")
    server.scene.add_grid("/grid", width=6.0, height=6.0)
    robot = ViserUrdf(server, urdf, root_node_name="/robot", mesh_color_override=(0.72, 0.76, 0.82, 0.35))
    sphere_mesh = trimesh.creation.icosphere(subdivisions=2, radius=1.0)
    link_frames = {link: server.scene.add_frame(f"/links/{link}", show_axes=False) for link in frames.links}
    frozen_nodes = {link: server.scene.add_batched_meshes_simple(
        f"/links/{link}/frozen", vertices=sphere_mesh.vertices, faces=sphere_mesh.faces, batched_positions=centers,
        batched_wxyzs=np.tile([1., 0., 0., 0.], (len(radii), 1)), batched_scales=radii, batched_colors=(71, 171, 210), opacity=0.22)
        for link, (centers, radii) in model.spheres.items()}
    frozen_boxes = [server.scene.add_box(f"/links/base_link/frozen_cuboid_{k}", dimensions=tuple(half * 2), position=center,
                                         wxyz=np.roll(Rotation.from_matrix(rotation).as_quat(), 1), color=(71, 171, 210), opacity=0.1)
                    for k, (rotation, center, half) in enumerate(model.base_cuboids)]
    nodes, scene_nodes, gizmo = {}, [], dict(handle=None, preview=None, link=None)

    def guarded(function):
        @functools.wraps(function)
        def wrapper(*items):
            with lock:
                return function(*items)
        return wrapper

    def cost(element):
        return costs["ms_per"][element.cost_group]

    # --- drawing ------------------------------------------------------------------------------------
    def look(element, contributions, selection):
        if element.id == state["selected"]:
            return (220, 40, 200), 0.95
        if not element.kept:
            return (160, 160, 160), 0.12
        if coloring.value == COLORINGS[1]:
            return ((240, 135, 30) if element.origin == "auto" else (40, 170, 90)), 0.9
        if contributions.get(element.id, (set(), set()))[0]:
            return (215, 50, 45), 0.9
        if any(detector.detected for detector in engine.involved(element.id, selection)):
            return (235, 190, 40), 0.9
        return (120, 135, 150), 0.6

    def draw(element, contributions, selection):
        tint, opacity = look(element, contributions, selection)
        key = (tint, opacity, element.digest())
        if state["colors"].get(element.id) == key:
            return
        if element.id in nodes:
            nodes.pop(element.id).remove()
        if element.kind == "sphere":
            handle = server.scene.add_icosphere(f"/links/{element.link}/el/{element.id}", radius=element.radius, color=tint,
                                                position=tuple(element.center), opacity=opacity, subdivisions=2)
        else:
            handle = server.scene.add_box(f"/links/base_link/el/{element.id}", dimensions=tuple(element.half * 2), color=tint,
                                          position=element.pose[:3, 3], wxyz=wxyz(element.pose), opacity=opacity * 0.6)
        handle.on_click(lambda _, element_id=element.id: select(element_id))
        nodes[element.id], state["colors"][element.id] = handle, key

    def drop_node(element_id):
        if element_id in nodes:
            nodes.pop(element_id).remove()
        state["colors"].pop(element_id, None)

    # --- summaries ----------------------------------------------------------------------------------
    def detection_table(detected):
        lines = ["| 接触（检出/总数，阶段数） | >0 mm | >2 mm |", "|---|---|---|"]
        for key, label in CLASSES.items():
            for group, group_label in enumerate(GROUPS):
                if not frames.totals.get((key, group, 0.)):
                    continue
                cells = []
                for threshold in THRESHOLDS_MM:
                    total = frames.totals.get((key, group, threshold), {})
                    found = sum((key, group, threshold, rp) in detected for rp in total)
                    cells.append(f"**{found}**/{len(total)}" if found < len(total) else f"{found}/{len(total)}")
                lines.append(f"| {label}（{group_label}） | {cells[0]} | {cells[1]} |")
        return lines

    def rejection_table(summary):
        rows = collections.Counter()
        for rp in summary["rejected"]:
            index, code = divmod(rp, len(PHASES))
            row = (PHASES[code], GROUPS[frames.group[index]], frames.meta[index]["kind"])
            rows[row + ("all",)] += 1
            rows[row + ("clean",)] += rp not in frames.touched
        totals = collections.Counter((PHASES[rp % len(PHASES)], GROUPS[frames.group[rp // len(PHASES)]], frames.meta[rp // len(PHASES)]["kind"])
                                     for rp in np.flatnonzero(frames.bounds[:, 1] > frames.bounds[:, 0]).tolist())
        lines = ["| 被拒的阶段 | 被拒/周期 | 其中网格无接触 |", "|---|---|---|"]
        for (phase, group, kind), total in sorted(totals.items(), key=lambda item: (PHASES.index(item[0][0]), item[0][1:])):
            if rows[(phase, group, kind, "all")]:
                lines.append(f"| {phase}（{group} {kind}） | {rows[(phase, group, kind, 'all')]}/{total} | {rows[(phase, group, kind, 'clean')]} |")
        return lines

    def refresh(save=True):
        selection = engine.kept()
        summary = engine.summary(selection)
        contributions = engine.contributions(selection)
        kept = [engine.elements[element_id] for element_id in selection]
        by_group = collections.Counter(element.cost_group for element in kept)
        spheres = collections.Counter(element.link for element in kept if element.kind == "sphere")
        total = sum(cost(element) for element in kept)
        approach, inside = summary["approach"]
        overview.content = "  \n".join(
            [f"保留：球 **{sum(spheres.values())}**（" + "，".join(f"{link} {n}" for link, n in sorted(spheres.items())) + "）"
             f"，底座长方体 **{by_group['base_cuboid']}**；去掉 {len(engine.elements) - len(kept)} 个",
             f"每箱规划耗时增加 **≈{total:.0f} ms**（冻结模型中位 {costs['baseline_round_ms']:.0f} ms 的 "
             f"+{total / costs['baseline_round_ms'] * 100:.1f}%）",
             "，".join(f"{COST_GROUPS[group]} {n} × {costs['ms_per'][group]:.1f} ms" for group, n in sorted(by_group.items())),
             f"_{costs['source']}_"])
        detection.content = "\n".join(detection_table(summary["detected"]))
        rejections.content = "\n".join(rejection_table(summary) + [
            "", f"基线 approach 的被拒帧在接触段内：{inside}/{approach}" + (f"（{inside / approach * 100:.0f}%）" if approach else "")])
        for element in engine.elements.values():
            draw(element, contributions, selection)
        options = (UNSELECTED,) + tuple(engine.elements)
        if tuple(element_choice.options) != options:
            element_choice.options = options
        refresh_browse(summary)
        refresh_selected(contributions)
        if save:
            args.state.write_text(json.dumps(dict(elements=engine.state()), ensure_ascii=False) + "\n")
            engine.save()

    def counts(items):
        out = collections.Counter((key, group, threshold) for key, group, threshold, _ in items)
        return "，".join(f"{CLASSES[key]}（{GROUPS[group]}）{n}" + (f"（>2 mm {out[(key, group, 2.)]}）" if out[(key, group, 2.)] else "")
                        for (key, group, threshold), n in sorted(out.items()) if threshold == 0.) or "无"

    def clean_counts(rps):
        return "，".join(f"{phase} {n}" for phase, n in collections.Counter(PHASES[rp % len(PHASES)] for rp in rps).most_common()) or "无"

    def refresh_selected(contributions=None):
        element = engine.elements.get(state["selected"])
        if element is None:
            info.content = "点击场景中的元素，或在上面的下拉框中选择。"
            return
        if keep.value != element.kept:
            keep.value = element.kept
        selection = engine.kept()
        own = engine.static[element.id]
        origin = {"auto": "自动", "manual": "手动"}.get(element.origin, "改过")
        lines = [f"**{element.id}**（{origin} · {'保留' if element.kept else '已去掉'}）", element.describe(),
                 f"时间代价 ≈ **{cost(element):.1f} ms/箱**（{COST_GROUPS[element.cost_group]}按组均摊）",
                 f"单独拒帧：{len(own.frames)} 帧，{len(own.rejected)} 个阶段"]
        if element.kept:
            unique, clean = (contributions or engine.contributions(selection))[element.id]
            lines += [f"独占检出（去掉就漏）：{counts(unique)}", f"独自造成的误拒（网格无接触的阶段）：{len(clean)}（{clean_counts(clean)}）"]
        else:
            trial = selection | {element.id}
            gained = engine.detected(trial) - engine.detected(selection)
            extra = engine.summary(trial)["clean"] - engine.summary(selection)["clean"]
            lines += [f"加回来会多检出：{counts(gained)}", f"加回来会多误拒：{len(extra)}（{clean_counts(extra)}）"]
        info.content = "  \n".join(lines)

    def refresh_browse(summary):
        missing = []
        for (key, group, threshold), total in frames.totals.items():
            for rp, (depth, deepest) in total.items() if threshold == 0. else ():
                if (key, group, 0., rp) not in summary["detected"]:
                    missing.append((depth, key, group, rp, deepest))
        contacts = {"（选择一个未检出的接触）": None}
        for depth, key, group, rp, deepest in sorted(missing, reverse=True):
            contacts[f"{depth:.1f} mm · {CLASSES[key]}（{GROUPS[group]}）· {frames.name(rp)}"] = (rp, deepest)
        rows, clean = summary["rows"], {"（选择一个误拒的阶段）": None}
        for rp in sorted(summary["clean"]):
            start, end = frames.bounds[rp]
            inside = rows[(rows >= start) & (rows < end)]
            clean[f"{PHASES[rp % len(PHASES)]} · {frames.name(rp)} · {len(inside)} 帧"] = (rp, int(frames.local[inside[0]]))
        state["browse_contacts"], state["browse_clean"] = contacts, clean
        for handle, options in ((missing_choice, contacts), (clean_choice, clean)):
            if tuple(handle.options) != tuple(options):
                handle.options = tuple(options)
        browse_note.content = f"未检出的接触 {len(contacts) - 1} 个，误拒的阶段 {len(clean) - 1} 个（选中后场景跳到该帧）"

    def select(element_id):
        with lock:
            previous, state["selected"] = state["selected"], element_id
            selection = engine.kept()
            contributions = engine.contributions(selection)
            for item in (previous, element_id):
                if item in engine.elements:
                    draw(engine.elements[item], contributions, selection)
            if element_choice.value != (element_id or UNSELECTED):
                element_choice.value = element_id or UNSELECTED
            refresh_selected(contributions)

    def change(function):
        """A GUI action that changes the patch: keeps the previous state for undo, then refreshes."""
        @functools.wraps(function)
        def wrapper(*items):
            with lock:
                history.append(engine.state())
                del history[:-50]
                status.content = "计算中…"
                try:
                    function(*items)
                finally:
                    status.content = ""
                refresh()
        return wrapper

    # --- poses and frames -----------------------------------------------------------------------------
    def show(view, rp=None, local=None):
        urdf.update_cfg({name: view.values[name] for name in urdf.actuated_joint_names})
        robot.update_cfg(np.asarray([view.values[name] for name in urdf.actuated_joint_names]))
        for link, frame in link_frames.items():
            matrix = urdf.get_transform(link, urdf.base_link)
            frame.position, frame.wxyz = matrix[:3, 3], wxyz(matrix)
        for handle in scene_nodes:
            handle.remove()
        scene_nodes.clear()
        for name, matrix, dimensions in view.objects:
            scene_nodes.append(server.scene.add_box(f"/scene/{name}", dimensions=tuple(dimensions), position=matrix[:3, 3],
                                                   wxyz=wxyz(matrix), color=(170, 170, 170), opacity=0.15, visible=show_scene.value))
        for name, matrix, dimensions in view.tasks:
            scene_nodes.append(server.scene.add_box(f"/scene/task/{name}", dimensions=tuple(dimensions), position=matrix[:3, 3],
                                                   wxyz=wxyz(matrix), color=(220, 60, 60), opacity=0.4, visible=show_scene.value))
        frame_note.content = view.note + ("  \n" + rejecting(rp, local) if rp is not None else "")

    def rejecting(rp, local):
        start, end = frames.bounds[rp]
        nearest = int(np.argmin(np.abs(frames.local[start:end] - local)))
        row, selection, names = start + nearest, engine.kept(), []
        cached = "" if frames.local[row] == local else f"（按最近的缓存帧，第 {frames.local[row]} 帧）"

        def contains(detector):
            position = np.searchsorted(detector.frames, row)
            return position < len(detector.frames) and detector.frames[position] == row

        for element_id in sorted(selection):
            if contains(engine.static[element_id]):
                names.append(element_id)
            names += [f"{element_id} × {other}" for other, detector in engine.partners[element_id].items()
                      if other in selection and element_id < other and contains(detector)]
        where = "在接触段内" if frames.in_contact[row] else "网格无接触"
        return f"此帧{where}{cached}，被拒于：" + ("，".join(names) if names else "无（保留的元素都不拒此帧）")

    def open_frame(rp, local, note):
        index, code = divmod(rp, len(PHASES))
        meta = frames.meta[index]
        path = run_dirs[meta["set"]] / "full" / f"{meta['run']}.json"
        show(ViewPose(note, path, PHASES[code], local, f"{note}：`{meta['set']}/{meta['run']}` {PHASES[code]} 第 {local} 帧", urdf, locked),
             rp, local)

    def camera(label):
        offset, link = VIEWS[label]
        target = urdf.get_transform(link, urdf.base_link)[:3, 3] + (np.array([0., 0., 0.9]) if label == "整机" else 0.)
        return tuple(target + np.asarray(offset)), tuple(target)

    # --- GUI ------------------------------------------------------------------------------------------
    with server.gui.add_folder("当前方案"):
        overview = server.gui.add_markdown("")
        status = server.gui.add_markdown("")
        undo = server.gui.add_button("撤销上一步")
        prune_threshold = server.gui.add_dropdown("精简时保持", ("全部检出", "只保持 >2 mm 的检出"), initial_value="全部检出")
        prune = server.gui.add_button("精简：去掉多余元素（贵的先去）")
        keep_all = server.gui.add_button("全部保留")
        drop_all = server.gui.add_button("全部去掉")
        reset = server.gui.add_button("恢复初始方案")
    with server.gui.add_folder("检出"):
        detection = server.gui.add_markdown("")
    with server.gui.add_folder("误拒", expand_by_default=False):
        rejections = server.gui.add_markdown("")
    with server.gui.add_folder("选中元素"):
        element_choice = server.gui.add_dropdown("元素", (UNSELECTED,) + tuple(engine.elements), initial_value=UNSELECTED)
        keep = server.gui.add_checkbox("保留", initial_value=True)
        info = server.gui.add_markdown("")
        move = server.gui.add_button("移动 / 改尺寸")
        delete = server.gui.add_button("删除（只限手动加的）")
    with server.gui.add_folder("添加 / 编辑"):
        target_choice = server.gui.add_dropdown("加在", SPHERE_LINKS + (CUBOID,), initial_value=SPHERE_LINKS[0])
        radius = server.gui.add_slider("球半径 mm", min=3., max=80., step=0.5, initial_value=12.)
        size = [server.gui.add_slider(f"长方体 {axis} mm", min=5., max=800., step=1., initial_value=60.) for axis in "xyz"]
        place = server.gui.add_button("放控制器")
        commit = server.gui.add_button("添加 / 应用修改")
        cancel = server.gui.add_button("取消")
        edit_note = server.gui.add_markdown("先选连杆和尺寸，点「放控制器」，拖动后点「添加 / 应用修改」。")
    with server.gui.add_folder("浏览"):
        missing_choice = server.gui.add_dropdown("未检出的接触", ("（选择一个未检出的接触）",))
        clean_choice = server.gui.add_dropdown("误拒的阶段", ("（选择一个误拒的阶段）",))
        preset_choice = server.gui.add_dropdown("预设姿态", tuple(presets) or ("（无）",))
        browse_note = server.gui.add_markdown("")
        frame_note = server.gui.add_markdown("")
        view_buttons = server.gui.add_button_group("视角", tuple(VIEWS))
    with server.gui.add_folder("显示", expand_by_default=False):
        coloring = server.gui.add_dropdown("着色", COLORINGS, initial_value=COLORINGS[0])
        show_robot = server.gui.add_checkbox("机器人实体", initial_value=True)
        show_frozen = server.gui.add_checkbox("冻结的球与长方体（蓝）", initial_value=True)
        show_base_spheres = server.gui.add_checkbox("底座的冻结球", initial_value=False)
        show_scene = server.gui.add_checkbox("场景（待抓箱红色）", initial_value=True)
        server.gui.add_markdown("选中：紫。去掉的元素：浅灰。")
    with server.gui.add_folder("导出", expand_by_default=False):
        export = server.gui.add_button("导出当前方案")
        exported = server.gui.add_markdown("")

    # --- gizmo ----------------------------------------------------------------------------------------
    def clear_gizmo():
        for key in ("preview", "handle"):
            if gizmo[key] is not None:
                gizmo[key].remove()
                gizmo[key] = None
        state["editing"] = None

    def draw_preview():
        if gizmo["handle"] is None:
            return
        if gizmo["preview"] is not None:
            gizmo["preview"].remove()
        name = gizmo["handle"].name + "/preview"
        if gizmo["link"] == "base_link":
            gizmo["preview"] = server.scene.add_box(name, dimensions=tuple(s.value / 1000 for s in size), color=(40, 170, 90), opacity=0.45)
        else:
            gizmo["preview"] = server.scene.add_icosphere(name, radius=radius.value / 1000, color=(40, 170, 90), opacity=0.6, subdivisions=2)

    def open_gizmo(link, position, rotation=(1., 0., 0., 0.)):
        clear_gizmo()
        gizmo["link"] = link
        gizmo["handle"] = server.scene.add_transform_controls(
            f"/links/{link}/gizmo", scale=0.35 if link == "base_link" else 0.12, position=position, wxyz=rotation,
            disable_rotations=link != "base_link", depth_test=False)
        draw_preview()

    @place.on_click
    @guarded
    def _(_):
        selected = engine.elements.get(state["selected"])
        if target_choice.value == CUBOID:
            if selected is not None and selected.kind == "cuboid":
                open_gizmo("base_link", selected.pose[:3, 3], wxyz(selected.pose))
            else:
                open_gizmo("base_link", (0.3, 0., 0.3))
        else:
            link = target_choice.value
            here = selected is not None and selected.kind == "sphere" and selected.link == link
            open_gizmo(link, selected.center if here else model.centroid[link])
        edit_note.content = "拖动控制器（坐标在该连杆系中），尺寸用上面的滑条；点「添加 / 应用修改」后计算约几秒。"

    @move.on_click
    @guarded
    def _(_):
        element = engine.elements.get(state["selected"])
        if element is None:
            return
        if element.kind == "sphere":
            target_choice.value, radius.value = element.link, round(element.radius * 2000) / 2
            open_gizmo(element.link, element.center)
        else:
            target_choice.value = CUBOID
            for slider, value in zip(size, element.half * 2000):
                slider.value = float(np.clip(round(float(value)), 5, 800))
            open_gizmo("base_link", element.pose[:3, 3], wxyz(element.pose))
        state["editing"] = element.id
        edit_note.content = f"正在编辑 **{element.id}**：拖动或改尺寸，点「添加 / 应用修改」。"

    @cancel.on_click
    @guarded
    def _(_):
        clear_gizmo()
        edit_note.content = "已取消。"

    @commit.on_click
    @change
    def _(_):
        if gizmo["handle"] is None:
            return
        position, rotation, link = np.asarray(gizmo["handle"].position, float), np.asarray(gizmo["handle"].wxyz, float), gizmo["link"]
        cuboid = link == "base_link"
        editing = engine.elements.get(state["editing"])
        if editing is not None and (editing.kind == "cuboid") == cuboid:
            engine.remove(editing.id)
            drop_node(editing.id)
            element_id, origin = editing.id, "manual" if editing.origin == "manual" else "edited"
        else:
            number = 1
            while f"manual#{number:02d}" in engine.elements:
                number += 1
            element_id, origin = f"manual#{number:02d}", "manual"
        if cuboid:
            pose = np.eye(4)
            pose[:3, :3] = Rotation.from_quat(np.roll(rotation, -1)).as_matrix()
            pose[:3, 3] = position
            element = Element(element_id, "cuboid", origin, pose=pose, half=np.asarray([s.value for s in size]) / 2000)
        else:
            element = Element(element_id, "sphere", origin, link=link, center=position, radius=radius.value / 1000)
        engine.add(element)
        clear_gizmo()
        state["selected"] = element.id
        edit_note.content = f"已{'修改' if editing is not None else '添加'} **{element.id}**。"

    @delete.on_click
    @change
    def _(_):
        element = engine.elements.get(state["selected"])
        if element is not None and element.origin == "manual":
            engine.remove(element.id)
            drop_node(element.id)
            state["selected"] = None

    @keep.on_update
    def _(_):
        element = engine.elements.get(state["selected"])
        if element is not None and element.kept != keep.value:
            change(lambda: setattr(element, "kept", keep.value))()

    @element_choice.on_update
    def _(_):
        chosen = element_choice.value if element_choice.value in engine.elements else None
        if chosen != state["selected"]:
            select(chosen)

    def set_all(value):
        for element in engine.elements.values():
            element.kept = value

    keep_all.on_click(change(lambda _: set_all(True)))
    drop_all.on_click(change(lambda _: set_all(False)))

    @prune.on_click
    @change
    def _(_):
        result = engine.prune(engine.kept(), 0. if prune_threshold.value == "全部检出" else 2., cost)
        for element in engine.elements.values():
            element.kept = element.id in result

    @reset.on_click
    @change
    def _(_):
        for element_id in list(engine.elements):
            engine.remove(element_id)
            drop_node(element_id)
        for element in initial_elements(args.patch, args.patched_cuboids):
            engine.add(element)
        state["selected"] = None

    @undo.on_click
    @guarded
    def _(_):
        if not history:
            return
        status.content = "撤销中…"
        engine.restore(history.pop())
        for element_id in list(nodes):
            if element_id not in engine.elements:
                drop_node(element_id)
        if state["selected"] not in engine.elements:
            state["selected"] = None
        status.content = ""
        refresh()

    @missing_choice.on_update
    @guarded
    def _(_):
        item = state["browse_contacts"].get(missing_choice.value)
        if item is not None:
            open_frame(item[0], item[1], "未检出的接触（最深处）")

    @clean_choice.on_update
    @guarded
    def _(_):
        item = state["browse_clean"].get(clean_choice.value)
        if item is not None:
            open_frame(item[0], item[1], "误拒（第一个被拒的缓存帧）")

    @preset_choice.on_update
    @guarded
    def _(_):
        if preset_choice.value in presets:
            show(ViewPose(*presets[preset_choice.value], urdf, locked))

    @view_buttons.on_click
    def _(event):
        position, target = camera(view_buttons.value)
        event.client.camera.position, event.client.camera.look_at = position, target
        event.client.camera.up_direction = (0., 0., 1.)

    @coloring.on_update
    @guarded
    def _(_):
        selection = engine.kept()
        contributions = engine.contributions(selection)
        for element in engine.elements.values():
            draw(element, contributions, selection)

    radius.on_update(lambda _: draw_preview())
    for slider in size:
        slider.on_update(lambda _: draw_preview())
    target_choice.on_update(lambda _: clear_gizmo() if state["editing"] is None else None)
    show_robot.on_update(lambda _: setattr(robot, "show_visual", show_robot.value))

    def frozen_visibility(_=None):
        for link, node in frozen_nodes.items():
            node.visible = show_frozen.value and (link != "base_link" or show_base_spheres.value)
        for node in frozen_boxes:
            node.visible = show_frozen.value

    show_frozen.on_update(frozen_visibility)
    show_base_spheres.on_update(frozen_visibility)
    show_scene.on_update(lambda _: [setattr(node, "visible", show_scene.value) for node in scene_nodes])

    @export.on_click
    @guarded
    def _(_):
        out = export_selection(engine, model, args, costs, args.export / datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
        exported.content = f"已导出到 `{out}`：patch.json、cuboids.json、两份机器人配置、summary.json"

    if presets:
        show(ViewPose(*next(iter(presets.values())), urdf, locked))
    frozen_visibility()
    refresh(save=False)
    server.initial_camera.position, server.initial_camera.look_at = camera("整机")
    server.initial_camera.up = (0., 0., 1.)
    return dict(server=server, state=state, gizmo=gizmo, history=history, element_choice=element_choice, keep=keep, info=info,
                overview=overview, detection=detection, rejections=rejections, prune=prune, prune_threshold=prune_threshold,
                keep_all=keep_all, drop_all=drop_all, reset=reset, undo=undo, target_choice=target_choice, radius=radius,
                size=size, place=place, commit=commit, cancel=cancel, move=move, delete=delete, missing_choice=missing_choice,
                clean_choice=clean_choice, preset_choice=preset_choice, frame_note=frame_note, coloring=coloring,
                export=export, exported=exported)


if __name__ == "__main__":
    main()
