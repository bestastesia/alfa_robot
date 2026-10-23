#!/usr/bin/env python3
"""MOTION-311 scene variants: docking offsets, mirror-grouped box cells and the instance registry.

The docking offset shifts the pick-side scene (ground, side/front walls, ceiling, wall boxes, fillers)
by (dx, dy) relative to the robot after every task configuration, as the MOTION-327 harness did; the
conveyor side and the placement targets stay fixed. Splits group box cells by left/right mirror
symmetry, cell = (row, min(column, 4 - column)), so mirrored tasks never straddle a split. A task is
test if any of its boxes lies in a test cell, else dev if any lies in a dev cell, else train; every
offset and seed of a task stays in that task's split.
"""
import argparse
import hashlib
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from curobo_core.fixtures import tasks

SHIFTED_IDS = {"ground", "left_wall", "right_wall", "front_wall", "ceiling"}
TOP_ROWS = (0, 1)
SIDE_ROWS = (2, 3, 4)
STREAMS = {"train_dev": 311, "test": 3111, "seen_test": 3112, "out_of_range": 3113, "reserve": 3114}
VARIANTS = {"train_dev": {"dual": 3, "single": 5}, "test": {"dual": 2, "single": 6}, "seen_test": {"dual": 1, "single": 1},
            "out_of_range": {"dual": 1, "single": 1}, "reserve": {"dual": 2, "single": 2}}
OFFSET_LIMIT_M = .05
OUT_OF_RANGE_LIMIT_M = .08


class DockingOffsetMixin:
    """Shifts the pick-side scene by scene_offset (m) after every task configuration."""

    scene_offset = (0.0, 0.0)

    def configure_task(self, boxes):
        result = super().configure_task(boxes)
        dx, dy = self.scene_offset
        if dx or dy:
            objects = tuple(
                replace(item, pose=replace(item.pose, position=(item.pose.position[0] + dx, item.pose.position[1] + dy,
                                                                item.pose.position[2])))
                if item.object_id in SHIFTED_IDS or item.object_id.startswith(("wall_box_", "task_fill_")) else item
                for item in self.snapshot.objects)
            self.snapshot = replace(self.snapshot, objects=objects, revision=self.snapshot.revision + 1)
        return result


def box_cell(box_id):
    row, column = divmod(int(box_id), 5)
    return row, min(column, 4 - column)


def all_tasks():
    dual = [boxes for _, boxes in tasks() + tasks([1, 0])]
    single = [{("left" if box_id % 5 >= 3 else "right"): box_id} for box_id in range(25)]
    return dual + single


def task_key(boxes):
    return "-".join(f"{side[0].upper()}{box_id:02d}" for side, box_id in sorted(boxes.items()))


def draw_cells(seed=STREAMS["train_dev"]):
    """3 test cells (one per column class, at least one top-suction and one side-suction row), then
    2 dev cells (one top row, one side row) from the remaining cells."""
    rng = np.random.default_rng(seed)
    cells = [(row, column) for row in range(5) for column in range(3)]
    while True:
        test = [cells[index] for index in rng.choice(len(cells), 3, replace=False)]
        if (sorted(column for _, column in test) == [0, 1, 2] and any(row in TOP_ROWS for row, _ in test)
                and any(row in SIDE_ROWS for row, _ in test)):
            break
    remaining = [cell for cell in cells if cell not in test]
    top = [cell for cell in remaining if cell[0] in TOP_ROWS]
    side = [cell for cell in remaining if cell[0] in SIDE_ROWS]
    dev = [top[int(rng.integers(len(top)))], side[int(rng.integers(len(side)))]]
    return sorted(test), sorted(dev)


def task_split(boxes, test_cells, dev_cells):
    cells = {box_cell(box_id) for box_id in boxes.values()}
    if cells & set(test_cells):
        return "test"
    if cells & set(dev_cells):
        return "dev"
    return "train"


def draw_offsets(rng, count, out_of_range=False):
    offsets = []
    while len(offsets) < count:
        limit = OUT_OF_RANGE_LIMIT_M if out_of_range else OFFSET_LIMIT_M
        offset = np.round(rng.uniform(-limit, limit, 2), 4)
        if out_of_range and np.max(np.abs(offset)) <= OFFSET_LIMIT_M:
            continue
        offsets.append([float(value) for value in offset])
    return offsets


def build_registry():
    test_cells, dev_cells = draw_cells()
    rngs = {name: np.random.default_rng(seed) for name, seed in STREAMS.items()}
    instances = []
    for boxes in all_tasks():
        split = task_split(boxes, test_cells, dev_cells)
        kind = "dual" if len(boxes) == 2 else "single"
        key = task_key(boxes)
        groups = []
        if split == "test":
            groups.append(("test", draw_offsets(rngs["test"], VARIANTS["test"][kind])))
            groups.append(("out_of_range", draw_offsets(rngs["out_of_range"], VARIANTS["out_of_range"][kind], True)))
        else:
            groups.append((split, [[0.0, 0.0]] + draw_offsets(rngs["train_dev"], VARIANTS["train_dev"][kind])))
            groups.append(("seen_test", draw_offsets(rngs["seen_test"], VARIANTS["seen_test"][kind])))
        groups.append(("reserve", draw_offsets(rngs["reserve"], VARIANTS["reserve"][kind])))
        for group, offsets in groups:
            for variant, offset in enumerate(offsets):
                instances.append(dict(instance=f"{key}-{group}-v{variant}", task=boxes, task_key=key, kind=kind,
                                      split=split, group=group, variant=variant, offset_m=offset,
                                      cells=sorted({box_cell(box_id) for box_id in boxes.values()}),
                                      top_grasp=all(box_id // 5 <= 1 for box_id in boxes.values())))
    body = dict(schema_version=1, streams=STREAMS, variants=VARIANTS, offset_limit_m=OFFSET_LIMIT_M,
                out_of_range_limit_m=OUT_OF_RANGE_LIMIT_M, test_cells=test_cells, dev_cells=dev_cells,
                cell_rule="cell=(row, min(column, 4-column)); test if any box in a test cell, else dev, else train",
                instances=instances)
    return dict(body, sha256=registry_digest(body))


def registry_digest(body):
    content = {key: value for key, value in body.items() if key != "sha256"}
    return hashlib.sha256(json.dumps(content, sort_keys=True, ensure_ascii=False).encode()).hexdigest()


def load_registry(path):
    body = json.loads(Path(path).read_text())
    if registry_digest(body) != body["sha256"]:
        raise ValueError(f"registry {path} does not match its sha256")
    return body


def select(registry, group=None, split=None, kind=None):
    return [item for item in registry["instances"] if (group is None or item["group"] == group)
            and (split is None or item["split"] == split) and (kind is None or item["kind"] == kind)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    registry = build_registry()
    args.output.write_text(json.dumps(registry, ensure_ascii=False, indent=1) + "\n")
    counts = {}
    for item in registry["instances"]:
        counts.setdefault(f"{item['group']}/{item['kind']}", set()).add(item["instance"])
    print(json.dumps(dict(sha256=registry["sha256"], test_cells=registry["test_cells"], dev_cells=registry["dev_cells"],
                          counts={key: len(value) for key, value in sorted(counts.items())}), ensure_ascii=False))


if __name__ == "__main__":
    main()
