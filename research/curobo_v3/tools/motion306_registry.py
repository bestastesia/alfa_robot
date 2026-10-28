"""MOTION-306 instance registry: every MOTION-311 instance at every box-size level.

Cells, splits, docking offsets and sealed groups are inherited unchanged. HELDOUT_LEVELS are never used
for training: S35x38x42 interpolates every dimension, S40x37x45 combines individually seen dimensions.
"""
import json
from pathlib import Path

from v3_box_geometry import SIZE_LEVELS
from v3_scene_variants import build_registry, registry_digest

HELDOUT_LEVELS = ("S35x38x42", "S40x37x45")
PATH = Path(__file__).resolve().parents[1] / "config/motion306_registry.json"


def build_size_registry():
    base = build_registry()
    instances = [dict(item, instance=f"{item['instance']}-{level}", base_instance=item["instance"], box_level=level,
                      box_dims=list(dims), size_split="heldout" if level in HELDOUT_LEVELS else "seen")
                 for item in base["instances"] for level, dims in SIZE_LEVELS.items()]
    body = dict(schema_version=1, base_registry_sha256=base["sha256"],
                size_levels={level: list(dims) for level, dims in SIZE_LEVELS.items()},
                heldout_levels=list(HELDOUT_LEVELS),
                size_rule="one box size per scene (depth, width, height m); heldout levels never trained",
                instances=instances)
    return dict(body, sha256=registry_digest(body))


def main():
    registry = build_size_registry()
    PATH.write_text(json.dumps(registry, ensure_ascii=False, separators=(",", ":")) + "\n")
    print(json.dumps(dict(path=str(PATH), instances=len(registry["instances"]), sha256=registry["sha256"])))


if __name__ == "__main__":
    main()
