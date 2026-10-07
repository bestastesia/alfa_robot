from dataclasses import dataclass
from pathlib import Path

from .scene import Pose, SceneSnapshot, digest
from .fixtures import WallLayout


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
ACTIVE_JOINTS = ("updown", *(f"{side}_joint{index}" for side in ("left", "right") for index in range(1, 8)))


@dataclass(frozen=True)
class PlannerAssets:
    robot_config: Path = WORKSPACE_ROOT / "generated/v3_analytic_071cb95/alfa_v322_suction_final.yml"
    mobile_robot_config: Path = WORKSPACE_ROOT / "generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml"
    urdf: Path = WORKSPACE_ROOT / "generated/v3_analytic_071cb95/model/robot_meter_meshes.urdf"
    named_poses: Path = WORKSPACE_ROOT / "models/robot_description_analytic_071cb95/config/named_poses_analytic_proxy.yaml"
    box_fit: Path = WORKSPACE_ROOT / "generated/v3_suction_v322_6bb184b/frozen_collision_model/box_voxel_edge_corner_final.json"
    target_poses: Path = WORKSPACE_ROOT / "generated/current_carry_target_6d.json"
    wall_layout: WallLayout = WallLayout()


@dataclass(frozen=True)
class PlanRequest:
    """Targets are side-grasp reference Tool0 poses; top mode preserves the implied box destination."""
    snapshot: SceneSnapshot
    tasks: tuple
    targets: tuple
    suction_mode: str = "auto"
    search_seed: int | None = None

    def __post_init__(self):
        if self.search_seed is not None and (type(self.search_seed) is not int or not 0 <= self.search_seed < 2**32):
            raise ValueError("search_seed must be an unsigned 32-bit integer or None")
        if self.suction_mode not in ("auto", "side", "top"):
            raise ValueError("suction_mode must be auto, side or top")
        object.__setattr__(self, "tasks", tuple(tuple(item) for item in self.tasks))
        object.__setattr__(self, "targets", tuple(tuple(item) for item in self.targets))
        sides = {side for side, _ in self.tasks}
        if not sides or not sides <= {"left", "right"} or len(sides) != len(self.tasks):
            raise ValueError("one or two distinct active arms are required")
        box_ids = [box_id for _, box_id in self.tasks]
        if any(type(box_id) is not int or box_id < 0 for box_id in box_ids) or len(set(box_ids)) != len(box_ids):
            raise ValueError("invalid task box IDs")
        if len(self.targets) != 2 or {name for name, _ in self.targets} != {"left_tool0", "right_tool0"}:
            raise ValueError("both complete Tool0 targets are required, including the empty arm carry pose")
        if not all(isinstance(pose, Pose) for _, pose in self.targets):
            raise ValueError("target must be a Pose")
        self.snapshot.state.ordered(ACTIVE_JOINTS)
        for box_id in box_ids:
            try:
                self.snapshot.object(f"wall_box_{box_id:02d}")
            except StopIteration as error:
                raise ValueError(f"task box {box_id} is absent from the scene snapshot") from error

    def to_dict(self):
        from dataclasses import asdict

        document = {"snapshot": self.snapshot.to_dict(), "tasks": dict(self.tasks),
                    "targets": {name: asdict(pose) for name, pose in self.targets}}
        if self.suction_mode != "side":  # Preserve identities of legacy side-only requests.
            document["suction_mode"] = self.suction_mode
        if self.search_seed is not None:
            document["search_seed"] = self.search_seed
        return document

    @classmethod
    def from_dict(cls, document):
        return cls(SceneSnapshot.from_dict(document["snapshot"]),
                   tuple(document["tasks"].items()),
                   tuple((name, Pose(**pose)) for name, pose in document["targets"].items()),
                   document.get("suction_mode", "side"), document.get("search_seed"))

    @property
    def identity(self):
        return digest(self.to_dict())
