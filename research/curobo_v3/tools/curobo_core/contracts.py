from dataclasses import dataclass
from pathlib import Path

from .scene import Pose, SceneSnapshot, digest


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


@dataclass(frozen=True)
class PlanRequest:
    snapshot: SceneSnapshot
    tasks: tuple
    targets: tuple

    def __post_init__(self):
        object.__setattr__(self, "tasks", tuple(tuple(item) for item in self.tasks))
        object.__setattr__(self, "targets", tuple(tuple(item) for item in self.targets))
        sides = {side for side, _ in self.tasks}
        if not sides or not sides <= {"left", "right"} or len(sides) != len(self.tasks):
            raise ValueError("full-cycle planner requires one task per active arm")
        box_ids = [box_id for _, box_id in self.tasks]
        if any(not isinstance(box_id, int) or box_id < 0 for box_id in box_ids) or len(set(box_ids)) != len(self.tasks):
            raise ValueError("invalid task box IDs")
        if len(self.targets) != len(sides) or {name for name, _ in self.targets} != {side + '_tool0' for side in sides}:
            raise ValueError("complete Tool0 targets are required for active arms")
        if not all(isinstance(pose, Pose) for _, pose in self.targets):
            raise ValueError("target must be a Pose")
        self.snapshot.state.ordered(ACTIVE_JOINTS)
        for box_id in box_ids:
            self.snapshot.object(f"wall_box_{box_id:02d}")

    def to_dict(self):
        from dataclasses import asdict

        return {"snapshot": self.snapshot.to_dict(), "tasks": dict(self.tasks),
                "targets": {name: asdict(pose) for name, pose in self.targets}}

    @classmethod
    def from_dict(cls, document):
        return cls(SceneSnapshot.from_dict(document["snapshot"]),
                   tuple(document["tasks"].items()),
                   tuple((name, Pose(**pose)) for name, pose in document["targets"].items()))

    @property
    def identity(self):
        return digest(self.to_dict())
