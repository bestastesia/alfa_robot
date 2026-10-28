import numpy as np
import torch

from v3_current_urdf_forward import CurrentUrdfForwardPlanner, current_assets
from v3_timed_single_arm_planner import TimedSingleArmPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS


class TimedCurrentUrdfPlanner(TimedSingleArmPlanner, CurrentUrdfForwardPlanner):
    trajectory_optimization_enabled = False

    def __init__(self, args=None, sample_dt=.025):
        super().__init__(args or current_assets(), sample_dt=sample_dt)
        velocity, acceleration, jerk = (values.copy() for values in self._timed_limits)
        for index, name in enumerate(self._timed_joint_names):
            if name.startswith(('left_joint', 'right_joint')):
                wrist = int(name.rsplit('joint', 1)[1]) >= 5
                velocity[index] = min(velocity[index], np.deg2rad(30. if wrist else 15.))
                acceleration[index] = min(acceleration[index], np.deg2rad(60. if wrist else 30.))
        self._timed_limits = velocity, acceleration, jerk

    def _timed_phase_valid(self, phase, positions, boxes, original_snapshot):
        if not super()._timed_phase_valid(phase, positions, boxes, original_snapshot):
            return False
        indices = [self._timed_joint_names.index(name) for name in ACTIVE_JOINTS]
        active = torch.tensor(np.ascontiguousarray(np.asarray(positions)[:, indices]), device='cuda', dtype=torch.float32)
        try:
            validity = self._timed_phase_validity(phase, boxes, original_snapshot, positions[0])
            validity.collision_weights = torch.tensor([5.] + [1.] * 14, device='cuda')
            for offset in range(0, len(active) - 1, 16):
                starts = active[offset:offset + 16]
                ends = active[offset + 1:offset + 17]
                starts = starts[:len(ends)]
                if not bool(validity.edges(starts, ends, resolution=np.deg2rad(.5)).all()):
                    return False
            return True
        finally:
            self.snapshot = original_snapshot


    def full_cycle(self, boxes, progress=lambda text: None):
        return super().full_cycle(boxes, progress)
