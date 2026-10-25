"""Stage-local features from known task/state; no future candidates, outcomes or value tables."""
import numpy as np

from motion311_learning.contracts import potential
from v3_candidate_ranker import ACTIVE_JOINTS, ARMS

STATE_DIM = 3 + 4 + 15 + 15 + 2 * 12 + 2 * 12 + 6 + 2 + 1
PHYSICS_DIM = 3 + 8 + 2 * 3 + 2 * 3


def state_features(stage, previous, home, targets, carry_targets, box_centers, active_sides, top, offset, cost):
    """94-vector of current/known task information, never chosen future configurations."""
    onehot = np.eye(3)[stage]
    flags = np.array([len(active_sides) == 2, top, 'left' in active_sides, 'right' in active_sides], dtype=float)
    flatten = lambda mapping: np.concatenate([np.concatenate([mapping[s][:3, 3], mapping[s][:3, :3].reshape(-1)])
                                              if s in mapping else np.zeros(12) for s in ARMS])
    centers = np.concatenate([np.asarray(box_centers.get(s, np.zeros(3))) for s in ARMS])
    vector = np.concatenate([onehot, flags, np.asarray(previous), np.asarray(home), flatten(targets), flatten(carry_targets),
                             centers, np.asarray(offset), [potential(cost)]])
    if vector.shape != (STATE_DIM,):
        raise ValueError(f"state shape {vector.shape}, expected {STATE_DIM}")
    return vector


class PhysicsFeatures:
    def __init__(self, planner):
        self.planner = planner
        from v3_candidate_ranker import Kinematics
        limits = Kinematics(planner.robot['kinematics']['urdf_path'])
        self.lower, self.upper = limits.lower, limits.upper

    def candidates(self, rows, previous, targets, active_sides):
        """Limit margins, sine/cosine of elbow-like joints, wrist/tool and elbow positions; current IK only."""
        rows = np.asarray(rows, dtype=float).reshape(-1, 15)
        output = []
        for row in rows:
            margin = np.minimum(row - self.lower, self.upper - row)
            margins = np.array([margin[0], margin[1:8].min(), margin[8:].min()])
            angles = row[[2, 4, 9, 11]]
            self.planner.fk(row)
            tools, elbows = [], []
            for side in ARMS:
                tools.extend(self.planner.fk_robot.get_transform(side + '_tool0', self.planner.fk_robot.base_link)[:3, 3])
                elbows.extend(self.planner.fk_robot.get_transform(side + '_link4', self.planner.fk_robot.base_link)[:3, 3])
            output.append(np.concatenate([margins, np.sin(angles), np.cos(angles), tools, elbows]))
        return np.asarray(output, dtype=np.float64).reshape(-1, PHYSICS_DIM)
