import copy
import time

import numpy as np
import torch

from curobo_core.planner import CycleBlocked
from v3_current_urdf_forward import CurrentUrdfForwardPlanner, current_assets
from v3_timed_single_arm_planner import TimedSingleArmPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS


class TimedCurrentUrdfPlanner(TimedSingleArmPlanner, CurrentUrdfForwardPlanner):
    trajectory_optimization_enabled = True

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
                if not bool(validity.edges(starts, ends, resolution=np.deg2rad(getattr(self, 'timed_validation_resolution_deg', .5))).all()):
                    return False
            return True
        finally:
            self.snapshot = original_snapshot

    def _shortcut_phase(self, phase, rows, boxes, original_snapshot):
        if phase not in ('approach', 'transport', 'conveyor_place', 'return_home'):
            return rows, dict(raw_waypoints=len(rows), shortcut_waypoints=len(rows))
        validity = self._timed_phase_validity(phase, boxes, original_snapshot, rows[0])
        indices = [self._timed_joint_names.index(name) for name in ACTIVE_JOINTS]
        active = torch.tensor(np.ascontiguousarray(rows[:, indices]), device='cuda', dtype=torch.float32)
        selected = [0]
        try:
            while selected[-1] < len(rows) - 1:
                source = selected[-1]
                destination = source + 1
                for end in range(len(rows), source + 1, -16):
                    begin = max(source + 1, end - 16)
                    candidates = active[begin:end]
                    starts = active[source].reshape(1, -1).expand(len(candidates), -1)
                    legal = validity.edges(starts, candidates, resolution=np.deg2rad(.05))
                    accepted = torch.nonzero(legal, as_tuple=False).reshape(-1)
                    if len(accepted):
                        destination = begin + int(accepted[-1])
                        break
                selected.append(destination)
        finally:
            self.snapshot = original_snapshot
        return rows[selected], dict(raw_waypoints=len(rows), shortcut_waypoints=len(selected))

    def plan_geometric_cycle(self, boxes, progress):
        return CurrentUrdfForwardPlanner.full_cycle(self, boxes, progress)

    def full_cycle(self, boxes, progress=lambda text: None):
        began = time.perf_counter()
        result = self.plan_geometric_cycle(boxes, progress)
        if not result.get('success'):
            return result
        progress('几何规划完成：GPU验碰Shortcut、平滑与速度/加速度/jerk时间化')
        geometric = copy.deepcopy(result)
        snapshot = self.snapshot
        try:
            self.time_parameterize_result(result)
            for phase in ('approach', 'contact_approach', 'extract', 'transport', 'conveyor_place', 'return_home'):
                original = np.asarray([row for row, label in zip(geometric['frames'], geometric['phases']) if label == phase])
                timed = np.asarray([row for row, label in zip(result['frames'], result['phases']) if label == phase])
                if not len(timed) or not np.allclose(original[[0, -1]], timed[[0, -1]], atol=1e-6, rtol=0):
                    raise ValueError(f'{phase}时间化改变阶段端点')
            result['geometric_measured_wall_ms'] = geometric['measured_wall_ms']
            result['geometric_frame_count'] = len(geometric['frames'])
            result['trajectory_postprocessing'] = dict(enabled=True, source='PR65', method='GPU validated shortcut and smooth time parameterization',
                sample_dt_s=self.trajectory_sample_dt, velocity_limits=self._timed_limits[0].tolist(),
                acceleration_limits=self._timed_limits[1].tolist(), jerk_limits=self._timed_limits[2].tolist(),
                endpoint_preserved=True, extraction_payload_path_validated=False)
            result['measured_wall_ms'] = (time.perf_counter() - began) * 1000
            return result
        except Exception as error:
            geometric.update(success=False, blocked_stage='轨迹后处理验收', blocked_details=str(error),
                             trajectory_postprocessing=dict(enabled=True, success=False))
            self.last_partial = geometric
            raise CycleBlocked('轨迹后处理验收', str(error), geometric) from error
        finally:
            self.snapshot = snapshot
