import json
import time
from pathlib import Path

from v3_polar_refined_cycle import PolarRefinedCyclePlanner
from v3_current_urdf_timed import TimedCurrentUrdfPlanner
from curobo_core.scene import digest


ROOT = Path(__file__).resolve().parents[1]
POLICY_PATH = ROOT / 'config/new_warp.json'


class NewWarpPlanner(PolarRefinedCyclePlanner):
    conveyor_release_drop_m = 0.
    pipeline_label = '新WARP · 球面软参考＋解析先验＋整段30轮＋Informed Connect'
    pipeline_description = '新WARP：球面变杆长软参考→128点平滑冗余角解析→整段联合30轮→合法先验引导Informed Connect→0.25°最终验收。参考碰撞不直接拒绝任务，无合法先验仍可搜索。'
    result_directory = ROOT / 'generated/new_warp_sync/live/geometric'

    def __init__(self, args=None, reference_source=None, release_box_center_z_m=None):
        self.strategy_policy = json.loads(POLICY_PATH.read_text())
        if self.strategy_policy['reference_points'] != 128 or self.strategy_policy['joint_adjustment_rounds'] != 30:
            raise ValueError('当前新WARP固定128点和30轮，配置不能与实现不一致')
        super().__init__(args, reference_source)
        self.release_box_center_z_m = (self.strategy_policy['release_box_center_z_m']
                                      if release_box_center_z_m is None else float(release_box_center_z_m))
        if not .2 < self.release_box_center_z_m < 2.4:
            raise ValueError('放箱箱心高度超出当前场景有效范围')
        self.placement_clearance_m = self.release_box_center_z_m - self.conveyor_height_m - self.box_fit['dimensions_m'][2] / 2

    def full_cycle(self, boxes, progress=lambda text: None):
        result = super().full_cycle(boxes, progress)
        result['strategy'] = dict(self.strategy_policy, release_box_center_z_m=self.release_box_center_z_m,
                                  policy_identity=digest((self.strategy_policy, self.release_box_center_z_m)), model_identity=self.model_id)
        result['conveyor_release_adjustment'] = dict(box_center_z_m=self.release_box_center_z_m,
                                                   source='config/new_warp.json absolute height, applied once')
        return result

    def validate_saved_result(self, result):
        expected = digest((self.strategy_policy, self.release_box_center_z_m))
        if result.get('strategy', {}).get('policy_identity') != expected:
            raise ValueError('保存结果的WARP策略或释放高度与当前配置不符，须重新计算')
        if result.get('model_id') != self.model_id:
            raise ValueError('保存结果的模型与当前模型不符')


class TimedNewWarpPlanner(TimedCurrentUrdfPlanner, NewWarpPlanner):
    pipeline_label = '新WARP（PR65后处理独立开关）'
    result_directory = ROOT / 'generated/new_warp_sync/live/timed'
    timed_validation_resolution_deg = .25

    def plan_geometric_cycle(self, boxes, progress):
        return NewWarpPlanner.full_cycle(self, boxes, progress)

    def full_cycle(self, boxes, progress=lambda text: None):
        if not self.trajectory_optimization_enabled:
            return NewWarpPlanner.full_cycle(self, boxes, progress)
        began = time.perf_counter()
        result = TimedCurrentUrdfPlanner.full_cycle(self, boxes, progress)
        if result.get('trajectory_postprocessing', {}).get('enabled'):
            result['trajectory_postprocessing']['geometric_strategy'] = 'new_warp_v1'
            result['measured_wall_ms'] = (time.perf_counter() - began) * 1000
        return result
