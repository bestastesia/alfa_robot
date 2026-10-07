from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from v3_full_cycle_demo import planning_statistics


class DemoStatisticsTest(unittest.TestCase):
    def test_chinese_recorded_timing_selected_attempt_and_missing_fields(self):
        result = {
            'total_ms': 1234.5, 'selected_mode_total_ms': 900.,
            'timing_ms': {'transport_pipeline_total': 100.},
            'transport_timing_breakdown_ms': {'rrt': 40.},
            'selected_candidate': 2, 'contact_ik_count': 8,
            'attempts': [{'candidate': 1, 'approach': {'first_solution_ms': 999.}},
                         {'candidate': 2, 'approach': {'first_solution_ms': 12., 'search_ms': 20.,
                          'raw_waypoints': 27, 'shortcut_waypoints': 10, 'shortcut_applied': True}}],
            'home_search_stats': {'fallback': True, 'failed_search': {'search_ms': 2000.}},
            'phases': ['home', 'approach', 'approach'], 'frames': [[0], [1], [2]],
        }
        text = planning_statistics(result)
        for expected in ('本轮累计规划耗时：1234.50 ms', '选中方案耗时：900.00 ms',
                         '接近路径搜索与检查', '40.00 ms', '27→10', '12.00 ms',
                         '不能重复累加', '不是动作执行耗时', '反向复用', '2000.00 ms',
                         '| 初始→吸附起点 | 2 |', '未记录'):
            self.assertIn(expected, text)
        self.assertNotIn('999.00 ms', text)
        empty = planning_statistics({})
        self.assertIn('本轮累计规划耗时：未记录', empty)
        self.assertNotIn('本轮累计规划耗时：0.00', empty)


if __name__ == '__main__':
    unittest.main()
