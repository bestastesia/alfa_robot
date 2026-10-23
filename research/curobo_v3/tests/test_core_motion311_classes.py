from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from v3_label_classes import CENSORED, CLASSES, classify


def blocked(stage, details="", **extra):
    return dict(success=False, blocked_stage=stage, blocked_details=details, **extra)


class ClassificationTest(unittest.TestCase):
    def test_success_and_missing_result(self):
        self.assertEqual(classify("dual", dict(success=True)), ("success", None))
        self.assertEqual(classify("dual", None), ("exception", None))

    def test_timeouts_are_censored_not_infeasible(self):
        self.assertEqual(classify("dual", blocked("龙头车放置RRT", "2秒预算内未找到无碰撞路径"))[0], "budget_exhausted")
        self.assertEqual(classify("dual", blocked("全链路候选验证", selection_attempts=[]))[0], "budget_exhausted")
        approach = dict(root=0, rejected="Home到位", approach=dict(iterations=40))
        self.assertEqual(classify("dual", blocked("全链路候选验证", selection_attempts=[approach]))[0], "budget_exhausted")
        trace = [dict(call="single_path", ok=False, failure=None)]
        self.assertEqual(classify("single", blocked("运输RRT", "单臂15轴携箱搜索超时"), trace)[0], "budget_exhausted")
        self.assertTrue(set(CENSORED) <= set(CLASSES))

    def test_dense_validation_is_a_solver_failure(self):
        self.assertEqual(classify("dual", blocked("保持转身姿态回Home", "2秒内无路径：{'failure': '密化轨迹验收失败'}"))[0],
                         "solver_or_validation_failure")
        trace = [dict(call="single_path", ok=False, failure="单臂完整15轴密化碰撞验收失败")]
        self.assertEqual(classify("single", blocked("运输RRT", "单臂15轴携箱搜索超时"), trace)[0], "solver_or_validation_failure")
        self.assertEqual(classify("dual", blocked("龙头车放置目标IK"))[0], "solver_or_validation_failure")
        self.assertEqual(classify("single", blocked("龙头车放置目标IK"))[0], "policy_infeasible")

    def test_policy_failures(self):
        self.assertEqual(classify("dual", blocked("抽离候选验收"))[0], "policy_infeasible")
        approach = dict(rejected="Home到位", approach=dict(failure="left预抓取反向直线IK无解@0.010m"))
        self.assertEqual(classify("dual", blocked("全链路候选验证", selection_attempts=[approach]))[0], "policy_infeasible")
        self.assertEqual(classify("single", blocked("运输终点圆柱验收"))[0], "policy_infeasible")
        extraction = blocked("抽离与到位", attempts=[dict(candidate=1, rejected="抽离运动学/机器人碰撞/携箱终点碰撞")])
        self.assertEqual(classify("single", extraction)[0], "policy_infeasible")
        backoff = [dict(call="cartesian_single", along_normal=False, ok=True), dict(call="cartesian_single", along_normal=True, ok=False)]
        self.assertEqual(classify("single", blocked("抽离与到位"), backoff)[0], "policy_infeasible")

    def test_single_approach_search(self):
        trace = [dict(call="cartesian_single", along_normal=False, ok=True), dict(call="cartesian_single", along_normal=True, ok=True),
                 dict(call="single_path", ok=False, failure=None)]
        self.assertEqual(classify("single", blocked("抽离与到位"), trace)[0], "budget_exhausted")

    def test_scene_and_unknown(self):
        self.assertEqual(classify("single", blocked("接触IK", "单臂接触无合法候选"))[0], "scene_infeasible")
        self.assertEqual(classify("dual", blocked("底盘前进"))[0], "scene_infeasible")
        self.assertEqual(classify("dual", blocked("planning_failure", "No legal analytic contacts"))[0], "scene_infeasible")
        self.assertEqual(classify("dual", blocked("planning_failure", "something else"))[0], "exception")
        self.assertEqual(classify("dual", blocked("未知阶段"))[0], "exception")


if __name__ == "__main__":
    unittest.main()
