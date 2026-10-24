"""MOTION-311 outcome classes of a planning round (pure Python, shared by the label and A/B harnesses).

success                       the round planned completely
policy_infeasible             proven infeasible under the fixed contract: no fixed-psi analytic solution,
                              collision of a fixed configuration, straight-line/orientation tolerance,
                              parking or envelope gates
budget_exhausted              an RRT did not connect within its budget; censored, reported as
                              "not connected within budget", never as infeasible
solver_or_validation_failure  cuRobo numerical place IK found nothing, or a found path failed the dense
                              re-validation; censored
scene_infeasible              independent of the candidate: Home collision, base advance, no contact IK,
                              no collision-free placement solution
exception                     anything else (kept for review)
"""
CLASSES = ("success", "policy_infeasible", "budget_exhausted", "solver_or_validation_failure", "scene_infeasible", "exception")
CENSORED = ("budget_exhausted", "solver_or_validation_failure")
SCENE_STAGES = {"Home", "初始姿态", "底盘前进", "接触IK"}
SCENE_MARKERS = ("No legal analytic contacts", "没有合法", "无碰撞放置解")
POLICY_STAGES = {"抽离候选验收", "反算放置底盘位", "龙头车放置起点", "运输终点", "运输终点圆柱验收", "放箱IK碰撞"}
SEARCH_STAGES = {"龙头车放置RRT", "保持转身姿态回Home", "运输RRT"}
DENSE_MARKER = "密化"
APPROACH_POLICY_MARKERS = ("无解", "验收失败", "碰撞")


def classify(kind, result, trace=()):
    """Return (class, blocked stage) for a round result; trace lists single-arm cartesian/search calls."""
    if result is None:
        return "exception", None
    if result.get("success"):
        return "success", None
    stage = result.get("blocked_stage")
    details = str(result.get("blocked_details") or "")
    if stage in SCENE_STAGES:
        return "scene_infeasible", stage
    if stage == "planning_failure":
        return ("scene_infeasible" if any(marker in details for marker in SCENE_MARKERS) else "exception"), stage
    if stage in POLICY_STAGES:
        return "policy_infeasible", stage
    if stage == "龙头车放置目标IK":
        return ("solver_or_validation_failure" if kind == "dual" else "policy_infeasible"), stage
    if stage == "龙头车放置RRT细验":
        return "solver_or_validation_failure", stage
    if stage in SEARCH_STAGES:
        last = [call for call in trace if call.get("call") == "single_path"]
        dense = DENSE_MARKER in details or (bool(last) and DENSE_MARKER in str(last[-1].get("failure") or ""))
        return ("solver_or_validation_failure" if dense else "budget_exhausted"), stage
    if stage == "全链路候选验证":
        return dual_selection_class(result), stage
    if stage == "抽离与到位":
        return single_prefix_class(result, trace), stage
    return "exception", stage


def approach_class(failure):
    failure = str(failure or "")
    if DENSE_MARKER in failure:
        return "solver_or_validation_failure"
    if any(marker in failure for marker in APPROACH_POLICY_MARKERS):
        return "policy_infeasible"
    return "budget_exhausted"


def dual_selection_class(result):
    attempts = result.get("selection_attempts") or []
    if not attempts:
        return "budget_exhausted"
    last = attempts[-1]
    if last.get("rejected") == "Home到位":
        return approach_class((last.get("approach") or {}).get("failure"))
    return "exception"


def single_prefix_class(result, trace):
    if result.get("attempts"):
        return "policy_infeasible"
    backs = [call for call in trace if call.get("call") == "cartesian_single" and call.get("along_normal")]
    if not backs or not backs[-1].get("ok"):
        return "policy_infeasible"
    searches = [call for call in trace if call.get("call") == "single_path"]
    if not searches:
        return "policy_infeasible"
    return approach_class(searches[-1].get("failure")) if searches[-1].get("failure") else "budget_exhausted"
