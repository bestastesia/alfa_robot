"""Build per-task tables and a standalone visual for three IK decisions plus final task outcome."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


STAGES = {0: "接触 IK", 1: "回中/携箱 IK", 2: "放箱 IK", 3: "最终任务"}
TERMS = ("approach_s", "extraction_s", "nearest_carry_s", "carry_mean_excess_s", "carry_scarcity_s",
         "transport_s", "home_proxy_s", "updown_soft_s", "placement_s", "return_home_s",
         "joint_limit_soft_s", "approach_direct_risk_s", "carry_direct_risk_s",
         "placement_direct_risk_s", "home_direct_risk_s")
TIMES = ("generation_ms", "feature_ms", "lookahead_ms", "risk_ms", "rank_ms", "screen_ms", "step_wall_ms")


def finite(value):
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def load_arm(spec):
    arm_id, separator, rest = spec.partition("=")
    label, separator2, directory = rest.partition(":")
    if not separator or not separator2:
        raise ValueError("arm must be ID=LABEL:INSTANCES_DIR")
    paths = sorted(Path(directory).glob("*.json"))
    rows = [json.loads(path.read_text()) for path in paths]
    if len(rows) != 96 or len({row["instance"] for row in rows}) != 96:
        raise ValueError(f"{arm_id} requires exactly 96 unique default-size instances")
    return arm_id, label, rows


def outcome_row(arm_id, label, episode, stage):
    outcomes = {int(row["stage"]): row for row in episode.get("outcomes", [])}
    outcome = outcomes.get(stage)
    common = dict(function_id=arm_id, function_label=label, instance=episode["instance"], mode=episode["mode"],
                  stage=stage, stage_label=STAGES[stage], episode_success=bool(episode["success"]),
                  status=episode["status"], failure_stage=episode.get("failure_stage"), reason=episode.get("reason"),
                  selection_total_ms=finite(episode.get("selection_total_ms")),
                  planning_wall_ms=finite(episode.get("planning_wall_ms")),
                  raw_t_path_s=finite(episode.get("raw_t_path_s")),
                  executed_t_path_s=finite(episode.get("executed_t_path_s")),
                  motion_duration_s=finite(episode.get("motion_duration_s")))
    if stage == 3:
        return {**common, "reached": True, "stage_pass": bool(episode["success"]), "stage_class": episode["status"],
                "three_ik_indices": "/".join(str(row.get("action_index")) for row in episode.get("outcomes", [])),
                "three_ik_updown_m": "/".join(str(finite(row.get("info", {}).get("action_q", [None])[0]))
                                                for row in episode.get("outcomes", []))}
    if outcome is None:
        return {**common, "reached": False, "stage_pass": False, "stage_class": "not_reached"}
    info, screening, cost = outcome.get("info", {}), outcome.get("screening", {}), outcome.get("b1_cost", {})
    params, q = outcome.get("action_params", []), info.get("action_q", [])
    terms, risks = cost.get("selected_terms_s", {}), cost.get("selected_direct_risk", {})
    row = {**common, "reached": True, "stage_pass": info.get("class_") in ("stage_success", "success"),
           "stage_class": info.get("class_"), "stage_reason": info.get("reason"),
           "candidate_count": outcome.get("candidate_count"), "action_index": outcome.get("action_index"),
           "action_rank": outcome.get("action_rank"), "u_normalized": finite(params[0]) if len(params) > 0 else None,
           "updown_m": finite(q[0]) if q else None,
           "left_swivel_deg": finite(params[1] * 180.) if len(params) > 1 else None,
           "right_swivel_deg": finite(params[2] * 180.) if len(params) > 2 else None,
           "selected_cost_s": finite(cost.get("selected_total_s")),
           "joint_limit_margin_fraction": finite(cost.get("selected_joint_limit_margin_fraction")),
           "screen_checks": screening.get("checks"), "screen_valid": screening.get("valid"),
           "screen_rejected": sum(screening.get("rejected", {}).values()),
           "screen_rejected_json": json.dumps(screening.get("rejected", {}), ensure_ascii=False, separators=(",", ":")),
           "stage_cost_s": finite(info.get("stage_cost_s")), "cumulative_cost_s": finite(info.get("cumulative_cost_s"))}
    for name in TERMS:
        row[name] = finite(terms.get(name))
    for name in ("approach", "carry", "place", "home"):
        row["risk_" + name] = finite(risks.get(name))
    for name in TIMES:
        row[name] = finite(screening.get("screen_ms")) if name == "screen_ms" else finite(outcome.get(name))
    for index in range(15):
        row[f"q{index}"] = finite(q[index]) if len(q) > index else None
    return row


def percentile(values, value):
    values = [float(item) for item in values if item is not None and math.isfinite(float(item))]
    return float(np.percentile(values, value)) if values else None


def summarize(function_id, label, rows, stage, mode):
    selected = [row for row in rows if row["function_id"] == function_id and row["stage"] == stage and
                (mode == "all" or row["mode"] == mode)]
    reached = [row for row in selected if row["reached"]]
    passed = [row for row in selected if row["stage_pass"]]
    result = dict(function_id=function_id, function_label=label, stage=stage, stage_label=STAGES[stage], mode=mode,
                  tasks=len(selected), reached=len(reached), passed=len(passed),
                  reach_rate=len(reached) / len(selected) if selected else None,
                  pass_rate_total=len(passed) / len(selected) if selected else None,
                  pass_rate_conditional=len(passed) / len(reached) if reached else None)
    metrics = ("candidate_count", "action_rank", "updown_m", "left_swivel_deg", "right_swivel_deg",
               "selected_cost_s", "joint_limit_margin_fraction", "generation_ms", "lookahead_ms", "risk_ms",
               "rank_ms", "screen_ms", "step_wall_ms", "selection_total_ms", "planning_wall_ms", "raw_t_path_s",
               "executed_t_path_s", "motion_duration_s")
    for name in metrics:
        values = [row.get(name) for row in reached]
        result[name + "_p10"] = percentile(values, 10)
        result[name + "_p50"] = percentile(values, 50)
        result[name + "_p90"] = percentile(values, 90)
        result[name + "_p95"] = percentile(values, 95)
    return result


def write_csv(path, rows, fields):
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def html_document(data):
    payload = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    template = '''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>三种 IK + 最终任务参数拉表</title>
<style>
:root{{--bg:#f5f7fb;--p:#fff;--t:#16202c;--m:#647185;--l:#d8e0ea;--b:#2878d4;--g:#16835e;--o:#c76b14;--r:#bd4148;--ink:#17324d;--soft:#eef3f7}}@media(prefers-color-scheme:dark){{:root{{--bg:#11161d;--p:#1a2029;--t:#edf2f8;--m:#aeb8c7;--l:#35404f;--b:#70aaf3;--g:#58c69a;--o:#eda25c;--r:#ef8086;--ink:#d9e8f6;--soft:#222b36}}}}*{{box-sizing:border-box}}body{{margin:0;background:var(--bg);color:var(--t);font:14px/1.5 system-ui,"Microsoft YaHei",sans-serif}}main{{max-width:1500px;margin:auto;padding:22px}}h1{{margin:0;font-size:30px}}h2{{margin-top:32px;font-size:22px}}.lead,.muted{{color:var(--m)}}code{{font:13px/1.55 ui-monospace,SFMono-Regular,Consolas,monospace;color:var(--ink)}}.foundation{{display:grid;grid-template-columns:minmax(320px,1.15fr) minmax(300px,.85fr);gap:12px;margin:14px 0}}.explain{{background:var(--p);border:1px solid var(--l);padding:16px}}.explain h3,.function h3{{margin:0 0 8px;color:var(--ink)}}.formula{{background:var(--soft);border-left:4px solid var(--b);padding:10px 12px;margin:10px 0;overflow:auto}}.functions{{display:grid;grid-template-columns:repeat(auto-fit,minmax(330px,1fr));gap:11px}}.function{{background:var(--p);border:1px solid var(--l);padding:15px;min-height:220px}}.function.selected{{border-top:4px solid var(--b)}}.function.candidate{{border-top:4px solid var(--g)}}.function p{{margin:7px 0}}.function .formula{{min-height:58px}}.tag{{display:inline-block;padding:2px 7px;margin-left:6px;border:1px solid var(--l);color:var(--m);font-size:12px;font-weight:400;vertical-align:2px}}.tag.keep{{color:var(--b);border-color:var(--b)}}.tag.try{{color:var(--g);border-color:var(--g)}}.b2{{background:var(--p);border:1px solid var(--l);padding:16px;margin-top:12px}}.b2grid{{display:grid;grid-template-columns:1fr 1fr;gap:12px}}.b2table{{overflow:auto;border:1px solid var(--l);margin-top:13px}}.b2table table{{min-width:980px}}.warning{{border-left:4px solid var(--o);background:color-mix(in srgb,var(--o) 10%,var(--p));padding:10px 12px}}.rule{{border:0;border-top:1px solid var(--l);margin:28px 0}}.controls{{display:flex;flex-wrap:wrap;gap:12px;background:var(--p);border:1px solid var(--l);padding:12px;margin:16px 0}}label{{display:flex;gap:7px;align-items:center}}select{{padding:7px;background:var(--p);color:var(--t);border:1px solid var(--l)}}.stats{{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px}}.stat{{background:var(--p);border:1px solid var(--l);padding:13px}}.stat b{{display:block;font-size:22px}}.tabs{{display:flex;gap:8px;flex-wrap:wrap;margin:18px 0}}button{{border:1px solid var(--l);background:var(--p);color:var(--t);padding:9px 14px;cursor:pointer}}button.on{{background:var(--b);color:white}}.bars{{background:var(--p);border:1px solid var(--l);padding:15px}}.barrow{{display:grid;grid-template-columns:150px 1fr 70px;gap:9px;align-items:center;margin:8px 0}}.track{{height:19px;background:color-mix(in srgb,var(--m) 18%,transparent)}}.bar{{height:100%;background:var(--b)}}.best{{background:var(--g)}}.tablewrap{{overflow:auto;max-height:620px;border:1px solid var(--l);background:var(--p)}}table{{border-collapse:collapse;width:100%;min-width:1250px}}th,td{{padding:7px 9px;border-bottom:1px solid var(--l);white-space:nowrap;text-align:right}}th{{position:sticky;top:0;background:var(--p);z-index:2;color:var(--m)}}th:first-child,td:first-child,th:nth-child(2),td:nth-child(2){{text-align:left}}tr.fail td{{color:var(--r)}}tr.pass td:first-child{{border-left:4px solid var(--g)}}.termgrid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:10px}}.term{{background:var(--p);border:1px solid var(--l);padding:12px}}.term h3{{margin:0 0 8px}}@media(max-width:850px){{.foundation,.b2grid{{grid-template-columns:1fr}}}}@media(max-width:700px){{main{{padding:14px}}.barrow{{grid-template-columns:105px 1fr 55px}}.functions{{grid-template-columns:1fr}}}}
</style></head><body><main><h1>三种 IK + 最终任务参数拉表</h1><p class="lead">7 种自定义函数 × 96 个任务。切换阶段后同时查看成功率、候选参数、代价分项、选择时间和完整任务结果。</p>
<h2>共同的数学基础</h2>
<div class="foundation">
  <section class="explain"><h3>所有运动项统一成“秒”</h3><p>关节构型记为 <code>q</code>，关节速度上限记为 <code>v</code>。两个构型之间的解析时间下界为：</p><div class="formula"><code>T_lb(q_a,q_b) = max_j |q_b[j] - q_a[j]| / v[j]</code></div><p>它表示所有关节同时按速度上限运动时，最慢关节至少需要多久。这样关节变化、升降变化、风险惩罚都能用“等效秒”相加，数值含义可解释。</p></section>
  <section class="explain"><h3>先硬约束，再软排序</h3><p>IK/Pose、真实关节限位、接触/携箱碰撞、完整抽离、放置门禁和最终 RRT 校核都是硬约束，不合格候选不能执行。</p><p>下面 7 个函数只给剩余合法候选排序。直连风险使用 9 个关节插值采样点：</p><div class="formula"><code>rho = 1 - 合法采样点数 / 9<br>风险代价 = 2.0 × rho 秒</code></div><p>直线碰撞不代表绕行 RRT 必然失败，因此 <code>rho</code> 是软风险，不是成功判据。</p></section>
</div>
<h2>7 种自定义函数</h2>
<div class="functions">
  <article class="function selected"><h3>B1 v2b <span class="tag keep">当前保留方案</span></h3><p><strong>原理：</strong>每个阶段都用当前运动时间加便宜的后续前瞻。放置阶段再惩罚放箱直连和回 Home 直连中的碰撞比例。</p><div class="formula"><code>J_contact = T(Home,qc) + T(qc,qe) + min_k T(qe,qk)<br>J_carry = T(qe,qk)<br>J_place = T(qk,qp) + T(qp,Home) + 2rho_place + 2rho_home</code></div><p><code>qc/qe/qk/qp</code> 分别是接触、抽离终点、携箱和放置构型。默认尺寸结果为 75/96。</p></article>
  <article class="function"><h3>限位保护</h3><p><strong>原理：</strong>继承 v2b，并对靠近任一关节上下限的候选增加连续惩罚。归一化最小限位裕度为：</p><div class="formula"><code>m(q)=min_j min((q[j]-l[j])/(u[j]-l[j]), (u[j]-q[j])/(u[j]-l[j]))<br>P_limit=2 × clip((0.05-m)/0.05, 0, 1)</code></div><p>裕度 ≥5% 不加分；从 5% 靠近限位时线性增加；到限位加满 2 秒。真实越界仍由硬约束拒绝。结果 75/96，但执行 T_lb 中位升至 29.98 秒。</p></article>
  <article class="function"><h3>升降稳定</h3><p><strong>原理：</strong>继承 v2b，在携箱构型选择时减少升降轴 Updown 相对上一阶段的变化。</p><div class="formula"><code>P_updown = 10 × |updown_carry - updown_previous| 秒</code></div><p>每变化 0.1 m 增加 1 秒。它表达“少升降可能更稳定”的偏好，但不会改变升降合法范围。结果 74/96，说明少动并不一定带来更好的空间连通性。</p></article>
  <article class="function"><h3>候选池鲁棒</h3><p><strong>原理：</strong>继承 v2b，并在接触阶段偏好“后续有多个相近好解”的接触构型，而不只看唯一最近携箱解。</p><div class="formula"><code>t_min = 最快合法携箱 T_lb<br>t_mean8 = 前 8 个最快合法携箱 T_lb 的均值<br>r = 合法携箱池中 T_lb ≤ t_min+0.5s 的占比<br>P_pool = 0.5 × max(t_mean8-t_min,0) + 1.0 × (1-r)</code></div><p>第一项惩罚候选质量断层，第二项惩罚近优解稀缺。结果 74/96：候选多不等于最终选中的携箱终点更适合放置。</p></article>
  <article class="function candidate"><h3>回中偏置 <span class="tag try">最佳开发候选</span></h3><p><strong>原理：</strong>继承 v2b，在携箱阶段把“离 Home 近”作为下游姿态质量的廉价代理。</p><div class="formula"><code>P_home_proxy = 0.25 × T_lb(q_carry, Home)</code></div><p>只使用四分之一权重，避免为回中牺牲过多当前运输代价。默认尺寸为 76/96；但五尺寸仅比 v2b 净增 1/480 且有 11 个结果互换，因此尚未晋升。</p></article>
  <article class="function"><h3>回程优先</h3><p><strong>原理：</strong>继承 v2b，在放置阶段把“放置构型回 Home”的运动时间权重从 1.0 提高到 1.5。</p><div class="formula"><code>J_place = T(q_carry,q_place) + 1.5 × T(q_place,Home)<br>　　　　+ 2rho_place + 2rho_home</code></div><p>等价于在 v2b 上额外增加 <code>0.5 × T(q_place,Home)</code>。结果仍为 75/96，说明它与已有回 Home 项和直连风险大体重复。</p></article>
  <article class="function"><h3>均衡组合</h3><p><strong>原理：</strong>把前面各独立偏好以半强度一次性组合，权重在看结果前冻结，不做事后调参。</p><div class="formula"><code>P_combo = 0.25×mean_excess + 0.5×scarcity<br>　　　　+ 0.125×T(carry,Home) + 5×DeltaUpdown<br>　　　　+ 1.0×limit_shortfall<br>放置回 Home 系数 = 1.25</code></div><p>v2b 的放置/Home 直连风险仍各为 2 秒。结果 75/96，选择 P95 最快为 262.57 ms，但执行 T_lb 中位增加到 28.92 秒，说明合理软项不能默认直接相加。</p></article>
</div>
<h2>B2：把 3 个 IK 当成一组评分</h2>
<section class="b2"><div class="b2grid"><div><h3>完整组级代价</h3><p>选择对象从单个 IK 改为 <code>g=(q_contact,q_carry,q_place)</code>，实际链包含接触派生的抽离终点：</p><div class="formula"><code>Home → contact → extraction → carry → placement → Home<br><br>J_group = J_motion + 0.75 P_turn + 1.00 P_reverse<br>　　　　+ 0.50 P_redundancy + 2.00 P_collision + 1.00 P_limit</code></div><p>所有权重都是等效秒。内部按低代价排序，页面分数用 <code>100 exp(-(J-J_best)/5s)</code> 转成“越高越好”。</p><p><strong>分层 beam：</strong>开发消融从 16 个 pair 缩减到 8 个且四任务选择不变；每对独立生成 64 个放置候选，最终最多评价 512 个完整组。</p></div><div><h3>为什么不能只看顺畅</h3><p><code>P_turn</code> 衡量相邻段归一化关节运动模式变化，<code>P_reverse</code> 衡量关节往返，<code>P_redundancy</code> 衡量 Updown 和双臂旋转的二阶变化。</p><p class="warning">真实 GPU 证据表明端点平滑不能稳定预测规划轨迹 jerk、总关节变化或反向次数，因此 dev96 候选 B2-time 已将三个端点平滑权重设为零。</p></div></div><div id="b2summary" class="b2table"></div><h3>真实 GPU 四模式同源配对</h3><div id="b2smoke" class="b2table"></div><h3>默认尺寸 dev96 同源配对</h3><div id="b2dev96" class="b2table"></div><p class="warning">B2-time-beam8 仅在 dual-side 净增 1/36；总体为 76/96 对 75/96，但有 5 个结果互换，轨迹代价中位不改善且总墙钟增加。当前正式选择仍是 v2b，独立 test 未解封。</p></section>
<hr class="rule">
<h2>逐阶段结果与参数</h2>
<div class="tabs" id="tabs"></div><div class="controls"><label>函数<select id="fn"></select></label><label>模式<select id="mode"></select></label><label>结果<select id="result"><option value="all">全部</option><option value="success">最终成功</option><option value="fail">最终失败</option></select></label></div>
<div class="stats" id="stats"></div><h2>函数横向对比</h2><div class="bars" id="bars"></div><h2>当前函数的阶段参数与代价</h2><div class="termgrid" id="terms"></div><h2>逐任务明细</h2><div class="tablewrap"><table><thead id="thead"></thead><tbody id="tbody"></tbody></table></div>
<script>const D=__PAYLOAD__;let stage=0;const fns=[...new Map(D.rows.map(x=>[x.function_id,x.function_label]))];const modes=['all',...new Set(D.rows.map(x=>x.mode))];const $=id=>document.getElementById(id);function fmt(v,d=2){{return v==null?'—':Number(v).toFixed(d)}}function pc(v){{return v==null?'—':(v*100).toFixed(1)+'%'}}
function init(){{drawB2();['接触 IK','回中/携箱 IK','放箱 IK','最终任务'].forEach((x,i)=>{{let b=document.createElement('button');b.textContent=x;b.onclick=()=>{{stage=i;draw()}};$('tabs').appendChild(b)}});fns.forEach(([id,n])=>$('fn').add(new Option(n,id)));modes.forEach(x=>$('mode').add(new Option(x==='all'?'全部模式':x,x)));['fn','mode','result'].forEach(x=>$(x).onchange=draw);draw()}}
function drawB2(){{let d=D.group_diagnostic;if(!d){{$('b2summary').innerHTML='<p class="muted">尚未载入历史三元组诊断。</p>';return}}let h=['函数','共同任务成功','顺畅度 P50','顺畅等效代价 P50','最顺/并列次数','平均顺畅排名','转向变化 P50','反向幅度 P50','冗余曲率 P50'];$('b2summary').innerHTML='<table><thead><tr>'+h.map(x=>`<th>${{x}}</th>`).join('')+'</tr></thead><tbody>'+d.summary.map(x=>`<tr><td>${{x.function_label}}</td><td>${{x.final_success_common}} / ${{x.common_tasks}}</td><td>${{fmt(x.smoothness_score_p50,2)}}</td><td>${{fmt(x.smoothness_cost_s_p50,3)}} s</td><td>${{x.smoothest_or_tied_count}}</td><td>${{fmt(x.smoothness_rank_mean,2)}}</td><td>${{fmt(x.turn_fraction_p50,3)}}</td><td>${{fmt(x.reversal_fraction_p50,3)}}</td><td>${{fmt(x.redundancy_curvature_p50,3)}}</td></tr>`).join('')+'</tbody></table>'}}
function drawB2Smoke(){{let d=D.b2_paired_smoke;if(!d){{$('b2smoke').innerHTML='<p class="muted">尚未载入 GPU 配对结果。</p>';return}}let h=['模式','B2选择','选择差','规划差','执行T_lb差','运动时长差','jerk差','关节变差','反向次数差'];$('b2smoke').innerHTML='<table><thead><tr>'+h.map(x=>`<th>${{x}}</th>`).join('')+'</tr></thead><tbody>'+d.rows.map(x=>`<tr><td>${{x.mode}}</td><td>${{fmt(x.b2_selection_ms,0)}} ms</td><td>${{fmt(x.delta_selection_ms,0)}} ms</td><td>${{fmt(x.delta_planning_ms,0)}} ms</td><td>${{fmt(x.delta_executed_t_lb_s,3)}} s</td><td>${{fmt(x.delta_motion_duration_s,3)}} s</td><td>${{fmt(x.delta_jerk_ratio,5)}}</td><td>${{fmt(x.delta_normalized_joint_variation,3)}}</td><td>${{fmt(x.delta_joint_direction_reversals,0)}}</td></tr>`).join('')+'</tbody></table>'}};drawB2Smoke()
function drawB2Dev96(){{let d=D.b2_dev96_paired;if(!d){{$('b2dev96').innerHTML='<p class="muted">尚未载入 dev96 配对结果。</p>';return}}let a=d.v2b,b=d.b2_time_beam8,p=d.paired_b2_minus_v2b;let rows=[['严格成功',`${{a.success}} / 96`,`${{b.success}} / 96`,'+1'],['选择 P50 / P95',`${{fmt(a.selection.p50,0)}} / ${{fmt(a.selection.p95,0)}} ms`,`${{fmt(b.selection.p50,0)}} / ${{fmt(b.selection.p95,0)}} ms`,`${{fmt(p.selection_ms.p50,0)}} ms`],['选择+规划 P50',`${{fmt(a.success_metrics.total_wall_ms.p50,0)}} ms`,`${{fmt(b.success_metrics.total_wall_ms.p50,0)}} ms`,`${{fmt(p.total_wall_ms.p50,0)}} ms`],['执行 T_lb P50',`${{fmt(a.success_metrics.executed_t_path_s.p50,2)}} s`,`${{fmt(b.success_metrics.executed_t_path_s.p50,2)}} s`,`${{fmt(p.executed_t_path_s.p50,2)}} s`],['运动时长 P50',`${{fmt(a.success_metrics.motion_duration_s.p50,2)}} s`,`${{fmt(b.success_metrics.motion_duration_s.p50,2)}} s`,`${{fmt(p.motion_duration_s.p50,2)}} s`],['jerk 比 P50',fmt(a.success_metrics.jerk_ratio.p50,5),fmt(b.success_metrics.jerk_ratio.p50,5),fmt(p.jerk_ratio.p50,5)]];$('b2dev96').innerHTML='<table><thead><tr><th>指标</th><th>v2b</th><th>B2-time beam8</th><th>B2-v2b 配对中位</th></tr></thead><tbody>'+rows.map(r=>'<tr>'+r.map(x=>`<td>${{x}}</td>`).join('')+'</tr>').join('')+'</tbody></table>'}};drawB2Dev96()
function filtered(){{return D.rows.filter(x=>x.stage===stage&&x.function_id===$('fn').value&&($('mode').value==='all'||x.mode===$('mode').value)&&($('result').value==='all'||(x.episode_success? 'success':'fail')===$('result').value))}}
function summary(id){{return D.summary.find(x=>x.function_id===id&&x.stage===stage&&x.mode===$('mode').value)}}function draw(){{[...$('tabs').children].forEach((b,i)=>b.classList.toggle('on',i===stage));let s=summary($('fn').value),rows=filtered();$('stats').innerHTML=stage<3?`<div class="stat"><span>到达该阶段</span><b>${{s?.reached??0}} / ${{s?.tasks??0}}</b><small>${{pc(s?.reach_rate)}}</small></div><div class="stat"><span>阶段通过</span><b>${{s?.passed??0}}</b><small>条件通过 ${{pc(s?.pass_rate_conditional)}}</small></div><div class="stat"><span>Updown 中位</span><b>${{fmt(s?.updown_m_p50,3)}} m</b><small>P10–P90 ${{fmt(s?.updown_m_p10,3)}}～${{fmt(s?.updown_m_p90,3)}}</small></div><div class="stat"><span>阶段规划中位</span><b>${{fmt(s?.step_wall_ms_p50,0)}} ms</b><small>P95 ${{fmt(s?.step_wall_ms_p95,0)}} ms</small></div>`:`<div class="stat"><span>完整成功</span><b>${{s?.passed??0}} / ${{s?.tasks??0}}</b><small>${{pc(s?.pass_rate_total)}}</small></div><div class="stat"><span>三次选择总耗时</span><b>${{fmt(s?.selection_total_ms_p50,0)}} ms</b><small>P95 ${{fmt(s?.selection_total_ms_p95,0)}} ms</small></div><div class="stat"><span>执行 T_lb 中位</span><b>${{fmt(s?.executed_t_path_s_p50)}} s</b><small>P10–P90 ${{fmt(s?.executed_t_path_s_p10)}}～${{fmt(s?.executed_t_path_s_p90)}}</small></div><div class="stat"><span>完整规划中位</span><b>${{fmt(s?.planning_wall_ms_p50,0)}} ms</b><small>P95 ${{fmt(s?.planning_wall_ms_p95,0)}} ms</small></div>`;drawBars();drawTerms(s);drawTable(rows)}}
function drawBars(){{let vals=fns.map(([id,n])=>[id,n,summary(id)]),max=Math.max(...vals.map(x=>x[2]?.pass_rate_total||0),.01);$('bars').innerHTML=vals.map(([id,n,s])=>`<div class="barrow"><span>${{n}}</span><div class="track"><div class="bar ${{id==='home_bias'?'best':''}}" style="width:${{100*(s?.pass_rate_total||0)/max}}%"></div></div><b>${{pc(s?.pass_rate_total)}}</b></div>`).join('')}
function drawTerms(s){{if(stage===3){{$('terms').innerHTML=`<div class="term"><h3>完整耗时</h3>选择 P50/P95：${{fmt(s?.selection_total_ms_p50,0)}} / ${{fmt(s?.selection_total_ms_p95,0)}} ms<br>规划 P50/P95：${{fmt(s?.planning_wall_ms_p50,0)}} / ${{fmt(s?.planning_wall_ms_p95,0)}} ms</div><div class="term"><h3>完整轨迹</h3>Raw T_lb：${{fmt(s?.raw_t_path_s_p50)}} s<br>执行 T_lb：${{fmt(s?.executed_t_path_s_p50)}} s<br>运动时长：${{fmt(s?.motion_duration_s_p50)}} s</div>`;return}}$('terms').innerHTML=`<div class="term"><h3>候选与筛选</h3>候选数 P50：${{fmt(s?.candidate_count_p50,0)}}<br>选中排名 P50/P90：${{fmt(s?.action_rank_p50,0)}} / ${{fmt(s?.action_rank_p90,0)}}</div><div class="term"><h3>冗余参数</h3>Updown：${{fmt(s?.updown_m_p50,3)}} m<br>左ψ：${{fmt(s?.left_swivel_deg_p50,1)}}°<br>右ψ：${{fmt(s?.right_swivel_deg_p50,1)}}°</div><div class="term"><h3>选择时间</h3>生成：${{fmt(s?.generation_ms_p50,1)}} ms<br>前瞻：${{fmt(s?.lookahead_ms_p50,1)}} ms<br>风险：${{fmt(s?.risk_ms_p50,1)}} ms<br>筛选：${{fmt(s?.screen_ms_p50,1)}} ms</div><div class="term"><h3>代价</h3>选中总代价：${{fmt(s?.selected_cost_s_p50)}} s<br>阶段实际 T_lb：${{fmt(s?.stage_cost_s_p50)}} s<br>限位裕度：${{fmt(s?.joint_limit_margin_fraction_p50,3)}}</div>`}}
function drawTable(rows){{let heads=stage<3?['任务','模式','最终','阶段','候选','排名','Updown m','左ψ°','右ψ°','代价 s','阶段T_lb s','生成 ms','前瞻 ms','风险 ms','筛选 ms','规划 ms','失败原因']:['任务','模式','最终','失败阶段','失败原因','3 IK索引','3 IK Updown','选择 ms','规划 ms','Raw T_lb s','执行T_lb s','运动时长 s'];$('thead').innerHTML='<tr>'+heads.map(x=>`<th>${{x}}</th>`).join('')+'</tr>';$('tbody').innerHTML=rows.map(x=>{{let a=stage<3?[x.instance,x.mode,x.episode_success?'成功':'失败',x.stage_pass?'通过':x.stage_class,x.candidate_count,x.action_rank,fmt(x.updown_m,3),fmt(x.left_swivel_deg,1),fmt(x.right_swivel_deg,1),fmt(x.selected_cost_s),fmt(x.stage_cost_s),fmt(x.generation_ms,1),fmt(x.lookahead_ms,1),fmt(x.risk_ms,1),fmt(x.screen_ms,1),fmt(x.step_wall_ms,0),x.stage_reason||x.reason||'']:[x.instance,x.mode,x.episode_success?'成功':'失败',x.failure_stage??'',x.reason||'',x.three_ik_indices||'',x.three_ik_updown_m||'',fmt(x.selection_total_ms,1),fmt(x.planning_wall_ms,0),fmt(x.raw_t_path_s),fmt(x.executed_t_path_s),fmt(x.motion_duration_s)];return `<tr class="${{x.episode_success?'pass':'fail'}}">`+a.map(v=>`<td>${{v??''}}</td>`).join('')+'</tr>'}}).join('')}};init();</script></main></body></html>'''
    return template.replace("{{", "{").replace("}}", "}").replace("__PAYLOAD__", payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", action="append", required=True, help="ID=LABEL:instances directory")
    parser.add_argument("--group-diagnostic", type=Path, help="optional B2 selected-triple diagnostic JSON")
    parser.add_argument("--b2-paired-smoke", type=Path, help="optional real GPU v2b/B2 paired smoke JSON")
    parser.add_argument("--b2-dev96-paired", type=Path, help="optional same-source v2b/B2 dev96 paired JSON")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    arms = [load_arm(spec) for spec in args.arm]
    rows = [outcome_row(arm_id, label, episode, stage) for arm_id, label, episodes in arms
            for episode in episodes for stage in range(4)]
    modes = ["all"] + sorted({row["mode"] for row in rows})
    summary = [summarize(arm_id, label, rows, stage, mode) for arm_id, label, _ in arms
               for stage in range(4) for mode in modes]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    long_fields = list(dict.fromkeys(key for row in rows for key in row))
    summary_fields = list(dict.fromkeys(key for row in summary for key in row))
    write_csv(args.output_dir / "three_ik_plus_final_long.csv", rows, long_fields)
    write_csv(args.output_dir / "three_ik_plus_final_summary.csv", summary, summary_fields)
    data = {"rows": rows, "summary": summary, "functions": [{"id": a, "label": b} for a, b, _ in arms],
            "group_diagnostic": (json.loads(args.group_diagnostic.read_text()) if args.group_diagnostic else None),
            "b2_paired_smoke": (json.loads(args.b2_paired_smoke.read_text()) if args.b2_paired_smoke else None),
            "b2_dev96_paired": (json.loads(args.b2_dev96_paired.read_text()) if args.b2_dev96_paired else None)}
    (args.output_dir / "three_ik_plus_final_data.json").write_text(json.dumps(data, ensure_ascii=False, indent=1) + "\n")
    (args.output_dir / "three_ik_plus_final_visual.html").write_text(html_document(data))


if __name__ == "__main__":
    main()
