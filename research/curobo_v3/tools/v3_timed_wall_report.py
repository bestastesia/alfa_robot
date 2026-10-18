#!/usr/bin/env python3
"""Generate a self-contained round/phase planning and motion timing report."""
import argparse
import html
import json
from pathlib import Path


PHASE_LABELS = {
    "approach": "初始→预接触", "contact_approach": "最后3cm贴近", "extract": "抽离35cm",
    "transport": "携箱运输", "conveyor_place": "固定底盘放箱", "return_home": "回Home",
}


def planning_ledger(result):
    pipeline = result.get("pipeline_timing_ms", {})
    lazy = result.get("lazy_extraction", {}).get("timings_ms", {})
    if "prepare_checkers" in pipeline:
        rows = [
            ("接触候选生成", lazy.get("contact_generation_ms")),
            ("抽离终点预测", lazy.get("endpoint_prediction_ms")),
            ("预测排序", lazy.get("prediction_ranking_ms")),
            ("检查器准备", pipeline.get("prepare_checkers")),
            ("抽离候选完整验收", pipeline.get("all_extract_and_validation")),
            ("放置目标匹配", pipeline.get("goal_matching")),
            ("多根/多目标搜索", pipeline.get("multiroot_search")),
            ("选中前后缀验收", pipeline.get("selected_prefix_suffix_validation")),
            ("组装、放箱、回Home及重复验收", pipeline.get("assembly_and_repeat_validation")),
        ]
    else:
        timing = result.get("timing_ms", {})
        rows = [
            ("单臂接触候选", timing.get("contact_candidates_ms")),
            ("单臂抽离与到位", timing.get("extraction_and_approach_ms")),
            ("单臂携箱运输", timing.get("transport_ms")),
            ("放箱与回Home后缀", result.get("conveyor_suffix_ms")),
            ("顶吸底盘前进验收", timing.get("base_advance_check")),
        ]
    rows = [(name, value) for name, value in rows if value is not None]
    remainder = result.get("total_ms", 0.) - sum(value for _, value in rows)
    if remainder > 0:
        rows.append(("其他框架与结果组装", remainder))
    return rows


def phase_planning_detail(result, phase):
    """Explain additive ownership separately from nested planner diagnostics."""
    pipeline = result.get("pipeline_timing_ms", {})
    lazy = result.get("lazy_extraction", {}).get("timings_ms", {})
    timing = result.get("timing_ms", {})
    attempts = result.get("attempts", [])
    selected_attempt = next((item for item in reversed(attempts) if item.get("approach")), {})
    approach = selected_attempt.get("approach", {})
    transport = result.get("transport_search_stats", {})
    place = result.get("conveyor_search_stats", {})
    home = result.get("conveyor_home_search_stats", {})
    validation = result.get("validation_detail_ms", {})
    if "prepare_checkers" in pipeline:
        additive = {
            "approach": [("接触候选生成", lazy.get("contact_generation_ms")),
                         ("检查器准备", pipeline.get("prepare_checkers"))],
            "contact_approach": [("选中前后缀联合验收", pipeline.get("selected_prefix_suffix_validation"))],
            "extract": [("抽离终点预测", lazy.get("endpoint_prediction_ms")),
                        ("预测排序", lazy.get("prediction_ranking_ms")),
                        ("抽离候选完整验收", pipeline.get("all_extract_and_validation"))],
            "transport": [("放置目标匹配", pipeline.get("goal_matching")),
                          ("多根/多目标搜索", pipeline.get("multiroot_search"))],
            "conveyor_place": [("放箱与回Home组装/重复验收（共享）",
                                pipeline.get("assembly_and_repeat_validation"))],
            "return_home": [("放箱与回Home组装/重复验收（共享）",
                             pipeline.get("assembly_and_repeat_validation"))],
        }.get(phase, [])
    else:
        shared = timing.get("extraction_and_approach_ms")
        additive = {
            "approach": [("单臂接触候选", timing.get("contact_candidates_ms")),
                         ("接触/抽离/到位联合规划（共享）", shared)],
            "contact_approach": [("接触/抽离/到位联合规划（共享）", shared)],
            "extract": [("接触/抽离/到位联合规划（共享）", shared)],
            "transport": [("单臂携箱运输搜索", timing.get("transport_ms"))],
            "conveyor_place": [("放箱与回Home后缀（共享）", result.get("conveyor_suffix_ms"))],
            "return_home": [("放箱与回Home后缀（共享）", result.get("conveyor_suffix_ms"))],
        }.get(phase, [])
    diagnostics = {
        "approach": [
            ("RRT首次找到路径", approach.get("first_solution_ms")),
            ("RRT首次完整验收通过", approach.get("first_verified_solution_ms")),
            ("RRT搜索总墙钟", approach.get("search_ms")),
            ("预接触规划+验证管线总墙钟", approach.get("contact_pipeline_ms")),
        ],
        "contact_approach": [
            ("共享的预接触规划+验证管线", approach.get("contact_pipeline_ms")),
        ],
        "extract": [
            ("抽离候选实际计算（包含于完整验收）", lazy.get("extraction_work_ms")),
        ],
        "transport": [
            ("RRT首次找到路径", transport.get("first_solution_ms")),
            ("RRT首次完整验收通过", transport.get("first_verified_solution_ms")),
            ("RRT搜索总墙钟", transport.get("search_ms")),
        ],
        "conveyor_place": [
            ("放箱RRT首次找到路径", place.get("first_solution_ms")),
            ("放箱RRT首次完整验收通过", place.get("first_verified_solution_ms")),
            ("放箱RRT搜索总墙钟", place.get("search_ms")),
            ("组装与全轨迹重复验收", validation.get("assembly_revalidate_actual")),
        ],
        "return_home": [
            ("回Home RRT首次找到路径", home.get("first_solution_ms")),
            ("回Home RRT首次完整验收通过", home.get("first_verified_solution_ms")),
            ("回Home RRT搜索总墙钟", home.get("search_ms")),
            ("组装与全轨迹重复验收（与放箱共享）", validation.get("assembly_revalidate_actual")),
        ],
    }.get(phase, [])
    notes = {
        "approach": ["上面的“核心规划归属账”不是纯RRT时间；RRT数据列在嵌套诊断中。"],
        "contact_approach": ["最后3cm由初始→接触的联合路径一次生成，没有独立再跑一遍RRT。"],
        "extract": ["抽离预测、排序和候选验收属于联合候选管线，不是单条RRT。"],
        "transport": ["多根/多目标搜索是该阶段主要RRT账；首次解、验收和总搜索互相嵌套。"],
        "conveyor_place": ["该共享项主要是组装与全轨迹重复碰撞验收，不是放箱RRT本身。"],
        "return_home": ["与放箱共享同一后缀总账；不能把共享项在两个阶段重复相加。"],
    }.get(phase, [])
    timing_detail = result.get("trajectory_timing_breakdown_ms", {}).get("phases", {}).get(phase, {})
    segment = next((item for item in result.get("timed_segments", []) if item.get("name") == phase), {})
    timing_diagnostics = []
    if "shortcut_edge_check_ms" in segment:
        timing_diagnostics = [
            f"快捷化路径点：{segment['raw_waypoints']} → {segment['shortcut_waypoints']}",
            f"精确候选边：{segment['shortcut_candidate_edges']}条，分{segment['shortcut_edge_calls']}批",
            f"GPU碰撞状态：{segment['shortcut_padded_states']}个（有效采样{segment['shortcut_actual_states']}个）",
            f"其中edge碰撞检查：{segment['shortcut_edge_check_ms']:.1f} ms",
        ]
    timing_labels = {
        "shortcut_ms": "路径快捷化/直连碰撞搜索",
        "parameterization_ms": "速度、加速度与jerk时间参数化",
        "collision_validation_ms": "时间化轨迹碰撞与阶段门禁验证",
        "assembly_ms": "采样结果组装与常量关节处理",
    }
    return {
        "additive": [(name, value) for name, value in additive if value is not None],
        "diagnostics": [(name, value) for name, value in diagnostics if value is not None],
        "notes": notes,
        "timing": [(timing_labels[key], timing_detail.get(key, 0.)) for key in timing_labels
                   if key in timing_detail],
        "timing_diagnostics": timing_diagnostics,
    }


def phase_planning(result, phase):
    return [(name, value) for name, value in phase_planning_detail(result, phase)["additive"]
            if value >= 10.]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    document = json.loads(args.input.read_text())
    rounds = []
    for number, record in enumerate(document["rounds"], 1):
        result = record["result"]
        phases = {segment["name"]: {
            "motion_s": segment["duration_s"], "frames": len(segment["positions"]),
            "parameterizer": segment.get("parameterizer"),
            "planning": phase_planning(result, segment["name"]),
        } for segment in result["timed_segments"]}
        rounds.append({"number": number, "task": record["task"], "planning_ms": result["total_ms"],
                       "timing_ms": result.get("trajectory_timing_ms", 0.),
                       "motion_s": result["motion_duration_s"], "ledger": planning_ledger(result),
                       "phases": phases, "validation": result["timed_validation"]})
    data = json.dumps(rounds, ensure_ascii=False).replace("</", "<\\/")
    options = "".join(f'<option value="{key}">{label}</option>' for key, label in PHASE_LABELS.items())
    page = f'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Mentor rebase · 全搬运耗时</title><style>
:root{{--bg:#f4f6f8;--card:#fff;--text:#17202a;--muted:#667085;--line:#d8dee6;--blue:#2563eb}}*{{box-sizing:border-box}}body{{margin:0;background:var(--bg);color:var(--text);font:14px/1.55 system-ui,sans-serif}}main{{max-width:1200px;margin:auto;padding:24px}}.card{{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:18px;margin-bottom:16px}}h1,h2{{margin:0 0 10px}}.controls{{display:flex;gap:12px;flex-wrap:wrap}}select{{padding:8px;border:1px solid var(--line);border-radius:7px;background:var(--card);color:var(--text)}}.metrics{{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}}.metric{{background:#edf3ff;padding:12px;border-radius:8px}}.metric b{{display:block;font-size:21px}}table{{width:100%;border-collapse:collapse}}th,td{{padding:8px;border-bottom:1px solid var(--line);text-align:left}}small,.muted{{color:var(--muted)}}@media(max-width:700px){{.metrics{{grid-template-columns:repeat(2,1fr)}}}}@media(prefers-color-scheme:dark){{:root{{--bg:#10151c;--card:#18212c;--text:#e7edf4;--muted:#a8b3c2;--line:#344253}}.metric{{background:#223148}}}}@media print{{.controls{{display:none}}}}
</style></head><body><main><section class="card"><h1>Mentor 主线重放 + 时间化轨迹</h1><p class="muted">15轮覆盖25箱；选择轮次和运动阶段查看运动时间、规划记录与约束利用率。</p><div class="controls"><label>轮次 <select id="round">{''.join(f'<option value="{i}">第{i+1}轮</option>' for i in range(len(rounds)))}</select></label><label>阶段 <select id="phase">{options}</select></label></div></section><section class="card"><div class="metrics" id="metrics"></div></section><section class="card"><h2>所选阶段</h2><div id="phase-detail"></div></section><section class="card"><h2>本轮规划总账</h2><p class="muted">下表各行可直接相加，等于本轮规划时间；小于10ms仍保留在总计但不显示。</p><div id="ledger"></div></section></main><script>
const rounds={data};const rl=document.querySelector('#round'),pl=document.querySelector('#phase');function table(rows){{return '<table><thead><tr><th>项目</th><th>耗时</th></tr></thead><tbody>'+rows.map(x=>`<tr><td>${{x[0]}}</td><td>${{x[1].toFixed(2)}} ms</td></tr>`).join('')+'</tbody></table>'}}function render(){{const r=rounds[+rl.value],p=r.phases[pl.value];document.querySelector('#metrics').innerHTML=`<div class="metric"><b>${{(r.planning_ms/1000).toFixed(3)}}s</b>本轮规划</div><div class="metric"><b>${{r.motion_s.toFixed(3)}}s</b>本轮运动</div><div class="metric"><b>${{(r.timing_ms/1000).toFixed(3)}}s</b>时间参数化</div><div class="metric"><b>${{Object.values(r.task).join(' / ')}}</b>箱号</div>`;document.querySelector('#phase-detail').innerHTML=p?`<p><b>运动时间：${{p.motion_s.toFixed(3)}}s</b> · ${{p.frames}}帧 · ${{p.parameterizer}}</p>${{p.planning.length?table(p.planning):'<p>该阶段没有独立计时；耗时包含在共享规划管线中。</p>'}}`:'<p>本轮没有该阶段。</p>';document.querySelector('#ledger').innerHTML=table(r.ledger.filter(x=>x[1]>=10));}}rl.onchange=pl.onchange=render;render();
</script></body></html>'''
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(page)
    print(args.output)


if __name__ == "__main__":
    main()
