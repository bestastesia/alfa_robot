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


def phase_planning(result, phase):
    pipeline = result.get("pipeline_timing_ms", {})
    lazy = result.get("lazy_extraction", {}).get("timings_ms", {})
    timing = result.get("timing_ms", {})
    if "prepare_checkers" in pipeline:
        mapping = {
            "approach": [("接触候选生成", lazy.get("contact_generation_ms")),
                         ("选中前后缀联合验收（共享）", pipeline.get("selected_prefix_suffix_validation"))],
            "contact_approach": [("选中前后缀联合验收（共享）", pipeline.get("selected_prefix_suffix_validation"))],
            "extract": [("终点预测", lazy.get("endpoint_prediction_ms")),
                        ("预测排序", lazy.get("prediction_ranking_ms")),
                        ("完整抽离验收", pipeline.get("all_extract_and_validation"))],
            "transport": [("目标匹配", pipeline.get("goal_matching")),
                          ("多根/多目标搜索", pipeline.get("multiroot_search"))],
            "conveyor_place": [("放箱RRT（组装阶段内部）", result.get("conveyor_search_stats", {}).get("search_ms")),
                               ("组装与重复验收（与回Home共享）", pipeline.get("assembly_and_repeat_validation"))],
            "return_home": [("回Home RRT（组装阶段内部）", result.get("conveyor_home_search_stats", {}).get("search_ms")),
                            ("组装与重复验收（与放箱共享）", pipeline.get("assembly_and_repeat_validation"))],
        }
    else:
        mapping = {
            "approach": [("抽离与到位联合规划（共享）", timing.get("extraction_and_approach_ms"))],
            "contact_approach": [("抽离与到位联合规划（共享）", timing.get("extraction_and_approach_ms"))],
            "extract": [("抽离与到位联合规划（共享）", timing.get("extraction_and_approach_ms"))],
            "transport": [("单臂携箱搜索", timing.get("transport_ms"))],
            "conveyor_place": [("放箱RRT", result.get("conveyor_search_stats", {}).get("search_ms")),
                               ("放箱与回Home后缀（共享）", result.get("conveyor_suffix_ms"))],
            "return_home": [("回Home RRT", result.get("conveyor_home_search_stats", {}).get("search_ms")),
                            ("放箱与回Home后缀（共享）", result.get("conveyor_suffix_ms"))],
        }
    return [(name, value) for name, value in mapping.get(phase, ()) if value is not None and value >= 10.]


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
