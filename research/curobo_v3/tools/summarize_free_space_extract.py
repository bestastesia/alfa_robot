"""CPU-only summary of paired free-space extraction experiments."""
import argparse
from collections import defaultdict
import html
import json
import math
from pathlib import Path
import statistics


def sign_p(values):
    signs = [value > 0 for value in values if abs(value) > 1e-9]
    if not signs:
        return 1.
    smaller = min(sum(signs), len(signs)-sum(signs))
    return min(1., 2*sum(math.comb(len(signs), k) for k in range(smaller+1))/2**len(signs))


def summarize(documents):
    cases, blocks, trials = defaultdict(list), defaultdict(list), []
    merged = {}
    import copy
    for document in documents:
        for raw in document['records']:
            key = raw['case'], raw['seed']
            if key not in merged:
                merged[key] = copy.deepcopy(raw)
                for variant in merged[key]['variants']:
                    variant['selected_campaign'] = 'initial'
                continue
            previous = merged[key]
            if raw['baseline'] != previous['baseline']:
                raise ValueError(f'Cannot pool different baseline solves for {key}')
            previous_xyz = next(v for v in previous['variants'] if v['name'] == 'adaptive_xyz_nonaxial')
            additional = next(v for v in raw['variants'] if v['name'] == 'adaptive_xyz_nonaxial')
            accepted = [v for v in (previous_xyz, additional) if v['usable']]
            winner = min(accepted, key=lambda v: v['whole_cycle_s']) if accepted else previous_xyz
            selected_campaign = previous_xyz.get('selected_campaign', 'initial') if winner is previous_xyz else 'clearance'
            combined = copy.deepcopy(winner)
            combined['selected_campaign'] = selected_campaign
            combined['attempts'] = previous_xyz.get('attempts', []) + [dict(a, campaign='clearance') for a in additional.get('attempts', [])]
            combined['wall_s'] = previous_xyz.get('wall_s', 0) + additional.get('wall_s', 0)
            previous['variants'] = [combined if v['name'] == 'adaptive_xyz_nonaxial' else v for v in previous['variants']]
    for record in merged.values():
        baseline = record['baseline']
        candidate = next((v for v in record['variants'] if v['name'] == 'adaptive_xyz_nonaxial'), {})
        collision_ok = bool(record.get('baseline_loaded_collision')) and all(
            item['invalid'] == 0 for item in record.get('baseline_loaded_collision', {}).values())
        paired = bool(baseline.get('success') and collision_ok and candidate.get('usable'))
        control = next((v for v in record['variants'] if v['name'] == 'short_x_control'), {})
        shared = [stage for stage in baseline['stages'] if stage['phase'] not in ('extract','transport')]
        shared_ok = len(shared) == 4 and all(stage.get('trajectory_optimization',{}).get('solver_success',True)
            and stage.get('time_optimization',{}).get('success') and stage.get('time_optimization',{}).get('dynamic_valid') for stage in shared)
        trial = dict(baseline_eligible=bool(baseline.get('success') and collision_ok), candidate_full_success=bool(shared_ok and candidate.get('usable')), control_usable=control.get('usable', False), control_motion_s=control.get('whole_cycle_s'), case=record['case'], seed=record['seed'], baseline_success=baseline.get('success', False),
            baseline_motion_s=baseline.get('motion_s'), baseline_loaded_s=record.get('baseline_loaded_s'),
            candidate_usable=candidate.get('usable', False), paired=paired,
            candidate_motion_s=candidate.get('whole_cycle_s'),
            candidate_loaded_s=candidate.get('timing', {}).get('duration_s'),
            improvement_pct=candidate.get('whole_cycle_improvement_pct'),
            selected_campaign=candidate.get('selected_campaign', 'initial'), candidate_compute_s=candidate.get('wall_s'), selected_rank=candidate.get('selected_rank'),
            displacement_m=candidate.get('displacement_m'),
            eligible_endpoint_count=candidate.get('eligible_endpoint_count'),
            error=candidate.get('error'), attempts=[dict(rank=a['rank'], usable=a['usable'],
                displacement_m=a['displacement_m'], duration_s=a.get('duration_s'),
                lm_success=a['lm']['success'], collision=a.get('collision'),
                native_geometry_success=a.get('geometry', {}).get('solver_success'),
                campaign=a.get('campaign','initial'), failure=a.get('failure') or a['lm'].get('failure'), search_start_valid=a.get('inflated_search_start_valid'), search_solutions=a.get('search',{}).get('solutions_found'), wall_s=a.get('wall_s')) for a in candidate.get('attempts', [])])
        cases[trial['case']].append(trial)
        trials.append(trial)
        if paired:
            blocks[trial['seed']].append(trial['improvement_pct'])
    groups = []
    for name, group in cases.items():
        accepted = [trial for trial in group if trial['paired']]
        improvements = [trial['improvement_pct'] for trial in accepted]
        groups.append(dict(case=name, attempted=len(group), baseline_success=sum(t['baseline_success'] for t in group),
            candidate_usable=sum(t['candidate_usable'] for t in group),
            candidate_full_success=sum(t['candidate_full_success'] for t in group), paired=len(accepted),
            baseline_loaded_median_s=statistics.median(t['baseline_loaded_s'] for t in accepted) if accepted else None,
            candidate_loaded_median_s=statistics.median(t['candidate_loaded_s'] for t in accepted) if accepted else None,
            baseline_median_s=statistics.median(t['baseline_motion_s'] for t in accepted) if accepted else None,
            candidate_median_s=statistics.median(t['candidate_motion_s'] for t in accepted) if accepted else None,
            short_x_control_usable=sum(t['control_usable'] for t in group),
            short_x_control_median_s=statistics.median(t['control_motion_s'] for t in group if t['control_usable']) if any(t['control_usable'] for t in group) else None,
            improvement_median_pct=statistics.median(improvements) if accepted else None,
            improvement_min_pct=min(improvements) if accepted else None,
            improvement_max_pct=max(improvements) if accepted else None,
            sign_test_two_sided_p=sign_p(improvements)))
    # Keep repeated seeds across different scenes in one block, not 21 independent samples.
    complete_blocks = {seed: statistics.mean(values) for seed, values in blocks.items() if len(values) == len(cases)}
    block_values = list(complete_blocks.values())
    all_paired = [t for t in trials if t['paired']]
    return dict(cases=groups, trials=trials, attempted=len(trials), paired=len(all_paired),
        baseline_full_success=sum(t['baseline_success'] for t in trials),
        candidate_full_success=sum(t['candidate_full_success'] for t in trials),
        reliability_not_reduced=bool(all_paired and len(all_paired) == sum(t['baseline_eligible'] for t in trials)),
        complete_seed_blocks=complete_blocks, block_sign_test_two_sided_p=sign_p(block_values),
        median_improvement_pct=statistics.median(t['improvement_pct'] for t in all_paired) if all_paired else None,
        practical_threshold_pct=10.,
        significant_practical_gain=bool(len(all_paired) == sum(t['baseline_eligible'] for t in trials) and len(complete_blocks) >= 6 and sign_p(block_values) < .05
            and all_paired and statistics.median(t['improvement_pct'] for t in all_paired) >= 10.))


def render(report, output):
    esc = html.escape
    def num(value, digits=3):
        return '—' if value is None else f'{value:.{digits}f}'
    verdict = '达到本轮“全配对有效＋≥10%候选运动收益＋种子分块符号检验p<0.05”的探索性标准' if report['significant_practical_gain'] else '本轮尚未证明有显著且实用的全周期提速'
    rows = ''.join(f"<tr><th>{esc(c['case'])}</th><td>{c['baseline_success']}/{c['attempted']}</td><td>{c['candidate_usable']}/{c['attempted']}（全段{c['candidate_full_success']}）</td><td>{c['paired']}</td><td>{num(c['baseline_median_s'])}</td><td>{num(c['candidate_median_s'])}</td><td>{num(c['improvement_median_pct'],1)}%</td><td>{num(c['sign_test_two_sided_p'],4)}</td></tr>" for c in report['cases'])
    details = ''.join(f"<tr data-case='{esc(t['case'])}'><td>{esc(t['case'])}</td><td>{t['seed']}</td><td>{'通过' if t['paired'] else '不纳入有效收益'}</td><td>{num(t['baseline_motion_s'])}</td><td>{num(t['candidate_motion_s'])}</td><td>{num(t['improvement_pct'],1)}%</td><td>{num(t['candidate_compute_s'])}</td><td><details><summary>候选与失败</summary><pre>{esc(json.dumps(t,ensure_ascii=False,indent=1))}</pre></details></td></tr>" for t in report['trials'])
    buttons = ''.join('<button onclick="filterRows(\'' + esc(c['case']) + '\')">' + esc(c['case']) + '</button>' for c in report['cases'])
    control_rows = ''.join(f"<tr><th>{esc(c['case'])}</th><td>{c['short_x_control_usable']}/{c['attempted']}</td><td>{num(c['short_x_control_median_s'])}</td></tr>" for c in report['cases'])
    page = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>PR82 空域XYZ抽离实验</title>
<style>body{{font:16px/1.65 system-ui,sans-serif;max-width:1250px;margin:auto;padding:30px;background:#f5f7fa;color:#192538}}h1,h2{{line-height:1.3}}section,aside{{background:white;border:1px solid #dce1e8;border-radius:12px;padding:22px;margin:22px 0}}.cards{{display:flex;gap:16px;flex-wrap:wrap}}.card{{flex:1;min-width:180px;background:#e9eff8;padding:18px;border-radius:10px}}.card strong{{display:block;font-size:28px}}table{{border-collapse:collapse;width:100%;font-size:14px}}th,td{{padding:10px;border-bottom:1px solid #dce1e8;text-align:left;vertical-align:top}}thead{{background:#1d3554;color:white}}tbody tr:nth-child(even){{background:#f0f4f8}}.scroll{{overflow-x:auto}}pre{{max-width:520px;max-height:500px;overflow:auto;font-size:12px}}.warn{{border-left:5px solid #c7801b}}button{{padding:8px 14px;margin:4px;cursor:pointer}}code{{word-break:break-word}}@media(prefers-color-scheme:dark){{body{{background:#111821;color:#dbe5f0}}section,aside{{background:#1b2634;border-color:#3c4d64}}.card,tbody tr:nth-child(even){{background:#24354b}}td,th{{border-color:#3c4d64}}}}@media print{{body{{padding:0;background:white;color:black}}section{{break-inside:avoid}}button{{display:none}}details pre{{max-height:none}}}}</style>
<header><p>2026-11-11 · 配对离线研究 · 基线 ee6dba3（PR82）</p><h1>在已清空的XYZ空间抽离：是否真的更快？</h1><p><strong>{verdict}</strong></p></header>
<div class="cards"><div class="card">有效配对<strong>{report['paired']} / {report['attempted']}</strong></div><div class="card">全周期收益中位数<strong>{num(report['median_improvement_pct'],1)}%</strong></div><div class="card">完整种子分块<strong>{len(report['complete_seed_blocks'])}</strong></div><div class="card">分块双侧p值<strong>{num(report['block_sign_test_two_sided_p'],4)}</strong></div></div>
<section><h2>场景汇总</h2><p>正收益表示候选运动时间变短；只在基线全阶段原生标志通过、两者携箱采样通过时计算有效配对。失败数始终保留。</p><div class="scroll"><table><thead><tr><th>场景</th><th>基线通过</th><th>候选携箱通过</th><th>有效配对</th><th>基线周期中位数s</th><th>候选周期中位数s</th><th>配对收益中位数</th><th>单场景双侧p</th></tr></thead><tbody>{rows}</tbody></table></div></section>
<section><h2>短X控制：不要混淆收益来源</h2><div class="scroll"><table><thead><tr><th>场景</th><th>10cm短X携箱有效</th><th>短X全周期候选中位数s</th></tr></thead><tbody>{control_rows}</tbody></table></div><p>最终基线完整原生/动力学标志{report['baseline_full_success']}/{report['attempted']}，替代后的全段标志{report['candidate_full_success']}/{report['attempted']}；共享接近阶段失败仍保留。短X控制不是XYZ方案；主表使用的真实非轴向方案必须有Y或Z位移。主表中位数仅在同一有效配对集合计算，短X表展示自身候选集合，不用两个不同集合中位数作因果归因。</p></section><section><h2>做了什么，保持什么</h2><p>5×5显式占位箱格；top_outer为L24/R20（移除21/22/23），upper_inner为L18/R16、upper_outer为L19/R15（均移除上方20/21/23/24，保留中央22）。编号均0-based。基线和候选使用完全相同的剩余障碍。</p><p>同一吸附关节状态、同一运输终点、同一其他四阶段；只替换携箱抽离/运输。候选在59点XYZ终点网格中解析预测/碰撞筛选，保留短X控制，并探索上方、上方侧向和整格上方净空族（主表仅计真正非轴向候选），沿用原LM、1.10膨胀RRT、LM后处理及Fast时间化，不固定35cm。候选搜索规模和失败逐个显示。</p><p>限值保持每箱10kg、速度60°/s、jerk500rad/s³、原扭矩与0.995缩放；动力学4001点诊断。提速比较的是抽离＋运输合计及全周期，而不是改阶段名字。</p></section>
<aside class="warn"><h2>不要把这些结果误读成……</h2><ul><li>不是新完整规划器的端到端计算提速：使用同一个基线候选选型，保留四阶段，额外三候选搜索的计算另列。整格上方净空补测复用同一已求解基线，重新求解候选；不将复用轨迹称为新全流程计算。</li><li>不是实机或连续碰撞认证；仍是PR82原40球载箱模型与离散研究采样，可能漏实体边角或采样间碰撞。</li><li>不是与PR发布环境完全一致：固定Fast源码和模型哈希通过，本机Torch/NumPy/SciPy版本与原发布记录不同；两种方法在同一当前环境配对重跑。</li><li>候选族根据先行失败补充，属于样本内小规模探索，不是独立确认性实验，不能外推整墙、生产成功率。多个场景使用相同种子时按种子分块，避免伪造独立样本。</li><li>首轮整段自由几何优化出现碰撞及原生失败，未计作成功；简单拼接/重拟合也不是稳定大幅提速。</li></ul></aside>
<section><h2>每个种子的证据</h2><div><button onclick="filterRows('')">全部</button>{buttons}</div><div class="scroll"><table><thead><tr><th>场景</th><th>种子</th><th>有效性</th><th>基线s</th><th>候选s</th><th>收益</th><th>候选计算s</th><th>详情</th></tr></thead><tbody id="trials">{details}</tbody></table></div></section>
<footer><p>可复核原始证据：实验目录内每种子comparison.json、原基线六段npz、替代候选npz、summary.json、provenance.json；方法与可复制命令见工作树research/curobo_v3/FREE_SPACE_EXTRACT.md。</p></footer><script>function filterRows(name){{document.querySelectorAll('#trials tr').forEach(row=>row.hidden=!!name&&row.dataset.case!==name)}}</script></html>'''
    output.write_text(page)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summaries', nargs='*', type=Path)
    parser.add_argument('--report', type=Path, help='Render an existing compact report without raw trajectories')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if bool(args.summaries) == bool(args.report):
        parser.error('Supply raw campaign summaries OR --report, not both/neither')
    if args.report:
        report = json.loads(args.report.read_text())
    else:
        report = summarize([json.loads(path.read_text()) for path in args.summaries])
        report['evidence_summaries'] = [str(path) for path in args.summaries]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix('.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    render(report, args.output)
    print(json.dumps({k: v for k, v in report.items() if k not in ('trials',)}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    assert sign_p([1]*7) == .015625
    assert sign_p([1]*5) == .0625
    assert sign_p([1, -1]) == 1.
    main()
