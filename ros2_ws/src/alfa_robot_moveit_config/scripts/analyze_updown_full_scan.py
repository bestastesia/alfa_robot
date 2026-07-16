import csv,json,math
from pathlib import Path
from collections import defaultdict,Counter
import argparse

parser = argparse.ArgumentParser(description='分析13组任务的1cm Updown全高度IK/碰撞扫描')
parser.add_argument('run_root', type=Path)
args = parser.parse_args()
root = args.run_root
physical_upper=0.92

def runs(vals):
 vals=sorted(vals); out=[]
 if not vals:return out
 a=b=vals[0]
 for v in vals[1:]:
  if round(v-b,2)==0.01:b=v
  else:out.append((a,b));a=b=v
 out.append((a,b)); return out

def runstr(rs): return '; '.join(f'{a:.2f}-{b:.2f}' if a!=b else f'{a:.2f}' for a,b in rs) or '无'
rows=[]; summaries=[]
for p in sorted(root.glob('*/stage_snapshot.json')):
 d=json.loads(p.read_text()); task=p.parent.name.split('_',1)[1]
 by=defaultdict(lambda:{'generated':0,'ik_legal':0,'ik_reject':Counter(),'scene_pass':0,'scene_reject':0,'collision_reasons':Counter()})
 for r in d.get('all_ik_candidate_records',[]):
  h=round(float(r['h'])+1e-9,2); x=by[h]; x['generated']+=1
  if r.get('legal'):x['ik_legal']+=1
  else:x['ik_reject'][r.get('rejection_reason','unknown')]+=1
 for r in d.get('records',[]):
  by[round(float(r['h'])+1e-9,2)]['scene_pass']+=1
 for r in d.get('scene_rejected_records',[]):
  h=round(float(r['h'])+1e-9,2); by[h]['scene_reject']+=1
  by[h]['collision_reasons'][r.get('scene_rejection_reason','unknown')]+=1
 for i in range(100):
  h=round(i/100,2); x=by[h]; checked=x['scene_pass']+x['scene_reject']
  rows.append({
   'task':task,'left_box_id':d['left_box_id'],'right_box_id':d['right_box_id'],'h_m':f'{h:.2f}',
   'within_urdf_limit':h<=physical_upper,'generated_branches':x['generated'],'ik_legal_branches':x['ik_legal'],
   'ik_rejected_branches':x['generated']-x['ik_legal'],'collision_checked_unique':checked,
   'collision_pass':x['scene_pass'],'collision_reject':x['scene_reject'],
   'collision_pass_rate_pct':f'{100*x["scene_pass"]/checked:.2f}' if checked else '',
   'ik_rejection_reasons':json.dumps(x['ik_reject'],ensure_ascii=False,sort_keys=True),
   'collision_rejection_reasons':json.dumps(x['collision_reasons'],ensure_ascii=False,sort_keys=True),
  })
 valid={h:x for h,x in by.items() if h<=physical_upper}
 feasible=[h for h,x in valid.items() if x['scene_pass']>0]
 maxpass=max((x['scene_pass'] for x in valid.values()),default=0)
 robust_threshold=max(1,math.ceil(maxpass*0.5)) if maxpass else 0
 robust=[h for h,x in valid.items() if robust_threshold and x['scene_pass']>=robust_threshold]
 checked=sum(x['scene_pass']+x['scene_reject'] for x in valid.values())
 passed=sum(x['scene_pass'] for x in valid.values())
 collisions=sum(x['scene_reject'] for x in valid.values())
 top_collision=Counter()
 for x in valid.values():top_collision.update(x['collision_reasons'])
 summaries.append({
  'task':task,'left_box_id':d['left_box_id'],'right_box_id':d['right_box_id'],
  'feasible_interval_m':runstr(runs(feasible)),'feasible_height_count':len(feasible),
  'max_collision_free_branches_per_height':maxpass,'robust_threshold_branches':robust_threshold,
  'robust_interval_m':runstr(runs(robust)),'robust_height_count':len(robust),
  'collision_checked_unique':checked,'collision_pass':passed,'collision_reject':collisions,
  'overall_collision_pass_rate_pct':f'{100*passed/checked:.2f}' if checked else '0.00',
  'top_collision_reason':top_collision.most_common(1)[0][0] if top_collision else '',
  'top_collision_reason_count':top_collision.most_common(1)[0][1] if top_collision else 0,
 })
with (root/'height_scan_1cm.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
with (root/'task_height_summary.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=summaries[0].keys());w.writeheader();w.writerows(summaries)
md=['# 13组任务 Updown 全高度扫描分析','', '- 扫描申请范围：`0.00~0.99 m`，步长 `0.01 m`。','- 当前 URDF 物理范围：`0.00~0.92 m`；`0.93~0.99 m`仅用于暴露配置越界，不纳入推荐。','- Joint2 限位：左右均为 `±90°`。','- “可用区间”：该高度至少存在1个附着箱体场景碰撞通过的双臂IK解。','- “稳健区间”：该高度的通过分支数不少于本任务单高度最大通过数的50%。','', '|任务|可用区间(m)|稳健区间(m)|每高度最多通过分支|碰撞通过/检查|总通过率|主要碰撞原因|','|---|---|---|---:|---:|---:|---|']
for s in summaries:
 md.append(f"|{s['task']}|{s['feasible_interval_m']}|{s['robust_interval_m']}|{s['max_collision_free_branches_per_height']}|{s['collision_pass']}/{s['collision_checked_unique']}|{s['overall_collision_pass_rate_pct']}%|{s['top_collision_reason']} ({s['top_collision_reason_count']})|")
(root/'ANALYSIS.md').write_text('\n'.join(md)+'\n')
print('\n'.join(md))
print('\nFILES',root/'height_scan_1cm.csv',root/'task_height_summary.csv',root/'ANALYSIS.md')


# 无第三方绘图库，直接生成可浏览的 SVG，避免系统 NumPy/Matplotlib ABI 冲突。
tasks = [s['task'] for s in summaries]
width, panel_w, panel_h = 1500, 720, 210
height = 80 + ((len(tasks) + 1) // 2) * panel_h
svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">']
svg.append('<rect width="100%" height="100%" fill="#10151d"/>')
svg.append('<text x="30" y="38" fill="#f0f4fa" font-size="24" font-family="sans-serif">13-task Updown 1cm IK / collision scan (Joint2 ±90°)</text>')
for index, task in enumerate(tasks):
 col, row = index % 2, index // 2
 x0, y0 = 45 + col * 745, 70 + row * panel_h
 plot_x, plot_y, plot_w, plot_h = x0 + 48, y0 + 32, 620, 130
 task_rows = [r for r in rows if r['task'] == task and r['within_urdf_limit']]
 max_y = max([int(r['collision_pass']) + int(r['collision_reject']) for r in task_rows] + [1])
 def px(h): return plot_x + h / physical_upper * plot_w
 def py(v): return plot_y + plot_h - v / max_y * plot_h
 svg.append(f'<rect x="{x0}" y="{y0}" width="690" height="185" rx="8" fill="#18212d"/>')
 svg.append(f'<text x="{x0+12}" y="{y0+23}" fill="#f0f4fa" font-size="17" font-family="sans-serif">{task.replace("_", "/")}</text>')
 svg.append(f'<line x1="{plot_x}" y1="{plot_y+plot_h}" x2="{plot_x+plot_w}" y2="{plot_y+plot_h}" stroke="#7d8998"/>')
 svg.append(f'<line x1="{plot_x}" y1="{plot_y}" x2="{plot_x}" y2="{plot_y+plot_h}" stroke="#7d8998"/>')
 svg.append(f'<line x1="{px(0.3)}" y1="{plot_y}" x2="{px(0.3)}" y2="{plot_y+plot_h}" stroke="#5ba7ff" stroke-dasharray="5,4"/>')
 pass_points = ' '.join(f'{px(float(r["h_m"])):.1f},{py(int(r["collision_pass"])):.1f}' for r in task_rows)
 reject_points = ' '.join(f'{px(float(r["h_m"])):.1f},{py(int(r["collision_reject"])):.1f}' for r in task_rows)
 svg.append(f'<polyline points="{pass_points}" fill="none" stroke="#32d17d" stroke-width="2"/>')
 svg.append(f'<polyline points="{reject_points}" fill="none" stroke="#ff6262" stroke-width="1.5"/>')
 svg.append(f'<text x="{plot_x}" y="{plot_y+plot_h+18}" fill="#aeb8c5" font-size="11">0.00</text>')
 svg.append(f'<text x="{plot_x+plot_w-28}" y="{plot_y+plot_h+18}" fill="#aeb8c5" font-size="11">0.92</text>')
 svg.append(f'<text x="{plot_x+plot_w-150}" y="{plot_y+15}" fill="#32d17d" font-size="11">pass</text>')
 svg.append(f'<text x="{plot_x+plot_w-105}" y="{plot_y+15}" fill="#ff6262" font-size="11">reject</text>')
svg.append('</svg>')
(root/'updown_height_scan.svg').write_text('\n'.join(svg))
print('CHART', root/'updown_height_scan.svg')
