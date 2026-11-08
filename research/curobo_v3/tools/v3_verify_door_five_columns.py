#!/usr/bin/env python3
"""Independent structural gate for the five vertical door-pair sequence."""
import argparse,json,math
from pathlib import Path
from curobo_core.door_recenter import DOOR_COLUMN_ORDER,door_pair

def first_after(phases,name,start=-1):
    try:return phases.index(name,start+1)
    except ValueError:return -1

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--sequence',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();result=json.loads(a.sequence.read_text());rounds=result.get('rounds',[])
 checks=dict(sequence_success=result.get('success') is True,five_rounds=len(rounds)==5,column_order=result.get('column_order')==list(DOOR_COLUMN_ORDER),completed_top_two_rows=set(result.get('completed_box_ids',()))==set(range(15,25)),remaining_lower_rows=set(result.get('remaining_box_ids',()))==set(range(15)),no_final_attachments=not result.get('final_snapshot',{}).get('attachments',()),trajectory_cache_disabled=result.get('plan_cache',{}).get('enabled') is False)
 round_checks=[]
 for index,column in enumerate(DOOR_COLUMN_ORDER):
  lower,upper=door_pair(column);record=rounds[index] if index<len(rounds) else {};path=Path(record.get('path',''));d=json.loads(path.read_text()) if path.is_file() else {};phases=d.get('phases',[]);task={str(k):int(v) for k,v in d.get('task',{}).items()};upper_side=next((s for s,v in task.items() if v==upper),None);lower_side=next((s for s,v in task.items() if v==lower),None);upper_attach=upper_side+'_attach' if upper_side else '';lower_attach=lower_side+'_attach' if lower_side else '';lower_extract=lower_side+'_extract' if lower_side else '';lower_recenter=lower_side+'_recenter' if lower_side else '';upper_extract=upper_side+'_extract' if upper_side else ''
  order=[];cursor=-1
  for name in (upper_attach,lower_attach):cursor=first_after(phases,name,cursor);order.append(cursor)
  rack=first_after(phases,'rack_support_transfer',cursor)
  if rack>=0:cursor=rack;order.append(cursor)
  for name in (lower_extract,lower_recenter):cursor=first_after(phases,name,cursor);order.append(cursor)
  if rack>=0:cursor=first_after(phases,upper_attach,cursor);order.append(cursor)
  cursor=first_after(phases,upper_extract,cursor);order.append(cursor)
  pair_recenter=min((value for name in ('left_recenter','right_recenter','dual_recenter_align') if (value:=first_after(phases,name,cursor))>=0),default=-1);cursor=pair_recenter;order.append(cursor)
  for name in ('base_reposition','conveyor_place','release','return_home','base_return'):cursor=first_after(phases,name,cursor);order.append(cursor)
  attachments=d.get('attachments_by_frame',());wait_indices=[i for i,p in enumerate(phases) if p==lower_recenter];held=set(x['object_id'] for x in attachments[wait_indices[-1]]) if wait_indices and attachments else set();expected_wait=({f'wall_box_{lower:02d}'} if rack>=0 else {f'wall_box_{lower:02d}',f'wall_box_{upper:02d}'});before_release=next((i for i,p in enumerate(phases) if p=='release'),-1);both_before_release=before_release>0 and {x['object_id'] for x in attachments[before_release-1]}=={f'wall_box_{lower:02d}',f'wall_box_{upper:02d}'}
  releases={event['object_id'] for event in d.get('predicted_scene_events',()) if event.get('phase')=='release'};first_frame=d.get('frames',[[0]*18])[0];base_y,base_yaw=first_frame[1],math.degrees(first_frame[2]);expected={0:(-.2,-30.),1:(-.4,0.),2:(0.,0.),3:(.4,0.),4:(.2,30.)}[column];column_aligned=abs(base_y-expected[0])<1e-6 and abs(base_yaw-expected[1])<1e-6
  passed=(record.get('success') is True and record.get('column')==column and record.get('lower_box')==lower and record.get('upper_box')==upper and upper_side and lower_side and all(v>=0 for v in order) and order==sorted(order) and held==expected_wait and both_before_release and releases=={f'wall_box_{lower:02d}',f'wall_box_{upper:02d}'} and not d.get('final_snapshot',{}).get('attachments',()) and d.get('plan_cache',{}).get('enabled') is False and d.get('navigation_path_validated') is False and column_aligned and bool(d.get('torque_validation',{}).get('success')) and bool(d.get('time_parameterized')))
  round_checks.append(dict(round=index+1,column=column,success=bool(passed),upper_side=upper_side,lower_side=lower_side,rack_supported=rack>=0,wait_attachments=sorted(held),phase_positions=order,column_aligned=column_aligned,symmetry_derived=bool(d.get('symmetry_source'))))
 checks['all_round_contracts']=len(round_checks)==5 and all(x['success'] for x in round_checks);output=dict(success=all(checks.values()),checks=checks,rounds=round_checks,total_ms=result.get('total_ms'),motion_duration_s=result.get('motion_duration_s'));a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n');print(json.dumps(output,ensure_ascii=False));raise SystemExit(0 if output['success'] else 1)
if __name__=='__main__':main()
