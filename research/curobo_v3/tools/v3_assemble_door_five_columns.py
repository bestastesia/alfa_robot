#!/usr/bin/env python3
"""Assemble five independently certified door rounds in mentor column order."""
import argparse,json,time
from pathlib import Path
from curobo_core.door_recenter import DOOR_COLUMN_ORDER,door_pair

def prune(value,removed):
    if isinstance(value,list):return [prune(item,removed) for item in value]
    if not isinstance(value,dict):return value
    out={key:prune(item,removed) for key,item in value.items()}
    if {'model_id','state','objects','attachments'}<=out.keys():
        out['objects']=[item for item in out['objects'] if not (item.get('object_id','').startswith('wall_box_') and int(item['object_id'][-2:]) in removed)]
    return out

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--round',type=Path,action='append',required=True);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
    if len(a.round)!=5:p.error('provide exactly five --round paths in order 4,0,3,1,2')
    a.output_dir.mkdir(parents=True,exist_ok=True);removed=set();rounds=[];aggregate=dict(success=False,sequence=True,sequence_kind='five_vertical_door_pairs',column_order=list(DOOR_COLUMN_ORDER),frames=[],phases=[],payload=[],attachments_by_frame=[],time_from_start_s=[],velocities=[],accelerations=[],jerks=[],frame_rounds=[],predicted_scene_events=[],stage_timing_ms={},stage_breakdown_ms={},stage_timing_spans=[],timed_segments=[],round_frame_ranges=[],rounds=[])
    wall_started=time.perf_counter()
    for number,(column,path) in enumerate(zip(DOOR_COLUMN_ORDER,a.round)):
        result=json.loads(path.read_text());lower,upper=door_pair(column)
        if result.get('success') is not True or int(result.get('column',-1))!=column:raise ValueError(f'round {number+1} is not a successful column {column} result')
        result=prune(result,removed);result.update(round_index=number,column=column,removed_box_ids=sorted(removed),plan_cache=dict(enabled=False,hit=False,symmetry_derived=bool(result.get('symmetry_source'))))
        round_path=a.output_dir/f'round-{number+1:02d}-column-{column}.json';round_path.write_text(json.dumps(result,ensure_ascii=False)+'\n')
        frame_offset=len(aggregate['frames']);time_offset=aggregate['time_from_start_s'][-1] if aggregate['time_from_start_s'] else 0.;wall_offset=sum(item['total_ms'] for item in rounds)
        for key in ('frames','phases','payload','attachments_by_frame','velocities','accelerations','jerks'):aggregate[key].extend(result[key])
        aggregate['time_from_start_s'].extend(time_offset+value for value in result['time_from_start_s']);aggregate['frame_rounds'].extend([number]*len(result['frames']))
        for stage,value in result.get('stage_timing_ms',{}).items():aggregate['stage_timing_ms'][stage]=aggregate['stage_timing_ms'].get(stage,0.)+value
        for stage,detail in result.get('stage_breakdown_ms',{}).items():
            target=aggregate['stage_breakdown_ms'].setdefault(stage,{})
            for key,value in detail.items():target[key]=target.get(key,0.)+value
        aggregate['stage_timing_spans'].extend(dict(item,start_ms=item['start_ms']+wall_offset,round=number,column=column) for item in result.get('stage_timing_spans',()))
        aggregate['timed_segments'].extend(dict(item,frame_start=item['frame_start']+frame_offset,frame_end=item['frame_end']+frame_offset,round=number,column=column) for item in result.get('timed_segments',()))
        for event in result.get('predicted_scene_events',()):
            item=dict(event);item['frame_index']=item.get('frame_index',0)+frame_offset;item['round']=number;item['column']=column;aggregate['predicted_scene_events'].append(item)
        aggregate['round_frame_ranges'].append(dict(round=number,column=column,start=frame_offset,end=len(aggregate['frames'])-1,lower_box=lower,upper_box=upper))
        record=dict(round=number+1,column=column,lower_box=lower,upper_box=upper,path=str(round_path),success=True,total_ms=result['total_ms'],motion_duration_s=result['motion_duration_s'],symmetry_derived=bool(result.get('symmetry_source')));rounds.append(record);removed.update((lower,upper))
    aggregate.update(rounds=rounds,completed_box_ids=sorted(removed),remaining_box_ids=sorted(set(range(25))-removed),total_ms=sum(item['total_ms'] for item in rounds),assembly_wall_ms=(time.perf_counter()-wall_started)*1000,success=True,motion_duration_s=aggregate['time_from_start_s'][-1],joint_names=json.loads(Path(rounds[0]['path']).read_text())['joint_names'],initial_snapshot=json.loads(Path(rounds[0]['path']).read_text())['initial_snapshot'],final_snapshot=json.loads(Path(rounds[-1]['path']).read_text())['final_snapshot'],plan_cache=dict(enabled=False,hit=False))
    (a.output_dir/'five-columns.json').write_text(json.dumps(aggregate,ensure_ascii=False)+'\n');(a.output_dir/'summary.json').write_text(json.dumps({key:aggregate[key] for key in ('success','column_order','completed_box_ids','remaining_box_ids','total_ms','motion_duration_s','assembly_wall_ms')},ensure_ascii=False,indent=2)+'\n');print(json.dumps({'success':True,'rounds':len(rounds),'motion_duration_s':aggregate['motion_duration_s'],'total_ms':aggregate['total_ms']},ensure_ascii=False))
if __name__=='__main__':main()
