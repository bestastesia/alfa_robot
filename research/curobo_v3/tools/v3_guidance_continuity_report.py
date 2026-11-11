import json
from pathlib import Path

import numpy as np


def ranges(indices):
    output=[]
    for value in indices:
        value=int(value)
        if output and value==output[-1][-1]+1:
            output[-1][-1]=value
        else:
            output.append([value,value])
    return output


def analyze(entry):
    diagnostic=entry['node_diagnostics']
    rows=np.asarray(diagnostic['joints'])
    identifiers=np.asarray(diagnostic['point_indices'])
    legal=np.asarray(diagnostic['legal'],bool)
    collision=np.asarray(diagnostic['collision_valid'],bool)
    layers=[rows[(identifiers==point)&legal] for point in range(128)]
    counts=[len(layer) for layer in layers]
    transitions=[]
    components=[]
    previous=None
    bottleneck=None
    run_start=None
    layer_indices=[np.flatnonzero((identifiers==point)&legal) for point in range(128)]
    chain_indices={}
    parents=[]
    def save_component(end):
        node=int(bottleneck.argmin())
        for layer in range(end,run_start-1,-1):
            chain_indices[str(layer)]=int(layer_indices[layer][node])
            if layer>run_start:
                node=int(parents[layer-run_start-1][node])
    for index,layer in enumerate(layers):
        if not len(layer):
            if previous is not None:
                save_component(index-1)
                components.append(dict(start=run_start,end=index-1,minimax_step_deg=float(np.rad2deg(bottleneck.min()))))
            previous=None
            continue
        if previous is None:
            previous=layer
            bottleneck=np.zeros(len(layer))
            run_start=index
            parents=[]
            continue
        difference=np.abs(previous[:,None]-layer[None])
        difference[:,:,0]*=np.deg2rad(15)/.1
        edges=difference.max(-1)
        minimum=float(edges.min())
        costs=np.maximum(bottleneck[:,None],edges)
        parents.append(costs.argmin(0))
        bottleneck=costs.min(0)
        arg=np.unravel_index(edges.argmin(),edges.shape)
        delta=np.abs(previous[arg[0]]-layer[arg[1]])
        transitions.append(dict(from_point=index-1,to_point=index,min_step_deg=float(np.rad2deg(minimum)),
            best_pair_updown_cm=float(delta[0]*100),best_pair_largest_arm_joint_deg=float(np.rad2deg(delta[1:].max()))))
        previous=layer
    if previous is not None:
        save_component(127)
        components.append(dict(start=run_start,end=127,minimax_step_deg=float(np.rad2deg(bottleneck.min()))))
    return dict(covered_points=sum(value>0 for value in counts),legal_per_point=counts,
        empty_ranges=ranges(np.flatnonzero(np.asarray(counts)==0)),
        all_collision_rejected_ranges=ranges([point for point in range(128) if not collision[identifiers==point].any()]),
        covered_run_minimax=components,
        discontinuous_boundaries=[edge for edge in transitions if edge['min_step_deg']>10],
        worst_boundaries=sorted(transitions,key=lambda edge:edge['min_step_deg'],reverse=True)[:8],
        full_128_chain_minimax_deg=components[0]['minimax_step_deg'] if len(components)==1 and components[0]['start']==0 and components[0]['end']==127 else None,
        best_chain_row_indices=chain_indices,
        metric='Max absolute raw joint step, updown scaled 0.1m=15deg; no wrap across bounded joints. Minimax DP over every retained configuration, free endpoints within each nonempty run. 10deg is diagnostic only; no edge collision certification.')


def main():
    root=Path('/mnt/mydisk/ALFA/alfa_robot_v3_curobo/research/curobo_v3/generated/wrist_all_stage_20261022')
    source=root/'worst_three_ik'
    output=root/'continuity_diagnostic'
    output.mkdir(exist_ok=True)
    selection=[[],[],['approach'],['approach','return_home'],['approach','return_home'],['transport'],['conveyor_place'],['approach','return_home']]
    manifest=[]
    report=[]
    for index,item in enumerate(json.loads((source/'manifest.json').read_text())):
        document=json.loads((source/item['path']).read_text())
        for comparison in document['comparisons']:
            for method,entry in comparison['methods'].items():
                if entry.get('node_diagnostics'):
                    analysis=analyze(entry)
                    report.append(dict(task=item['label'],phase=comparison['phase'],method=method,**analysis))
                    entry['continuity_analysis']=analysis
        chosen=[comparison for comparison in document['comparisons'] if comparison['phase'] in selection[index]]
        if not chosen:
            chosen=[None]
        for comparison in chosen:
            events=[]
            if comparison:
                for method in ('lm','full','analytic'):
                    entry=comparison['methods'][method]
                    events.append(dict(entry,phase=comparison['phase']+' · '+method,
                        reference_tracks=comparison['reference_tracks'],reference_scene_objects=comparison['reference_metadata'].get('reference_scene_objects',[])))
            filename=f'{len(manifest):02d}.json'
            result=dict(task=item['task'],success=document['cycle_success'],guided_search_events=events,
                blocked_stage=document.get('blocked_stage'),blocked_details=document.get('blocked_details'),
                diagnostic_run_kind='Matched three-method frozen stage data; continuity computed over all retained configurations')
            (output/filename).write_text(json.dumps(result,ensure_ascii=False))
            label=item['label']+(' · '+comparison['phase'] if comparison else ' · 前置抽离失败')
            manifest.append(dict(label=label,task=item['task'],path=filename,success=result['success']))
    (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    (output/'all_stage_continuity.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    for row in report:
        if any(m['label'].startswith(row['task']+' · '+row['phase']) for m in manifest):
            print(row['task'],row['phase'],row['method'],'covered',row['covered_points'],'gaps',row['empty_ranges'],
                'minimax',row['full_128_chain_minimax_deg'],'worst',[(x['from_point'],round(x['min_step_deg'],1)) for x in row['worst_boundaries'][:2]])


if __name__=='__main__':
    main()
