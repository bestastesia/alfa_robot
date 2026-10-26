import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from curobo_core.adapter import matrix_pose
from curobo_core.scene import SceneObject,Pose
from v3_wall25_demo import WallPlanner,SCHEDULE
from v3_random_error_runner import RandomErrorRunner
from v3_rezero_task_planner import migrated_assets
from v3_tcp_rezero_pipeline import tool_to_box
from v3_zero_model_candidate import ROOT


class ContainerPlanner(WallPlanner):
    def bind_request(self,request):
        active=self.remaining_boxes
        self.remaining_boxes={box for box in active if box//25==self.layer}
        try:scene=super().bind_request(request)
        finally:self.remaining_boxes=active
        future=[pose for box,pose in self.world_boxes.items() if box in active and box//25>self.layer]
        if future:
            points=np.array([pose[:3,3] for pose in future])
            lower=points.min(0)-np.array([.15,.2,.2]);upper=points.max(0)+np.array([.15,.2,.2])
            obstacle=SceneObject('future_walls_solid',tuple(upper-lower),Pose(tuple((upper+lower)/2)))
            self.snapshot=replace(self.snapshot,objects=(*self.snapshot.objects,obstacle))
        return scene


def generate(folder):
    folder.mkdir(parents=True,exist_ok=True)
    planner=ContainerPlanner(migrated_assets());original=planner.original_snapshot
    front=next(item.pose.position[0]-item.dimensions_m[0]/2 for item in original.objects if item.object_id=='front_wall')
    wall=next(item for item in original.objects if item.object_id=='left_wall')
    opening=wall.pose.position[0]-wall.dimensions_m[0]/2
    last=front-.20;spacing=.31
    layers=int(np.floor((last-.15-opening)/spacing))+1
    centers=last-spacing*np.arange(layers-1,-1,-1)
    boxes={}
    for layer,x in enumerate(centers):
        for local in range(25):
            pose=np.eye(4);pose[:3,3]=[x,(local%5-2)*.41,.201+(local//5)*.41];boxes[layer*25+local]=pose
    cfg=json.loads((ROOT/'config/tcp_scene_preview.json').read_text());h=json.loads((ROOT/'config/rezero_home.json').read_text())
    home=dict(cfg['home']);home['updown']=h['updown_m']
    home.update({f'{side}_joint{index}':float(np.deg2rad(angle)) for side in ('left','right') for index,angle in enumerate(h[side+'_deg'],1)})
    remaining=set(boxes);rounds=[];runner=RandomErrorRunner();runner.planner=planner
    manifest=dict(world_boxes={str(key):value.tolist() for key,value in boxes.items()},rounds=rounds,
        completed_boxes=[],success=False,scope='整集装箱离线试验；未取后方箱墙实心保守包络；导航瞬移到位',layer_count=layers)
    previous_base=float(centers[0]-last)
    print('CONTAINER',layers,'walls',len(boxes),'boxes','centers',centers.tolist(),flush=True)
    for layer,x in enumerate(centers):
        offset=float(x-last)
        planner.layer=layer;planner.world_boxes=boxes
        planner.conveyor_center_x_m=-2.3+offset
        objects=tuple(replace(item,pose=replace(item.pose,position=(item.pose.position[0]+offset,*item.pose.position[1:]))) if item.object_id.startswith('conveyor') else item for item in original.objects)
        planner.original_snapshot=replace(original,objects=objects)
        for local_task in SCHEDULE:
            task={side:layer*25+box for side,box in local_task.items()}
            planner.remaining_boxes=set(remaining);planner.demo_task=task
            top=min(local_task.values())//5<=1;mode='top' if top else 'side'
            base=np.eye(4);base[0,3]=offset+(.3 if top else 0.)
            evidence=dict(task_label=f'第{layer+1}面 '+str(local_task),parking_world_from_footprint=base.tolist(),parking_yaw_deg=0.,inputs={})
            for side,box in task.items():
                evidence['inputs'][side]=dict(grasp_mode=mode,tcp_in_footprint=(np.linalg.inv(base)@boxes[box]@np.linalg.inv(tool_to_box(mode))).tolist(),roll_pitch_yaw_deg=[0.,0.,0.])
            index=len(rounds)+1
            print('ROUND',index,'WALL',layer+1,local_task,flush=True)
            result=runner.run(evidence,home,folder/f'round_{index:03d}',lambda text:print(text,flush=True))
            result.update(wall_task=task,remaining_before=sorted(remaining),wall_index=layer,conveyor_offset_m=offset)
            if result.get('frames'):
                initial=np.asarray(result['frames'][0]).copy();names=result['joint_names'];initial[names.index('base_x')]=previous_base
                result['frames'].insert(0,initial.tolist());result['phases'].insert(0,'navigation_to_pick');result['payload'].insert(0,False)
            if result.get('success'):
                lastrow=np.asarray(result['frames'][-1]).copy();names=result['joint_names']
                lastrow[names.index('base_x')]=offset;lastrow[names.index('base_y')]=0.;lastrow[names.index('base_yaw')]=0.
                result['frames'].append(lastrow.tolist());result['phases'].append('base_return');result['payload'].append(False)
                previous_base=offset;remaining-=set(task.values())
            rounds.append(result);manifest.update(completed_boxes=sorted(set(boxes)-remaining),success=not remaining)
            pending=folder/'wall.pending.json'
            pending.write_text(json.dumps(manifest,ensure_ascii=False))
            pending.replace(folder/'wall.json')
            print('ROUND_RESULT',index,result.get('success'),result.get('blocked_stage'),result.get('blocked_details'),flush=True)
            if not result.get('success'):return


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--folder',type=Path,default=Path('/tmp/container_demo_20261021'))
    generate(parser.parse_args().folder)
